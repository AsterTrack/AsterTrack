/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/**
 * NOTE:
 * This file is licensed under the MPL despite currently being deeply integrated into the project.
 * The intent is that if you need it badly enough, it is easy to extract it to use in your own project.
 * This would require removing reference to, among others, obs_data.hpp, but it is very much doable.
 */

#include "reconstruction_internal.hpp"
#include "util/eigenalg.hpp"

//#define LOG_MAX_LEVEL LTrace
#include "util/log.hpp"
#include "util/util.hpp"

#include <cassert>

/**
 * Stratification of decomposed perspective view and point matrices
 */

template<typename Scalar>
static void SeparateProjectionMatrix(const Eigen::Ref<const Eigen::Matrix<Scalar,3,3>> matrix, Eigen::Ref<Eigen::Matrix<Scalar,3,3>> projection, Eigen::Ref<Eigen::Matrix<Scalar,3,3>> rotation)
{ // RQ decomposition to get a projection matrix and a rotation matrix from a general projection matrix

	// Apply QR decomposition to linear part of view matrix
	// TODO: Consider using more accurate QR decomposition
	Eigen::HouseholderQR<Eigen::Matrix<Scalar,3,3>> qr_C(matrix.transpose());

	// Get transposed matrixes Q^T and R^T from QR decomposition
	Eigen::Matrix<Scalar,3,3> Q_t = qr_C.householderQ().transpose();
	Eigen::Matrix<Scalar,3,3> R_t = qr_C.matrixQR().template triangularView<Eigen::Upper>().transpose();

	// Correct to get RQ instead of QR, such that R is as close to diagonal as possible
	Eigen::Matrix<Scalar,3,3> Q_u;
	Q_u.row(0) = R_t.row(1).cross(R_t.row(2)).normalized();
	Q_u.row(1) = Q_u.row(0).cross(R_t.row(2)).normalized();
	Q_u.row(2) = Q_u.row(0).cross(Q_u.row(1));
	projection = R_t * Q_u.transpose();
	rotation = Q_u * Q_t;
}

/* static void SeparateProjectionMatrix(Eigen::Matrix3d matrix, Eigen::Matrix3d &projection, Eigen::Matrix3d &rotation)
{ // Polar decomposition to get a symmetric projection matrix and a orthogonal rotation matrix from a general projection matrix
	Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU|Eigen::ComputeFullV> svd(matrix);
	projection = svd.matrixV()*Eigen::DiagonalMatrix<double,3>(svd.singularValues())*svd.matrixV().transpose();
	rotation = svd.matrixU()*svd.matrixV().transpose();
} */

// TODO: Consider just using EulerAngles to approximate, seems to do fine

Eigen::Matrix4d getStratificationMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P)
{
	assert(V.cols() == 4);
	assert(P.rows() == 4);
	int viewCount = V.rows()/3;

	// Determine the 4x4 H to stratify V and P: C H H^-1 P
	Eigen::MatrixXd B_solve = Eigen::MatrixXd(viewCount*2,4); // Rightmost column B of H
	Eigen::MatrixXd Q_solve = Eigen::MatrixXd(viewCount*4,10); // Coefficients of a 4x4 symmetric matrix Q=AA^T, to get left 4x3 matrix A of H

	auto setQMultRowCoefficients = [](auto coeff, auto l, auto r){
		coeff(0) = l.x()*r.x();
		coeff(1) = l.x()*r.y() + l.y()*r.x();
		coeff(2) = l.x()*r.z() + l.z()*r.x();
		coeff(3) = l.x()*r.w() + l.w()*r.x();
		coeff(4) = l.y()*r.y();
		coeff(5) = l.y()*r.z() + l.z()*r.y();
		coeff(6) = l.y()*r.w() + l.w()*r.y();
		coeff(7) = l.z()*r.z();
		coeff(8) = l.z()*r.w() + l.w()*r.z();
		coeff(9) = l.w()*r.w();
	};

	// Accumulate equations to solve for B and Q=AA^T
	// TODO: Choose option A, B or C - factorisationMatrix vs recreated, and fast&ugly vs slow&simple
	Eigen::VectorXd Td = (V*P).rowwise().sum(); // A
	//Eigen::VectorXd Td = factorisationMatrix.rowwise().sum(); // B
	for (int v = 0; v < viewCount; v++)
	{
		auto V_v = V.block<3,4>(v*3,0);

		// Determine row-sum of denormalised matrix
		Eigen::Vector3d T = Eigen::Vector3d::Zero();
		// Option A:
		T = Td.segment<3>(v*3);
		// Option B:
		//auto T_v = recViewNormInv.block<3,3>(0,v*3);
		//T.x() = Td(v*3+2)*T_v(0,2) + T_v(0,0)*Td(v*3+0);
		//T.y() = Td(v*3+2)*T_v(1,2) + T_v(1,1)*Td(v*3+1);
		//T.z() = Td(v*3+2)*T_v(2,2);
		// Option C:
		//for (int p = 0; p < colCount; p++)
		//	T += recViewNormInv.block<3,3>(0,v*3) * factorisationMatrix.block<3,1>(v*3,p);

		// Enter equations for b
		B_solve.row(v*2+0) = V_v.row(2)/T.z() - V_v.row(0)/T.x();
		B_solve.row(v*2+1) = V_v.row(2)/T.z() - V_v.row(1)/T.y();

		// Enter equations for Q
		// First three equations assume principal point = (0,0), generally not quite true, introduces slight errors
		setQMultRowCoefficients(Q_solve.row(v*4+0), V_v.row(0), V_v.row(0));
		setQMultRowCoefficients(Q_solve.row(v*4+1), V_v.row(1), V_v.row(1));
		Q_solve.row(v*4+0) -= Q_solve.row(v*4+1); // row0 * row0 - row1 * row1 = 0
		setQMultRowCoefficients(Q_solve.row(v*4+1), V_v.row(0), V_v.row(1)); // row0 * row1 = 0
		setQMultRowCoefficients(Q_solve.row(v*4+2), V_v.row(0), V_v.row(2)); // row0 * row2 = 0
		setQMultRowCoefficients(Q_solve.row(v*4+3), V_v.row(1), V_v.row(2)); // row1 * row2 = 0

		if (v == 0)
		{ // Use one view to fix the scale
			//setQMultRowCoefficients(Q_solve.row((recViewCount*4), V_v.row(2), V_v.row(2)); // row2 * row2 = 1
			// TODO: How to set 1
		}
		// TODO: Find one more constraint to allow calibration of just two view (e.g. same projection matrix)
	}

	// Solve for B and Q in least squares sense (avoid trivial 0 solution)
	Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinV> svd_B(B_solve);
	Eigen::Vector4d B = svd_B.matrixV().rightCols(1);
	Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinV> svd_Q(Q_solve);
	Eigen::Matrix<double,10,1> Qc = svd_Q.matrixV().rightCols(1);
	Eigen::Matrix4d Q; // Symmetric 4x4 matrix from 10 coefficients
	Q<< Qc(0), Qc(1), Qc(2), Qc(3),
		Qc(1), Qc(4), Qc(5), Qc(6),
		Qc(2), Qc(5), Qc(7), Qc(8),
		Qc(3), Qc(6), Qc(8), Qc(9);

	if (SHOULD_LOGC(LTrace))
	{
		Eigen::Matrix3d MMT = V.block<3,4>(0,0) * Q * V.block<3,4>(0,0).transpose();
		if (MMT(1,1) <= 0)
		{
			LOGC(LDebug, "Flipping Q because diagonal of MM^T is negative (checked %f)!", MMT(1,1));
			Q = -Q;
		}
	}

	LOGC(LDebug, "Stratification solve errors: B: %f, Q: %f", (B_solve*B).norm(), (Q_solve*Qc).norm());

	// Factor symmetric Q into AA^T of rank 3
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> evd_Q(Q, Eigen::ComputeEigenvectors);
	Eigen::Matrix<double,4,3> A;
	A.col(0) = evd_Q.eigenvectors().col(3) * std::sqrt(evd_Q.eigenvalues()(3));
	A.col(1) = evd_Q.eigenvectors().col(2) * std::sqrt(evd_Q.eigenvalues()(2));
	A.col(2) = evd_Q.eigenvectors().col(1) * std::sqrt(evd_Q.eigenvalues()(1));

	LOGC(LDebug, "Eigenvalues of Q: (%f, %f, %f) - %f", evd_Q.eigenvalues()(3), evd_Q.eigenvalues()(2), evd_Q.eigenvalues()(1), evd_Q.eigenvalues()(0));

	// Apply correction to camera matrices and positions
	Eigen::Matrix4d H;
	H << A, B;
	return H;
}

Eigen::Matrix4d getCorrectionMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P)
{
	assert(V.cols() == 4);
	assert(P.rows() == 4);
	int viewCount = V.rows()/3;

	// Check to make sure projection allows for orthogonal rotation, and positive projection diagonal, else modify H
	// Without this, the reprojection error would be fine, but the projection matrix can't be properly separated
	// TODO: Try to integrate in constraints for euclidean stratification, might be the final constraint allowing for two camera calibration
	Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
	for (int v = 0; v < viewCount; v++)
	{
		// Normalise camera matrix
		Eigen::Matrix<double,3,3> V_v = V.block<3,3>(v*3,0);
		V_v /= std::abs(V_v(2,2));

		bool back = (V.block<3,4>(v*3,0) * P).row(2).sum() < 0;
		if (back)
		{ // Flip - TODO: Find out why this happens
			LOGC(LDarn, "    View matrix %d is flipped!", v);
			V_v = -V_v;
		}

		Eigen::Matrix3d P_v, R_v;
		SeparateProjectionMatrix<double>(V_v, P_v, R_v);

		if (R_v.determinant() <= 0)
		{ // Should not happen afaik
			LOGC(LWarn, "    View matrix %ds' rotation matrix was invalid!", v);
			continue;
		}

		// Apply needed correction to H
		if (P_v(0,0) < 0)
			H(0,0) = -1;
		if (P_v(1,1) < 0)
			H(1,1) = -1;
		if (P_v(2,2) < 0)
			H(2,2) = -1;
		LOGC(LDebug, "View %d provided a correction of %f,%f,%f,%f", v, H(0,0), H(1,1), H(2,2), H(3,3));

		// Should only need to accommodate one valid camera to correct all of them
		break;
	}

	return H;
}

std::optional<ErrorMessage> SeparatePerspectiveProjection(Eigen::Matrix<double,3,4> V_v, int v, const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix, CameraCalib &cameraCalib)
{
	assert(P.rows() == 4);
	assert(P.cols() == measurementMatrix.cols());

	LOGC(LDebug, "Reprojection error of view %d initially: %fpx RMSE", v, std::sqrt(calculateReprojectionErrorSq(V_v, v, P, measurementMatrix))*PixelFactor);

	// Check polarity
	if ((V_v * P).row(2).sum() < 0)
	{
		LOGC(LDebug, "Flipping view matrix!");
		V_v = -V_v;
	}

	// Normalise camera matrix
	V_v /= std::abs(V_v(2,2));

	// Separate camera matrix into projection (~diagonal) and rotation (orthogonal) matrix with RQ decomposition
	Eigen::Matrix3d P_v, R_v;
	SeparateProjectionMatrix<double>(V_v.leftCols<3>(), P_v, R_v);

	// If the number of negatives is uneven, something got messed up - probably in getCorrectionMatrix
	int neg = (P_v(0,0) < 0) + (P_v(1,1) < 0) + (P_v(2,2) < 0) + (R_v.determinant() < 0);
	if (neg % 2 == 1)
		LOGC(LWarn, "Got uneven amount of negatives (%d), will not be able to correctly separate matrices for view %d!", neg, v);

	// Correct projection matrix signs
	for (int i = 0; i < 3; i++)
	{
		if (P_v(i,i) < 0)
		{
			P_v.col(i) *= -1;
			R_v.row(i) *= -1;
		}
	}
	// The only reason R_v can be assumed to be orthogonal now (and thus a valid rotation matrix)
	// is because getCorrectionMatrix assured that previously during euclidean stratification

	// Get center (from original, non-noise-removed projection matrices)
	Eigen::Vector3d center = -R_v.transpose() * P_v.inverse() * V_v.rightCols<1>();

	// Remove noise values from projection matrix, keeping just FoV and principal point (approximately zero)
	P_v.triangularView<Eigen::StrictlyLower>().setZero();
	P_v(0,1) = 0;
	P_v /= P_v(2,2);

	auto verifyDecomposition = [&](Eigen::Matrix3d P_v, Eigen::Matrix3d R_v, Eigen::Vector3d C_T) -> std::optional<ErrorMessage>
	{
		double reprojectionError = 0;
		int ptCnt = 0;
		int front = 0, back = 0;
		for (int p = 0; p < measurementMatrix.cols(); p++)
		{
			Eigen::Vector2d measure = measurementMatrix.block<2,1>(v*3, p);
			if (measure.hasNaN()) continue;
			ptCnt++;
			Eigen::Vector3d point3D = P.block<4,1>(0,p).hnormalized();
			Eigen::Vector3d reprojection = P_v * R_v * (point3D - C_T);
			if (reprojection.z() > 0) front++;
			else back++;
			reprojectionError += (measure-reprojection.hnormalized()).squaredNorm();
		}
		reprojectionError = std::sqrt(reprojectionError/ptCnt);
		LOGC(LInfo, "    Final reprojection error across %d recovered observations: %fpx RMSE", ptCnt, reprojectionError*PixelFactor);

		if (R_v.determinant() < 0)
		{ // Should not happen, if so then getCorrectionMatrix failed
			LOGC(LWarn, "    ----- Rotation matrix invalid!!!!");
			return "Camera Decomposition resulted in invalid rotation matrix!";
		}
		if (front < back)
		{ // Should not happen, if so then getCorrectionMatrix failed
			LOGC(LWarn, "    ----- View matrices flipped!!!!");
			return "Camera Decomposition resulted in flipped view matrix!";
		}
		if (reprojectionError*PixelFactor > 10)
		{
			LOGC(LInfo, "    ----- Unusually high reprojection RMSE!");
			return std::nullopt;
		}
		return std::nullopt;
	};

	// Record calibration values
	cameraCalib.f = P_v(0,0);
	cameraCalib.fInv = 1.0/P_v(0,0);
	cameraCalib.principalPoint.x() = P_v(0,2);
	cameraCalib.principalPoint.y() = P_v(1,2);
	cameraCalib.transform.linear() = R_v.transpose();
	cameraCalib.transform.translation() = center;
	cameraCalib.UpdateDerived();

	// Verify it is correct
	return verifyDecomposition(P_v, R_v, center);
}