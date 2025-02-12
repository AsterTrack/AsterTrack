/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef EIGEN_ALG_H
#define EIGEN_ALG_H

#include "util/eigenutil.hpp"

#include "Eigen/Eigenvalues"

#include <array>

//#define POLAR_DECOMPOSITION
#ifdef POLAR_DECOMPOSITION
#include "unsupported/Eigen/MatrixFunctions" // matrix sqrt
#endif

/**
 * General algorithms using Eigen
 */


/**
 * Project 3D target bounds into 2D bounds within a camera view using its MVP matrix
 */
template<typename Scalar>
Bounds2<Scalar> projectBounds(const Eigen::Transform<Scalar,3,Eigen::Projective> &mvp, Bounds3<Scalar> const &bounds)
{
	// Optimized bound projection, based around the fact that each bound side has four same coordinates (AABB in target-space)
	// Doesn't account for edges crossing the camera-plane!
	// So if bounding box is partly behind, partly in front of camera, results might be smaller than expected
	Bounds2<Scalar> bounds2D;

	if (bounds.includes((mvp * Eigen::Matrix<Scalar,4,1>(0,0,0,1)).hnormalized()))
	{ // If inside the bounding box, bounds should be full-screen
		bounds2D.minX = bounds2D.minY = -1;
		bounds2D.maxX = bounds2D.maxY = +1;
		return bounds2D;
	}

	// Extract relevant rows
	Eigen::Matrix<Scalar,3,4> m;
	m.row(0) = mvp.matrix().row(0);
	m.row(1) = mvp.matrix().row(1);
	m.row(2) = mvp.matrix().row(3);

	// Project all needed coordinates separately
	std::array<Eigen::Matrix<Scalar,3,1>, 7> cols;
	cols[0] = m.col(0) * bounds.minX;
	cols[1] = m.col(1) * bounds.minY;
	cols[2] = m.col(2) * bounds.minZ;
	cols[3] = m.col(0) * bounds.maxX;
	cols[4] = m.col(1) * bounds.maxY;
	cols[5] = m.col(2) * bounds.maxZ;
	cols[6] = m.col(3);

	std::array<Eigen::Matrix<Scalar,3,1>, 8> corners;
	corners[0] = cols[6]+cols[0]+cols[1]+cols[2];
	corners[1] = cols[6]+cols[3]+cols[1]+cols[2];
	corners[2] = cols[6]+cols[0]+cols[4]+cols[2];
	corners[3] = cols[6]+cols[0]+cols[1]+cols[5];
	corners[4] = cols[6]+cols[0]+cols[4]+cols[5];
	corners[5] = cols[6]+cols[3]+cols[4]+cols[2];
	corners[6] = cols[6]+cols[3]+cols[1]+cols[5];
	corners[7] = cols[6]+cols[3]+cols[4]+cols[5];

	// Assemble projected corners and find bound
	for (int i = 0; i < 8; i++)
	{
		if (corners[i].z() > 0)
			bounds2D.include(corners[i].hnormalized());
	}
	return bounds2D;
}

/**
 * Determine the transform from the second point cloud to the first using the Least Squares Method by Kabsch
 * If Type is Eigen::Isometry, only translation and rotation is regarded
 * If Type is Eigen::Affine, translation, rotation and scale is regarded
 */
template<typename Scalar, int Type = Eigen::Isometry>
static Eigen::Transform<Scalar,3,Type> kabsch(Eigen::Ref<Eigen::Matrix<Scalar,Eigen::Dynamic,3>> trMat, Eigen::Ref<Eigen::Matrix<Scalar,3,Eigen::Dynamic>> mkMat)
{
	static_assert(Type == Eigen::Isometry || Type == Eigen::Affine);
	int ptCount = std::min(trMat.rows(), mkMat.cols());

	// Calculate center of mass for both marker and triangulated point set
	Eigen::Matrix<Scalar,3,1> trCenter = Eigen::Matrix<Scalar,3,1>::Zero();
	Eigen::Matrix<Scalar,3,1> mkCenter = Eigen::Matrix<Scalar,3,1>::Zero();
	for (int j = 0; j < ptCount; j++)
	{
		trCenter += trMat.row(j);
		mkCenter += mkMat.col(j);
	}
	trCenter /= ptCount;
	mkCenter /= ptCount;

//	Scalar trScale = 0, mkScale = 0; // Not needed, optimised version used instead
	Scalar mkScaleMS = 0;
	if (Type == Eigen::Affine)
	{ // Correct for scale
		for (int j = 0; j < ptCount; j++)
		{
			trMat.row(j) -= trCenter;
			mkMat.col(j) -= mkCenter;
//			trScale += trMat.row(j).norm();
//			mkScale += mkMat.col(j).norm();
			mkScaleMS += mkMat.col(j).squaredNorm();
		}
//		trScale /= ptCount;
//		mkScale /= ptCount;
	}
	else if (Type == Eigen::Isometry)
	{ // Assume same scale
		for (int j = 0; j < ptCount; j++)
		{
			trMat.row(j) -= trCenter;
			mkMat.col(j) -= mkCenter;
		}
	}

	// Compute optimal rotation matrix and transformation
	Eigen::Transform<Scalar,3,Type> transform;
#ifdef POLAR_DECOMPOSITION
	if (Type == Eigen::Isometry)
	{ // Try using polar decomposition (faster)
		Matrix3<Scalar> cov = trMat * mkMat;
		Matrix3<Scalar> affine = (cov.transpose()*cov).sqrt();
		Matrix3<Scalar> rot = cov * affine.inverse();

		transform.linear() = rot;
		transform.translation() = trCenter - transform.linear() * mkCenter;
		// Doesn't work yet, sometimes it's great, most of the time it's completely off
	}
	else
#endif
	{ // Compute using SVD (slow but always works)
		Eigen::JacobiSVD<Eigen::Matrix<Scalar,3,3>, Eigen::NoQRPreconditioner | Eigen::ComputeFullU | Eigen::ComputeFullV> svd(mkMat * trMat);
		Eigen::Matrix<Scalar,3,3> U = svd.matrixU().transpose();
		Eigen::Matrix<Scalar,3,3> V = svd.matrixV();
		Scalar d = (V * U).determinant(); // Might include reflection, check determinant

		// Assemble transform
		transform.linear() = V * Eigen::DiagonalMatrix<Scalar,3>(1, 1, d) * U;
		if (Type == Eigen::Affine)
			transform.linear() *= svd.singularValues().dot(Eigen::Matrix<Scalar,3,1>(1, 1, d)) / mkScaleMS; // *= trScale/mkScale, but optimised;
		transform.translation() = trCenter - transform.linear() * mkCenter;
	}	

	return transform;
}

/**
 * Solve for a fundamental matrix given the dataMatrix created from multiplying normalised point correspondences
 * Returns the confidence determined by the ratio between rank 2 and rank 3, if too small the fundamental matrix likely is not valid
 */
template<typename Scalar>
Scalar solveFundamentalMatrix(const Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> &dataMatrix, Eigen::Matrix<Scalar,3,3> &fundamentalMatrix)
{
	// Solve dataMatrix * f = 0 in least square sense
	// TODO: Choose either SVD or EVD based on speed and accuracy

	typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
	typedef Eigen::Matrix<Scalar,3,3> Matrix3;

	// Option: SVD
	/*Eigen::BDCSVD<MatrixX, Eigen::ComputeThinV> svd(dataMatrix);
	Eigen::Matrix<Scalar,9,1> fv = svd.matrixV().col(8);
	Eigen::Matrix<Scalar,9,1> rankValues = svd.singularValues();*/

	// Option: EVD
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar,9,9>> evd(dataMatrix.transpose()*dataMatrix, Eigen::ComputeEigenvectors);
	Eigen::Matrix<Scalar,9,1> fv = evd.eigenvectors().col(0);
	Eigen::Matrix<Scalar,9,1> rankValues = evd.eigenvalues().reverse();

	//LOGC(LTrace, "Fundamental Matrix Rank Values: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", rankValues(0), rankValues(1), rankValues(2), rankValues(3), rankValues(4), rankValues(5), rankValues(6), rankValues(7), rankValues(8));

	// Assemble coefficients to fundamental matrix
	Matrix3 F;
	F << fv(0), fv(1), fv(2), fv(3), fv(4), fv(5), fv(6), fv(7), fv(8);

	// Decompose Fd by using SVD to expose singular values
	Eigen::JacobiSVD<Matrix3, Eigen::ComputeFullU | Eigen::ComputeFullV> svd_rank(F);
	//LOGC(LTrace, "Fundamental matrix singular values before rank-2 truncation: %f, %f, %f\n", svd_rank.singularValues().x(), svd_rank.singularValues().y(), svd_rank.singularValues().z());

	// Reduce rank to 2 by removing the least significant (third) singular value
	// A fundamental matrix should be of rank 2, if rank 3 then because of numerical errors (from input or algorithm)
	Eigen::DiagonalMatrix<Scalar, 3> truncatedSV(svd_rank.singularValues().x(), svd_rank.singularValues().y(), 0);
	fundamentalMatrix = svd_rank.matrixU() * truncatedSV * svd_rank.matrixV().transpose();

	return svd_rank.singularValues().y() / svd_rank.singularValues().z();
}

#endif // EIGEN_ALG_H