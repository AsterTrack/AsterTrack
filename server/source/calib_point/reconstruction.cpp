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

#include "reconstruction.hpp"
#include "reconstruction_internal.hpp"
#include "util/eigenalg.hpp"

//#define LOG_MAX_LEVEL LTrace
#include "util/log.hpp"
#include "util/util.hpp"

#include <cassert>
#include <chrono>
#include <omp.h>

/**
 * Geometric reconstruction of observations used for calibration
 */

/**
 * Attempts to reconstruct the geometry (points, cameras calibration) of the scene given the observed points
 */
[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
std::optional<ErrorMessage> reconstructGeometry(const ObsPointData &data, std::vector<CameraCalib> &cameraCalibs, PointReconstructionParameters params, std::stop_token stopToken)
{
	ScopedLogCategory scopedLogCategory(LPointReconstruction);

	int viewCount = cameraCalibs.size();
	int pointCount = data.points.size();
	if (pointCount < 100)
	{
		LOGC(LError, "Too little data for reconstruction!");
		return "Too little data for reconstruction!";
	}

	// Build measurement matrix for point lookup and fill with point correspondences
	Eigen::MatrixXd measurementMatrix = Eigen::MatrixXd::Constant(viewCount*3, pointCount, NAN);
	{
		int colIndex = 0;
		for (auto &point : data.points)
		{
			for (auto &sample : point.samples)
			{
				if (cameraCalibs[sample.camera].lensValid())
					measurementMatrix.block<3,1>(sample.camera*3, colIndex) =
						undistortPoint<double>(cameraCalibs[sample.camera], sample.point.cast<double>()).homogeneous();
			}
			colIndex++;
		}
	}

	auto start = pclock::now();


	// ----- Point Normalisation

	// Build normalisation matrices to invert normalisations later
	Eigen::MatrixXd normMeasurementMatrix = Eigen::MatrixXd(viewCount*3, pointCount);
	Eigen::MatrixXd viewNormInv(3, viewCount*3);
	for (int v = 0; v < viewCount; v++)
		viewNormInv.middleCols<3>(v*3) = normalisePointRow(measurementMatrix.middleRows<3>(v*3), normMeasurementMatrix.middleRows<3>(v*3));
	Eigen::MatrixXd normMeasurementMatrixOriginal = normMeasurementMatrix;


	// ----- Projective Depths Estimation
	LOGC(LDebug, "-- Projective Depths Estimation");

	// Iteratively determine projective depths according to best strategy
	Eigen::MatrixXd projectiveDepthMatrix = Eigen::MatrixXd::Ones(viewCount, pointCount);
	Eigen::VectorXi viewsDiscarded = Eigen::VectorXi::Zero(viewCount);
	MatrixX<BOOL> depthsEstimated = estimateProjectiveDepths(projectiveDepthMatrix, normMeasurementMatrixOriginal, normMeasurementMatrix, params, viewsDiscarded);
	assert(depthsEstimated.rows() == viewCount && depthsEstimated.cols() == pointCount);

	int recViewCount = viewCount - viewsDiscarded.count();
	if (recViewCount < 2)
	{ // TODO: Reconstructing 2 cameras MAY work, allow for now to give user choice
		LOGC(LError, "Cannot recover any views/camera calibrations since only %d/%d had significant overlap!", recViewCount, viewCount);
		for (int i = 0; i < viewsDiscarded.size(); i++)
			LOGC(LError, " View %d got recoverable state %d!", i, viewsDiscarded[i]);
		return asprintf_s("Cannot recover any views/camera calibrations since only %d/%d had significant overlap!", recViewCount, viewCount);
	}
	if (stopToken.stop_requested())
		return std::nullopt;

	// Merge observations and their partial projectiveDepths into projectiveMatrix and discard irrecoverable views
	MatrixX<BOOL> projectiveDepthMissing(recViewCount, pointCount);
	MatrixX<BOOL> observationDataMissing(recViewCount, pointCount);
	Eigen::MatrixXd projectiveMatrix(recViewCount*3, pointCount);
	Eigen::MatrixXd recViewNormInv(3, recViewCount*3);
	for (int vv = 0, v = -1; vv < viewCount; vv++)
	{
		if (viewsDiscarded(vv)) continue;
		v++;
		for (int p = 0; p < pointCount; p++)
		{
			double projDepth = projectiveDepthMatrix(vv,p);
			auto obsData = normMeasurementMatrix.block<3,1>(vv*3, p);
			projectiveDepthMissing(v,p) = 1 - depthsEstimated(v, p);
			observationDataMissing(v,p) = obsData.hasNaN();
			projectiveMatrix.block<3,1>(v*3, p) = projDepth * obsData;
		}
		recViewNormInv.block<3,3>(0,v*3) = viewNormInv.block<3,3>(0,vv*3);
	}


	// ----- Matrix Balancing

	balanceMatrix3Triplet(projectiveMatrix);

	auto mid1 = pclock::now();

	if (stopToken.stop_requested())
		return std::nullopt;


	// ----- Missing Data Extrapolation
	LOGC(LDebug, "-- Missing Data Extrapolation");

	// Right now projectiveMatrix has data holes that need to be filled by extrapolation
	// Both due to missing points (NaNs) and missing projective depth (marked by projectiveDepthMissing)
	// For that, find basis for the vector space of rank 4 spanned by the projectiveMatrix
	// Then complete the columns of the projectiveMatrix as linear combinations of that basis
	Eigen::MatrixXd basis(recViewCount*3, 4);
	float noiseFactor = determineRank4Basis(basis, projectiveMatrix, projectiveDepthMissing, observationDataMissing, params, stopToken);
	bool recoverProblems = noiseFactor < params.basis.minRankFactor;
	if (std::isnan(noiseFactor))
	{ // Signal that recovery of basis is impossible for this submatrix
		LOGC(LError, "Failed to determine basis of data samples due to numerical errors!");
		return "Failed to determine basis of data samples due to numerical errors!";
	}
	if (stopToken.stop_requested())
		return std::nullopt;
	auto mid2 = pclock::now();
	Eigen::MatrixXd P_approx = Eigen::MatrixXd::Constant(4, pointCount, NAN);
	int pointsUnrecoverable = recoverPointData(projectiveMatrix, projectiveDepthMissing, basis, P_approx);

	LOGC(LDebug, "Approximated matrix P*X and projectiveMatrix have error of %f RMSE due to differences in projective depths!", 
		std::sqrt((projectiveMatrix - (basis*P_approx)).squaredNorm() / (recViewCount*3*pointCount)));

	auto mid3 = pclock::now();

	if (stopToken.stop_requested())
		return std::nullopt;


	// ----- Assemble Factorisation Matrix
	LOGC(LDebug, "-- Factorisation");

	int recPointCount = pointCount-pointsUnrecoverable;
	Eigen::MatrixXd factorisationMatrix(recViewCount*3, recPointCount);
	{ // Assemble factorisationMatrix from real and approximated measurements, combined with fully estimated projective depths from basis 
		for (int pp = 0, p = -1; pp < pointCount && p < recPointCount; pp++)
		{
			if (projectiveDepthMissing.col(pp).any()) continue;
			p++;
			// Column is fully determined, move points with projective depth recovered from basis into new matrix
			for (int v = 0; v < recViewCount; v++)
			{
				double projDepth = basis.row(v*3+2) * P_approx.col(pp);
				factorisationMatrix.block<3,1>(v*3,p) = projDepth * projectiveMatrix.block<3,1>(v*3,pp).hnormalized().homogeneous();
			}
		}
	}
	Eigen::MatrixXd recMeasurementMatrix(recViewCount*3, recPointCount);
	{ // Restrict measurement matrix to recoverable views and points (solely for verification purposes)
		for (int vv = 0, v = -1; vv < viewCount; vv++)
		{
			if (viewsDiscarded(vv)) continue;
			v++;
			for (int pp = 0, p = -1; pp < pointCount; pp++)
			{
				if (projectiveDepthMissing.col(pp).any()) continue;
				p++;
				recMeasurementMatrix.block<3,1>(v*3,p) = measurementMatrix.block<3,1>(vv*3,pp);
			}
		}
	}


	// ----- Matrix Balancing

	balanceMatrix3Triplet(factorisationMatrix);

	auto mid4 = pclock::now();

	if (stopToken.stop_requested())
		return std::nullopt;


	// ----- Rank-4 Factorisation

	Eigen::MatrixXd V_all, P_all;
	{ // Factorise complemented data into View matrices and Points

		Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd_M(factorisationMatrix);
		LOGC(LDebug, "Factorisation matrix with %d unrecoverable columns removed has rank %d (should be 4) with rank values (%.2f, %.2f, %.2f, %.2f) - %.2f!", 
			pointsUnrecoverable, (int)svd_M.rank(), svd_M.singularValues()(0), svd_M.singularValues()(1), svd_M.singularValues()(2), svd_M.singularValues()(3), svd_M.singularValues()(4));

		// Discard noise data in ranks over 4, split singular values arbitrarily (here in half with sqrt)
		Eigen::Matrix4d sigmaSqrt = svd_M.singularValues().head<4>().cwiseSqrt().asDiagonal();
		V_all = svd_M.matrixU().leftCols<4>() * sigmaSqrt;
		P_all = sigmaSqrt * svd_M.matrixV().leftCols<4>().transpose();

		// Revert point normalisation
		for (int v = 0; v < recViewCount; v++)
			V_all.block<3,4>(v*3,0) = recViewNormInv.block<3,3>(0,v*3) * V_all.block<3,4>(v*3,0);
	}

	// Test reprojection error on original (non-estimated) measurements
	LOGC(LDebug, "Reprojection error of factorisation on measurements: %fpx RMSE", 
		std::sqrt(calculateReprojectionErrorSq(V_all, P_all, recMeasurementMatrix))*PixelFactor);

	auto mid5 = pclock::now();


	// ----- Bundle Adjustment

	// TODO: Bundle adjustment on P*X?
	// Not necessary for currently tested setups, good enough to immediately go to non-linear optimisation


	// ----- Euclidean Stratification
	LOGC(LDebug, "-- Euclidean Stratification");

	// Apply correction to camera matrices and positions
	Eigen::Matrix4d H_strat = getStratificationMatrix(V_all, P_all);
	Eigen::MatrixXd V_corrected = V_all * H_strat;
	Eigen::MatrixXd P_corrected = H_strat.inverse() * P_all; // Only needed for verification

	// Normalise positions - seems to be necessary
	for (int p = 0; p < recPointCount; p++)
		P_corrected.col(p) /= P_corrected(3,p);

	// Custom correction to assure we can always recover projection matrices with positive FoVs and in desired coordinate system
	Eigen::Matrix4d H_cor = getCorrectionMatrix(V_corrected, P_corrected);
	V_corrected = V_corrected * H_cor;
	P_corrected = H_cor * P_corrected;

	auto mid6 = pclock::now();

	// Test reprojection error on original (non-estimated) measurements
	LOGC(LDebug, "Reprojection error of stratified factorisation on measurements: %fpx RMSE", 
		std::sqrt(calculateReprojectionErrorSq(V_corrected, P_corrected, recMeasurementMatrix))*PixelFactor);


	// ----- Projective Factorisation
	LOGC(LDebug, "-- Projective Factorisation");

	std::optional<ErrorMessage> error;

	for (int vv = 0, v = -1; vv < viewCount; vv++)
	{
		LOGC(LDebug, "------------");

		if (viewsDiscarded(vv))
		{
			LOGC(LWarn, "Can not recover view %d!", vv);
			continue;
		}
		v++;

		int validMeasurements = 0;
		for (int p = 0; p < recPointCount; p++)
		{
			if (!recMeasurementMatrix.block<3,1>(v*3, p).hasNaN())
				validMeasurements++;
		}

		LOGC(LInfo, "Results for view %d for camera #%u (%d) with %d/%d regarded points filled:",
			vv, cameraCalibs[vv].id, cameraCalibs[vv].index, recPointCount-validMeasurements, recPointCount);

		// Decompose view projection matrix in V_corrected into camera projection and transformation and update cameraCalibs with it
		error = SeparatePerspectiveProjection(V_corrected.block<3,4>(v*3,0), v, P_corrected, recMeasurementMatrix, cameraCalibs[vv]);
		if (error) break;
	}
	LOGC(LDebug, "------------");

	auto end = pclock::now();
	LOGC(LDebug, 
		"-- Reconstruction took %.2fms: "
			"%.2fms proj. depth, %.2fms basis, %.2fms filling, %.2fms interim, %.2fms factorisation, %.2fms stratification, %.2fms camera factorisation", 
		dtMS(start, end), 
			dtMS(start, mid1), dtMS(mid1, mid2), dtMS(mid2, mid3), dtMS(mid3, mid4), dtMS(mid4, mid5), dtMS(mid5, mid6), dtMS(mid6, end));

	return error;
}