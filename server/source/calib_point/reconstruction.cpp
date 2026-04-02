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


	// ----- Measurement Extrapolation Loop using Projective Depth Estimation

	Eigen::MatrixXd projectiveDepthMatrix = Eigen::MatrixXd::Ones(viewCount, pointCount);
	Eigen::ArrayX<BOOL> maskedViews(viewCount), maskedPoints(pointCount);
	while (!stopToken.stop_requested())
	{
		auto t0 = pclock::now();

		// ----- Projective Depths Estimation
		LOGC(LDebug, "-- Projective Depths Estimation");

		// Determine projective depths for a submatrix that needs filling (or whole matrix if not)
		// These will allow (partial) factorisatin with SVD, without, the matrix has too many free scaling variables
		projectiveDepthMatrix.setOnes();
		MatrixX<BOOL> depthsEstimated = estimateProjectiveDepths(projectiveDepthMatrix, normMeasurementMatrixOriginal, normMeasurementMatrix, params);
		assert(depthsEstimated.rows() == viewCount && depthsEstimated.cols() == pointCount);
		if (depthsEstimated.all())
		{ // Got all depths, all measurements, can proceed
			LOGC(LInfo, "Proceeding after recovery with all data points estimated!");
			maskedViews.setZero();
			maskedPoints.setZero();
			break;
		}
		if (stopToken.stop_requested())
			break;

		// Find relevant submatrix to continue with
		int recViewCount = (depthsEstimated.cast<int>().rowwise().sum().array() > params.FM.minPairwiseCorrespondences*2).cast<int>().sum();
		int recPointCount = (depthsEstimated.cast<int>().colwise().sum().array() == viewCount).cast<int>().sum();
		if (!params.strategy.forceOneshotReconstruction &&
			recViewCount == viewCount && recPointCount > pointCount*params.strategy.stopAtPointRecoveryRate)
		{ // May leave some cameras with less data than they should have, but useful for testing
			LOGC(LInfo, "Proceeding after %.2f%% of points (%d/%d) across all views have been recovered!",
				(float)recPointCount/pointCount*100.0f, recPointCount, pointCount);
			maskedViews.setZero();
			maskedPoints = (depthsEstimated.cast<int>().colwise().sum().array() == viewCount).cast<BOOL>();
			break;
		}
		// Take all samples for oneshot, may recover missing projective depths, and are more desparate for data
		int pointObsLimit = params.strategy.forceOneshotReconstruction? 1 : 2;
		// Filter points first to inform rough view sample count
		maskedPoints = (depthsEstimated.cast<int>().colwise().sum().array() < pointObsLimit).cast<BOOL>();
		for (int p = 0; p < pointCount; p++)
			if (maskedPoints(p))
				depthsEstimated.col(p).setZero();
		// Filter views at last to ensure they are helpful
		maskedViews = (depthsEstimated.cast<int>().rowwise().sum().array() < params.FM.minPairwiseCorrespondences*2).cast<BOOL>();
		for (int v = 0; v < viewCount; v++)
			if (maskedViews(v))
				depthsEstimated.row(v).setZero();
		// And finally filter points again incase they are now insufficiently covered
		maskedPoints = (depthsEstimated.cast<int>().colwise().sum().array() < pointObsLimit).cast<BOOL>();
		recViewCount = viewCount - maskedViews.cast<int>().sum();
		recPointCount = pointCount - maskedPoints.cast<int>().sum();
		if (depthsEstimated.cast<int>().sum() == recPointCount * recViewCount)
		{
			LOGC(LInfo, "Proceeding after complete recovery of %d/%d cameras and %d/%d points!", recViewCount, viewCount, recPointCount, pointCount);
			break;
		}
		LOGC(LInfo, "Filtered down to %d/%d views and %d/%d points for this iterations submatrix!", recViewCount, viewCount, recPointCount, pointCount);

		// Merge observations and their partial projectiveDepths into projectiveMatrix and discard irrecoverable views
		MatrixX<BOOL> projectiveDepthMissing(recViewCount, recPointCount);
		MatrixX<BOOL> observationDataMissing(recViewCount, recPointCount);
		Eigen::MatrixXd projectiveMatrix(recViewCount*3, recPointCount);
		for (int vv = 0, v = -1; vv < viewCount; vv++)
		{
			if (maskedViews(vv)) continue;
			v++;
			for (int pp = 0, p = -1; pp < pointCount; pp++)
			{
				if (maskedPoints(pp)) continue;
				p++;
				double projDepth = projectiveDepthMatrix(vv,pp);
				auto obsData = normMeasurementMatrix.block<3,1>(vv*3, pp);
				projectiveDepthMissing(v,p) = 1 - depthsEstimated(vv, pp);
				observationDataMissing(v,p) = obsData.hasNaN();
				projectiveMatrix.block<3,1>(v*3, p) = projDepth * obsData;
			}
		}


		// ----- Matrix Balancing

		balanceMatrix3Triplet(projectiveMatrix);

		auto t1 = pclock::now();

		if (stopToken.stop_requested())
			break;


		// ----- Data Basis Estimation
		LOGC(LDebug, "-- Data Basis Estimation");

		// Right now projectiveMatrix has data holes that need to be filled by extrapolation
		// Both due to missing points and missing projective depths (marked by respective missing-array)
		// For that, find basis for the vector space of rank 4 spanned by the projectiveMatrix
		Eigen::MatrixXd basis(recViewCount*3, 4);
		float noiseFactor = determineRank4Basis(basis, projectiveMatrix, projectiveDepthMissing, observationDataMissing, params, stopToken);
		bool recoverProblems = noiseFactor < params.basis.minRankFactor;
		if (std::isnan(noiseFactor))
		{ // Signal that recovery of basis is impossible for this submatrix
			// Already took measures that is should be recoverable, so can only fully abort iterations here
			LOGC(LError, "Failed to determine basis of data samples due to numerical errors!");
			return "Failed to determine basis of data samples due to numerical errors!";
		}
		if (stopToken.stop_requested())
			break;

		auto t2 = pclock::now();


		// ----- Missing Data Extrapolation
		LOGC(LDebug, "-- Missing Data Extrapolation");

		// Now use basis to complete the columns of the projectiveMatrix as linear combinations of that basis
		// Then basis*P_approx will be an estimation of the factorisation of the submatrix
		// Points that are not (yet) recoverable may still be NAN in P_approx
		Eigen::MatrixXd P_approx = Eigen::MatrixXd::Constant(4, recPointCount, NAN);
		auto recoverStats = recoverPointData(projectiveMatrix, projectiveDepthMissing, basis, P_approx);
		int recoveredPoints = recoverStats.first;
		int unrecoverablePoints = recoverStats.second;

		LOGC(LDebug, "Approximated matrix P*X and projectiveMatrix have error of %f RMSE due to differences in projective depths!", 
			std::sqrt((projectiveMatrix - (basis*P_approx)).squaredNorm() / (recViewCount*3*pointCount)));

		if (recoveredPoints == 0)
		{ // Can't improve further
			assert(unrecoverablePoints != 0); // Should have stopped right after depth estimation already
			LOGC(LInfo, "Proceeding after failing to recover any of the %d points, with %d/%d views and %d/%d points considered.",
				unrecoverablePoints, recViewCount, viewCount, recPointCount, pointCount);
			for (int pp = 0, p = -1; pp < pointCount; pp++)
			{
				if (maskedPoints(pp)) continue;
				p++;
				if (P_approx.col(p).hasNaN())
					maskedPoints(pp) = 1;
			}
			break;
		}

		// Copy recovered/extrapolated measurements
		for (int vv = 0, v = -1; vv < viewCount; vv++)
		{
			if (maskedViews(vv)) continue;
			v++;
			for (int pp = 0, p = -1; pp < pointCount; pp++)
			{
				if (maskedPoints(pp)) continue;
				p++;
				if (P_approx.col(p).hasNaN()) continue;
				auto obsData = normMeasurementMatrix.block<3,1>(vv*3, pp);
				auto recData = projectiveMatrix.block<3,1>(v*3, p);
				// Could keep projective depth, algorithms should be able to handle it
				// But it will be reestimated anyway, it is of no value to keep
				if (obsData.hasNaN())
					obsData = recData.hnormalized().homogeneous();
				// Recover missing projective depths only in oneshot - otherwise, redetermine all anew next iteration
				if (params.strategy.forceOneshotReconstruction)
					projectiveDepthMatrix(vv, pp) = basis.row(v*3+2) * P_approx.col(p);
			}
		}

		auto t3 = pclock::now();

		LOGC(LInfo, "-- Submatrix recovery of %dx%d took %.2fms: %.2fms proj. depth, %.2fms basis, %.2fms filling", 
			recViewCount*3, recPointCount, dtMS(t0, t3), dtMS(t0, t1), dtMS(t1, t2), dtMS(t2, t3));

		if (params.strategy.forceOneshotReconstruction)
		{
			// Mask off all points not fully recovered
			for (int pp = 0, p = -1; pp < pointCount; pp++)
			{
				if (maskedPoints(pp)) continue;
				p++;
				if (P_approx.col(p).hasNaN())
					maskedPoints(pp) = 1;
			}
			break;
		}
	}

	if (stopToken.stop_requested())
		return std::nullopt;


	// ----- Assemble Factorisation Matrix
	LOGC(LDebug, "-- Assemble Factorisation Matrix");

	int recViewCount = viewCount - maskedViews.cast<int>().sum();
	int recPointCount = pointCount - maskedPoints.cast<int>().sum();
	if (recViewCount < 2 || recPointCount < 100)
	{ // TODO: Reconstructing 2 cameras MAY work, allow for now to give user choice
		std::string error = asprintf_s("Cannot recover any views/camera calibrations since only %d/%d are recoverable with %d/%d points!",
			recViewCount, viewCount, recPointCount, pointCount);
		LOGC(LError, "%s", error.c_str());
		for (int i = 0; i < maskedViews.size(); i++)
			LOGC(LError, " View %d got recoverable state %d!", i, maskedViews[i]);
		return error;
	}

	// Properly subsample matrices using final masked views and points for factorisation
	Eigen::MatrixXd recMeasurementMatrix(recViewCount*3, recPointCount);
	Eigen::MatrixXd factorisationMatrix(recViewCount*3, recPointCount);
	for (int vv = 0, v = -1; vv < viewCount; vv++)
	{
		if (maskedViews(vv)) continue;
		v++;
		for (int pp = 0, p = -1; pp < pointCount; pp++)
		{
			if (maskedPoints(pp)) continue;
			p++;
			recMeasurementMatrix.block<3,1>(v*3, p) = measurementMatrix.block<3,1>(vv*3, pp);
			factorisationMatrix.block<3,1>(v*3, p) = projectiveDepthMatrix(vv,pp) * normMeasurementMatrix.block<3,1>(vv*3, pp);
		}
	}


	// ----- Matrix Balancing

	balanceMatrix3Triplet(factorisationMatrix);

	auto t4 = pclock::now();

	if (stopToken.stop_requested())
		return std::nullopt;


	// ----- Rank-4 Factorisation
	LOGC(LDebug, "-- Rank-4 Factorisation");

	Eigen::MatrixXd V_all, P_all;
	{ // Factorise complemented data into View matrices and Points

		Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd_M(factorisationMatrix);
		auto &val = svd_M.singularValues();
		LOGC(LDebug, "Factorisation matrix (%d x %d) with %d unrecoverable columns removed has rank %d (should be 4) with rank values (%.2f, %.2f, %.2f, %.2f) - %.2f!", 
			recViewCount*3, recPointCount, pointCount-recPointCount, (int)svd_M.rank(), val(0), val(1), val(2), val(3), val(4));

		// Discard noise data in ranks over 4, split singular values arbitrarily (here in half with sqrt)
		Eigen::Matrix4d sigmaSqrt = svd_M.singularValues().head<4>().cwiseSqrt().asDiagonal();
		V_all = svd_M.matrixU().leftCols<4>() * sigmaSqrt;
		P_all = sigmaSqrt * svd_M.matrixV().leftCols<4>().transpose();

		// Revert point normalisation
		for (int vv = 0, v = -1; vv < viewCount; vv++)
		{
			if (maskedViews(vv)) continue;
			v++;
			V_all.block<3,4>(v*3,0) = viewNormInv.block<3,3>(0,vv*3) * V_all.block<3,4>(v*3,0);
		}
	}

	// Test reprojection error on original (non-estimated) measurements
	LOGC(LDebug, "Reprojection error of factorisation on measurements: %fpx RMSE", 
		std::sqrt(calculateReprojectionErrorSq(V_all, P_all, recMeasurementMatrix))*PixelFactor);

	auto t5 = pclock::now();


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

	// Test reprojection error on original (non-estimated) measurements
	LOGC(LDebug, "Reprojection error of stratified factorisation on measurements: %fpx RMSE", 
		std::sqrt(calculateReprojectionErrorSq(V_corrected, P_corrected, recMeasurementMatrix))*PixelFactor);

	auto t6 = pclock::now();


	// ----- Projective Factorisation
	LOGC(LDebug, "-- Projective Factorisation");

	std::optional<ErrorMessage> error;

	for (int vv = 0, v = -1; vv < viewCount; vv++)
	{
		LOGC(LDebug, "------------");

		if (maskedViews(vv))
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
	LOGC(LInfo, "-- Reconstruction took %.2fms: %.2fms extrapolation, %.2fms factorisation, %.2fms stratification, %.2fms separation", 
		dtMS(start, end), dtMS(start, t4), dtMS(t4, t5), dtMS(t5, t6), dtMS(t6, end));

	return error;
}