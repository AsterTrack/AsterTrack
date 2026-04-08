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

//#define LOG_MAX_LEVEL LTrace
#include "util/log.hpp"
#include "util/util.hpp"

#include <numeric>
#include <algorithm>
#include <map>
#include <cassert>
#include <omp.h>

/**
 * Recovery of measurement matrix by estimating projective depths and filling missing data
 */

[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
MatrixX<BOOL> estimateProjectiveDepths(
	Eigen::MatrixXd &projectiveDepthMatrix,
	const Eigen::MatrixXd &originalMeasurementMatrix,
	const Eigen::MatrixXd &measurementMatrix,
	const PointReconstructionParameters &params,
	Eigen::VectorXi &viewsDiscarded)
{
	int viewCount = measurementMatrix.rows()/3;
	int pointCount = measurementMatrix.cols();
	assert(projectiveDepthMatrix.rows() == viewCount);
	assert(projectiveDepthMatrix.cols() == pointCount);
	assert(viewsDiscarded.size() == viewCount);

	MatrixX<BOOL> pointsObserved(viewCount, pointCount);
	MatrixX<BOOL> pointsFilled(viewCount, pointCount);
	Eigen::MatrixXi viewCorrespondences = Eigen::MatrixXi::Zero(viewCount, viewCount);
	for (int p = 0; p < pointCount; p++)
	{
		for (int v = 0; v < viewCount; v++)
		{
			pointsObserved(v, p) = !std::isnan(originalMeasurementMatrix(v*3,p));
			pointsFilled(v, p) = !std::isnan(measurementMatrix(v*3,p));
		}
		for (int v = 0; v < viewCount; v++)
		{
			if (pointsObserved(v, p))
				viewCorrespondences.col(v) += pointsObserved.col(p).cast<int>();
		}
	}
	// Ensure it's symmetric and remove diagonal
	viewCorrespondences = (viewCorrespondences + viewCorrespondences.transpose()) / 2;
	viewCorrespondences.diagonal().setZero();
	// From now on, we'll access by views, so transpose for better storage access pattern (points as dominant dimension)
	pointsObserved.transposeInPlace();
	pointsFilled.transposeInPlace();

	// May only initialise projective depth once per point, then transfer to other cameras
	VectorX<BOOL> projDepthsUninitialised = VectorX<BOOL>::Ones(pointCount);
	MatrixX<BOOL> pointsRecoverable = pointsFilled;
	MatrixX<BOOL> pointsRecovered = MatrixX<BOOL>::Zero(pointCount, viewCount);

	const bool ALLOW_FURTHER_POINT_INITS = false;
	const bool ALLOW_INDIRECT_SOURCES = false;

	// Recovered views and their point count with projective depths
	std::map<int, int> recovered;
	auto getNextBestTransfer = [&]() -> std::pair<int,int>
	{
		// Choose next best transfer from one view to another
		// REQUIREMENT: Sufficient viewCorrespondences between them to calculate fundamental matrix
		// BENEFIT: Missing projective depths in target view that source view has (initialised or OR already transferred)
		// Requirement is static, easy to check
		// Benefit requires matching of vector of points still recoverable in target view against bit-OR of:
		// - vector of points already recovered in source view
		// - bit-AND of vector of points in source view and vector of points still not initialised
		// These fields are implemented as matrix int matrices

		if (recovered.empty())
		{ // Select seeded view
			Eigen::VectorXi viewsRecover = (viewCorrespondences.array() >= params.FM.minPairwiseCorrespondences*2).cast<int>().colwise().sum();
			Eigen::VectorXi pairsRecover = viewCorrespondences.colwise().sum();
			Eigen::VectorXi recoverFactor = pairsRecover.cwiseProduct(viewsRecover);
			int bestView, bestTransfer;
			int factor = recoverFactor.maxCoeff(&bestView);
			viewCorrespondences.col(bestView).maxCoeff(&bestTransfer);
			recovered.insert({ bestView, 0 });
			recovered.insert({ bestTransfer, 1 });
			// Debug
			recoverFactor(bestView) = 0;
			int secondFactor = recoverFactor.maxCoeff();
			LOGC(LInfo, "Selected view %d as center with %d observations (%d originally), recoverable: %d views, %d pairs, factor %d (next best %d).",
				bestView, pointsFilled.col(bestView).cast<int>().sum(), pointsObserved.col(bestView).cast<int>().sum(), viewsRecover(bestView), pairsRecover(bestView), factor, secondFactor);
			LOGC(LInfo, "Selected view %d as next with %d / %d original points overlapping.",
				bestTransfer, viewCorrespondences(bestView, bestTransfer), pointsObserved.col(bestTransfer).cast<int>().sum());
			return { bestView, bestTransfer };
		}

		// Iteratively determine next best recovery path, accounting for number of indirection from seeded view
		std::pair<int,int> bestNext = { -1, -1 };
		float bestWeight = 0;
		int bestTransferrable = 0, bestExTransfer = 0, bestNewTransfer = 0;
		for (int s = 0; s < viewCount; s++)
		{
			if (viewsDiscarded(s)) continue;
			if (!ALLOW_INDIRECT_SOURCES && (!recovered.contains(s) || recovered[s] != 0))
				continue;
			int indirection = recovered.contains(s)? recovered[s]+1 : 1;
			for (int t = 0; t < viewCount; t++)
			{
				if (viewsDiscarded(t)) continue;
				if (s == t) continue;
				int correspondences = viewCorrespondences(s, t);
				if (correspondences < params.FM.minPairwiseCorrespondences) continue;

				auto projDepthsNewInSource = pointsFilled.col(s).array() * projDepthsUninitialised.array();
				int newlyTransferrable = (pointsRecoverable.col(t).array() * projDepthsNewInSource.array()).cast<int>().sum();
				int existingTransferrable = (pointsRecoverable.col(t).array() * pointsRecovered.col(s).array()).cast<int>().sum();
				int transferrable = existingTransferrable + (ALLOW_FURTHER_POINT_INITS? newlyTransferrable : 0);

				float weight = std::pow<float>(transferrable, 0.6f) * std::pow<float>(correspondences, 0.8f) / std::pow<float>(indirection, 0.6f);
				if (weight > bestWeight)
				{ // Propagating from source view to this one is the current best next propagation possible
					bestWeight = weight;
					bestTransferrable = transferrable;
					bestExTransfer = existingTransferrable;
					bestNewTransfer = newlyTransferrable;
					bestNext = { s, t };
				}
				LOGC(LDebug, "    Weight between %d-%d with %d in src, %d pot in tgt: %.1f, T:%d, C:%d, I:%d",
					s, t, pointsRecovered.col(s).array().cast<int>().sum(), pointsRecoverable.col(t).array().cast<int>().sum(), weight, transferrable, correspondences, indirection);
			}
		}
		if (bestWeight > 0)
		{
			int indirection = recovered.contains(bestNext.first)? recovered[bestNext.first]+1 : 1;
			int correspondences = viewCorrespondences(bestNext.first, bestNext.second);
			LOGC(LInfo, "Found pair (%d, %d) to be best (weight %.1f) with %d transferrable (%d ex, %d new), %d correspondences, and indirection of %d",
				bestNext.first, bestNext.second, bestWeight, bestTransferrable, bestExTransfer, bestNewTransfer, correspondences, indirection);
			if (correspondences < params.FM.minPairwiseCorrespondences || bestTransferrable < 100)
				return { -1, -1 };
			if (correspondences < params.FM.minPairwiseCorrespondences*2)
				indirection++; // While we'll try to recover, it's really bad
			recovered.insert({ bestNext.second, indirection });
			return bestNext;
		}
		return { -1, -1 };
	};

	while (true)
	{ // Iteratively determine projective depths of other views
		auto transfer = getNextBestTransfer();
		if (transfer.first < 0) break;
		int s = transfer.first, t = transfer.second;

		if (ALLOW_FURTHER_POINT_INITS || recovered.size() == 2)
		{ // Initialise uninitialised projective depths of source camera with 1
			int newlyInit = 0;
			for (int p = 0; p < pointCount; p++)
			{
				if (projDepthsUninitialised(p) == 0) continue;
				if (measurementMatrix.block<3,1>(transfer.first*3, p).hasNaN()) continue;
				projDepthsUninitialised(p) = 0;
				pointsRecoverable(p, s) = 0;
				pointsRecovered(p, s) = 1;
				newlyInit++;
			}
			LOGC(LInfo, "View strategy: Transfer from camera %d to camera %d - newly initialised %d / %d projective depths!", s, t, newlyInit, pointCount);
		}
		else
			LOGC(LInfo, "View strategy: Transfer from camera %d to camera %d!", s, t);

		// Determine fundamental matrix from rows of measurement matrix
		Eigen::Matrix3d F;
		std::pair<int,double> results = calculateFundamentalMatrix(originalMeasurementMatrix.middleRows<3>(s*3), originalMeasurementMatrix.middleRows<3>(t*3), F, params.FM.minPairwiseCorrespondences);
		if (results.first < params.FM.minPairwiseCorrespondences || results.second < params.FM.minConfidence)
		{ // View does not have sufficient correspondence with center view 
			// TODO: Multiple recovery strategy iterations until view can be recovered
			viewsDiscarded(t) = 1;
			LOGC(LWarn, "---- Retroactively abandoning view %d because it only shares %d points of %f confidence with best view %d!", t, results.first, results.second, s);
			continue;
		}

		// Find epipole as kernel of F
		Eigen::Vector3d e = F.jacobiSvd<Eigen::ComputeFullU>().matrixU().rightCols<1>();

		for (int p = 0; p < pointCount; p++)
		{
			if (!pointsRecovered(p, s))
				continue; // No source projective depth to transfer
			if (!pointsRecoverable(p, t))
				continue; // Is already determined
			auto x_b = measurementMatrix.block<3,1>(s*3, p) * projectiveDepthMatrix(s, p);
			if (x_b.hasNaN())
				continue; // Missing data in source camera
			auto x_v = measurementMatrix.block<3,1>(t*3, p) * projectiveDepthMatrix(t, p);
			if (x_v.hasNaN())
				continue; // Missing data in target camera
			// Determine projective depth inferred from reference projective depth of source camera
			Eigen::Vector3d a = e.cross(x_v);
			float projDepthFac = std::abs(a.dot(F * x_b) / a.squaredNorm());
			projectiveDepthMatrix(t, p) *= projDepthFac;
			pointsRecoverable(p, t) = 0;
			pointsRecovered(p, t) = 1;
		}
	}

	// Return again to normal point-dominant access
	pointsRecovered.transposeInPlace();
	return pointsRecovered;
}

[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
float determineRank4Basis(
	Eigen::MatrixXd &basis,
	const Eigen::MatrixXd &projectiveMatrix,
	const MatrixX<BOOL> &projectiveDepthMissing,
	const MatrixX<BOOL> &observationDataMissing,
	const PointReconstructionParameters &params,
	std::stop_token stopToken)
{
	assert(projectiveMatrix.cols() == projectiveDepthMissing.cols());
	assert(projectiveMatrix.rows() == projectiveDepthMissing.rows()*3);
	int pointCount = projectiveMatrix.cols();
	int viewCount = projectiveMatrix.rows()/3;

	// Find basis for the vector space of rank 4 spanned by M
	// First collect 4-tuples of the columns of M that are full rank (complete them to handle missing data in them)
	// Then take orthogonal complements of each of the vector spaces spanned by these matrices and combine them into N
	// Finally calculate orthogonal complements of that combined vector space spanned by N
	// The result is a noise-resistant basis for the target vector space spanned by M

	int colsN = 0;
	int tested4Tuples = 0;

	// Limit the number of maximum possible computation
	int minNColumns = std::max<int>(params.basis.nColMin, pointCount*viewCount*params.basis.nColMinFactor);
	int maxNColumns = pointCount*viewCount*params.basis.nColMaxFactor;
	int max4Tuples = pointCount*viewCount*params.basis.tupleMaxFactor;
	int max4TupleTests = pointCount*viewCount*params.basis.tupleTestMaxFactor;
	LOGC(LDebug, "Planning on testing %d random quadruplets of %d points!", max4TupleTests, pointCount);

	std::vector<Eigen::MatrixXd> partN;

#define PARALLEL
#ifdef PARALLEL
	std::atomic<int> totalPartN = 0, totalColsN = 0;

	omp_set_num_threads(std::min<int>(omp_get_max_threads(), params.basis.maxParallelism));
	#pragma omp parallel
#endif
	{
	std::vector<int> selectablePoints;
	VectorX<BOOL> selectedViews = VectorX<BOOL>::Ones(viewCount);

#ifdef PARALLEL
	std::vector<Eigen::MatrixXd> priv_partN;

	auto p0 = sclock::now();

	#pragma omp for nowait schedule(static, 100) reduction(+:colsN) reduction(+:tested4Tuples)
	for (int j = 0; j < max4TupleTests; j++)
#else
	while (colsN < maxNColumns && partN.size() < max4Tuples && tested4Tuples++ < max4TupleTests && !stopToken.stop_requested())
#endif
	{
#ifdef PARALLEL
		int tempColsN = totalColsN.load();
		int tempPartsN = totalPartN.load();
		if (tempColsN >= maxNColumns || tempPartsN >= max4Tuples) continue;
		if (stopToken.stop_requested()) continue;
#endif

		// Prepare selection buffers
		selectablePoints.resize(pointCount);
		std::iota(selectablePoints.begin(), selectablePoints.end(), 0);
		// TODO: Only select points whose projective depths are even initialised (not set in projDepthsUninitialised)
		selectedViews.setOnes();

		int indices[4];
		bool good = false;
		for (int i = 0; i < 4; i++)
		{
			good = false;
			for (int k = 0; k < 10 && !selectablePoints.empty() && !good; k++)
			{ // Find next column that results in a useable tuple
				indices[i] = selectablePoints[rand() % selectablePoints.size()];
				auto selectableEnd = remove_swap(selectablePoints.begin(), selectablePoints.end(), indices[i]);
				selectablePoints.erase(selectableEnd, selectablePoints.end());
				// Determine if its a good pick
				auto sharedViews = selectedViews.array() * (1 - observationDataMissing.col(indices[i]).array());
				if (sharedViews.sum() < 2) continue;
				// Found next column
				good = true;
				selectedViews = sharedViews;
				//auto selectableEnd = std::remove_if(selectablePoints.begin(), selectableEnd, [&](auto &col){ return false; });
				//selectablePoints.erase(selectableEnd, selectablePoints.end());
			}
			if (!good) break;
		}
		if (!good) continue;
		int selViewCount = selectedViews.cast<int>().sum();

		// Determine number of columns that need to be added
		Eigen::VectorXi obsDataMissing = Eigen::VectorXi::Zero(viewCount);
		Eigen::VectorXi projDepthMissing = Eigen::VectorXi::Zero(viewCount);
		for (int i = 0; i < 4; i++)
		{
			obsDataMissing += observationDataMissing.col(indices[i]).cast<int>();
			projDepthMissing += projectiveDepthMissing.col(indices[i]).cast<int>();
		}
		// For any view that has ANY point missing: Not contributing, add three columns of (1, 1, 1)
		// For any view that has just projective depths missing, add all their source points as columns (if >= 3, might as well just add (1, 1, 1))
		int newCols = 0;
		for (int v = 0; v < viewCount; v++)
		{
			if (!selectedViews(v)) continue;
			if (obsDataMissing(v) > 0 || projDepthMissing(v) >= 3)
				newCols += 3;
			else
				newCols += projDepthMissing(v);
		}

		// Extend 4 columns to B
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(selViewCount*3, 4+newCols);
		int newColIndex = 4;
		for (int v = 0, vv = 0; v < viewCount; v++)
		{
			if (!selectedViews(v)) continue;
			if (obsDataMissing(v) > 0 || projDepthMissing(v) >= 3)
			{
				assert(false); // Currently disabled by selectedViews
				B(vv*3+0,newColIndex++) = 1;
				B(vv*3+1,newColIndex++) = 1;
				B(vv*3+2,newColIndex++) = 1;
				vv++;
				continue;
			}
			for (int i = 0; i < 4; i++)
			{
				Eigen::Vector3d x = projectiveMatrix.block<3,1>(v*3, indices[i]);
				if (!projectiveDepthMissing(v, indices[i])) 
					B.block<3,1>(vv*3,i) = x;
				else // Still have coordinates, already embedded
					B.block<3,1>(vv*3,newColIndex++) = x;
			}
			vv++;
		}
		assert(newColIndex == 4+newCols);

		// Check that B is full rank, else discard
		Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeFullU> svd_B(B);
		int rankB = svd_B.rank();
		int colCount = selViewCount*3 - rankB;
		if (rankB >= 4+newCols && colCount > 0)
		{ // Extract null space / orthogonal complement of B
			LOGC(LTrace, "B of rank %d with %d columns has added a nullspace of rank %d / %d", rankB, 4+newCols, colCount, selViewCount*3);
			Eigen::MatrixXd orthComplementB = svd_B.matrixU().block(0, rankB, selViewCount*3, colCount);
			Eigen::MatrixXd nullspace = Eigen::MatrixXd::Zero(viewCount*3, colCount);
			for (int v = 0, vv = 0; v < viewCount; v++)
			{ // Transfer nullspace to all views, keep 0 for disabled views
				if (!selectedViews(v)) continue;
				nullspace.middleRows<3>(v*3) = orthComplementB.middleRows<3>(vv*3);
				vv++;
			}
#ifdef PARALLEL
			priv_partN.push_back(nullspace);
			totalColsN.fetch_add(colCount);
			totalPartN.fetch_add(1);
			tested4Tuples++;
#else
			partN.push_back(nullspace);
#endif
			colsN += colCount;
		}
		else
		{
			LOGC(LTrace, "B of rank %d with %d columns is not full rank", rankB, 4+newCols);
		}
	}

#ifdef PARALLEL
	#pragma omp critical
	{
		auto p1 = sclock::now();
		LOGC(LDebug, "Thread %d added %d/%d potential parts in %fms!", omp_get_thread_num(), (int)priv_partN.size(), totalPartN.load(), dtMS(p0, p1));
		if (partN.capacity() != totalPartN.load())
			partN.reserve(totalPartN.load());
		std::copy(priv_partN.begin(), priv_partN.end(), std::back_inserter(partN));
	}
#endif
	}
#ifdef PARALLEL
	omp_set_num_threads(omp_get_max_threads());
#endif

	if (stopToken.stop_requested())
		return NAN;

	// Combine all collected orthogonal complements together as N
	LOGC(LDebug, "N has %d columns in total with %d/%d tuples accepted!", colsN, (int)partN.size(), tested4Tuples);
	if (colsN < minNColumns)
	{
		LOGC(LError, "Basis cannot be reliably determined with column count %d < %d!", colsN, minNColumns);
		LOGC(LError, "Likely hit limits: %d / %d tuples recorded, %d / %d tested.", (int)partN.size(), max4Tuples, tested4Tuples, max4TupleTests);
		return NAN;
	}
	Eigen::MatrixXd N = Eigen::MatrixXd(viewCount*3, colsN);
	int colN = 0;
	for (int i = 0; i < partN.size(); i++)
	{ // Concatenate into N
		N.block(0, colN, viewCount*3, partN[i].cols()) = partN[i];
		colN += partN[i].cols();
	}

	// Get orthogonal complement of dimension 4, which is the target vector space of M (up to noise ofc)
	Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeFullU> svd_N(N);
	basis = svd_N.matrixU().block(0, viewCount*3-4, viewCount*3, 4);
	Eigen::Matrix<double,5,1> rankValues = svd_N.singularValues().tail<5>().reverse();

	float noiseFactor = rankValues(4) / rankValues(3);
	if (noiseFactor < params.basis.minRankFactor)
	{ // Expected full rank due to noise, less than this means not enough 4-tuples were added
		LOGC(LWarn, "Failed to completely constrain target vector space, increase number of regarded 4-tuples or data!");
		LOGC(LWarn, "   Noise factor %f, full rank values (%f, %f, %f, %f) - %f",
			noiseFactor, rankValues(0), rankValues(1), rankValues(2), rankValues(3), rankValues(4));
		LOGC(LWarn, "    Limits: %d Columns within [%d,%d],  %d / %d tuples recorded, %d / %d tested.",
			colsN, minNColumns, maxNColumns, (int)partN.size(), max4Tuples, tested4Tuples, max4TupleTests);
	}

	// Size of basis: 3*recViewCount rows and 4 columns (the basis vectors)
	LOGC(LDebug, "Extracted target vector space basis with noise factor %f (%f / %f)", noiseFactor, rankValues(4), rankValues(3));

	return noiseFactor;
}

[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
int recoverPointData(
	Eigen::MatrixXd &projectiveMatrix,
	MatrixX<BOOL> &projectiveDepthMissing,
	const Eigen::MatrixXd &basis, Eigen::MatrixXd &P_approx)
{
	int pointCount = projectiveMatrix.cols();
	int viewCount = projectiveMatrix.rows()/3;
	assert(projectiveDepthMissing.cols() == pointCount);
	assert(projectiveDepthMissing.rows() == viewCount);
	assert(basis.rows() == viewCount*3);
	assert(basis.cols() == 4);
	assert(P_approx.rows() == 4);
	assert(P_approx.cols() == pointCount);

	int pointsUnrecoverable = 0;
	double recoveryError = 0, confirmationError = 0;
	int pointsRecovered = 0, pointsConfirmed = 0;
	for (int p = 0; p < pointCount; p++)
	{
		if (projectiveDepthMissing.col(p).any())
		{ // Complete point column if possible
			int missingProjDepths = projectiveDepthMissing.col(p).count();
			int usefulRows = viewCount*3 - missingProjDepths*3;
			if (usefulRows < 4)
			{ // TODO: These can be potentially recovered with more iterations of spreading newly estimated projective depths
				LOGC(LTrace, "Col %d cannot be used so far, only %d / %d rows useful!", p, usefulRows, viewCount*3);
				pointsUnrecoverable++;
				continue;
			}
			LOGC(LTrace, "Col %d needs adjustment and correction, with %d / %d rows useless!", p, missingProjDepths*3, viewCount*3);

			// Truncate columns by removing rows that need to be recovered
			Eigen::VectorXd truncatedCol(usefulRows);
			Eigen::MatrixXd truncatedBasis(usefulRows, 4);
			int fc = 0;
			for (int v = 0; v < viewCount; v++)
			{
				if (projectiveDepthMissing(v,p)) continue;
				truncatedCol.segment<3>(fc*3) = projectiveMatrix.block<3,1>(v*3, p);
				truncatedBasis.middleRows<3>(fc*3) = basis.middleRows<3>(v*3);
				fc++;
			}

			// For better conditioning (TODO: Need to apply to coeff inversely)
			//truncatedBasis.colwise().normalise();

			// Get coefficients (3D point) to extrapolate point column (reprojections) from basis (projection matrices)
			auto svd = truncatedBasis.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>();
			if (svd.rank() < 4)
			{
				LOGC(LTrace, "Col %d cannot be used so far, only rank %d with %d / %d rows picked!", p, (int)svd.rank(), usefulRows, viewCount*3);
				pointsUnrecoverable++;
				continue;
			}
			Eigen::Vector4d coeff = svd.solve(truncatedCol);

			// Recover rows using basis and coefficients
			Eigen::VectorXd estCol = basis*coeff;
			for (int v = 0; v < viewCount; v++)
			{
				if (projectiveDepthMissing(v, p))
				{ // Either only projective depth or also point missing
					if (projectiveMatrix.block<3,1>(v*3, p).hasNaN())
					{ // Fill missing point and projective depth with estimation
						projectiveMatrix.block<3,1>(v*3, p) = estCol.segment<3>(v*3);
					}
					else 
					{ // Only update scale (projective depth)
						projectiveMatrix.block<3,1>(v*3, p) *= estCol(v*3+2) / projectiveMatrix(v*3+2, p);
					}
					projectiveDepthMissing(v, p) = 0;
				}
			}

			// Record point
			P_approx.col(p) = coeff;

			// Record recovery error
			pointsRecovered++;
			double error = std::sqrt((estCol - projectiveMatrix.col(p)).squaredNorm()/(viewCount*3));
			recoveryError += error;
			LOGC(LTrace, "Incompleted coeffs %f, %f, %f, %f yielded error %f after recovery!", coeff(0), coeff(1), coeff(2), coeff(3), error);
		}
		else
		{ // Confirm point column with basis

			// Determine point from basis and projectiveMatrix
			Eigen::VectorXd coeff = basis.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(projectiveMatrix.col(p));

			// Record point
			P_approx.col(p) = coeff;
			pointsConfirmed++;

			// Record error
			double error = std::sqrt((basis*coeff - projectiveMatrix.col(p)).squaredNorm()/(viewCount*3));
			confirmationError += error;
			LOGC(LTrace, "Complete coeffs %f, %f, %f, %f confirm base with error %f!", coeff(0), coeff(1), coeff(2), coeff(3), error);
		}
	}
	LOGC(LDebug, "Fully recovered %d columns with %d unrecoverable and %d confirmed!", pointsRecovered, pointsUnrecoverable, pointsConfirmed);
	LOGC(LDebug, "Average recovery error of %f and average confirmation error of %f!", recoveryError/pointsRecovered, confirmationError/pointsConfirmed);

	return pointsUnrecoverable;
}