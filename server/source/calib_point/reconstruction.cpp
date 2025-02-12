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


/* Function Prototypes */

class CenterViewStrategy;
class SuccessiveViewStrategy;

template<typename RecoveryStrategy>
static void estimateProjectiveDepths(const Eigen::Ref<const Eigen::MatrixXd> &normMeasurementMatrix, 
	const PointReconstructionParameters &params,
	Eigen::MatrixXd &projectiveDepthMatrix, Eigen::VectorXi &viewsDiscarded);

static Eigen::MatrixXd determineRank4Basis(const Eigen::Ref<const Eigen::MatrixXd> &projectiveMatrix,
	const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> &projectiveDepthMissing,
	const PointReconstructionParameters &params);

static int recoverPointData(Eigen::Ref<Eigen::MatrixXd> projectiveMatrix,
	Eigen::Ref<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> projectiveDepthMissing,
	const Eigen::Ref<const Eigen::MatrixXd> &basis, Eigen::MatrixXd &P_approx);

static Eigen::Matrix4d getStratificationMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P);

static Eigen::Matrix4d getCorrectionMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P);

static double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::Matrix<double,3,4>> &V, int v,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix);

static double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::MatrixXd> V,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix);

template<typename Scalar>
static void SeparateProjectionMatrix(const Eigen::Ref<const Eigen::Matrix<Scalar,3,3>> matrix, Eigen::Ref<Eigen::Matrix<Scalar,3,3>> projection, Eigen::Ref<Eigen::Matrix<Scalar,3,3>> rotation);

static bool SeparatePerspectiveProjection(Eigen::Matrix<double,3,4> V_v, int v, const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix, CameraCalib &cameraCalib);

/**
 * Normalises the given 3xN row of points by centering and scaling to a distance of sqrt(2), and returns the inverse of that normalisation transform
 */
static Eigen::Matrix3d normalisePointRow(const Eigen::Ref<const Eigen::MatrixXd> &pointRow, Eigen::Ref<Eigen::MatrixXd> normRow);

/**
 * Balances the given (3*M)xN matrix of points so that on average each point (3x1) has the norm 1
 */
static void balanceMatrix3Triplet(Eigen::Ref<Eigen::MatrixXd> matrix);

/**
 * Calculates the fundamental matrix from two corresponding 3xN rows of points (may have NaNs)
 */
static std::pair<int,double> calculateFundamentalMatrix(const Eigen::Ref<const Eigen::MatrixXd> &pointsA, const Eigen::Ref<const Eigen::MatrixXd> &pointsB, Eigen::Matrix3d &fundamentalMatrix, int minPointCount = 8);


/* Functions */

/**
 * Attempts to reconstruct the geometry (points, cameras calibration) of the scene given the observed points
 */
bool reconstructGeometry(const ObsPointData &data, std::vector<CameraCalib> &cameraCalibs, PointReconstructionParameters params)
{
	ScopedLogCategory scopedLogCategory(LPointReconstruction);

	int viewCount = cameraCalibs.size();
	int pointCount = data.points.size();
	if (pointCount < 100)
	{
		LOGC(LError, "Too little data for calibration!\n");
		return false;
	}

	// Build measurement matrix for point lookup and fill with point correspondences
	Eigen::MatrixXd measurementMatrix = Eigen::MatrixXd::Constant(viewCount*3, pointCount, NAN);
	{
		int colIndex = 0;
		for (auto &point : data.points)
		{
			for (auto &sample : point.samples)
				//if (cameraCalibs[sample.camera].id != CAMERA_ID_NONE)
				measurementMatrix.block<3,1>(sample.camera*3, colIndex) =
					undistortPoint<double>(cameraCalibs[sample.camera], sample.point.cast<double>()).homogeneous();
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


	// ----- Projective Depths Estimation
	LOGC(LDebug, "-- Projective Depths Estimation\n");

	// Iteratively determine projective depths according to best strategy
	Eigen::MatrixXd projectiveDepthMatrix; // (viewCount, pointCount)
	Eigen::VectorXi viewsDiscarded; // (viewCount)
	estimateProjectiveDepths<CenterViewStrategy>(normMeasurementMatrix, params, projectiveDepthMatrix, viewsDiscarded);

	int recViewCount = viewCount - viewsDiscarded.count();
	if (recViewCount < 3)
	{ // TODO: Change to 2 once euclidean stratification can work with two cameras
		LOGC(LError, "Cannot recover any views/camera calibrations since only %d/%d had significant overlap!\n", recViewCount, viewCount);
		for (int i = 0; i < viewsDiscarded.size(); i++)
			LOGC(LError, " View %d got recoverable state %d!", i, viewsDiscarded[i]);
		return false;
	}

	// Merge observations and their partial projectiveDepths into projectiveMatrix and discard irrecoverable views
	Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> projectiveDepthMissing(recViewCount, pointCount);
	Eigen::MatrixXd projectiveMatrix(recViewCount*3, pointCount);
	Eigen::MatrixXd recViewNormInv(3, recViewCount*3);
	for (int vv = 0, v = -1; vv < viewCount; vv++)
	{
		if (viewsDiscarded(vv)) continue;
		v++;
		for (int p = 0; p < pointCount; p++)
		{
			projectiveDepthMissing(v,p) = std::isnan(projectiveDepthMatrix(vv,p));
			double projDepth = projectiveDepthMissing(v,p)? 1 : projectiveDepthMatrix(vv,p);
			projectiveMatrix.block<3,1>(v*3, p) = projDepth * normMeasurementMatrix.block<3,1>(vv*3, p);
		}
		recViewNormInv.block<3,3>(0,v*3) = viewNormInv.block<3,3>(0,vv*3);
	}


	// ----- Matrix Balancing

	balanceMatrix3Triplet(projectiveMatrix);

	auto mid1 = pclock::now();


	// ----- Missing Data Extrapolation
	LOGC(LDebug, "-- Missing Data Extrapolation\n");

	// Right now projectiveMatrix has data holes that need to be filled by extrapolation
	// Both due to missing points (NaNs) and missing projective depth (marked by projectiveDepthMissing)
	// For that, find basis for the vector space of rank 4 spanned by the projectiveMatrix
	// Then complete the columns of the projectiveMatrix as linear combinations of that basis
	Eigen::MatrixXd basis = determineRank4Basis(projectiveMatrix, projectiveDepthMissing, params);
	if (basis.hasNaN()) // basis (recViewCount*3, 4)
		return false;
	auto mid2 = pclock::now();
	Eigen::MatrixXd P_approx;  // (4, pointCount)
	int pointsUnrecoverable = recoverPointData(projectiveMatrix, projectiveDepthMissing, basis, P_approx);

	LOGC(LDebug, "Approximated matrix P*X and projectiveMatrix have error of %f RMSE due to differences in projective depths!\n", 
		std::sqrt((projectiveMatrix - (basis*P_approx)).squaredNorm() / (recViewCount*3*pointCount)));

	auto mid3 = pclock::now();


	// ----- Assemble Factorisation Matrix
	LOGC(LDebug, "-- Factorisation\n");

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


	// ----- Rank-4 Factorisation

	Eigen::MatrixXd V_all, P_all;
	{ // Factorise complemented data into View matrices and Points

		Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV> svd_M(factorisationMatrix);
		LOGC(LDebug, "Factorisation matrix with %d unrecoverable columns removed has rank %d (should be 4) with rank values (%.2f, %.2f, %.2f, %.2f) - %.2f!\n", 
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
	LOGC(LDebug, "Reprojection error of factorisation on measurements: %fpx RMSE\n", 
		std::sqrt(calculateReprojectionErrorSq(V_all, P_all, recMeasurementMatrix))*PixelFactor);

	auto mid5 = pclock::now();


	// ----- Bundle Adjustment

	// TODO: Bundle adjustment on P*X?
	// Not necessary for currently tested setups, good enough to immediately go to non-linear optimisation


	// ----- Euclidean Stratification
	LOGC(LDebug, "-- Euclidean Stratification\n");

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
	LOGC(LDebug, "Reprojection error of stratified factorisation on measurements: %fpx RMSE\n", 
		std::sqrt(calculateReprojectionErrorSq(V_corrected, P_corrected, recMeasurementMatrix))*PixelFactor);


	// ----- Projective Factorisation
	LOGC(LDebug, "-- Projective Factorisation\n");

	bool reconstructionSuccess = true;
	
	for (int vv = 0, v = -1; vv < viewCount; vv++)
	{
		LOGC(LDebug, "------------");

		if (viewsDiscarded(vv))
		{
			LOGC(LWarn, "Can not recover view %d!\n", vv);
			continue;
		}
		v++;
		LOGC(LInfo, "Results for view %d:\n", vv);

		// Decompose view projection matrix in V_corrected into camera projection and transformation and update cameraCalibs with it
		if (!SeparatePerspectiveProjection(V_corrected.block<3,4>(v*3,0), v, P_corrected, recMeasurementMatrix, cameraCalibs[vv]))
			reconstructionSuccess = false;
	}
	LOGC(LDebug, "------------");

	auto end = pclock::now();
	LOGC(LDebug, 
		"-- Reconstruction took %.2fms: "
			"%.2fms proj. depth, %.2fms basis, %.2fms filling, %.2fms interim, %.2fms factorisation, %.2fms stratification, %.2fms camera factorisation\n", 
		dt(start, end), 
			dt(start, mid1), 	dt(mid1, mid2), dt(mid2, mid3), dt(mid3, mid4), dt(mid4, mid5), 	dt(mid5, mid6), 		dt(mid6, end));

	return reconstructionSuccess;
}


class CenterViewStrategy {
public:
	Eigen::VectorXi::Index bestView;
	int viewCount, numRecoverable;
	CenterViewStrategy(const Eigen::Ref<const Eigen::MatrixXi> &viewCorrespondences, Eigen::VectorXi &viewsDiscarded, int minCorrespondences)
	{
		auto viewsRecoverable = (viewCorrespondences.array() >= minCorrespondences).cast<int>();
		// Select view with the most overlap to other views
		numRecoverable = viewsRecoverable.colwise().sum().maxCoeff(&bestView);
		viewsDiscarded = 1-viewsRecoverable.col(bestView).array();
		LOGC(LDebug, "Selected view %d as center!\n", (int)bestView);
		viewCount = viewCorrespondences.rows();
	}
	class iterator {
		int CUR, CNT, BEST;
	public:
		iterator(int cur, int cnt, int best) : CNT(cnt), BEST(best)
		{
			CUR = cur == best? cur+1 : cur;
		}
		iterator& operator++()
		{
			if (++CUR == BEST) CUR++;
			return *this;
		}
		iterator operator++(int) {iterator retval = *this; ++(*this); return retval;}
		bool operator==(iterator other) const {return CUR == other.CUR;}
		bool operator!=(iterator other) const {return !(*this == other);}
		std::pair<int,int> operator*() { return { BEST, CUR }; }
		// iterator traits
		using value_type = std::pair<int,int>;
		using pointer = const std::pair<int,int>*;
		using reference = const std::pair<int,int>&;
		using iterator_category = std::input_iterator_tag;
	};
	iterator begin() { return iterator(0, viewCount, bestView); }
	iterator end() { return iterator(viewCount, viewCount, bestView); }
	iterator::value_type front() { return *begin(); }
	iterator::value_type back() { return *end(); }
};

class SuccessiveViewStrategy {
public:
	int viewCount, numRecoverable;
	SuccessiveViewStrategy(const Eigen::Ref<const Eigen::MatrixXi> &viewCorrespondences, Eigen::VectorXi &viewsDiscarded, int minCorrespondences)
	{
		viewCount = viewCorrespondences.rows();
		viewsDiscarded = Eigen::VectorXi::Zero(viewCount);
		numRecoverable = -1; // Just don't know
	}
	class iterator {
		int CUR, CNT;
	public:
		iterator(int cur, int cnt) : CUR(cur), CNT(cnt) {}
		iterator& operator++()
		{
			CUR++;
			return *this;
		}
		iterator operator++(int) {iterator retval = *this; ++(*this); return retval;}
		bool operator==(iterator other) const {return CUR == other.CUR;}
		bool operator!=(iterator other) const {return !(*this == other);}
		std::pair<int,int> operator*() { return { CUR, CUR+1 }; }
		// iterator traits
		using value_type = std::pair<int,int>;
		using pointer = const std::pair<int,int>*;
		using reference = const std::pair<int,int>&;
		using iterator_category = std::input_iterator_tag;
	};
	iterator begin() { return iterator(0, viewCount); }
	iterator end() { return iterator(viewCount-1, viewCount); }
	iterator::value_type front() { return *begin(); }
	iterator::value_type back() { return *end(); }
};

template<typename RecoveryStrategy>
static void estimateProjectiveDepths(const Eigen::Ref<const Eigen::MatrixXd> &normMeasurementMatrix,
	const PointReconstructionParameters &params,
	Eigen::MatrixXd &projectiveDepthMatrix, Eigen::VectorXi &viewsDiscarded)
{
	int pointCount = normMeasurementMatrix.cols();
	int viewCount = normMeasurementMatrix.rows()/3;
	projectiveDepthMatrix = Eigen::MatrixXd::Constant(viewCount, pointCount, NAN);

	Eigen::MatrixXi viewCorrespondences = Eigen::MatrixXi::Zero(viewCount, viewCount);
	Eigen::VectorXi viewOverlap = Eigen::VectorXi(viewCount);
	for (int p = 0; p < pointCount; p++)
	{
		for (int v = 0; v < viewCount; v++)
			viewOverlap(v) = std::isnan(normMeasurementMatrix(v*3,p))? 0 : 1;
		if (viewOverlap.sum() >= 2)
		{
			for (int v = 0; v < viewCount; v++)
			{
				if (viewOverlap(v))
				{
					viewCorrespondences.row(v) += viewOverlap;
					viewCorrespondences.col(v) += viewOverlap;
				}
			}
		}
	}
	
	// currently only one center view is picked and only views with direct overlap to this view will be recovered.
	// TODO: This can be remedied by iterative overlap

	RecoveryStrategy recoverStrategy(viewCorrespondences, viewsDiscarded, params.FM.minPairwiseCorrespondences*2);
	{ // Initialise projective depths of selected camera with 1
		auto transfer = recoverStrategy.front();
		for (int p = 0; p < pointCount; p++)
		{
			if (!normMeasurementMatrix.block<3,1>(transfer.first*3, p).hasNaN())
				projectiveDepthMatrix(transfer.first,p) = 1;
		}
	}

	for (auto transfer : recoverStrategy)
	{ // Iteratively determine projective depths of other views

		int b = transfer.first, v = transfer.second;

		LOGC(LDebug, "View strategy: Transfer from camera %d to camera %d!", b, v);

		// Determine fundamental matrix from rows of measurement matrix
		Eigen::Matrix3d F;
		std::pair<int,double> results = calculateFundamentalMatrix(normMeasurementMatrix.middleRows<3>(b*3), normMeasurementMatrix.middleRows<3>(v*3), F, params.FM.minPairwiseCorrespondences);

		if (results.first < params.FM.minPairwiseCorrespondences || results.second < params.FM.minConfidence)
		{ // View does not have sufficient correspondence with center view 
			// TODO: Multiple recovery strategy iterations until view can be recovered
			viewsDiscarded(v) = 1;
			LOGC(LWarn, "---- Retroactively abandoning view %d because it only shares %d points of %f confidence with best view %d!\n", v, results.first, results.second, b);
			continue;
		}

		// Find epipole as kernel of F
		Eigen::Vector3d e = F.jacobiSvd<Eigen::ComputeFullU>().matrixU().rightCols<1>();

		for (int p = 0; p < pointCount; p++)
		{
			if (!std::isnan(projectiveDepthMatrix(v,p)))
				continue; // Is already determined
			auto x_b = normMeasurementMatrix.block<3,1>(b*3, p);
			if (x_b.hasNaN())
				continue; // Missing data in source camera
			auto x_v = normMeasurementMatrix.block<3,1>(v*3, p);
			if (x_v.hasNaN())
				continue; // Missing data in target camera
			// Determine projective depth inferred from reference projective depth of source camera
			Eigen::Vector3d a = e.cross(x_v);
			projectiveDepthMatrix(v,p) = std::abs(a.dot(F*x_b)/a.squaredNorm() * projectiveDepthMatrix(b,p));
		}
	}
}

static std::vector<std::array<int,4>> generateQuadruplets(int n)
{
	std::vector<std::array<int,4>> quadruplets;
	quadruplets.reserve(n*n*n);
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < n; ++j)
		{
			if (i == j) continue;
			for (int k = 0; k < n; ++k)
			{
				if (i == k || j == k) continue;
				for (int l = 0; l < n; ++l)
				{
					if (i == l || j == l || k == l) continue;
					quadruplets.push_back({ i, j, k, l });
				}
			}
		}
	}
	return quadruplets;
}

static Eigen::MatrixXd determineRank4Basis(const Eigen::Ref<const Eigen::MatrixXd> &projectiveMatrix,
	const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> &projectiveDepthMissing,
	const PointReconstructionParameters &params)
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

	/* auto t0 = sclock::now();
	auto permIndices = generateQuadruplets(pointCount);
	auto t1 = sclock::now();

	LOGC(LDebug, "Took %fms to generate %d quadruplets for %d points!\n", dt(t0,t1), permIndices.size(), pointCount); */

	// Limit the number of maximum possible computation
	// TODO: Better limits, adapted to camera count etc.
	int minNColumns = std::max<int>(params.basis.nColMin, pointCount*params.basis.nColMinFactor);
	int maxNColumns = pointCount*params.basis.nColMaxFactor;
	int max4Tuples = pointCount*params.basis.tupleMaxFactor;
	int max4TupleTests = pointCount*params.basis.tupleTestMaxFactor;
	LOGC(LDebug, "Planning on testing %d random quadruplets of %d points!", max4TupleTests, pointCount);

	std::vector<Eigen::MatrixXd> partN;

	/* #pragma omp parallel
	{
	std::vector<Eigen::MatrixXd> priv_partN;
	auto p0 = sclock::now();

    #pragma omp for nowait schedule(static, 100) reduction(+:colsN)
    for (int j = 0; j < max4TupleTests; j++)
	{
		int indices[4];
		for (int i = 0; i < 4; i++)
		{
			bool searching = true;
			while (searching)
			{
				indices[i] = rand()%pointCount;
				bool unique = true;
				for (int j = 0; j < i; j++)
					unique = unique && indices[j] != indices[i];
				if (unique) searching = false;
				// TODO: Discard columns early if they do not have full rank on shared known points
			}
		}
		
		// Determine number of columns that need to be added
		int newCols = 0;
		for (int i = 0; i < 4; i++)
		{
			int missingPointCoeff = projectiveMatrix.col(indices[i]).array().isNaN().count();
			int missingProjDepths = projectiveDepthMissing.col(indices[i]).count();
			// Add up, account that each missing vector (3 coeff) means one missing proj depth
			newCols += missingPointCoeff + missingProjDepths - missingPointCoeff/3;
		}

		// Extend 4 columns to B
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(viewCount*3, 4+newCols);
		int newColIndex = 4;
		for (int i = 0; i < 4; i++)
		{
			for (int v = 0; v < viewCount; v++)
			{
				Eigen::Vector3d x = projectiveMatrix.block<3,1>(v*3, indices[i]);
				if (x.hasNaN()) 
				{ // Missing point
					B(v*3+0,newColIndex++) = 1;
					B(v*3+1,newColIndex++) = 1;
					B(v*3+2,newColIndex++) = 1;
					continue;
				}
				if (projectiveDepthMissing(v, indices[i])) 
				{ // Still have coordinates, already embedded
					B.block<3,1>(v*3,newColIndex++) = x.normalized();
					continue;
				}
				B.block<3,1>(v*3,i) = x;
			}
			// Make all columns normalised to 1 for conditioning
			//B.col(i).normalise(); // Doesn't actually do much
		}

		// Check that B is full rank, else discard
		Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeFullU> svd_B(B);
		int rankB = svd_B.rank();
		if (rankB >= 4+newCols) // TODO: Still take even if not full rank, any nullspace should be fine
		{ // Extract null space / orthogonal complement of B
			int colCount = viewCount*3-rankB;
			LOGC(LTrace, "B of rank %d has added %d orthogonal columns out of %d dimensions\n", rankB, colCount, viewCount*3);
			Eigen::MatrixXd orthComplementB = svd_B.matrixU().block(0, rankB, viewCount*3, colCount);
			//partN.push_back(orthComplementB);
			priv_partN.push_back(orthComplementB);
			LOGC(LTrace, "B of rank %d is not full rank (%d columns)\n", rankB, 4+newCols);
			colsN += colCount;
		}
		else 
		{
			LOGC(LTrace, "B of rank %d is not full rank (%d columns)\n", rankB, 4+newCols);
		}
    }
	auto p1 = sclock::now();
	LOGC(LDebug, "Thread %d added %d potential parts in %fms!\n", omp_get_thread_num(), priv_partN.size(), dt(p0, p1));
    #pragma omp critical
    partN.insert(partN.end(), priv_partN.begin(), priv_partN.end());
	} */

	//int p = -1;
	while (colsN < maxNColumns && partN.size() < max4Tuples && tested4Tuples++ < max4TupleTests)
	{
		//p++;
		//auto indices = permIndices[p];
		int indices[4];
		for (int i = 0; i < 4; i++)
		{
			bool searching = true;
			while (searching)
			{
				indices[i] = rand()%pointCount;
				bool unique = true;
				for (int j = 0; j < i; j++)
					unique = unique && indices[j] != indices[i];
				if (unique) searching = false;
				// TODO: Discard columns early if they do not have full rank on shared known points
			}
		}

		// Determine number of columns that need to be added
		int newCols = 0;
		for (int i = 0; i < 4; i++)
		{
			int missingPointCoeff = projectiveMatrix.col(indices[i]).array().isNaN().count();
			int missingProjDepths = projectiveDepthMissing.col(indices[i]).count();
			// Add up, account that each missing vector (3 coeff) means one missing proj depth
			newCols += missingPointCoeff + missingProjDepths - missingPointCoeff/3;
		}

		// Extend 4 columns to B
		Eigen::MatrixXd B = Eigen::MatrixXd::Zero(viewCount*3, 4+newCols);
		int newColIndex = 4;
		for (int i = 0; i < 4; i++)
		{
			for (int v = 0; v < viewCount; v++)
			{
				Eigen::Vector3d x = projectiveMatrix.block<3,1>(v*3, indices[i]);
				if (x.hasNaN()) 
				{ // Missing point
					B(v*3+0,newColIndex++) = 1;
					B(v*3+1,newColIndex++) = 1;
					B(v*3+2,newColIndex++) = 1;
					continue;
				}
				if (projectiveDepthMissing(v, indices[i])) 
				{ // Still have coordinates, already embedded
					B.block<3,1>(v*3,newColIndex++) = x.normalized();
					continue;
				}
				B.block<3,1>(v*3,i) = x;
			}
			// Make all columns normalised to 1 for conditioning
			//B.col(i).normalise(); // Doesn't actually do much
		}

		// Check that B is full rank, else discard
		Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeFullU> svd_B(B);
		int rankB = svd_B.rank();
		if (rankB >= 4+newCols) // TODO: Still take even if not full rank, any nullspace should be fine
		{ // Extract null space / orthogonal complement of B
			int colCount = viewCount*3-rankB;
			LOGC(LTrace, "B of rank %d has added %d orthogonal columns out of %d dimensions\n", rankB, colCount, viewCount*3);
			Eigen::MatrixXd orthComplementB = svd_B.matrixU().block(0, rankB, viewCount*3, colCount);
			partN.push_back(orthComplementB);
			colsN += colCount;
		}
		else 
		{
			LOGC(LTrace, "B of rank %d is not full rank (%d columns)\n", rankB, 4+newCols);
		}
	}

	// Combine all collected orthogonal complements together as N
	LOGC(LDebug, "N has %d columns in total with %d/%d tuples accepted!\n", colsN, (int)partN.size(), tested4Tuples);
	if (colsN < minNColumns)
	{
		LOGC(LError, "Basis cannot be reliably determined with column count %d < %d!\n", colsN, minNColumns);
		LOGC(LError, "Likely hit limits: %d / %d tuples recorded, %d / %d tested.\n", (int)partN.size(), max4Tuples, tested4Tuples, max4TupleTests);
		return Eigen::MatrixXd::Constant(viewCount*3, 4, NAN);
	}
	Eigen::MatrixXd N = Eigen::MatrixXd(viewCount*3, colsN);
	int colN = 0;
	for (int i = 0; i < partN.size(); i++)
	{ // Concatenate into N
		N.block(0, colN, viewCount*3, partN[i].cols()) = partN[i];
		colN += partN[i].cols();
	}

	// Get orthogonal complement of dimension 4, which is the target vector space of M (up to noise ofc)
	// TODO: Choose either SVD or EVD
	/*Eigen::BDCSVD<Eigen::MatrixXd, Eigen::ComputeFullU> svd_N(N);
	basis = svd_N.matrixU().block(0, recViewCount*3-4, recViewCount*3, 4);
	Eigen::Matrix<double,5,1> rankValues = svd_N.singularValues().tail<5>().reverse();*/
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> evd_N(N*N.transpose(), Eigen::ComputeEigenvectors);
	Eigen::MatrixXd basis = evd_N.eigenvectors().block(0, 0, viewCount*3, 4);
	Eigen::Matrix<double,5,1> rankValues = evd_N.eigenvalues().head<5>();

	if (rankValues(3)*params.basis.minRankFactor > rankValues(4))
	{ // Expected full rank due to noise, less than this means not enough 4-tuples were added
		LOGC(LWarn, "Failed to completely constrain target vector space, increase number of regarded 4-tuples or data! Rank values (%f, %f, %f, %f) - %f\n",
			rankValues(0), rankValues(1), rankValues(2), rankValues(3), rankValues(4));
		LOGC(LWarn, "Limits: %d Columns within [%d,%d],  %d / %d tuples recorded, %d / %d tested.\n",
			colsN, minNColumns, maxNColumns, (int)partN.size(), max4Tuples, tested4Tuples, max4TupleTests);
	}

	// Size of basis: 3*recViewCount rows and 4 columns (the basis vectors)
	LOGC(LDebug, "Extracted target vector space basis! Noise rank difference %f vs %f\n", rankValues(3), rankValues(4));

	return basis;
}

static int recoverPointData(Eigen::Ref<Eigen::MatrixXd> projectiveMatrix,
	Eigen::Ref<Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>> projectiveDepthMissing,
	const Eigen::Ref<const Eigen::MatrixXd> &basis, Eigen::MatrixXd &P_approx)
{
	assert(projectiveMatrix.cols() == projectiveDepthMissing.cols());
	assert(projectiveMatrix.rows() == projectiveDepthMissing.rows()*3);
	assert(projectiveMatrix.rows() == basis.rows());
	assert(basis.cols() == 4);
	int pointsUnrecoverable = 0;
	int pointCount = projectiveMatrix.cols();
	int viewCount = projectiveMatrix.rows()/3;
	P_approx = Eigen::MatrixXd::Zero(4, pointCount);

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
				LOGC(LTrace, "Col %d cannot be used so far, only %d / %d rows useful!\n", p, usefulRows, viewCount*3);
				pointsUnrecoverable++;
				continue;
			}
			LOGC(LTrace, "Col %d needs adjustment and correction, with %d / %d rows useless!\n", p, missingProjDepths*3, viewCount*3);

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
			Eigen::Vector4d coeff = truncatedBasis.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(truncatedCol);

			// Recover rows using basis and coefficients
			Eigen::VectorXd estCol = basis*coeff;
			for (int v = 0; v < viewCount; v++)
			{
				if (projectiveDepthMissing(v,p))
				{ // Either only projective depth or also point missing
					if (projectiveMatrix.block<3,1>(v*3, p).hasNaN())
					{ // Fill missing point and projective depth with estimation
						projectiveMatrix.block<3,1>(v*3, p) = estCol.segment<3>(v*3);
					}
					else 
					{ // Only update scale (projective depth)
						projectiveMatrix.block<3,1>(v*3, p) *= estCol(v*3+2) / projectiveMatrix(v*3+2, p);
					}
					projectiveDepthMissing(v,p) = false;
				}
			}

			// Record point
			P_approx.col(p) = coeff;

			// Record recovery error
			pointsRecovered++;
			double error = std::sqrt((estCol - projectiveMatrix.col(p)).squaredNorm()/(viewCount*3));
			recoveryError += error;
			LOGC(LTrace, "Incompleted coeffs %f, %f, %f, %f yielded error %f after recovery!\n", coeff(0), coeff(1), coeff(2), coeff(3), error);
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
			LOGC(LTrace, "Complete coeffs %f, %f, %f, %f confirm base with error %f!\n", coeff(0), coeff(1), coeff(2), coeff(3), error);
		}
	}
	LOGC(LDebug, "Fully recovered %d columns with %d unrecoverable and %d confirmed!", pointsRecovered, pointsUnrecoverable, pointsConfirmed);
	LOGC(LDebug, "Average recovery error of %f and average confirmation error of %f!\n", recoveryError/pointsRecovered, confirmationError/pointsConfirmed);

	return pointsUnrecoverable;
}

static Eigen::Matrix4d getStratificationMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P)
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
			LOGC(LDebug, "Flipping Q because diagonal of MM^T is negative (checked %f)!\n", MMT(1,1));
			Q = -Q;
		}
	}

	LOGC(LDebug, "Stratification solve errors: B: %f, Q: %f\n", (B_solve*B).norm(), (Q_solve*Qc).norm());

	// Factor symmetric Q into AA^T of rank 3
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> evd_Q(Q, Eigen::ComputeEigenvectors);
	Eigen::Matrix<double,4,3> A;
	A.col(0) = evd_Q.eigenvectors().col(3) * std::sqrt(evd_Q.eigenvalues()(3));
	A.col(1) = evd_Q.eigenvectors().col(2) * std::sqrt(evd_Q.eigenvalues()(2));
	A.col(2) = evd_Q.eigenvectors().col(1) * std::sqrt(evd_Q.eigenvalues()(1));

	LOGC(LDebug, "Eigenvalues of Q: (%f, %f, %f) - %f\n", evd_Q.eigenvalues()(3), evd_Q.eigenvalues()(2), evd_Q.eigenvalues()(1), evd_Q.eigenvalues()(0));

	// Apply correction to camera matrices and positions
	Eigen::Matrix4d H;
	H << A, B;
	return H;
}


static Eigen::Matrix4d getCorrectionMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P)
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
			LOGC(LDarn, "    View matrix %d is flipped!\n", v);
			V_v = -V_v;
		}

		Eigen::Matrix3d P_v, R_v;
		SeparateProjectionMatrix<double>(V_v, P_v, R_v);

		if (R_v.determinant() <= 0)
		{ // Should not happen afaik
			LOGC(LWarn, "    View matrix %ds' rotation matrix was invalid!\n", v);
			continue;
		}

		// Apply needed correction to H
		if (P_v(0,0) < 0)
			H(0,0) = -1;
		if (P_v(1,1) < 0)
			H(1,1) = -1;
		if (P_v(2,2) < 0)
			H(2,2) = -1;
		LOGC(LDebug, "View %d provided a correction of %f,%f,%f,%f\n", v, H(0,0), H(1,1), H(2,2), H(3,3));

		// Should only need to accommodate one valid camera to correct all of them
		break;
	}

	return H;
}

static double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::Matrix<double,3,4>> &V, int v,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix)
{
	assert(P.rows() == 4);
	assert(P.cols() == measurementMatrix.cols());

	int ptCnt = 0;
	double reprojectionErrorSq = 0.0;
	for (int p = 0; p < measurementMatrix.rows(); p++)
	{
		Eigen::Vector2d measure = measurementMatrix.block<2,1>(v*3,p);
		if (measure.hasNaN()) continue;
		Eigen::Vector3d reprojection = V * P.block<4,1>(0,p);
		reprojectionErrorSq += (measure-reprojection.hnormalized()).squaredNorm();
		ptCnt++;
	}
	return reprojectionErrorSq / ptCnt;
	
}

static double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::MatrixXd> V,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix)
{
	assert(V.cols() == 4);
	assert(P.rows() == 4);
	assert(V.rows() == measurementMatrix.rows());
	assert(P.cols() == measurementMatrix.cols());

	// Test reprojection error on original (non-estimated) measurements
	int ptCnt = 0;
	double reprojectionErrorSq = 0;
	for (int v = 0; v < measurementMatrix.rows(); v+=3)
	{
		for (int p = 0; p < measurementMatrix.cols(); p++)
		{
			Eigen::Vector2d measure = measurementMatrix.block<2,1>(v,p);
			if (measure.hasNaN()) continue;
			Eigen::Vector3d reprojection = V.block<3,4>(v,0) * P.block<4,1>(0,p);
			reprojectionErrorSq += (measure-reprojection.hnormalized()).squaredNorm();
			ptCnt++;
		}
	}
	return reprojectionErrorSq / ptCnt;
}

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

static bool SeparatePerspectiveProjection(Eigen::Matrix<double,3,4> V_v, int v, const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix, CameraCalib &cameraCalib)
{
	assert(P.rows() == 4);
	assert(P.cols() == measurementMatrix.cols());

	LOGC(LDebug, "Reprojection error of view %d initially: %fpx RMSE\n", v, std::sqrt(calculateReprojectionErrorSq(V_v, v, P, measurementMatrix))*PixelFactor);

	// Check polarity
	if ((V_v * P).row(2).sum() < 0)
	{
		LOGC(LDebug, "Flipping view matrix!\n");
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
		LOGC(LWarn, "Got uneven amount of negatives (%d), will not be able to correctly separate matrices for view %d!\n", neg, v);

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

	auto verifyDecomposition = [&](Eigen::Matrix3d P_v, Eigen::Matrix3d R_v, Eigen::Vector3d C_T)
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
		LOGC(LInfo, "    Final reprojection error across %d recovered observations: %fpx RMSE\n", ptCnt, reprojectionError*PixelFactor);

		if (R_v.determinant() < 0)
		{ // Should not happen, if so then getCorrectionMatrix failed
			LOGC(LWarn, "    ----- Rotation matrix invalid!!!!\n");
			return false;
		}
		if (front < back)
		{ // Should not happen, if so then getCorrectionMatrix failed
			LOGC(LWarn, "    ----- View matrices flipped!!!!\n");
			return false;
		}
		if (reprojectionError*PixelFactor > 10)
		{
			LOGC(LInfo, "    ----- Unusually high reprojection RMSE!\n");
			return true;
		}
		return true;
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

/**
 * Normalises the given 3xN row of points by centering and scaling to a distance of sqrt(2) and writes it into normRow, and returns the inverse of that normalisation transform
 */
static Eigen::Matrix3d normalisePointRow(const Eigen::Ref<const Eigen::MatrixXd> &pointRow, Eigen::Ref<Eigen::MatrixXd> normRow)
{
	// Data normalisation, center and scale raw points
	Eigen::Vector2d imageCentroid = Eigen::Vector2d::Zero();
	double imageScale = 0;
	int imagePointCount = 0;

	// Get centroids
	for (int p = 0; p < pointRow.cols(); p++)
	{
		if (pointRow.col(p).hasNaN()) continue;
		imageCentroid += pointRow.block<2,1>(0,p) / pointRow(2,p);
		imagePointCount++;
	}
	imageCentroid /= imagePointCount;

	// Apply centroids and get scale
	for (int p = 0; p < pointRow.cols(); p++)
	{
		if (pointRow.col(p).hasNaN()) continue;
		normRow.block<2,1>(0,p) = pointRow.block<2,1>(0,p)/pointRow(2,p) - imageCentroid;
		normRow(2,p) = 1;
		imageScale += normRow.block<2,1>(0,p).norm();
	}
	imageScale /= imagePointCount*sqrt(2);

	// Apply scale
	for (int p = 0; p < pointRow.cols(); p++)
	{
		if (pointRow.col(p).hasNaN())
			normRow.col(p).setConstant(NAN);
		else
			normRow.block<2,1>(0,p) /= imageScale;
	}

	// Build normalisation matrices to invert normalisations later
	Eigen::Matrix3d normInv = Eigen::Vector3d(imageScale,imageScale,1).asDiagonal();
	normInv.block<2,1>(0,2) = imageCentroid;
	return normInv;
}

/**
 * Balances the given (3*M)xN matrix of points so that on average each point (3x1) has the norm 1
 */
static void balanceMatrix3Triplet(Eigen::Ref<Eigen::MatrixXd> matrix)
{
	int rows = matrix.rows()/3, cols = matrix.cols();

	// Perform second balancing on just the projective depths and point scales (proxy matrix)
	int maxIt = 10;
	double avgRowChange, avgColChange;
	do
	{
		maxIt--;
		avgRowChange = 0;
		for (int r = 0; r < rows; r++)
		{
			int ptCnt = 0;
			double ptSum = 0;
			for (int v = 0; v < cols; v++)
			{
				if (!matrix.block<3,1>(r*3,v).hasNaN())
				{
					ptCnt++;
					ptSum += matrix.block<3,1>(r*3,v).squaredNorm();
				}
			}
			double rowScaleSq = std::sqrt(ptCnt/ptSum);
			matrix.middleRows<3>(r*3) *= rowScaleSq;
			avgRowChange += std::abs(rowScaleSq-1);
		}
		avgRowChange /= rows;
		avgColChange = 0;
		for (int v = 0; v < cols; v++)
		{
			int ptCnt = 0;
			double ptSum = 0;
			for (int r = 0; r < rows; r++)
			{
				if (!matrix.block<3,1>(r*3,v).hasNaN())
				{
					ptCnt++;
					ptSum += matrix.block<3,1>(r*3,v).squaredNorm();
				}
			}
			double colScaleSq = std::sqrt(ptCnt/ptSum);
			matrix.col(v) *= colScaleSq;
			avgColChange += std::abs(colScaleSq-1);
		}
		avgColChange /= cols;
		LOGC(LTrace, "Balancing iteration change: row %f, col %f\n", avgRowChange, avgColChange);
	} while ((avgRowChange > 0.01 || avgColChange > 0.01) && maxIt > 0);
}

/**
 * Calculates the fundamental matrix from two corresponding 3xN rows of points (may have NaNs)
 */
static std::pair<int,double> calculateFundamentalMatrix(const Eigen::Ref<const Eigen::MatrixXd> &pointsA, const Eigen::Ref< const Eigen::MatrixXd> &pointsB, Eigen::Matrix3d &fundamentalMatrix, int minPointCount)
{
	int colCount = std::min(pointsA.cols(), pointsB.cols());
	if (colCount < minPointCount)
	{
		LOGC(LWarn, "    Cannot calculate fundamental matrix from %d points!\n", colCount);
		return { colCount, 0 };
	}
	int pointCount = 0;
	for (int p = 0; p < colCount; p++)
		if (!pointsA.col(p).hasNaN() && !pointsB.col(p).hasNaN())
			pointCount++;
	LOGC(LDebug, "Calculating fundamental matrix with %d/%d overlapping points!", pointCount, colCount);
	if (pointCount < minPointCount)
	{
		LOGC(LWarn, "    Cannot calculate fundamental matrix from %d/%d non-nan points!\n", pointCount, colCount);
		return { pointCount, 0 };
	}

	Eigen::MatrixXd normA = Eigen::MatrixXd(3, pointsA.cols());
	Eigen::MatrixXd normB = Eigen::MatrixXd(3, pointsB.cols());
	Eigen::Matrix3d TA = normalisePointRow(pointsA, normA);
	Eigen::Matrix3d TB = normalisePointRow(pointsB, normB);

	// Build data matrix
	Eigen::MatrixXd dataMatrix(pointCount, 9);
	int r = 0;
	for (int p = 0; p < colCount; p++)
	{
		if (pointsA.col(p).hasNaN() || pointsB.col(p).hasNaN()) continue;
		Eigen::Vector2d a = normA.block<2,1>(0,p);
		Eigen::Vector2d b = normB.block<2,1>(0,p);
		Eigen::Matrix<double, 1, 9> fcoeff;
		fcoeff << a.x()*b.x(), a.y()*b.x(), b.x(), a.x()*b.y(), a.y()*b.y(), b.y(), a.x(), a.y(), 1.0;
		dataMatrix.row(r++) = fcoeff;
	} 


	double confidence = solveFundamentalMatrix(dataMatrix, fundamentalMatrix);

	// Revert normalisations
	fundamentalMatrix = TB.transpose() * fundamentalMatrix * TA;

	// Frobenius Norm
	//fundamentalMatrix->normalise();
	// Alternative Norm
	/*Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd_rank(fundamentalMatrix);
	Eigen::DiagonalMatrix<double, 3> truncatedSV(1, svd_rank.singularValues().y()/svd_rank.singularValues().x(), 0);
	fundamentalMatrix = svd_rank.matrixU() * truncatedSV * svd_rank.matrixV().transpose();*/

	// Verify error
	/*double error = 0;
	for (int p = 0; p < colCount; p++)
	{
		if (!pointsA.col(p).hasNaN() && !pointsB.col(p).hasNaN())
			error += std::abs((pointsB.template block<3,1>(0,p)/pointsB(2,p)).transpose() * fundamentalMatrix * (pointsA.template block<3,1>(0,p)/pointsA(2,p)));
	}
	error /= pointCount;
	LOGC(LDebug, "    Fundamental matrix leaves %f average point error and has %f confidence\n", error, confidence);*/

	return { pointCount, confidence };
}