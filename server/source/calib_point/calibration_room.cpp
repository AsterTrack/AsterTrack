/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "calibration_room.hpp"

#include "calib/optimisation.hpp"
#include "calib/opt/utilities.hpp" // getErrorStats
#include "calib/opt/ReprojectionError.hpp"

#include "util/log.hpp"
#include "util/eigenutil.hpp"
#include "util/eigenalg.hpp"

#include "dbscan/dbscan.hpp"

#include "Eigen/Eigenvalues"

/**
 * Calibration of room floor and extends
 */

/**
 * Calculates, logs and returns overall camera reprojection errors. Also marks all outliers
 * Returns avg, stdDev, max in -1 to 1 coordinates
 */
OptErrorRes determinePointOutliers(ObsData &data, const std::vector<CameraCalib> &cameraCalibs, const PointOutlierErrors &params)
{
	if (data.points.totalSamples == 0 && data.targets.empty())
	{
		LOGC(LError, "Got no points nor targets in optimisation data!");
		return {};
	}
	ScopedLogCategory optLogCategory(LOptimisation, false);

	assert(data.targets.empty());

	typedef ScalarInternal Scalar;
	std::vector<CameraCalib_t<Scalar>> camerasInternal(cameraCalibs.size());
	for (int c = 0; c < cameraCalibs.size(); c++)
		camerasInternal[c] = cameraCalibs[c];

	// Initialise optimisation error term, preparing the data (minus existing outliers)
	ReprojectionError<Scalar> errorTerm(camerasInternal, data);
	if (errorTerm.values() == 0)
	{
		LOGC(LWarn, "Got no data to calculate outliers for!\n");
		return {};
	}
	VectorX<Scalar> errorVec(errorTerm.values());
	VectorX<Scalar> cameraErrorsIn = VectorX<Scalar>::Zero(cameraCalibs.size());
	VectorX<Scalar> cameraErrorsTotal = VectorX<Scalar>::Zero(cameraCalibs.size());
	std::vector<int> cameraInCounts(cameraCalibs.size(), 0);
	std::vector<int> cameraTotalCounts(cameraCalibs.size(), 0);

	// Create initial error stats
	errorTerm.m_options.ignoreOutliers = false;
	errorTerm.calculateError(camerasInternal, errorVec);
	if (errorVec.hasNaN())
	{
		LOGC(LWarn, "Error Vector has NaNs!\n");
		LOGC(LWarn, "Error Vector size: %d, tri points count: %d!\n", (int)errorVec.size(), (int)data.points.points.size());
	}

	// Initialise grid buckets for each camera
	struct Bucket
	{
		uint32_t num;
		float errorSum;
		float errorM2;
	};
	thread_local std::vector<std::vector<Bucket>> gridBuckets;
	auto getBucket = [&params, &cameraCalibs](int c, Eigen::Vector2f pos) {
		int x = std::min((int)((pos.x()+1)/2 * params.gridSize.x()), params.gridSize.x()-1);
		int y = std::min((int)((pos.y()+1)/2 * params.gridSize.y()), params.gridSize.y()-1);
		return &gridBuckets[c][y * params.gridSize.x() + x];
	};
	if (params.gridSize.prod() > 0)
	{
		gridBuckets.resize(cameraCalibs.size());
		for (int c = 0; c < gridBuckets.size(); c++)
		{
			gridBuckets[c].clear();
			gridBuckets[c].resize(params.gridSize.prod(), { 0, 0, 0 });
		}
	}

	// Get unfiltered errors for each camera and for each grid bucket
	int index = 0;
	for (auto &point : data.points.points)
	{
		/* if (point.outlier)
		{ // TODO: Let outliers keep influencing stats used to determine more outliers?
			index += point.samples.size();
			continue;
		} */
		for (auto &sample : point.samples)
		{
			if (std::isnan(errorVec(index)))
			{
				index++;
				continue;
			}
			cameraTotalCounts[sample.camera]++;
			cameraErrorsTotal(sample.camera) += errorVec(index)*errorVec(index);
			// Update bucket
			auto &bucket = *getBucket(sample.camera, sample.point);
			double avgBefore = bucket.num == 0? errorVec(index) : bucket.errorSum/bucket.num;
			double deviationBefore = errorVec(index)-avgBefore;
			// Update num and sum
			bucket.num++;
			bucket.errorSum += errorVec(index);
			// Update M2 value
			double avgAfter = bucket.errorSum/bucket.num;
			double deviationAfter = errorVec(index)-avgAfter;
			bucket.errorM2 += deviationBefore * deviationAfter;
			index++;
		}
	}

	// Convert intermediate stats for each bucket to final ones
	for (int c = 0; c < cameraCalibs.size(); c++)
	{
		for (int b = 0; b < gridBuckets[c].size(); b++)
		{
			auto &bucket = gridBuckets[c][b];
			// Calculate average
			bucket.errorSum /= bucket.num;
			// Calculate standard deviation from M2
			bucket.errorM2 = std::sqrt(bucket.errorM2 / bucket.num);
		}
	}

	// Compile error stats with outliers and set outlier limits
	auto baseErrors = getErrorStats(errorVec, 0);
	double outlierLimit = baseErrors.mean + params.sigma.consider*baseErrors.stdDev;
	double outlierLimitForce = baseErrors.mean + params.sigma.force*baseErrors.stdDev;

	// Mark outliers, using global error limits as well as each grid bucket to veto
	index = 0;
	int newOutlierPoints = 0, newOutlierSamples = 0;
	int savedPoints = 0, savedSamples = 0;
	for (auto &point : data.points.points)
	{
		if (point.outlier)
		{ // Probably just keep as outlier
			for (auto &sample : point.samples)
				errorVec(index++) = -0.0f;
			continue;
		}
		bool isOutlier = false, isOutlierForce = false, bucketSave = false;
		for (int s = 0; s < point.samples.size(); s++)
		{
			auto &sample = point.samples[s];
			Scalar error = errorVec(index + s);
			if (std::isnan(error))
				isOutlierForce = true;
			if (error > outlierLimit)
				isOutlier = true;
			if (error > outlierLimitForce)
				isOutlierForce = true;
			if (error < outlierLimitForce)
			{
				auto &bucket = *getBucket(sample.camera, sample.point);
				if (bucket.num < errorVec.size()/params.gridSize.prod()/2 && error < bucket.errorSum+bucket.errorM2*params.sigma.bucket)
				{ // Veto any outlier observation
					bucketSave = true;
				}
			}
		}
		if (isOutlierForce || (isOutlier && !bucketSave))
		{ // New Outlier
			point.outlier = true;
			data.points.outlierPoints++;
			data.points.outlierSamples += point.samples.size();
			newOutlierPoints++;
			newOutlierSamples += point.samples.size();
			for (auto &sample : point.samples)
				errorVec(index++) = -0.0f;
		}
		else
		{
			if (isOutlier)
			{
				savedPoints++;
				savedSamples += point.samples.size();
			}
			for (auto &sample : point.samples)
			{
				cameraInCounts[sample.camera]++;
				float error = errorVec(index++);
				cameraErrorsIn(sample.camera) += error*error;
			}
		}
	}

	// Compile error stats without outliers
	LOGCL("-- Determined new outliers: %d points (%d saved), %d samples (%d saved)!\n",
		newOutlierPoints, savedPoints, newOutlierSamples, savedSamples);
	OptErrorRes filteredErrors = getErrorStats(errorVec, data.points.outlierSamples);
	if (SHOULD_LOGCL())
	{
		Scalar optError = errorVec.cwiseProduct(errorVec).stableNorm();
		int over1PxCount = (errorVec.array() > (1.0 / PixelFactor)).count();
		int over10PxCount = (errorVec.array() > (10.0 / PixelFactor)).count();
		LOGCL("    Error Filtered: %lf opt -- %fpx +- %fpx, %fpx max -- RMSE %lfpx -- Of %d, %d > 10px, %d > 1px, %d < 1px, %d outliers\n",
			optError, filteredErrors.mean*PixelFactor, filteredErrors.stdDev*PixelFactor, filteredErrors.max*PixelFactor, filteredErrors.rmse*PixelFactor,
			(int)errorVec.size(), over10PxCount, over1PxCount, (int)errorVec.size()-over1PxCount-data.points.outlierSamples, data.points.outlierSamples);
		for (int c = 0; c < cameraCalibs.size(); c++)
			LOGCL("        Camera %d has %fpx RMSE reprojection error without outliers\n",
				c, std::sqrt(cameraErrorsIn(c)/cameraInCounts[c])*PixelFactor);
	}
	return filteredErrors;
}

template<typename Scalar>
void StaticPointSamples<Scalar>::update(const std::vector<CameraCalib> &calibs)
{
	std::vector<std::vector<Vector2<Scalar>>> samplePoints(calibs.size());
	std::vector<const std::vector<Vector2<Scalar>>*> samplePointGroups(calibs.size());
	TriangulatedPoint_t<Scalar> sampleTri(pos, 0, 0, calibs.size());
	for (int c = 0; c < calibs.size(); c++)
	{
		samplePointGroups[c] = &samplePoints[c];
		if (samples[c].first == 0) continue;
		sampleTri.blobs[c] = 0;
		samplePoints[c].push_back(samples[c].second / samples[c].first);
	}
	pos = refineTriangulationIterative<Scalar>(samplePointGroups, calibs, sampleTri);
}


template void StaticPointSamples<double>::update(const std::vector<CameraCalib> &calibs);

/**
 * Calibrate a transformation to the room coordinate system given 3 or more calibrated points on the floor, with the first being the new origin
 * The distance between the first two points should be passed as distance12 to determine the scale
 */
template<typename Scalar>
int estimateFloorTransform(const std::vector<CameraCalib_t<Scalar>> &calibs, const std::vector<StaticPointSamples<Scalar>> &floorPoints, Scalar distance12,
	Matrix3<Scalar> &roomOrientation, Affine3<Scalar> &roomTransform)
{
	if (floorPoints.size() < 3 || floorPoints.front().confidence < 1)
	{
		LOG(LPointCalib, LError, "    Origin (Point 1) is not calibrated!\n");
		return 0;
	}
	int pointCount = 0;
	for (int i = 0; i < floorPoints.size(); i++)
	{
		if (floorPoints[i].confidence > 1)
			pointCount++;
	}
	if (pointCount < 3)
	{
		LOG(LPointCalib, LError, "    Got %d points calibrated, need at least 3 to calibrate the floor!\n", pointCount);
		return 0;
	}
	Vector3<Scalar> origin = floorPoints.front().pos;

	// Perform Principle Component Analysis to get the least represented axis, which is our axis going vertically through the plane of the points
	Eigen::Matrix3X<Scalar> N = Eigen::Matrix3X<Scalar>(3, pointCount);
	int index = 0;
	for (int i = 0; i < floorPoints.size(); i++)
	{
		if (floorPoints[i].confidence > 1)
			N.col(index++) = floorPoints[i].pos-origin;
	}
	// TODO: Choose either SVD or EVD
	/* Eigen::JacobiSVD<Matrix3<Scalar>, Eigen::ComputeFullU> svd_N(N);
	Vector3<Scalar> axis = svd_N.matrixU().block<3,1>(0, 2);
	Vector3<Scalar> rankValues = svd_N.singularValues().tail<3>().reverse(); */
	Eigen::SelfAdjointEigenSolver<Matrix3<Scalar>> evd_N(N*N.transpose(), Eigen::ComputeEigenvectors);
	Vector3<Scalar> axis = evd_N.eigenvectors().template block<3,1>(0, 0);
	Vector3<Scalar> rankValues = evd_N.eigenvalues().template head<3>();
	LOG(LPointCalib, LDebug, "    PCA for rotation has ranks (%f, %f, %f)\n", rankValues(2), rankValues(1), rankValues(0));

	// TODO: Account for marker size by shifting floor plane up by their radius
	// Supporting varying sizes sounds like a pain though

	// Flip room up axis based on camera positions
	Scalar flipAxis = 0;
	for (auto &calib : calibs)
		if (!calib.invalid())
			flipAxis += axis.dot(calib.transform.translation() - origin);
	if (flipAxis < 0)
		axis = -axis;

	if (axis.z() < 0) axis = -axis;

	if (rankValues(0)*10 > rankValues(1))
	{
		LOG(LPointCalib, LError, "    Failed to completely constrain the floor orientation! Rank values (%f, %f) - %f\n", rankValues(2), rankValues(1), rankValues(1));
		return 0;
	}

	// Get transform to calibrated room from up axis, origin and scale
	roomOrientation = Eigen::Quaternion<Scalar>::FromTwoVectors(axis, Vector3<Scalar>::UnitZ()).toRotationMatrix();
	Scalar scaling = distance12 / N.col(1).norm();
	roomTransform.linear() = roomOrientation * Eigen::DiagonalMatrix<Scalar,3>(scaling, scaling, scaling);
	roomTransform.translation() = -roomTransform.linear()*origin;
	return pointCount;
}
template int estimateFloorTransform(const std::vector<CameraCalib_t<double>> &calibs, const std::vector<StaticPointSamples<double>> &floorPoints, double distance12,
	Eigen::Matrix3d &roomOrientation, Eigen::Affine3d &roomTransform);
template int estimateFloorTransform(const std::vector<CameraCalib_t<float>> &calibs, const std::vector<StaticPointSamples<float>> &floorPoints, float distance12,
	Eigen::Matrix3f &roomOrientation, Eigen::Affine3f &roomTransform);

/**
 * Normalise the calibration
 */
template<typename Scalar>
void getCalibNormalisation(const std::vector<CameraCalib_t<Scalar>> &calibs, Matrix3<Scalar> &roomOrientation,
	Affine3<Scalar> &roomTransform, Scalar height, Scalar pairwiseDist)
{
	Vector3<Scalar> origin = Vector3<Scalar>::Zero();
	int c = 0;
	for (auto &calib : calibs)
	{
		if (calib.invalid()) continue;
		origin += calib.transform.translation();
		c++;
	}
	if (c < 2) return; 
	origin /= c;

	// Perform Principle Component Analysis to get the least represented axis, which is our axis going vertically through the plane of the points
	Eigen::Matrix3X<Scalar> N = Eigen::Matrix3X<Scalar>(3, c);
	c = 0;
	for (auto &calib : calibs)
		if (!calib.invalid())
			N.col(c++) = calib.transform.translation()-origin;

	// TODO: Choose either SVD or EVD
	Eigen::JacobiSVD<Eigen::Matrix3X<Scalar>, Eigen::ComputeFullU> svd_N(N);
	Vector3<Scalar> axis = svd_N.matrixU().template block<3,1>(0, 2);
	//Vector3<Scalar> rankValues = svd_N.singularValues().template tail<3>().reverse();
	/* Eigen::SelfAdjointEigenSolver<Eigen::Matrix3X<Scalar>> evd_N(N*N.transpose(), Eigen::ComputeEigenvectors);
	Vector3<Scalar> axis = evd_N.eigenvectors().template block<3,1>(0, 0);
	Vector3<Scalar> rankValues = evd_N.eigenvalues().template head<3>(); */

	// Flip room up axis based on camera positions
	Scalar flipAxis = 0;
	for (auto &calib : calibs)
		if (!calib.invalid())
			flipAxis += axis.dot(calib.transform.translation() - origin);
	if (flipAxis < 0)
		axis = -axis;

	// Get transform to calibrated room from up axis, origin and scale
	roomOrientation = Eigen::Quaternion<Scalar>::FromTwoVectors(axis, Vector3<Scalar>::UnitZ()).toRotationMatrix();
	Scalar scaling = pairwiseDist/2 / N.colwise().norm().mean();
	roomTransform.linear() = roomOrientation * Eigen::DiagonalMatrix<Scalar,3>(scaling, scaling, scaling);
	roomTransform.translation() = -roomTransform.linear()*origin + Vector3<Scalar>(0, 0, height);
}
template void getCalibNormalisation(const std::vector<CameraCalib_t<double>> &calibs,
	Eigen::Matrix3d &roomOrientation,Eigen::Affine3d &roomTransform, double height, double pairwiseDist);
template void getCalibNormalisation(const std::vector<CameraCalib_t<float>> &calibs,
	Eigen::Matrix3f &roomOrientation, Eigen::Affine3f &roomTransform, float height, float pairwiseDist);

/**
 * Attempt to transfer the room calibration between calibrations - cameras should match and mostly retain their relations
 * This is intended to be used between optimisations to ensure the room calibration isn't eroded away
 * But it can also transfer if cameras moved as long as two didn't move
 */
template<typename Scalar>
bool transferRoomCalibration(const std::vector<CameraCalib_t<Scalar>> &calibsSrc, const std::vector<CameraCalib_t<Scalar>> &calibsTgt,
	Matrix3<Scalar> &roomOrientation, Affine3<Scalar> &roomTransform)
{
	roomTransform.setIdentity();
	roomOrientation.setIdentity();

	struct Correction
	{
		float weight = 0;
		int agreeingCamera;
		Isometry3<Scalar> transform = Isometry3<Scalar>::Identity();
		float scale;
	};
	std::vector<Correction> guesses;
	std::vector<float> guessesScale;
	int unchangedCameras = 0;
	for (int c = 0; c < calibsSrc.size(); c++)
	{
		if (!calibsSrc[c].valid()) continue;
		if (!calibsTgt[c].valid()) continue;
		// Guess scale by relation to other cameras
		std::vector<Eigen::Vector<Scalar,1>> scaleGuesses;
		for (int cc = 0; cc < calibsSrc.size(); cc++)
		{
			if (c == cc) continue;
			if (!calibsSrc[cc].valid()) continue;
			if (!calibsTgt[cc].valid()) continue;
			float distSrc = (calibsSrc[c].transform.translation() - calibsSrc[cc].transform.translation()).norm();
			float distTgt = (calibsTgt[c].transform.translation() - calibsTgt[cc].transform.translation()).norm();
			scaleGuesses.push_back(Eigen::Vector<Scalar,1>(distSrc/distTgt));
		}
		if (scaleGuesses.empty()) continue;
		auto scaleGroups = dbscan<1,double,int>(scaleGuesses, 0.01, 2);
		if (!scaleGroups.empty())
		{ // Found agreeing scales, take scale with most agreements as only guess
			float scale = 0.0f;
			for (int i : scaleGroups.front())
				scale += scaleGuesses[i](0);
			scale /= scaleGroups.front().size();
			if (scale < 0.001f || scale > 10000.0f)
				continue;
			scaleGuesses = { Eigen::Vector<Scalar,1>(scale) };
		}
		LOG(LPointCalib, LTrace, "Finding corrective transform for camera %d, %d scale guesses (after %d groups)",
			c, (int)scaleGuesses.size(), (int)scaleGroups.size());

		for (auto scale : scaleGuesses)
		{
			// Find corrective transformation assuming scale guess is correct
			Isometry3<Scalar> corr = calibsTgt[c].transform;
			corr.translation() *= scale(0);
			corr = calibsSrc[c].transform * corr.inverse();

			int agreeing = 0;
			float errorSum = 0;
			for (int cc = 0; cc < calibsSrc.size(); cc++)
			{
				Isometry3<Scalar> corrDiff = calibsTgt[c].transform;
				corrDiff.translation() *= scale(0);
				corrDiff = corr * corrDiff;
				corrDiff = calibsSrc[c].transform * corrDiff.inverse();
				Scalar tError = corrDiff.translation().norm(), rError = Eigen::AngleAxis<Scalar>(corrDiff.rotation()).angle()/PI*180;
				if (cc == c)
					LOG(LPointCalib, LTrace, "    Correction by same camera yields errors %f, %f", tError*200, rError);
				if (cc == c) continue;
				LOG(LPointCalib, LTrace, "    Correction yields pair error %f, %f", tError*200, rError);
				if (tError > 0.01f || rError > 2) continue; // Doesn't agree
				agreeing++;
				errorSum += tError * 200 + rError;
			}
			if (agreeing == 0) continue;

			Correction correction;
			correction.agreeingCamera = agreeing+1;
			correction.weight = agreeing / (1.0+errorSum);
			correction.transform = corr;
			correction.scale = scale(0);
			guesses.push_back(correction);
			guessesScale.push_back(scale(0));
			unchangedCameras = std::max(agreeing+1, unchangedCameras);
			LOG(LPointCalib, LTrace, "Got correction with %d cameras and weight %f for scale %f", agreeing+1, correction.weight, scale(0));
		}
	}
	if (unchangedCameras < 2) return false;
	LOG(LPointCalib, LTrace, "Found %d corrective transformation guesses, consolidating them:", (int)guesses.size());

	// Either do another dbscan on guessesScale
	// Or just go with best guess and consolidate with agreeing guesses:
	std::sort(guesses.begin(), guesses.end(), [](auto &a, auto &b){ return a.weight > b.weight; });

	Isometry3<Scalar> accumIsometry, isometry;
	accumIsometry.matrix().setZero();
	Scalar accumScale = 0;
	int contribCameras = 0;
	for (int i = 0; i < guesses.size(); i++)
	{
		if (guesses[i].agreeingCamera < unchangedCameras)
			continue;
		if (contribCameras > 0)
		{ // Verify it agrees
			Isometry3<Scalar> diff = guesses[i].transform * isometry.inverse();
			Scalar tError = diff.translation().norm(), rError = Eigen::AngleAxis<Scalar>(diff.rotation()).angle()/PI*180;
			Scalar sError = guessesScale[i]/(accumScale/contribCameras);
			LOG(LPointCalib, LTrace, "    Correction agrees with pair error %f, %f, %f%%", tError*200, rError, (sError-1)*100);
			if (tError > 0.01f || rError > 2 || std::abs(sError-1) > 0.01) continue; // Doesn't agree
		}
		accumIsometry.matrix() += guesses[i].transform.matrix();
		accumScale += guessesScale[i];
		contribCameras++;
		isometry.matrix() = accumIsometry.matrix() / contribCameras;
	}
	roomTransform = isometry * Eigen::UniformScaling<Scalar>(accumScale/contribCameras);
	roomOrientation = isometry.linear();
	return true;
}
template bool transferRoomCalibration(const std::vector<CameraCalib_t<double>> &calibsSrc, const std::vector<CameraCalib_t<double>> &calibsTgt,
	Matrix3<double> &roomOrientation, Affine3<double> &roomTransform);