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

#include "pipeline.hpp"

#include "ui/shared.hpp" // Signals

#include "calib_point/reconstruction.hpp"
#include "calib_point/calibration_room.hpp"
#include "calib/obs_data.inl"

#include "util/log.hpp"

#include "scope_guard/scope_guard.hpp"

#include <numeric>

static void ThreadCalibrationReconstruction(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras);
static void ThreadCalibrationOptimisation(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras);


// ----------------------------------------------------------------------------
// Calibration
// ----------------------------------------------------------------------------

void InitPointCalibration(PipelineState &pipeline)
{
	auto &ptCalib = pipeline.pointCalib;
	// Wait for all calibration threads to stop if they are still running
	ptCalib.control.stop();
	// Init/Clean the rest
	ptCalib.planned = false;
	ptCalib.settings = {};
	ptCalib.state = {};
	ptCalib.floorPoints.clear();
}

template<typename Scalar>
static void NormaliseCalibration(std::vector<CameraCalib_t<Scalar>> &calibs, Scalar height = 4.5, Scalar pairwiseDist = 4.0)
{
	Eigen::Matrix<Scalar,3,1> origin = Eigen::Matrix<Scalar,3,1>::Zero();
	for (auto &calib : calibs)
		origin += calib.transform.translation();
	origin /= calibs.size();

	// Perform Principle Component Analysis to get the least represented axis, which is our axis going vertically through the plane of the points
	Eigen::Matrix<Scalar,3,Eigen::Dynamic> N = Eigen::Matrix<Scalar,3,Eigen::Dynamic>(3, calibs.size());
	for (int c = 0; c < calibs.size(); c++)
		N.col(c) = calibs[c].transform.translation()-origin;

	// TODO: Choose either SVD or EVD
	Eigen::JacobiSVD<Eigen::Matrix<Scalar,3,Eigen::Dynamic>, Eigen::ComputeFullU> svd_N(N);
	Eigen::Matrix<Scalar,3,1> axis = svd_N.matrixU().template block<3,1>(0, 2);
	//Eigen::Matrix<Scalar,3,1> rankValues = svd_N.singularValues().template tail<3>().reverse();
	/* Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar,3,Eigen::Dynamic>> evd_N(N*N.transpose(), Eigen::ComputeEigenvectors);
	Eigen::Matrix<Scalar,3,1> axis = evd_N.eigenvectors().template block<3,1>(0, 0);
	Eigen::Matrix<Scalar,3,1> rankValues = evd_N.eigenvalues().template head<3>(); */

	// Flip room up axis based on camera orientations
	Scalar alignment = 0;
	for (int c = 0; c < calibs.size(); c++)
	{
		Eigen::Vector3<Scalar> forward = calibs[c].transform.rotation() * Eigen::Matrix<Scalar,3,1>::UnitZ();
		alignment += axis.dot(forward);
	}
	if (alignment > 0)
		axis = -axis;

	// Get transform to calibrated room from up axis, origin and scale
	Eigen::Matrix<Scalar,3,3> rotation = Eigen::Quaternion<Scalar>::FromTwoVectors(axis, Eigen::Matrix<Scalar,3,1>::UnitZ()).toRotationMatrix();
	Scalar scaling = pairwiseDist / N.colwise().norm().mean();
	Eigen::Transform<Scalar,3,Eigen::Affine> transform;
	transform.linear() = rotation * Eigen::DiagonalMatrix<Scalar,3>(scaling, scaling, scaling);
	transform.translation() = -transform.linear()*origin + Eigen::Matrix<Scalar,3,1>(0, 0, height);

	// Apply normalised transform
	for (auto &calib : calibs)
	{
		calib.transform.linear() = rotation * calib.transform.linear();
		calib.transform.translation() = transform * calib.transform.translation();
		calib.UpdateDerived();
	}
}

void UpdatePointCalibration(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras, std::shared_ptr<FrameRecord> &frame)
{
	auto &ptCalib = pipeline.pointCalib;

	if (ptCalib.control.running())
	{
		if (ptCalib.control.finished)
		{
			ptCalib.control.stop();
			SignalPipelineUpdate();
		}
	}
	else if (ptCalib.planned)
	{
		ptCalib.planned = false;
		ptCalib.control.init();
		if (ptCalib.settings.typeFlags & 0b01)
			ptCalib.control.thread = new std::thread(ThreadCalibrationReconstruction, ptCalib.control.stop_source.get_token(), &pipeline, cameras);
		else if (ptCalib.settings.typeFlags & 0b10)
			ptCalib.control.thread = new std::thread(ThreadCalibrationOptimisation, ptCalib.control.stop_source.get_token(), &pipeline, cameras);
		SignalPipelineUpdate();
	}

	if (ptCalib.normaliseRoom)
	{
		ptCalib.normaliseRoom = false;
		std::vector<CameraCalib> calibs(pipeline.cameras.size());
		for (int c = 0; c < pipeline.cameras.size(); c++)
			calibs[c] = cameras[c]->calib;
		NormaliseCalibration(calibs);
		for (int c = 0; c < pipeline.cameras.size(); c++)
		{
			if (pipeline.cameras[c]->calib.invalid()) continue;
			pipeline.cameras[c]->calib = calibs[c];
		}
		SignalServerEvent(EVT_UPDATE_CALIBS);
		SignalPipelineUpdate();
	}

	// TODO: Room calibration should be considered largely untested since it hasn't been used in a while
	// Need to improve experience / instructions, visualise points on the floor, account for physical marker sizes, etc.
	if (ptCalib.calibrateFloor)
	{
		ptCalib.calibrateFloor = false;
		LOG(LPointCalib, LDebug, "Attempting to calibrate the floor!\n");
		Eigen::Matrix3d roomOrientation;
		Eigen::Affine3d roomTransform;
		int pointCount = calibrateFloor<double>(ptCalib.floorPoints, ptCalib.distance12, roomOrientation, roomTransform);
		if (pointCount > 0)
		{
			LOG(LPointCalib, LDebug, "Calibrated floor with %d points!\n", pointCount);
			for (auto &cam : pipeline.cameras)
			{
				if (cam->calib.invalid()) continue;
				cam->calib.transform.linear() = roomOrientation * cam->calib.transform.linear();
				cam->calib.transform.translation() = roomTransform * cam->calib.transform.translation();
				cam->calib.UpdateDerived();
			}
			SignalServerEvent(EVT_UPDATE_CALIBS);
			SignalPipelineUpdate();
		}
		else
		{
			LOG(LPointCalib, LDebug, "Failed to calibrate the floor!\n");
		}
	}
	else
	{
		if (!ptCalib.floorPoints.empty() && pipeline.phase == PHASE_Calibration_Point)
		{ // Triangulate unoccupied 2D points, since tracking stage is not running
			auto &track = pipeline.tracking;
			const auto &detect = pipeline.params.detect;

			// Aggregate points and camera calibrations
			std::vector<CameraCalib> calibs(cameras.size());
			std::vector<std::vector<Eigen::Vector2f> const *> points2D(cameras.size());
			std::vector<std::vector<int> const *> relevantPoints2D(calibs.size());
			std::vector<std::vector<int>> remainingPoints2D(calibs.size());
			for (int c = 0; c < cameras.size(); c++)
			{
				calibs[c]= cameras[c]->calib;
				points2D[c] = &frame->cameras[calibs[c].index].points2D;
				remainingPoints2D[c].resize(points2D[c]->size());
				std::iota(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), 0);
				relevantPoints2D[c] = &remainingPoints2D[c];
			}

			// Clear past frames' triangulations
			track.triangulations3D.clear();
			track.discarded3D.clear();
			track.points3D.clear();

			auto &params = pipeline.params.tri;

			// Find potential point correspondences as TriangulatedPoints
			triangulateRayIntersections(calibs, points2D, relevantPoints2D, track.triangulations3D,
				params.maxIntersectError, params.minIntersectError);

			// Resolve conflicts by assigning points based on confidences and reevaluating confidences
			// Note this uses internal data from triangulateRayIntersections to help resolve
			resolveTriangulationConflicts(track.triangulations3D, params.maxIntersectError);

			// Remove the least confident points
			filterTriangulatedPoints(track.triangulations3D, track.discarded3D,
				params.minIntersectionConfidence);
			LOG(LTracking, LDebug, "%d triangulated points detected", (int)track.triangulations3D.size());

			// Refine point positions
			track.points3D.reserve(track.triangulations3D.size());
			for (int p = 0; p < track.triangulations3D.size(); p++)
			{
				refineTriangulationIterative<float>(points2D, calibs, track.triangulations3D[p], params.refineIterations);
				track.points3D.push_back(track.triangulations3D[p].pos);
			}
		}

		for (int i = 0; i < ptCalib.floorPoints.size(); i++)
		{
			auto &point = ptCalib.floorPoints[i];
			if (!point.sampling) continue;
			if (pipeline.tracking.points3D.empty())
			{
				LOG(LPointCalib, LDebug, "No markers visible to calibrate floor point!\n");
				ptCalib.floorPoints.erase(ptCalib.floorPoints.begin()+i);
				i--;
				continue;
			}
			if (point.sampleCount > 0)
			{
				Eigen::Vector3d curPos = point.pos / point.sampleCount;
				if ((curPos.cast<float>() - pipeline.tracking.points3D[0]).norm() < 0.005)
				{
					point.pos += pipeline.tracking.points3D[0].cast<double>();
					point.sampleCount++;
				}
			}
			else
			{
				point.pos = pipeline.tracking.points3D[0].cast<double>();
				point.sampleCount++;
				LOG(LPointCalib, LDebug, "== Started calibrating point %d!\n", i);
			}
			if (point.sampleCount > 100)
			{
				point.pos = point.pos / point.sampleCount;
				LOG(LPointCalib, LDebug, "== Calibrated point %d with %d samples!\n", i, point.sampleCount);
				point.sampling = false;
				point.confidence = 10;
			}
		}
	}
}

static void ThreadCalibrationReconstruction(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras)
{
	const auto exitNotifier = sg::make_scope_guard([&]() noexcept { pipeline->pointCalib.control.finished = true; });

	ScopedLogCategory scopedLogCategory(LPointCalib, true);
	ScopedLogLevel scopedLogLevel(LDebug);

	LOGC(LInfo, "=======================\n");

	// Copy data
	auto &ptCalib = pipeline->pointCalib;
	PointCalibParameters params = ptCalib.params;
	SequenceData observations = *pipeline->seqDatabase.contextualRLock();
	std::vector<CameraMode> modes(observations.temporary.size());
	std::vector<CameraCalib> calibs(observations.temporary.size());
	for (int c = 0; c < cameras.size(); c++)
	{
		int index = cameras[c]->index;
		modes[index] = cameras[c]->mode;
		calibs[index] = cameras[c]->calib;
		calibs[index].id = cameras[c]->id;
		calibs[index].index = cameras[c]->index;
	}

	ObsData pointData = {};
	addTriangulatableObservations(pointData.points, observations.markers);

	auto rec1 = pclock::now();

	LOGC(LInfo, "== Reconstructing Geometry:\n");
	bool success = reconstructGeometry(pointData.points, calibs, params.reconstruction);
	if (success) LOGC(LInfo, "== Finished Reconstructing Calibration!\n")
	else LOGC(LInfo, "== Failed to properly reconstruct geometry!\n")

	auto rec2 = pclock::now();

	{
		ScopedLogLevel scopedLogLevel(LInfo);
		LOGCL("== Reconstructed Cameras:\n");
		DebugSpecificCameraParameters(calibs, modes);
	}

	if (pipeline->isSimulationMode)
	{
		LOGC(LInfo, "== Adjusting calibration to ground truth simulation setup:\n");
		auto errors = AlignWithGT(*pipeline, calibs);
		if (errors.first < 50 && errors.second < 10)
			LOGC(LInfo, "    Adjusted room calibration to Ground Truth with error of %.3fmm and %.3fdg\n", errors.first, errors.second)
		else
			LOGC(LWarn, "    Room calibration is different from Ground Truth simulation setup! Had error of %.3fmm and %.3fdg\n", errors.first, errors.second);
	}
	else
	{ // Normalise calibration so that all cameras are close to a plane, looking down, 2m above the ground
		NormaliseCalibration(calibs);
	}

	if (stopToken.stop_requested())
	{
		LOGC(LInfo, "== Aborted Reconstructing Calibration!\n");
		LOGC(LInfo, "=======================\n");
		return;
	}

	std::shared_lock shared_lock(pipeline->pipelineLock);

	{ // Copy point data into observation database (for future continuous calibration)
		auto db_lock = pipeline->obsDatabase.contextualLock();
		db_lock->points = std::move(pointData.points);

		ScopedLogLevel scopedLogLevel(LInfo);

		LOGCL("== Updating Error Maps:\n");
		updateErrorMaps(*pipeline, *db_lock, calibs);

		LOGCL("== Determining Reprojection Errors:\n");
		ptCalib.state.errors = updateReprojectionErrors(*db_lock, calibs);
	}

	{ // Update calibration
		std::unique_lock pipeline_lock(pipeline->pipelineLock);
		for (int c = 0; c < calibs.size(); c++)
			pipeline->cameras[calibs[c].index]->calib = calibs[c];
	}

	// Update fundamental matrices using calibration
	UpdateCalibrationRelations(*pipeline, *pipeline->calibration.contextualLock(), observations);

	SignalServerEvent(EVT_UPDATE_CALIBS);
	SignalPipelineUpdate();

	LOGC(LInfo, "== Done Reconstructing Calibration!\n");

	LOGC(LDebug, "== Reconstruction took %.2fms in total!\n", dt(rec1, rec2));

	LOGC(LInfo, "=======================\n");
}

static void ThreadCalibrationOptimisation(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras)
{
	const auto exitNotifier = sg::make_scope_guard([&]() noexcept { pipeline->pointCalib.control.finished = true; });

	ScopedLogCategory scopedLogCategory(LPointCalib, true);
	ScopedLogLevel scopedLogLevel(LInfo);

	LOGC(LInfo, "=======================\n");

	// Copy data
	auto &ptCalib = pipeline->pointCalib;
	auto settings = ptCalib.settings;
	PointCalibParameters params = ptCalib.params;
	SequenceData observations = *pipeline->seqDatabase.contextualRLock();
	std::vector<CameraMode> modes(observations.temporary.size());
	std::vector<CameraCalib> calibs(observations.temporary.size());
	for (int c = 0; c < cameras.size(); c++)
	{
		int index = cameras[c]->index;
		modes[index] = cameras[c]->mode;
		calibs[index] = cameras[c]->calib;
		calibs[index].id = cameras[c]->id;
		calibs[index].index = cameras[c]->index;
	}

	// New database with selected points
	ObsData pointData = {};
	addTriangulatableObservations(pointData.points, observations.markers);

	/* {
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL( "== Updating Initial Reprojection Errors:\n");
		ptCalib.state.errors = updateReprojectionErrors(*db_lock, calibs);
	} */

	{
		ScopedLogLevel scopedLogLevel(LInfo);
		LOGCL("== Determining Outliers by Reprojection Error:\n");
		ptCalib.state.errors = determinePointOutliers(pointData, calibs, params.outliers);
	}

	{
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL("== Updating Error Maps:\n");
		updateErrorMaps(*pipeline, pointData, calibs);
	}

	/* {
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL( "== Updating Reprojection Errors:\n");
		ptCalib.state.errors = updateReprojectionErrors(pointData, calibs);
	} */

	// Perform optimisation step
	LOGC(LInfo, "== Optimising Calibration:\n");

	auto lastIt = pclock::now();

	auto itUpdate = [&](OptErrorRes errors)
	{
		ptCalib.state.errors = errors;
		ptCalib.state.numSteps++;

		float dtOpt = dt(lastIt, pclock::now());
		lastIt = pclock::now();

		bool hasNaN = std::isnan(errors.mean) || std::isnan(errors.stdDev) || std::isnan(errors.max);
		if (hasNaN)
			LOGC(LWarn, "Current errors has NaNs after optimisation step %d: (%f, %f, %f)\n",
				ptCalib.state.numSteps, errors.mean, errors.stdDev, errors.max);

		{ // Update calibration
			std::unique_lock pipeline_lock(pipeline->pipelineLock);
			for (int c = 0; c < calibs.size(); c++)
				pipeline->cameras[calibs[c].index]->calib = calibs[c];
		}

		SignalServerEvent(EVT_UPDATE_CALIBS);
		SignalPipelineUpdate();

		if (errors.max > errors.mean + params.outliers.sigma.trigger*errors.stdDev)
		{
			ScopedLogLevel scopedLogLevel(LDebug);
			LOGCL("-- Determining more outliers:");
			ptCalib.state.errors = determinePointOutliers(pointData, calibs, params.outliers);
		}
		LOGC(LDebug, "-- Finished optimisation step in %.2fms + %.2fms", dtOpt, dt(lastIt, pclock::now()));

		return !stopToken.stop_requested() && ptCalib.state.numSteps < settings.maxSteps && !hasNaN;
	};

	ptCalib.state.lastStopCode = optimiseCameras(settings.options, pointData, calibs, itUpdate);
	LOGC(LInfo, "%s", getStopCodeText(ptCalib.state.lastStopCode));

	ptCalib.state.complete = !stopToken.stop_requested() && ptCalib.state.numSteps < settings.maxSteps;

	{
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL("-- Determining Outliers by Reprojection Error:\n");
		ptCalib.state.errors = determinePointOutliers(pointData, calibs, params.outliers);
	}

	{
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL( "-- Updating Error Maps:\n");
		updateErrorMaps(*pipeline, pointData, calibs);
	}

	{
		ScopedLogLevel scopedLogLevel(LInfo);
		LOGCL("-- Camera parameters:\n");
		DebugSpecificCameraParameters(calibs, modes);
	}

	{
		ScopedLogLevel scopedLogLevel(LInfo);
		LOGCL("== Finished Optimisations:\n");
		ptCalib.state.errors = updateReprojectionErrors(pointData, calibs);
	}

	if (pipeline->isSimulationMode)
	{
		LOGC(LInfo, "== Adjusting calibration to ground truth simulation setup:\n");
		auto errors = AlignWithGT(*pipeline, calibs);
		if (errors.first < 50 && errors.second < 10)
			LOGC(LInfo, "    Adjusted room calibration to Ground Truth with error of %.3fmm and %.3fdg\n", errors.first, errors.second)
		else
			LOGC(LWarn, "    Room calibration is different from Ground Truth simulation setup! Had error of %.3fmm and %.3fdg\n", errors.first, errors.second);
	}

	if (!stopToken.stop_requested())
	{
		LOGC(LInfo, "== Done Optimising Calibration!\n");
	}
	else // Since we've already overwrote calibration while calibrating anyway
		LOGC(LInfo, "== Aborted Optimising Calibration! Accepting results nonetheless!\n");

	{ // Replace shared point data with new subset used for calibration
		auto db_lock = pipeline->obsDatabase.contextualLock();
		db_lock->points = std::move(pointData.points);
		// TODO: Clear up seqDatabase and obsDatabase relation for initial point calib and future cont calib
		// Keep newer points and update errors?
		// Only other source is continuous calibration in the future
		// Point calibration already only adds to obsDatabase in batches whenever a thread is instructed, reading from seqDatabase
		// Potentially get continuous calib to iteratively add data from seqDatabase to obsDatabase and then always use that as data source?
	}

	{ // Update calibration
		std::unique_lock pipeline_lock(pipeline->pipelineLock);
		for (int c = 0; c < calibs.size(); c++)
			pipeline->cameras[calibs[c].index]->calib = calibs[c];
	}

	// Update fundamental matrices using calibration
	UpdateCalibrationRelations(*pipeline, *pipeline->calibration.contextualLock(), observations);

	SignalServerEvent(EVT_UPDATE_CALIBS);
	SignalPipelineUpdate();

	LOGC(LDebug, "=======================\n");
}
