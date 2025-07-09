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
#include "util/eigenutil.hpp"

#include "scope_guard/scope_guard.hpp"

#include <numeric>

static void ThreadCalibrationReconstruction(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras);
static void ThreadCalibrationOptimisation(std::stop_token stopToken, PipelineState *pipeline, std::vector<CameraPipeline*> cameras);


// ----------------------------------------------------------------------------
// Calibration
// ----------------------------------------------------------------------------

static void ApplyTransformation(std::vector<CameraCalib> &calibs, Eigen::Matrix3d orientation, Eigen::Affine3d transform);

void ResetPointCalibration(PipelineState &pipeline)
{
	// Wait for all calibration threads to stop if they are still running
	pipeline.pointCalib.control.stop();
	// Init/Clean the rest
	pipeline.pointCalib.planned = false;
	pipeline.pointCalib.settings = {};
	pipeline.pointCalib.state = {};
	pipeline.pointCalib.room.floorPoints.clear();
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

	// TODO: Room calibration should be considered largely untested since it hasn't been used in a while
	// Need to improve experience / instructions, visualise points on the floor, account for physical marker sizes, etc.
	if (!ptCalib.room.floorPoints.empty() && ptCalib.room.floorPoints.back().sampling)
	{
		auto &point = ptCalib.room.floorPoints.back();
		if (pipeline.tracking.triangulations3D.empty())
		{
			LOG(LPointCalib, LDebug, "No markers visible to calibrate floor point!\n");
			ptCalib.room.floorPoints.pop_back();
		}
		else
		{
			TriangulatedPoint tri;
			if (point.sampleCount == 0)
			{
				LOG(LPointCalib, LDebug, "== Started calibrating point %d!\n", (int)ptCalib.room.floorPoints.size());
				tri = pipeline.tracking.triangulations3D.front();
			}
			else
			{
				float closest = 0.005f;
				for (auto &t : pipeline.tracking.triangulations3D)
				{
					float dist = (point.pos.cast<float>() - t.pos).norm();
					if (dist < closest)
					{
						tri = t;
						closest = dist;
					}
				}
			}
			if (!tri.blobs.empty())
			{
				point.samples.resize(pipeline.cameras.size(), { 0, Eigen::Vector2d::Zero() });
				for (int c = 0; c < tri.blobs.size(); c++)
				{
					if (tri.blobs[c] == InvalidBlob) continue;
					auto &sample = point.samples[c];
					sample.second += frame->cameras[c].points2D[tri.blobs[c]].cast<double>();
					sample.first++;
					point.sampleCount++;
				}
				point.update(pipeline.getCalibs());
			}
			if (point.sampleCount > 1000 || (point.sampleCount > 300 && point.startObservation-frame->num > 100))
			{
				LOG(LPointCalib, LDebug, "== Calibrated point %d with %d samples!\n", (int)ptCalib.room.floorPoints.size(), point.sampleCount);
				point.sampling = false;
				point.confidence = 10;
			}
		}
	}
	if (ptCalib.room.floorPoints.size() >= 2)
	{ // Keep these invariant to potential calibration changes
		// TODO: Consider updating these only when calibration changes occur - similar to UpdateCalibrationRelations
		auto calibs = pipeline.getCalibs();
		for (auto &floorPoint : pipeline.pointCalib.room.floorPoints)
			floorPoint.update(calibs);
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
	{
		Eigen::Matrix3d roomOrientation;
		Eigen::Affine3d roomTransform;
		getCalibNormalisation<double>(calibs, roomOrientation, roomTransform);
		ApplyTransformation(calibs, roomOrientation, roomTransform);
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
		UpdateErrorMaps(*pipeline, *db_lock, calibs);

		LOGCL("== Determining Reprojection Errors:\n");
		ptCalib.state.errors = updateReprojectionErrors(*db_lock, calibs);
	}

	{ // Update calibration
		std::unique_lock pipeline_lock(pipeline->pipelineLock);
		AdoptNewCalibrations(*pipeline, calibs, true);
	}
	SignalCameraCalibUpdate(calibs);

	UpdateCalibrationRelations(*pipeline, *pipeline->calibration.contextualLock(), observations);

	LOGC(LInfo, "== Done Reconstructing Calibration!\n");

	LOGC(LDebug, "== Reconstruction took %.2fms in total!\n", dtMS(rec1, rec2));

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
		UpdateErrorMaps(*pipeline, pointData, calibs);
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

		float dtOpt = dtMS(lastIt, pclock::now());
		lastIt = pclock::now();

		bool hasNaN = std::isnan(errors.mean) || std::isnan(errors.stdDev) || std::isnan(errors.max);
		if (hasNaN)
			LOGC(LWarn, "Current errors has NaNs after optimisation step %d: (%f, %f, %f)\n",
				ptCalib.state.numSteps, errors.mean, errors.stdDev, errors.max);

		{ // Update calibration
			std::unique_lock pipeline_lock(pipeline->pipelineLock);
			AdoptNewCalibrations(*pipeline, calibs, true);
		}
		SignalCameraCalibUpdate(calibs);

		if (errors.max > errors.mean + params.outliers.sigma.trigger*errors.stdDev)
		{
			ScopedLogLevel scopedLogLevel(LDebug);
			LOGCL("-- Determining more outliers:");
			ptCalib.state.errors = determinePointOutliers(pointData, calibs, params.outliers);
		}
		LOGC(LDebug, "-- Finished optimisation step in %.2fms + %.2fms", dtOpt, dtMS(lastIt, pclock::now()));

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
		UpdateErrorMaps(*pipeline, pointData, calibs);
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
		AdoptNewCalibrations(*pipeline, calibs, true);
	}
	SignalCameraCalibUpdate(calibs);

	UpdateCalibrationRelations(*pipeline, *pipeline->calibration.contextualLock(), observations);

	LOGC(LDebug, "=======================\n");
}

bool CalibrateRoom(PipelineState &pipeline)
{
	auto &ptCalib = pipeline.pointCalib;
	if (ptCalib.room.floorPoints.size() < 3)
	{
		LOG(LPointCalib, LDebug, "Could not calibrate floor!\n");
		return false;
	}

	LOG(LPointCalib, LDebug, "Attempting to calibrate the floor!\n");

	// Re-evaluate positions incase calibration changed since observation
	auto calibs = pipeline.getCalibs();
	for (auto &point : ptCalib.room.floorPoints)
		point.update(calibs);

	// Estimate a transform to align floor plane to points with correct scale
	Eigen::Matrix3d roomOrientation;
	Eigen::Affine3d roomTransform;
	int pointCount = estimateFloorTransform<double>(ptCalib.room.floorPoints, ptCalib.room.distance12, roomOrientation, roomTransform);
	if (pointCount > 0)
	{
		LOG(LPointCalib, LInfo, "Calibrated floor with %d points!\n", pointCount);
		LOG(LPointCalib, LInfo, "Scaled calibration by %f during floor calibration!",
			roomTransform.linear().colwise().norm().mean());
		ApplyTransformation(calibs, roomOrientation, roomTransform);
		AdoptNewCalibrations(pipeline, calibs, false);
		SignalCameraCalibUpdate(calibs);
		return true;
	}
	else
	{
		LOG(LPointCalib, LDebug, "Failed to calibrate the floor!\n");
		return false;
	}
}

void AdoptNewCalibrations(PipelineState &pipeline, std::vector<CameraCalib> &calibs, bool copyRoomCalib)
{
	if (copyRoomCalib)
	{
		std::vector<CameraCalib> oldCalibs(calibs.size());
		for (int c = 0; c < calibs.size(); c++)
			oldCalibs[c] = pipeline.cameras[calibs[c].index]->calib;
		Eigen::Matrix3d roomOrientation;
		Eigen::Affine3d roomTransform;
		if (transferRoomCalibration(oldCalibs, calibs, roomOrientation, roomTransform))
			ApplyTransformation(calibs, roomOrientation, roomTransform);
	}

	for (auto &cam : pipeline.cameras)
	{
		cam->calibBackup = cam->calib; // Create backup
		cam->calib.id = CAMERA_ID_NONE; // Invalidate existing
		for (auto &calib : calibs)
		{ // Try to find valid new calibration
			if (calib.invalid()) continue;
			if (calib.id == cam->id)
			{ // Adopt
				cam->calib = calib;
				break;
			}
		}
		// Make sure index is correct
		cam->calib.index = cam->index;
	}
}

static void ApplyTransformation(std::vector<CameraCalib> &calibs, Eigen::Matrix3d orientation, Eigen::Affine3d transform)
{
	for (auto &calib : calibs)
	{
		if (calib.invalid()) continue;
		calib.transform.linear() = orientation * calib.transform.linear();
		calib.transform.translation() = transform * calib.transform.translation();
		calib.UpdateDerived();
	}
}