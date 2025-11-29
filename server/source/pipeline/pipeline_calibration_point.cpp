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

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <numeric>

static void ThreadCalibrationReconstruction(PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<ThreadControl> control, std::shared_ptr<PipelineState::PointCalibState> state);
static void ThreadCalibrationOptimisation(PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<ThreadControl> control, std::shared_ptr<PipelineState::PointCalibState> state);
static void ThreadCalibrationRoom(PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<ThreadControl> control, std::shared_ptr<PipelineState::PointCalibState> state);


// ----------------------------------------------------------------------------
// Calibration
// ----------------------------------------------------------------------------

static void ApplyTransformation(std::vector<CameraCalib> &calibs, Eigen::Matrix3d orientation, Eigen::Affine3d transform);

void ResetPointCalibration(PipelineState &pipeline)
{
	// Wait for all calibration threads to stop if they are still running
	if (pipeline.pointCalib.control)
		pipeline.pointCalib.control->stop_source.request_stop();
	// Asynchronously wait for calibration threads
	threadPool.push([](int, std::shared_ptr<ThreadControl> &control)
	{ /* Destructor */ }, std::move(pipeline.pointCalib.control));

	// Init/Clean the rest
	pipeline.pointCalib.planned = false;
	pipeline.pointCalib.settings = {};
	pipeline.pointCalib.state = std::make_shared<PipelineState::PointCalibState>();
	pipeline.pointCalib.room.contextualLock()->floorPoints.clear();
}

void UpdatePointCalibrationStatus(PipelineState &pipeline)
{
	auto &ptCalib = pipeline.pointCalib;

	std::vector<CameraPipeline*> cameras;
	cameras.reserve(pipeline.cameras.size());
	for (auto &cam : pipeline.cameras)
		cameras.push_back(cam.get());

	if (ptCalib.control)
	{
		if (!ptCalib.control->running())
			ptCalib.control = nullptr;
		else if (ptCalib.control->finished)
		{
			ptCalib.control->stop();
			ptCalib.control = nullptr;
			SignalPipelineUpdate();
		}
	}
	else if (ptCalib.planned)
	{
		ptCalib.planned = false;
		ptCalib.control = std::make_shared<ThreadControl>();
		ptCalib.control->init();
		if (ptCalib.settings.typeFlags & 0b001)
			ptCalib.control->thread = new std::thread(ThreadCalibrationReconstruction, &pipeline, cameras, ptCalib.control, ptCalib.state);
		else if (ptCalib.settings.typeFlags & 0b010)
			ptCalib.control->thread = new std::thread(ThreadCalibrationOptimisation, &pipeline, cameras, ptCalib.control, ptCalib.state);
		else if (ptCalib.settings.typeFlags & 0b100)
			ptCalib.control->thread = new std::thread(ThreadCalibrationRoom, &pipeline, cameras, ptCalib.control, ptCalib.state);
		SignalPipelineUpdate();
	}
}

void UpdatePointCalibration(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras, std::shared_ptr<FrameRecord> &frame)
{
	auto &ptCalib = pipeline.pointCalib;

	UpdatePointCalibrationStatus(pipeline);

	// TODO: Improve UX of room calibration
	// Allow selecting which point to use if there's multiple (currently needs one visible only)
	// Account for physical marker sizes, etc.
	auto roomCalib = ptCalib.room.contextualLock();
	if (!roomCalib->floorPoints.empty() && roomCalib->floorPoints.back().sampling)
	{
		auto &point = roomCalib->floorPoints.back();
		if (pipeline.tracking.triangulations3D.empty())
		{
			LOG(LPointCalib, LDebug, "No markers visible to calibrate floor point!\n");
			if (point.sampleCount == 0)
				SignalErrorToUser("No markers visible, please put a marker on the floor!");
			else
				SignalErrorToUser("Visibility of marker got interrupted, please try again!");
			roomCalib->floorPoints.pop_back();
		}
		else
		{
			TriangulatedPoint tri;
			if (point.sampleCount == 0)
			{
				// Using the best available triangulation (tris are sorted) is actually a fine metric
				/* if (pipeline.tracking.triangulations3D.size() > 1)
				{
					LOG(LPointCalib, LWarn, "== There are multiple markers visible, cannot start calibrating floor point %d!\n", (int)roomCalib->floorPoints.size());
					SignalErrorToUser("There are multiple markers visible, please make sure only the marker on the floor is visible.");
					roomCalib->floorPoints.pop_back();
				}
				else */
				{
					LOG(LPointCalib, LDebug, "== Started calibrating point %d!\n", (int)roomCalib->floorPoints.size());
					tri = pipeline.tracking.triangulations3D.front();
				}
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
				LOG(LPointCalib, LDebug, "== Calibrated point %d with %d samples!\n", (int)roomCalib->floorPoints.size(), point.sampleCount);
				point.sampling = false;
				point.confidence = 10;
			}
		}
	}
	if (roomCalib->floorPoints.size() >= 2)
	{ // Keep these invariant to potential calibration changes
		// TODO: Consider updating these only when calibration changes occur - similar to UpdateCalibrationRelations
		auto calibs = pipeline.getCalibs();
		for (auto &floorPoint : roomCalib->floorPoints)
			floorPoint.update(calibs);
	}
}

static void ThreadCalibrationReconstruction(PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<ThreadControl> control, std::shared_ptr<PipelineState::PointCalibState> state)
{
	std::stop_token stopToken = control->stop_source.get_token();
	const auto exitNotifier = sg::make_scope_guard([&]() noexcept { control->finished = true; });

	ScopedLogCategory scopedLogCategory(LPointCalib, true);
	ScopedLogLevel scopedLogLevel(LDebug);

	LOGC(LInfo, "=======================\n");

	// Copy data
	PointCalibParameters params = pipeline->pointCalib.params;
	SequenceData observations = *pipeline->seqDatabase.contextualRLock();
	std::vector<CameraMode> modes(pipeline->cameras.size());
	std::vector<CameraCalib> calibs(pipeline->cameras.size());
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
	auto error = reconstructGeometry(pointData.points, calibs, params.reconstruction);
	if (error)
	{
		LOGC(LInfo, "== Failed to properly reconstruct geometry!\n");
		SignalErrorToUser(error.value());
	}
	else LOGC(LInfo, "== Finished Reconstructing Calibration!\n");
	
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
			LOGC(LInfo, "    Adjusted room calibration to Ground Truth with error of %.3fmm and %.3f°\n", errors.first, errors.second);
		else
			LOGC(LWarn, "    Room calibration is different from Ground Truth simulation setup! Had error of %.3fmm and %.3f°\n", errors.first, errors.second);
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

	{ // Copy point data into observation database (for future continuous calibration)
		auto db_lock = pipeline->obsDatabase.contextualLock();
		db_lock->points = std::move(pointData.points);

		ScopedLogLevel scopedLogLevel(LInfo);

		LOGCL("== Updating Error Maps:\n");
		UpdateErrorMaps(*pipeline, *db_lock, calibs);

		LOGCL("== Determining Reprojection Errors:\n");
		state->errors = updateReprojectionErrors(*db_lock, calibs);
	}

	// Update calibration
	AdoptNewCalibrations(*pipeline, calibs);
	UpdateCalibrationRelations(*pipeline, *pipeline->calibration.contextualLock(), observations);
	SignalCameraCalibUpdate(calibs);

	LOGC(LInfo, "== Done Reconstructing Calibration!\n");

	LOGC(LDebug, "== Reconstruction took %.2fms in total!\n", dtMS(rec1, rec2));

	LOGC(LInfo, "=======================\n");
}

static void ThreadCalibrationOptimisation(PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<ThreadControl> control, std::shared_ptr<PipelineState::PointCalibState> state)
{
	std::stop_token stopToken = control->stop_source.get_token();
	const auto exitNotifier = sg::make_scope_guard([&]() noexcept { control->finished = true; });

	ScopedLogCategory scopedLogCategory(LPointCalib, true);
	ScopedLogLevel scopedLogLevel(LInfo);

	LOGC(LInfo, "=======================\n");

	// Copy data
	auto settings = pipeline->pointCalib.settings;
	PointCalibParameters params = pipeline->pointCalib.params;
	SequenceData observations = *pipeline->seqDatabase.contextualRLock();
	std::vector<CameraMode> modes(pipeline->cameras.size());
	std::vector<CameraCalib> calibs(pipeline->cameras.size());
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
		state->errors = updateReprojectionErrors(*db_lock, calibs);
	} */

	{
		ScopedLogLevel scopedLogLevel(LInfo);
		LOGCL("== Determining Outliers by Reprojection Error:\n");
		state->errors = determinePointOutliers(pointData, calibs, params.outliers);
	}

	{
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL("== Updating Error Maps:\n");
		UpdateErrorMaps(*pipeline, pointData, calibs);
	}

	/* {
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL( "== Updating Reprojection Errors:\n");
		state->errors = updateReprojectionErrors(pointData, calibs);
	} */

	// Perform optimisation step
	LOGC(LInfo, "== Optimising Calibration:\n");

	auto lastIt = pclock::now();

	auto itUpdate = [&](OptErrorRes errors)
	{
		state->errors = errors;
		state->numSteps++;

		float dtOpt = dtMS(lastIt, pclock::now());
		lastIt = pclock::now();

		bool hasNaN = std::isnan(errors.mean) || std::isnan(errors.stdDev) || std::isnan(errors.max);
		if (hasNaN)
			LOGC(LWarn, "Current errors has NaNs after optimisation step %d: (%f, %f, %f)\n",
				state->numSteps, errors.mean, errors.stdDev, errors.max);

		// Update calibration
		AdoptNewCalibrations(*pipeline, calibs);
		SignalCameraCalibUpdate(calibs);

		if (errors.max > errors.mean + params.outliers.sigma.trigger*errors.stdDev)
		{
			ScopedLogLevel scopedLogLevel(LDebug);
			LOGCL("-- Determining more outliers:");
			state->errors = determinePointOutliers(pointData, calibs, params.outliers);
		}
		LOGC(LDebug, "-- Finished optimisation step in %.2fms + %.2fms", dtOpt, dtMS(lastIt, pclock::now()));

		return !stopToken.stop_requested() && state->numSteps < settings.maxSteps && !hasNaN;
	};

	if (!stopToken.stop_requested() && state->numSteps < settings.maxSteps)
	{
		state->lastStopCode = optimiseCameras(settings.options, pointData, calibs, itUpdate);
		LOGC(LInfo, "%s", getStopCodeText(state->lastStopCode));
	}

	state->complete = !stopToken.stop_requested() && state->numSteps < settings.maxSteps;

	{
		ScopedLogLevel scopedLogLevel(LDebug);
		LOGCL("-- Determining Outliers by Reprojection Error:\n");
		state->errors = determinePointOutliers(pointData, calibs, params.outliers);
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
		state->errors = updateReprojectionErrors(pointData, calibs);
	}

	if (pipeline->isSimulationMode)
	{
		LOGC(LInfo, "== Adjusting calibration to ground truth simulation setup:\n");
		auto errors = AlignWithGT(*pipeline, calibs);
		if (errors.first < 50 && errors.second < 10)
			LOGC(LInfo, "    Adjusted room calibration to Ground Truth with error of %.3fmm and %.3f°\n", errors.first, errors.second);
		else
			LOGC(LWarn, "    Room calibration is different from Ground Truth simulation setup! Had error of %.3fmm and %.3f°\n", errors.first, errors.second);
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
	}

	// Update calibration
	AdoptNewCalibrations(*pipeline, calibs);
	UpdateCalibrationRelations(*pipeline, *pipeline->calibration.contextualLock(), observations);
	SignalCameraCalibUpdate(calibs);

	LOGC(LDebug, "=======================\n");
}

static void ThreadCalibrationRoom(PipelineState *pipeline, std::vector<CameraPipeline*> cameras, std::shared_ptr<ThreadControl> control, std::shared_ptr<PipelineState::PointCalibState> state)
{
	std::stop_token stopToken = control->stop_source.get_token();
	const auto exitNotifier = sg::make_scope_guard([&]() noexcept { control->finished = true; });

	ScopedLogCategory scopedLogCategory(LPointCalib, true);
	ScopedLogLevel scopedLogLevel(LInfo);

	LOGC(LInfo, "=======================\n");

	auto roomCalib = pipeline->pointCalib.room.contextualLock();
	std::vector<CameraCalib> calibs(pipeline->cameras.size());
	for (int c = 0; c < cameras.size(); c++)
	{
		int index = cameras[c]->index;
		calibs[index] = cameras[c]->calib;
	}

	LOG(LPointCalib, LDebug, "Attempting to calibrate the floor!\n");

	// Re-evaluate positions in case calibration changed since observation
	for (auto &point : roomCalib->floorPoints)
		point.update(calibs);

	// Estimate a transform to align floor plane to points with correct scale
	Eigen::Matrix3d roomOrientation;
	Eigen::Affine3d roomTransform;
	auto error = estimateFloorTransform<double>(calibs, roomCalib->floorPoints, roomCalib->distance12, roomOrientation, roomTransform);
	if (error)
	{
		LOG(LPointCalib, LError, "Failed to calibrate the floor: %s", error->c_str());
		SignalErrorToUser(error.value());
	}
	else
	{
		LOG(LPointCalib, LInfo, "Successfully calibrated floor with %d points!\n", (int)roomCalib->floorPoints.size());
		LOG(LPointCalib, LInfo, "Scaled calibration by %f during floor calibration!",
			roomTransform.linear().colwise().norm().mean());

		// Update calibration
		ApplyTransformation(calibs, roomOrientation, roomTransform);
		AdoptNewCalibrations(*pipeline, calibs, true);
		SignalCameraCalibUpdate(calibs);
	}

	LOGC(LDebug, "=======================\n");
}

void AdoptNewCalibrations(PipelineState &pipeline, std::vector<CameraCalib> &calibs, bool isLoadOrRoomCalib)
{
	if (!isLoadOrRoomCalib)
	{ // Loaded calibs and new room calibration do not need old room calibration transferred
		ScopedLogLevel scopedLogLevel(LDebug);

		std::vector<CameraCalib> roomCalib(calibs.size());
		for (int c = 0; c < calibs.size(); c++)
		{
			assert(pipeline.cameras[calibs[c].index]->id == calibs[c].id);
			roomCalib[c] = pipeline.cameras[calibs[c].index]->calibRoom;
		}

		Eigen::Matrix3d roomOrientation;
		Eigen::Affine3d roomTransform;
		std::map<int,float> usedCmeras;
		bool strict = true;
		auto error = transferRoomCalibration(roomCalib, calibs, roomOrientation, roomTransform, usedCmeras, strict);
		if (error && error->code != -2)
		{
			LOG(LPointCalib, LError, "Failed to transfer room calibration (strict): %s", error->c_str());
			strict = false;
			error = transferRoomCalibration(roomCalib, calibs, roomOrientation, roomTransform, usedCmeras, strict);
			if (error)
				LOG(LPointCalib, LError, "Failed to transfer room calibration (lax): %s", error->c_str());
		}
		if (error)
		{
			pipeline.pointCalib.roomState.lastTransferSuccess = 0;
			pipeline.pointCalib.roomState.lastTransferError = error;
			pipeline.pointCalib.roomState.unchangedCameras.clear();
		}
		else
		{
			pipeline.pointCalib.roomState.lastTransferSuccess = strict? 2 : 1;
			pipeline.pointCalib.roomState.lastTransferError = std::nullopt;
			pipeline.pointCalib.roomState.unchangedCameras = std::move(usedCmeras);
			ApplyTransformation(calibs, roomOrientation, roomTransform);
		}
	}
	else
	{
		pipeline.pointCalib.roomState.lastTransferSuccess = -1;
		pipeline.pointCalib.roomState.lastTransferError = std::nullopt;
		pipeline.pointCalib.roomState.unchangedCameras.clear();
	}

	std::unique_lock pipeline_lock(pipeline.pipelineLock);
	bool lensesChanged = false;
	for (auto &cam : pipeline.cameras)
	{
		cam->calibBackup = cam->calib; // Create backup
		for (auto &calib : calibs)
		{ // Try to find valid new calibration
			if (calib.invalid()) continue;
			if (calib.id == cam->id)
			{ // Adopt
				if (calib.lensID != cam->calib.lensID)
					lensesChanged = true;
				calib.lensID = cam->calib.lensID;
				cam->calib = calib;
				break;
			}
		}
		// Make sure index is correct
		cam->calib.index = cam->index;
		if (isLoadOrRoomCalib)
			cam->calibRoom = cam->calib;
	}
	if (lensesChanged && !isLoadOrRoomCalib)
	{ // TODO: User edited lenses while calibration process was ongoing, may result in unintended behaviour
		// We CAN adopt the ID for the next calibration, but we may have overwritten the distortion parameters loaded from a preset
		// Least we can do is warn user
		SignalErrorToUser("Lens assignments have been edited while calibration was underway.\n"
			"If you loaded a preset, the distortion parameters have been overwritten by the calibration process.\n"
			"If you intended to pre-load the distortion parameters, you may have to re-load the lens preset.");

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