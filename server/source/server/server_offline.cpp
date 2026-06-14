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

#include "server.hpp"
#include "server_internal.hpp"

#include "ui/shared.hpp" // Signals to UI

#include "device/tracking_controller.hpp"
#include "device/tracking_camera.hpp"
#include "device/parsing.hpp"

#include "target/detection3D.hpp"

#include "util/log.hpp"
#include "util/util.hpp" // printBuffer, TimePoint_t
#include "util/debugging.hpp"
#include "util/eigenutil.hpp"
#include "util/threading.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <numeric> // iota

// Coprocessing thread
static void OfflineCoprocessingThread(std::stop_token stop_token, ServerState *statePtr);

// ----------------------------------------------------------------------------
// Simulation Mode
// ----------------------------------------------------------------------------

void StartSimulation(ServerState &state)
{
	if (state.mode != MODE_None)
		return;
	assert(state.controllers.empty());
	assert(state.cameras.empty());

	// Initialise state
	state.mode = MODE_Simulation;
	state.pipeline.isSimulationMode = true;
	state.pipeline.keepInternalData = true;

	{ // Setup cameras
		std::unique_lock dev_lock(state.deviceAccessMutex); // cameras
		for (int c = 0; c < state.config.simulation.cameraDefinitions.size(); c++)
		{
			auto camDef = state.config.simulation.cameraDefinitions[c];
			EnsureCamera(state, camDef.id)->label = asprintf_s("Camera %u", camDef.id);
		}
	}

	{ // Log testing calibrations
		ScopedLogLevel scopedLogLevelInfo(LInfo);
		std::vector<CameraCalib> testingCalibrations;
		for (auto &cam : state.pipeline.cameras)
			testingCalibrations.push_back(cam->simulation.calib);
		LOG(LDefault, LInfo, "Loaded %d simulated calibrations:\n", (int)testingCalibrations.size());
		//DebugCameraParameters(testingCalibrations);
		std::vector<CameraMode> modes;
		modes.reserve(testingCalibrations.size());
		for (auto &calib : testingCalibrations)
		{ // Real one not needed for debugging
			modes.push_back(CameraMode(1280, 800));
			//modes.push_back(getCameraMode(state, calib.id));
		}
		DebugSpecificCameraParameters(testingCalibrations, modes);
	}

	{ // Initialise simulation
		auto simLock = state.pipeline.simulation.contextualLock();
		simLock->objects.clear();
		simLock->resetState();
	}

	// Add testing targets for which there isn't already a calibration
	for (const auto &simTarget : state.config.simulation.trackingTargets)
	{
		const std::string &label = simTarget.first;
		const TargetCalibration3D &calib = simTarget.second;
		int id = 0;

		// Check if this target has been calibrated yet
		auto existingCalib = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
			[&](auto &t) { return t.label == simTarget.first; });
		if (existingCalib != state.trackerConfigs.end())
		{ // Determine offset of calibrated target template compared to ground truth
			LOG(LDefault, LInfo, "Using calibrated target template %s (%d) for simulated target %s!\n",
				existingCalib->label.c_str(), existingCalib->id, label.c_str());
			id = existingCalib->id;

			// Set ground truth target as point cloud
			std::vector<TriangulatedPoint> triPoints;
			triPoints.reserve(calib.markers.size());
			for (const auto &marker : calib.markers)
				triPoints.emplace_back(marker.pos, state.pipeline.params.tri.maxIntersectError, state.pipeline.params.tri.minIntersectionConfidence);
			std::vector<int> triIndices(triPoints.size());
			std::iota(triIndices.begin(), triIndices.end(), 0);

			// Detect match using calibrated target
			auto cand = detectTarget3D(existingCalib->calib, triPoints, triIndices,
				state.pipeline.params.detect.tri.sigmaError, state.pipeline.params.detect.tri.poseSigmaError, false);
			if (cand.points.size() > 0)
			{ // Read out offset transform and correct for it to get accurate error calculations
				for (auto &marker : existingCalib->calib.markers)
				{
					marker.pos = cand.pose * marker.pos;
					marker.nrm = cand.pose.rotation() * marker.nrm;
				}
				LOG(LDefault, LDebug, "Correcting for %.4fmm calibration offset\n", cand.pose.translation().norm() * 10);
			}

			// No need to re-generate lookup tables for target detection
		}
		else
		{ // Add ground truth tracker config

			for (auto &tracker : state.trackerConfigs)
				id = std::min(id, tracker.id);
			id--;

			LOG(LDefault, LInfo, "Using ground truth target template for simulated target %s (%d) with %d points!\n",
				label.c_str(), id, (int)calib.markers.size());

			TrackerConfig tracker(id, label, TargetCalibration3D(calib), TargetDetectionConfig(), true);
			state.trackerConfigs.push_back(std::move(tracker));
		}

		state.pipeline.simulation.contextualLock()->objects.push_back(
			SimulatedObject { .id = id, .label = label, .target = calib, .motionPreset = 2 }
		);
	}

	{ // Align calibrations to simulated calibrations
		auto calibs = state.pipeline.getCalibs();
		AlignWithGT(state.pipeline, calibs);
		AdoptNewCalibrations(state.pipeline, calibs, true);
	}

	// Debug calibrations
	for (auto &cam : state.pipeline.cameras)
	{
		const CameraCalib &tCam = cam->simulation.calib;
		const CameraCalib &cCam = cam->calib;
		Eigen::Vector3d posGT = tCam.transform.translation();
		Eigen::Vector3d rotGT = getEulerXYZ(tCam.transform.rotation()) / PI * 180;
		LOG(LDefault, LTrace, "Cam %d testing transform: Pos/m (%.3f, %.3f, %.3f), Rot/° (%.2f, %.2f, %.2f)\n",
			  cam->id, posGT.x(), posGT.y(), posGT.z(), rotGT.x(), rotGT.y(), rotGT.z());
		if (cCam.transform.translation().sum() != 0)
		{
			Eigen::Vector3d posCB = cCam.transform.translation();
			Eigen::Vector3d rotCB = getEulerXYZ(cCam.transform.rotation()) / PI * 180;
			Eigen::Vector3d tDiff = cCam.transform.translation() - tCam.transform.translation();
			Eigen::Matrix3d rDiff = tCam.transform.rotation() * cCam.transform.rotation().transpose();
			double tError = tDiff.norm(), rError = Eigen::AngleAxis<CVScalar>(rDiff).angle() / PI * 180;
			LOG(LDefault, LTrace, "Cam %d calibrated transform: Pos/m (%.3f, %.3f, %.3f), Rot/° (%.2f, %.2f, %.2f), Error: (%.4fmm, %.4f°)\n",
				  cam->id, posCB.x(), posCB.y(), posCB.z(), rotCB.x(), rotCB.y(), rotCB.z(), tError * 1000, rError);
		}
	}
	LOG(LDefault, LInfo, "=======================\n");

	// Start simulation thread
	assert(state.coprocessingThread == NULL);
	state.coprocessingThread = new std::jthread(OfflineCoprocessingThread, &state);

	IntegrationsInit(state.io, state.config);

	SignalServerEvent(EVT_MODE_SIMULATION_START);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

void StopSimulation(ServerState &state)
{
	if (state.mode != MODE_Simulation)
		return;

	StopStreaming(state);

	StopCoprocessingThread(state);

	IntegrationsCleanup(state.io, state.config);

	std::scoped_lock dev_lock(state.deviceAccessMutex, state.pipeline.pipelineLock);

	// Reset state
	state.mode = MODE_None;
	state.cameras.clear();
	ResetPipelineState(state.pipeline);
	for (auto &tracker : state.trackerConfigs)
		tracker.imu = nullptr;

	SignalServerEvent(EVT_MODE_SIMULATION_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

// ----------------------------------------------------------------------------
// Replay Mode
// ----------------------------------------------------------------------------

void StartReplay(ServerState &state, std::vector<CameraConfigRecord> cameras)
{
	if (state.mode != MODE_None)
		return;

	// Initialise state
	state.mode = MODE_Replay;
	state.pipeline.isSimulationMode = false;
	state.pipeline.keepInternalData = true;

	{ // Setup cameras
		std::unique_lock dev_lock(state.deviceAccessMutex); // cameras 
		for (auto cam : cameras)
		{
			EnsureCamera(state, cam.ID);
			// TODO: Ensure replay camera has the same mode (from image frameX/frameY)? Or not important?
		}
	}

	// Setup IMUs
	state.pipeline.record.imus.clear();
	state.pipeline.record.imus.reserve(state.stored.imus.size());
	for (auto &storedIMU : state.stored.imus)
	{
		auto imu = std::make_shared<IMURecord>(*storedIMU);
		imu->index = state.pipeline.record.imus.size();
		state.pipeline.record.imus.push_back(std::move(imu));
		// Don't setup samples, Start Streaming deletes prior frame and imu records
		// Instead enter samples during replay for relevant frames
	}

	{ // Log testing calibrations
		ScopedLogLevel scopedLogLevelInfo(LInfo);
		std::vector<CameraCalib> testingCalibrations;
		for (auto &cam : state.pipeline.cameras)
			testingCalibrations.push_back(cam->simulation.calib);
		LOG(LDefault, LInfo, "Loaded %d simulated calibrations:\n", (int)testingCalibrations.size());
		//DebugCameraParameters(testingCalibrations);
		std::vector<CameraMode> modes;
		modes.reserve(testingCalibrations.size());
		for (auto &calib : testingCalibrations)
		{ // Real one not needed for debugging
			modes.push_back(CameraMode(1280, 800));
			//modes.push_back(getCameraMode(state, calib.id));
		}
		DebugSpecificCameraParameters(testingCalibrations, modes);
	}

	// Start replay thread
	assert(state.coprocessingThread == NULL);
	state.coprocessingThread = new std::jthread(OfflineCoprocessingThread, &state);

	IntegrationsInit(state.io, state.config);

	SignalServerEvent(EVT_MODE_SIMULATION_START);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

void StopReplay(ServerState &state)
{
	if (state.mode != MODE_Replay)
		return;
	// Essentially same as StopSimulation

	StopStreaming(state);

	StopCoprocessingThread(state);

	IntegrationsCleanup(state.io, state.config);

	std::scoped_lock dev_lock(state.deviceAccessMutex, state.pipeline.pipelineLock);

	// Reset state
	state.mode = MODE_None;
	state.cameras.clear();
	ResetPipelineState(state.pipeline);
	for (auto &tracker : state.trackerConfigs)
		tracker.imu = nullptr;

	// Reset replay
	state.recording = {};
	state.stored.frames.cull_clear();
	state.stored.imus.clear();
	state.stored.frames.delete_culled();
	state.pipeline.params.detect.suspendDetections = false;

	SignalServerEvent(EVT_MODE_SIMULATION_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

// ----------------------------------------------------------------------------
// Coprocessing (Simulation + Replay)
// ----------------------------------------------------------------------------

static void OfflineCoprocessingThread(std::stop_token stop_token, ServerState *statePtr)
{
	ServerState &state = *statePtr;
	PipelineState &pipeline = state.pipeline;

	SetCurrentThreadName("Replay / Sim");

	auto copyDetectionFromStoredRecord = [&state, &pipeline](const std::shared_ptr<FrameRecord> &frameStored, std::shared_ptr<FrameRecord> &frameRecord)
	{
		auto hasTracker = [](const std::shared_ptr<FrameRecord> &frame, auto predicate)
		{
			return frame && std::find_if(frame->trackers.begin(), frame->trackers.end(), predicate) != frame->trackers.end();
		};
		for (const TrackerRecord &trackerRecord : frameStored->trackers)
		{
			if (!trackerRecord.result.isDetected() && !(state.simCopyAlsoFromTracked && trackerRecord.result.isTracked())) continue;
			auto existing = std::find_if(frameRecord->trackers.begin(), frameRecord->trackers.end(),
				[&](const auto &t){ return t.id == trackerRecord.id; });
			if (existing != frameRecord->trackers.end()) continue;
			if (state.simCopyLimitedReinstatement && trackerRecord.result.isTracked())
			{ // Must've just lost tracking, or denied reinstation before
				auto framesRecord = pipeline.record.frames.getView();
				if (!hasTracker(framesRecord[frameRecord->num-1], [&](auto &t){ return t.id == trackerRecord.id && t.result.hasFlag(TrackingResult::REMOVED); }))
					continue;
				// Just lost the tracker, but was still tracking in stored records
				// Check if tracker was still tracking from before last detection in stored frames
				auto framesStored = state.stored.frames.getView();
				bool reinstate = false;
				for (int f = frameRecord->num-1; f > 0; f--)
				{
					if (hasTracker(framesRecord[f], [&](auto &t){ return t.id == trackerRecord.id && t.result.isDetected(); }))
						break; // Tracker allready got reinstate with or after stored detection
					if (hasTracker(framesStored[f], [&](auto &t){ return t.id == trackerRecord.id && t.result.isDetected(); }))
					{ // Found stored detection first, so can reinstate
						reinstate = true;
						break;
					}
				}
				if (!reinstate) continue;
			}
			// Copy detection record - if copied from tracking, clearly denote it as such to allow filtering
			TrackerRecord detectRecord = trackerRecord;
			if (!trackerRecord.result.isDetected())
				detectRecord.result = TrackingResult::COPIED_DETECTION;
			frameRecord->trackers.push_back(std::move(detectRecord));			
			// Switch from dormant to tracked target (if not already tracked)
			auto dormantTarget = std::find_if(pipeline.tracking.dormantTargets.begin(), pipeline.tracking.dormantTargets.end(),
				[&](const auto &d){ return d.id == trackerRecord.id; });
			if (dormantTarget != pipeline.tracking.dormantTargets.end())
			{
				pipeline.tracking.trackedTargets.emplace_back(std::move(*dormantTarget), trackerRecord.pose.observed, frameRecord->time, frameRecord->num, pipeline.params.track);
				pipeline.tracking.dormantTargets.erase(dormantTarget);
			}
			// Probably no need to do the same for TrackedMarker
			SignalTrackerDetected(trackerRecord.id);
		}
	};

	while (!stop_token.stop_requested())
	{
		if (!state.isStreaming)
		{
			UpdatePipelineStatus(pipeline);
			IntegrationsUpdate(state.io, state); // Send camera positions and manager exposed trackers
			IntegrationsReceive(state.io, state); // Keep I/O connections alive
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		if (dbg_isBreaking.load())
		{ // Algorithm hit breakpoint, wait for it to continue
			dbg_isBreaking.wait(true);
			// TODO: Somehow call UpdatePipelineStatus regularly while halted to update background thread status
		}

		if (!state.isStreaming || stop_token.stop_requested())
			continue;

		// Check after breakpoint to allow for halting while in breakpoint
		if (state.simAdvance == 0)
		{ // Wait for next frame advance
			state.simWaiting = true;
			state.simWaiting.notify_all();
			//state.simAdvance.wait(0);
			while (state.simAdvance == 0)
			{ // Instead of wait, to allow thread updates
				UpdatePipelineStatus(state.pipeline);
				IntegrationsUpdate(state.io, state); // Send camera positions and manager exposed trackers
				IntegrationsReceive(state.io, state); // Keep I/O connections alive
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
			state.simWaiting = false;
		}

		if (!state.isStreaming || stop_token.stop_requested())
			continue;

		uint64_t desiredFrameIntervalUS = 1000000 / state.controllerConfig.framerate;

		std::unique_lock pipeline_lock(pipeline.pipelineLock); // for frameNum/GenerateSimulationData
		std::shared_ptr<FrameRecord> frameRecord = nullptr;
		std::size_t frame = pipeline.frameNum+1;
		if (state.mode == MODE_Simulation)
		{
			frameRecord = std::make_shared<FrameRecord>();
			frameRecord->num = frameRecord->ID = frame;
			frameRecord->time = sclock::now();
			frameRecord->timeUTC = std::chrono::system_clock::now();
			GenerateSimulationData(pipeline, *frameRecord);
			if (!pipeline.isSimulationMode)
			{ // Allow disabling of GT calculations easily
				for (auto &cam : frameRecord->cameras)
				{
					cam.simulation.points2GTMarker.clear();
					cam.simulation.GTMarkers2Point.clear();
				}
			}
		}
		else if (state.mode == MODE_Replay)
		{
			if (frame == 0)
			{ // Reset time
				state.recording.replayTime = sclock::now();
				// Ensure timestamp is set properly
				for (auto &imu : pipeline.record.imus)
				{
					imu->samplesRaw.cull_clear();
					imu->samplesFused.cull_clear();
				}
				for (auto &imu : pipeline.record.imus)
				{
					imu->samplesRaw.delete_culled();
					imu->samplesFused.delete_culled();
				}
			}
			else if (state.recording.recordings.size() > 1)
			{ // Check if on transition from one replay to another
				for (int i = 1; i < state.recording.recordings.size(); i++)
				{
					auto &recording = state.recording.recordings[i];
					if (recording.frameStart > frame) break;
					if (recording.frameStart < frame) continue;
					// Frame is indeed start of a recording, interrupt and restart tracking
					// Ensures tracking results are deterministic whether recording is loaded individually or appended
					// NOTE: KEEP IN LINE WITH StartStreaming!
					state.isStreaming = false;
					for (auto &tracker : state.trackerConfigs)
						ServerUpdateTrackerConditions(state, tracker, true);
					InitPipelineStreaming(state.pipeline);
					state.isStreaming = true;
					for (auto &tracker : state.trackerConfigs)
						ServerUpdateTrackerConditions(state, tracker, true);
					auto &lastRec = state.recording.recordings[i-1];
					LOG(LSimulation, LDebug, "Transitioned from recording %d '%s' to %d '%s'!",
						lastRec.number, lastRec.label.c_str(), recording.number, recording.label.c_str());
				}
			}

			auto framesStored = state.stored.frames.getView();
			if (frame < framesStored.endIndex() && framesStored[frame])
			{
				frameRecord = std::make_shared<FrameRecord>();
				auto &loadedRecord = framesStored[frame];
				assert(loadedRecord->num == frame);
				frameRecord->num = loadedRecord->num;
				frameRecord->ID = loadedRecord->ID;
				frameRecord->time = state.recording.replayTime + (loadedRecord->time - framesStored.front()->time);
				frameRecord->timeUTC = convertClock<std::chrono::system_clock::time_point>(frameRecord->time);
				frameRecord->cameras = loadedRecord->cameras;
				if (frame+1 < framesStored.endIndex())
				{ // Replicate original frame pacing
					auto &nextRecord = framesStored[frame+1];
					if (nextRecord)
						desiredFrameIntervalUS = std::chrono::duration_cast<std::chrono::microseconds>(nextRecord->time - loadedRecord->time).count();
					// TODO: Set desired frame end time to better stick to frame time, otherwise replay quickly gets out of sync and VRPN clients will be unhappy due to apparent high latency
				}

				if (!state.keepUnmatchedObservations || !pipeline.simulation.contextualRLock()->replace.empty())
				{ // Replace some target observations with simulated data (configurable in UI)
					ReplaceTargetObservations(pipeline, *frameRecord, loadedRecord->trackers, state.keepUnmatchedObservations);
				}

				// Copy imu samples a bit ahead of the frame into record
				auto targetIMUTime = loadedRecord->time + std::chrono::milliseconds(10);
				assert(pipeline.record.imus.size() == state.stored.imus.size());
				for (int i = 0; i < state.stored.imus.size(); i++)
				{
					auto &imu = pipeline.record.imus[i];
					{
						auto samples = state.stored.imus[i]->samplesFused.getView();
						auto it = samples.pos(imu->samplesFused.getView().endIndex());
						for (; it != samples.end() && it->timestamp < targetIMUTime; it++)
						{ // Copy sample, re-mapping timestamp to current replay time
							auto sample = *it;
							sample.timestamp = state.recording.replayTime + (sample.timestamp - framesStored.front()->time);
							imu->samplesFused.insert(it.index(), sample);
						}
					}
					{
						auto samples = state.stored.imus[i]->samplesRaw.getView();
						auto it = samples.pos(imu->samplesRaw.getView().endIndex());
						for (; it != samples.end() && it->timestamp < targetIMUTime; it++)
						{ // Copy sample, re-mapping timestamp to current replay time
							auto sample = *it;
							sample.timestamp = state.recording.replayTime + (sample.timestamp - framesStored.front()->time);
							imu->samplesRaw.insert(it.index(), sample);
						}
					}
				}

				for (auto &cam : state.cameras)
				{ // Set camera image in cameras
					if (frameRecord->cameras.size() <= cam->pipeline->index || !frameRecord->cameras[cam->pipeline->index].image)
						continue;
					// Asnynchronously decompress camera image record
					threadPool.push([&cam](int, std::shared_ptr<CameraImageRecord> imageRecord)
					{
						auto image = decompressCameraImageRecord(imageRecord);
						if (!image) return; // Image jpeg is faulty, don't store record

						// Store as most recent decompressed image
						cam->receiving.latestFrameImage.swap(image);
						cam->receiving.latestFrameImageRecord.swap(imageRecord);
						// Swap so deconstructor of last frame happens after switch

						SignalCameraRefresh(cam->id);
					}, frameRecord->cameras[cam->pipeline->index].image); // new shared_ptr
				}

				// TODO: If desired, fake a target "detection" using prerecorded tracking results
				// This would speed up verification of tracking for optimising parameters
				// only slightly inaccurate for parameters affecting detection
			}
		}
		if (!frameRecord)
		{
			UpdatePipelineStatus(state.pipeline);
			pipeline_lock.unlock();
			IntegrationsUpdate(state.io, state); // Send camera positions and manager exposed trackers
			IntegrationsReceive(state.io, state); // Keep I/O connections alive
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		int dropout = state.simDropoutIndex.load();
		if (dropout >= state.simDropoutSeverity.size())
			state.simDropoutIndex = -1;
		else if (dropout >= 0)
		{ // Drop a random amount of blobs
			float droprate = state.simDropoutSeverity[dropout];
			state.simDropoutIndex = dropout+1;
			for (auto &camera : frameRecord->cameras)
			{
				auto blobIt = camera.rawPoints2D.begin();
				while (blobIt != camera.rawPoints2D.end())
				{
					float chance = (double)rand() / RAND_MAX;
					if (chance > droprate)
						blobIt++;
					else
						blobIt = camera.rawPoints2D.erase(blobIt);
				}
			}
		}
		if (!pipeline.record.frames.insert(frame, frameRecord)) // new shared_ptr
		{ // Should not happen unless frameRecords culling is incorrectly used in replay mode
			UpdatePipelineStatus(state.pipeline);
			pipeline_lock.unlock();
			std::this_thread::sleep_for(std::chrono::microseconds(desiredFrameIntervalUS));
			continue;
		}
		pipeline_lock.unlock();

		if (!state.isStreaming || stop_token.stop_requested())
			continue;

		auto frameReceiveTime = sclock::now();

		{
			IntegrationsUpdate(state.io, state);
			IntegrationsReceive(state.io, state);

			state.pipeline.params.detect.suspendDetections = state.mode == MODE_Replay && state.simCopyDetectionsFromStored;

			ProcessFrame(pipeline, frameRecord); // new shared_ptr

			if (state.mode == MODE_Replay && state.simCopyDetectionsFromStored)
			{
				auto framesStored = state.stored.frames.getView();
				if (frameRecord->num < framesStored.size() && framesStored[frameRecord->num])
				{
					copyDetectionFromStoredRecord(framesStored[frameRecord->num], frameRecord);
				}
			}

			IntegrationsSendFrame(state.io, state, frameRecord);

			SignalCameraRefresh(0);
			SignalPipelineUpdate();
		}

		{ // Special cases for advancing
			int count = state.simAdvance;
			if (count > 0)
			{ // Advance limited amount of frames, if it fails to reduce, no matter
				state.simAdvance.compare_exchange_weak(count, count-1);
				state.simAdvance.notify_all();
			}
			else if (count == -2)
			{ // Advance until next image - halt advance since we received the next image
				bool haveImageData = false;
				for (auto &cam : state.cameras)
					haveImageData |= (bool)frameRecord->cameras[cam->pipeline->index].image;
				if (haveImageData)
				{
					state.simAdvance = 0;
					state.simAdvance.notify_all();
				}
				else {} // Advance freely, do nothing
			}
			else if (count == -1)
			{} // Advance freely, do nothing
		}

		if (!state.simAdvanceQuickly)
			std::this_thread::sleep_until(frameReceiveTime + std::chrono::microseconds(desiredFrameIntervalUS));

		// Waiting for processing of last frame to finish
		//while (threadPool.n_idle() != threadPool.size())
		//	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}