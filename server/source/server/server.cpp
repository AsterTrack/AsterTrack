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
#include "signals.hpp" // Signals to Server

#include "device/tracking_controller.hpp"
#include "device/tracking_camera.hpp"

#include "comm/socket.hpp"
#include "comm/usb.hpp"

#include "util/log.hpp"
#include "util/util.hpp" // printBuffer, TimePoint_t
#include "util/eigenutil.hpp" // UpdateDerived used in CameraCalib()

// Processing
#include "ctpl/ctpl.hpp"
ctpl::thread_pool threadPool = ctpl::thread_pool(6);

// util/debugging.hpp
#include "util/debugging.hpp"
std::atomic<bool> dbg_isBreaking;
std::atomic<int> dbg_debugging;

// ----------------------------------------------------------------------------
// Server Lifetime
// ----------------------------------------------------------------------------

bool ServerInit(ServerState &state)
{
	auto handleConfigError = [](const std::optional<ErrorMessage> &&error, bool missingOK)
	{
		if (error && (!missingOK || error->code != ENOENT))
			SignalErrorToUser(error.value());
	};

	// Read configurations
	handleConfigError(parseGeneralConfigFile(persistentConfigFolder + "/general_config.json", state.config), false);
	state.generalConfigDirty = false;
	// TODO: Add UI and storage of general config
	handleConfigError(parseCameraConfigFile(persistentConfigFolder + "/camera_config.json", state.cameraConfig), true);
	state.cameraConfigDirty = false;
	// Also load lens presets in case they are needed for camera calib UI
	handleConfigError(parseLensPresets(persistentConfigFolder + "/lens_presets.json", state.lensPresets, state.defaultLens), true);
	if (state.lensPresets.empty())
		handleConfigError(parseLensPresets(persistentConfigFolder + "/lens_presets_builtin.json", state.lensPresets, state.defaultLens), false);

	// Load calibrations
	handleConfigError(parseCameraCalibrations(permanentStoreFolder + "/camera_calib.json", state.cameraCalibrations), true);
	state.cameraCalibsDirty = false;
	handleConfigError(parseTrackerConfigurations(trackerConfigFolder, state.trackerConfigs), true);
	if (state.trackerConfigs.empty())
	{
		auto error = parseTrackerConfigurationsLegacy(permanentStoreFolder + "/trackers.json", state.trackerConfigs);
		if (!error && !state.trackerConfigs.empty())
			handleConfigError(storeTrackerConfigurations(trackerConfigFolder, state.trackerConfigs), false);
		handleConfigError(std::move(error), true);
	}

	{ // Debug calibration
		ScopedLogLevel scopedLogLevelInfo(LInfo);
		LOG(LDefault, LInfo, "Loaded %d stored calibrations:\n", (int)state.cameraCalibrations.size());
		std::vector<CameraMode> modes;
		modes.reserve(state.cameraCalibrations.size());
		for (auto &calib : state.cameraCalibrations)
		{ // Real one not needed for debugging
			modes.push_back(CameraMode(1280, 800));
			//modes.push_back(getCameraMode(state, calib.id));
		}
		DebugSpecificCameraParameters(state.cameraCalibrations, modes);
	}

	// Initialise USB subsystem (for e.g. hotplug)
	state.libusb_context = comm_init_context();

	// Initialise socket once for all wireless comms
	if (socket_initialise() < 0)
	{
		LOG(LServer, LDebug, "Failed to initialise socket!\n");
	}

	// Initialise wireless server for cameras
	WirelessServerInit();
	state.server.host = getHostnameString();

	return true;
}

void ServerExit(ServerState &state)
{
	StopSimulation(state);
	StopReplay(state);
	StopDeviceMode(state);

	while (threadPool.n_idle() != threadPool.size())
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

	// Cleanup USB devices and USB context
	if (state.libusb_context)
		comm_exit_context(state.libusb_context);
	state.libusb_context = nullptr;

	WirelessServerCleanup();

	socket_cleanup();

	state.trackerConfigs.clear();
}

void StopCoprocessingThread(ServerState &state)
{
	if (!state.coprocessingThread) return;
	state.coprocessingThread->request_stop();
	state.simAdvance = -1;
	state.simAdvance.notify_all();
	dbg_debugging = 0;
	dbg_debugging.notify_all();
	dbg_isBreaking = false;
	dbg_isBreaking.notify_all();
	delete state.coprocessingThread;
	state.coprocessingThread = NULL;
}

// ----------------------------------------------------------------------------
// Config Updates
// ----------------------------------------------------------------------------

bool ServerUpdateTrackerIMU(ServerState &state, TrackerConfig &tracker)
{
	// TODO: RemoteIMU identity problem on reconnection (1/2)
	// This can not determine a IMU was disconnected and reconnected as a different IMUDevice

	bool updatedIMU = false;
	// Ensure IMU is properly associated
	if (tracker.imu && tracker.imu->id != tracker.imuIdent)
	{ // Switched / Removed IMU
		tracker.imu = nullptr;
		updatedIMU = true;
	}
	if (!tracker.imu && tracker.imuIdent)
	{ // Attempt to find IMU among those connected
		for (auto &imu : state.pipeline.record.imus)
		{
			if (imu->id != tracker.imuIdent) continue;
			tracker.imu = imu;
			updatedIMU = true;
		}
	}
	return updatedIMU;
}

void ServerUpdateTrackerConfig(ServerState &state, TrackerConfig &tracker, bool updatedIMU)
{
	if (!state.isStreaming) return;
	if (!tracker.triggered) return;

	// Set (or update) tracked object with tracker config
	if (tracker.type == TrackerConfig::TRACKER_TARGET)
		SetTrackedTarget(state.pipeline, tracker.id, tracker.label, tracker.calib, tracker.detectionConfig);
	else if (tracker.type == TrackerConfig::TRACKER_MARKER)
		SetTrackedMarker(state.pipeline, tracker.id, tracker.label, tracker.markerSize);
	else if (tracker.type == TrackerConfig::TRACKER_VIRTUAL)
		SetVirtualTracker(state.pipeline, tracker.id, tracker.label, tracker.virtConfig);

	if (updatedIMU)
	{ // Ensure tracked object receives updated IMU association
		DisassociateIMU(state.pipeline, tracker.id);
		if (tracker.imu)
			AssociateIMU(state.pipeline, tracker.imu, tracker.id, tracker.imuCalib);
	}
}

void ServerUpdateTrackerConditions(ServerState &state, TrackerConfig &tracker, bool initial, int manualTrigger, int manualExpose)
{
	if (!state.isStreaming)
	{
		tracker.tracked = tracker.triggered = tracker.exposed = false;
		return;
	}

	if (!tracker.triggered)
	{ // Check trigger conditions
		if (manualTrigger > 0)
			tracker.triggered = true;
		else if (tracker.trigger == TrackerConfig::TRIGGER_BY_DEFAULT && initial)
			tracker.triggered = true;
		else if ((tracker.trigger & TrackerConfig::TRIGGER_ON_IMU_CONNECT) && tracker.imu)
			tracker.triggered = true;
		else if ((tracker.trigger & TrackerConfig::TRIGGER_ON_IO_CONNECT) && tracker.connected)
			tracker.triggered = true;

		if (tracker.triggered)
		{ // Update pipeline with triggered tracker
			ServerUpdateTrackerIMU(state, tracker);
			ServerUpdateTrackerConditions(state, tracker, true);
			ServerUpdateTrackerConfig(state, tracker, true);
		}
	}
	else if (manualTrigger < 0)
	{ // Remove tracker
		tracker.triggered = false;
		if (tracker.type == TrackerConfig::TRACKER_TARGET)
			RemoveTrackedTarget(state.pipeline, tracker.id);
		else if (tracker.type == TrackerConfig::TRACKER_MARKER)
			RemoveTrackedMarker(state.pipeline, tracker.id);
		else if (tracker.type == TrackerConfig::TRACKER_VIRTUAL)
			RemoveVirtualTracker(state.pipeline, tracker.id);
	}

	if (!tracker.exposed)
	{ // Check expose conditions
		if (manualExpose > 0)
			tracker.exposed = true;
		else if (tracker.expose == TrackerConfig::EXPOSE_BY_DEFAULT && initial)
			tracker.exposed = true;
		else if (tracker.expose == TrackerConfig::EXPOSE_ONCE_TRIGGERED && tracker.triggered)
			tracker.exposed = true;
		else if (tracker.expose == TrackerConfig::EXPOSE_ONCE_TRACKED && tracker.tracked)
			tracker.exposed = true;
		// CheckTrackingIO will update integrations with new exposed state accordingly
	}
	else if (manualExpose < 0)
	{ // CheckTrackingIO will update integrations with new exposed state accordingly
		tracker.exposed = false;
	}
}

// ----------------------------------------------------------------------------
// Signals
// ----------------------------------------------------------------------------

void SignalTrackerDetected(int trackerID)
{
	for (auto &tracker : GetState().trackerConfigs)
	{
		if (tracker.id != trackerID) continue;
		tracker.tracked = true;
		ServerUpdateTrackerConditions(GetState(), tracker);
		return;
	}
}

void SignalTargetCalibUpdate(int trackerID, TargetCalibration3D calib)
{
	for (auto &tracker : GetState().trackerConfigs)
	{
		if (tracker.id != trackerID) continue;
		tracker.calib = calib;
		tracker.targetDirty = true;
		if (tracker.isSimulated)
		{ // Ensure calibrated, simulated targets are adopted as normal targets and stored
			tracker.isSimulated = false;
		}
		// Update pipeline - this update may have come from pipeline, still
		ServerUpdateTrackerConfig(GetState(), tracker);
		return;
	}
	std::string label = asprintf_s("New Target %d", trackerID);
	GetState().trackerConfigs.push_back(TrackerConfig(trackerID, label, std::move(calib), TargetDetectionConfig()));
}

void SignalIMUCalibUpdate(int trackerID, IMUIdent ident, IMUCalib calib)
{
	for (auto &tracker : GetState().trackerConfigs)
	{
		if (tracker.id != trackerID) continue;
		tracker.imuIdent = ident;
		tracker.imuCalib = calib;
		tracker.configDirty = true;
		// Update pipeline - this update may have come from pipeline, still
		bool updatedIMU = ServerUpdateTrackerIMU(GetState(), tracker);
		ServerUpdateTrackerConditions(GetState(), tracker);
		ServerUpdateTrackerConfig(GetState(), tracker, updatedIMU);
		return;
	}
}

void SignalCameraCalibUpdate(std::vector<CameraCalib> calibs)
{
	auto &state = GetState();
	for (auto &calib : calibs)
	{ // Integrate changed calibs into stored list of calibrations
		if (calib.invalid()) continue;
		int i;
		for (i = 0; i < state.cameraCalibrations.size(); i++)
		{
			if (state.cameraCalibrations[i].id == calib.id)
			{ // Found calibration entry, update calibration data
				state.cameraCalibrations[i] = calib;
				break;
			}
		}
		if (i >= state.cameraCalibrations.size())
		{ // New camera with calibration, add as calibration entry
			state.cameraCalibrations.push_back(calib);
		}
		GetState().cameraCalibsDirty = true;
	}
	// Signal UI that calibrations got updated
	SignalServerEvent(EVT_UPDATE_CALIBS);
	// This update is assumed to have come from pipeline, so no need to update it
}

void SignalErrorToUser(ErrorMessage error)
{
	LOG(LGUI, LWarn, "Error: %s", error.c_str());
	// TODO: Consider making thread-safe. But is called very rarely
	GetState().errors.push(std::move(error));	
}

// ----------------------------------------------------------------------------
// Camera Setup
// ----------------------------------------------------------------------------

CameraMode getCameraMode(ServerState &state, CameraID id)
{
	CameraConfig &config = state.cameraConfig.getCameraConfig(id);
	// TODO:Rework CameraMode to properly support cropping/zooming, it is only surface level supported right now, and ugly
	return CameraMode(config.width, config.height);
}

static std::shared_ptr<CameraPipeline> EnsureCameraPipeline(ServerState &state, CameraID id)
{
	auto cam = std::find_if(state.pipeline.cameras.begin(), state.pipeline.cameras.end(), [id](const auto &c) { return c->id == id; });
	if (cam != state.pipeline.cameras.end())
		return *cam;
	std::unique_lock pipeline_lock(state.pipeline.pipelineLock);

	std::shared_ptr<CameraPipeline> camera = std::make_shared<CameraPipeline>();
	camera->id = id;
	camera->index = state.pipeline.cameras.size();
	state.pipeline.cameras.push_back(camera); // new shared_ptr

	// Assign slot in observations
	state.pipeline.seqDatabase.contextualLock()->verifyCameraCount(state.pipeline.cameras.size());

	// Setup camera settings
	camera->mode = getCameraMode(state, id);

	// Add calibration
	for (int i = 0; i < state.cameraCalibrations.size(); i++)
	{ // Try to find calibration from those loaded
		if (state.cameraCalibrations[i].id == camera->id)
		{ // Camera has stored calibration
			camera->calib = state.cameraCalibrations[i];
			camera->calib.index = camera->index;
			break;
		}
	}
	if (state.mode == MODE_Simulation || state.mode == MODE_Replay)
	{ // Assign simulated calibration
		for (int i = 0; i < state.config.simulation.cameraDefinitions.size(); i++)
		{
			if (state.config.simulation.cameraDefinitions[i].id == camera->id)
			{ // Camera has simulated calibration
				camera->simulation.calib = state.config.simulation.cameraDefinitions[i];
				camera->simulation.calib.index = camera->index;
				if (camera->calib.invalid())
					camera->calib = camera->simulation.calib;
				break;
			}
		}
	}
	if (camera->calib.valid())
	{ // Calculate fundamental matrices from calibration
		auto lock = folly::detail::lock(folly::detail::wlock(state.pipeline.calibration), folly::detail::rlock(state.pipeline.seqDatabase));
		UpdateCalibrationRelations(state.pipeline, *std::get<0>(lock), *std::get<1>(lock), camera->index);
	}
	else
	{
		if (state.defaultLens > 0)
			camera->calib = CameraCalib(state.lensPresets[state.defaultLens]);
		else camera->calib = CameraCalib();
		LOG(LDefault, LInfo, "No calibration found for camera %u!\n", camera->id);
	}
	camera->calibRoom = camera->calib;

	return camera;
}

std::shared_ptr<TrackingCameraState> EnsureCamera(ServerState &state, CameraID id)
{
	auto cam = std::find_if(state.cameras.begin(), state.cameras.end(), [id](const auto &c) { return c->id == id; });
	if (cam != state.cameras.end())
		return *cam;
	std::shared_ptr<TrackingCameraState> camera = std::make_shared<TrackingCameraState>();
	camera->id = id;
	camera->pipeline = EnsureCameraPipeline(state, id);
	camera->state.contextualLock()->lastDeviceChange = sclock::now();
	state.cameras.push_back(camera); // new shared_ptr
	// TODO: Setup sync groups in EnsureCamera (based on prior config, e.g. in UI) 1/4
	return camera;
}

std::shared_ptr<TrackingCameraState> GetCamera(ServerState &state, CameraID id)
{
	auto cam = std::find_if(state.cameras.begin(), state.cameras.end(), [id](const auto &c) { return c->id == id; });
	if (cam != state.cameras.end())
		return *cam;
	return nullptr;
}

// ----------------------------------------------------------------------------
// Streaming
// ----------------------------------------------------------------------------

bool StartStreaming(ServerState &state)
{
	if (state.mode == MODE_None || state.isStreaming)
		return false;
	LOG(LDefault, LInfo, "Starting stream!\n");

	/* if (state.cameras.empty())
	{
		LOG(LDefault, LWarn, "Will not start streaming with 0 tracking cameras!\n");
		return false;
	} */

	std::unique_lock pipeline_lock(state.pipeline.pipelineLock);

	// Clean any prior streaming state
	// TODO: Find a way to retain records and calibration state of last Streaming (without leaving current mode!)
	// Currently after stop	ping stream, all data is retained, and only deleted after starting stream again or leaving mode
	// Now instead of deleting it upon starting stream again, append to the records and keep calibration state intact
	// Needs support for frame num generated by controller to be different from frameNum in pipeline
	// NOTE: Also edit clearing records in EVT_START_STREAMING
	ResetPipelineData(state.pipeline);

	// Initialise state - NOTE: KEEP IN LINE WITH Replay transition between captures!
	InitPipelineStreaming(state.pipeline);
	state.isStreaming = true;
	state.lastStreamingStart = sclock::now();

	// Setup trackers - NOTE: KEEP IN LINE WITH Replay transition between captures!
	for (auto &tracker : state.trackerConfigs)
	{ // Check initial trigger conditions of trackers
		ServerUpdateTrackerConditions(state, tracker, true);
	}

	if (state.mode == MODE_Device)
	{
		std::shared_lock dev_lock(state.deviceAccessMutex);

		// Configure sync groups
		SetupSyncGroups(state);

		// Tell devices (wireless and wired) to start streaming
		DevicesStartStreaming(state);

		// Start realtime processing thread
		assert(state.rtProcessingThread == NULL);
		state.rtProcessingThread = new std::jthread(RealtimeProcessingThread, &state);
	}

	SignalServerEvent(EVT_START_STREAMING);

	LOG(LDefault, LInfo, "Started stream!\n");
	return true;
}

void StopStreaming(ServerState &state)
{
	if (!state.isStreaming)
		return;
	LOG(LDefault, LInfo, "Stopping stream!\n");

	if (state.mode == MODE_Device)
	{
		std::shared_lock dev_lock(state.deviceAccessMutex);

		// Tell devices (wireless and wired) to stop streaming
		DevicesStopStreaming(state);

		// Sync Group not attached to any controller - remove
		DeleteVirtualSyncGroup(state);

		// Reset streaming states
		ResetStreamState(*state.stream.contextualLock());

		// Join realtime processing thread
		state.processing_cv.notify_all();
		delete state.rtProcessingThread;
		state.rtProcessingThread = NULL;
	}

	state.isStreaming = false;
	ResetPipelineStreaming(state.pipeline);

	SignalServerEvent(EVT_STOP_STREAMING);

	LOG(LDefault, LInfo, "Stopped stream!\n");

	for (auto &tracker : state.trackerConfigs)
	{ // Reset after streaming and check expose conditions
		ServerUpdateTrackerConditions(state, tracker);
	}
	// Update exposed state
	IntegrationsUpdate(state.io, state);
	for (auto &tracker : state.trackerConfigs)
	{
		assert(!tracker.exposed && tracker.connected == 0);
	}
}