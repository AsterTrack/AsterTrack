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

//#define LOG_MAX_LEVEL LTrace

#include "server.hpp"

#include "ui/shared.hpp" // Signals to UI
#include "signals.hpp" // Signals to Server

#include "device/tracking_controller.hpp"
#include "device/tracking_camera.hpp"
#include "device/parsing.hpp"

#include "comm/wireless_server_client.hpp"
#include "comm/usb.hpp" // USB Device comm

#include "pipeline/pipeline.hpp"
#include "target/detection3D.hpp"

#include "io/vrpn.hpp" // Outputting data over VRPN

#include "util/log.hpp"
#include "util/util.hpp" // printBuffer, TimePoint_t
#include "util/debugging.hpp"
#include "util/eigenutil.hpp"

//#include <execinfo.h>
//#include <signal.h>

#include <chrono>
#include <memory>
#include <numeric> // iota

#include "ctpl/ctpl.hpp"
#include <omp.h>

#include "hidapi/hidapi.h"

/**
 * Main program flow
 */

// Processing
ctpl::thread_pool threadPool = ctpl::thread_pool(6);

// util/debugging.hpp
std::atomic<bool> dbg_isBreaking;
std::atomic<int> dbg_debugging;


/* Functions */

static void DeviceSupervisorThread(std::stop_token stop_token, ServerState *statePtr);
static void RealtimeProcessingThread(std::stop_token stop_token, ServerState *statePtr);
static void SimulationThread(std::stop_token stop_token, ServerState *statePtr);

bool ServerInit(ServerState &state)
{
	state.libusb_context = comm_init_context();

	auto handleConfigError = [](const std::optional<ErrorMessage> &&error, bool missingOK)
	{
		if (error && (!missingOK || error->code != ENOENT))
			SignalErrorToUser(error.value());
	};

	// Read configurations
	handleConfigError(parseGeneralConfigFile("store/general_config.json", state.config), false);
	state.generalConfigDirty = false;
	// TODO: Add UI and storage of general config
	handleConfigError(parseCameraConfigFile("store/camera_config.json", state.cameraConfig), true);
	state.cameraConfigDirty = false;
	// Also load lens presets in case they are needed for camera calib UI
	handleConfigError(parseLensPresets("store/lens_presets.json", state.lensPresets, state.defaultLens), true);
	if (state.lensPresets.empty())
		handleConfigError(parseLensPresets("store/lens_presets_builtin.json", state.lensPresets, state.defaultLens), false);

	// Load calibrations
	handleConfigError(parseCameraCalibrations("store/camera_calib.json", state.cameraCalibrations), true);
	state.cameraCalibsDirty = false;
	handleConfigError(parseTrackerConfigurations("store/trackers.json", state.trackerConfigs), true);
	state.trackerConfigDirty = state.trackerCalibsDirty = state.trackerIMUsDirty = false;

	{
		// Debug calibration
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

	omp_set_num_threads(omp_get_max_threads());

	WirelessServerInit();
	state.server.host = WirelessServerGetHostname();

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

	ResetIO(state);

	WirelessServerCleanup();
}

void ServerUpdatedTrackerConfig(ServerState &state, TrackerConfig &tracker)
{
	state.trackerConfigDirty = true;

	if (!state.isStreaming) return;
	// Trigger tracker to be considered for tracking

	bool updatedIMU = false;
	if (tracker.imuIdent)
	{ // Ensure IMU is properly associated
		if (tracker.imu && tracker.imu->id != tracker.imuIdent)
		{ // Switched / Removed IMU
			tracker.imu = nullptr;
			updatedIMU = true;
		}
		if (!tracker.imu)
		{ // Attempt to find IMU among those connected
			for (auto &imu : state.pipeline.record.imus)
			{
				if (imu->id != tracker.imuIdent) continue;
				tracker.imu = imu;
				updatedIMU = true;
			}
		}
	}

	if (!tracker.triggered)
	{ // Ensure trigger conditions have been met
		if (tracker.trigger == TrackerConfig::TRIGGER_ALWAYS)
			tracker.triggered = true;
		else if (tracker.trigger == TrackerConfig::TRIGGER_ON_IMU_CONNECT && tracker.imu)
			tracker.triggered = true;
		else if (tracker.trigger == TrackerConfig::TRIGGER_ON_IO_CONNECT && tracker.connected)
			tracker.triggered = true;
		else
			return;
	}

	// Set (or update) tracked object with tracker config
	if (tracker.type == TrackerConfig::TRACKER_TARGET)
		SetTrackedTarget(state.pipeline, tracker.id, tracker.label, tracker.calib, tracker.detectionConfig);
	else if (tracker.type == TrackerConfig::TRACKER_MARKER)
		SetTrackedMarker(state.pipeline, tracker.id, tracker.label, tracker.markerSize);

	if (updatedIMU)
	{ // Ensure tracked object receives updated IMU association
		DisassociateIMU(state.pipeline, tracker.id);
		if (tracker.imu)
			AssociateIMU(state.pipeline, tracker.imu, tracker.id, tracker.imuCalib);
	}
}

// Signals

void SignalTrackerDetected(int trackerID)
{
	for (auto &tracker : GetState().trackerConfigs)
	{
		if (tracker.id != trackerID) continue;
		tracker.tracked = true;
		return;
	}
}

void SignalTargetCalibUpdate(int trackerID, TargetCalibration3D calib)
{
	for (auto &tracker : GetState().trackerConfigs)
	{
		if (tracker.id != trackerID) continue;
		tracker.calib = calib;
		if (tracker.isSimulated)
		{ // Ensure calibrated, simulated targets are adopted as normal targets and stored
			tracker.isSimulated = false;
		}
		GetState().trackerCalibsDirty = true;
		// Update pipeline - this update may have come from pipeline, still
		ServerUpdatedTrackerConfig(GetState(), tracker);
		return;
	}
}

void SignalIMUCalibUpdate(int trackerID, IMUIdent ident, IMUCalib calib)
{
	for (auto &tracker : GetState().trackerConfigs)
	{
		if (tracker.id != trackerID) continue;
		tracker.imuIdent = ident;
		tracker.imuCalib = calib;
		GetState().trackerIMUsDirty = true;
		// Update pipeline - this update may have come from pipeline, still
		ServerUpdatedTrackerConfig(GetState(), tracker);
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
	GetState().errors.push(std::move(error));	
}

// ----------------------------------------------------------------------------
// Pipeline Setup
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
		LOG(LDefault, LInfo, "No calibration found for camera %d!\n", camera->id);
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
// Device Mode
// ----------------------------------------------------------------------------

void StartWirelessServer(ServerState &state)
{
	if (state.server.thread)
		return;
	LOG(LServer, LInfo, "Starting network server!\n");
	// Init server
	if (state.server.portSet.empty())
		state.server.portSet = "45732";
	state.server.portUsed = state.server.portSet;
	state.server.socket = WirelessServerOpen(state.server.portUsed);
	if (state.server.socket == 0)
	{
		LOG(LServer, LError, "Could not start network server!\n");
		return;
	}
	state.server.callbacks.userData1 = &state;
	state.server.callbacks.onIdentify = [](ClientCommState &client) { // Find or setup camera
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		LOG(LServer, LInfo, "Established server connection to camera with id %d!\n", client.otherIdent.id);
		std::unique_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(50));
		if (!dev_lock.owns_lock())
		{ // Likely StopDeviceMode waiting on us to stop thread
			if (state.mode != MODE_Device)
				LOG(LServer, LDarn, "Aborted wireless camera connection because we're exiting device mode!\n");
			else
			 	LOG(LServer, LError, "Aborted wireless camera connection because of failure to aquire lock for unknown reason!\n");
			return false;
		}
		// Setup camera with server connection
		std::shared_ptr<TrackingCameraState> camera = EnsureCamera(state, client.otherIdent.id);
		if (camera->client)
		{ // Already had a wireless client connection, perhaps camera crashed and didn't terminate the last connection properly
			camera->client->thread->request_stop();
			LOG(LServer, LWarn, "Camera #%u was already wirelessly connected, maybe had a stale connection?", camera->id);
		}
		camera->client = &client;
		client.callbacks.userData2 = camera;
		{
			auto camState = camera->state.contextualLock();
			camState->hadServerConnected = true;
			// Clear any possible error state
			if (camState->error.encountered && dtMS(camState->error.time, sclock::now()) > 1000)
			{ // Might need this overlap protection to prevent race-conditions
				camState->error.recoverTime = sclock::now();
				camState->error.recovered = true;
				camState->error.encountered = false;
			}
		}
		// Setup stream subsystem
		if (!camera->sync)
		{ // Init with internal sync if no controller is known to be connected
			SetCameraSyncNone(*state.stream.contextualLock(), camera, 1000.0f / 144);
		}
		SignalServerEvent(EVT_UPDATE_CAMERAS);
		return true;
	};
	state.server.callbacks.onDisconnect = [](ClientCommState &client) { // Remove camera
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		if (client.callbacks.userData2)
		{ // Camera was already identified
			if (client.callbacks.userData2.use_count() == 1)
				LOG(LServer, LWarn, "Disconnecting camera was already erased from devices!\n");
			TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
			if (camera.client != &client)
			{
				LOG(LServer, LWarn, "Stale client connection thread of camera #%u disconnected!\n", camera.id);
				return;
			}
			{
				auto state = camera.state.contextualLock();
				state->lastWirelessConnection = sclock::now();
			}
			if (client.callbacks.userData2.use_count() == 1)
				LOG(LServer, LWarn, "Disconnecting camera was already erased from devices!\n");
			camera.client = NULL;
			LOG(LServer, LInfo, "Camera with id %d lost server connection!\n", camera.id);
			// If camera is now unreachable, DeviceSupervisorThread will wait for it to reconnect before removing it entirely
			client.callbacks.userData2 = nullptr;
		}
		else
		{
			LOG(LServer, LWarn, "Lost connection to camera before it could be identified!\n");
		}
	};
	state.server.callbacks.onReceivePacketHeader = [](ClientCommState &client, PacketHeader &header, TimePoint_t receiveTime)
	{
		if (!header.isStreamPacket() && header.tag != PACKET_FRAME_SIGNAL)
			return true;
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		std::shared_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(10));
		if (!dev_lock.owns_lock()) return false; // Likely StopDeviceMode waiting on us to stop thread
		assert(client.callbacks.userData2);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		if (!camera.sync)
		{
			LOG(LStreaming, LTrace, "Wireless Camera %d send a streaming packet but was not set up for streaming!\n", camera.id);
			return false;
		}
		if (header.tag == PACKET_FRAME_SIGNAL)
		{ // Frame is starting to be processed
			auto sync_lock = camera.sync->contextualLock();
			if (sync_lock->source == SYNC_NONE)
			{ // No sync, this is only source for SOF timing, estimate actual SOF
				// Update estimate of SOF (assuming regular SOFs)
				TimePoint_t timeSOF = receiveTime - std::chrono::microseconds(8000);
				//timeSOF = UpdateSOFPredictions(*camera.sync, header.frameID, timeSOF);
				FrameID frameID = EstimateFullFrameID(*sync_lock, header.frameID);
				RegisterSOF(*sync_lock, frameID, timeSOF);
			}
			auto frame = RegisterCameraFrame(*sync_lock, camera.syncIndex, header.frameID);
			if (frame)
				LOG(LStreaming, LDebug, "Wireless Camera %d announced packet for frame %d (%d)!\n", camera.id, frame->ID, header.frameID);
			else
				LOG(LStreaming, LDebug, "Wireless Camera %d announced packet for non-existant frame with ID %d!\n", camera.id, header.frameID);
			return true;
		}
		else if (header.isStreamPacket())
		{
			if (!RegisterStreamPacket(*camera.sync->contextualLock(), camera.syncIndex, header.frameID, receiveTime))
			{
				LOG(LStreaming, LTrace, "Wireless Camera %d sent a streaming packet but was not set up for streaming!\n", camera.id);
				return false;
			}
		}
		return true;
	};
	state.server.callbacks.onReceivePacketBlock = [](ClientCommState &client, PacketHeader &header, uint8_t *data, unsigned int len, TimePoint_t receiveTime)
	{
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		std::shared_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(10));
		if (!dev_lock.owns_lock()) return; // Likely StopDeviceMode waiting on us to stop thread
		assert(client.callbacks.userData2);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		// Update statistics of streaming packet
		if (header.isStreamPacket())
		{
			if (!RegisterStreamBlock(*camera.sync->contextualLock(), camera.syncIndex, header.frameID))
			{
				LOG(LParsing, LError, "---- Received stream block for non-existant frame %d or unregistered stream packet!\n", header.frameID);
			}
		}
	};
	state.server.callbacks.onReceivePacket = [](ClientCommState &client, PacketHeader &header, uint8_t *data, unsigned int len, TimePoint_t receiveTime, bool erroneous)
	{
		assert(client.callbacks.userData2);
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		std::shared_lock dev_lock(state.deviceAccessMutex, std::chrono::milliseconds(10));
		if (!dev_lock.owns_lock()) return; // Likely StopDeviceMode waiting on us to stop thread
		if (header.isStreamPacket())
		{
			auto cameraFrame = ReadStreamingPacket(camera, header, data, len, erroneous);
			int blobCount = cameraFrame.rawPoints2D.size();
			auto sync_lock = camera.sync->contextualLock();
			SyncedFrame *frame = RegisterStreamPacketComplete(*sync_lock, camera.syncIndex, header.frameID, std::move(cameraFrame), erroneous);
			if (frame)
			{ // Packet existed for frame
				LOG(LStreaming, LTrace, "Camera %d (TCP) fully transmitted stream packet %d for frame %d (%d) with %d blobs!\n",
					camera.id, header.tag, frame->ID, frame->ID&0xFF, blobCount);
				if (frame->previouslyProcessed)
				{
					LOG(LStreaming, LTrace, "---- Camera %d finally transmitted stream packet for frame %d with %d blobs, %.2fms into the frame, %.2fms after processing!\n",
						camera.id, frame->ID, blobCount, dtMS(frame->SOF, sclock::now()), dtMS(frame->lastProcessed, sclock::now()));
				}
			}
			else
			{
				LOG(LParsing, LError, "---- Completed stream packet for non-existant frame %d or unregistered stream packet!\n", header.frameID);
			}
		}
		else
		{
			ReadCameraPacket(camera, header, data, len, erroneous);
		}
	};

	// Start comm thread
	state.server.thread = new std::jthread(WirelessServerThread, &state.server);
}

void StopWirelessServer(ServerState &state)
{
	if (!state.server.thread)
		return;
	LOG(LServer, LInfo, "Stopping network server!\n");
	state.server.portUsed.clear();
	// Join server thread, it will do cleanup
	delete state.server.thread;
	state.server.thread = NULL;
}

bool StartDeviceMode(ServerState &state)
{
	if (state.mode != MODE_None)
		return false;

	// Initialise state
	state.mode = MODE_Device;
	state.pipeline.isSimulationMode = false;
	state.pipeline.keepInternalData = false;

	// Connect to IMU providers
	hid_init();
	{ // Fetch IMU providers
		auto imu_lock = state.imuProviders.contextualLock();
		std::unique_lock hid_lock(state.hid_access);
		detectAsterTrackReceivers(*imu_lock);
		detectSlimeVRReceivers(*imu_lock);
		initialiseRemoteIMUs(*imu_lock);
		// TODO: Add more IMU integrations here
	}

	// Connect new controllers
	DetectNewControllers(state);

	// Start server for wireless cameras
	StartWirelessServer(state);

	// Start device supervisor thread
	assert(state.coprocessingThread == NULL);
	state.coprocessingThread = new std::jthread(DeviceSupervisorThread, &state);

	SignalServerEvent(EVT_MODE_DEVICE_START);
	SignalServerEvent(EVT_UPDATE_CAMERAS);

	return true;
}

void StopDeviceMode(ServerState &state)
{
	if (state.mode != MODE_Device)
		return;

	// Stop streaming
	if (state.isStreaming)
		StopStreaming(state);

	LOG(LDefault, LInfo, "Disconnecting!\n");

	// Join coprocessing thread
	state.parsing_cv.notify_all();
	delete state.coprocessingThread;
	state.coprocessingThread = NULL;

	std::scoped_lock dev_lock(state.deviceAccessMutex, state.pipeline.pipelineLock);
	state.mode = MODE_None;

	// Disconnect from controllers and their wired-only cameras
	while (!state.controllers.empty())
		DisconnectController(state, *state.controllers.back());

	// Stop wireless server
	StopWirelessServer(state);

	// Disconnect from all remaining wireless cameras
	while (!state.cameras.empty())
		CameraCheckDisconnected(state, *state.cameras.back());

	// Diconnect IMU providers
	state.imuProviders.contextualLock()->clear();
	hid_exit();
	for (auto &tracker : state.trackerConfigs)
		tracker.imu = nullptr;

	assert(state.controllers.empty());
	assert(state.cameras.empty());

	// Reset state
	ResetPipelineState(state.pipeline);

	SignalServerEvent(EVT_MODE_DEVICE_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

static void DeviceSupervisorThread(std::stop_token stop_token, ServerState *statePtr)
{
	ServerState &state = *statePtr;

	int it = 0;

	static bool checkingIMU = false;
	TimePoint_t lastContCheck = sclock::now();
	TimePoint_t lastIMUCheck = sclock::now();
	TimePoint_t lastIOCheck = sclock::now();

	while (!stop_token.stop_requested())
	{
		it++;

		TimePoint_t now = sclock::now();

		std::shared_lock dev_lock(state.deviceAccessMutex);

#if !defined(_WIN32)
		// Does work, but takes over a second on windows because no hotplugging support
		if (dtMS(lastContCheck, now) > 100)
		{ // Detect check for new controllers
			//DetectNewControllers(*state);
			lastContCheck = now;
		}
#endif

		// Check for any disconnected controllers
		for (int c = 0; c < state.controllers.size(); c++)
		{
			TrackingControllerState &controller = *state.controllers[c];
			if (!controller.comm->deviceConnected)
			{
				LOG(LControllerDevice, LWarn, "Communication link of controller died!\n");
				// TODO: Disconnect controller asynchronously somehow
				// Why? Does this take long? Not sure anymore, need to check
				DisconnectController(state, controller);
				c--;
				continue;
			}

			HandleController(state, controller);
		}

		// Check for any disconnected cameras (due to both UART / Wifi link being inactive for a period of time)
		for (int c = 0; c < state.cameras.size(); c++)
		{
			TrackingCameraState &camera = *state.cameras[c];
			CameraID id = camera.id;
			if (CameraCheckDisconnected(state, camera))
			{
				LOG(LCameraDevice, LWarn, "Camera %d had no remaining communication links!\n", id);
				c--;
				continue;
			}
			if (!camera.storage.receivedInfo && camera.hasComms() && dtMS(camera.storage.lastFetchTime, sclock::now()) > 500)
			{
				LOG(LCameraDevice, LInfo, "Requesting info from camera #%u", camera.id);
				camera.storage.lastFetchTime = sclock::now();
				camera.sendPacket(PACKET_CAMERA_INFO, nullptr, 0);
			}
		}

		if (dtMS(lastIOCheck, now) > 10)
		{ // Check Tracking IO for low-priority updates
			lastIOCheck = now;
			CheckTrackingIO(state);
		}

		if (!checkingIMU && dtMS(lastIMUCheck, now) > 100)
		{ // Check for new IMU providers
			lastIMUCheck = now;
			checkingIMU = true;
			threadPool.push([](int){
				auto &state = GetState();
				auto providers = *state.imuProviders.contextualLock();
				{ // Fetch new IMU providers
					std::unique_lock hid_lock(state.hid_access);
					detectAsterTrackReceivers(providers);
					detectSlimeVRReceivers(providers);
					initialiseRemoteIMUs(providers);
					// TODO: Add more IMU integrations here
				}
				*state.imuProviders.contextualLock() = providers;
				checkingIMU = false;
			});
		}

		// Fetch IO packets for e.g. Remote IMUs
		FetchTrackingIO(state);

		bool imusRegistered = false;
		int updatedIMUs = 0;
		IMUDeviceList removedIMUs, addedIMUs;
		{ // Fetch new packets from IMUs
			auto imuLock = state.imuProviders.contextualLock();
			for (auto imuProvider = imuLock->begin(); imuProvider != imuLock->end();)
			{
				auto status = imuProvider->get()->poll(updatedIMUs, removedIMUs, addedIMUs);
				if (status == IMU_STATUS_DISCONNECTED)
				{
					imuProvider = imuLock->erase(imuProvider);
					continue;
				}
				if (status == IMU_STATUS_DEVICES_CONNECTED)
					imusRegistered = true;
				imuProvider++;
			}
		}
		if (!addedIMUs.empty() || !removedIMUs.empty())
		{ // Asynchronously register new IMUs
			threadPool.push([](int, IMUDeviceList &addedIMUs, IMUDeviceList &removedIMUs){
				ServerState &state = GetState();
				// In threadPool so we're not blocking (waiting for frame processing) in device supervisor thread
				std::unique_lock pipeline_lock(state.pipeline.pipelineLock);
				for (auto &imuDevice : addedIMUs)
				{
					if (!imuDevice || imuDevice->index >= 0) continue;
					// TODO: Handle receiver replugging, should probably detect IMU as the same
					// Either replace existing (Probably move old samples in to new IMU?)
					// Or just add new and replace any tracking references
					//auto ex = std::find_if(state.pipeline.record.imus.begin(), state.pipeline.record.imus.end(),
					//	[&](auto &i){ return i->id == imu->id; });
					//if (ex != state.pipeline.record.imus.end())
					// Else just add to imu record
					imuDevice->index = state.pipeline.record.imus.size();
					auto imu = std::static_pointer_cast<IMU>(imuDevice);
					state.pipeline.record.imus.push_back(imu); // new shared_ptr
					if (state.isStreaming)
					{
						auto trackerConfig = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
							[&](auto &cfg){ return cfg.imuIdent == imu->id; });
						if (trackerConfig != state.trackerConfigs.end())
						{
							AssociateIMU(state.pipeline, imu, trackerConfig->id, trackerConfig->imuCalib);
							trackerConfig->imu = imu;
						}
						else
						{
							OrphanIMU(state.pipeline, imu); // new shared_ptr
							LOG(LTracking, LInfo, "Added IMU as orphaned tracked IMU!");
						}
					}
				}
				for (auto &imu : removedIMUs)
				{
					// TODO: Handle removal of IMUs - currently just ignored, and trackers using them will just not get any updates, which is fine
				}
				SignalPipelineUpdate();
			}, std::move(addedIMUs), std::move(removedIMUs));
		}

		if (state.usePacketQueue)
		{
			for (int c = 0; c < state.controllers.size(); c++)
			{ // Parse new packets from controllers
				TrackingControllerState &controller = *state.controllers[c];
				if (controller.comm->deviceConnected)
					ParseControllerPackets(state, controller);
			}
		}

		if (state.isStreaming)
		{ // Frame consistency supervision and processing stream frames once deemed complete
			auto stream_lock = state.stream.contextualLock();
			bool startedProcessing = false;
			if (state.usePacketQueue || dtMS(stream_lock->lastMaintainTime, now) > 5)
			{ // Maintain stream state only if we've not already called MaintainStreamState with a recent USB packet
				startedProcessing = MaintainStreamState(*stream_lock);
			}
			if (!startedProcessing && state.lowLatencyIMU && updatedIMUs > 0)
			{ // Notify realtime processing thread of new IMU samples at least
				state.processing_cv.notify_one();
			}
		}

		if (!state.isStreaming)
		{ // Update status of background processes even when not streaming (anymore)
			UpdatePipelineStatus(state.pipeline);
		}

		dev_lock.unlock();

		// Interval only affects IMU polling rate, not USB packets by controllers
		int interval = imusRegistered? (state.lowLatencyIMU? 1 : 1) : 20;
		TimePoint_t wakeup = sclock::now() + std::chrono::milliseconds(interval);
		std::unique_lock parsing_lock(state.parsing_m);
		state.parsing_cv.wait_until(parsing_lock, wakeup);
	}
}

static void RealtimeProcessingThread(std::stop_token stop_token, ServerState *statePtr)
{
	ServerState &state = *statePtr;
	PipelineState &pipeline = state.pipeline;

	while (!stop_token.stop_requested())
	{
		auto frames = pipeline.record.frames.getView();
		if (frames.empty())
		{}
		else if (pipeline.frameNum+1 < frames.endIndex())
		{ // Process most recent fully-received frame (may skip frames!)
			std::shared_ptr<FrameRecord> frame;
			if (pipeline.frameNum+3 < frames.endIndex())
				frame = frames.back(); // new shared_ptr
			else // Process realtime unless 2 more frames are already waiting, then skip 2
				frame = frames[pipeline.frameNum+1];

			auto start = sclock::now();
			ProcessFrame(pipeline, frames.back()); // new shared_ptr
			auto end = sclock::now();

			if (state.lowLatencyIMU)
			{
				// TODO: Integrate IMU samples ontop of latest frame for lower latency (2/3)
				// Then PushTrackingIO with updated tracking results
			}

			PushTrackingIO(state, frame);

			SignalCameraRefresh(0);
		}
		else if (state.lowLatencyIMU)
		{
			// TODO: Integrate IMU samples ontop of latest frame for lower latency (3/3)
			// Then PushTrackingIO without full frame

			SignalCameraRefresh(0);
		}

		// Can wait long, getting notified on any new frame and IMU sample
		TimePoint_t wakeup = sclock::now() + std::chrono::milliseconds(100);
		std::unique_lock processing_lock(state.processing_m);
		state.processing_cv.wait_until(processing_lock, wakeup);
	}
}

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
			EnsureCamera(state, camDef.id)->label = asprintf_s("Camera %d", camDef.id);
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
		simLock->blobPxStdDev = state.config.simulation.blobPxStdDev;
		simLock->objects.clear();
		simLock->resetState();
	}

	// Add testing targets for which there isn't already a calibration
	for (auto simTarget : state.config.simulation.trackingTargets)
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

			TrackerConfig tracker(id, label, TargetCalibration3D(calib), TargetDetectionConfig());
			tracker.isSimulated = true;
			state.trackerConfigs.push_back(std::move(tracker));
		}

		state.pipeline.simulation.contextualLock()->objects.push_back(
			SimulatedObject { .id = id, .label = label, .target = calib, .motionPreset = 2, .motion = motionPresets[2] }
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
	state.coprocessingThread = new std::jthread(SimulationThread, &state);

	SignalServerEvent(EVT_MODE_SIMULATION_START);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

void StopSimulation(ServerState &state)
{
	if (state.mode != MODE_Simulation)
		return;

	StopStreaming(state);

	// Join coprocessing thread
	state.coprocessingThread->request_stop();
	state.simAdvance = -1;
	state.simAdvance.notify_all();
	delete state.coprocessingThread;
	state.coprocessingThread = NULL;

	std::scoped_lock dev_lock(state.deviceAccessMutex, state.pipeline.pipelineLock);

	// Reset state
	state.mode = MODE_None;
	state.cameras.clear();
	ResetPipelineState(state.pipeline);

	SignalServerEvent(EVT_MODE_SIMULATION_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

static void SimulationThread(std::stop_token stop_token, ServerState *statePtr)
{
	ServerState &state = *statePtr;
	PipelineState &pipeline = state.pipeline;

	while (!stop_token.stop_requested())
	{
		if (!state.isStreaming)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			UpdatePipelineStatus(pipeline);
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
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

			auto framesStored = state.stored.frames.getView();
			if (frame < framesStored.endIndex() && framesStored[frame])
			{
				frameRecord = std::make_shared<FrameRecord>();
				auto &loadedRecord = framesStored[frame];
				assert(loadedRecord->num == frame);
				frameRecord->num = loadedRecord->num;
				frameRecord->ID = loadedRecord->ID;
				frameRecord->time = state.recording.replayTime + (loadedRecord->time - framesStored.front()->time);
				frameRecord->cameras = loadedRecord->cameras;
				if (frame+1 < framesStored.endIndex())
				{ // Replicate original frame pacing
					auto &nextRecord = framesStored[frame+1];
					if (nextRecord)
						desiredFrameIntervalUS = std::chrono::duration_cast<std::chrono::microseconds>(nextRecord->time - loadedRecord->time).count();
					// TODO: Set desired frame end time to better stick to frame time, otherwise replay quickly gets out of sync and VRPN clients will be unhappy due to apparent high latency
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
						cam->receiving.latestFrameImage = std::move(image);
						cam->receiving.latestFrameImageRecord = std::move(imageRecord);

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
			CheckTrackingIO(state);
			//FetchTrackingIO(*state);

			ProcessFrame(pipeline, frameRecord); // new shared_ptr

			PushTrackingIO(state, frameRecord);

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

		if (state.simAdvanceQuickly)
			std::this_thread::sleep_for(std::chrono::microseconds(500));
		else
			std::this_thread::sleep_until(frameReceiveTime + std::chrono::microseconds(desiredFrameIntervalUS));

		// Waiting for processing of last frame to finish
		//while (threadPool.n_idle() != threadPool.size())
		//	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
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
	state.coprocessingThread = new std::jthread(SimulationThread, &state);

	SignalServerEvent(EVT_MODE_SIMULATION_START);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

void StopReplay(ServerState &state)
{
	if (state.mode != MODE_Replay)
		return;
	// Essentially same as StopSimulation

	StopStreaming(state);

	// Join coprocessing thread
	state.coprocessingThread->request_stop();
	state.simAdvance = -1;
	state.simAdvance.notify_all();
	delete state.coprocessingThread;
	state.coprocessingThread = NULL;

	std::scoped_lock dev_lock(state.deviceAccessMutex, state.pipeline.pipelineLock);

	// Reset state
	state.mode = MODE_None;
	state.cameras.clear();
	ResetPipelineState(state.pipeline);
	state.stored.frames.cull_clear();
	state.stored.imus.clear();
	state.recording = {};
	state.stored.frames.delete_culled();

	SignalServerEvent(EVT_MODE_SIMULATION_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
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

	// Initialise state
	InitPipelineStreaming(state.pipeline);
	state.isStreaming = true;

	// Setup trackers
	for (auto &tracker : state.trackerConfigs)
	{
		ServerUpdatedTrackerConfig(state, tracker);
	}

	if (state.mode == MODE_Device)
	{
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
		DevicesStopStreaming(state);

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
	{
		tracker.triggered = false;
		tracker.tracked = false;
	}
}

void ProcessStreamFrame(SyncGroup &sync, SyncedFrame &frame, bool premature)
{
	// Frame arrived, possibly incomplete or out-of-date, consider for realtime processing

	if (!premature)
	{ // Implies this if the final time this frame is processed
		// Also the only time it's processed if all cameras sent streaming packets on time


		if (frame.outdated)
		{ // Implies future frames were already complete and fully processed
			// Should always be true if this frame is not complete and only finally processed after a long time
			LOG(LStreaming, LDebug, "--------- Frame ID %d (%d) final processing is outdated!\n", frame.ID, frame.ID&0xFF);
		}

		if (frame.previouslyProcessed && frame.completed == frame.expecting)
		{
			LOG(LStreaming, LDebug, "Finally completing processing of frame ID %d (%d) after %.2fms!\n",
				frame.ID, frame.ID&0xFF, dtMS(frame.SOF, sclock::now()));
		}
		else if (frame.completed != frame.expecting)
		{
			LOG(LStreaming, LDebug, "Finally discarding frame ID %d (%d) after %.2fms!\n",
				frame.ID, frame.ID&0xFF, dtMS(frame.SOF, sclock::now()));
		}
	}

	if (frame.previouslyProcessed)
	{ // Realtime processing currently cannot handle the same frame twice
		// Optimally, we could retrace and improve tracking with new data
		// Especially if tracking was lost

		// TODO: Record into frameRecords anyway for recording
		return;
	}
	// Follow up with realtime processing
	if (frame.outdated)
	{ // Should not happen because it should have gotten a premature processing
		// Except at the very beginning where premature processing conditions have not yet initialised
		if (frame.ID > 50)
			LOG(LStreaming, LDarn, "--------- Frame ID %d (%d) processing is outdated!\n", frame.ID, frame.ID&0xFF);
		// TODO: If latency stdDev is ridiculously large, this might be true even later
		// for frames that had missing packets and were only prematurely processed very late
		return;
	}

	if (premature)
	{ // Implies incomplete, WILL be called another time, either once complete or when discarded
		LOG(LStreaming, LDarn, "Frame %d (%d) has %d/%d cameras completed when prematurely processed!\n", frame.ID, frame.ID&0xFF, frame.completed, frame.expecting);
		for (auto &camera : sync.cameras)
		{
			if (!camera) continue; // Removed while streaming
			if (frame.cameras.size() <= camera->syncIndex) continue; // Newly added after frame
			// - Non-exclusive, might have been a vacant spot, but not an issue
			auto &camRecv = frame.cameras[camera->syncIndex];
			if (!camRecv.announced) continue;
			if (camRecv.erroneous || !camRecv.complete)
			{ // If packet was fully received
				LOG(LStreaming, LTrace, "    Camera %d packet is %s, status %s\n",
					camera->id, camRecv.complete? "complete" : "incomplete", camRecv.erroneous? "error" : "good");
			}
			else
			{
				LOG(LStreaming, LTrace, "    Camera %d has not received any packet despite being announced!\n", camera->id);
			}
		}
	}

	PipelineState &pipeline = GetState().pipeline;

	// Accept for realtime processing, create FrameState

	std::shared_ptr<FrameRecord> frameRecord = std::make_shared<FrameRecord>();
	frameRecord->cameras.resize(pipeline.cameras.size());
	for (auto &camera : sync.cameras)
	{ // Setup camera data
		if (!camera) continue; // Removed while streaming - have to ignore even if data was valid
		if (frame.cameras.size() <= camera->syncIndex) continue; // Newly added after frame
		// - Non-exclusive, might have been a vacant spot, but not an issue
		auto &camRecv = frame.cameras[camera->syncIndex];
		if (!camRecv.announced)
		{
			LOG(LStreaming, LDebug, "--- Camera %d has no blob data announced for frame %d!\n", camera->id, frame.ID);
			continue;
		}
		if (!camRecv.complete)
		{
			LOG(LStreaming, LDarn, "--- Camera %d has incomplete blob data for frame %d!\n", camera->id, frame.ID);
			continue;
		}
		if (camRecv.erroneous)
		{
			LOG(LStreaming, LDarn, "--- Camera %d has erroneous blob data for frame %d!\n", camera->id, frame.ID);
			continue;
		}
		// Packet was properly received
		frameRecord->cameras[camera->pipeline->index] = std::move(camRecv.record);
	}
	for (auto &camera : sync.cameras)
	{ // Copy over image data received before processing started
		if (!camera) continue; // Removed while streaming - have to ignore even if data was valid
		if (camera->receiving.latestFrameImageRecord && camera->receiving.latestFrameImageRecord->frameID == frame.ID)
		{ // May happen if frame processing was unreasonably delayed
			frameRecord->cameras[camera->pipeline->index].image = camera->receiving.latestFrameImageRecord;
		}
	}
	frameRecord->time = frame.SOF;
	frameRecord->ID = frame.ID;
	// Insert frame into record in the correct order
	auto frames = pipeline.record.frames.getView();
	if (frames.empty() || frames.back()->time < frame.SOF)
		frameRecord->num = pipeline.record.frames.push_back(frameRecord); // new shared_ptr
	else
	{
		// TODO: Ensure frames are processed/appended to frame record in chronological order even between sync groups
		// This is also complicated IF we allow frame IDs to be arbitrary or duplicate between sync groups
		// See note on future frameID plans in record.hpp - space for frames should be pre-determined
		LOG(LStreaming, LError, "Dropped frame %d because frame %d from %.2fms later was already recorded - cannot insert in order!",
			frame.ID, frames.back()->ID, dtMS(frame.SOF, frames.back()->time));
	}

	GetState().processing_cv.notify_one();
}


// ----------------------------------------------------------------------------
// Tracking Data IO
// ----------------------------------------------------------------------------

void SetupIO(ServerState &state)
{
	auto io_lock = std::unique_lock(state.io.mutex);
	state.io.useVRPN = true;
	// Setup VRPN Connection
	std::string connectionName = ":" + std::to_string(vrpn_DEFAULT_LISTEN_PORT_NO);
	state.io.vrpn_server = opaque_ptr<vrpn_Connection>(vrpn_create_server_connection(connectionName.c_str()));
	state.io.vrpn_server->setAutoDeleteStatus(true);
}

void ResetIO(ServerState &state)
{
	auto io_lock = std::unique_lock(state.io.mutex);
	state.io.vrpn_trackers.clear();
	state.io.vrpn_server = nullptr;
	state.io.useVRPN = false;

	for (auto &tracker : state.trackerConfigs)
	{
		tracker.exposed = false;
		tracker.connected = false;
	}
}

void CheckTrackingIO(ServerState &state)
{
	auto io_lock = std::unique_lock(state.io.mutex);
	if (!state.io.vrpn_server) return;

	for (auto &tracker : state.trackerConfigs)
	{
		if (!tracker.exposed)
		{ // Check expose conditions
			if (tracker.expose == TrackerConfig::EXPOSE_ALWAYS)
				tracker.exposed = true;
			else if (tracker.expose == TrackerConfig::EXPOSE_ONCE_TRIGGERED && tracker.triggered)
				tracker.exposed = true;
			else if (tracker.expose == TrackerConfig::EXPOSE_ONCE_TRACKED && tracker.tracked)
				tracker.exposed = true;
			else
			 	continue;
		}
		auto io_tracker = state.io.vrpn_trackers.find(tracker.id);
		if (io_tracker == state.io.vrpn_trackers.end())
		{
			std::string path = tracker.label;
			//std::string path = asprintf_s("AsterTarget_%.4d", tracker.id);
			LOG(LIO, LInfo, "Exposing VRPN Tracker '%s'", path.c_str());
			auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(tracker.id, path.c_str(), state.io.vrpn_server.get());
			io_tracker = state.io.vrpn_trackers.insert({ tracker.id, std::move(vrpn_tracker) }).first;
		}
		if (!tracker.connected && io_tracker->second->isConnected())
		{
			LOG(LIO, LInfo, "VRPN Tracker '%s' has been connected!", tracker.label.c_str());
			tracker.connected = true;
			ServerUpdatedTrackerConfig(state, tracker);
		}
		else if (tracker.connected && !io_tracker->second->isConnected())
		{
			LOG(LIO, LInfo, "VRPN Tracker '%s' has been disconnected!", tracker.label.c_str());
			tracker.connected = false;
			ServerUpdatedTrackerConfig(state, tracker);
		}
	}

	// Add cameras as trackers to give clients an opportunity to display them as references
	// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
	for (auto &camera : state.pipeline.cameras)
	{
		auto io_tracker = state.io.vrpn_trackers.find(camera->id);
		if (io_tracker == state.io.vrpn_trackers.end())
		{
			std::string path = asprintf_s("AsterCamera_%d", camera->index);
			auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(camera->id, path.c_str(), state.io.vrpn_server.get());
			io_tracker = state.io.vrpn_trackers.insert({ camera->id, std::move(vrpn_tracker) }).first;
		}
	}

	LOG(LIO, LTrace, "Updating individual trackers!\n");
	for (auto &trackerIO : state.io.vrpn_trackers)
	{ // Handle mainloop (mostly ping-pong) here as well
		trackerIO.second->mainloop();
	}
}

void FetchTrackingIO(ServerState &state)
{
	auto io_lock = std::unique_lock(state.io.mutex);
	if (state.io.vrpn_server)
	{
		// Fetch incoming packets
		LOG(LIO, LTrace, "Updating server connection to fetch IMU packets!\n");
		state.io.vrpn_server->mainloop();
		if (!state.io.vrpn_server->doing_okay())
		{
			LOG(LIO, LWarn, "VRPN Connection Error!\n");
		}
		else
			LOG(LIO, LTrace, "     Server connection is doing ok!\n");
	}
}

void PushTrackingIO(ServerState &state, std::shared_ptr<FrameRecord> &frame)
{
	auto io_lock = std::unique_lock(state.io.mutex);
	if (state.io.vrpn_server)
	{
		for (auto &trackRecord : frame->trackers)
		{
			auto io_tracker = state.io.vrpn_trackers.find(trackRecord.id);
			if (io_tracker == state.io.vrpn_trackers.end()) continue;
			// TODO: Send both poseObserved and poseFiltered?
			io_tracker->second->updatePose(0, frame->time, trackRecord.poseFiltered);
		}

		for (auto &camera : state.pipeline.cameras)
		{
			auto io_tracker = state.io.vrpn_trackers.find(camera->id);
			if (io_tracker == state.io.vrpn_trackers.end()) continue;
			// Send static position of cameras as reference for clients
			// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
			io_tracker->second->updatePose(0, frame->time, camera->calib.transform.cast<float>());
		}

		LOG(LIO, LTrace, "Updating server connection to push tracker packets!\n");
		state.io.vrpn_server->send_pending_reports();
		if (!state.io.vrpn_server->doing_okay())
		{
			LOG(LIO, LWarn, "VRPN Connection Error!\n");
		}
		else
			LOG(LIO, LTrace, "     Server connection is doing ok!\n");
	}
}