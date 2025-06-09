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

#include "ui/shared.hpp" // Signals

#include "device/tracking_controller.hpp"
#include "device/tracking_camera.hpp"
#include "device/parsing.hpp"

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

#include "hidapi/hidapi.h"

#define USB_PACKET_QUEUE // Keep USB thread free for timing by queuing packets for device thread to parse

/**
 * Main program flow
 */

// Processing
ctpl::thread_pool threadPool = ctpl::thread_pool(6);

// util/debugging.hpp
std::atomic<bool> dbg_isBreaking;
std::atomic<int> dbg_debugging;


/* Functions */

static void DeviceSupervisorThread(std::stop_token stop_token, ServerState *state);
static void SimulationThread(std::stop_token stop_token, ServerState *state);

bool ServerInit(ServerState &state)
{
	state.libusb_context = comm_init_context();

	// Read configurations
	parseGeneralConfigFile("store/general_config.json", state.config);
	parseCameraConfigFile("store/camera_config.json", state.cameraConfig);

	// Load calibrations
	parseCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
	parseTargetCalibrations("store/target_calib.json", state.pipeline.tracking.targetCalibrations);

	// Load IMU configs & calibrations
	parseIMUConfigs("store/imu_config.json", *imuConfigs.contextualLock());

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

	//srand(1111110);
	srand(time(0)); // Sufficient to get new random values for Eigen::Random every time

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
}

void ServerStoreCameraCalib(ServerState &state)
{
	// Update list of calibrations with current set of cameras
	for (auto &cam : state.pipeline.cameras)
	{
		int i;
		for (i = 0; i < state.cameraCalibrations.size(); i++)
		{
			if (state.cameraCalibrations[i].id == cam->id)
			{ // Found calibration entry, update calibration data
				state.cameraCalibrations[i] = cam->calib;
				break;
			}
		}
		if (i >= state.cameraCalibrations.size())
		{ // New camera with calibration, add as calibration entry
			state.cameraCalibrations.push_back(cam->calib);
		}
	}

	// Write both as current calibration
	storeCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
}

void ServerStoreTargetCalib(ServerState &state)
{
	// Collect current set of stored target templates
	std::vector<TargetCalibration3D> targetCalibrations;
	targetCalibrations.reserve(state.pipeline.tracking.targetCalibrations.size());
	for (int i = 0; i < state.pipeline.tracking.targetCalibrations.size(); i++)
	{ // TODO: Differentiate simulated targets some better way?
		// Or just store them separately in server like is planned anyway, and only combine in pipeline?
		if (state.pipeline.tracking.targetCalibrations[i].id >= 0)
			targetCalibrations.push_back(state.pipeline.tracking.targetCalibrations[i]);
	}

	// Write both as current calibration
	storeTargetCalibrations("store/target_calib.json", targetCalibrations);
}

void ServerStoreIMUConfig(ServerState &state)
{
	// Update list of IMU configs with current set of IMUs
	auto configs_lock = imuConfigs.contextualLock();
	auto &configs = *configs_lock;
	for (auto &imu : state.pipeline.record.imus)
	{
		int i;
		for (i = 0; i < configs.size(); i++)
		{
			if (configs[i].id == imu->id)
			{ // Found calibration entry, update calibration data
				configs[i].tracker = imu->tracker;
				break;
			}
		}
		if (i >= configs.size())
		{ // Add as new IMU
			configs.emplace_back(imu->id, imu->tracker);
		}
	}

	// Write both as current calibration
	storeIMUConfigs("store/imu_config.json", configs);
}

void ServerStoreConfiguration(ServerState &state)
{
	storeCameraConfigFile("store/camera_config.json", state.cameraConfig);
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

static std::shared_ptr<CameraPipeline> EnsureCameraPipeline(ServerState &state, int id)
{
	auto cam = std::find_if(state.pipeline.cameras.begin(), state.pipeline.cameras.end(), [id](const auto &c) { return c->id == id; });
	if (cam != state.pipeline.cameras.end())
		return *cam;
	std::unique_lock pipeline_lock(state.pipeline.pipelineLock); // cameras

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
		LOG(LDefault, LInfo, "No calibration found for camera %d!\n", camera->id);
	}

	return camera;
}

std::shared_ptr<TrackingCameraState> EnsureCamera(ServerState &state, CameraID id)
{
	auto cam = std::find_if(state.cameras.begin(), state.cameras.end(), [id](const auto &c) { return c->id == id; });
	if (cam != state.cameras.end())
		return *cam;
	std::unique_lock dev_lock(state.deviceAccessMutex); // cameras
	std::shared_ptr<TrackingCameraState> camera = std::make_shared<TrackingCameraState>();
	camera->id = id;
	camera->pipeline = EnsureCameraPipeline(state, id);
	state.cameras.push_back(camera); // new shared_ptr
	// TODO: Setup sync groups in EnsureCamera (based on prior config, e.g. in UI) 1/3
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

// TODO: Reevaluate need for wifi connection to cameras and potentially re-enable server
// It did work at some point, and setting up wifi on cameras on-demand from server is working now
// Would allow to transfer larger amounts of data like full quality images that are less latency-critical but require large bandwidth
// Of course, might be possible to have the cameras only connect through wifi, but I fail to see the use for now
// If going fully wireless, better use a designated protocol for it like nrf24s have
// TimeSync over wireless would be horrible

/* static void StartDeviceServer(ServerState &state)
{
	if (state.server.threadRun)
		return;
	LOG(LServer, LInfo, "Starting network server!\n");
	// Init server
	state.server.socket = ServerInit("8888");
	if (state.server.socket == 0)
	{
		LOG(LServer, LError, "Could not start network server!\n");
		return;
	}
	state.server.callbacks.userData1 = &state;
	state.server.callbacks.onIdentify = [](ClientCommState &client) { // Find or setup camera
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		std::shared_ptr<TrackingCameraState> camera = DeviceSetupCamera(state, client.otherIdent.id);
		LOG(LServer, LInfo, "Established server connection to camera with id %d!\n", camera->id);
		// Setup stream subsystem
		if (!camera->sync)
		{ // Init with internal sync if no controller is known to be connected
			SetCameraSyncNone(state.stream, camera);
		}
		// Setup connection
		camera->client = &client;
		client.callbacks.userData2 = camera;
		SignalServerEvent(EVT_UPDATE_CAMERAS);
	};
	state.server.callbacks.onDisconnect = [](ClientCommState &client) { // Remove camera
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		if (client.callbacks.userData2)
		{ // Camera was already identified
			TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
			camera.client = NULL;
			if (DeviceCheckCameraDisconnect(state, camera))
			{
				LOG(LServer, LWarn, "Camera with id %d lost server connection, now unreachable! Total %d cameras connected!\n",
					camera.id, (int)state.cameras.size());
			}
			else
			{
				LOG(LServer, LWarn, "Camera with id %d lost server connection!\n", camera.id);
			}
		}
		else
		{
			LOG(LServer, LWarn, "Lost connection to camera before it could be identified!\n");
		}

	};
	state.server.callbacks.onReceivePacketHeader = [](ClientCommState &client, PacketHeader &header)
	{
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		std::shared_lock dev_lock(state.deviceAccessMutex);
		if (header.tag == PACKET_FRAME_SIGNAL)
		{ // Frame is starting to be processed
			// TODO: This is only for cameras solely connected over wifi. Unimportant, should outright remove TimeSync here
			if (camera.sync && camera.sync->source == SYNC_NONE)
			{ // No sync, this is only source for SOF timing, estimate actual SOF
				LOG(LStreaming, LTrace, "Camera %d announced packet for frame %d!\n", camera.id, header.frameID);
				// Update estimate of SOF (assuming regular SOFs)
				TimePoint_t timeSOF = sclock::now() - std::chrono::microseconds(8000);
				//timeSOF = UpdateSOFPredictions(*camera.sync, header.frameID, timeSOF);
				RegisterSOF(*camera.sync, header.frameID, timeSOF);
				RegisterCameraFrame(camera, header.frameID);
			}
			return true;
		}
		auto parse_lock = camera.parse.contextualLock();
		// Check if a packet of same tag is currently being parsed
		for (auto &recv : parse_lock->receiving)
		{
			if (recv.second->header.tag == header.tag)
			{
				LOG(LStreaming, LDebug, "Server received new packet of tag %d (%d) before last (%d) finished parsing with %d/%d bytes read\n",
					header.tag, header.frameID, recv.second->header.frameID, (int)recv.second->received, recv.second->header.length);
			}
		}
		int headerBlockID = header.frameID * header.tag;
		std::shared_ptr<ParsePacket> parsePacket = RegisterPacketHeader(*parse_lock, header, headerBlockID);
		if (header.isStreamPacket())
		{ // Register that camera is receiving frame data
			RegisterStreamPacket(camera, header.frameID, sclock::now(), parsePacket);
		}
		return true;
	};
	state.server.callbacks.onReceivePacketBlock = [](ClientCommState &client, PacketHeader &header, uint8_t *data, unsigned int len)
	{
		ServerState &state = *((ServerState*)client.callbacks.userData1);
		TrackingCameraState &camera = *std::static_pointer_cast<TrackingCameraState>(client.callbacks.userData2);
		std::shared_lock dev_lock(state.deviceAccessMutex);
		// Update statistics of streaming packet
		if (header.isStreamPacket())
		{
			if (!RegisterStreamBlock(camera, header.frameID))
			{
				LOG(LParsing, LError, "---- Received stream block for non-existant frame %d or unregistered stream packet!\n", header.frameID);
			}
		}
		// Continue parsing packet
		int headerBlockID = header.frameID * header.tag;
		auto parse_lock = camera.parse.contextualLock();
		std::shared_ptr<ParsePacket> parsePacket = ParsePacketBlock(*parse_lock, data, len, headerBlockID);
		if (parsePacket && parsePacket->complete)
		{ // Successfully read block
			if (parsePacket->erroneous)
			{
				LOG(LStreaming, LDebug, "---- Failed to read packet with tag %d for frame %d!\n", header.tag, header.frameID);
			}
			if (header.isStreamPacket())
			{
				LOG(LParsing, LTrace, "Parsed final block of streaming packet with tag %d\n", header.tag);
				RegisterStreamPacketComplete(camera, *parse_lock, header.frameID, false);
				MaintainStreamState(state.stream);
			}
			else
			{
				LOG(LParsing, LTrace, "Parsed final block of general packet with tag %d\n", header.tag);
				ReadCameraPacket(camera, *parsePacket);
			}
			ClearPacketBlock(*parse_lock, headerBlockID);
			// FrameRecord might still reference it if isStreamPacket, but that's fine since it's a shared_ptr
		}
	};
	// Start comm thread
	state.server.threadRun = true;
	state.server.thread = new std::thread(ServerThread, &state.server);
}

static void StopDeviceServer(ServerState &state)
{
	if (!state.server.threadRun)
		return;
	LOG(LServer, LInfo, "Stopping network server!\n");
	// Join server thread, it will do cleanup
	state.server.threadRun = false;
	if (state.server.thread != NULL && state.server.thread->joinable())
		state.server.thread->join();
	state.server.thread = NULL;
} */

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

	//StartDeviceServer(state);

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
	LOG(LDefault, LInfo, "Disconnecting!\n");

	// Join coprocessing thread
	delete state.coprocessingThread;
	state.coprocessingThread = NULL;

	// Stop streaming
	if (state.isStreaming)
		StopStreaming(state);

	// Disconnect from controllers
	while (!state.controllers.empty())
		DisconnectController(state, *state.controllers.back());

	// Stop server and disconnect from cameras
	//StopDeviceServer(state);

	// Diconnect IMU providers
	state.imuProviders.contextualLock()->clear();
	hid_exit();

	// Reset state
	state.mode = MODE_None;
	assert(state.controllers.empty() && state.cameras.empty());
	ResetPipelineState(state.pipeline);

	SignalServerEvent(EVT_MODE_DEVICE_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

static void DeviceSupervisorThread(std::stop_token stop_token, ServerState *state)
{
	int it = 0;

	static bool checkingIMU = false;
	TimePoint_t lastIMUCheck = sclock::now();

	while (!stop_token.stop_requested())
	{
		it++;

#if !defined(_WIN32)
		// Does work, but takes over a second on windows because no hotplugging support
		if (it % 100 == 0)
		{ // Detect new controllers
			//DetectNewControllers(*state);
		}
#endif

		// Check comm with every controller
		for (int c = 0; c < state->controllers.size(); c++)
		{
			TrackingControllerState &controller = *state->controllers[c];
			if (!controller.comm->deviceConnected)
			{
				LOG(LControllerDevice, LWarn, "Communication link of controller died!\n");
				// TODO: Disconnect controller asynchronously somehow
				// Why? Does this take long? Not sure anymore, need to check
				DisconnectController(*state, controller);
				c--;
				continue;
			}

			HandleController(*state, controller);
		}

		if (!checkingIMU && dtMS(lastIMUCheck, sclock::now()) > 500)
		{ // Regularly check for new IMU providers
			checkingIMU = true;
			lastIMUCheck = sclock::now();
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

		bool imusRegistered = false, imusUpdated = false, imusChanged = false;
		{ // Fetch new state from IMUs
			auto imuLock = state->imuProviders.contextualLock();
			imusRegistered = !imuLock->empty();
			for (auto imuProvider = imuLock->begin(); imuProvider != imuLock->end();)
			{
				assert(*imuProvider);
				// TODO: Fix seg fault here after re-entering device mode once
				int updatedDevices = 0, changedDevices = 0;
				if (imuProvider->get()->poll(updatedDevices, changedDevices) == IMU_STATUS_DISCONNECTED)
				{
					imusChanged = true;
					imuProvider = imuLock->erase(imuProvider);
					continue;
				}
				if (updatedDevices > 0) imusUpdated = true;
				if (changedDevices > 0)
				{
					imusChanged = true;
					std::unique_lock pipeline_lock(state->pipeline.pipelineLock);
					for (auto &imu : imuProvider->get()->devices)
					{
						if (!imu || imu->index >= 0) continue;
						// TODO: Handle receiver replugging, should probably detect IMU as the same
						// Either replace existing (Probably move old samples in to new IMU?)
						// Or just add new and replace any tracking references
						//auto ex = std::find_if(state->pipeline.record.imus.begin(), state->pipeline.record.imus.end(),
						//	[&](auto &i){ return i->driver == imu->driver && i->device == imu->device; });
						//if (ex != state->pipeline.record.imus.end())
						// Else just add to imu record
						imu->index = state->pipeline.record.imus.size();
						state->pipeline.record.imus.push_back(std::static_pointer_cast<IMU>(imu));
						if (state->isStreaming)
						{
							// Add as individual tracker first until assigned to a tracker
							state->pipeline.tracking.orphanedIMUs.emplace_back(std::static_pointer_cast<IMU>(imu), state->pipeline.params.track);
							LOG(LTracking, LInfo, "Added IMU as orphaned tracked IMU!");
						}
					}
				}
				imuProvider++;
			}
		}
		if (imusUpdated)
			SignalPipelineUpdate();
		if (imusChanged)
		{ // TODO: Handle disconnected IMUs?
		}

		// Frame consistency supervision and processing stream frames once deemed complete
		TimePoint_t s0 = sclock::now();
		if (state->isStreaming)
			MaintainStreamState(*state->stream.contextualLock());
		TimePoint_t s1 = sclock::now();
		if (dtUS(s0, s1) > 2000)
		{
			LOG(LParsing, LDarn, "Maintaining stream state took %.2fms!",
				dtMS(s0,s1));
		}

		// TODO: Consider pushing new state from IMUs to IO if no frame was finished in a bit

		if (!state->isStreaming && !imusRegistered)
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		else if (!state->isStreaming)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
#ifndef USB_PACKET_QUEUE
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
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

	// Setup cameras
	for (int c = 0; c < state.config.simulation.cameraDefinitions.size(); c++)
	{
		auto camDef = state.config.simulation.cameraDefinitions[c];
		EnsureCamera(state, camDef.id)->label = asprintf_s("Camera %d", camDef.id);
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
	auto &targets = state.pipeline.tracking.targetCalibrations;
	for (int i = 0; i < state.config.simulation.trackingTargets.size(); i++)
	{
		const TargetCalibration3D &targetDef = state.config.simulation.trackingTargets[i];

		state.pipeline.simulation.contextualLock()->objects.push_back(
			SimulatedObject { .target = &targetDef, .motionPreset = 2, .motion = motionPresets[2] }
		);

		// Check if this target has been calibrated yet
		auto existingCalib = std::find_if(targets.begin(), targets.end(), [targetDef](auto &t)
										  { return t.id == targetDef.id; });
		if (existingCalib != targets.end())
		{ // Determine offset of calibrated target template compared to ground truth
			LOG(LDefault, LInfo, "Using calibrated target template %d for simulated target %s (%d)!\n",
				existingCalib->id, targetDef.label.c_str(), targetDef.id);

			// Set ground truth target as point cloud
			std::vector<TriangulatedPoint> triPoints;
			triPoints.reserve(targetDef.markers.size());
			for (const auto &marker : targetDef.markers)
				triPoints.emplace_back(marker.pos, state.pipeline.params.tri.maxIntersectError, state.pipeline.params.tri.minIntersectionConfidence);
			std::vector<int> triIndices(triPoints.size());
			std::iota(triIndices.begin(), triIndices.end(), 0);

			// Detect match using calibrated target
			auto cand = detectTarget3D(*existingCalib, triPoints, triIndices, state.pipeline.params.detect.tri.sigmaError, state.pipeline.params.detect.tri.poseSigmaError, false);
			if (cand.points.size() > 0)
			{ // Read out offset transform and correct for it to get accurate error calculations
				for (auto &marker : existingCalib->markers)
				{
					marker.pos = cand.pose * marker.pos;
					marker.nrm = cand.pose.rotation() * marker.nrm;
				}
				LOG(LDefault, LDebug, "Correcting for %.4fmm calibration offset\n", cand.pose.translation().norm() * 10);
			}

			// No need to re-generate lookup tables for target detection
		}
		else
		{
			LOG(LDefault, LInfo, "Using ground truth target template for simulated target %s (%d) with %d points!\n",
				targetDef.label.c_str(), targetDef.id, (int)targetDef.markers.size());

			// Add ground truth as target template
			targets.push_back(targetDef);
		}
	}

	{ // Align calibrations to simulated calibrations
		std::vector<CameraCalib> calibs;
		for (auto &cam : state.pipeline.cameras)
			calibs.push_back(cam->calib);
		AlignWithGT(state.pipeline, calibs);
		for (int c = 0; c < calibs.size(); c++)
			state.pipeline.cameras[c]->calib = calibs[c];
		SignalServerEvent(EVT_UPDATE_CALIBS);
	}

	// Debug calibrations
	for (auto &cam : state.pipeline.cameras)
	{
		const CameraCalib &tCam = cam->simulation.calib;
		const CameraCalib &cCam = cam->calib;
		Eigen::Vector3d posGT = tCam.transform.translation();
		Eigen::Vector3d rotGT = getEulerXYZ(tCam.transform.rotation()) / PI * 180;
		LOG(LDefault, LTrace, "Cam %d testing transform: Pos/m (%.3f, %.3f, %.3f), Rot/\u00B0 (%.2f, %.2f, %.2f)\n",
			  cam->id, posGT.x(), posGT.y(), posGT.z(), rotGT.x(), rotGT.y(), rotGT.z());
		if (cCam.transform.translation().sum() != 0)
		{
			Eigen::Vector3d posCB = cCam.transform.translation();
			Eigen::Vector3d rotCB = getEulerXYZ(cCam.transform.rotation()) / PI * 180;
			Eigen::Vector3d tDiff = cCam.transform.translation() - tCam.transform.translation();
			Eigen::Matrix3d rDiff = tCam.transform.rotation() * cCam.transform.rotation().transpose();
			double tError = tDiff.norm(), rError = Eigen::AngleAxis<CVScalar>(rDiff).angle() / PI * 180;
			LOG(LDefault, LTrace, "Cam %d calibrated transform: Pos/m (%.3f, %.3f, %.3f), Rot/\u00B0 (%.2f, %.2f, %.2f), Error: (%.4fmm, %.4f\u00B0)\n",
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

static void SimulationThread(std::stop_token stop_token, ServerState *state)
{
	PipelineState &pipeline = state->pipeline;

	while (!stop_token.stop_requested())
	{
		if (!state->isStreaming)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		if (dbg_isBreaking.load())
		{ // Algorithm hit breakpoint, wait for it to continue
			dbg_isBreaking.wait(true);
		}

		if (!state->isStreaming || stop_token.stop_requested())
			break;

		// Check after breakpoint to allow for halting while in breakpoint
		if (state->simAdvance == 0)
		{ // Wait for next frame advance
			state->simWaiting = true;
			state->simWaiting.notify_all();
			state->simAdvance.wait(0);
			state->simWaiting = false;
		}

		if (!state->isStreaming || stop_token.stop_requested())
			break;

		long desiredFrameIntervalUS = 1000000 / state->controllerConfig.framerate;

		std::unique_lock pipeline_lock(pipeline.pipelineLock); // for frameNum/frameRecordReplayPos
		std::shared_ptr<FrameRecord> frameRecord = std::make_shared<FrameRecord>();
		if (state->mode == MODE_Simulation)
		{
			frameRecord->num = frameRecord->ID = pipeline.record.frames.push_back(frameRecord); // new shared_ptr
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
		else if (state->mode == MODE_Replay)
		{
			std::size_t frame = pipeline.frameNum+1;
			if (!pipeline.record.frames.insert(frame, frameRecord)) // new shared_ptr
			{ // Should not happen unless frameRecords culling is incorrectly used in replay mode
				pipeline.frameNum++;
				std::this_thread::sleep_for(std::chrono::microseconds(desiredFrameIntervalUS));
				continue;
			}

			if (frame == 0)
			{ // Reset time
				state->recording.replayTime = sclock::now();
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

			auto framesStored = state->stored.frames.getView();
			if (frame >= framesStored.endIndex() || !framesStored[frame])
			{ // No frame records to replay
				frameRecord->num = frameRecord->ID = frame;
				frameRecord->time = sclock::now();
				frameRecord->cameras.resize(pipeline.cameras.size());
			}
			else
			{
				auto &loadedRecord = framesStored[frame];
				assert(loadedRecord->num == frame);
				assert(loadedRecord->cameras.size() == pipeline.cameras.size());
				frameRecord->num = loadedRecord->num;
				frameRecord->ID = loadedRecord->ID;
				frameRecord->time = state->recording.replayTime + (loadedRecord->time - framesStored.front()->time);
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
				assert(pipeline.record.imus.size() == state->stored.imus.size());
				for (int i = 0; i < state->stored.imus.size(); i++)
				{
					auto &imu = pipeline.record.imus[i];
					{
						auto samples = state->stored.imus[i]->samplesFused.getView();
						auto it = samples.pos(imu->samplesFused.getView().endIndex());
						for (; it != samples.end() && it->timestamp < targetIMUTime; it++)
						{ // Copy sample, re-mapping timestamp to current replay time
							auto sample = *it;
							sample.timestamp = state->recording.replayTime + (sample.timestamp - framesStored.front()->time);
							imu->samplesFused.insert(it.index(), sample);
						}
					}
					{
						auto samples = state->stored.imus[i]->samplesRaw.getView();
						auto it = samples.pos(imu->samplesRaw.getView().endIndex());
						for (; it != samples.end() && it->timestamp < targetIMUTime; it++)
						{ // Copy sample, re-mapping timestamp to current replay time
							auto sample = *it;
							sample.timestamp = state->recording.replayTime + (sample.timestamp - framesStored.front()->time);
							imu->samplesRaw.insert(it.index(), sample);
						}
					}
				}

				for (auto &cam : state->cameras)
				{ // Set camera image in cameras
					if (!frameRecord->cameras[cam->pipeline->index].image)
						continue;
					// Asnynchronously decompress camera image record
					threadPool.push([&cam](int, std::shared_ptr<CameraImageRecord> imageRecord)
					{
						auto image = decompressCameraImageRecord(imageRecord);
						if (!image) return; // Image jpeg is faulty, don't store record

						// Store as most recent decompressed image
						cam->state.latestFrameImage = std::move(image);
						cam->state.latestFrameImageRecord = std::move(imageRecord);

						SignalCameraRefresh(cam->id);
					}, frameRecord->cameras[cam->pipeline->index].image); // new shared_ptr
				}

				// TODO: If desired, fake a target "detection" using prerecorded tracking results
				// This would speed up verification of tracking for optimising parameters
				// only slightly inaccurate for parameters affecting detection
			}
		}
		pipeline_lock.unlock();

		if (!state->isStreaming || stop_token.stop_requested())
			break;

		auto frameReceiveTime = sclock::now();

		{
			auto start = sclock::now();
			ProcessFrame(pipeline, frameRecord); // new shared_ptr
			auto end = sclock::now();

			UpdateTrackingIO(*state, frameRecord);

			SignalCameraRefresh(0);
			SignalPipelineUpdate();
		}

		{ // Special cases for advancing
			int count = state->simAdvance;
			if (count > 0)
			{ // Advance limited amount of frames, if it fails to reduce, no matter
				state->simAdvance.compare_exchange_weak(count, count-1);
				state->simAdvance.notify_all();
			}
			else if (count == -2)
			{ // Advance until next image - halt advance since we received the next image
				bool haveImageData = false;
				for (auto &cam : state->cameras)
					haveImageData |= (bool)frameRecord->cameras[cam->pipeline->index].image;
				if (haveImageData)
				{
					state->simAdvance = 0;
					state->simAdvance.notify_all();
				}
				else {} // Advance freely, do nothing
			}
			else if (count == -1)
			{} // Advance freely, do nothing
		}

		if (state->simAdvanceQuickly)
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

	// Setup cameras
	for (auto cam : cameras)
	{
		EnsureCamera(state, cam.ID);
		// TODO: Ensure replay camera has the same mode (from image frameX/frameY)? Or not important?
	}

	// Setup IMUs
	state.pipeline.record.imus.clear();
	state.pipeline.record.imus.reserve(state.stored.imus.size());
	for (auto &storedIMU : state.stored.imus)
	{
		auto imu = std::make_shared<IMURecord>(*storedIMU);
		auto tracker = getIMUTracker(imu->id);
		if (!imu->tracker || imu->tracker.id == tracker.id)
			imu->tracker = tracker;
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

	// Clean any prior streaming state
	// TODO: Find a way to retain records and calibration state of last Streaming (without leaving current mode!)
	// Currently after stop	ping stream, all data is retained, and only deleted after starting stream again or leaving mode
	// Now instead of deleting it upon starting stream again, append to the records and keep calibration state intact
	// Needs support for frame num generated by controller to be different from frameNum in pipeline
	ResetPipelineData(state.pipeline);

	// Initialise state
	InitPipelineStreaming(state.pipeline);
	state.isStreaming = true;

	if (state.mode == MODE_Device)
	{
		DevicesStartStreaming(state);
	}

	SignalServerEvent(EVT_START_STREAMING);

	LOG(LDefault, LInfo, "Started stream!\n");
	return true;
}

void StopStreaming(ServerState &state)
{
	if (state.mode == MODE_None || !state.isStreaming)
		return;
	LOG(LDefault, LInfo, "Stopping stream!\n");

	if (state.mode == MODE_Device)
	{
		DevicesStopStreaming(state);
	}

	state.isStreaming = false;
	ResetPipelineStreaming(state.pipeline);

	SignalServerEvent(EVT_STOP_STREAMING);

	LOG(LDefault, LInfo, "Stopped stream!\n");

	// TODO: Move IMU Config storage to on-demand callback?
	ServerStoreIMUConfig(state);
}

void ProcessStreamFrame(SyncGroup &sync, SyncedFrame &frame, bool premature)
{
	// Frame arrived, possibly incomplete or out-of-date, consider for realtime processing

	if (!premature)
	{ // Implies this if the final time this frame is processed
		// Also the only time it's processed if all cameras sent streaming packets on time

		// TODO: Record into frameRecords anyway for later processing

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
		return;
	}
	// Follow up with realtime processing
	if (frame.outdated)
	{ // Should not happen because it should have gotten a premature processing
		// Except at the very beginning where premature processing conditions have not yet initialised
		if (frame.ID > 50)
			LOG(LStreaming, LDebug, "--------- Frame ID %d (%d) processing is outdated!\n", frame.ID, frame.ID&0xFF);
		// TODO: If latency stdDev is ridiculously large, this might be true even later
		// for frames that had missing packets and were only prematurely processed very late
		return;
	}

	if (premature)
	{ // Implies incomplete, WILL be called another time, either once complete or when discarded
		LOG(LStreaming, LDebug, "Frame %d (%d) has %d/%d cameras completed when prematurely processed!\n", frame.ID, frame.ID&0xFF, frame.completed, frame.expecting);
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
	frameRecord->time = frame.SOF;
	frameRecord->ID = frame.ID;
	frameRecord->num = pipeline.record.frames.push_back(frameRecord); // new shared_ptr
	for (auto &camera : sync.cameras)
	{ // Copy over image data received before processing started
		if (!camera) continue; // Removed while streaming - have to ignore even if data was valid
		if (camera->state.latestFrameImageRecord && camera->state.latestFrameImageRecord->frameID == frame.ID)
		{ // May happen if frame processing was unreasonably delayed
			frameRecord->cameras[camera->pipeline->index].image = camera->state.latestFrameImageRecord;
		}
	}

	// Queue for processing in a separate thread
	threadPool.push([&](int, std::shared_ptr<FrameRecord> frame){
		auto start = sclock::now();
		ProcessFrame(pipeline, frame); // new shared_ptr
		auto end = sclock::now();

		if (GetState().mode == MODE_None)
			return;

		std::shared_lock dev_lock(GetState().deviceAccessMutex);

		UpdateTrackingIO(GetState(), frame);

		for (int c = 0; c < frame->cameras.size(); c++)
		{
			if (frame->cameras[c].received)
				SignalCameraRefresh(pipeline.cameras[c]->id);
		}
	}, frameRecord); // new shared_ptr
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
}

void UpdateTrackingIO(ServerState &state, std::shared_ptr<FrameRecord> &frame)
{
	auto io_lock = std::unique_lock(state.io.mutex);
	if (!state.io.vrpn_server) return;

	for (auto &trackRecord : frame->trackers)
	{
		auto io_tracker = state.io.vrpn_trackers.find(trackRecord.id);
		if (io_tracker == state.io.vrpn_trackers.end())
		{
			auto target = std::find_if(state.pipeline.tracking.targetCalibrations.begin(), state.pipeline.tracking.targetCalibrations.end(),
				[&](auto &t){ return t.id == trackRecord.id; });
			if (target == state.pipeline.tracking.targetCalibrations.end()) continue;
			std::string path = target->label;
			//std::string path = asprintf_s("AsterTarget_%.4d", trackedTarget.id);
			LOG(LIO, LInfo, "Exposing VRPN Tracker '%s'", path.c_str());
			auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(trackRecord.id, path.c_str(), state.io.vrpn_server.get());
			io_tracker = state.io.vrpn_trackers.insert({ trackRecord.id, std::move(vrpn_tracker) }).first;
		}

		// TODO: Send both poseObserved and poseFiltered?
		io_tracker->second->updatePose(0, frame->time, trackRecord.poseFiltered);
		io_tracker->second->mainloop();
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

		io_tracker->second->updatePose(0, frame->time, camera->calib.transform.cast<float>());
		io_tracker->second->mainloop();
	}

	state.io.vrpn_server->mainloop();
	if (!state.io.vrpn_server->doing_okay())
	{
		LOG(LIO, LWarn, "VRPN Connection Error!\n");
	}
}