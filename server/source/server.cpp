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

static void ReadUSBPacket(ServerState &state, TrackingControllerState &controller, uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint);
static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length, void *userState, std::shared_ptr<void> &userDevice, bool success);
static void onUSBPacketIN(uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint, void *userState, std::shared_ptr<void> &userDevice);

bool ServerInit(ServerState &state)
{
	state.libusb_context = comm_init_context();

	// Read configurations
	parseGeneralConfigFile("store/general_config.json", state.config);
	parseCameraConfigFile("store/camera_config.json", state.cameraConfig);

	// Load calibrations
	parseCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
	parseTargetCalibrations("store/target_calib.json", state.pipeline.tracking.targetTemplates3D);
	
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
	std::vector<TargetTemplate3D> targetTemplates;
	targetTemplates.reserve(state.pipeline.tracking.targetTemplates3D.size());
	for (int i = 0; i < state.pipeline.tracking.targetTemplates3D.size(); i++)
	{ // TODO: Differentiate simulated targets some better way?
		// Or just store them separately in server like is planned anyway, and only combine in pipeline?
		if (state.pipeline.tracking.targetTemplates3D[i].id >= 0)
			targetTemplates.push_back(state.pipeline.tracking.targetTemplates3D[i]);
	}

	// Write both as current calibration
	storeTargetCalibrations("store/target_calib.json", targetTemplates);
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

static int DetectNewControllers(ServerState &state)
{
	// Setup new controllers
	std::vector<std::shared_ptr<USBCommState>> newControllers = comm_connect(state.libusb_context);
	for (auto &comm : newControllers)
	{
		std::shared_ptr<TrackingControllerState> controller = std::make_shared<TrackingControllerState>();
		comm->userState = &state;
		comm->userDevice = std::static_pointer_cast<void>(controller); // new shared_ptr
		comm->onControlResponse = onControlResponse;
		comm->onUSBPacketIN = onUSBPacketIN;
		controller->comm = std::move(comm);
		controller->endpoints.resize(1);
		controller->id = state.controllers.size();
		// TODO: Make controller IDs persistent, crucial for future persistent setup configurations
		state.controllers.push_back(std::move(controller));
	}

	if (!newControllers.empty())
		LOG(LControllerDevice, LInfo, "%d new controllers connected (%d total)!\n", (int)newControllers.size(), (int)state.controllers.size());
	return newControllers.size();
}

static void DisconnectController(ServerState &state, TrackingControllerState &controller)
{
	std::unique_lock dev_lock(state.deviceAccessMutex); // controllers

	// Clean up cameras if they are now unconnected
	for (int c = 0; c < controller.cameras.size(); c++)
	{
		if (!controller.cameras[c])
			continue;
		controller.cameras[c]->controller = nullptr;
		controller.cameras[c]->port = -1;
		DeviceCheckCameraDisconnect(state, *controller.cameras[c]);
	}

	// Clean up controller
	comm_disconnect(controller.comm);
	controller.sync = nullptr;
	if (controller.syncGen)
	{ // Delete sync group
		DeleteSyncGroup(*state.stream.contextualLock(), std::move(controller.syncGen));
	}

	// Remove controller
	auto c = std::find_if(state.controllers.begin(), state.controllers.end(), [&controller](const auto &c) { return *c == controller; });
	state.controllers.erase(c);

	// Stop streaming if there are no cameras left
	if (state.cameras.empty() && state.isStreaming)
	{
		LOG(LDefault, LWarn, "Leaving streaming mode since all cameras are disconnected after controller disconnect!");
		StopStreaming(state);
	}
}

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

	// Connect to IMU providers
	hid_init();
	{ // Fetch IMU providers
		auto imu_lock = state.imuProviders.contextualLock();
		std::unique_lock hid_lock(state.hid_access);
		detectSlimeVRReceivers(*imu_lock);
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

std::shared_ptr<TrackingCameraState> DeviceSetupCamera(ServerState &state, CameraID id)
{
	std::shared_ptr<TrackingCameraState> camera = EnsureCamera(state, id);

	// Sync setup data
	DeviceUpdateCameraSetup(state, *camera);

	// Make sure secondary features match expected state
	DeviceUpdateStream(*camera);
	DeviceUpdateVis(*camera);

	return camera;
}

bool DeviceCheckCameraDisconnect(ServerState &state, TrackingCameraState &camera)
{
	if (camera.client || camera.controller)
		return false;
	// No other connection, remove camera

	std::unique_lock dev_lock(state.deviceAccessMutex); // cameras

	// Clean up stream state
	RemoveCameraSync(*state.stream.contextualLock(), camera);

	// Keep pipeline camera alive

	// Remove camera device
	auto cam = std::find_if(state.cameras.begin(), state.cameras.end(), [&camera](const auto &c) { return c->id == camera.id; });
	assert(cam != state.cameras.end());
	state.cameras.erase(cam);

	SignalServerEvent(EVT_UPDATE_CAMERAS);
	if (state.cameras.empty() && state.isStreaming)
	{
		LOG(LDefault, LWarn, "Leaving streaming mode since all cameras are disconnected after camera disconnect!");
		StopStreaming(state);
	}
	return true;
}

// ----------------------------------------------------------------------------
// Device Comms
// ----------------------------------------------------------------------------

void DeviceUpdateCameraSetup(ServerState &state, TrackingCameraState &device)
{
	CameraConfig &config = state.cameraConfig.configurations[state.cameraConfig.cameraConfigs[device.id]];
	ConfigPacket packet = {};
	packet.width = config.width;
	packet.height = config.height;
	if (device.sync && device.sync->contextualRLock()->source != SYNC_NONE)
		packet.fps = config.framerate;
	else
		packet.fps = 144;
		// TODO: Implement control over FPS in free-running cameras
		// Currently not sure how to instruct the OV9281 to do so. Also unimportant
		// Same in DevicesSetupSyncGroups
	packet.shutterSpeed = config.exposure;
	packet.analogGain = config.gain;
	packet.extTrig = config.synchronised;
	packet.strobe = config.enableStrobe;
	// TODO: Implement control over filter switcher on both Pi and STM32
	//packet.filter = 
	packet.strobeOffset = config.strobeOffset;
	packet.strobeLength = config.strobeLength;
	packet.blobProc = config.blobProcessing;
	uint8_t setup[CONFIG_PACKET_SIZE];
	storeConfigPacket(packet, setup);
	device.sendPacket(PACKET_CFG_SETUP, setup, CONFIG_PACKET_SIZE);
}
bool DeviceUpdateWireless(ServerState &state, TrackingCameraState &device)
{
	// Send wireless config to Tracking Camera
	auto &config = device.config.wireless;
	std::vector<uint8_t> wireless(config.enabled? 4 : 2);
	wireless[0] = config.enabled;
	wireless[1] = config.Server;
	if (config.enabled)
	{
		uint16_t credSize = state.wpa_supplicant_conf.size();
		if (credSize > 2000)
		{ // TODO: wpa_supplicant limited by CTRL_TRANSFER_SIZE and USBD_CTRL_MAX_PACKET_SIZE
			// Implement feedback on failure in UI
			LOG(LGUI, LWarn, "wpa_supplicant is too long to send over control transfers!");
			credSize = 0;
			return false;
		}
		wireless[2] = credSize >> 8;
		wireless[3] = credSize & 0xFF;
		wireless.reserve(2+2+credSize);
		wireless.insert(wireless.end(), state.wpa_supplicant_conf.begin(), state.wpa_supplicant_conf.begin()+credSize);
	}
	config.updating = true; // Awaiting a status packet to notify server of actual state
	config.failed = false; // Clear past failure
	device.sendPacket(PACKET_CFG_WIFI, wireless.data(), wireless.size());
	return true;
}
/**
 * Mode 0: Image Stream off
 * Mode 1: Image Stream every X frames
 * Mode 2: Single Image Request of frame X (tbd)
 */
static void DeviceSendImageRequest(TrackingCameraState &device, uint8_t mode, ImageRequest &request)
{
	// Send image request to Tracking Camera
	int len = 1;
	uint8_t stream[12];
	stream[0] = mode;
	if (mode > 0)
	{
		*((uint16_t *)&stream[1]) = request.bounds.minX;
		*((uint16_t *)&stream[3]) = request.bounds.minY;
		*((uint16_t *)&stream[5]) = request.bounds.maxX;
		*((uint16_t *)&stream[7]) = request.bounds.maxY;
		stream[9] = request.subsampling;
		stream[10] = request.jpegQuality;
		stream[11] = request.frame;
		len = 12;
	}
	device.sendPacket(PACKET_CFG_IMAGE, stream, len);
}
void DeviceUpdateStream(TrackingCameraState &device)
{
	DeviceSendImageRequest(device, device.config.imageStreaming.enabled, device.config.imageStreaming.request);
}
void DeviceUpdateVis(TrackingCameraState &device)
{
	// Send vis config to Tracking Camera
	auto &config = device.config.hdmiVis;
	int len = 1;
	uint8_t vis[8];
	vis[0] = config.enabled;
	if (vis[0])
	{ // Width, Height, interval in frames
		*((uint16_t *)&vis[1]) = config.width;
		*((uint16_t *)&vis[3]) = config.height;
		vis[5] = config.fps;
		vis[6] = config.displayFrame;
		vis[7] = config.displayBlobs;
		len = 8;
	}
	device.sendPacket(PACKET_CFG_VIS, vis, len);
}

static void LogUSBStats(TrackingControllerState &controller, long timeUS)
{
	// Called from the same thread, so fine without mutex
	//controller.comm->statMutex.lock();
	std::map<void*, TransferStats> stats;
	controller.comm->streamingEPStats.swap(stats);
	float largestLag = controller.comm->largestIntLag;
	controller.comm->largestIntLag = 0;
	//controller.comm->statMutex.unlock();

	std::map<int, std::tuple<int, int, int>> endpoints;
	for (auto &stat : stats)
	{
		auto &ep = endpoints[stat.second.ep];
		std::get<0>(ep)++;
		std::get<1>(ep) += stat.second.receiveCount;
		std::get<2>(ep) += stat.second.receiveBytes;
	}
	LOG(LUSB, endpoints.empty()? LTrace : LDebug, "%d eps received a transfer over the past %.2fs, with the largest combined lag of %.2fms\n",
		(int)endpoints.size(), timeUS/1000000.0f, largestLag);
	for (auto &ep : endpoints)
	{
		#define INT_TRANSFER_SIZE 1024 // From usb.cpp
		LOG(LUSB, LDebug, "    EP %d: %d queues with %d transfers of %d bytes total, %.0f%% fill rate, %.0f%% throughput, %.0f%% transfer rate\n",
			ep.first, std::get<0>(ep.second), std::get<1>(ep.second), std::get<2>(ep.second),
			(float)std::get<2>(ep.second)/std::get<1>(ep.second)/INT_TRANSFER_SIZE*100,
			std::get<2>(ep.second)/(INT_TRANSFER_SIZE*8*timeUS/1000.0f)*100,
			std::get<1>(ep.second)/(8*timeUS/1000.0f)*100);
		for (auto &stat : stats)
		{
			if (stat.second.ep != ep.first) continue;
			LOG(LUSB, LDebug, "        Queue %p: %d transfers of %d bytes total, %.0f%% fill rate\n",
				stat.first, stat.second.receiveCount, stat.second.receiveBytes,
				(float)stat.second.receiveBytes/stat.second.receiveCount/INT_TRANSFER_SIZE*100);
		}
	}
}

static void DeviceSupervisorThread(std::stop_token stop_token, ServerState *state)
{
	int it = 0;
	auto checkControlRequest = [](TrackingControllerState &controller, TrackingControllerState::USBRequest &request, const char* label, USBCommand command, int intervalMS)
	{
		float deltaT = dtMS(request.submitted, sclock::now());
		if (request.transfer < 0 && deltaT > intervalMS)
		{ // Interval lapsed, request transfer
			request.submitted = sclock::now();
			request.transfer = comm_submit_control_request(controller.comm, command, 0, 0);
			if (request.transfer < 0)
			{
				LOG(LUSB, LWarn, "Failed to request %s control transfer!\n", label);
			}
			else
			{
				LOG(LUSB, LTrace, "Allocated control transfer %d to %s request\n",
					request.transfer.load(), label);
			}
		}
		else if (request.transfer >= 0 && deltaT > 20)
		{ // Just waiting longer since it probably indicates an error with the controller and we don't want to spam the log
			// But libusb will timeout the transfer before anyway
			LOG(LUSB, LWarn, "Controller didn't respond to control transfer %d (%s) in %fms, cancelling!\n",
				request.transfer.load(), label, deltaT);
			comm_cancel_control_request(controller.comm, request.transfer);
			request.transfer = -1;
			request.stalling = false;
		}
		else if (request.transfer >= 0 && deltaT > 10 && !request.stalling)
		{ // Over 10ms to answer transfer is very odd
			LOG(LUSB, LWarn, "Controller didn't respond to control transfer %d (%s) in %fms!\n",
				request.transfer.load(), label, deltaT);
			request.stalling = true;
		}
	};

	bool checkingIMU = false;
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
		for (int c = 0; c < state->controllers.size();)
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

			if (controller.comm->commStreaming || state->isStreaming)
			{ // TODO: Send camera disconnect events via interrupt endpoints, too
				// Currently, we rely on getting some rare control requests through to find out a camera disconnected
				// This is quite suboptimal, to be honest. Allow controller to send status packets on it's own when it changed
				checkControlRequest(controller, controller.statusReq, "Status", COMMAND_IN_STATUS, 50);
			}
			else
			{
				checkControlRequest(controller, controller.statusReq, "Status", COMMAND_IN_STATUS, 50);
				checkControlRequest(controller, controller.debugReq, "Debug", COMMAND_IN_DEBUG, 10);
				checkControlRequest(controller, controller.eventReq, "Event", COMMAND_IN_EVENTS, 100);
				checkControlRequest(controller, controller.packetReq, "Packet", COMMAND_IN_PACKETS, 100);
			}

			if (!controller.comm->commStreaming)
				controller.comm->lastUSBStatCheck = sclock::now();
			// Requires mutex in usb transfer if called here, not good
			/*long timeUS = dtUS(controller.comm->lastUSBStatCheck, sclock::now());
			if (timeUS > 2000000)
			{
				controller.comm->lastUSBStatCheck = sclock::now();
				LogUSBStats(controller, timeUS);
			} */

#ifdef USB_PACKET_QUEUE
			static TimePoint_t last = sclock::now(); // TODO: This is static, so shared between controllers. Makes no sense.
			TimePoint_t start = sclock::now();
			int packetParsed = 0;
			//while(true)
			{
				TimePoint_t s0 = sclock::now();
				std::vector<TrackingControllerState::USBPacket> packets;
				{
					auto queue_lock = controller.packetQueue.contextualLock();
					while (!queue_lock->empty())
					{
						packets.push_back(std::move(queue_lock->front()));
						queue_lock->pop();
					}
				}
				TimePoint_t s1 = sclock::now();
				float accumMS = dtMS(last, s1);
				last = s0;
				for (auto &packet : packets)
				{
					ReadUSBPacket(*state, controller, packet.data.data(), packet.data.size(), packet.receiveTime, packet.endpoint);
				}
				packetParsed += packets.size();
				TimePoint_t s2 = sclock::now();
				if (dtUS(s0, s2) > 2000)
				{
					LOG(LParsing, LDarn, "Parsing %d usb packets accumulated over %.2fms took %.2fms of processing plus %ldus of dequeing!",
						(int)packets.size(), accumMS, dtMS(s1, s2), dtUS(s0,s1));
				}
				//if (dtUS(start, s2) > 5000)
				//{
				//	LOG(LParsing, LDarn, "Been parsing %d packets for %.2fms, with %d left in queue, leaving loop!",
				//		packetParsed, dtMS(start, s2), packets.size());
				//	break;
				//}
			}
#endif

			// Handle next controller
			c++;
		}

		if (!checkingIMU && dtMS(lastIMUCheck, sclock::now()) > 500)
		{ // Regularly check for new IMU providers
			checkingIMU = true;
			lastIMUCheck = sclock::now();
			threadPool.push([&checkingIMU](int){
				auto &state = GetState();
				auto providers = *state.imuProviders.contextualLock();
				{ // Fetch new IMU providers
					std::unique_lock hid_lock(state.hid_access);
					detectSlimeVRReceivers(providers);
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
				int updatedDevices, changedDevices;
				if (imuProvider->get()->poll(updatedDevices, changedDevices) == IMU_STATUS_DISCONNECTED)
				{
					imusChanged = true;
					imuProvider = imuLock->erase(imuProvider);
					continue;
				}
				if (changedDevices > 0) imusChanged = true;
				if (updatedDevices > 0) imusUpdated = true;
				imuProvider++;
			}
		}
		if (imusUpdated)
			SignalPipelineUpdate();

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
	auto &targets = state.pipeline.tracking.targetTemplates3D;
	for (int i = 0; i < state.config.simulation.trackingTargets.size(); i++)
	{
		const TargetTemplate3D &targetDef = state.config.simulation.trackingTargets[i];

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
		if (!state->isStreaming || pipeline.cameras.empty())
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
		std::shared_ptr<FrameRecord> frameRecord = std::make_shared<FrameRecord>();;
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
			if (state->recordFrameCount <= state->recordReplayFrame)
			{ // No frame records to replay (or only loaded observations and there were never any frame records)
				frameRecord->num = frameRecord->ID = pipeline.record.frames.push_back(frameRecord); // new shared_ptr
				frameRecord->time = sclock::now();
				frameRecord->cameras.resize(pipeline.cameras.size());
			}
			else
			{
				auto stored = state->record.frames.getView();
				auto &loadedRecord = stored[state->recordReplayFrame];
				if (!loadedRecord)
				{
					state->recordReplayFrame++;
					std::this_thread::sleep_for(std::chrono::microseconds(desiredFrameIntervalUS));
					continue;
				}
				if (state->recordReplayFrame == 0)
					state->recordReplayTime = sclock::now();
				if (!pipeline.record.frames.insert(state->recordReplayFrame, frameRecord)) // new shared_ptr
					continue;
				frameRecord->num = state->recordReplayFrame;
				frameRecord->ID = loadedRecord->ID;
				frameRecord->time = state->recordReplayTime + (loadedRecord->time - stored.front()->time);
				frameRecord->cameras = loadedRecord->cameras;
				assert(frameRecord->cameras.size() == pipeline.cameras.size());
				if (state->recordReplayFrame++ < state->recordFrameCount)
				{ // Replicate original frame pacing
					auto &nextRecord = stored[state->recordReplayFrame];
					if (nextRecord)
						desiredFrameIntervalUS = std::chrono::duration_cast<std::chrono::microseconds>(nextRecord->time - loadedRecord->time).count();
					// TODO: Set desired frame end time to better stick to frame time, otherwise replay quickly gets out of sync and VRPN clients will be unhappy due to apparent high latency
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

		//threadPool.push([&](int, std::shared_ptr<FrameRecord> frameRecord)
		{
			auto start = sclock::now();
			ProcessFrame(pipeline, frameRecord); // new shared_ptr
			auto end = sclock::now();

			UpdateTrackingIO(*state, frameRecord);

			SignalCameraRefresh(0);
			SignalPipelineUpdate();
		}//, frameRecord); // new shared_ptr

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
	state.recordReplayFrame = 0;

	// Setup cameras
	for (auto cam : cameras)
	{
		EnsureCamera(state, cam.ID);
		// TODO: Ensure replay camera has the same mode (from image frameX/frameY)? Or not important?
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
	state.record.frames.cull_clear();
	state.recordReplayFrame = 0;
	state.recordFrameCount = 0;

	SignalServerEvent(EVT_MODE_SIMULATION_STOP);
	SignalServerEvent(EVT_UPDATE_CAMERAS);
}

// ----------------------------------------------------------------------------
// Streaming
// ----------------------------------------------------------------------------

void DevicesOpenCommChannels(ServerState &state)
{
	// Communicate with controllers to enter streaming state
	for (auto &controller : state.controllers)
	{
		ResetTimeSync(*controller->timeSync.contextualLock());

		// Open the comm channels (interrupt transfers) to the controller
		if (!comm_startStream(controller->comm))
		{
			LOG(LControllerDevice, LError, "Failed to open comm channels to controller!\n");
		}
		else
		{
			LOG(LControllerDevice, LInfo, "Opened comm channels to controller!\n");
			controller->endpoints.resize(controller->comm->streamingEndpoints.load()+1);
		}
	}
}

void DevicesStartTimeSync(ServerState &state)
{
	// Communicate with controllers to enter streaming state
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;

		// Request to start establishing a solid timesync with both the host (this) and the cameras
		comm_submit_control_data(controller->comm, COMMAND_OUT_TIME_SYNC, true, 0);
		LOG(LControllerDevice, LInfo, "Requesting to start time sync with controller!\n");
	}
}

void DevicesSetupCameras(ServerState &state)
{
	for (auto &cam : state.cameras)
	{
		if (!cam->client && !cam->controller->comm->commStreaming) continue;

		// Send setup data incase it didn't already happen
		DeviceUpdateCameraSetup(state, *cam);

		// Make sure secondary features match expected state
		DeviceUpdateStream(*cam);
		DeviceUpdateVis(*cam);
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	for (auto &cam : state.cameras)
	{
		if (!cam->client && !cam->controller->comm->commStreaming) continue;

		// Communicate with camera to start streaming (startup time is high)
		cam->sendModeSet(TRCAM_FLAG_STREAMING | TRCAM_MODE_BLOB);
	}
}

void DevicesSetupSyncGroups(ServerState &state)
{
	auto stream_lock = state.stream.contextualLock();

	// Setup sync groups
	std::shared_ptr<Synchronised<SyncGroup>> masterSync;
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;

		// Setup sync group
		controller->sync = nullptr;
		if (masterSync && controller->syncGen)
		{ // Supposed to be synced up with master, not generate own sync
			DeleteSyncGroup(*stream_lock, std::move(controller->syncGen));
			controller->syncGen = nullptr;
		}
		else if (!masterSync && !controller->syncGen)
		{ // No master and no own sync yet, set up to generate sync
			controller->syncGen = std::make_shared<Synchronised<SyncGroup>>();
			stream_lock->syncGroups.push_back(controller->syncGen); // new shared_ptr
			// TODO: Controller might have external sync source, set to SYNC_EXTERNAL then
			auto sync_lock = controller->syncGen->contextualLock();
			sync_lock->source = SYNC_INTERNAL;
			sync_lock->frameIntervalMS = 1000.0f / state.controllerConfig.framerate;
		}
		else if (controller->syncGen)
		{ // Clear existing syncGen
			ClearSyncGroup(*controller->syncGen->contextualLock());
		}
		// TODO: Give control which one generates signal, but not 100% necessary
		// TODO: Allow for multiple syncGroups (so multiple controllers with syncGen)
		if (controller->syncGen)
			masterSync = controller->syncGen; // new shared_ptr

		// Select sync source for controller
		controller->sync = masterSync? masterSync : controller->syncGen;

		// Enter cameras into sync group
		for (int c = 0; c < controller->cameras.size(); c++)
		{
			if (controller->cameras[c])
			{
				SetCameraSync(*stream_lock, controller->cameras[c], controller->sync);
			}
		}
	}

	// Reset sync group states
	ResetStreamState(*stream_lock);

	for (auto &cam : state.cameras)
	{
		if (cam->sync)
		{
			// TODO: Implement control over FPS in free-running cameras
			// also see other TODO on this (same in DeviceUpdateCameraSetup)
			// E.g. OV9281 drivers cannot set framerate yet
			// So its always running at 144Hz for 1280x800 and 253Hz for 640x400
			auto sync_lock = cam->sync->contextualLock();
			if (sync_lock->source == SYNC_NONE)
				sync_lock->frameIntervalMS = 1000.0f / 144;
		}
	}
}

void DevicesStartSync(ServerState &state)
{
	// Set external sync source for relevant controllers
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;

		if (controller->syncGen && controller->syncGen->contextualRLock()->source == SYNC_INTERNAL)
			continue; // Start generating last
		
		if (controller->sync && controller->sync->contextualRLock()->source != SYNC_NONE)
		{ // Request to copy external sync input
			comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_EXTERNAL, 0, 0);
		}
		else
		{ // Request to not do any sync - cameras will need to be free-running
			comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_RESET, 0, 0);
		}
	}

	// Set generating sync source for relevant controllers
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;
		if (!controller->syncGen) continue;
		// Request to start generating frame signals
		auto sync_lock = controller->syncGen->contextualRLock();
		if (sync_lock->source == SYNC_INTERNAL)
			comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_GENERATE, (uint16_t)1000.0f/sync_lock->frameIntervalMS, 0);
	}
}

void DevicesUpdateSyncMask(ServerState &state)
{
	// Setup sync masks for all controllers
	for (auto &controller : state.controllers)
	{
		if (!controller->comm->commStreaming) continue;
		if (!controller->sync || controller->sync->contextualRLock()->source == SYNC_NONE) continue;

		// Select cameras that have been setup and chosen for streaming
		uint16_t portMask = 0;
		for (auto &camera : controller->cameras)
		{
			if (camera && (camera->mode & TRCAM_FLAG_STREAMING) == TRCAM_FLAG_STREAMING)
				portMask |= 1 << camera->port;
		}
		comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_MASK, 0x00, portMask);
	}
}

bool DevicesAwaitModeChanges(int timeoutMS)
{
	int waiting = modeChangesWaiting.load();
	// Wait for cameras to report their mode change status
	//modeChangesConfirmed.wait(waiting);
	// I wish, no variants with timeout exist, can't rely on hardware
	int tries = 0;
	while (tries++ < (timeoutMS/10) && modeChangesConfirmed.load() < waiting)
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	return modeChangesConfirmed.load() >= waiting;
}

static void SetupVirtualSyncGroup(ServerState &state)
{
	if (!state.controllers.empty()) return;
	auto stream_lock = state.stream.contextualLock();
	if (!stream_lock->syncGroups.empty()) return;
	// Setup virtual sync group
	stream_lock->syncGroups.push_back(std::make_shared<Synchronised<SyncGroup>>()); // new shared_ptr
	auto sync_lock = stream_lock->syncGroups.back()->contextualLock();
	ResetSyncGroup(*sync_lock);
	sync_lock->source = SYNC_VIRTUAL;
	sync_lock->frameIntervalMS = 1000.0f / state.controllerConfig.framerate;
	// Start with a frame that will be continued
	SyncedFrame frame = {};
	frame.cameras.resize(sync_lock->cameras.size());
	frame.ID = 0;
	frame.approxSOF = false;
	frame.SOF = sclock::now();
	LOG(LStreaming, LInfo, "Started virtual frame generation!\n");
	sync_lock->frames.push_back(std::move(frame));
	sync_lock->frameCount++;
}

static void DeleteVirtualSyncGroup(ServerState &state)
{
	auto stream_lock = state.stream.contextualLock();
	if (stream_lock->syncGroups.empty()) return;
	for (auto it = stream_lock->syncGroups.begin(); it != stream_lock->syncGroups.end();)
	{
		if (it->get()->contextualRLock()->source == SYNC_VIRTUAL)
			it = stream_lock->syncGroups.erase(it);
		else it++;
	}
}

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
		std::shared_lock dev_lock(state.deviceAccessMutex);

		modeChangesConfirmed = modeChangesWaiting = 0;

		DevicesOpenCommChannels(state);

		// Give controller and camera some time to initialise, not strictly needed
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		DevicesStartTimeSync(state);

		// Give controller and camera some time to initialise, not strictly needed
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));

		DevicesSetupCameras(state);

		DevicesSetupSyncGroups(state);

		LOG(LGUI, LInfo, "Setup cameras and sync groups");
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// Wait for cameras to report their mode change status, signalling that they are ready to receive frame syncs
		DevicesAwaitModeChanges(500);
		LOG(LCameraDevice, LInfo, "%d / %d mode changes are confirmed!", modeChangesConfirmed.load(), modeChangesWaiting.load());

		DevicesStartSync(state);

		LOG(LGUI, LInfo, "Starting camera sync");

		DevicesUpdateSyncMask(state);

		// Incase no controller is connected, provide virtual sync group for e.g. IMUs
		SetupVirtualSyncGroup(state);
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
		std::shared_lock dev_lock(state.deviceAccessMutex);

		// Communicate with cameras to stop streaming
		for (auto &cam : state.cameras)
			cam->sendModeSet(TRCAM_STANDBY);

		// Communicate with controllers to prepare to stop streaming
		for (auto &controller : state.controllers)
		{
			// Request to stop triggering new frames
			comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_RESET, 0x00, 0);
			// Request to disable time sync
			comm_submit_control_data(controller->comm, COMMAND_OUT_TIME_SYNC, false, 0);
		}

		// Wait for cameras to report their mode change status, signalling the end of stream data being sent
		DevicesAwaitModeChanges(100);

		// Communicate with controllers to close comm channels
		for (auto &controller : state.controllers)
		{
			// Stop receiving transfers from controller
			comm_stopStream(controller->comm);
			controller->endpoints.clear();
			controller->endpoints.resize(1);

			for (auto &port : controller->ports)
				ResetPacketPort(port);
		}

		// Sync Group not attached to any controller - remove
		DeleteVirtualSyncGroup(state);

		// Reset streaming states
		ResetStreamState(*state.stream.contextualLock());

		// Reset camera device state
		for (auto &cam : state.cameras)
			cam->state = {}; // TODO: This also clears camera error state, might not be desired
	}
	if (state.mode == MODE_Replay)
	{
		state.recordReplayFrame = 0;
	}

	state.isStreaming = false;
	ResetPipelineStreaming(state.pipeline);

	SignalServerEvent(EVT_STOP_STREAMING);

	LOG(LDefault, LInfo, "Stopped stream!\n");
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

	for (auto &trackedTarget : frame->tracking.targets)
	{
		auto io_tracker = state.io.vrpn_trackers.find(trackedTarget.id);
		if (io_tracker == state.io.vrpn_trackers.end())
		{
			auto target = std::find_if(state.pipeline.tracking.targetTemplates3D.begin(), state.pipeline.tracking.targetTemplates3D.end(),
				[&](auto &t){ return t.id == trackedTarget.id; });
			if (target == state.pipeline.tracking.targetTemplates3D.end()) continue;
			std::string path = target->label;
			//std::string path = asprintf_s("AsterTarget_%.4d", trackedTarget.id);
			LOG(LIO, LInfo, "Exposing VRPN Tracker '%s'", path.c_str());
			auto vrpn_tracker = make_opaque<vrpn_Tracker_AsterTrack>(trackedTarget.id, path.c_str(), state.io.vrpn_server.get());
			io_tracker = state.io.vrpn_trackers.insert({ trackedTarget.id, std::move(vrpn_tracker) }).first;
		}

		// TODO: Send both poseObserved and poseFiltered?
		io_tracker->second->updatePose(0, frame->time, trackedTarget.poseFiltered);
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
			auto vrpn_tracker = make_opaque<vrpn_Tracker_AsterTrack>(camera->id, path.c_str(), state.io.vrpn_server.get());
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

// ----------------------------------------------------------------------------
// USB Handlers
// ----------------------------------------------------------------------------

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
	frameRecord->num = pipeline.record.frames.push_back(frameRecord); // new shared_ptr
	frameRecord->ID = frame.ID;
	frameRecord->time = frame.SOF;
	frameRecord->cameras.resize(pipeline.cameras.size());
	for (auto &camera : sync.cameras)
	{
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

	// Queue for processing in a separate thread
	threadPool.push([&](int, std::shared_ptr<FrameRecord> frame){
		auto start = sclock::now();
		ProcessFrame(pipeline, frame); // new shared_ptr
		auto end = sclock::now();

		std::shared_lock dev_lock(GetState().deviceAccessMutex);

		UpdateTrackingIO(GetState(), frame);

		for (int c = 0; c < frame->cameras.size(); c++)
		{
			if (frame->cameras[c].received)
				SignalCameraRefresh(pipeline.cameras[c]->id);
		}
	}, frameRecord); // new shared_ptr
}

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data,
	int length, void *userState, std::shared_ptr<void> &userDevice, bool success)
{
	TrackingControllerState &controller = *static_cast<TrackingControllerState*>(userDevice.get());
	ServerState &state = *(ServerState *)userState;
	auto clearControlRequest = [&](TrackingControllerState::USBRequest &request, const char* label)
	{
		if (request.stalling)
		{
			float deltaT = dtMS(request.submitted, sclock::now());
			if (success)
			{
				LOG(LUSB, LWarn, "Stalled control transfer %d (%s) completed after %fms",
					request.transfer.load(), label, deltaT);
			}
			else
			{ // Libusb timeout
				LOG(LUSB, LWarn, "Stalled control transfer %d (%s) got cancelled after %fms",
					request.transfer.load(), label, deltaT);
			}
			request.stalling = false;
		}
		request.transfer = -1;
	};
	switch (request)
	{
	case COMMAND_IN_STATUS:
		clearControlRequest(controller.statusReq, "Status");
		break;
	case COMMAND_IN_DEBUG:
		clearControlRequest(controller.debugReq, "Debug");
		break;
	case COMMAND_IN_EVENTS:
		clearControlRequest(controller.eventReq, "Event");
		break;
	case COMMAND_IN_PACKETS:
		clearControlRequest(controller.packetReq, "Packet");
		break;
	default:
		break;
	}
	if (!success)
		return;

	switch (request)
	{
	case COMMAND_IN_STATUS:
	{ // Camera Status Response
		ReadStatusPacket(state, controller, data, length);
		break;
	}
	case COMMAND_IN_DEBUG:
	{ // Debug Response
		ReadDebugPacket(controller, data, length);
		break;
	}
	case COMMAND_IN_EVENTS:
	{ // Event Response
		if (state.isStreaming)
		{ // TimeSync is only accurate when streaming anyways
			ReadEventPacket(controller, data, length);
		}
		break;
	}
	case COMMAND_IN_PACKETS:
	{ // Packet Response
		if (length <= USB_PACKET_HEADER)
			break;
#ifdef USB_PACKET_QUEUE
		// Queue packet instead of handling it here to keep USB thread clear
		auto queue_lock = controller.packetQueue.contextualLock();
		queue_lock->emplace(sclock::now(), 0, std::vector<uint8_t>(data+USB_PACKET_HEADER, data+length));
#else
		ReadUSBPacket(state, controller, data+USB_PACKET_HEADER, length-USB_PACKET_HEADER, sclock::now(), 0);
#endif
		break;
	}
	default:
	{
		std::stringstream hexBuf;
		printBuffer(hexBuf, data, length);
		LOG(LControllerDevice, LWarn, "Unknown Control Response %d (%d): %s\n", request, length, hexBuf.str().c_str());
		break;
	}
	}
}

static void onUSBPacketIN(uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint, void *userState, std::shared_ptr<void> &userDevice)
{
	TrackingControllerState &controller = *static_cast<TrackingControllerState*>(userDevice.get());
	ServerState &state = *((ServerState*)userState);
	if (state.mode != MODE_Device || !state.isStreaming)
	{ // Not expecting any transfer
		LOG(LUSB, LDebug, "Unexpected transfer IN while not streaming!\n");
		return;
	}
	if (!controller.comm || !controller.comm->deviceConnected)
	{
		LOG(LUSB, LError, "Unexpected transfer after device was disconnected!\n");
		return;
	}
	if (endpoint >= controller.endpoints.size())
	{
		LOG(LUSB, LError, "Unexpected transfer on endpoint %d! Only set up %d\n", endpoint, (int)controller.endpoints.size());
		return;
	}

	// No need to lock state.deviceAccessMutex, TrackingControllerState is guaranteed to still exist, before destruction comm is cleaned up properly
	// Make sure cameras are only destroyed well after any transfers might reference them - or copy shared_ptr when accessing them here
	//std::shared_lock dev_lock(state.deviceAccessMutex);
	// TODO: Properly replace all those random mutexes (deviceAccess, pipeline) with Synchronised constructs where necessary
	// Or find another construct that efficiently and easily enforces and protects these relevant states
	// Currently, there's no major problem, but it's unsustainable

	{ // Read packet header
		USBPacketHeader header = parseUSBPacketHeader(data);
		// Make sure packet is continuous within endpoint
		auto &stats = controller.endpoints[endpoint]; // Assumes all endpoints are continuous, e.g. ep 1, 2, 3, 4
		if (header.counter != (stats.counter+1)%256)
		{ // Controller must have thought it send a packet but didn't - this really shouldn't happen
			LOG(LUSB, LError, "Dropped packet in endpoint %d! Skipped from %d to %d", endpoint, stats.counter, header.counter);
		}
		else
		{ // Is continuous, got timestamp that the last packet within the endpoint was sent at for use with time sync
			float commLag = dtMS(stats.lastReceived, sclock::now());
			if (commLag > 20)
			{ // Still fine, but controller should be responsible for sending a packet every ms at least to keep up time sync
				LOG(LTimesync, LWarn, "Controller comms dropped out for %fms, this is bad for time sync!", commLag);
			}
			else if (commLag > 5)
			{ // Still fine, but controller should be responsible for sending a packet every ms at least to keep up time sync
				LOG(LTimesync, LDarn, "Controller comms dropped out for %fms, this is suboptimal for time sync!", commLag);
			}
			// Update time sync with the last packet in the endpoint as the newly gathered sample point
			auto sync_lock = controller.timeSync.contextualLock();
			std::pair<uint64_t, TimePoint_t> packetSentSync = UpdateTimeSync(*sync_lock, header.lastTimestamp, 1<<24, stats.lastReceived);
			LOG(LTimesync, LTrace, "Packet of size %d had timestamp %uus for last packet received %.2fms ago, estimated to be sent %.2fms ago (this packet was received %.2fms ago)\n",
				length, header.lastTimestamp, dtMS(stats.lastReceived, sclock::now()), dtMS(packetSentSync.second, sclock::now()), dtMS(receiveTime, sclock::now()));

			if (packetSentSync.first > 0 && controller.recordTimeSyncMeasurements)
				controller.timeSyncMeasurements.push_back({ packetSentSync.first, packetSentSync.second, stats.lastReceived });
		}
		stats = { header.counter, receiveTime };

		data += USB_PACKET_HEADER;
		length -= USB_PACKET_HEADER;
	}

	if (length == 0)
	{ // Null-packets can be sent to enforce timesync when nothing else needs to be sent
		return;
	}

#ifdef USB_PACKET_QUEUE
	// Queue packet instead of handling it here to keep USB thread clear
	auto queue_lock = controller.packetQueue.contextualLock();
	queue_lock->emplace(receiveTime, endpoint, std::vector<uint8_t>(data, data+length));
#else
	ReadUSBPacket(state, controller, data, length, receiveTime, endpoint);
	MaintainStreamState(*state.stream.contextualLock());
#endif

	long timeUS = dtUS(controller.comm->lastUSBStatCheck, sclock::now());
	if (timeUS > 2000000)
	{
		controller.comm->lastUSBStatCheck = sclock::now();
		if (timeUS < 3000000)
			LogUSBStats(controller, timeUS);
	}
}

static void ReadUSBPacket(ServerState &state, TrackingControllerState &controller, uint8_t *data, int length, TimePoint_t receiveTime, uint8_t endpoint)
{
	struct PacketState
	{
		ServerState *state;
		TrackingControllerState *controller;
		TimePoint_t receiveTime;
		int endpoint;
	} packetState = { &state, &controller, receiveTime, endpoint };

	// TODO: This is not properly synchronised/secured, deviceAccessMutex is not even locked
	// Consider at least copying shared_ptr of camera

	// Each USB frame contains multiple blocks of data from multiple ports
	// Each block belongs to exactly one packet from any port
	// This function extracts the blocks and executes the callbacks
	// Importantly, it also makes sure the blocks are processed in the correct order
	// It does this using a continuous blockID and storing out-of-order blocks
	// But for low latency, it can parse packets in parallel, so that a delayed block
	// Does not hinder parsing of blocks of newer packets
	HandlePacketBlocks(
		controller.ports, data, length, &packetState,
		[](void *userData, SignalTag signal, uint8_t *data, uint16_t length) { // onSignal
			PacketState &packetState = *((PacketState*)userData);
			if (signal == SIGNAL_INVALID)
			{
				LOG(LParsing, LTrace, "Skipped invalid block of length %d on endpoint %d\n", length, packetState.endpoint);
			}
			else if (signal == SIGNAL_SOF)
			{
				ReadSOFPacket(*packetState.controller, data, length, packetState.receiveTime);
			}
			else if (signal == SIGNAL_DEBUG)
			{
				ReadDebugPacket(*packetState.controller, data, length);
			}
			else if (signal == SIGNAL_EVENT)
			{
				ReadEventPacket(*packetState.controller, data, length);
			}
		},
		[](void *userData, int port, PacketBlocks &packet) { // onPacketHeader
			PacketState &packetState = *((PacketState*)userData);
			std::shared_ptr<TrackingCameraState> &camera = packetState.controller->cameras[port];
			if (!camera)
			{
				packet.ignored = true;
				LOG(LStreaming, LError, "---- Packet from camera on port %d that has not been set up yet!\n", port);
				return;
			}
			if (!camera->sync && (packet.header.tag == PACKET_FRAME_SIGNAL || packet.header.isStreamPacket()))
			{
				packet.ignored = true;
				LOG(LStreaming, LTrace, "Camera %d (Port %d) send a streaming packet but was not set up for streaming!\n", camera->id, port);
				return;
			}

			if (packet.header.tag == PACKET_FRAME_SIGNAL)
			{ // Frame is starting to be processed
				auto sync_lock = camera->sync->contextualLock();
				SyncedFrame *frame = RegisterCameraFrame(*sync_lock, camera->syncIndex, packet.header.frameID);
				if (!frame)
				{
					packet.ignored = true;
				}
				else
				{
					LOG(LStreaming, LTrace, "Camera %d (Port %d) announced packet for frame %d (%d)!\n", camera->id, port, frame->ID, packet.header.frameID&0xFF);
				}
			}
			else if (packet.header.isStreamPacket())
			{ // Register that camera is receiving frame data
				if (!RegisterStreamPacket(*camera->sync->contextualLock(), camera->syncIndex, packet.header.frameID, packetState.receiveTime))
					packet.ignored = true;
			}
		},
		[](void *userData, int port, PacketBlocks &packet, uint8_t *data, int length) { // onPacketBlock
			if (packet.ignored) return;
			PacketState &packetState = *((PacketState*)userData);
			std::shared_ptr<TrackingCameraState> &camera = packetState.controller->cameras[port];
			if (!camera)
			{
				packet.ignored = true;
				LOG(LStreaming, LError, "---- Camera on port %d got removed since packet was first received!\n", port);
				return;
			}

			if (packet.header.isStreamPacket())
			{ // Update statistics of streaming packet
				if (!RegisterStreamBlock(*camera->sync->contextualLock(), camera->syncIndex, packet.header.frameID))
				{
					LOG(LParsing, LError, "---- Received stream block for non-existant frame %d or unregistered stream packet!\n", packet.header.frameID);
					packet.ignored = true;
				}
			}
		},
		[](void *userData, int port, PacketBlocks &packet) { // onPacketDone
			if (packet.ignored) return;
			PacketState &packetState = *((PacketState*)userData);
			std::shared_ptr<TrackingCameraState> &camera = packetState.controller->cameras[port];
			if (!camera)
			{
				packet.ignored = true;
				LOG(LStreaming, LError, "---- Camera on port %d got removed since packet was first received!\n", port);
				return;
			}

			if (packet.erroneous)
			{
				LOG(LStreaming, LError, "Camera %d had packet %d discarded because of one or more long missing blocks after header %d!\n",
					camera->id, packet.header.tag, packet.headerBlockID);
				if (packet.header.isStreamPacket())
				{
					RegisterStreamPacketComplete(*camera->sync->contextualLock(), camera->syncIndex, packet.header.frameID, {}, true);
				}
				return;
			}
			if (!VerifyChecksum(packet))
				packet.erroneous = true;
			packet.data.resize(packet.header.length-CHECKSUM_SIZE);
			if (packet.header.isStreamPacket())
			{
				auto cameraFrame = ReadStreamingPacket(*camera, packet);
				auto sync_lock = camera->sync->contextualLock();
				SyncedFrame *frame = RegisterStreamPacketComplete(*sync_lock, camera->syncIndex, packet.header.frameID, std::move(cameraFrame), false);
				if (frame)
				{ // Packet existed for frame
					LOG(LStreaming, LTrace, "Camera %d (Port %d) fully transmitted stream packet %d for frame %d (%d)!\n",
						camera->id, port, packet.header.tag, frame->ID, frame->ID&0xFF);
				}
				else
				{
					LOG(LParsing, LError, "---- Completed stream packet for non-existant frame %d or unregistered stream packet!\n", packet.header.frameID);
				}
			}
			else
			{
				std::shared_lock dev_lock(packetState.state->deviceAccessMutex);
				ReadCameraPacket(*camera, packet);
			}
		});
}