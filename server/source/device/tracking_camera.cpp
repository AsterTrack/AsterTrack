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
#include "tracking_camera.hpp"
#include "tracking_controller.hpp"

#include "comm/usb.hpp"

#include "ui/shared.hpp" // Signals

#include "util/log.hpp"

bool TrackingCameraState::sendPacket(PacketTag tag, uint8_t *data, unsigned int length)
{
	if (state.error.contextualRLock()->encountered)
		return false; // Cannot handle packet at this time, waiting for recovery
	if (controller)
	{
		return comm_submit_control_data(controller->comm, COMMAND_OUT_SEND_PACKET,
			(uint16_t)tag, (uint16_t)(1<<port), data, (uint16_t)length) >= 0;
	}
	/* else if (client && client->ready)
	{
		return comm_writeHeader(*client, tag, (uint16_t)length)
			&& comm_write(*client, data, (uint16_t)length);
	} */
	else
		return false;
}

bool TrackingCameraState::sendModeSet(uint8_t setMode, bool handleIndividually)
{
	if (mode != modeSet.mode)
	{
		LOG(LCameraDevice, LWarn, "Camera %d requesting mode %x while still waiting on mode %x requested %fms ago!",
			id, setMode, modeSet.mode, dtMS(modeSet.time, sclock::now()));
		return false;
	}
	if (modeSet.mode == setMode)
	{
		LOG(LCameraDevice, LWarn, "Camera %d already requested mode %x %fms ago!",
			id, modeSet.mode, dtMS(modeSet.time, sclock::now()));
		return false;
	}
	if (!sendPacket(PACKET_CFG_MODE, &setMode, 1))
	{
		LOG(LCameraDevice, LWarn, "Camera %d failed to send packet to request mode %x!", id, setMode);
		return false;
	}
	modeSet = { (TrCamMode)setMode, sclock::now(), handleIndividually };
	return true;
}

void TrackingCameraState::recvModeSet(uint8_t recvMode)
{
	bool toggleStreaming = (mode & TRCAM_FLAG_STREAMING) != (recvMode & TRCAM_FLAG_STREAMING);
	if (modeSet.mode != (TrCamMode)recvMode)
	{ // Might happen due to error message
		LOG(LCameraDevice, LWarn, "Camera %d left mode %x and entered mode %x which was not requested (mode %x requested %fms ago)!",
			id, mode, recvMode, modeSet.mode, dtMS(modeSet.time, sclock::now()))
	}
	mode = (TrCamMode)recvMode;
	modeSet.mode = (TrCamMode)recvMode;
	if (toggleStreaming && modeSet.handleIndividually)
	{ // If not handled in bulk, update camera status here
		if (controller)
			ControllerUpdateSyncMask(*controller);
	}
	LOG(LCameraDevice, LInfo, "Camera %d entered mode %x!", id, mode);
}

std::shared_ptr<TrackingCameraState> CameraSetupDevice(ServerState &state, CameraID id)
{
	std::shared_ptr<TrackingCameraState> camera = EnsureCamera(state, id);

	// Sync setup data
	CameraUpdateSetup(state, *camera);

	// Make sure secondary features match expected state
	CameraUpdateStream(*camera);
	CameraUpdateVis(*camera);

	return camera;
}

bool CameraCheckDisconnected(ServerState &state, TrackingCameraState &camera)
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

bool CameraRestartStreaming(ServerState &state, std::shared_ptr<TrackingCameraState> &camera)
{
	if (camera->controller && !camera->controller->comm->commStreaming)
	{
		LOG(LCameraDevice, LError, "Camera %d cannot be set to stream again because controller is not in streaming mode anymore!", camera->id);
		return false;
	}

	// Send setup data incase it didn't already happen
	CameraUpdateSetup(state, *camera);

	// Make sure secondary features match expected state
	CameraUpdateStream(*camera);
	CameraUpdateVis(*camera);

	// Give camera some time to configure itself
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// Setup sync group
	if (state.cameraConfig.getCameraConfig(camera->id).synchronised && camera->controller
		&& camera->controller->sync && camera->controller->sync->contextualRLock()->source != SYNC_NONE)
	{ // TODO: Setup sync groups in EnsureCamera (based on prior config, e.g. in UI) 3/3 - Remove this
		SetCameraSync(*state.stream.contextualLock(), camera, camera->controller->sync);
	}
	else {
		SetCameraSyncNone(*state.stream.contextualLock(), camera, 1000.0f / 144);
	}

	// Communicate with camera to start streaming (startup time is high)
	return camera->sendModeSet(TRCAM_FLAG_STREAMING | TRCAM_MODE_BLOB);
}

// ----------------------------------------------------------------------------
// Device Comms
// ----------------------------------------------------------------------------

void CameraUpdateSetup(ServerState &state, TrackingCameraState &device)
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
		// Same in SetupSyncGroups
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
bool CameraUpdateWireless(ServerState &state, TrackingCameraState &device)
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
static void CameraSendImageRequest(TrackingCameraState &device, uint8_t mode, ImageRequest &request)
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
void CameraUpdateStream(TrackingCameraState &device)
{
	CameraSendImageRequest(device, device.config.imageStreaming.enabled, device.config.imageStreaming.request);
}
void CameraUpdateVis(TrackingCameraState &device)
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