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
#include "comm/wireless_server_client.hpp"

#include "comm/usb.hpp"
#include "comm/uart.h"

#include "ui/shared.hpp" // Signals

#include "util/log.hpp"

bool TrackingCameraState::hasComms()
{
	if (client && client->ready) return true;
	if (controller && state.contextualRLock()->commState == COMM_SBC_READY) return true;
	return false;
}

bool TrackingCameraState::sendPacket(PacketTag tag, uint8_t *data, unsigned int length)
{
	if (state.contextualRLock()->error.encountered)
	{
		LOG(LCameraDevice, LError, "Cannot send packets to Camera %d because it is still recovering from an error!", id);
		return false; // Cannot handle packet at this time, waiting for recovery
	}
	if (!hasComms())
	{
		LOG(LCameraDevice, LError, "Cannot send packets to Camera %d because it is not started up yet!", id);
		return false;
	}
	if (client && client->ready && length > 100)
	{ // Prefer wireless for "larger" packets
		return comm_write(*client, tag, data, (uint16_t)length);
	}
	else if (controller && state.contextualRLock()->commState == COMM_SBC_READY)
	{ // Prefer UART for small packets
		thread_local std::vector<uint8_t> packetBuffer;
		if (length > 0)
		{
			packetBuffer.resize(length + PACKET_CHECKSUM_SIZE);
			memcpy(packetBuffer.data(), data, length);
			if (tag >= PACKET_HOST_COMM)
				calculateForwardPacketChecksum(packetBuffer.data(), length, packetBuffer.data()+length);
			else // We should not be sending these packets, but do allow for it
				calculateDirectPacketChecksum(packetBuffer.data(), length, packetBuffer.data()+length);
			LOG(LCameraDevice, LTrace, "Sending packet to camera with checksum %.8x!\n", *(uint32_t*)(packetBuffer.data()+length));
		}
		else
			packetBuffer.clear();
		return comm_submit_control_data(controller->comm, COMMAND_OUT_SEND_PACKET,
			(uint16_t)tag, (uint16_t)(1<<port), packetBuffer.data(), (uint16_t)packetBuffer.size()) >= 0;
	}
	else if (client && client->ready)
	{ // Fall back to wireless for small packets
		return comm_write(*client, tag, data, (uint16_t)length);
	}
	else
		return false;
}

bool TrackingCameraState::sendModeSet(uint8_t setMode, bool handleIndividually)
{
	if (!hasComms())
	{
		LOG(LCameraDevice, LError, "Camera %d requesting mode failed because it has not started up yet!", id);
		return false;
	}
	/* if (mode != modeSet.mode && dtMS(modeSet.time, sclock::now()) < 500)
	{
		LOG(LCameraDevice, LWarn, "Camera %d requesting mode %x while still waiting on mode %x requested %fms ago!",
			id, setMode, modeSet.mode, dtMS(modeSet.time, sclock::now()));
		return false;
	} */
	if (modeSet.mode == setMode && dtMS(modeSet.time, sclock::now()) < 500)
	{
		LOG(LCameraDevice, LWarn, "Camera %d already requested mode %x %fms ago!",
			id, modeSet.mode, dtMS(modeSet.time, sclock::now()));
		return false;
	}
	uint8_t modePacket[1];
	modePacket[0] = setMode;
	if (!sendPacket(PACKET_CFG_MODE, modePacket, sizeof(modePacket)))
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
	// NOTE: recvMode might genuinely be != modeSet
	// E.g. error may force camera to change mode on its own
	// Or some modes, e.g. BGCALIB with Options ACCEPT/DISCARD, proceed to enter different modes (BLOB)
	mode = (TrCamMode)recvMode;
	modeSet.mode = (TrCamMode)recvMode;
	if (toggleStreaming && modeSet.handleIndividually)
	{ // If not handled in bulk, update camera status here
		if (controller)
			ControllerUpdateSyncMask(*controller);
	}
	LOG(LCameraDevice, LInfo, "Camera %d entered mode %x!", id, mode);
}

bool TrackingCameraState::updateBackgroundCalib(BackgroundCalibOpt opt)
{
	switch (opt)
	{
		case BG_CALIB:
			receiving.background.contextualLock()->tiles.clear();
			if (GetState().isStreaming)
				return sendModeSet(TRCAM_FLAG_STREAMING | TRCAM_MODE_BGCALIB);
		case BG_RESET:
			receiving.background.contextualLock()->tiles.clear();
			return sendModeSet((GetState().isStreaming? TRCAM_FLAG_STREAMING : 0) | TRCAM_MODE_BGCALIB | TRCAM_OPT_BGCALIB_RESET);
		case BG_ACCEPT:
			return sendModeSet((GetState().isStreaming? TRCAM_FLAG_STREAMING : 0) | TRCAM_MODE_BGCALIB | TRCAM_OPT_BGCALIB_ACCEPT);
		case BG_DISCARD:
			receiving.background.contextualLock()->tiles.clear();
			return sendModeSet((GetState().isStreaming? TRCAM_FLAG_STREAMING : 0) | TRCAM_MODE_BGCALIB | TRCAM_OPT_BGCALIB_DISCARD);
	}
	return false;
}

static bool CameraControllerDisconnected(const ServerState &state, const TrackingCameraState &camera, const TrackingCameraState::Status &camStatus)
{
	if (!camera.controller)
		return true; // Lost all controller association
	if ((camStatus.commState&COMM_READY) == COMM_READY)
		return false; // Currently connected
	if (dtMS(camStatus.lastConnecting, sclock::now()) < 1000)
		return false; // Show camera reconnecting message (might be switching between MCU and Pi)
	if (dtMS(camStatus.lastConnected, sclock::now()) < 5000)
		return false; // Show camera lost message
	return true;
}

static bool CameraServerDisconnected(const ServerState &state, const TrackingCameraState &camera, const TrackingCameraState::Status &camStatus)
{
	if (camera.client && camera.client->ready)
		return false; // Definitely still connected
	if (camStatus.hadServerConnected && dtMS(camStatus.lastWirelessConnection, sclock::now()) < 10000)
		return false;
	return true;
}

bool CameraCheckDisconnected(ServerState &state, TrackingCameraState &camera)
{
	TrackingCameraState::Status camStatus = *camera.state.contextualRLock(); // Copy
	if (state.mode == MODE_Device)
	{ // Allow certain device states to object to removal
		if (!CameraServerDisconnected(state, camera, camStatus))
			return false;
		if (!CameraControllerDisconnected(state, camera, camStatus))
			return false;
		if (dtMS(camStatus.lastDeviceChange, sclock::now()) < 1000)
		{ // Prevent high-frequency state changes
			// Case 1: Just ran EnsureCamera but comms were not set up yet (race condition, though is has been mitigated)
			// Case 2: Camera moved between two controllers and both are still claiming it. Wait for comm timeout on old one 
			return false;
		}
		if (camStatus.error.encountered && dtMS(camStatus.error.time, sclock::now()) < 10000)
			return false; // Show camera error message
		if (camera.firmware)
		{
			auto fw = camera.firmware->contextualRLock();
			if (fw->code == FW_STATUS_REQAPPLY || fw->code == FW_STATUS_UPDATING)
			{ // Don't remove while it might still be updating
				if (dtMS(fw->lastActivity, sclock::now()) < 50000)
				{
					// TODO: Protect camera ports doing firmware update
					return false;
				}
			}
			// Even then, info might still be relevant for the UI
			if (dtMS(fw->lastActivity, sclock::now()) < 10000)
			{
				// TODO: Protect camera ports doing firmware update
				return false;
			}
			// TODO: Protect camera ports doing firmware update
			// May release protection here
		}
	}
	// Can safely remove camera

	std::unique_lock dev_lock(state.deviceAccessMutex); // cameras

	if (camera.controller)
	{ // Disassociate from controller
		camera.controller->cameras[camera.port] = nullptr;
		camera.controller = nullptr;
		camera.port = -1;
	}

	// Clean up stream state
	RemoveCameraSync(*state.stream.contextualLock(), camera);

	// Keep pipeline camera for the remainder of the session

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
	if (!camera->hasComms())
	{
		LOG(LCameraDevice, LError, "Camera %d cannot be set to stream again because it has not started up yet!", camera->id);
		return false;
	}

	// Setup sync group
	// TODO: Setup sync groups in EnsureCamera (based on prior config, e.g. in UI) 3/4
	// Remove this, should already be set up at this point
	auto &config = state.cameraConfig.getCameraConfig(camera->id);
	if (config.synchronised && camera->controller && camera->controller->sync)
		SetCameraSync(*state.stream.contextualLock(), camera, camera->controller->sync);
	else
		SetCameraSyncNone(*state.stream.contextualLock(), camera, 1000.0f / config.framerate);

	// Send setup data incase it didn't already happen
	CameraUpdateSetup(state, *camera);

	// Make sure secondary features match expected state
	CameraUpdateStream(*camera);
	CameraUpdateVis(*camera);

	// Give camera some time to configure itself
	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// Communicate with camera to start streaming (startup time is high)
	return camera->sendModeSet(TRCAM_FLAG_STREAMING | TRCAM_MODE_BLOB);
}

// ----------------------------------------------------------------------------
// Device Comms
// ----------------------------------------------------------------------------

void CameraUpdateSetup(ServerState &state, TrackingCameraState &device)
{
	CameraConfig &config = state.cameraConfig.getCameraConfig(device.id);
	ConfigPacket packet = {};
	packet.width = config.width;
	packet.height = config.height;
	if (device.sync)
	{ // Use already set-up sync groups, do not parse config again
		auto sync_lock = device.sync->contextualRLock();
		packet.fps = 1000.0f / sync_lock->frameIntervalMS;
		packet.extTrig = sync_lock->source != SYNC_NONE;
	}
	else
	{ // Should not happen, all cameras should have a sync group
		LOG(LCameraDevice, LError, "Sync group of camera has not been set up yet!");
		packet.fps = config.framerate;
		packet.extTrig = false;
	}
	packet.shutterSpeed = config.exposure;
	packet.analogGain = config.gain;
	packet.strobe = config.enableStrobe;
	// TODO: Implement control over filter switcher on both Pi and STM32
	//packet.filter = 
	packet.strobeOffset = config.strobeOffset;
	packet.strobeLength = config.strobeLength;
	packet.blobProc = config.blobProcessing;
	uint8_t setup[CONFIG_PACKET_SIZE];
	storeConfigPacket(packet, setup);
	device.sendPacket(PACKET_CFG_SETUP, setup, sizeof(setup));
}

bool CameraUpdateWireless(ServerState &state, TrackingCameraState &device, WirelessAction action)
{
	// Send wireless config to Tracking Camera
	auto &wireless = device.config.wireless;

	std::vector<WirelessAction> actions;
	if (action != WIRELESS_ACTION_NONE)
		actions.push_back(action);

	bool sendCreds = wireless.setConfig & WIRELESS_CONFIG_WIFI;
	std::size_t credSize = sendCreds? state.wpa_supplicant_conf.size() : 0;
	if (credSize > 2000)
	{ // TODO: wpa_supplicant limited by CTRL_TRANSFER_SIZE and USBD_CTRL_MAX_PACKET_SIZE
		LOG(LGUI, LWarn, "wpa_supplicant is too long to send over control transfers!");
		credSize = 0;
		return false;
	}
	bool serverEnabled = wireless.setConfig & WIRELESS_CONFIG_SERVER;
	if (state.server.host.empty())
		state.server.host = WirelessServerGetHostname();
	assert(!state.server.portUsed.empty());
	std::string hostStr = state.server.host + ":" + state.server.portUsed;
	uint8_t hostSize = serverEnabled? hostStr.size() : 0;
	std::vector<uint8_t> packet(WIRELESS_PACKET_HEADER + credSize + hostSize + actions.size());
	packet[0] = wireless.setConfig;
	packet[1] = actions.size();
	packet[2] = credSize >> 8;
	packet[3] = credSize & 0xFF;
	packet[4] = hostSize;
	// 3 free byte for future use
	for (int i = 0; i < actions.size(); i++)
		packet[WIRELESS_PACKET_HEADER+i] = actions[i];
	if (credSize > 0)
		memcpy(packet.data()+WIRELESS_PACKET_HEADER+actions.size(), state.wpa_supplicant_conf.data(), credSize);
	if (hostSize > 0)
		memcpy(packet.data()+WIRELESS_PACKET_HEADER+actions.size()+credSize, hostStr.data(), hostSize);

	wireless.sendTime = sclock::now();
	wireless.error.clear(); // Clear past failure
	device.sendPacket(PACKET_CFG_WIFI, packet.data(), packet.size());
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
	std::vector<uint8_t> stream(mode > 0? 12 : 1);
	stream[0] = mode;
	if (mode > 0)
	{
		*((uint16_t *)&stream[1]) = request.bounds.min.x();
		*((uint16_t *)&stream[3]) = request.bounds.min.y();
		*((uint16_t *)&stream[5]) = request.bounds.max.x();
		*((uint16_t *)&stream[7]) = request.bounds.max.y();
		stream[9] = request.subsampling;
		stream[10] = request.jpegQuality;
		stream[11] = request.frame;
	}
	device.sendPacket(PACKET_CFG_IMAGE, stream.data(), stream.size());
}

void CameraUpdateStream(TrackingCameraState &device)
{
	CameraSendImageRequest(device, device.config.imageStreaming.enabled, device.config.imageStreaming.request);
}

void CameraUpdateVis(TrackingCameraState &device)
{
	// Send vis config to Tracking Camera
	auto &config = device.config.hdmiVis;
	std::vector<uint8_t> vis(config.enabled? 8 : 1);
	vis[0] = config.enabled;
	if (vis[0])
	{ // Width, Height, interval in frames
		*((uint16_t *)&vis[1]) = config.width;
		*((uint16_t *)&vis[3]) = config.height;
		vis[5] = config.fps;
		vis[6] = config.displayFrame;
		vis[7] = config.displayBlobs;
	}
	device.sendPacket(PACKET_CFG_VIS, vis.data(), vis.size());
}

std::vector<std::string> CameraDescribeInfo(const CameraStoredInfo &info)
{
	std::vector<std::string> desc;
	if (info.sbcFWVersion.num == 0 && info.mcuFWVersion.num == 0)
		return desc;
	desc.push_back(asprintf_s("SBC Firmware v%d.%d.%d (Build %.2x - %s)",
		info.sbcFWVersion.major, info.sbcFWVersion.minor, info.sbcFWVersion.patch, info.sbcFWVersion.build, info.sbcFWDescriptor.c_str()));
	desc.push_back(asprintf_s("MCU Firmware v%d.%d.%d (Build %.2x - %s)",
		info.mcuFWVersion.major, info.mcuFWVersion.minor, info.mcuFWVersion.patch, info.mcuFWVersion.build, info.mcuFWDescriptor.c_str()));
	desc.push_back(asprintf_s("SBC: %s", info.sbcHWDescriptor.c_str()));
	if (!info.mcuHWDescriptorParts.empty())
	{
		desc.push_back(asprintf_s("Hardware Descriptor: %s", info.mcuHWDescriptorParts.front().c_str()));
		for (int i = 1; i < info.mcuHWDescriptorParts.size(); i++)
			desc.push_back(asprintf_s("    Appended: %s", info.mcuHWDescriptorParts[i].c_str()));
	}
	desc.push_back(asprintf_s("Hardware Serial Number: %.8x%.8x%.8x",
		info.hardwareSerial.serial32[0], info.hardwareSerial.serial32[1], info.hardwareSerial.serial32[2]));
	for (int i = 0; i < info.subpartSerials.size(); i++)
		//if (info.subpartSerials[i] != (uint64_t)-1)
		desc.push_back(asprintf_s("    Subpart %d: %.8x%.8x", i, (uint32_t)(info.subpartSerials[i] >> 32), (uint32_t)info.subpartSerials[i]));
	desc.push_back(asprintf_s("SBC Revision Code %x, Serial Number: %.8x",
		info.sbcRevisionCode & 0xFFFFFF, info.sbcSerialNumber));
	desc.push_back(asprintf_s("MCU Unique ID: %.8x%.8x%.8x",
		info.mcuUniqueID[0], info.mcuUniqueID[1], info.mcuUniqueID[2]));
	return desc;
}