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

#ifndef STATE_H
#define STATE_H

#include <string>
#include <cstdint>
#include <vector>
#include <queue>
#include <atomic>
#include <mutex>

#include <linux/fb.h>

#include "util/util.hpp"
#include "util/eigendef.hpp"
#include "camera/gcs.hpp"
#include "comm/packet.hpp"
#include "comm/protocol_stream.hpp"
#include "comm/timesync.hpp"
#include "blob/parameters.hpp"

struct TrackingCameraMode 
{
	bool streaming = false;
	TrCamMode mode = TRCAM_STANDBY;
	uint8_t opt = TRCAM_OPT_NONE;
};

struct CommState 
{
	std::mutex writeAccess;
	bool enabled = false, started = false, ready = false, writing = false;
	uint8_t checksum;
	ProtocolState protocol = {};
	bool rsp_id = false, rsp_ack = false;
	int timeout = 0, timeout_send = 0;
	IdentPacket ownIdent = {};
	IdentPacket expIdent = {};
	IdentPacket otherIdent = {};

	void *port;
	bool (*start)(void *port);
	int (*wait)(void *port, uint32_t timeoutUS);
	void (*stop)(void *port);
	int (*read)(void *port, uint8_t *data, uint32_t length);
	int (*write)(void *port, const uint8_t *data, uint32_t length);
	void (*submit)(void *port);
	void (*flush)(void *port);
};

struct VisualisationState
{
	bool initialised = false;
	bool enabled = false;
	int fbfd = 0;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	bool displayBlobs = false;
	bool displayFrame = false;
	int width = 640, height = 480;
	int interval = 10;
};

struct ImageStreamState
{
	bool enabled = false;
	Bounds2<int> bounds;
	uint8_t subsampling;
	uint8_t jpegQuality;
	uint16_t frame; // Either interval, or 8bits of past frameID to send
};

struct WirelessState
{
	bool enabled;
	bool SSH, Server;
	// Actual reported state:
	bool dirty, updating, needsStatusPacket;
	bool connected, failed;
	std::string SSID, IP, error;
};

struct TrackingCameraState
{
	TrackingCameraMode curMode = {};
	// Camera options
	std::string camName = "/dev/video0";
	GCS_CameraParams camera = {
		.devName = (char*)camName.c_str(),
		.width = 1280,
		.height = 800,
		.stride = 0, // Will be set by gcs
		.fps = 144,
		.shutterSpeed = 100,
		.digitalGain = 1,
		.analogGain = 1,
		.grayscale = 1,
		.extTrig = 0,
		.strobe = 1,
		.strobeOffset = 0,
		.strobeLength = 10
	};
	// QPU options
	std::string codeFile = "qpu_blob_tiled_min.bin";
	bool enableQPU[12] = { 1,1,1,0, 1,1,0,1, 1,0,1,0 };
	// TODO: Some QPUs do not work with this qpu code (bug), disable them
	// Do not waste too much time on this, I already did. These cores don't work with this specific code
	// Feel free to scour https://github.com/Seneral/VC4CV for more info / debugging, not sure there is much
	// As far as I remember, the VPM writes seem to write wrong output on just these cores
	// CPU options
	ThresholdingParameters thresholds; // Copy separate for QPU thread
	BlobProcessingParameters blobParams;
	// Visualisation options
	VisualisationState visualisation = {};
	// Frame streaming state
	ImageStreamState streaming = {};
	std::queue<ImageStreamState> imageRequests;
	// Logging options
	bool writeStatLogs = true;
	// Unique ID stored on-device (generated on first boot)
	int32_t id = 0;
	// UART & Server comm
	CommState uart = {};
	std::string serialName = "/dev/ttyAMA0";
	CommState server = {};
	std::string server_host = "";
	std::string server_port = "8888";
	WirelessState wireless = {};
	// Comm Synchronisation
	std::atomic<bool> updateSetupQPU = { false };
	std::atomic<bool> updateSetupCPU = { false };
	std::atomic<bool> updateMode = { false };
	TrCamMode newModeRaw = TRCAM_STANDBY;
	TrackingCameraMode newMode = {};
	ConfigPacket newConfigPacket;
	struct {
		std::queue<std::pair<uint32_t, TimePoint_t>> frameSOFs;
		StatDistf SOF2RecvDelay; // Models delay from trigger to receiving the frame through V4L2
		TimeSync time;
		std::mutex access;
	} sync = {};
};

#endif // STATE_H