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
#include <queue>
#include <atomic>
#include <mutex>

#include <linux/fb.h>

#include "util/util.hpp"
#include "util/eigendef.hpp"
#include "camera/gcs.hpp"
#include "comm/packet.hpp"
#include "comm/firmware.hpp"
#include "comm/wireless.hpp"
#include "blob/parameters.hpp"
#include "util/stats.hpp"

struct TrackingCameraMode 
{
	bool streaming = false;
	TrCamMode mode = TRCAM_STANDBY;
	uint8_t opt = TRCAM_OPT_NONE;
};

struct VisualisationState
{
	bool initialised = false;
	bool enabled = false;
	int fbfd = -1;
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
	std::string codeFile = "/home/tc/TrackingCamera/qpu_blob_tiled_min.bin";
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
	CameraID id = 0;
	// UART & Server comm
	bool noComms;
	bool enableUART;
	std::string serialName = "/dev/ttyAMA0";
	std::string server_host = "";
	std::string server_port = "48532";
	WirelessState wireless = {};
	FirmwareUpdateState firmware = {};
	FirmwareUpdateFlags postFirmwareActions = FW_FLAGS_NONE;
	// Comm Synchronisation
	std::atomic<bool> updateSetupQPU = { false };
	std::atomic<bool> updateSetupCPU = { false };
	std::atomic<bool> updateMode = { false };
	TrCamMode newModeRaw = TRCAM_STANDBY;
	TrackingCameraMode newMode = {};
	ConfigPacket newConfigPacket;
};

struct FrameSync
{
	std::queue<std::pair<uint8_t, TimePoint_t>> frameSOFs;
	StatDistf SOF2RecvDelay; // Models delay from trigger to receiving the frame through V4L2
	std::mutex access;
};

extern TrackingCameraState state;
extern FrameSync framesync;

#endif // STATE_H