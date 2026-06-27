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

#include "visualisation.hpp"
#include "processing/qpu_cores_masking.hpp"
#include "util/eigendef.hpp"
#include "camera/gcs.hpp"
#include "comm/packet.hpp"
#include "comm/firmware.hpp"
#include "comm/wireless.hpp"
#include "blob/parameters.hpp"

struct TrackingCameraMode 
{
	bool streaming = false;
	TrCamMode mode = TRCAM_STANDBY;
	uint8_t opt = TRCAM_OPT_NONE;
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
		.strobeLength = 10,
		// Required for QPU masking:
		.padB = 2, 
		.padValue = 0xFF,
	};
	// QPU options
	std::string qpuProgBin = "/home/tc/TrackingCamera/qpu_blob_tiled_min.bin";
	QPUCoreMasking qpuCores = {};
	bool noQPU = false;
	bool verifyQPU = false;
	bool displayVerifyMask = false;
	// VPU options
	std::string vpuProgBin = "/home/tc/TrackingCamera/vpu_programs.bin";
	bool noVPU = false;
	bool verifyVPU = false;
	// CPU options
	ThresholdingParameters thresholds; // Copy separate for QPU thread
	BlobProcessingParameters blobParams = {
		// Set default parameters when run interactively
		.classification = {
			.resegmentSingleClusters = false,
			.blobRefinementThreshold = 100000000,
		}
	};
	// Visualisation options
	VisualisationState visualisation = {};
	// Frame streaming state
	ImageStreamState streaming = {};
	std::queue<ImageStreamState> imageRequests;
	// Logging options
	bool writeStatLogs = true;
	// MCU control
	bool noMCU = false;
	bool probeMode = false;
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
	// Comm Synchronisation
	std::atomic<bool> updateSetupQPU = { false };
	std::atomic<bool> updateSetupCPU = { false };
	std::atomic<bool> updateMode = { false };
	TrCamMode newModeRaw = TRCAM_STANDBY;
	TrackingCameraMode newMode = {};
	ConfigPacket newConfigPacket;
};


extern TrackingCameraState state;

bool options_read(TrackingCameraState &state, int argc, char **argv);
void acceptCPUConfig(TrackingCameraState &state);
void acceptQPUConfig(TrackingCameraState &state);

#endif // STATE_H