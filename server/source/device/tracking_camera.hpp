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

#ifndef TRACKING_CAMERA_H
#define TRACKING_CAMERA_H

#include "comm/streaming.hpp" // SyncGroup, FrameID, TruncFrameID
#include "comm/packet.hpp" // StatPacket
#include "pipeline/record.hpp"
#include "camera_firmware.hpp"

#include "util/eigendef.hpp"
#include "util/synchronised.hpp"
#include "util/util.hpp" // TimePoint_t

#include <atomic>
#include <vector>

// Forward-declared opaque structs
struct ServerState; // server.hpp
struct TrackingControllerState; // device/tracking_controller.hpp
struct CameraPipeline; // pipeline/pipeline.hpp
struct ClientCommState; // comm/server.hpp

struct ImageRequest
{
	Bounds2i bounds;
	uint8_t subsampling;
	uint8_t jpegQuality;
	uint16_t frame;				// Either interval, or 8bits of past frameID to send (tbd)
};

struct TrackingCameraState
{
	CameraID id = CAMERA_ID_NONE;
	std::string label;

	// Pipeline State
	std::shared_ptr<CameraPipeline> pipeline;

	// Indirect communication channel over controller
	std::shared_ptr<TrackingControllerState> controller;
	int port = -1;
	// Direct communication channel over TCP
	ClientCommState *client = NULL;

	// Realtime streaming state 
	std::shared_ptr<Synchronised<SyncGroup>> sync;
	int syncIndex;

	// Device mode
	TrCamMode mode = TRCAM_STANDBY;
	struct {
		TrCamMode mode;
		TimePoint_t time;
		bool handleIndividually = true;
	} modeSet = {};
	inline bool hasSetMode(TrCamMode Mode) const { return (modeSet.mode&TRCAM_MODE_MASK) & Mode; }
	inline bool isMode(TrCamMode Mode) const { return (mode&TRCAM_MODE_MASK) & Mode; }
	inline bool hasSetStreaming() const { return (modeSet.mode&TRCAM_FLAG_STREAMING) == TRCAM_FLAG_STREAMING; }
	inline bool isStreaming() const { return (mode&TRCAM_FLAG_STREAMING) == TRCAM_FLAG_STREAMING; }

	bool sendPacket(PacketTag tag, uint8_t *data, unsigned int length, bool writeChecksum);
	bool sendModeSet(uint8_t mode, bool handleIndividually = true);
	void recvModeSet(uint8_t mode);

	enum BackgroundCalibOpt { BG_CALIB, BG_RESET, BG_ACCEPT, BG_DISCARD };
	bool updateBackgroundCalib(BackgroundCalibOpt opt);

	/**
	* Device configuration
	*/

	struct ImageStreaming
	{ // Camera image streaming configuration
		bool enabled;
		bool adaptive = true;		// Adapt to zoomed view instead of always sending full screen
		int resolutionScale = 3;	// Preference for resolution scaling (depending frame or view resolution)
		bool focusQuality;			// Whether to prefer image quality instead of framerate

		// Current settings requested for camera image - set based on above settings
		ImageRequest request;
	};

	struct HDMIVis
	{ // HDMI Visualisation configuration
		bool enabled;
		int width = 480, height = 400, fps = 10;
		bool displayFrame = true, displayBlobs = true;
	};

	struct Wireless
	{ // Wireless configuration
		WirelessConfig config = WIRELESS_CONFIG_SSH;
		// Actual reported state:
		WirelessConfigStatus wifiStatus, sshStatus, serverStatus;
		std::string SSID, IP, error;
		TimePoint_t sendTime;
		TimePoint_t errorTime;
	};

	struct 
	{
		ImageStreaming imageStreaming = {};
		HDMIVis hdmiVis = {};
		Wireless wireless = {};
	} config = {};

	bool selectedForFirmware;
	CameraFirmwareUpdateRef firmware;

	/**
	* Receiving state
	*/

	struct VisualDebug
	{ // Visual Debug data sent by the TrackingCamera
		bool hasBlob;
		// Normal blob data
		Eigen::Vector2f pos; // In image space
		float size;
		// Bounds
		Bounds2i bounds; // In pixel space
		Bounds2i lastBounds; // In pixel space
		Eigen::Vector2f offset; // To last bounds, in pixel space
		// Point debug
		int refinementMethod;
		int dbgPtCount;
		// Grayscale image and binary mask
		std::vector<uint8_t> image;
		std::vector<uint8_t> mask;
		std::vector<Eigen::Vector2f> boundPoints;
	};

	struct BackgroundCalib
	{ // Background calibration data sent by the TrackingCamera
		std::vector<Eigen::Vector2f> tiles;
		float tileSize;
	};

	struct ImageReceiving
	{ // Camera image currently being received from the TrackingCamera
		FrameID frameID;
		std::vector<uint8_t> jpeg;
		int received;
		bool erroneous;
	};

	struct
	{
		Synchronised<VisualDebug> visualDebug = {};
		Synchronised<BackgroundCalib> background = {};
		std::shared_ptr<CameraImageRecord> latestFrameImageRecord = nullptr;
		std::shared_ptr<const CameraImage> latestFrameImage = nullptr;
		ImageReceiving parsingFrameImage = {};
		Synchronised<StatPacket> statistics = {};
	} receiving = {};


	/**
	* Camera Comm & Error Status
	*/

	struct Errors
	{ // Camera error data sent by the TrackingCamera
		bool encountered, recovered;
		ErrorTag code;
		bool serious;
		TimePoint_t time, recoverTime;
	};
	struct Status
	{
		bool hadPiConnected, hadMCUConnected;
		TimePoint_t lastConnected, lastConnecting;
		ControllerCommState commState;
		Errors error;
	};
	Synchronised<Status> state = {};
};

bool CameraCheckDisconnected(ServerState &state, TrackingCameraState &camera);

/**
 * Attempt to re-start streaming after failure
 */
bool CameraRestartStreaming(ServerState &state, std::shared_ptr<TrackingCameraState> &camera);

void CameraUpdateSetup(ServerState &state, TrackingCameraState &device);
bool CameraUpdateWireless(ServerState &state, TrackingCameraState &device);
void CameraUpdateStream(TrackingCameraState &device);
void CameraUpdateVis(TrackingCameraState &device);

#endif // TRACKING_CAMERA_H
