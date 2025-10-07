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

#ifndef SERVER_H
#define SERVER_H

#include "config.hpp" // Config
#include "pipeline/pipeline.hpp" // Main processing pipeline
#include "comm/streaming.hpp" // Streaming blob data in
#include "comm/server.hpp"
#include "imu/device.hpp"
#include "recording.hpp"

#include "util/memory.hpp"

#include <thread>
#include <atomic>

/**
 * Main program flow
 */


/* Structures */

// Forward-declared opaque structs
struct TrackingControllerState; // device/tracking_controller.hpp
struct TrackingCameraState; // device/tracking_camera.hpp
class vrpn_Connection; // io/vrpn.hpp
class vrpn_Tracker_AsterTrack; // io/vrpn.hpp
struct libusb_context_int; // comm/usb.hpp

// Defined later
struct ServerState;

extern ServerState StateInstance;
static inline ServerState &GetState() { return StateInstance; }

enum ServerMode
{
	MODE_None = 0,
	MODE_Device,
	MODE_Simulation,
	MODE_Replay
};

struct ServerState
{
	// System State
	ServerMode mode = MODE_None;
	bool isLoading, isStreaming;
	PipelineState pipeline = {};

	// Data stored in files
	GeneralConfig config = {};
	CameraConfigMap cameraConfig;
	ControllerConfig controllerConfig; // TODO: Add support for multiple controllers with different configs
	std::map<int, LensCalib> lensPresets;
	int defaultLens;
	std::vector<CameraCalib> cameraCalibrations;
	std::vector<TrackerConfig> trackerConfigs; // TODO: Synchronised
	std::string wpa_supplicant_conf;
	bool generalConfigDirty, cameraConfigDirty, trackerConfigDirty;
	bool cameraCalibsDirty, trackerCalibsDirty, trackerIMUsDirty;
	bool lensPresetsDirty;
	// TODO: Should not store on disk by default, but provide option to store wifi credentials using secret service on linux 

	// Device setup
	recursive_shared_mutex deviceAccessMutex; // Protects controllers, cameras
	std::vector<std::shared_ptr<TrackingControllerState>> controllers;
	std::vector<std::shared_ptr<TrackingCameraState>> cameras;

	// Device comm state
	libusb_context_int *libusb_context;
	//ServerCommState server;
	Synchronised<StreamState> stream = {};

	// Thread for current mode
	std::jthread *coprocessingThread = NULL;
	std::jthread *rtProcessingThread = NULL;

	// Realtime Processing (only for device mode)
	std::mutex parsing_m, processing_m;
	std::condition_variable parsing_cv, processing_cv;
	// Enables integration of IMU samples past the latest frame for lower latency at the cost of accurracy
	// TODO: Integrate IMU samples ontop of latest frame for lower latency (1/3)
	bool lowLatencyIMU = true;
	// Aims to keep USB thread free for time sync by queuing packets for coprocessing thread to parse
	// Since parsing may take too long due to debug builds, system load, bugs, etc.
	bool usePacketQueue = false;

	// Simulation/Replay control
	std::atomic<int> simAdvance = { -1 };
	std::atomic<bool> simWaiting = { false };
	bool simAdvanceQuickly;
	// Dropout Simulation
	std::atomic<int> simDropoutIndex = { -1 };
	std::vector<float> simDropoutSeverity = { 1 };
	// Loaded records for replay
	struct {
		std::vector<std::string> captures;
		std::vector<std::string> tracking;
		std::vector<RecordingSegment> segments;
		std::size_t frames = 0;
		TimePoint_t replayTime;
	} recording = {};
	TrackingRecord stored;

	// IMU Device integration
	std::mutex hid_access;
	Synchronised<IMUDeviceProviderList> imuProviders;

	// All integrations
	struct 
	{
		std::mutex mutex;

		// VRPN IO
		bool useVRPN = false;
		opaque_ptr<vrpn_Connection> vrpn_server;
		std::map<int, std::shared_ptr<vrpn_Tracker_AsterTrack>> vrpn_trackers;
	} io;
};


/* Functions */

bool ServerInit(ServerState &state);
void ServerExit(ServerState &state);
void ServerUpdatedTrackerConfig(ServerState &state, TrackerConfig &tracker);

std::shared_ptr<TrackingCameraState> EnsureCamera(ServerState &state, CameraID id);
std::shared_ptr<TrackingCameraState> GetCamera(ServerState &state, CameraID id);
CameraMode getCameraMode(ServerState &state, CameraID id);

bool StartDeviceMode(ServerState &state);
void StopDeviceMode(ServerState &state);

void StartServer(ServerState &state);

void StartSimulation(ServerState &state);
void StopSimulation(ServerState &state);

void StartReplay(ServerState &state, std::vector<CameraConfigRecord> cameras);
void StopReplay(ServerState &state);

bool StartStreaming(ServerState &state);
void StopStreaming(ServerState &state);

void SetupIO(ServerState &state);
void ResetIO(ServerState &state);
void CheckTrackingIO(ServerState &state);
void FetchTrackingIO(ServerState &state);
void PushTrackingIO(ServerState &state, std::shared_ptr<FrameRecord> &frame);

#endif // SERVER_H