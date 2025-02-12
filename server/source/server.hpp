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
	bool isStreaming;
	PipelineState pipeline = {};

	// Data stored in files
	GeneralConfig config = {};
	CameraConfigMap cameraConfig;
	ControllerConfig controllerConfig; // TODO: Add support for multiple controllers with different configs
	std::vector<CameraCalib> cameraCalibrations;
	std::string wpa_supplicant_conf;
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

	// Simulation/Replay control
	std::atomic<int> simAdvance = { -1 };
	std::atomic<bool> simWaiting = { false };
	bool simAdvanceQuickly;
	// Replay data
	std::string loadedFramePath;
	unsigned int loadedFrameOffset;
	std::vector<FrameRecord> loadedFrameRecords;
	long frameRecordReplayPos;
	TimePoint_t frameRecordReplayTime;

	// All integrations
	struct 
	{
		std::mutex mutex;

		// VRPN IO
		bool useVRPN = false;
		opaque_ptr<vrpn_Connection> vrpn_server;
		std::map<int, opaque_ptr<vrpn_Tracker_AsterTrack>> vrpn_trackers;
	} io;
};


/* Functions */

bool ServerInit(ServerState &state);
void ServerExit(ServerState &state);
void ServerStoreCameraCalib(ServerState &state);
void ServerStoreTargetCalib(ServerState &state);
void ServerStoreConfiguration(ServerState &state);

std::shared_ptr<TrackingCameraState> EnsureCamera(ServerState &state, CameraID id);
std::shared_ptr<TrackingCameraState> GetCamera(ServerState &state, CameraID id);
CameraMode getCameraMode(ServerState &state, CameraID id);
std::shared_ptr<TrackingCameraState> DeviceSetupCamera(ServerState &state, CameraID id);
bool DeviceCheckCameraDisconnect(ServerState &state, TrackingCameraState &camera);
void DevicesUpdateSyncMask(ServerState &state);

bool StartDeviceMode(ServerState &state);
void StopDeviceMode(ServerState &state);

void StartServer(ServerState &state);

void DeviceUpdateCameraSetup(ServerState &state, TrackingCameraState &device);
bool DeviceUpdateWireless(ServerState &state, TrackingCameraState &device);
void DeviceUpdateStream(TrackingCameraState &state);
void DeviceUpdateVis(TrackingCameraState &state);
bool DevicesAwaitModeChanges(int timeoutMS);

void StartSimulation(ServerState &state);
void StopSimulation(ServerState &state);

void StartReplay(ServerState &state, std::vector<CameraConfigRecord> cameras);
void StopReplay(ServerState &state);

bool StartStreaming(ServerState &state);
void StopStreaming(ServerState &state);

void SetupIO(ServerState &state);
void ResetIO(ServerState &state);
void UpdateTrackingIO(ServerState &state, std::shared_ptr<FrameRecord> &frame);

#endif // SERVER_H