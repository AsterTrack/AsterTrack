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

#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H

#include "controller_firmware.hpp"
#include "comm/packet.h"
#include "comm/protocol_packet.hpp" // PacketProtocolState
#include "comm/timingRecord.hpp" // TimingRecord
#include "comm/controller.h"

#include "util/timesync.hpp" // TimeSync
#include "util/blocked_vector.hpp"
#include "util/synchronised.hpp"
#include "util/util.hpp"

#include <vector>
#include <queue>

// Forward-declared opaque structs
struct ServerState; // server.hpp
struct TrackingCameraState; // device/tracking_camera.hpp
struct USBCommState; // comm/usb.hpp
struct SyncGroup; // comm/streaming.hpp

const char *getControllerEventName(ControllerEventID event);

struct ControllerEventLog
{
	uint64_t timestamp;
	double timestampUS;
	TimePoint_t syncedTime;
	ControllerEventID id;
	bool isNewEvent;
};

struct TrackingControllerState
{
	uint32_t id = 0;

	// Cameras connected to it
	std::vector<std::shared_ptr<TrackingCameraState>> cameras;
	int newCamerasConnecting;

	// USB device comms
	std::shared_ptr<USBCommState> comm;

	// USB Control transfer status
	struct USBRequest
	{
		std::atomic<int> transfer = { -1 };
		TimePoint_t submitted;
		bool stalling;
	};
	USBRequest debugReq, eventReq, packetReq, statusReq;

	// State of each USB endpoint for ensuring continuity and time sync
	struct USBEndpoint
	{
		uint8_t counter = -1;
		TimePoint_t lastReceived;
	};
	std::vector<USBEndpoint> endpoints;

	// Queue of packets to parse in supervisor thread, so that USB thread stays free to time usb packets accurately
	struct USBPacket
	{
		TimePoint_t receiveTime;
		int endpoint;
		std::vector<uint8_t> data;
	};
	Synchronised<std::queue<USBPacket>> packetQueue;

	// Port states for assembling packets from USB blocks
	std::vector<PacketProtocolPort> ports;

	// State of TimeSync between host and this controller
	SynchronisedS<TimeSync> timeSync = {};

	// Sync Groups this controller belongs to
	std::shared_ptr<Synchronised<SyncGroup>> sync;
	std::shared_ptr<Synchronised<SyncGroup>> syncGen; // If set, this controller generates/shares the sync for this group

	// Reported device status
	struct
	{
		ControllerStatusFlags flags;
		ControllerSyncConfig syncCfg;
		Hysteresis<float, 1, 0.7f> voltagePD;
		Hysteresis<float, 1, 0.7f> voltageExt;
		ControllerPowerState powerState;
	} status = {};

	// Event Log
	BlockedQueue<ControllerEventLog, 4096> eventLog;
	// TimeSync and Latency Log
	TimingRecord timingRecord = TimingRecord(true, true);

	// Firmware update
	bool selectedForFirmware;
	ControllerFirmwareUpdateRef firmware;
};
static bool operator==(const TrackingControllerState& a, const TrackingControllerState& b) { return &a == &b; }

int DetectNewControllers(ServerState &state);
void DisconnectController(ServerState &state, TrackingControllerState &controller);
void DevicesStartStreaming(ServerState &state);
void DevicesStopStreaming(ServerState &state);
void HandleController(ServerState &state, TrackingControllerState &controller);
void ParseControllerPackets(ServerState &state, TrackingControllerState &controller);

bool ControllerUpdateSyncMask(TrackingControllerState &controller);

#endif // TRACKING_CONTROLLER_H
