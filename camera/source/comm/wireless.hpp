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

#ifndef WIRELESS_H
#define WIRELESS_H

#include "comm/packet.hpp"
#include "util/util.hpp"

#include <cstdint>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

struct WirelessState
{
	std::thread *monitorThread;
	std::mutex mutex;
	std::atomic<bool> monitor;

	WirelessConfig config, changed; // Temporary configuration
	WirelessStatus wifi, ssh, server; // Current status
	WirelessStatus wifiFlags, sshFlags, serverFlags; // Current flags
	std::string serverHost, serverPort; // Cached server configuration

	bool sendStatus;
	TimePoint_t lastStatus, lastCheck;
	std::string SSID, IP, error;
};

struct TrackingCameraState;


void initWirelessMonitor(TrackingCameraState &state);

void stopWirelessMonitor(TrackingCameraState &state);

void fillWirelessStatusPacket(TrackingCameraState &state, std::vector<uint8_t> &packet);

bool parseWirelessConfigPacket(TrackingCameraState &state, uint8_t *packet, uint16_t length);

#endif // WIRELESS_H