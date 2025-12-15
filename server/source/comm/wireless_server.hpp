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

#ifndef COMM_SERVER_H
#define COMM_SERVER_H

#include "util/util.hpp"

#include <cstdint>
#include <string>
#include <thread>
#include "util/memory.hpp" // opaque_ptr

/* Structures */

// Forward-declared opaque structs
struct ClientCommState; // comm/wireless_server_client.hpp
struct PacketHeader; // comm/packet.hpp

struct TrackingCameraCallbacks
{
	void *userData1 = NULL;
	std::shared_ptr<void> userData2 = NULL;
	void (*onIdentify)(ClientCommState &client);
	void (*onDisconnect)(ClientCommState &client);
	bool (*onReceivePacketHeader)(ClientCommState &client, PacketHeader &header);
	void (*onReceivePacketBlock)(ClientCommState &client, PacketHeader &header, uint8_t *data, unsigned int length);
};

struct ServerCommState 
{
	std::atomic<bool> threadRun;
	std::thread *thread;

	std::vector<opaque_ptr<ClientCommState>> clients;

	int socket = -1;

	TrackingCameraCallbacks callbacks;
};


/* Functions */

int ServerInit(std::string port);

void ServerThread(ServerCommState *serverState);

#endif // COMM_SERVER_H