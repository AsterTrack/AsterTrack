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

#include "comm/protocol_stream.hpp"

#include <cstdint>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <list>
#include <memory> // shared_ptr

/* Structures */

struct ServerCommState;
struct TrackingCameraCallbacks;
struct ClientCommState;

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

	std::list<ClientCommState> clients;

	int socket = -1;

	TrackingCameraCallbacks callbacks;
};

struct ClientCommState 
{
	std::thread *thread;

	int socket = -1;
	std::mutex writeAccess;
	ProtocolState protocol = {};

	TrackingCameraCallbacks callbacks;

	std::atomic<bool> enabled = { false }, started = { false }, ready = { false };
	bool rsp_id = false, rsp_ack = false;
	int ident_timeout = 0, ident_interval = 0;
	IdentPacket ownIdent;
	IdentPacket expIdent;
	IdentPacket otherIdent;

	ClientCommState() = default;
	ClientCommState(ClientCommState &&other) noexcept
	{
		*this = std::move(other);
	}
	ClientCommState& operator=(ClientCommState &&other) noexcept
	{
		thread = other.thread;
		other.thread = nullptr;
		enabled = other.enabled.load();
		started = other.started.load();
		ready = other.ready.load();
		socket = other.socket;
		protocol = other.protocol;
		callbacks = other.callbacks;
		// TODO: Add other members
		return *this;
	}
};


/* Functions */

int ServerInit(std::string port);

void ServerThread(ServerCommState *serverState);

bool comm_writeHeader(ClientCommState &client, PacketTag tag, uint16_t packetLength);
bool comm_write(ClientCommState &client, const uint8_t *data, uint16_t length);
void comm_abort(ClientCommState &client);

#endif // COMM_SERVER_H