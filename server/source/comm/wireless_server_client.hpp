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

#ifndef COMM_SERVER_CLIENT_H
#define COMM_SERVER_CLIENT_H

#include "comm/wireless_server.hpp"
#include "comm/protocol_stream.hpp"

#include <cstdint>
#include <atomic>
#include <mutex>
#include <thread>


/* Structures */

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

bool comm_writeHeader(ClientCommState &client, PacketTag tag, uint16_t packetLength);
bool comm_write(ClientCommState &client, const uint8_t *data, uint16_t length);
void comm_abort(ClientCommState &client);

#endif // COMM_SERVER_CLIENT_H