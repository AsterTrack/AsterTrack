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

#ifndef COMM_H
#define COMM_H

#include "../state.hpp"
#include "comm/protocol_stream.hpp"

#include <cstdint>
#include <vector>

// Sending a packet:
// comm_packet (claims mutex)
// comm_write ...
// comm_submit (releases mutex)
// Optionally, comm_flush (wait for write)
// OR:
// comm_send

struct CommPacket
{
	PacketHeader header;
	std::vector<uint8_t> data;
	std::vector<uint8_t> largePacketHeader;
};

struct CommState 
{
	bool enabled = false, started = false, ready = false, writing = false, error = false;
	CommMedium medium;
	ProtocolState protocol = {};
	IdentPacket ownIdent = {};
	IdentPacket expIdent = {};
	IdentPacket otherIdent = {};

	std::mutex writeAccess;
	CRC32 crc;
	int sentSize;
	int totalSize;

	bool realtimeControlled;
	std::queue<CommPacket> packetQueue;

	std::thread *thread;
	void *port;
	bool (*start)(void *port);
	void (*stop)(void *port);
	int (*wait)(void *port, uint32_t timeoutUS);
	void (*configure)(void *port, uint32_t rate);
	int (*read)(void *port, uint8_t *data, uint32_t length);
	int (*write)(void *port, const uint8_t *data, uint32_t length);
	void (*submit)(void *port);
	void (*flush)(void *port);
};

struct CommList
{
	std::array<CommState*,COMM_MEDIUM_MAX> medium;

	inline CommState* get(CommMedium affinity)
	{
		assert(affinity >= 0 && affinity < COMM_MEDIUM_MAX);
		if (medium[affinity] && medium[affinity]->ready)
			return medium[affinity];
		for (int i = 0; i < COMM_MEDIUM_MAX; i++)
			if (medium[i] && medium[i]->ready)
				return medium[i];
		// Nothing ready, doesn't matter which comm to select
		return nullptr;
	}
};

extern CommList comms;
extern CommMedium realTimeAff, largeDataAff;

void comm_enable(CommState &comm, TrackingCameraState *state, CommMedium medium);
void comm_disable(CommState &comm);

/* Direct packet sending functions assuming full control over comm TX */
bool comm_packet(CommState *commPtr, PacketHeader header);
void comm_write(CommState *commPtr, const uint8_t *data, uint32_t length);
void comm_submit(CommState *commPtr);

/* Send packet immediately, assuming full control over comm TX */
bool comm_send_immediate(CommState *commPtr, PacketHeader header, const uint8_t *data);
/* Send part of a large packet, depending on budget, assuming full control over comm TX */
int comm_send_block(CommState *commPtr, PacketHeader header, const std::vector<uint8_t> &data, std::vector<uint8_t> &largePacketHeader, int budget, int &sent);
/* Queue packet to be send via comm in it's respective controlling thread */
bool comm_queue_send(CommState *commPtr, PacketHeader header, std::vector<uint8_t> &&data);
/* Queue large packet to be send (in blocks, if necessary) via comm in it's respective controlling thread */
bool comm_queue_send(CommState *commPtr, PacketHeader header, std::vector<uint8_t> &&data, std::vector<uint8_t> largePacketHeader);

/* Send packet, immediately if in controlling thread, queue if not.  */
bool comm_send(CommState *commPtr, PacketHeader header, const uint8_t *data);
/* Send packet, immediately if in controlling thread, queue if not.  */
bool comm_send(CommState *commPtr, PacketHeader header, std::vector<uint8_t> &&data);
/* Send packet, immediately if in controlling thread, queue if not.  */
bool comm_send(CommMedium comm, PacketHeader header, const uint8_t *data);
/* Send packet, immediately if in controlling thread, queue if not.  */
bool comm_send(CommMedium comm, PacketHeader header, std::vector<uint8_t> &&data);

bool comm_interject(CommState *commPtr);
void comm_force_close(CommState *commPtr);

inline bool comm_anyReady(CommList &comms)
{
	bool anyReady = false;
	for (int i = 0; i < COMM_MEDIUM_MAX; i++)
		anyReady = anyReady || (comms.medium[i] && comms.medium[i]->ready);
	return anyReady;
}

inline bool comm_anyEnabled(CommList &comms)
{
	bool anyEnabled = false;
	for (int i = 0; i < COMM_MEDIUM_MAX; i++)
		anyEnabled = anyEnabled || (comms.medium[i] && comms.medium[i]->enabled);
	return anyEnabled;
}

void CommThread(CommState *comm_ptr, TrackingCameraState *state_ptr);

#endif // COMM_H