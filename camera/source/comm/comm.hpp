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
#include "comm/uart.h"

#include <cstdint>
#include <vector>

// Sending a packet:
// comm_packet (claims mutex)
// comm_write ...
// comm_submit (releases mutex)
// Optionally, comm_flush (wait for write)

void comm_init();

void comm_close(CommState &comm);

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

/* Start sending a packet to host */
void* comm_packet(CommState *commPtr, PacketHeader header);
void comm_write(CommState *commPtr, void* packet, const uint8_t *data, uint32_t length);
void comm_submit(CommState *commPtr, void* packet);

void comm_NAK(CommState &comm);

inline void comm_flush(CommState &comm)
{
	if (comm.started)
		comm.flush(comm.port);
}

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