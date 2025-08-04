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
{ // Wrapper to:
	// - avoid dynamic allocation, instead allocating on stack
	// - allow passing passing of a single CommState without overload
	std::array<CommState*,2> arr;
	int cnt;
	CommList() : arr{}, cnt(0) {}
	CommList(CommState &comm) : arr{&comm}, cnt(1) {}
};

/* Start sending a packet to host */
void* comm_packet(CommList comms, PacketHeader header);
void comm_write(CommList comms, void* packet, const uint8_t *data, uint32_t length);
void comm_submit(CommList comms, void* packet);

inline void comm_flush(CommState &comm)
{
	if (comm.started)
		comm.flush(comm.port);
}
inline void comm_flush(CommList comms)
{
	for (int i = 0; i < comms.cnt; i++)
		comm_flush(*comms.arr[i]);
}

void comm_NAK(CommState &comm);
inline void comm_NAK(CommList comms)
{
	for (int i = 0; i < comms.cnt; i++)
		comm_NAK(*comms.arr[i]);
}

inline bool comm_anyReady(CommList comms)
{
	bool anyReady = false;
	for (int i = 0; i < comms.cnt; i++)
		anyReady = anyReady || comms.arr[i]->ready;
	return anyReady;
}

inline bool comm_anyEnabled(CommList comms)
{
	bool anyEnabled = false;
	for (int i = 0; i < comms.cnt; i++)
		anyEnabled = anyEnabled || comms.arr[i]->enabled;
	return anyEnabled;
}

void CommThread(CommState *comm_ptr, TrackingCameraState *state_ptr);

#endif // COMM_H