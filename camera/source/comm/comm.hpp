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

inline void comm_write(std::vector<CommState*> &comms, const uint8_t *data, uint32_t length)
{
	static_assert(CHECKSUM_SIZE == 1); // Else, need to account for length % CHECKSUM_SIZE != 0
	checksum_t *begin = (checksum_t*)data;
	checksum_t *end = (checksum_t*)(data+length);
	checksum_t checksum = 0;
	for (auto el = begin; el != end; el++)
		checksum += *el;
	for (int i = 0; i < comms.size(); i++)
	{
		if (!comms[i]->writing) continue;
		int ret = comms[i]->write(comms[i]->port, data, length);
		if (ret < 0)
		{
			comms[i]->writing = false;
			comms[i]->writeAccess.unlock();
			comm_close(*comms[i]);
		}
		else
		{
			comms[i]->checksum += checksum;
		}
	}
}
inline void comm_write(CommState &comm, const uint8_t *data, uint32_t length)
{
	if (comm.writing)
	{ // Loop should get optimised away
		std::vector<CommState*> tmp = { &comm };
		comm_write(tmp, data, length);
	}
}

inline void comm_packet(std::vector<CommState*> &comms, PacketHeader header)
{
	header.length++; // Checksum, transparent
	UARTPacketRef packet;
	writeUARTPacketHeader(&packet, header);
	for (int i = 0; i < comms.size(); i++)
	{
		// Initiate packet
		if (!comms[i]->ready) continue;
		comms[i]->writeAccess.lock();
		comms[i]->writing = true;
		// Write header
		int ret = comms[i]->write(comms[i]->port, (uint8_t*)&packet, sizeof(packet));
		if (ret < 0)
		{
			comms[i]->writeAccess.unlock();
			comm_close(*comms[i]);
			continue;
		}
		comms[i]->checksum = 0;
		comms[i]->writing = true;
	}
}
inline void comm_packet(CommState &comm, PacketHeader header)
{
	if (comm.ready)
	{ // Loop should get optimised away
		std::vector<CommState*> tmp = { &comm };
		comm_packet(tmp, header);
	}
}

inline void comm_submit(CommState &comm)
{
	if (!comm.writing) return;
	uint8_t buffer[CHECKSUM_SIZE+UART_TRAILING_SEND];
	*((checksum_t*)&buffer[0]) = comm.checksum;
	for (int i = 0; i < UART_TRAILING_SEND; i++)
		buffer[CHECKSUM_SIZE+i] = UART_TRAILING_BYTE;
	comm.write(comm.port, buffer, sizeof(buffer));
	comm.submit(comm.port);
	comm.writing = false;
	comm.writeAccess.unlock();
}
inline void comm_submit(std::vector<CommState*> &comms)
{
	for (int i = 0; i < comms.size(); i++)
		comm_submit(*comms[i]);
}

inline void comm_flush(CommState &comm)
{
	if (comm.started)
		comm.flush(comm.port);
}
inline void comm_flush(std::vector<CommState*> &comms)
{
	for (int i = 0; i < comms.size(); i++)
		comm_flush(*comms[i]);
}

void comm_NAK(CommState &comm);
inline void comm_NAK(std::vector<CommState*> &comms)
{
	for (int i = 0; i < comms.size(); i++)
		comm_NAK(*comms[i]);
}

inline bool comm_anyReady(std::vector<CommState*> &comms)
{
	bool anyReady = false;
	for (int i = 0; i < comms.size(); i++)
		anyReady = anyReady || comms[i]->ready;
	return anyReady;
}

inline bool comm_anyEnabled(std::vector<CommState*> &comms)
{
	bool anyEnabled = false;
	for (int i = 0; i < comms.size(); i++)
		anyEnabled = anyEnabled || comms[i]->enabled;
	return anyEnabled;
}

void CommThread(CommState *comm_ptr, TrackingCameraState *state_ptr);

#endif // COMM_H