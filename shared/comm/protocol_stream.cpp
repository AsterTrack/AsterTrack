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

#include "protocol_stream.hpp"
#include "uart.h"

#include <cstdio>
#include <cstring>

void proto_clear(ProtocolState &comm)
{
	comm.cmdSkip = false;
	comm.isCmd = false;
	comm.tail = 0;
	comm.head = 0;
	comm.pos = 0;
}

/**
 * Clean up past commands from buffer
 */
void proto_clean(ProtocolState &comm, bool move)
{
	int rem = comm.tail - comm.head;
	if (rem == 0)
	{ // No unhandled data waiting, clear buffer
		comm.cmdSkip = false;
		comm.tail = 0;
		comm.head = 0;
	}
	else if (rem > 0 && comm.head > 0 && move)
	{ // Unhandled data waiting, move data ahead in buffer (disallowed if data of current command has to be retained)
		memmove(comm.rcvBuf.data(), comm.rcvBuf.data()+comm.head, rem);
		comm.cmdSkip = false;
		comm.tail = rem;
		comm.pos = comm.pos-comm.head;
		comm.head = 0;
	}
}

bool proto_rcvCmd(ProtocolState &comm)
{
	// Skip when no new data and no existing unhandled data exists
	if (comm.head+1+PACKET_HEADER_SIZE > comm.tail) return false;
	//printf("Parsing from %d to %d\n", comm.head, comm.tail);
	if (comm.cmdSkip)
	{ // Skip stray # incase last header has not been validated
		printf("Skipping header %d!\n", comm.header.tag);
		comm.head++;
		comm.isCmd = false;
	}
	if (comm.isCmd)
	{ // If current command exist, make skippable, no further parsing needed
		comm.cmdSkip = true;
		return true;
	}
	// Locate next command
	if (comm.rcvBuf[comm.head] != UART_LEADING_BYTE)
	{ // Skip garbage bytes (should not happen on a clean connection) or UART_TRAILING_BYTEs
		while (comm.head < comm.tail && comm.rcvBuf[comm.head] != UART_LEADING_BYTE)
			comm.head++;
		// If no command is found, clear to reset buffer
		if (comm.head == comm.tail)
		{
			proto_clear(comm);
			return false;
		}
		comm.pos = comm.head;
	}
	while (comm.head+1 < comm.tail && comm.rcvBuf[comm.head+1] == UART_LEADING_BYTE)
	{ // Move past variable amount of leading bytes
		// gotta leave one ahead so we can return and wait for more bytes and still detect the header
		comm.head++;
	}
	// Make sure the full command header is received
	if (comm.head+1+PACKET_HEADER_SIZE > comm.tail) return false;
	comm.head++; // Skip last leading byte
	comm.header = parsePacketHeader(comm.rcvBuf.data() + comm.head);
	comm.pos = comm.head+PACKET_HEADER_SIZE;
	comm.cmdSz = comm.header.length;
	comm.isCmd = true;
	comm.cmdSkip = true; // If header not validated, will skip
	return true;
}

bool proto_fetchCmd(ProtocolState &comm)
{ // Command ID is valid, wait until it's received in full
	comm.cmdSkip = false;
	if (comm.tail >= comm.pos+comm.cmdSz)
	{ // Received in full
		comm.cmdPos = comm.pos; // Save data pos
		comm.pos += comm.cmdSz; // Advance to after command
		comm.head = comm.pos;	// Also advance to after command
		//printf("Received cmd %d of size %d from cmd pos %d to end %d [%d, %d, %d]\n", comm.header.tag, comm.cmdSz, comm.cmdPos, comm.head, comm.rcvBuf[comm.head-1], comm.rcvBuf[comm.head], comm.rcvBuf[comm.head+1]);
		proto_clean(comm, false); // Check if buffer can be reset
		comm.isCmd = false;
		return true;
	}
	else
	{ // Move receive head to beginning of buffer to make it easier to parse packet once it does arrive
		proto_clean(comm, true);
		if (PACKET_HEADER_SIZE+comm.cmdSz > comm.rcvBuf.size())
			comm.rcvBuf.resize(PACKET_HEADER_SIZE+comm.cmdSz);
	}
	return false;
}

unsigned int proto_handleCmdBlock(ProtocolState &comm)
{ // Command ID is valid, consider block to be handled
	comm.cmdSkip = false;
	unsigned int end = comm.head+PACKET_HEADER_SIZE+comm.cmdSz;
	unsigned int len = std::min(end, comm.tail) - comm.pos;
	comm.cmdPos = comm.pos; // Save data pos
	comm.pos += len; // Advance to current read tail
	if (end <= comm.tail)
	{ // Handle final block of packet
		comm.head = comm.pos = end;	// Also advance to after command
		proto_clean(comm, false); // Check if buffer can be reset
		comm.isCmd = false;
	}
	return len;
}