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

#ifndef PROTOCOL_STREAM_H
#define PROTOCOL_STREAM_H


#include "packet.h"

#include <vector>
#include <cstdint>


struct ProtocolState 
{
	unsigned int tail;
	unsigned int head;
	unsigned int pos;
	bool isCmd;
	bool cmdNAK;
	bool cmdACK;
	PacketHeader header;
	unsigned int cmdSz;
	unsigned int cmdPos;
	bool cmdSkip;
	std::vector<uint8_t> rcvBuf = std::vector<uint8_t>(1024);
};

void proto_clear(ProtocolState &comm);

/**
 * Clean up past commands from buffer
 */
void proto_clean(ProtocolState &comm, bool move=false);

bool proto_rcvCmd(ProtocolState &comm);

bool proto_fetchCmd(ProtocolState &comm);

unsigned int proto_handleCmdBlock(ProtocolState &comm);

#endif // PROTOCOL_STREAM_H