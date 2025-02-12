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

#ifndef PROTOCOL_PACKET_H
#define PROTOCOL_PACKET_H

#include "comm/packet.h"

#include <cstdint>
#include <vector>
#include <list>

/**
 * Breaks up out-of-order USB Packets into blocks and reassembles them into packets in order
 */


struct PacketBlocks
{
	PacketHeader header;
	uint8_t headerBlockID;
	uint8_t nextBlockID;
	uint32_t readLength;
	std::vector<uint8_t> data;
	bool ignored, erroneous;
};

struct PacketProtocolPort
{
	uint8_t latestBlockID = 0;
	// Currently receiving packets
	std::list<PacketBlocks> packets;
	// Store out-of-order blocks (not certain which packet they belong to)
	typedef std::pair<int16_t,std::vector<uint8_t>> OOOBlock;
	std::list<OOOBlock> oooBlocks;
	int oooCount = 0;
};

/**
 * Handle a USB packet with many blocks from packets
 * Callbacks are called to handle each block (and it's associated packet) separately
 * Manages out-of-order blocks for packets
 */
void HandlePacketBlocks(std::vector<PacketProtocolPort> &ports, uint8_t *data, int length, void *userData,
	void (*onSignal)(void *userData, SignalTag signal, uint8_t *data, uint16_t length),
	void (*onPacketHeader)(void *userData, int port, PacketBlocks &packet),
	void (*onPacketBlock)(void *userData, int port, PacketBlocks &packet, uint8_t *data, int length),
	void (*onPacketDone)(void *userData, int port, PacketBlocks &packet));

/**
 * Clear all state from the port
 * Do when a larger lag has been detected and all previously receiving packets are expected to be discarded
 */
void ResetPacketPort(PacketProtocolPort &port);

/**
 * Calculate the checksum of size CHECKSUM_SIZE and returns whether it matches the packet tail
 */
bool VerifyChecksum(const PacketBlocks &block);

#endif // PROTOCOL_PACKET_H