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

//#define LOG_MAX_LEVEL LTrace
//#define LOG_MAX_LEVEL LDarn // Even Debug is too much for the log files when it's a realtime system

#include "protocol_packet.hpp"

#include "util/util.hpp" // shortDiff, printBuffer
#include "util/log.hpp"

#include <cstring>

/**
 * Breaks up out-of-order USB Packets into blocks and reassembles them into packets in order
 */


/* Variables and Utility */

// Distance from latestBlockID at which a block and it's packet are considered stale and removed
// Can't be too high to avoid mix-ups with new packets
// This could happen if a lag causes latestBlockID to jump too much at once
// Large lags should externally reset the camera state
int staleLimit = BLOCK_ID_SIGNAL/5;

struct BlockDistComp
{
	uint8_t id;
	BlockDistComp(uint8_t curID) { id = curID; }
	bool operator() (const PacketBlocks& b1, const PacketBlocks& b2)
	{ 
		return shortDiff<uint8_t, int>(b1.nextBlockID, id, 128, BLOCK_ID_SIGNAL) 
				> shortDiff<uint8_t, int>(b2.nextBlockID, id, 128, BLOCK_ID_SIGNAL);
	}
};


/* Functions */


/**
 * Clear stale packets and blocks and remove them
 */
static void ClearStalePackets(std::vector<PacketProtocolPort> &ports, void *userData,
	void (*onPacketDone)(void *userData, int port, PacketBlocks &packet))
{
	for (int p = 0; p < ports.size(); p++)
	{
		PacketProtocolPort &port = ports[p];
		if (!port.packets.empty())
		{ // Check for stale packets (that might already be marked as discarded)
			// Sort by next packet expected, so stale packets first
			port.packets.sort(BlockDistComp(port.latestBlockID));
			auto &waiting = port.packets.front();
			int diff = shortDiff<uint8_t, int>(waiting.nextBlockID, port.latestBlockID, 128, BLOCK_ID_SIGNAL);
			if (diff >= staleLimit)
			{ // Too old, completely discard, missing blocks are not expected to arrive
				// Delete continuous sequence after missing blockID from oooBlocks (all belong to stale packet)
				// Might leave orphaned oooBlocks of stale packet if packet had another missing block
				// These will be cleaned up individually as they become stale
				LOG(LProtocol, LWarn, "Discarding packet %d because %d is still missing after %d arrived, read %d out of %d bytes!\n",
					waiting.header.tag, waiting.nextBlockID, port.latestBlockID, waiting.readLength, waiting.header.length);
				waiting.erroneous = true;
				if (onPacketDone)
					onPacketDone(userData, p, waiting);
				int deleteID = (waiting.nextBlockID+1) % BLOCK_ID_SIGNAL;
				for (auto oooBlock = port.oooBlocks.begin(); oooBlock != port.oooBlocks.end(); oooBlock++)
				{
					if (oooBlock->first < 0) break; // Dormant blocks at the end
					if (oooBlock->first == deleteID)
					{ // Consume oooBlock, move it to the end for future use
						port.oooCount--;
						oooBlock->first = -1;
						port.oooBlocks.splice(port.oooBlocks.end(), port.oooBlocks, oooBlock);
						// Continue searching for the next
						deleteID = (deleteID+1) % BLOCK_ID_SIGNAL;
						oooBlock = port.oooBlocks.begin();
					}
				}
				// Remove stale packet
				port.packets.pop_front();
			}
		}
		if (port.oooCount > 0)
		{ // Find stale (and thus orphaned) oooBlocks
			// Ordered by arrival time, so need to search full list
			for (auto oooBlock = port.oooBlocks.begin(); oooBlock != port.oooBlocks.end(); oooBlock++)
			{
				if (oooBlock->first < 0) break; // Dormant blocks at the end
				if (shortDiff<uint8_t, int>((uint8_t)oooBlock->first, port.latestBlockID, 128, BLOCK_ID_SIGNAL) >= staleLimit)
				{ // Delete stale block
					LOG(LProtocol, LDarn, "Discarding ooo block %d after %d arrived!\n", oooBlock->first, port.latestBlockID);
					oooBlock = port.oooBlocks.erase(oooBlock);
					port.oooCount--;
					break;
				}
			}
		}
	}
}

void HandlePacketBlocks(std::vector<PacketProtocolPort> &ports, uint8_t *data, int length, void *userData,
	void (*onSignal)(void *userData, SignalTag signal, uint8_t *data, uint16_t length),
	void (*onPacketHeader)(void *userData, int port, PacketBlocks &packet),
	void (*onPacketBlock)(void *userData, int port, PacketBlocks &packet, uint8_t *data, int length),
	void (*onPacketDone)(void *userData, int port, PacketBlocks &packet))
{
	int pos = 0;
	while (pos+BLOCK_HEADER_SIZE <= length)
	{
		BlockHeader header = parseBlockHeader(data+pos);
		if (pos+BLOCK_HEADER_SIZE+header.skip+header.size > length)
		{
			LOG(LProtocol, LWarn, "Invalid block header (id %d) received, sizes %d+%d > %d remaining!\n",
				header.blockID, header.skip, header.size, length-pos-BLOCK_HEADER_SIZE);
			break;
		}
		pos += BLOCK_HEADER_SIZE+header.skip;
		if (header.isSignal)
		{ // Signal
		 	LOG(LProtocol, LTrace, "Received signal %d of size %d+%d, data at pos %d\n",
				header.signal, BLOCK_HEADER_SIZE+header.skip, header.size, pos);
			if (onSignal)
				onSignal(userData, header.signal, &data[pos], header.size);
			pos += header.size;
			continue;
		}
		// else regular block

		// Verify block header
		if (header.portNr >= ports.size())
		{
			LOG(LProtocol, LError, "--------- Port %d does not exist.\n", header.portNr);
			break;
		}
		// Filter useless blocks (shouldn't happen)
		if (header.size == 0)
		{
			LOG(LProtocol, LWarn, "BLOCK %d (%d): Useless block, shouldn't happen!\n", header.blockID, header.size);
			break;
		}

		PacketProtocolPort &port = ports[header.portNr];
		int latestDiff = shortDiff<uint8_t, int>(port.latestBlockID, header.blockID, 128, BLOCK_ID_SIGNAL);
		if (latestDiff > 0) port.latestBlockID = header.blockID;

		auto packet = port.packets.end();
		uint32_t posD = pos, lenD = header.size;
		if (header.isFirstPacketBlock)
		{
			if (header.size < PACKET_HEADER_SIZE)
			{
				LOG(LProtocol, LError, "BLOCK %d (%d): Too short, invalid block header?\n", header.blockID, header.size);
				break;
			}
			port.packets.emplace_back(
				parsePacketHeader(data+pos),
				header.blockID, header.blockID, 0);
			packet = std::prev(port.packets.end());
			packet->data.resize(packet->header.length);
			LOG(LProtocol, LTrace, "BLOCK %d (%d): New packet %d of size %d!\n",
				header.blockID, header.size, packet->header.tag, packet->header.length);
			if (onPacketHeader)
				onPacketHeader(userData, header.portNr, *packet);
			posD += PACKET_HEADER_SIZE;
			lenD -= PACKET_HEADER_SIZE;
		}
		else
		{ // Find matching packet
			for (packet = port.packets.begin(); packet != port.packets.end(); packet++)
			{
				if (packet->nextBlockID == header.blockID)
					break;
			}

			if (packet == port.packets.end())
			{ // Out-of-order packet, has to wait for prior blocks
				int oldSize = port.oooBlocks.size();
				while (port.oooBlocks.size() <= port.oooCount)
					port.oooBlocks.push_back({});
				auto oooBlock = std::next(port.oooBlocks.begin(), port.oooCount);
				oooBlock->first = header.blockID;
				oooBlock->second.resize(header.size);
				memcpy(oooBlock->second.data(), &data[pos], header.size);
				pos += header.size;
				LOG(LProtocol, LDarn, "BLOCK %d (%d): Potential ooo block, added to queue in pos %d, grown from size %d to %d!\n",
					header.blockID, header.size, port.oooCount, oldSize, (int)port.oooBlocks.size());
				port.oooCount++;
				continue;
			}
			else
			{
				LOG(LProtocol, LTrace, "BLOCK %d (%d): Continuation for packet tagged %d\n",
					header.blockID, header.size, packet->header.tag);
			}
		}

		memcpy(&packet->data[packet->readLength], &data[posD], std::min(lenD, packet->header.length-packet->readLength));
		packet->readLength += lenD;
		packet->nextBlockID = (packet->nextBlockID + 1) % BLOCK_ID_SIGNAL;
		if (lenD > 0)
		{
			if (onPacketBlock)
				onPacketBlock(userData, header.portNr, *packet, &data[posD], lenD);
		}

		// Try to find next blocks in buffer in case they came out of order
		for (auto oooBlock = port.oooBlocks.begin(); oooBlock != port.oooBlocks.end();)
		{
			if (packet->readLength >= packet->header.length) break; // Packet doesn't expect any further blocks
			if (oooBlock->first < 0) break; // Dormant blocks at the end
			if (oooBlock->first != packet->nextBlockID)
			{
				oooBlock++;
				continue;
			}
			// Read block
			memcpy(&packet->data[packet->readLength], &data[posD], std::min(lenD, packet->header.length-packet->readLength));
			packet->readLength += oooBlock->second.size();
			packet->nextBlockID = (packet->nextBlockID + 1) % BLOCK_ID_SIGNAL;
			if (onPacketBlock)
				onPacketBlock(userData, header.portNr, *packet, oooBlock->second.data(), oooBlock->second.size());
			// Consume oooBlock. move it to the end for future use
			LOG(LProtocol, LDebug, "   Consumed follow-up block %d from ooo-queue, %d remaining\n", oooBlock->first, port.oooCount-1);
			port.oooCount--;
			oooBlock->first = -1;
			port.oooBlocks.splice(port.oooBlocks.end(), port.oooBlocks, oooBlock);
			// Continue search for next block
			oooBlock = port.oooBlocks.begin();
		}

		if (packet->readLength > packet->header.length)
		{
			LOG(LProtocol, LWarn, "Already received more bytes (%d) than packet header announced (%d) with recent block with %d bytes of data!\n",
				packet->readLength, packet->header.length, lenD);
		}
		if (packet->readLength >= packet->header.length)
		{
			LOG(LProtocol, LTrace, "FINISHED packet %d of %d bytes, header block %d, ending block %d\n",
				packet->header.tag, packet->readLength, packet->headerBlockID, packet->nextBlockID);
			if (onPacketDone)
				onPacketDone(userData, header.portNr, *packet);
			port.packets.erase(packet);
		}

		pos += header.size;
	}

	ClearStalePackets(ports, userData, onPacketDone);
}

void ResetPacketPort(PacketProtocolPort &port)
{
	port.latestBlockID = 0;
	for (auto oooBlock = port.oooBlocks.begin(); oooBlock != port.oooBlocks.end(); oooBlock++)
		oooBlock->first = -1;
	port.oooCount = 0;
	port.packets.clear();
}

bool VerifyChecksum(const PacketBlocks &block)
{
	if (block.data.size() <= CHECKSUM_SIZE)
		return true;
	checksum_t *begin = (checksum_t*)block.data.data();
	checksum_t *end = (checksum_t*)(block.data.data()+block.data.size())-1;
	checksum_t checksum = 0;
	checksum_t packetChecksum = *end;
	for (auto el = begin; el != end; el++)
		checksum += *el;
	if (checksum != packetChecksum)
	{
		LOG(LParsing, LWarn, "DIFFERENT CHECKSUM! Camera checksum is %d, received %d!\n", packetChecksum, checksum);
		return false;
	}
	return true;
}