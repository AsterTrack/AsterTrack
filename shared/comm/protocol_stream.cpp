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
#include "packet.hpp"
#include "uart.h"

#include <cstdio>
#include <cstring>

void proto_clear(ProtocolState &comm)
{
	comm.cmdSkip = false;
	comm.isHeader = false;
	comm.isCmd = false;
	comm.cmdEnd = false;
	comm.cmdErr = false;
	comm.tail = 0;
	comm.head = 0;
	comm.blockPos = 0;
	comm.mrk = 0;
}

void proto_clean(ProtocolState &comm, bool move)
{
	int rem = comm.tail - comm.head;
	if (rem == 0)
	{ // No unhandled data waiting, clear buffer
		comm.cmdSkip = false;
		comm.tail = 0;
		comm.head = 0;
		comm.blockPos = 0;
		comm.mrk = 0;
	}
	else if (rem > 0 && comm.head > 10 && move)
	{ // Unhandled data waiting, move data ahead in buffer (disallowed if data of current command has to be retained)
		memmove(comm.rcvBuf.data(), comm.rcvBuf.data()+comm.head, rem);
		comm.cmdSkip = false;
		comm.tail = rem;
		if (comm.blockPos <= comm.head) comm.blockPos = 0;
		else comm.blockPos -= comm.head;
		if (comm.mrk <= comm.head) comm.mrk = 0;
		else comm.mrk -= comm.head;
		comm.head = 0;
	}
}

static inline void skipToEnd(ProtocolState &comm, bool error)
{
	comm.cmdErr = error;
	comm.cmdEnd = !error;
	comm.isCmd = false;
	comm.isHeader = false;
}

/**
 * Find marked bytes (by linux termios option PARMRK) and parse them properly
 * Input: Data buffer up to dataLen may be parsed, with a desired number of parsed bytes described by parseLen
 * Output: dataLen is number of bytes consumed, parseLen the number of bytes left. The remainder are undefined (between index parseLen and dataLen)
 * Returns whether one of the parsed bytes were marked to be erroneous 
 */
static bool scanMarkedBlock(uint8_t *data, uint32_t &dataLen, uint32_t &parseLen)
{
	bool error = false;
	int d = 0, p = 0; // Data(read) index, parsed(write) index
	for (; p < parseLen && d < dataLen; d++, p++)
	{
		if (data[d] == 0xFF)
		{ // Marked
			if (d+1 >= dataLen)
			{ // Marked byte inaccessible
				printf("Received marked byte cut off by data buffer!\n");
				dataLen = d;
				parseLen = p;
				return true;
			}
			if (data[d+1] == 0xFF)
			{
				data[p] = data[d+1]; // Copy 0xFF
				d += 1; // Skip doubled 0xFF
			}
			else if (data[d+1] == 0x00)
			{ // Frame or Parity Error
				if (d+2 >= dataLen)
				{ // Marked byte inaccessible
					printf("Received erroneous marked byte cut off by data buffer!\n");
					dataLen = d;
					parseLen = p;
					return true;
				}
				printf("Detected parity/frame error!\n");
				error = true;
				data[p] = data[d+2]; // Copy erroneously read value
				d += 2; // Skip two preceeding markers
			}
			else
			{ // Should not happen
				printf("Received invalid marked bytes!\n");
				error = true;
				d += 2; // Skip two markers?
			}
		}
		else
			data[p] = data[d];
	}
	dataLen = d;
	parseLen = p;
	return error;
}

/**
 * Scan the receive buffer for errors until the target bytes have been received
 * Returns true if bytes up until target have been received successfully
 * Returns false if there is not enough byzes or an error has been detected (setting cmdErr flag) 
 */
static bool scanUntil(ProtocolState &comm, uint32_t target)
{
	if (!comm.needsErrorScanning)
	{
		if (target <= comm.tail)
		{ // Done
			comm.mrk = target;
			return true;
		}
		comm.mrk = comm.tail;
		return false;
	}
	if (comm.mrk < comm.head) comm.mrk = comm.head;
	if (target <= comm.mrk) return true;
	unsigned int dataLen = comm.tail-comm.mrk, parseLen = target - comm.mrk;
	if (scanMarkedBlock(comm.rcvBuf.data()+comm.mrk, dataLen, parseLen))
	{ // Read length had parity/frame error
		skipToEnd(comm, true);
		// Consume parsed bytes up until error
		comm.mrk += dataLen;
		comm.head = comm.mrk;
		return false;
	}
	// Move yet unscanned, marked buffer forward (removing space used for marking)
	memcpy(comm.rcvBuf.data()+comm.mrk+parseLen, comm.rcvBuf.data()+comm.mrk+dataLen, comm.tail-(comm.mrk+dataLen));
	comm.mrk += parseLen;
	comm.tail += parseLen-dataLen;
	return comm.mrk >= target;
}

/**
 * Await and consume a block of bytes of at least length min.
 * Returns false if read bytes have been exhausted, and we need to wait for more
 * Returns true if a block of at least size min has been found, and comm.head points to the first byte after that block.
 */
static inline unsigned int awaitBlockOf(ProtocolState &comm, uint8_t byte, uint8_t min, uint8_t max)
{
	while (comm.head < comm.tail && comm.rcvBuf[comm.head] != byte)
		comm.head++;
	// Check number of trailing bytes
	int num = 0;
	for (; comm.head+num < comm.tail; num++)
		if (comm.rcvBuf[comm.head+num] != byte)
			break;
	if (num < max && comm.head+num == comm.tail)
	{ // Wait, even if num>min, since we can't confirm the block ends here (unless num >= max)
		proto_clean(comm, true); // Check if buffer can be reset
		return 0;
	}
	// Move past block
	comm.head += num;
	if (num >= min)
		return num;
	// Continue searching
	return awaitBlockOf(comm, byte, min, max);
}

bool proto_rcvCmd(ProtocolState &comm)
{
begin:
	// Skip when no new data and no existing unhandled data exists
	if (comm.head >= comm.tail) return false;
	//printf("Parsing from %d to %d\n", comm.head, comm.tail);
	if (comm.cmdSkip)
	{ // Did not signal to fetch full command, skip to end 
		printf("Skipping header %d!\n", comm.header.tag);
		skipToEnd(comm, false);
	}
	if (comm.isCmd)
	{ // If current command exist, make skippable, no further parsing needed
		comm.cmdSkip = true;
		return true;
	}
	if (comm.cmdErr || comm.cmdEnd)
	{
		// Await block of trailing bytes of a given minimum length
		int numTrailing = awaitBlockOf(comm, UART_TRAILING_BYTE, UART_TRAILING_RECV, UART_TRAILING_SEND);
		if (numTrailing == 0) return false; // Wait for more bytes
		// If so, clear flags and search for next command
		comm.cmdErr = false;
		comm.cmdEnd = false;
	}
	if (!comm.isHeader)
	{
		// Await block of trailing bytes of a given minimum length
		int numLeading = awaitBlockOf(comm, UART_LEADING_BYTE, UART_LEADING_RECV, UART_LEADING_SEND);
		if (numLeading == 0) return false; // Wait for more bytes
		comm.isHeader = true;
	}
	// Scan full header data for errors encoded with marked bytes
	int headPosInitial = comm.head;
	if (!scanUntil(comm, comm.head+PACKET_HEADER_SIZE+HEADER_CHECKSUM_SIZE))
	{
		if (comm.cmdErr)
			goto begin; // Header erroneous, skip to end
		return false; // Wait for more bytes
	}
	// Header is fully received, commit to parsing it
	comm.isHeader = false;
	if (!verifyHeaderChecksum(comm.rcvBuf.data() + comm.head))
	{
		printf("Received header %d with erroneous checksum!\n", comm.header.tag);
		skipToEnd(comm, true);
		goto begin; // Header erroneous, skip to end
	}
	comm.header = parsePacketHeader(comm.rcvBuf.data() + comm.head);
	if (comm.rcvBuf.size() < comm.head+UART_PACKET_OVERHEAD_SEND+comm.header.length)
	{ // Move data to the beginning
		proto_clean(comm, true);
		if (comm.rcvBuf.size() < UART_PACKET_OVERHEAD_SEND+comm.header.length)
		{ // Increase buffer size for long packets, accounting for marked bytes and final checksum
			unsigned int targetBufferSize = (UART_PACKET_OVERHEAD_SEND+comm.header.length)*3/2;
			comm.rcvBuf.resize(targetBufferSize);
		}
	}
	comm.cmdPos = comm.head+PACKET_HEADER_SIZE+HEADER_CHECKSUM_SIZE;
	comm.cmdSz = comm.header.length;
	// Skip packet if header is not validated
	comm.isCmd = true;
	comm.cmdSkip = true;
	// Initialise block data
	comm.blockPos = comm.cmdPos;
	comm.blockLen = 0;
	return true;
}

bool proto_fetchCmd(ProtocolState &comm)
{ // Command ID is valid, wait until it's received in full
	comm.cmdSkip = false;
	unsigned int endData = comm.head+PACKET_HEADER_SIZE+HEADER_CHECKSUM_SIZE+comm.cmdSz;
	unsigned int endPacket = comm.cmdSz > 0? endData+PACKET_CHECKSUM_SIZE : endData;
	bool done = scanUntil(comm, endPacket+UART_TRAILING_RECV);
	// Update block data
	comm.blockPos += comm.blockLen;
	comm.blockLen = std::min(comm.mrk, endData) - comm.blockPos;
	if (done)
	{ // Received in full
		comm.head = std::max(comm.head, endPacket-20); // Advance towards end of packet (buffer to account for dropped bytes)
		//printf("Received cmd %d of size %d from cmd pos %d to end %d [%d, %d, %d]\n", comm.header.tag, comm.cmdSz, comm.cmdPos, comm.head, comm.rcvBuf[comm.head-1], comm.rcvBuf[comm.head], comm.rcvBuf[comm.head+1]);
		skipToEnd(comm, false);

		comm.validChecksum = true;
		if (comm.cmdSz > 0)
		{ // Verify packet checksum
			uint8_t *packetChecksum = comm.rcvBuf.data()+comm.cmdPos+comm.cmdSz;
			uint8_t checksum[PACKET_CHECKSUM_SIZE];
			if (comm.header.tag < PACKET_HOST_COMM)
				calculateDirectPacketChecksum(comm.rcvBuf.data()+comm.cmdPos, comm.cmdSz, checksum);
			else
				calculateForwardPacketChecksum(comm.rcvBuf.data()+comm.cmdPos, comm.cmdSz, checksum);
			for (int i = 0; i < PACKET_CHECKSUM_SIZE; i++)
			{
				if (checksum[i] != packetChecksum[i])
				{
					printf("Fully received packet %d of size %d but checksum does not match!\n", comm.header.tag, comm.header.length);
					comm.validChecksum = false;
					break;
				}
			}
		}
		return true;
	}
	// Move receive head to beginning of buffer to make it easier to parse packet once it does arrive
	// comm.cmdErr may also have been set, and this packet abandoned
	proto_clean(comm, true);
	return false;
}