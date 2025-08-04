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

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include "packet.h"

/**
 * This file describes the UART protocol and implementation
 * Shared by controller, camera_mcu and camera_pi
*/


/* Protocol Defines */

#define UART_BAUD_RATE_SAFE		8000000
#define UART_BAUD_RATE_MAX		8000000

/**
 * Bytes used ahead of and after packets on error-prone, stream-like interfaces (like UART)
 * Leading byte (singular) signals a new packet
 * Trailing bytes (plural) provide buffer to protect following packets
 * - This is relevant when a packet looses a byte, as it would otherwise consume start of the next packet as part of itself
 * - With this, the error will get detected more reliably (not just wrong checksum, but trailing byte), and the next packets are safe
 */
#define UART_LEADING_BYTE			0b01011011
#define UART_TRAILING_BYTE			0b10100110

#define UART_LEADING_SEND 3
#define UART_LEADING_RECV 2
#define UART_TRAILING_SEND 5
#define UART_TRAILING_RECV 2

#define UART_PRE_OVERHEAD_SEND (UART_LEADING_SEND + PACKET_HEADER_SIZE + HEADER_CHECKSUM_SIZE)
#define UART_POST_OVERHEAD_SEND (PACKET_CHECKSUM_SIZE + UART_TRAILING_SEND)
#define UART_PACKET_OVERHEAD_SEND (UART_PRE_OVERHEAD_SEND + UART_POST_OVERHEAD_SEND)


/* Structures */

#pragma pack(push, 1)
typedef struct
{
	uint8_t start[UART_LEADING_SEND];
	uint8_t header[PACKET_HEADER_SIZE];
	uint8_t headerChecksum[HEADER_CHECKSUM_SIZE];
	uint8_t data[];
} UARTPacketRef;
#pragma pack(pop)


/* Functions */

static inline void calculateHeaderChecksum(uint8_t header[PACKET_HEADER_SIZE], uint8_t checksum[HEADER_CHECKSUM_SIZE])
{
	if (HEADER_CHECKSUM_SIZE == 1)
	{ // 8bit fletcher checksum
		uint16_t accum1 = 0, accum2 = 0;
		for (int i = 0; i < PACKET_HEADER_SIZE; i++)
			accum2 += accum1 += header[i];
		// Sadly, on rv32imac, this is an actual division operation, so use modulo 16
		//accum1 %= 15; accum2 %= 15;
		accum1 &= 0xF; accum2 &= 0xF;
		checksum[0] = (accum2 << 4) + accum1;
	}
}

static inline bool verifyHeaderChecksum(uint8_t header[PACKET_HEADER_SIZE+HEADER_CHECKSUM_SIZE])
{
	uint8_t checksum[HEADER_CHECKSUM_SIZE];
	for (int i = 0; i < HEADER_CHECKSUM_SIZE; i++)
		checksum[i] = 0;
	calculateHeaderChecksum(header, checksum);
	for (int i = 0; i < HEADER_CHECKSUM_SIZE; i++)
		if (checksum[i] != header[PACKET_HEADER_SIZE+i])
			return false;
    return true;
}

static inline void calculateDirectPacketChecksum(const uint8_t *data, uint16_t length, uint8_t checksum[PACKET_CHECKSUM_SIZE])
{
	if (PACKET_CHECKSUM_SIZE == 4)
	{ // Fletcher-32 - derived
		uint16_t accum1 = 0, accum2 = 0;
		for (int i = 0; i < length; i++)
			accum2 += accum1 += data[i];
		checksum[0] = accum1 & 0xFF;
		checksum[1] = accum1 >> 8;
		checksum[2] = accum2 & 0xFF;
		checksum[3] = accum2 >> 8;
	}
}

static inline void writeUARTPacketHeader(UARTPacketRef *packet, struct PacketHeader header)
{
	for (int i = 0; i < UART_LEADING_SEND; i++)
		packet->start[i] = UART_LEADING_BYTE;
	storePacketHeader(header, packet->header);
	for (int i = 0; i < HEADER_CHECKSUM_SIZE; i++)
		packet->headerChecksum[i] = 0;
	calculateHeaderChecksum(packet->header, packet->headerChecksum);
}

static inline void writeUARTPacketEnd(UARTPacketRef *packet, uint16_t dataLength)
{
	for (int i = 0; i < UART_TRAILING_SEND; i++)
		packet->data[dataLength+PACKET_CHECKSUM_SIZE+i] = UART_TRAILING_BYTE;
}

static inline void finaliseDirectUARTPacket(void *packet, struct PacketHeader header)
{
	UARTPacketRef *ref = (UARTPacketRef*)packet;
	writeUARTPacketHeader(ref, header);
	calculateDirectPacketChecksum(ref->data, header.length, ref->data+header.length);
	writeUARTPacketEnd(ref, header.length);
}

#endif // UART_PROTOCOL_H