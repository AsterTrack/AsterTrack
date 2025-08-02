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

typedef struct
{
	uint8_t start[UART_LEADING_SEND];
	uint8_t header[PACKET_HEADER_SIZE+HEADER_CHECKSUM_SIZE];
	uint8_t data[];
} UARTPacketRef;


/* Functions */


static inline void calculateHeaderChecksum(uint8_t header[PACKET_HEADER_SIZE], uint8_t checksum[HEADER_CHECKSUM_SIZE])
{
	if (HEADER_CHECKSUM_SIZE == 1)
	{ // Old simple checksum
		for (int i = 0; i < 4; i++)
			checksum[0] += header[i];
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

static inline void writeUARTPacketHeader(UARTPacketRef *packet, struct PacketHeader header)
{
	for (int i = 0; i < UART_LEADING_SEND; i++)
		packet->start[i] = UART_LEADING_BYTE;
	storePacketHeader(header, packet->header);
	for (int i = 0; i < HEADER_CHECKSUM_SIZE; i++)
		packet->header[PACKET_HEADER_SIZE+i] = 0;
	calculateHeaderChecksum(packet->header, packet->header+PACKET_HEADER_SIZE);
}

static inline void calculatePacketChecksum(uint8_t *data, uint16_t length, uint8_t checksum[PACKET_CHECKSUM_SIZE])
{
	if (PACKET_CHECKSUM_SIZE == 1)
	{ // Old simple checksum
		for (int i = 0; i < length; i++)
			checksum[0] += data[i];
	}
	if (PACKET_CHECKSUM_SIZE == 2)
	{
		uint16_t accum[2] = { checksum[0], checksum[1] };
		for (int i = 0; i < length; i++)
		{ // TODO: Proper CRC
			accum[0] += data[i];
			accum[0] += accum[0] >> 8;
			accum[1] += accum[0];
		}
		checksum[0] = accum[0];
		checksum[1] = accum[1];
	}
}

static inline void writeUARTPacketEnd(UARTPacketRef *packet, uint16_t dataLength)
{
	for (int i = 0; i < PACKET_CHECKSUM_SIZE; i++)
		packet->data[dataLength+i] = 0;
	calculatePacketChecksum(packet->data, dataLength, packet->data+dataLength);
	for (int i = 0; i < UART_TRAILING_SEND; i++)
		packet->data[dataLength+PACKET_CHECKSUM_SIZE+i] = UART_TRAILING_BYTE;
}

static inline void finaliseUARTPacket(void *packet, struct PacketHeader header)
{
	writeUARTPacketHeader((UARTPacketRef*)packet, header);
	writeUARTPacketEnd((UARTPacketRef*)packet, header.length);
}

#endif // UART_PROTOCOL_H