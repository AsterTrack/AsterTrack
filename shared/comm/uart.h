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

#define UART_LEADING_SEND 2
#define UART_LEADING_RECV 1
#define UART_TRAILING_SEND 5
#define UART_TRAILING_RECV 2

#define UART_PACKET_OVERHEAD_SEND (UART_LEADING_SEND + PACKET_HEADER_SIZE + UART_TRAILING_SEND)


/* Structures */

typedef struct
{
	uint8_t start[UART_LEADING_SEND];
	uint8_t header[PACKET_HEADER_SIZE];
	uint8_t data[];
} UARTPacketRef;


/* Functions */

static inline void writeUARTPacketHeader(UARTPacketRef *packet, struct PacketHeader header)
{
	for (int i = 0; i < UART_LEADING_SEND; i++)
		packet->start[i] = UART_LEADING_BYTE;
	storePacketHeader(header, packet->header);
}

static inline void writeUARTPacketEnd(UARTPacketRef *packet, uint16_t dataLength)
{
	for (int i = 0; i < UART_TRAILING_SEND; i++)
		packet->data[dataLength+i] = UART_TRAILING_BYTE;
}

static inline void finaliseUARTPacket(void *packet, struct PacketHeader header)
{
	writeUARTPacketHeader((UARTPacketRef*)packet, header);
	writeUARTPacketEnd((UARTPacketRef*)packet, header.length);
}

#endif // UART_PROTOCOL_H