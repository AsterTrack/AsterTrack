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

#ifndef __UARTD_H
#define __UARTD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "comm/packet.h"
#include "uartd_conf.h"


/* Structures */

typedef struct {
	enum ControllerCommState comm;
	struct IdentPacket identity;

	// Buffer state
	uint8_t *bufferPtr;
	//uint_fast16_t sentPos; // Not used, protection is assumed by large UART buffer, hard to keep track of with shared buffers anyway
	uint_fast16_t queuedPos;
	uint_fast16_t parsePos;

	// Parse state
	bool inHeader;
	bool inData;
	bool ignoreData;
	uint8_t *headerPtr;	// Beginning of header in buffer
	uint_fast8_t headerPos; // Bytes of header that are in headerRaw
	uint8_t headerRaw[PACKET_HEADER_SIZE]; // Bytes of header to parse
	struct PacketHeader header; // Parsed header
	uint_fast16_t dataPos; // Position of data in packet

	// Supervision
	TimePoint lastComm;
	TimePoint lastTimeSync;
	uint8_t lastAnnounceID;
	uint8_t lastStreamID;

	// Streaming state
	bool cameraEnabled; // TODO: Needed? Probably not. But nice to know
	bool frameSyncEnabled;

	// Positions for USBD_HS to keep track of which data has been queued and which has been sent

	// queuedPos is set to current parsePos once a packet header is read
	// Once the UART packet completes (or a 1021-byte block is there) a USB packet is queued and it will be advanced
	// Thus it will always live within the current packets boundaries

	// sendPos is packet-independent, any data after it should not be overwritten by UART DMA yet
	// After a USB packet is sent off, it will be advanced to the beginning of the first queued packet that is not sent
	// Note: Not used, protection is assumed by large UART buffer, hard to keep track of with shared buffers anyway

	// parsePos is the position of first-stage parsing that happens
} PortState;

typedef enum
{
	uartd_accept,	// Accept and process data body
	uartd_reset,	// Reset communications, discard data body
	uartd_ignore,	// Ignore any data body (signals)
	uartd_unknown	// Ignore any data body (unknown packet)
} uartd_respond;

typedef struct
{
	uartd_respond (*uartd_handle_header)(uint_fast8_t port);
	uartd_respond (*uartd_handle_data)(uint_fast8_t port, uint8_t* ptr, uint_fast16_t size);
	void (*uartd_handle_packet)(uint_fast8_t port, uint_fast16_t endPos);
} uartd_callbacks;


/* Variables */

// UART State
extern PortState portStates[UART_PORT_COUNT];

// Predefined messages
extern uint8_t msg_ack[1+PACKET_HEADER_SIZE], msg_nak[1+PACKET_HEADER_SIZE], msg_ping[1+PACKET_HEADER_SIZE];


/* Functions */

void DisableUARTInterrupts();
void EnableUARTInterrupts();
void EnterUARTPortZone(uint8_t port);
void LeaveUARTPortZone(uint8_t port);

/**
 * Use within main loop without protections
 */
void uartd_send(uint_fast8_t port, const uint8_t* data, uint_fast16_t len, bool isOwner);

/**
 * Use from within a UART interrupt or an zone/interrupt that can't be preempted by a UART interrupt
 */
void uartd_send_int(uint_fast8_t port, const uint8_t* data, uint_fast16_t len, bool isOwner);

/**
 * Use within main loop without protections
 */
void uartd_reset_port(uint_fast8_t port);

/**
 * Use from within a UART interrupt or an zone/interrupt that can't be preempted by a UART interrupt
 */
void uartd_reset_port_int(uint_fast8_t port);

void uartd_init(uartd_callbacks impl_callbacks);

static inline void uartd_ack_int(uint_fast8_t port)
{
	uartd_send_int(port, msg_ack, sizeof(msg_ack), true);
}
static inline void uartd_nak_int(uint_fast8_t port)
{
	uartd_send_int(port, msg_nak, sizeof(msg_nak), true);
}

#ifdef __cplusplus
}
#endif

#endif /* __UARTD_H */
