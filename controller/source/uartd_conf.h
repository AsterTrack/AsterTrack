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

#ifndef __UARTD_CONF_H
#define __UARTD_CONF_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "comm/packet.h"

// Amount of UART ports as implemented in uartd
#define UART_PORT_COUNT			8       // Should match CAM_PORT_COUNT in config.h

// Strategy: Write into huge UART buffer, observe from interrupts or main loop and set USB to DMA from parts of that buffer
#define UART_TX_BLOCKS			8
#define USBD_PACKET_SIZE		1024
#define UART_RX_BUFFER_SIZE		USBD_PACKET_SIZE*UART_TX_BLOCKS // Technically 1021 since 3bytes of USB Block Header are needed, but no matter

// Needs some headroom to write headers & co. to DMA a PacketRef directly from this buffer
// USB_PACKET_HEADER: Every USB packet has a counter and timestamp
// BLOCK_HEADER_SIZE: Block header describing data
// PACKET_HEADER_SIZE: In case header was split by UART buffer end, will be copied to beginning, too
#define RX_HEADROOM 			USB_PACKET_HEADER+BLOCK_HEADER_SIZE+PACKET_HEADER_SIZE

#ifdef __cplusplus
}
#endif

#endif /* __UARTD_CONF_H */
