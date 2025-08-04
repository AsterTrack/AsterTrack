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

#include "comm/uart.h"

// Amount of UART ports as implemented in uartd
#define UART_PORT_COUNT			1

#define UART_RX_BUFFER_SIZE		1024
#define UART_TX_BUFFER_SIZE		1024

#define UART_HEADROOM 			0

// Size temporary buffer for receiving packets addressed at the MCU, not the Host
// Only Ident packet has any meaningful amount of data
#define UART_TEMP_PACKET_BUF 32

// UART Timing
#define UART_COMM_TIMEOUT_MS		250		// Controller sends a ping every 100ms when not already streaming
#define UART_IDENT_STARTUP_DELAY_MS 50		// Delay after startup before first ident packet is sent to the controller
#define UART_IDENT_INTERVAL_MS		100		// Interval at which the ident packet is sent to the controller
#define UART_RESET_TIMEOUT_MS		20		// Timeout after comm loss (NAK) during which all comms are ignored

#ifdef __cplusplus
}
#endif

#endif /* __UARTD_CONF_H */
