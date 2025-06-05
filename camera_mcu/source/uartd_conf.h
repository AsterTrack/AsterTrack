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
#define UART_PORT_COUNT			1

#define UART_RX_BUFFER_SIZE		1024
#define RX_HEADROOM 			0

#ifdef __cplusplus
}
#endif

#endif /* __UARTD_CONF_H */
