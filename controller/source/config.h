/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>


#define CAM_PORT_COUNT			8       // Should match UART_PORT_COUNT in uartd_conf.h

#define WWDG_TIMEOUT			0x5F


// Functions

void Setup_Peripherals();

void SYNC_Output_Init();
void SYNC_Input_Init();
void SYNC_Reset();

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */
