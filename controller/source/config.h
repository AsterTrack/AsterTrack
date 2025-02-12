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

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "compat.h"

// SYNC pins

#define CAM_PORT_COUNT			8       // Should match UART_PORT_COUNT in uartd_conf.h

const static uint32_t GPIOA_SYNC_PINS =
	GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
const static uint32_t GPIOD_SYNC_PINS =
	GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

static uint32_t GPIOD_SYNC_PIN[CAM_PORT_COUNT] = {
	GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13, GPIO_PIN_12
};

// SYNC IO pin(s) on GPIOB

const static uint32_t GPIOB_SYNC_IO_PINS = GPIO_PIN_8;

// SYNC IO EXTI lines

const static uint32_t GPIOE_SYNC_EXTI_LINES = EXTI_LINE_3;


// Functions

void Setup_Peripherals();

void SYNC_Output_Init();
void SYNC_Input_Init();
void SYNC_Reset();

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */
