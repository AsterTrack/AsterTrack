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

#ifndef __CONFIG_IMPL_H
#define __CONFIG_IMPL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "compat.h"


#ifdef BOARD_OLD

// RJ45 LED pins
#define RJLED_GPIO_X GPIOA
const static uint32_t RJLED_GREEN_PIN = GPIO_PIN_4;
const static uint32_t RJLED_ORANGE_PIN = GPIO_PIN_7;

// WS2812 Pin
#define WS2812_GPIO_X GPIOA
const static uint32_t WS2812_PIN = GPIO_PIN_6;

// UART Select Pin
#define UARTSEL_GPIO_X GPIOA
const static uint32_t UARTSEL_PIN = 0;

// Filter Switcher Actuator Pins
#define FILTERSW_GPIO_X GPIOB
const static uint32_t FILTERSW_INFRARED_PIN = GPIO_PIN_0;
const static uint32_t FILTERSW_VISIBLE_PIN = GPIO_PIN_1;
const static uint32_t FILTERSW_PIN_SLEEP = GPIO_PIN_2;

// Sync Input pin
#define SYNC_GPIO_X GPIOB
const static uint32_t SYNC_PIN = GPIO_PIN_9;

// Camera FSIN pin
#define FSIN_GPIO_X GPIOA
const static uint32_t CAMERA_FSIN_PIN = GPIO_PIN_8;

// Camera STROBE pin
#define STROBE_GPIO_X GPIOB
const static uint32_t CAMERA_STROBE_PIN = 0;

// Button input pins
#define BUTTONS_GPIO_X GPIOA
const static uint32_t BUTTON_BOTTOM_PIN = GPIO_PIN_1;
const static uint32_t BUTTON_TOP_PIN = GPIO_PIN_2;

// VSense ADC pin
#define VSENSE_GPIO_X GPIOA
const static uint32_t VSENSE_ADC_PIN = GPIO_PIN_0;

#else

// RJ45 LED pins
#define RJLED_GPIO_X GPIOB
const static uint32_t RJLED_GREEN_PIN = GPIO_PIN_1;
const static uint32_t RJLED_ORANGE_PIN = GPIO_PIN_0;

// WS2812 Pin
#define WS2812_GPIO_X GPIOB
const static uint32_t WS2812_PIN = GPIO_PIN_8;

// UART Select Pin
#define UARTSEL_GPIO_X GPIOA
const static uint32_t UARTSEL_PIN = GPIO_PIN_3;

// Filter Switcher Actuator Pins
#define FILTERSW_GPIO_X GPIOA
const static uint32_t FILTERSW_INFRARED_PIN = GPIO_PIN_6;
const static uint32_t FILTERSW_VISIBLE_PIN = GPIO_PIN_7;
const static uint32_t FILTERSW_PIN_SLEEP = 0;

// Sync Input pin
#define SYNC_GPIO_X GPIOB
#define SYNC_GPIO_IDX 0x01  // EXTI Port Index
const static uint32_t SYNC_PIN = 0;

// Camera FSIN pin
#define FSIN_GPIO_X GPIOA
const static uint32_t CAMERA_FSIN_PIN = GPIO_PIN_4;

// Camera STROBE pin
#define STROBE_GPIO_X GPIOB
const static uint32_t CAMERA_STROBE_PIN = GPIO_PIN_9;

// Button input pins
#define BUTTONS_GPIO_X GPIOA
const static uint32_t BUTTON_BOTTOM_PIN = GPIO_PIN_0;
const static uint32_t BUTTON_TOP_PIN = GPIO_PIN_1;

// VSense ADC pin
#define VSENSE_GPIO_X GPIOA
const static uint32_t VSENSE_ADC_PIN = GPIO_PIN_2;

#endif

// Functions

void Setup_Peripherals();

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_IMPL_H */
