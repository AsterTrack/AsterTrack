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

#if defined(STM32G0)
#include "stm32g0xx_ll_gpio.h"
#endif
#include "compat.h"

#include "comm/packet.h"

#define V0_3 1
#define V0_4 2
#define V1_0 2
#define V1_1 3

#if BOARD_REV == V0_3 // using hardware sync line over Cat5e

// RJ45 LED pins
#define RJLED_GPIO_X GPIOA
#define RJLED_GREEN_PIN GPIO_PIN_4
#define RJLED_ORANGE_PIN GPIO_PIN_7

// WS2812 Pin
#define WS2812_GPIO_X GPIOA
#define WS2812_PIN GPIO_PIN_6

// UART Select Pin
#define UARTSEL_GPIO_X GPIOA
#define UARTSEL_TX_PIN 0 // N/A
#define UARTSEL_RX_PIN 0 // N/A

// Filter Switcher Actuator Pins
#define FILTERSW_GPIO_X GPIOB
#define FILTERSW_INFRARED_PIN GPIO_PIN_0
#define FILTERSW_VISIBLE_PIN GPIO_PIN_1
#define FILTERSW_PIN_SLEEP GPIO_PIN_2

// Sync Input pin
#define SYNC_GPIO_X GPIOB
#define SYNC_PIN GPIO_PIN_9

// Camera FSIN pin
#define FSIN_GPIO_X GPIOA
#define FSIN_PIN GPIO_PIN_8

// Camera STROBE pin
#define STROBE_GPIO_X GPIOB
#define STROBE_PIN 0 // N/A

// Button input pins
#define BUTTONS_GPIO_X GPIOA
#define BUTTON_BOTTOM_PIN GPIO_PIN_1
#define BUTTON_TOP_PIN GPIO_PIN_2
#define BUTTON_READ(GPIOX, PIN) (!GPIO_READ(GPIOX, PIN))

// VSense ADC pins
#define VSENSE_GPIO_X GPIOA
#define VSENSE_USB_PIN 0 // N/A
#define VSENSE_VCC_PIN GPIO_PIN_0

// Pi I2C pins
#define I2C_SXX_GPIO_X GPIOB
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_INT_GPIO_X GPIOB
#define I2C_INT_PIN GPIO_PIN_8

#define WWDG_TIMEOUT 0x7F

#else
// BOARD_REV == V1_0 (also V0.4.x) using UART sync
// BOARD_REV == V1_1 using UART or nRF sync

// RJ45 LED pins
#define RJLED_GPIO_X GPIOB
#define RJLED_GREEN_PIN GPIO_PIN_1
#define RJLED_ORANGE_PIN GPIO_PIN_0

// WS2812 Pin
#define WS2812_GPIO_X GPIOB
#define WS2812_PIN GPIO_PIN_8

// UART Select Pins
#define UARTSEL_GPIO_X GPIOA
#if BOARD_REV == V1_0
#define UARTSEL_TX_PIN GPIO_PIN_3
#define UARTSEL_RX_PIN GPIO_PIN_3
#elif BOARD_REV >= V1_1
#define UARTSEL_TX_PIN GPIO_PIN_11
#define UARTSEL_RX_PIN GPIO_PIN_12
#endif

// Filter Switcher Actuator Pins
#define FILTERSW_GPIO_X GPIOA
#define FILTERSW_INFRARED_PIN GPIO_PIN_6
#define FILTERSW_VISIBLE_PIN GPIO_PIN_7
#define FILTERSW_PIN_SLEEP 0 // N/A

// Sync Input pin
#define SYNC_GPIO_X GPIOB
#define SYNC_GPIO_IDX 0x01  // EXTI Port Index
#define SYNC_PIN 0 // N/A

// Camera FSIN pin
#define FSIN_GPIO_X GPIOA
#define FSIN_PIN GPIO_PIN_4

// Camera STROBE pin
#define STROBE_GPIO_X GPIOB
#if BOARD_REV == V1_0
#define STROBE_PIN GPIO_PIN_9
#elif BOARD_REV >= V1_1
#define STROBE_PIN 0 // N/A
#endif

// Button input pins
#define BUTTONS_GPIO_X GPIOA
#define BUTTON_BOTTOM_PIN GPIO_PIN_0
#define BUTTON_TOP_PIN GPIO_PIN_1
#if BOARD_REV == V1_0
#define BUTTON_READ(GPIOX, PIN) (!GPIO_READ(GPIOX, PIN))
#elif BOARD_REV >= V1_1
#define BUTTON_READ(GPIOX, PIN) GPIO_READ(GPIOX, PIN)
#endif

// VSense ADC pins
#define VSENSE_GPIO_X GPIOA
#if BOARD_REV == V1_0
#define VSENSE_USB_PIN 0 // N/A
#define VSENSE_VCC_PIN GPIO_PIN_2
#elif BOARD_REV >= V1_1
#define VSENSE_USB_PIN GPIO_PIN_2
#define VSENSE_VCC_PIN GPIO_PIN_3
#endif

// Pi I2C pins
#define I2C_SXX_GPIO_X GPIOB
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PIN GPIO_PIN_7
#define I2C_INT_GPIO_X GPIOA
#define I2C_INT_PIN GPIO_PIN_8

#define WWDG_TIMEOUT 0x7F

#endif

#if BOARD_REV >= V1_1

// Voltage Source Control pins
#define VSOURCE_DISABLE_5V_GPIO_X GPIOA
#define VSOURCE_DISABLE_5V_PIN GPIO_PIN_5
#define VSOURCE_SELECTED_GPIO_X GPIOC
#define VSOURCE_SELECTED_PIN GPIO_PIN_6

// nRF SPI pins
#define SPI_GPIO_X GPIOB
#define SPI_SCK_PIN GPIO_PIN_3
#define SPI_MISO_PIN GPIO_PIN_4
#define SPI_MOSI_PIN GPIO_PIN_5

// nRF Control pins
#define NRF_CTRL_GPIO_X GPIOB
#define NRF_CTRL_CE_PIN GPIO_PIN_2
#define NRF_CTRL_CSN_PIN GPIO_PIN_9

// nRF IRQ pin
#define NRF_IRQ_GPIO_X GPIOA
#define NRF_IRQ_PIN GPIO_PIN_15

#else

// Voltage Source Control pins
#define VSOURCE_DISABLE_5V_GPIO_X GPIOA
#define VSOURCE_DISABLE_5V_PIN 0 // N/A
#define VSOURCE_SELECTED_GPIO_X GPIOC
#define VSOURCE_SELECTED_PIN 0 // N/A

#endif

// Shared Defines

#define FSIN_PULSE_WIDTH_US		10
#define FILTER_SWITCHER_COIL_PULSE_MS	100

// Detected Hardware Features

extern bool hasHSEClock;

// Functions

void Setup_Peripherals();

void EnableADC();
void DisableADC();
void StartADCMonitor(bool monitorVUSB);
void StopADCMonitor();
uint32_t GetMillivolts();
bool IsUsingUSBPower();
void SetUSBPowerEnabled(bool enable);

enum CameraMCUFlashConfig ReadFlashConfiguration();
enum CameraMCUFlashConfig SetFlashConfiguration(enum CameraMCUFlashConfig config);

bool IsOTPEmpty();
uint16_t EraseFlashPage(uint16_t page);
uint16_t ProgramFlash(volatile uint32_t *address, uint32_t *data, uint16_t length);
uint16_t EraseAndProgramFlash(volatile uint32_t *address, uint32_t *data, uint16_t length);

void SwitchToBootloader();
void EnableWatchdog();
void SafeDelayMS(uint16_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_IMPL_H */
