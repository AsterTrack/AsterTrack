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

#if defined(STM32G0)
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_i2c.h"
#include "stm32g0xx_ll_rcc.h"
#endif

#include "comm/commands.h"
#include "config_impl.h"

#include <stdint.h>

/* Timing register value is computed with the STM32CubeMX Tool,
* Fast Mode @400kHz with I2CCLK = 64 MHz,
* rise time = 100ns, fall time = 10ns
* Timing Value = (uint32_t)0x00C0216C
*/
#define I2C_TIMING               __LL_I2C_CONVERT_TIMINGS(0x0, 0xC, 0x0, 0x21, 0x6C) // 0x00C0216C

#define I2C_ADDRESS_MATCH (MCU_I2C_ADDRESS << 1) // Needs to be left-shifted to fit register

static enum CameraMCUCommand command;
static uint8_t receiveBuffer[256];
static uint8_t transmitBuffer[256];
static uint8_t transmitLength;
static int receiveCounter, transmitCounter;
static bool processedRX = false;


/* Application Functions */

bool i2cd_handle_command(enum CameraMCUCommand command, uint8_t *data, uint8_t len);
uint8_t i2cd_prepare_response(enum CameraMCUCommand command, uint8_t *data, uint8_t len, uint8_t response[256]);


/* Driver Functions */

void i2c_driver_init()
{
	// GPIO Clocks are already active

	// DMA & UART Peripheral clock enable
	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	// AFIO
	//static_assert(I2C_SCL_PIN < GPIO_PIN_8 && I2C_SDA_PIN < GPIO_PIN_8);
	LL_GPIO_SetAFPin_0_7(I2C_SXX_GPIO_X, I2C_SCL_PIN, LL_GPIO_AF_6);
	LL_GPIO_SetAFPin_0_7(I2C_SXX_GPIO_X, I2C_SDA_PIN, LL_GPIO_AF_6);

	// Init GPIO pins
	// SCL:
	LL_GPIO_SetPinMode(I2C_SXX_GPIO_X, I2C_SCL_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(I2C_SXX_GPIO_X, I2C_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(I2C_SXX_GPIO_X, I2C_SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
#if defined(BOARD_OLD)
	LL_GPIO_SetPinPull(I2C_SXX_GPIO_X, I2C_SCL_PIN, LL_GPIO_PULL_UP);
#else
	LL_GPIO_SetPinPull(I2C_SXX_GPIO_X, I2C_SCL_PIN, LL_GPIO_PULL_NO);
#endif
	// SDA:
	LL_GPIO_SetPinMode(I2C_SXX_GPIO_X, I2C_SDA_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(I2C_SXX_GPIO_X, I2C_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(I2C_SXX_GPIO_X, I2C_SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
#if defined(BOARD_OLD)
	LL_GPIO_SetPinPull(I2C_SXX_GPIO_X, I2C_SDA_PIN, LL_GPIO_PULL_UP);
#else
	LL_GPIO_SetPinPull(I2C_SXX_GPIO_X, I2C_SDA_PIN, LL_GPIO_PULL_NO);
#endif

	// Init I2C
	LL_I2C_SetTiming(I2C1, I2C_TIMING);
	LL_I2C_SetOwnAddress1(I2C1, I2C_ADDRESS_MATCH, LL_I2C_OWNADDRESS1_7BIT);
	LL_I2C_EnableOwnAddress1(I2C1);
	//LL_I2C_EnableGeneralCall(I2C1);
	LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);

	NVIC_SetPriority(I2C1_IRQn, 2);
	NVIC_EnableIRQ(I2C1_IRQn);

	LL_I2C_Enable(I2C1);

	// Enable I2C1 interrupts
	LL_I2C_EnableIT_ADDR(I2C1);
	LL_I2C_EnableIT_NACK(I2C1);
	LL_I2C_EnableIT_ERR(I2C1);
	LL_I2C_EnableIT_STOP(I2C1);
}


/** I2C global interrupt handler */

void I2C1_IRQHandler(void)
{
	// Sequence is different from events on the line, for some reason:
	// RX 1			Start RX, 			RX Bytes, 			STOP 
	// RX 1 + TX 1:	Start RX, Start TX, RX Bytes, TX Bytes, STOP
	// TX Byte will occur one more than is actually queried (filling next byte)
	// So Start TX has to flush last transfers garbage byte from TXDR by setting TXE
	// There's no event for processing, so first TX Byte event will process the request

	if (LL_I2C_IsActiveFlag_ADDR(I2C1))
	{
		if (LL_I2C_GetAddressMatchCode(I2C1) == I2C_ADDRESS_MATCH)
		{ // Address matches own address
			LL_I2C_ClearFlag_ADDR(I2C1);

			if (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_READ)
			{ // Prepare to transmit data

				// Flush data in TXDR before writing first byte
				LL_I2C_ClearFlag_TXE(I2C1);

				// Enable TX interrupt
				LL_I2C_EnableIT_TX(I2C1);

				// Can reset TX counter here or later when processing
				transmitCounter = 0;
			}
			else
			{ // Prepare to receive command ( + potentially data)
				command = MCU_CMD_NONE;
				receiveCounter = 0;
				processedRX = false;
			}
		}
		else
		{ // Not being addressed (perhaps general call if enabled)
			LL_I2C_ClearFlag_ADDR(I2C1);
		}
	}
	else if (LL_I2C_IsActiveFlag_NACK(I2C1))
	{ // End of TX
		LL_I2C_ClearFlag_NACK(I2C1);
	}
	else if (LL_I2C_IsActiveFlag_RXNE(I2C1))
	{
		if (receiveCounter == 0)
		{ // Receiving command byte
			command = (enum CameraMCUCommand)LL_I2C_ReceiveData8(I2C1);
		}
		else
		{ // Receiving further data
			receiveBuffer[receiveCounter-1] = LL_I2C_ReceiveData8(I2C1);
		}
		receiveCounter++;
	}
	else if (LL_I2C_IsActiveFlag_TXIS(I2C1))
	{
		if (!processedRX)
		{ // First TX after receiving, process RX data
			transmitLength = 0;
			transmitCounter = 0;
			processedRX = true;
			if (receiveCounter > 0)
				transmitLength = i2cd_prepare_response(command, receiveBuffer, receiveCounter-1, transmitBuffer);
		}

		if (transmitCounter < transmitLength)
		{ // Write next byte
			LL_I2C_TransmitData8(I2C1, transmitBuffer[transmitCounter]);
		}
		else
		{ // Might be last byte, won't be sent. Or Pi is doing something strange.
			LL_I2C_TransmitData8(I2C1, 0x00);
		}
		transmitCounter++;
	}
	else if (LL_I2C_IsActiveFlag_STOP(I2C1))
	{ // Stopping RX or TX
		LL_I2C_ClearFlag_STOP(I2C1);

		if (!processedRX && receiveCounter > 0)
		{ // Just received data without expectation to respond, handle
			i2cd_handle_command(command, receiveBuffer, receiveCounter-1);
		}
	}
	else if (!LL_I2C_IsActiveFlag_TXE(I2C1))
	{
		// Currently transmitting data, and TXDR is not empty, so all good
	}
	else
	{
		// TODO: Handle as error?
	}
}