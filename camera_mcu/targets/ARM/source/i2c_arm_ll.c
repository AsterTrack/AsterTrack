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
#include "stm32g0xx_ll_dma.h"
#endif

#include "i2c_driver.h"
#include "config_impl.h"
#include "mcu/util.h"

#include <stdint.h>

// Timing register values
//#define I2C_TIMING		__LL_I2C_CONVERT_TIMINGS(0x0, 0xC, 0x0, 0x21, 0x6C) // @400kHz, 64 MHz I2CCLK, 100ns rise time, 10ns fall time (STM32CubeMX)
//#define I2C_TIMING		__LL_I2C_CONVERT_TIMINGS(0x0, 0xC, 0x0, 0x40, 0x4f) // @400kHz, 64 MHz I2CCLK, 100ns rise time, 10ns fall time (code, no analog filter)
//#define I2C_TIMING		__LL_I2C_CONVERT_TIMINGS(0x0, 0xC, 0x0, 0x3D, 0x4C) // @400kHz, 64 MHz I2CCLK, 100ns rise time, 10ns fall time (code, WITH analog filter)
//#define I2C_TIMING		__LL_I2C_CONVERT_TIMINGS(0x1, 0xA, 0x2, 0x18, 0x27) // @400kHz, 64 MHz I2CCLK, 250ns rise time, 100ns fall time (code, no analog filter)
//#define I2C_TIMING		__LL_I2C_CONVERT_TIMINGS(0x0, 0x6, 0x0, 0x14, 0x1a) // @1MHz, 64 MHz I2CCLK, 60ns rise time, 10ns fall time (code, WITH analog filter)
#define I2C_TIMING		__LL_I2C_CONVERT_TIMINGS(0x0, 0xC, 0x0, 0x21, 0x6C) // @400kHz, 64 MHz I2CCLK, 100ns rise time, 10ns fall time (STM32CubeMX)

#define I2C_ADDRESS_MATCH	(MCU_I2C_ADDRESS << 1) // Match 7-bit address to 8bit space in register

// TODO: Rasberry Pi has Problems with clock stretching, so ideally we'll disable it to avoid future problems
// See https://www.advamation.com/knowhow/raspberrypi/rpi-i2c-bug.html
// But enabling it leaves us no time to prepare bytes to send after RX
// So we have to determine the first byte to send before fully handling the received data
// Hence need to have at least 1 leading byte (value 0x00) the SBC knows to discard (MCU_LEADING_BYTES)
// No actual issues have been observed yet, so we keep clock stretching on for now
// But perhaps keeping the leading bytes just in case to prevent any protocol changes would be good... 
// NOSTRECH with no leading bytes will NOT work - ZERO reaction time for ISR to parse RX and prepare TX
#define I2C_USE_CLOCK_STRETCHING	(true || MCU_LEADING_BYTES == 0)
#define I2C_PREPENDED_BYTES			(MCU_LEADING_BYTES > 1? (MCU_LEADING_BYTES-1) : 0) // Prepended bytes in addition to the first leading byte

static uint8_t receiveBuffer[256];
static uint8_t transmitBuffer[256];


/* Driver Functions */

void i2c_driver_init()
{
	LL_I2C_Disable(I2C1);

	// GPIO Clocks are already active

	// DMA & UART Peripheral clock enable
	LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	// AFIO
	static_assert(I2C_SCL_PIN < GPIO_PIN_8 && I2C_SDA_PIN < GPIO_PIN_8);
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
	static_assert(I2C_USE_CLOCK_STRETCHING || (MCU_LEADING_BYTES > 0));
	if (!I2C_USE_CLOCK_STRETCHING)
		LL_I2C_DisableClockStretching(I2C1);
	LL_I2C_EnableAnalogFilter(I2C1);
	LL_I2C_SetTiming(I2C1, I2C_TIMING);
	LL_I2C_SetOwnAddress1(I2C1, I2C_ADDRESS_MATCH, LL_I2C_OWNADDRESS1_7BIT);
	LL_I2C_EnableOwnAddress1(I2C1);
	//LL_I2C_EnableGeneralCall(I2C1);
	LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);

	// Enable UART DMA TX & RX
	LL_I2C_EnableDMAReq_TX(I2C1);
	LL_I2C_EnableDMAReq_RX(I2C1);

	// Setup DMAMUX found on STM32G0 series
	#define DMA_CH_RX DMA_CHANNEL_4
	#define DMA_CH_TX DMA_CHANNEL_5
	LL_DMA_SetPeriphRequest(DMA1, DMA_CH_RX, LL_DMAMUX_REQ_I2C1_RX);
	LL_DMA_SetPeriphRequest(DMA1, DMA_CH_TX, LL_DMAMUX_REQ_I2C1_TX);
	LL_DMA_EnableIT_TC(DMA1, DMA_CH_TX);
	NVIC_SetPriority(I2C1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
	NVIC_EnableIRQ(DMA1_Ch4_5_DMAMUX1_OVR_IRQn);

	// Setup respective DMA Channel for UART RX
	LL_DMA_SetDataTransferDirection(DMA1, DMA_CH_RX, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA1, DMA_CH_RX, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, DMA_CH_RX, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, DMA_CH_RX, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, DMA_CH_RX, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, DMA_CH_RX, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, DMA_CH_RX, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(DMA1, DMA_CH_RX, (uint32_t)&I2C1->RXDR);
	LL_DMA_SetMemoryAddress(DMA1, DMA_CH_RX, (uint32_t)receiveBuffer);
	LL_DMA_SetDataLength(DMA1, DMA_CH_RX, sizeof(receiveBuffer));

	// Setup respective DMA Channel for UART TX
	LL_DMA_SetDataTransferDirection(DMA1, DMA_CH_TX, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
	LL_DMA_SetChannelPriorityLevel(DMA1, DMA_CH_TX, LL_DMA_PRIORITY_HIGH);
	LL_DMA_SetMode(DMA1, DMA_CH_TX, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, DMA_CH_TX, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, DMA_CH_TX, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, DMA_CH_TX, LL_DMA_PDATAALIGN_BYTE);
	LL_DMA_SetMemorySize(DMA1, DMA_CH_TX, LL_DMA_MDATAALIGN_BYTE);
	LL_DMA_SetPeriphAddress(DMA1, DMA_CH_TX, (uint32_t)&I2C1->TXDR);

	if (I2C_USE_CLOCK_STRETCHING)
		NVIC_SetPriority(I2C1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	else // Absolutely need to react to ISR within the send time of the first MCU_LEADING_BYTES
		NVIC_SetPriority(I2C1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
	NVIC_EnableIRQ(I2C1_IRQn);

	// Enable I2C1 interrupts
	I2C1->CR1 |= I2C_CR1_ERRIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE | I2C_CR1_ADDRIE;

	LL_I2C_Enable(I2C1);
	if (MCU_LEADING_BYTES > 0)
	{ // Alread need to set first leading byte to send on next TX Start
		I2C1->TXDR = 0x00;
	}
}


/** I2C global interrupt handler */

uint32_t GetRXLength()
{
	return sizeof(receiveBuffer) - LL_DMA_GetDataLength(DMA1, DMA_CH_RX);
}

void I2C1_IRQHandler() __IRQ;
void I2C1_IRQHandler(void)
{
	uint32_t ISR = I2C1->ISR;

	if (ISR & I2C_ISR_ADDR)
	{
		uint8_t addr = (ISR&I2C_ISR_ADDCODE) >> I2C_ISR_ADDCODE_Pos << 1;
		if (addr == I2C_ADDRESS_MATCH)
		{ // Address matches own address

			// Disable TXIE for excess reads again
			I2C1->CR1 &= ~I2C_CR1_TXIE;

			if (ISR & I2C_ISR_DIR)
			{ // Prepare to transmit data
				TimePoint start = GetTimePoint();

				// Handle RX data to determine TX data
				uint8_t transmitLength = 0;
				if (LL_DMA_IsEnabledChannel(DMA1, DMA_CH_RX) && GetRXLength() > 0)
					transmitLength = i2cd_prepare_response((enum CameraMCUCommand)receiveBuffer[0], receiveBuffer+1, GetRXLength()-1, transmitBuffer+I2C_PREPENDED_BYTES);
				LL_DMA_DisableChannel(DMA1, DMA_CH_RX);

				// Prepare TX data with further leading bytes and a trailing byte which will reset TXDR to 0 after expected transmit length has been reached
				for (int i = 0; i < I2C_PREPENDED_BYTES; i++)
					transmitBuffer[i] = 0;
				transmitBuffer[transmitLength] = 0;

				if (MCU_LEADING_BYTES == 0)
				{ // Prepare first byte and setup to DMA the rest
					I2C1->TXDR = transmitBuffer[0];
					LL_DMA_SetMemoryAddress(DMA1, DMA_CH_TX, (uint32_t)transmitBuffer+1);
					LL_DMA_SetDataLength(DMA1, DMA_CH_TX, I2C_PREPENDED_BYTES+transmitLength-1+1);
				}
				else
				{ // First leading byte is already in TXDR, setup to DMA the rest
					LL_DMA_SetMemoryAddress(DMA1, DMA_CH_TX, (uint32_t)transmitBuffer);
					LL_DMA_SetDataLength(DMA1, DMA_CH_TX, I2C_PREPENDED_BYTES+transmitLength+1);
				}
				LL_DMA_EnableChannel(DMA1, DMA_CH_TX);

				if (I2C_USE_CLOCK_STRETCHING)
				{ // RPi Clock Stretching bug requires this ISR to take at least 1/2 I2CLK
					// At 400kHz that is just above 1us, so ensure we spend at least 2 in this ISR
					while (GetTimePoint() < start+2*TICKS_PER_US);
				}
			}
			else
			{ // Prepare to receive command ( + potentially data)
				LL_DMA_SetMemoryAddress(DMA1, DMA_CH_RX, (uint32_t)receiveBuffer);
				LL_DMA_SetDataLength(DMA1, DMA_CH_RX, sizeof(receiveBuffer));
				LL_DMA_EnableChannel(DMA1, DMA_CH_RX);
			}
		}
		// Else not being addressed after all (perhaps general call if enabled)

		I2C1->ICR |= I2C_ICR_ADDRCF;
	}
	if (ISR & I2C_ISR_STOPF)
	{ // Stopping RX or TX

		if (MCU_LEADING_BYTES == 0)
			I2C1->ISR |= I2C_ISR_TXE; // Clear last TX data
		else // Ensure first byte written is 0 on next TX start
			I2C1->TXDR = 0x00;
		// NOTE: If I2C_USE_CLOCK_STRETCHING, we could do this in TX start, since sending will wait for the TX start handling
		// but with NOSTRETCH, this first byte gives us important additional reaction time to prepare remaining TX bytes

		// Unless SBC starts sending STOP in between its TX and RX, this will only handle commands that don't expect a response
		if (LL_DMA_IsEnabledChannel(DMA1, DMA_CH_RX) && GetRXLength() > 0)
			i2cd_handle_command((enum CameraMCUCommand)receiveBuffer[0], receiveBuffer+1, GetRXLength()-1);

		// Disable whatever DMA channel was active just now
		LL_DMA_DisableChannel(DMA1, DMA_CH_RX);
		LL_DMA_DisableChannel(DMA1, DMA_CH_TX);

		I2C1->ICR |= I2C_ICR_STOPCF;
	}
	if (ISR & I2C_ISR_TXIS)
	{
		// Only called when TX DMA of expected data length finished, but SBC reads more
		// At that point, just write null-bytes
		I2C1->TXDR = 0x00;
	}
	if (ISR & I2C_ISR_NACKF)
	{ // End of TX
		I2C1->ICR |= I2C_ICR_NACKCF;
	}
	if (ISR & I2C_ISR_OVR)
	{ // Overrun/Underrun
		I2C1->ICR |= I2C_ICR_OVRCF;
	}
	if (ISR & I2C_ISR_ARLO)
	{ // Arbitration lost
		I2C1->ICR |= I2C_ICR_ARLOCF;
	}
	if (ISR & I2C_ISR_BERR)
	{ // Bus error
		I2C1->ICR |= I2C_ICR_BERRCF;
	}
}

void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() __IRQ;
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_TC5(DMA1))
	{ // Sent all data, enable TXIE incase there are excess reads
		I2C1->CR1 |= I2C_CR1_TXIE;
		LL_DMA_ClearFlag_TC5(DMA1);
	}
}