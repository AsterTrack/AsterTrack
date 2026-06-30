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

#ifdef USE_SPI_NRF_SYNC

#if defined(STM32G0)
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_wwdg.h"
#endif

#include "spi_nrf_driver.h"
#include "config_impl.h"
#include "util.h"

#include <stdint.h>
#include <string.h>


volatile bool spi_nrf_lock;
static bool enabledReceiveCallback;
static uint8_t currentCommand;
static uint8_t length, txRem, rxPos;
static uint8_t status; // Status received during first 8bits
static uint8_t receiveBuffer[NRF_SPI_MAX_MESSAGE_LENGTH];
static uint8_t transmitBuffer[NRF_SPI_MAX_MESSAGE_LENGTH];
static uint8_t *transmitPtr;


/* Driver Functions */

void spi_driver_init()
{
	LL_SPI_Disable(SPI1);

	// GPIO Clocks are already active

	// DMA & SPI Peripheral clock enable
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	// Init GPIO pins
	// SCK:
	LL_GPIO_SetPinMode(SPI_GPIO_X, SPI_SCK_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(SPI_GPIO_X, SPI_SCK_PIN, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(SPI_GPIO_X, SPI_SCK_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(SPI_GPIO_X, SPI_SCK_PIN, LL_GPIO_PULL_NO);
	// MISO:
	LL_GPIO_SetPinMode(SPI_GPIO_X, SPI_MISO_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(SPI_GPIO_X, SPI_MISO_PIN, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(SPI_GPIO_X, SPI_MISO_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(SPI_GPIO_X, SPI_MISO_PIN, LL_GPIO_PULL_NO);
	// MOSI:
	LL_GPIO_SetPinMode(SPI_GPIO_X, SPI_MOSI_PIN, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinSpeed(SPI_GPIO_X, SPI_MOSI_PIN, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinOutputType(SPI_GPIO_X, SPI_MOSI_PIN, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull(SPI_GPIO_X, SPI_MOSI_PIN, LL_GPIO_PULL_NO);

	// Init SPI
	LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV32); // 2Mbaud. Could go up to 8Mbaud
	LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
	LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
	LL_SPI_SetTransferDirection(SPI1, LL_SPI_FULL_DUPLEX);
	LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);

	LL_SPI_DisableCRC(SPI1);
	LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
	LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
	LL_SPI_DisableNSSPulseMgt(SPI1);

	// Enable SPI interrupts
	LL_SPI_EnableIT_ERR(SPI1);
	LL_SPI_EnableIT_RXNE(SPI1);
	// Configure RXNE to trigger at 8bits in 32bit FIFO
	// 16bit works too but complicates last byte handling of odd-length packets.
	LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

	NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
	NVIC_EnableIRQ(SPI1_IRQn);

	LL_SPI_Enable(SPI1);
}

inline bool spi_send(uint8_t command, uint8_t *data, uint8_t len, bool callback)
{
	if (spi_nrf_lock)
		return false;
	spi_nrf_lock = true;
	// Notify nRF of upcoming read
	GPIO_RESET(NRF_CTRL_GPIO_X, NRF_CTRL_CSN_PIN);
	currentCommand = command;
	enabledReceiveCallback = callback;
	status = 0xFF;
	transmitPtr = data;
	uint8_t sendImmediate = len < 3? len : 3;
	txRem = len-sendImmediate;
	rxPos = 0;
	length = len;
	LL_SPI_TransmitData8(SPI1, command);
	for (int i = 0; i < sendImmediate; i++)
		LL_SPI_TransmitData8(SPI1, *transmitPtr++);
	if (txRem)
		LL_SPI_EnableIT_TXE(SPI1);
	return true;
}

bool spi_write(uint8_t command, uint8_t *data, uint8_t len)
{
	if (spi_nrf_lock)
		return false;
	memcpy(transmitBuffer, data, len);
	return spi_send(command, transmitBuffer, len, false);
}

bool spi_write_sync(uint8_t command, uint8_t *data, uint8_t len)
{
	if (!spi_write(command, data, len))
		return false;
	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);
	while (spi_nrf_lock);
	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);
	return true;
}

bool spi_read(uint8_t command, uint8_t len)
{
	if (spi_nrf_lock)
		return false;
	memset(transmitBuffer, 0xFF, len);
	return spi_send(command, transmitBuffer, len, true);
}


/** SPI global interrupt handler */

void SPI1_IRQHandler() __IRQ;
void SPI1_IRQHandler(void)
{
	if (LL_SPI_IsActiveFlag_OVR(SPI1))
	{ // Overrun Error, only relevant error
		LL_SPI_ClearFlag_OVR(SPI1);
	}
	if (LL_SPI_IsActiveFlag_RXNE(SPI1))
	{ // Called every time a byte is in FiFo
		//uint32_t fifo = LL_SPI_GetRxFIFOLevel(SPI1);
		if (status == 0xFF)
		{ // Receive status as first byte (guaranteed to be != 0xFF by nRF)
			status = LL_SPI_ReceiveData8(SPI1);
			spid_receive_status(currentCommand, status);
		}
		else
			receiveBuffer[rxPos++] = LL_SPI_ReceiveData8(SPI1);

		if (rxPos == length)
		{ // Received last byte, whether it was a read or write, this marks the end of the operation
			// Signal end to nRF, just needs to be over 2ns after last bit was received
			GPIO_SET(NRF_CTRL_GPIO_X, NRF_CTRL_CSN_PIN);
			// CSN needs to stay high for at least 50ns before next command starts
			// That's around 3 clock cycles at 64Mhz, so no extra handling required
			spi_nrf_lock = false; // Mark end of command, ready for next
			// Handle received data (excluding status)
			if (enabledReceiveCallback)
				spid_receive_response(currentCommand, receiveBuffer, length);
		}
	}
	if (LL_SPI_IsActiveFlag_TXE(SPI1) && txRem)
	{
		// TX FIFO is at least half-empty, so 16bit can be written at the very least
		// Or we could check if there is perhaps more empty if IRQHandler was delayed
		//uint32_t fifo = LL_SPI_GetTxFIFOLevel(SPI1);
		// Queue next byte
		LL_SPI_TransmitData8(SPI1, *transmitPtr++);
		txRem--;
		if (txRem)
		{ // Queue next byte
			LL_SPI_TransmitData8(SPI1, *transmitPtr++);
			txRem--;
		}
		// Could also merge into LL_SPI_TransmitData16, but only if at least 2 bytes are still queued

		// Disable TXE interrupt entirely when done, otherwise will keep spamming interrupt
		if (txRem == 0)
			LL_SPI_DisableIT_TXE(SPI1);
	}
}

#endif // USE_SPI_NRF_SYNC