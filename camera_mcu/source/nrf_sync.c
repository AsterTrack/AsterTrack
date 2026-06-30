
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
#include "stm32g030xx.h"
#include "stm32g0xx_ll_wwdg.h"
#include "stm32g0xx_ll_gpio.h"
#endif
#include "compat.h"

#include "nrf_sync.h"
#include "util.h"
#include "comm/commands.h"
#include "config_impl.h"
#include "spi_nrf_driver.h"
#include "mcu/timesync.h"

#include <string.h>

uint8_t ADDR_BROADCAST_SYNC[] = { 0x25, 0x7E, 0xA5 }; // LSB First
uint8_t ADDR_SYNC_CONTROLLER[] = { 0x2C, 0x7E, 0xA5 }; // LSB First

static uint8_t lastStatus;

static volatile bool lastSentSync = false;

static volatile bool preloadedTX = false;
static volatile bool preloadingTX = false;

static volatile bool handlingRX = false;
static uint8_t rxPipe;
static TimePoint rxTime;
static TimePoint commandTime;

void nrf_configure_rx(uint8_t cameraAddress[3])
{
	// Disable automatic acknowledgement for sync broadcast
	uint8_t EN_AA = 0b111101;
	spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &EN_AA, 1);

	// Enable data pipes 0 and 1 (default)
	uint8_t EN_RXADDR = 0b000011;
	spi_write_sync(0x02 | NRF_SPI_WRITE_REG, &EN_RXADDR, 1);

	// Setup address width to 3bytes
	uint8_t SETUP_AW = 0b01;
	spi_write_sync(0x03 | NRF_SPI_WRITE_REG, &SETUP_AW, 1);

	// Setup automatic retry (for TX to controller)
	// uint8_t SETUP_RETR = 0b00000011; // (Default)
	// spi_write_sync(0x04 | NRF_SPI_WRITE_REG, &SETUP_RETR, 1);

	// Setup RF channel used
	uint8_t RF_CH = 0b0001011;
	spi_write_sync(0x05 | NRF_SPI_WRITE_REG, &RF_CH, 1);

	// Setup RF configuration (default)
	// uint8_t RF_SETUP = 0b01011; // 2Mbps, highest gain
	// spi_write_sync(0x06 | NRF_SPI_WRITE_REG, &RF_SETUP, 1);

	static_assert(NRF_CAMERA_SPECIFIC_PIPE == 0);
	static_assert(NRF_SYNC_BROADCAST_PIPE == 1);

	// Setup unique RX address for data pipe 0 (LSB first)
	uint8_t *RX_ADDR_P0 = cameraAddress;
	spi_write_sync(0x0A | NRF_SPI_WRITE_REG, RX_ADDR_P0, 3);

	// Setup data pipe 1 as first broadcast (sync)
	uint8_t *RX_ADDR_P1 = ADDR_BROADCAST_SYNC;
	spi_write_sync(0x0B | NRF_SPI_WRITE_REG, RX_ADDR_P1, 3);

	// Setup fixed size of sync broadcast on data pipe 1
	uint8_t RX_PW_P1 = NRF_SYNC_BROADCAST_LEN;
	spi_write_sync(0x12 | NRF_SPI_WRITE_REG, &RX_PW_P1, 1);

	// Setup dynamic payload size for data pipe 0
	uint8_t DYNPD = 0b000001;
	spi_write_sync(0x1C | NRF_SPI_WRITE_REG, &DYNPD, 1);

	// Select desired feature flags
	uint8_t FEATURE = 0b100;
	spi_write_sync(0x1D | NRF_SPI_WRITE_REG, &FEATURE, 1);

	// Request to apply selected features
	uint8_t ACTIVATE = 0x73;
	spi_write_sync(NRF_SPI_ACTIVATE, &ACTIVATE, 1);
}

void nrf_start_rx()
{
	// Write config to start up and enter RX mode
	uint8_t CONFIG = 0b00001011;
	spi_write_sync(0x00 | NRF_SPI_WRITE_REG, &CONFIG, 1);

	// Enable RF (Receive)
	GPIO_SET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);
}

void nrf_configure_tx()
{
	// Enable automatic acknowledgement for static controller address
	uint8_t EN_AA = 0b111111;
	spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &EN_AA, 1);

	// Enable data pipes 0 and 1 (default)
	// uint8_t EN_RXADDR = 0b000011;
	// spi_write_sync(0x02 | NRF_SPI_WRITE_REG, &EN_RXADDR, 1);

	// Setup address width to 3bytes
	uint8_t SETUP_AW = 0b01;
	spi_write_sync(0x03 | NRF_SPI_WRITE_REG, &SETUP_AW, 1);

	// Setup automatic retry (for TX with Auto-Acknowledgements)
	// uint8_t SETUP_RETR = 0b00000011; // (Default)
	// spi_write_sync(0x04 | NRF_SPI_WRITE_REG, &SETUP_RETR, 1);

	// Setup RF channel used
	uint8_t RF_CH = 0b0001011;
	spi_write_sync(0x05 | NRF_SPI_WRITE_REG, &RF_CH, 1);

	// Setup RF configuration (default)
	// uint8_t RF_SETUP = 0b01011; // 2Mbps, highest gain
	// spi_write_sync(0x06 | NRF_SPI_WRITE_REG, &RF_SETUP, 1);

	// Data pipe 0 is used to receive any ACKs only
	// RX Address is configured as TX address when ACK is expected
	// DYNPD is kept off for now, for ACKs without payload
	// But ALSO need to disable Auto-ACK on pipe 0 if sending legacy ShockBurst packets!

	// Setup data pipe 1 as static controller address
	uint8_t *RX_ADDR_P1 = ADDR_SYNC_CONTROLLER;
	spi_write_sync(0x0B | NRF_SPI_WRITE_REG, RX_ADDR_P1, 3);

	// Setup dynamic payload size for data pipe 1
	uint8_t DYNPD = 0b000010;
	spi_write_sync(0x1C | NRF_SPI_WRITE_REG, &DYNPD, 1);

	// Select desired feature flags
	uint8_t FEATURE = 0b100;
	spi_write_sync(0x1D | NRF_SPI_WRITE_REG, &FEATURE, 1);

	// Request to apply selected features
	uint8_t ACTIVATE = 0x73;
	spi_write_sync(NRF_SPI_ACTIVATE, &ACTIVATE, 1);
}

void nrf_tx_powerup()
{
	// Write config to start up in TX mode
	uint8_t CONFIG = 0b00001010;
	spi_write_sync(0x00 | NRF_SPI_WRITE_REG, &CONFIG, 1);
}

bool nrf_tx_camera(uint8_t cameraAddress[3], uint8_t *data, uint8_t length)
{
	// Check for pending TX that's still awaiting a trigger
	if (preloadedTX) return false;

	// Setup camera address for TX
	uint8_t *TX_ADDR = cameraAddress;
	if (!spi_write_sync(0x10 | NRF_SPI_WRITE_REG, TX_ADDR, 3)) return false;

	// Enable Auto-Acknowledgement for pipe 0 for transmitting
	uint8_t EN_AA = 0b1111101;
	if (!spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &EN_AA, 1)) return false;

	// Setup camera address as RX Address for data pipe 0 for ACKs
	uint8_t *RX_ADDR_P0 = cameraAddress;
	if (!spi_write_sync(0x0A | NRF_SPI_WRITE_REG, RX_ADDR_P0, 3)) return false;

	// Write payload but don't wait, just pull CE high
	if (!spi_send(NRF_SPI_W_TX_PAYLOAD, data, length, true)) return false;

	// Enable RF, will transmit as soon as payload is transferred
	GPIO_SET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);

	lastSentSync = false;
	return true;
}

bool nrf_prepare_broadcast_sync(uint8_t data[NRF_SYNC_BROADCAST_LEN])
{
	// Wait for any existing transfer
	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);
	while (spi_nrf_lock);
	LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

	if (!lastSentSync)
	{
		// Setup camera address for TX
		uint8_t *TX_ADDR = ADDR_BROADCAST_SYNC;
		if (!spi_write_sync(0x10 | NRF_SPI_WRITE_REG, TX_ADDR, 3)) return false;

		// Disable Auto-Acknowledgement for pipe 0 for transmitting
		uint8_t EN_AA = 0b111110;
		if (!spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &EN_AA, 1)) return false;

		lastSentSync = true;
	}

	preloadingTX = true;
	preloadedTX = true;

	// Write payload but don't wait, just pull CE high
	return spi_send(NRF_SPI_W_TX_PAYLOAD, data, NRF_SYNC_BROADCAST_LEN, true);
}

void nrf_tx_trigger()
{
	// Enable RF, will transmit as soon as payload is transferred
	GPIO_SET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);
	delayUS(12);
	// Then if TX was already preloaded, disable again, nRF knows to send it from pulse
	__disable_irq();
	if (!preloadingTX)
		GPIO_RESET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);
	else // Triggered already, ensure CE is cleared when payload is loaded and transmitting
	 	preloadingTX = false;
	preloadedTX = false;
	__enable_irq();
}


/* ------ SPI NRF Behaviour ------ */

void spid_receive_status(uint8_t command, uint8_t status)
{
	commandTime = GetTimePoint();
	lastStatus = status;
	if (command == (0x07 | NRF_SPI_WRITE_REG))
	{
		if (lastStatus & NRF_STATUS_RX_DR)
		{
			handlingRX = true;
			rxTime = commandTime;
			rxPipe = (lastStatus & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_POS;
			//if (rxPipe == NRF_SYNC_BROADCAST_PIPE)
			// TODO: Got new sync packet, this is the earliest point we know
			// But we don't know the contents yet, so may not be useful
		}
		if (lastStatus & NRF_STATUS_TX_DS)
		{
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		}
		if (lastStatus & NRF_STATUS_MAX_RT)
		{
			//GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		}
	}
}

void spid_receive_response(uint8_t command, uint8_t *data, uint8_t len)
{
	switch (command)
	{
	case NRF_SPI_W_TX_PAYLOAD:
	{ // Fully transmitted payload, nRF knows to send it off, so disable CE again
		__disable_irq();
		if (!preloadingTX) // Was already triggered, 
			GPIO_RESET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);
		else // Done preloading, ensure trigger is a pulse
		 	preloadingTX = false;
		__enable_irq();
		return;
	}
	case (0x07 | NRF_SPI_WRITE_REG):
	{ // Cleared STATUS after IRQ, now handle according to status
		if (handlingRX)
		{
			if (rxPipe == NRF_SYNC_BROADCAST_PIPE) // Receive sync packet
				spi_read(NRF_SPI_R_RX_PAYLOAD, NRF_SYNC_BROADCAST_LEN);
			else // Read dynamic payload length
				spi_read(NRF_SPI_R_RX_PL_WID, 1);
		}
		return;
	}
	case NRF_SPI_R_RX_PL_WID:
	{ // Read RX payload length after IRQ for RX
		// Continue fetching actual payload
		spi_read(NRF_SPI_R_RX_PAYLOAD, data[0]);
		return;
	}
	case NRF_SPI_R_RX_PAYLOAD:
	{ // Read RX payload, handle
		if (rxPipe == NRF_SYNC_BROADCAST_PIPE)
			nrfd_receive_sync_packet(data, rxTime);
		else if (rxPipe == NRF_CAMERA_SPECIFIC_PIPE)
			nrfd_receive_camera_packet(data, len, rxTime);
		return;
	}
	default:
		return;
	}
}

void nrf_handle_interrupt()
{
	// Implicitly read and explicitly clear STATUS
	uint8_t STATUS = NRF_STATUS_CLEAR_MASK;
	spi_send(0x07 | NRF_SPI_WRITE_REG, &STATUS, 1, true);
}

#endif // USE_SPI_NRF_SYNC