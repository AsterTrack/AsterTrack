
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

/* Some fixed and base addresses LSB First */
// Address of sync base / controller of current RF channel
uint8_t ADDR_SYNC_CONTROLLER[] = { 0x2C, 0x7E, 0xA5 };
// Sub-Addresses of cameras (sharing the same 2 MSBs at the end, with variable LSB first)
uint8_t ADDR_BROADCAST_META[] = { 0x25, 0x7E, 0xA3 };
uint8_t ADDR_BROADCAST_SYNC[] = { 0x25, 0x7E, 0xA5 };
uint8_t ADDR_BROADCAST_TRIG[] = { 0x25, 0x7E, 0xA7 }; // TODO: Use number of slots beyond that for trigger channels?

bool resetToOwnRxAddress;
uint8_t ownRxAddress[3];

uint8_t Base_DYNPD;
uint8_t Base_EN_AA;
bool DP0_ESB_RX, DP0_ESB;

bool resetToOwnRxAddress;
uint8_t ownRxAddress[3];

static uint8_t statusLast;
static TimePoint statusTime;

static volatile uint8_t irqHandling = false;
static volatile bool irqPending = false;
static TimePoint irqHandlingTime;
static TimePoint irqTime;
static uint8_t rxPipe;

static volatile bool lastSentSync = false;
static volatile bool pendingTX = false;
static volatile bool preloadingTX = false;

void nrf_setup_camera(uint8_t cameraAddress[3])
{
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

	// Select desired feature flags
	uint8_t FEATURE = 0b100;
	spi_write_sync(0x1D | NRF_SPI_WRITE_REG, &FEATURE, 1);

	// Request to apply selected features
	uint8_t ACTIVATE = 0x73;
	spi_write_sync(NRF_SPI_ACTIVATE, &ACTIVATE, 1);

	static_assert(NRF_DIRECT_PIPE == 0);
	static_assert(NRF_META_BROADCAST_PIPE == 1);
	static_assert(NRF_SYNC_BROADCAST_PIPE == 2);
	static_assert(NRF_TRIG_BROADCAST_PIPE == 3);

	// Unique RX address in data pipe 0, so will have to be after every TX with Auto-ACk (just do in RX start)
	resetToOwnRxAddress = true;
	memcpy(ownRxAddress, cameraAddress, 3);

	// Setup camera sub-addresses data pipes
	uint8_t *RX_ADDR_P1 = ADDR_BROADCAST_META;
	spi_write_sync(0x0B | NRF_SPI_WRITE_REG, RX_ADDR_P1, 3);
	uint8_t *RX_ADDR_P2 = ADDR_BROADCAST_SYNC;
	spi_write_sync(0x0C | NRF_SPI_WRITE_REG, RX_ADDR_P2, 3);
	uint8_t *RX_ADDR_P3 = ADDR_BROADCAST_TRIG;
	spi_write_sync(0x0D | NRF_SPI_WRITE_REG, RX_ADDR_P3, 3);

	// Setup data pipes with fixed payload sizes
	uint8_t RX_PW_P2 = NRF_SYNC_BROADCAST_LEN;
	spi_write_sync(0x13 | NRF_SPI_WRITE_REG, &RX_PW_P2, 1);
	uint8_t RX_PW_P3 = NRF_TRIG_BROADCAST_LEN;
	spi_write_sync(0x14 | NRF_SPI_WRITE_REG, &RX_PW_P3, 1);

	// Setup data pipes with dynamic payload sizes
	Base_DYNPD = 0b000011;
	spi_write_sync(0x1C | NRF_SPI_WRITE_REG, &Base_DYNPD, 1);

	// Setup data pipes with automatic acknowledgement
	Base_EN_AA = 0b110011;
	spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &Base_EN_AA, 1);

	// Data Pipe 0 might get switched between basic ShockBurst and ESB, so keep track of state
	DP0_ESB_RX = DP0_ESB = (Base_DYNPD&1) && (Base_EN_AA&1);

	// Enable desired data pipes
	uint8_t EN_RXADDR = 0b001111;
	spi_write_sync(0x02 | NRF_SPI_WRITE_REG, &EN_RXADDR, 1);
}

void nrf_setup_sync_base()
{
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

	// Select desired feature flags
	uint8_t FEATURE = 0b100;
	spi_write_sync(0x1D | NRF_SPI_WRITE_REG, &FEATURE, 1);

	// Request to apply selected features
	uint8_t ACTIVATE = 0x73;
	spi_write_sync(NRF_SPI_ACTIVATE, &ACTIVATE, 1);

	// Data pipe 0 is used to receive any ACKs only
	// RX Address is configured as TX address when ACK is expected
	// DYNPD is kept off for now, for ACKs without payload
	// But ALSO need to disable Auto-ACK on pipe 0 if sending legacy ShockBurst packets!

	// Setup static sync base / controller address on data pipe 1
	uint8_t *RX_ADDR_P1 = ADDR_SYNC_CONTROLLER;
	spi_write_sync(0x0B | NRF_SPI_WRITE_REG, RX_ADDR_P1, 3);

	// Setup data pipes with dynamic payload sizes
	Base_DYNPD = 0b000010;
	spi_write_sync(0x1C | NRF_SPI_WRITE_REG, &Base_DYNPD, 1);

	// Setup data pipes with automatic acknowledgement
	Base_EN_AA = 0b111110;
	spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &Base_EN_AA, 1);

	// Data Pipe 0 might get switched between basic ShockBurst and ESB, so keep track of state
	DP0_ESB_RX = DP0_ESB = (Base_DYNPD&1) && (Base_EN_AA&1);

	// Enable desired data pipes
	uint8_t EN_RXADDR = 0b000011;
	spi_write_sync(0x02 | NRF_SPI_WRITE_REG, &EN_RXADDR, 1);
}

void nrf_rx_powerup()
{
	if (resetToOwnRxAddress)
	{ // Setup unique RX address for data pipe 0 (LSB first)
		uint8_t *RX_ADDR_P0 = ownRxAddress;
		spi_write_sync(0x0A | NRF_SPI_WRITE_REG, RX_ADDR_P0, 3);
	}

	if (DP0_ESB != DP0_ESB_RX)
	{ // Data Pipe was switched between Enhanced and normal ShockBurst, return to default
		DP0_ESB = DP0_ESB_RX;
		spi_write_sync(0x1C | NRF_SPI_WRITE_REG, &Base_DYNPD, 1);
		spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &Base_EN_AA, 1);
	}

	// Write config to start up and enter RX mode
	uint8_t CONFIG = 0b00001011;
	spi_write_sync(0x00 | NRF_SPI_WRITE_REG, &CONFIG, 1);

	// Enable RF (Receive)
	GPIO_SET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);
}

void nrf_tx_powerup()
{
	// Write config to start up in TX mode
	uint8_t CONFIG = 0b00001010;
	spi_write_sync(0x00 | NRF_SPI_WRITE_REG, &CONFIG, 1);
}

bool nrf_tx_general(uint8_t address[3], uint8_t *data, uint8_t length)
{
	// Check for pending TX that's still awaiting a trigger
	if (pendingTX)
	{ // Easiest to disallow, otherwise preloaded trigger would need to also handle trigger for this packet
		BREAK();
		return false;
	}

	// Setup target address for TX
	uint8_t *TX_ADDR = address;
	spi_write_sync(0x10 | NRF_SPI_WRITE_REG, TX_ADDR, 3);

	// Setup target address as RX Address for data pipe 0 for ACKs
	uint8_t *RX_ADDR_P0 = address;
	spi_write_sync(0x0A | NRF_SPI_WRITE_REG, RX_ADDR_P0, 3);

	if (!DP0_ESB)
	{ // Switch data pipe 0 to ESB for general transfer
		DP0_ESB = true;
		uint8_t DYNPD = Base_DYNPD | 1;
		uint8_t EN_AA = Base_EN_AA | 1;
		spi_write_sync(0x1C | NRF_SPI_WRITE_REG, &DYNPD, 1);
		spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &EN_AA, 1);
	}

	// Write payload
	spi_write_sync(NRF_SPI_W_TX_PAYLOAD, data, length);

	// Trigger with minimum pulse width of 10us, and transmit after payload is transferred
	nrf_tx_trigger();

	lastSentSync = false;
	return true;
}

bool nrf_tx_prepare_broadcast_sync(uint8_t data[NRF_SYNC_BROADCAST_LEN])
{
	// Check for pending TX that's still awaiting a trigger
	if (pendingTX)
	{ // Easiest to disallow, otherwise preloaded trigger would need to also handle trigger for this packet
		BREAK();
		return false;
	}
	pendingTX = true;

	if (!lastSentSync)
	{
		// Setup camera address for TX
		uint8_t *TX_ADDR = ADDR_BROADCAST_SYNC;
		spi_write_sync(0x10 | NRF_SPI_WRITE_REG, TX_ADDR, 3);

		// Disable Auto-Acknowledgement for pipe 0 for transmitting
		uint8_t EN_AA = 0b111110;
		spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &EN_AA, 1);

		if (DP0_ESB)
		{ // Switch data pipe 0 to normal ShockBurst for sync broadcast
			DP0_ESB = false;
			uint8_t DYNPD = Base_DYNPD & ~1;
			uint8_t EN_AA = Base_EN_AA & ~1;
			spi_write_sync(0x1C | NRF_SPI_WRITE_REG, &DYNPD, 1);
			spi_write_sync(0x01 | NRF_SPI_WRITE_REG, &EN_AA, 1);
		}

		lastSentSync = true;
	}

	// Write payload
	preloadingTX = true;
	spi_write_sync(NRF_SPI_W_TX_PAYLOAD, data, NRF_SYNC_BROADCAST_LEN);

	// We COULD just return here immediately without syncing
	// But docs say CE should start after TX transfer, so this serves as an easy way to ensure that
}

void nrf_tx_trigger()
{
	// Enable RF, will transmit as soon as payload is transferred
	GPIO_SET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);

	// Wait minimum pulse width
	delayUS(10);

	// Handle end of CE pulse, here or once preloading is done
	__disable_irq();
	if (!preloadingTX) // Already preloaded, so keep CE pulse at minimum
		GPIO_RESET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);
	else // Still preloading, keep CE high, CE is cleared once payload is loaded and transmitting
	 	preloadingTX = false;
	pendingTX = false;
	__enable_irq();
}


/* ------ SPI NRF Behaviour ------ */

void spid_receive_status(uint8_t command, uint8_t status)
{
	statusTime = GetTimePoint();
	statusLast = status;
	if (command == NRF_STATUS_CLEAR_REG && (statusLast & NRF_STATUS_IRQ_MASK))
	{ // Cleared IRQ in response to interrupt, adopt as IRQ to handle
		rxPipe = (statusLast & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_POS;
		if (rxPipe == NRF_TRIG_BROADCAST_PIPE && (statusLast & NRF_STATUS_RX_DR))
		{ // Trigger, react as fast as possible
			// TODO: Implement external trigger over nRF Sync
		}
	}
}

static inline void spid_query_rx()
{
	if (rxPipe == NRF_SYNC_BROADCAST_PIPE) // Receive sync packet, fetch directly
		spi_read_int(NRF_SPI_R_RX_PAYLOAD, NRF_SYNC_BROADCAST_LEN);
	else if (rxPipe == NRF_TRIG_BROADCAST_PIPE) // Receive trig packet, fetch directly
		spi_read_int(NRF_SPI_R_RX_PAYLOAD, NRF_TRIG_BROADCAST_LEN);
	else // Read dynamic payload length first
		spi_read_int(NRF_SPI_R_RX_PL_WID, 1);
}

static inline void spid_handle_rx(uint8_t *data, uint8_t len)
{
	if (rxPipe == NRF_SYNC_BROADCAST_PIPE)
		nrfd_receive_sync_packet(data, irqHandlingTime);
	else if (rxPipe == NRF_TRIG_BROADCAST_PIPE)
		nrfd_receive_trig_packet(data, irqHandlingTime);
	else if (rxPipe == NRF_DIRECT_PIPE)
		nrfd_receive_camera_packet(data, len, irqHandlingTime);
}

static inline void spid_handle_irq(TimePoint time)
{
	irqPending = false;
	irqHandling = statusLast;
	irqHandlingTime = time;
	// Implicitly read and explicitly clear STATUS
	uint8_t STATUS = NRF_STATUS_IRQ_MASK;
	spi_send_int(NRF_STATUS_CLEAR_REG, &STATUS, 1, true);
}

static inline bool nrfd_check_pending_irq()
{
	if (irqPending)
	{ // Received IRQ but SPI was busy with this transfer, handle IRQ now
		spid_handle_irq(irqTime);
		return true;
	}
	else if (statusLast & NRF_STATUS_IRQ_MASK)
	{ // Must have missed the IRQ somehow
		BREAK();
		spid_handle_irq(GetTimePoint());
		return true;
	}
	return false;
}

void spid_receive_response(uint8_t command, uint8_t *data, uint8_t len)
{
	switch (command)
	{
	case NRF_STATUS_CLEAR_REG:
	{ // Cleared STATUS after IRQ, now handle
		if (irqHandling & NRF_STATUS_TX_DS)
		{ // Sent last packet, don't know which one
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		}
		if (irqHandling & NRF_STATUS_MAX_RT)
		{ // Max Retries reached, failed to send last packet. Already cleared IRQ.
			//GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		}
		if (irqHandling & NRF_STATUS_RX_DR)
		{ // Received at least one packet
			spid_query_rx();
		}
		else
		{ // Done handling this IRQ, but may have missed an RX IRQ
			// Time window is tiny, from receiving status to clearing status in a two-byte transfer
			// So about 8us at 2Mbits SPI, or 2us at 8Mbits SPI
			// But this IRQ may also have been triggered during RX Handling, where that previous IRQ had two RX packets
			// Then there'd be a RX FIFO entry waiting for quite a while already, so just double check
			spi_read_int(NRF_FIFO_STATUS_REG, 1);
		}
		return;
	}
	case NRF_SPI_R_RX_PL_WID:
	{ // Read RX payload length, continue fetching actual payload
		spi_read_int(NRF_SPI_R_RX_PAYLOAD, data[0]);
		return;
	}
	case NRF_SPI_R_RX_PAYLOAD:
	{ // Read RX payload, handle
		spid_handle_rx(data, len);
		irqHandling = 0; // Done with that IRQ

		// Check if a newer IRQ has been received during RX handling
		// NOTE: This would be done in spid_complete_transfer anyway, but we need to check FIFO_STATUS JUST here
		if (!nrfd_check_pending_irq())
		{ // No pending IRQ, but may have received another packet between IRQ signal to clearing status, so check FIFO
			spi_read_int(NRF_FIFO_STATUS_REG, 1);
		}
		return;
	}
	case NRF_SPI_W_TX_PAYLOAD:
	{ // Fully transmitted payload, nRF knows to send it off, so disable CE again
		__disable_irq();
		if (!preloadingTX) // Was already triggered but left high to wait for preloading, clear here
			GPIO_RESET(NRF_CTRL_GPIO_X, NRF_CTRL_CE_PIN);
		else // Done preloading before trigger should be cleared, notify to make it a short pulse
		 	preloadingTX = false;
		__enable_irq();
		return;
	}
	case NRF_FIFO_STATUS_REG:
	{ // Double checking for any further RX packets after IRQ handling
		if (data[0] & NRF_FIFO_STATUS_RX_EMPTY)
		{ // No further RX waiting, finally done with IRQ handling
			return; // spid_complete_transfer will again check for any new pending IRQs
		}
		if (irqPending || (statusLast & NRF_STATUS_IRQ_MASK))
		{ // Got a new IRQ (RX or not), and a RX in FIFO, but no way to know for sure if it's old or from this IRQ
			statusLast |= NRF_STATUS_RX_DR; // In case IRQ was non-RX, ensure RX is handled anyway
			spid_handle_irq(irqHandlingTime); // Use old time, wrongly using newer is devastating for timesync
		}
		else
		{ // No IRQ pending, so there is another RX waiting in FIFO from last IRQ, handle it directly with known old timestamp
			rxPipe = (statusLast & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_POS;
			spid_query_rx();
		}
		return;
	}
	default:
		return;
	}
}

void spid_transfers_idle()
{
	nrfd_check_pending_irq();
}

void nrf_handle_interrupt()
{
	irqTime = GetTimePoint();
	irqPending = true;
	if (!irqHandling && spi_lock())
	{ // No other SPI transfer ongoing, can handle now
		spid_handle_irq(irqTime);
	}
	// Else handle after current transfers complete
}

#endif // USE_SPI_NRF_SYNC