
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

#define SPI_BYTE_TRANSFER_TIME_US	4 // For 2MBaud. 1/2 us/Baud * 8 Baud with Baud = Bit

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

static volatile bool irqPending = false;
static volatile bool irqNewRxStatus = false;
static TimePoint irqHandlingTime;
static TimePoint irqTime;
static uint8_t irqHandlingRxPipe;

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

static inline void spid_rx_on_pipe(uint8_t pipe)
{ // Earliest point at which we know a pipe received an RX
	if (pipe == NRF_TRIG_BROADCAST_PIPE)
	{ // Trigger, react as fast as possible
		// TODO: Implement external trigger over nRF Sync
	}
}

static inline void spid_query_rx()
{
	if (irqHandlingRxPipe == NRF_SYNC_BROADCAST_PIPE) // Receive sync packet, fetch directly
		spi_read_int(NRF_SPI_R_RX_PAYLOAD, NRF_SYNC_BROADCAST_LEN);
	else if (irqHandlingRxPipe == NRF_TRIG_BROADCAST_PIPE) // Receive trig packet, fetch directly
		spi_read_int(NRF_SPI_R_RX_PAYLOAD, NRF_TRIG_BROADCAST_LEN);
	else // Read dynamic payload length first
		spi_read_int(NRF_SPI_R_RX_PL_WID, 1);
}

static inline void spid_handle_rx(uint8_t *data, uint8_t len)
{
	if (irqHandlingRxPipe == NRF_SYNC_BROADCAST_PIPE)
		nrfd_receive_sync_packet(data, irqHandlingTime);
	else if (irqHandlingRxPipe == NRF_TRIG_BROADCAST_PIPE)
		nrfd_receive_trig_packet(data, irqHandlingTime);
	else if (irqHandlingRxPipe == NRF_DIRECT_PIPE)
		nrfd_receive_camera_packet(data, len, irqHandlingTime);
}

static inline void spid_handle_irq()
{
	irqPending = false;
	irqHandlingTime = irqTime;
	// Implicitly read and explicitly clear STATUS
	uint8_t STATUS = NRF_STATUS_IRQ_MASK;
	spi_send_int(NRF_STATUS_CLEAR_REG, &STATUS, 1, true);
}

void spid_receive_status(uint8_t command, uint8_t status)
{
	statusTime = GetTimePoint();
	statusLast = status;

	if (irqNewRxStatus)
	{ // Cleared IRQ in response to interrupt, adopt as IRQ to handle
		irqNewRxStatus = false;
		uint8_t rxPipe = statusLast & NRF_STATUS_RX_P_NO_MASK;
		if (rxPipe != NRF_STATUS_RX_EMPTY)
			spid_rx_on_pipe(rxPipe >> NRF_STATUS_RX_P_NO_POS);		
	}
}

void spid_receive_response(uint8_t command, uint8_t *data, uint8_t len)
{
	switch (command)
	{
	case NRF_STATUS_CLEAR_REG:
	{ // Read or cleared STATUS after IRQ, now handle
		if (statusLast & NRF_STATUS_TX_DS)
		{ // Sent last packet, don't know which one
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		}
		if (statusLast & NRF_STATUS_MAX_RT)
		{ // Max Retries reached, failed to send last packet. Already cleared IRQ.
			//GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		}
		if ((statusLast & NRF_STATUS_RX_DR) || (statusLast & NRF_STATUS_RX_P_NO_MASK) != NRF_STATUS_RX_EMPTY)
		{ // Received at least one packet, even if not an IRX - see NOTE: IRQ for multiple RX FIFO
			irqHandlingRxPipe = (statusLast & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_POS;
			spid_query_rx();
		}
		else
		{
			// NOTE: Missed IRQ during NRF_STATUS_CLEAR_REG
			// NRF_STATUS_CLEAR_REG may loose IRQs if they happen between status read and status clear
			// Time window is the two byte transfer length, so about 8us at 2Mbits SPI, or 2us at 8Mbits SPI
			// We can only hope to recover a RX_DR IRQ, since that shows up in RX_P_NO - see NOTE: IRQ for multiple RX FIFO
			// So once more update status with NOP and check RX_P_NO
			if (!irqPending)
				spi_read_int(NRF_SPI_NOP, 0);
			irqNewRxStatus = true; // Either way, if there is an RX in that status, it is new
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
		
		// NOTE: IRQ for multiple RX FIFO
		// If an RX packet was received right after another, before that first RX_DR was cleared, it may not get it's own RX_DR IRQ
		// So after handling any RX IRQ and reading it's RX FIFO entry, the fifo has to be checked again
		// Documentation recommends reading FIFO_STATUS, but a NOP reading status is faster AND has more information (RX_P_NO)
		// If there is not already another transfer queued, separately update status with NOP and check RX_P_NO
		if (!irqPending && !spi_is_sending())
			spi_read_int(NRF_SPI_NOP, 0);
		irqNewRxStatus = true; // Either way, if there is an RX in that status, it is new
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
	case NRF_SPI_NOP:
		// See NOTE: Missed IRQ during NRF_STATUS_CLEAR_REG
		// See NOTE: IRQ for multiple RX FIFO
		return;
	default:
		return;
	}
}

void spid_transfers_idle()
{
	// Done with transfor (or IRQ handling), check if there is a (new) IRQ pending
	// Last command should have been NRF_SPI_NOP or NRF_SPI_W_TX_PAYLOAD
	if (irqPending)
	{ // Received new IRQ while SPI was busy, handle it now
		spid_handle_irq();
	}
	else if ((statusLast & NRF_STATUS_RX_P_NO_MASK) != NRF_STATUS_RX_EMPTY)
	{ // No pending IRQ, but still RX in FIFO, handle directly
		// See NOTE: Missed IRQ during NRF_STATUS_CLEAR_REG
		// See NOTE: IRQ for multiple RX FIFO
		irqHandlingTime = statusTime - SPI_BYTE_TRANSFER_TIME_US*TICKS_PER_US;
		irqHandlingRxPipe = (statusLast & NRF_STATUS_RX_P_NO_MASK) >> NRF_STATUS_RX_P_NO_POS;
		spid_query_rx();
	}
}

void nrf_handle_interrupt()
{
	irqTime = GetTimePoint();
	irqPending = true;
	irqNewRxStatus = true;
	if (spi_lock())
	{ // No other SPI transfer ongoing, can handle now
		spid_handle_irq();
	}
	// Else handle in spid_complete_transfer later
}

#endif // USE_SPI_NRF_SYNC