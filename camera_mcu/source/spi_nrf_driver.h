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

#ifndef SPI_NRF_DRIVER_H
#define SPI_NRF_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef USE_SPI_NRF_SYNC

#define NRF_SPI_MAX_MESSAGE_LENGTH 32 // Excluding first command/status byte

#define NRF_SPI_WRITE_REG			0x20
#define NRF_SPI_ACTIVATE			0x50
#define NRF_SPI_R_RX_PAYLOAD		0x61
#define NRF_SPI_W_TX_PAYLOAD		0xA0
#define NRF_SPI_FLUSH_TX			0xE1
#define NRF_SPI_FLUSH_RX			0xE2
#define NRF_SPI_R_RX_PL_WID			0x60
#define NRF_SPI_NOP					0xFF

// Selected registers
#define NRF_STATUS_REG				0x07
#define NRF_STATUS_CLEAR_REG		(NRF_STATUS_REG | NRF_SPI_WRITE_REG)
#define NRF_FIFO_STATUS_REG			0x17

#define NRF_STATUS_RX_DR			0b1000000
#define NRF_STATUS_TX_DS			0b0100000
#define NRF_STATUS_MAX_RT			0b0010000
#define NRF_STATUS_RX_EMPTY		    0b0001110
#define NRF_STATUS_RX_P_NO_MASK		0b0001110
#define NRF_STATUS_RX_P_NO_POS		1
#define NRF_STATUS_TX_FULL			0b0000001
#define NRF_STATUS_IRQ_MASK			0b1110000

#define NRF_FIFO_STATUS_TX_REUSE	0b1000000
#define NRF_FIFO_STATUS_TX_FULL		0b0100000
#define NRF_FIFO_STATUS_TX_EMPTY	0x0010000
#define NRF_FIFO_STATUS_RX_FULL		0x0000010
#define NRF_FIFO_STATUS_RX_EMPTY	0x0000001

void spid_receive_status(uint8_t command, uint8_t status);
void spid_receive_response(uint8_t command, uint8_t *data, uint8_t len);
void spid_transfers_idle();

void spi_driver_init();

// Transfer functions, non-blocking. Only use _int version from SPI interrupts!
void spi_send_int(uint8_t command, uint8_t *data, uint8_t len, bool callback);
bool spi_send(uint8_t command, uint8_t *data, uint8_t len, bool callback);
void spi_write_int(uint8_t command, uint8_t *data, uint8_t len);
bool spi_write(uint8_t command, uint8_t *data, uint8_t len);
void spi_read_int(uint8_t command, uint8_t len);
bool spi_read(uint8_t command, uint8_t len);

/* Synchronisation functions, blocking. DO NOT CALL IN INTERRUPTS! */
// Try to lock for a single SPI transfer. If successful, returns true, and will be automatically unlocked at end of transfer.
bool spi_lock();
// Begin exclusive SPI access region, blocking.
void spi_begin();
// End exclusive SPI access region, blocking. If any SPI transfers are still ongoing, they are still protected.
void spi_end();
// Wait for any ongoing SPI transfers to finish, blocking.
void spi_sync();
// Returns if a SPI transfer is currently ongoing.
bool spi_is_sending();

// Transfer functions, blocking. Will busy-wait, still uses interrupts and callbacks. DO NOT CALL IN INTERRUPTS!
void spi_write_sync(uint8_t command, uint8_t *data, uint8_t len);
void spi_read_sync(uint8_t command, uint8_t len);

#endif

#ifdef __cplusplus
}
#endif

#endif // SPI_NRF_DRIVER_H