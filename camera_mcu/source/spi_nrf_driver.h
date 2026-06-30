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

void spid_receive_status(uint8_t command, uint8_t status);
void spid_receive_response(uint8_t command, uint8_t *data, uint8_t len);

void spi_driver_init();
bool spi_send(uint8_t command, uint8_t *data, uint8_t len, bool callback);
bool spi_write(uint8_t command, uint8_t *data, uint8_t len);
bool spi_read(uint8_t command, uint8_t len);

#endif

#ifdef __cplusplus
}
#endif

#endif // SPI_NRF_DRIVER_H