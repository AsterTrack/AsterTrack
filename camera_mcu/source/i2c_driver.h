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

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "comm/commands.h"

#include <stdint.h>
#include <stdbool.h>

// Prepended bytes in addition to the first leading byte
// See comments on MCU_LEADING_BYTES or I2C_USE_CLOCK_STRETCHING
#define I2C_PREPENDED_BYTES			(MCU_LEADING_BYTES > 1? (MCU_LEADING_BYTES-1) : 0)

#define I2C_RECEIVE_BUFFER_LEN      256
#define I2C_TRANSMIT_BUFFER_LEN     256

/* Structures */

// Implemented generally
bool i2cd_handle_command(enum CameraMCUCommand command, uint8_t *data, uint8_t len);
uint8_t i2cd_prepare_response(enum CameraMCUCommand command, uint8_t *data, uint8_t len, uint8_t **responsePtr);

// Implemented specfically
void i2c_driver_init();

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H