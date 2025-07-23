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

#ifndef COMMANDS_H
#define COMMANDS_H

/**
 * This file describes the I2C commands between camera_pi and camera_mcu
*/

#include <stdint.h>

#define MCU_I2C_ADDRESS			0x3E	// Max 7Bit (0x7F)
#define MCU_I2C_ID				0x23
#define MCU_PING_INTERVAL_MS	100
#define MCU_COMM_TIMEOUT_MS		250

enum CameraMCUCommand
{
	MCU_CMD_NONE = 0,
	MCU_REG_ID = 1,

	MCU_PING = 16,
	MCU_BOOT_FIRST, // Notify Pi is booted and may take time initialising
	MCU_BOOT,		// Notify Pi is booted even if main program fails to connect

	MCU_COMMANDS = 32,
	MCU_SWITCH_BOOTLOADER
};

#endif // COMMANDS_H