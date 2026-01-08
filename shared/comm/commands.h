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
#define MCU_PROBE_INTERVAL_MS	1000
#define MCU_PING_INTERVAL_MS	100
#define MCU_COMM_TIMEOUT_MS		250

enum CameraMCUCommand
{
	MCU_CMD_NONE = 0,
	MCU_REG_ID = 1,

	MCU_PING = 16,
	MCU_BOOT_FIRST, // Notify Pi is booted and may take time initialising
	MCU_BOOT,		// Notify Pi is booted even if main program fails to connect
	MCU_FETCH_INFO,	// Get Info (ID, serial, harware rev, etc.) stored on MCU
	MCU_GET_HW_STR,	// Get Hardware Descriptor String stored in OTP
	MCU_GET_FW_STR,	// Get Firmware Descriptor String stored in firmware
	MCU_GET_PARTS,	// Get list of Subpart Serial IDs stored in OTP
	MCU_UPDATE_ID,	// Tell MCU to update its ID with the one stored on the Pi

	MCU_COMMANDS = 32,
	MCU_SWITCH_BOOTLOADER
};

// Leading Zeros before the actual packet
// May be inserted in case clock-stretching is not used to give MCU time to process commands
// Currently, clock-stretching is used, as the Pi-Specific clock-stretching-bug does not seem to manifest
#define MCU_LEADING_BYTES		0

// Length of MCU_FETCH_INFO (may increase as long as backwards-compatibility is kept)
#define MCU_INFO_MAX_LENGTH		((2 + 7) * sizeof(uint32_t))
// Non-Zero Separator in strings indicating separate texts
#define MCU_MULTI_TEXT_SEP		'\2' // Unicode Text Start

#endif // COMMANDS_H