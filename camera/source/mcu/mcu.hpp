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

#ifndef MCU_H
#define MCU_H

#include "comm/packet.h"
#include "mcu/serial.hpp"

#include <string>
#include <mutex>
#include <vector>

extern std::string mcu_flash_file;

extern std::mutex mcu_mutex;

extern volatile bool mcu_exists;
extern volatile bool mcu_active;

struct MCU_StoredInfo
{
	CameraID cameraID; // Camera ID (32bit), stored in MCU Flash, used for unique ID in software
	VersionDesc mcuFWVersion; // MCU Firmware Version, embedded in firmware
	std::string mcuFWDescriptor; // MCU Firmware Descriptor, embedded in firmware
    uint8_t mcuOTPVersion; // Indicates revision in how OTP is formatted, potentially even affecting the format of the following members
	HardwareSerial hardwareSerial; // Camera Serial Number (64Bit) stored in MCU OTP, bound to hardware & production
    std::vector<uint64_t> subpartSerials; // Optional serial numbers of subparts
	std::string mcuHWDescriptor; // MCU Hardware Descriptor, written to OTP, may be composed of successively written texts separated by MCU_MULTI_TEXT_SEP
	uint32_t mcuUniqueID[3]; // MCU Chip Unique ID, 96Bit for STM32
};

bool mcu_initial_connect();
bool mcu_init();
bool mcu_probe();
bool mcu_reconnect();
void mcu_monitor();
void mcu_cleanup();
void mcu_reset();
bool mcu_probe_bootloader();
bool mcu_switch_bootloader();
bool mcu_flash_program(std::string filename);
bool mcu_verify_program(std::string filename);
void mcu_sync_info();
bool mcu_fetch_info(MCU_StoredInfo &info);
bool mcu_update_id(CameraID cameraID);
bool mcu_get_status();

#endif // MCU_H