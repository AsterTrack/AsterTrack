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

#include "comm/packet.hpp"
#include "mcu/serial.hpp"

#include <string>
#include <mutex>
#include <atomic>

extern std::string mcu_firmware_path;
extern FirmwareTagHeader mcu_firmware_tag;

extern std::mutex mcu_mutex;

extern std::atomic<uint16_t> floatingSupplyVoltageMV;

extern std::atomic<bool> mcu_exists;
extern std::atomic<bool> mcu_active;


bool mcu_initial_connect();
bool mcu_init();
bool mcu_probe();
bool mcu_reconnect();
void mcu_monitor();
void mcu_cleanup();
void mcu_reset();
bool mcu_probe_bootloader();
bool mcu_switch_bootloader();
bool mcu_flash_program(const std::string &filename);
bool mcu_verify_program(const std::string &filename);
bool mcu_read_firmware_tag(const std::string &filename, FirmwareTagHeader &tag);
void mcu_sync_info();
bool mcu_fetch_info(CameraStoredInfo &info, CameraStoredConfig &config);
bool mcu_update_id(CameraID cameraID);
bool mcu_get_status();

#endif // MCU_H