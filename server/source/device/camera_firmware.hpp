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

#ifndef CAMERA_FIRMWARE_H
#define CAMERA_FIRMWARE_H

#include "comm/packet.hpp"

#include "util/util.hpp"
#include "util/synchronised.hpp"

#include <vector>

// Forward-declared opaque structs
struct TrackingCameraState; // device/tracking_camera.hpp
struct FirmwareUpdatePlan; // device/camera_firmware.cpp

struct CameraFirmwareUpdateStatus
{
	int ID;
	TimePoint_t lastActivity;
	std::vector<std::vector<uint8_t>> packets;
	FirmwareStatus code;
	std::string text;
};

struct FirmwareUpdateStatus
{
	FirmwareStatus code;
	std::string text;
	std::stop_source abort;
	bool concluded;
	std::string sbc_fw_desc, mcu_fw_desc;
	std::shared_ptr<FirmwareUpdatePlan> update;
};

typedef std::shared_ptr<Synchronised<CameraFirmwareUpdateStatus>> CameraFirmwareUpdateRef;
typedef std::shared_ptr<Synchronised<FirmwareUpdateStatus>> FirmwareUpdateRef;

FirmwareUpdateRef PrepareFirmwareUpdate(std::string firmwareFile);
bool CamerasUpdateFirmware(std::vector<std::shared_ptr<TrackingCameraState>> &cameras, FirmwareUpdateRef &updateStatus);

bool CameraCheckFirmwareFile(const std::string &firmwareFile, std::string &firmwareDescriptor);

#endif // CAMERA_FIRMWARE_H
