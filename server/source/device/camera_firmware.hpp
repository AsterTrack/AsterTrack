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

#include "util/util.hpp"
#include "util/synchronised.hpp"

#include <vector>

// Forward-declared opaque structs
enum FirmwareStatus : uint8_t; // comm/packet.hpp
struct TrackingCameraState; // device/tracking_camera.hpp

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
};

typedef std::shared_ptr<Synchronised<CameraFirmwareUpdateStatus>> CameraFirmwareUpdateRef;
typedef std::shared_ptr<Synchronised<FirmwareUpdateStatus>> FirmwareUpdateRef;

FirmwareUpdateRef CamerasFlashFirmwareFile(std::vector<std::shared_ptr<TrackingCameraState>> &cameras, std::string firmware);

#endif // CAMERA_FIRMWARE_H
