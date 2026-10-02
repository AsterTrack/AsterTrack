/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include "comm/packet.h"

#include "../build/version.h"

union VersionDesc firmwareVersion = { { FW_MAJOR, FW_MINOR, FW_PATCH, FW_BUILD } };

// Text descriptors
#if !defined(FIRMWARE_DESCRIPTOR) || !defined(FIRMWARE_COMPATIBILITY)
#error "Script did not prepare version file properly!"
#endif
const char* firmwareDescriptor = FIRMWARE_DESCRIPTOR;
const uint16_t firmwareDescriptorLength = sizeof(FIRMWARE_DESCRIPTOR)-1;
const char* firmwareCompatibility = FIRMWARE_COMPATIBILITY;
const uint16_t firmwareCompatibilityLength = sizeof(FIRMWARE_COMPATIBILITY)-1;