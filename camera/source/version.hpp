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

#ifndef VERSION_H
#define VERSION_H

#include "comm/packet.h"
#include "mcu/mcu.hpp"

#include <string>

extern uint32_t cameraID;
extern std::string id_file;

extern const VersionDesc sbcFWVersion;
extern const std::string sbcFWDescriptor;

extern CameraStoredInfo storedInfo;

extern int numCPUCores;

// Get Camera ID, Firmware and Hardware Versions/Info, etc.
// Both from config files, system drivers
void gatherInfo(CameraID overrideID = 0);

void receivedInfoFromMCU(CameraStoredInfo &&info);

/* Returns whether MCU should be instructed to update its own stored ID */
bool receivedConfigFromMCU(CameraStoredConfig &&config);

void sendInfoPacket(CommMedium comm);

#endif // VERSION_H
