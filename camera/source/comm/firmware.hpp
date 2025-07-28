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

#ifndef FIRMWARE_H
#define FIRMWARE_H

#include "comm/packet.hpp"
#include "util/util.hpp"

#include <cstdint>
#include <vector>
#include <map>

struct TrackingCameraState;
struct CommState;

struct FirmwareTransferState
{
	uint8_t index;
	uint8_t sha256[32];
	uint8_t signMethod;
	std::vector<uint8_t> sign;

	FirmwareTransferType type;
	struct {
		std::string path;
	} file;
	struct {
		VersionDesc since;
		std::string deps;
	} package;

	std::vector<uint8_t> data;
	std::vector<bool> blockMap;
    bool completeAndValid;
};

struct FirmwareUpdateState
{
	int ID;
	FirmwareUpdateFlags flags;
	uint16_t blockSize;
    TimePoint_t startTime;
	TimePoint_t lastActivity;
	std::map<int,FirmwareTransferState> transfers;

	CommState *mainComm;
	bool applyingUpdate;
	TimePoint_t applyTime;
	bool appliedUpdate;
	bool abortedUpdate;

	operator bool()
	{
		if (ID == 0) return false;
		if (applyingUpdate && dtMS(lastActivity, sclock::now()) < 10000) return true;
		if (dtMS(lastActivity, sclock::now()) < 3000) return true;
		ID = 0;
		return false;
	}
};

bool SetupFirmwareUpdate(TrackingCameraState &state, CommState &comm, uint8_t *data, uint16_t length);
bool ReceiveFirmwareBlock(TrackingCameraState &state, CommState &comm, uint8_t *data, uint16_t length);
bool ReceiveFirmwareApplyRequest(TrackingCameraState &state, CommState &comm, uint8_t *data, uint16_t length);
bool ReceiveFirmwareStatus(TrackingCameraState &state, CommState &comm, uint8_t *data, uint16_t length);

bool ApplyFirmwareUpdate(TrackingCameraState &state);
void PostFirmwareUpdateActions(TrackingCameraState &state);

#endif // FIRMWARE_H