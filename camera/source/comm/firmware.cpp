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

#include "firmware.hpp"
#include "state.hpp"
#include "comm.hpp"
#include "mcu/mcu.hpp"

#include "hash/sha256.hpp"

#include <fstream>
#include <filesystem>
#include <thread>

static void SendFirmwareStatusPacket(TrackingCameraState &state, CommState &comm, uint8_t type, uint8_t status, uint8_t transfer = 0, uint16_t block = 0)
{
	std::vector<uint8_t> FWPacket(FIRMWARE_PACKET_HEADER);
	*(uint16_t*)(FWPacket.data()+0) = state.firmware.ID;
	// 1 free byte for future use
	*(uint8_t*)(FWPacket.data()+3) = type;
	*(uint8_t*)(FWPacket.data()+4) = status;
	*(uint8_t*)(FWPacket.data()+5) = transfer;
	*(uint16_t*)(FWPacket.data()+6) = block;

	comm_send(&comm, PacketHeader(PACKET_FW_STATUS, FWPacket.size()), std::move(FWPacket));
}

static void SendUpdateStatus(TrackingCameraState &state, CommState &comm, FirmwareStatus status)
{
	SendFirmwareStatusPacket(state, comm, (uint8_t)FW_STATUS_UPDATE, (uint8_t)status);
}
static void SendTransferStatus(TrackingCameraState &state, CommState &comm, FirmwareTXStatus status, uint8_t transfer)
{
	SendFirmwareStatusPacket(state, comm, (uint8_t)FW_STATUS_TRANSFER, (uint8_t)status, transfer);
}
static void SendBlockStatus(TrackingCameraState &state, CommState &comm, FirmwareTXStatus status, uint8_t transfer, uint16_t block)
{
	SendFirmwareStatusPacket(state, comm, (uint8_t)FW_STATUS_BLOCK, (uint8_t)status, transfer, block);
}

static bool validateFirmwarePacket(TrackingCameraState &state, const uint8_t *data, uint16_t length)
{
	if (length < FIRMWARE_PACKET_HEADER)
	{
		printf("Invalid Firmware status packet length of %d!\n", length);
		return false;
	}
	if (state.curMode.streaming)
	{
		printf("Received Firmware status packet while streaming!\n");
		return false;
	}
	uint16_t ID = *(uint16_t*)(data+0);
	if (ID == 0 || (state.firmware && state.firmware.ID != ID))
	{
		printf("Received Firmware Apply packet with invalid firmware ID!\n");
		return false;
	}
	return true;
}

static FirmwareTXStatus CheckFirmwareTransfer(FirmwareTransferState &transfer)
{
	if (transfer.completeAndValid)
		return FW_TX_TRANSFERRED;

	uint32_t transferredSize = 0;
	for (int i = 0; i < transfer.blockMap.size(); i++)
		if (!transfer.blockMap[i])
			return FW_TX_TRANSFERRING;

	uint8_t sha256[SHA256::HashBytes];
	SHA256 sha;
	sha.add(transfer.data.data(), transfer.data.size());
	sha.getHash(sha256);

	for (int i = 0; i < SHA256::HashBytes; i++)
	{
		if (transfer.sha256[i] != sha256[i])
		{
			printf("Firmware Update: Transfer %d has mismatched SHA256!\n", transfer.index);
			return FW_TX_ERROR;
		}
	}
	printf("Firmware Update: Transfer %d finished successfully!\n", transfer.index);
	transfer.completeAndValid = true;
	return FW_TX_TRANSFERRED;
}


static FirmwareStatus CheckFirmwareUpdate(FirmwareUpdateState &firmware)
{
	if (firmware.appliedUpdate)
		return FW_STATUS_UPDATED;

	if (firmware.applyingUpdate)
		return FW_STATUS_UPDATING;

	for (int i = 0; i < firmware.transfers.size(); i++)
	{
		if (!firmware.transfers[i].completeAndValid)
			return FW_STATUS_TRANSFERRING;
	}

	return FW_STATUS_TRANSFERRED;
}

bool SetupFirmwareUpdate(TrackingCameraState &state, CommState &comm, const uint8_t *data, uint16_t length)
{
	if (!validateFirmwarePacket(state, data, length)) return false;

	uint16_t ID = *(uint16_t*)(data+0);
	uint8_t flags = *(uint8_t*)(data+2);
	uint8_t transferCount = *(uint8_t*)(data+3);
	uint16_t blockSize = *(uint16_t*)(data+4);
	uint8_t signMethod = *(uint8_t*)(data+6);
	uint8_t signSize = *(uint8_t*)(data+7);
	// 2 free bytes for future use
	data += FIRMWARE_PACKET_HEADER;

	printf("Received Firmware Update Request %d with %d transfers!\n", ID, transferCount);
	if (signMethod > 0 && signSize > 0)
		printf("    Package is signed with method %d, sign size %d!\n", signMethod, signSize);
	state.firmware = {};
	state.firmware.ID = ID;
	state.firmware.flags = (FirmwareUpdateFlags)flags;
	state.firmware.blockSize = blockSize;
	state.firmware.startTime = sclock::now();
	state.firmware.lastActivity = sclock::now();
	state.firmware.transfers.clear();
	state.firmware.mainComm = &comm;

	for (int i = 0; i < transferCount; i++)
	{
		uint8_t index = *(uint8_t*)(data+0);
		uint8_t type = *(uint8_t*)(data+1);
		uint32_t size = *(uint32_t*)(data+2);
		data += FIRMWARE_FILE_HEADER;
		if (size > FIRMWARE_MAX_TRANSFER_SIZE)
		{
			printf("    Transfer %d of type %d, size %dMB is over size limit!\n", index, type, size/1000000);
			return false;
		}
		printf("    Announced transfer %d of type %d with size %.2fMB!\n", index, type, size/1000000.0f);

		auto &transfer = state.firmware.transfers[index];
		transfer.type = (FirmwareTransferType)type;
		transfer.index = index;
		transfer.data.resize(size);
		transfer.blockMap.resize((size+blockSize-1)/blockSize);

		memcpy(transfer.sha256, data, SHA256::HashBytes);
		data += SHA256::HashBytes;

		transfer.signMethod = signMethod;
		transfer.sign.resize(signSize);
		if (signSize) memcpy(transfer.sign.data(), data, signSize);
		data += signSize;

		if (transfer.type == FW_TX_TYPE_FILE)
		{
			uint16_t pathSize = *(uint16_t*)data;
			data += 2;
			if (pathSize > 0)
				transfer.file.path = std::string((char*)data, pathSize);
		}
		else if (transfer.type == FW_TX_TYPE_PACKAGE)
		{
			uint16_t depsSize = *(uint16_t*)data;
			data += 2;
			if (depsSize > 0)
				transfer.package.deps = std::string((char*)data, depsSize);
		}
	}
	SendUpdateStatus(state, comm, FW_STATUS_INITIATED);
	return true;
}

bool ReceiveFirmwareBlock(TrackingCameraState &state, CommState &comm, const uint8_t *data, uint16_t length)
{
	if (!validateFirmwarePacket(state, data, length)) return false;
	if (state.firmware.mainComm != &comm) return false;

	uint16_t ID = *(uint16_t*)(data+0);
	// 1 free byte for future use
	uint8_t index = *(uint8_t*)(data+3);
	uint16_t block = *(uint16_t*)(data+4);
	uint16_t size = *(uint16_t*)(data+6);
	// 2 free bytes for future use

	if (FIRMWARE_PACKET_HEADER+size != length)
	{
		printf("Received firmware block with invalid size!\n");
		SendBlockStatus(state, comm, FW_TX_ERROR, index, block);
		return false;
	}
	if (state.firmware.applyingUpdate)
	{
		printf("Received firmware block while applying update - ignoring!\n");
		return false;
	}

	auto transferIt = state.firmware.transfers.find(index);
	if (transferIt == state.firmware.transfers.end())
	{
		printf("Received firmware block for uninitialised transfer!\n");
		SendBlockStatus(state, comm, FW_TX_ERROR, index, block);
		return false;
	}
	auto &transfer = transferIt->second;
	state.firmware.lastActivity = sclock::now();

	if (size != state.firmware.blockSize && block != transfer.blockMap.size()-1)
	{
		printf("Received firmware block with size %d != blockSize before transfer end in packet of length %d!\n", size, length);
		SendBlockStatus(state, comm, FW_TX_ERROR, index, block);
		return false;
	}

	uint32_t offset = block*state.firmware.blockSize;
	memcpy(transfer.data.data()+offset, data+FIRMWARE_PACKET_HEADER, size);
	transfer.blockMap[block] = true;
	printf("Successfully received firmware block %d (offset %d, size %d) for transfer %d in packet of length %d!\n", block, offset, size, index, length);
	SendBlockStatus(state, comm, FW_TX_TRANSFERRED, index, block);

	FirmwareTXStatus status = CheckFirmwareTransfer(transfer);
	if (status == FW_TX_TRANSFERRED)
	{ // Transfer complete and has valid SHA256
		SendTransferStatus(state, comm, FW_TX_TRANSFERRED, index);

		FirmwareStatus status = CheckFirmwareUpdate(state.firmware);
		if (status != FW_STATUS_TRANSFERRING)
			SendUpdateStatus(state, comm, status);
	}
	else if (status == FW_TX_ERROR)
	{ // Thought we were complete (with CRC32) but SHA256 was not valid
		SendTransferStatus(state, comm, FW_TX_ERROR, index);
	}
	return true;
}

bool ReceiveFirmwareApplyRequest(TrackingCameraState &state, CommState &comm, const uint8_t *data, uint16_t length)
{
	if (!validateFirmwarePacket(state, data, length)) return false;
	if (state.firmware.mainComm != &comm) return false;

	uint16_t ID = *(uint16_t*)(data+0);
	// 8 free bytes for future use

	for (auto &transfer : state.firmware.transfers)
	{
		if (transfer.second.completeAndValid) continue;
		SendUpdateStatus(state, comm, FW_STATUS_FAILEDTRANSFER);
		return false;
	}
	printf("Received all transfers - requested to apply firmware update...\n");

	// This will signal main thread to apply to ensure it doesn't do anything that might disturb this
	// In the meantime, no firmware comms may edit anything while applyingUpdate
	state.firmware.applyingUpdate = true;
	state.firmware.appliedUpdate = false;
	state.firmware.applyTime = sclock::now();
	return true;
}

bool ApplyFirmwareUpdate(TrackingCameraState &state)
{
	if (!state.firmware.mainComm) return false;
	CommState &comm = *state.firmware.mainComm;

	float applyDelayMS = dtMS(state.firmware.applyTime, sclock::now());
	if (applyDelayMS > 100)
	{ // Maybe main thread was not in a state to receive a firmware update, in which case, glad we didn't do it
		printf("Main thread initiated applying firmware update only after %.2fms!\n", applyDelayMS);
		SendUpdateStatus(state, comm, FW_STATUS_ISSUE);
		return false;
	}
	if (!state.firmware.applyingUpdate)
		return false;

	SendUpdateStatus(state, comm, FW_STATUS_UPDATING);
	printf("Applying firmware update...\n");

	// Should be protected by applyingUpdate, but copy anyway to be safe
	auto transfers = state.firmware.transfers;

	struct FileSwap
	{
		std::filesystem::path target;
		std::filesystem::path updated;
		std::filesystem::path backup;
	};
	std::vector<FileSwap> files;

	bool aborted = false;
	for (auto &t : transfers)
	{
		auto &transfer = t.second;
		FileSwap swap = {};
		if (transfer.type == FW_TX_TYPE_FILE)
		{
			swap.target = transfer.file.path;
			swap.updated = transfer.file.path + ".updated";
			swap.backup = transfer.file.path + ".backup";
		}
		if (!swap.updated.empty())
		{
			std::ofstream out(swap.updated, std::ios::binary);
			if (!out.is_open())
			{
				aborted = true;
				break;
			}
			// TODO: Check for disk space before continuing to write
			files.push_back(std::move(swap));
			out.write((const char*)transfer.data.data(), transfer.data.size());
			out.close();
			if (out.fail())
			{
				aborted = true;
				break;
			}
		}
	}
	if (aborted || state.firmware.abortedUpdate)
	{
		printf("Applying firmware update was (safely) aborted due to I/O error!\n");
		for (auto file : files)
			std::filesystem::remove(file.updated);
		state.firmware.applyingUpdate = false;
		SendUpdateStatus(state, comm, FW_STATUS_ERROR);
		if (state.firmware.abortedUpdate)
			state.firmware = {};
		return false;
	}

	printf("Written all firmware update files, swapping!\n");
	std::error_code err;
	int f;
	for (f = 0; f < files.size(); f++)
	{
		std::filesystem::rename(files[f].target, files[f].backup, err);
		if (err)
		{ // Try our best to recover...
			printf("Encountered error backing up file %s: %s (%d)\n", files[f].target.c_str(), strerror(err.value()), err.value());
			aborted = true;
			break;
		}
		std::filesystem::rename(files[f].updated, files[f].target, err);
		if (err)
		{ // Try our best to recover...
			printf("Encountered error updating file %s: %s (%d)\n", files[f].target.c_str(), strerror(err.value()), err.value());
			std::filesystem::rename(files[f].backup, files[f].target, err);
			if (err)
				printf("Encountered error restoring file %s: %s (%d)\n", files[f].target.c_str(), strerror(err.value()), err.value());
			aborted = true;
			break;
		}
	}
	if (aborted)
	{ // Failure during renaming - highly unlikely, but try to recover
		for (int b = 0; b < f; b++)
		{
			std::filesystem::rename(files[b].backup, files[b].target, err);
			if (err)
				printf("Encountered error restoring file %s: %s (%d)\n", files[b].target.c_str(), strerror(err.value()), err.value());
		}
		printf("Recovered from update failure (hopefully safely)!\n");
		for (auto file : files)
			std::filesystem::remove(file.updated);
		state.firmware.applyingUpdate = false;
		SendUpdateStatus(state, comm, FW_STATUS_ERROR);
		if (state.firmware.abortedUpdate)
			state.firmware = {};
		return false;
	}

	printf("Successfully applied update (hopefully)!\n");
	for (auto file : files)
		std::filesystem::remove(file.backup);

	state.firmware.applyingUpdate = false;
	state.firmware.appliedUpdate = true;
	SendUpdateStatus(state, comm, FW_STATUS_UPDATED);
	if (state.firmware.abortedUpdate)
	{ // Applied the update despite server aborting, so have to apply post firmware actions anyway
		state.postFirmwareActions = state.firmware.flags;
		state.firmware = {};
	}
	return true;
}

void PostFirmwareUpdateActions(TrackingCameraState &state)
{
	if (state.postFirmwareActions & FW_FLASH_MCU)
	{
		std::unique_lock lock(mcu_mutex);
		if (mcu_switch_bootloader())
		{
			if (!mcu_verify_program(mcu_flash_file))
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				if (mcu_flash_program(mcu_flash_file))
					printf("Successfully flashed MCU with new firmware!\n");
				else
					printf("Failed to flash MCU with the firmware!\n");
			}
			else printf("Will not flash MCU, already flashed with firmware!\n");

			// Reset and probe
			mcu_reconnect();
		}
		else
			printf("Cannot flash MCU, failed to switch to the bootloader!\n");
	}

	if (state.postFirmwareActions & FW_REQUIRE_REBOOT)
	{
		printf("Rebooting Pi!\n");
		std::system("reboot");
	}

	state.postFirmwareActions = FW_FLAGS_NONE;
}

bool ReceiveFirmwareStatus(TrackingCameraState &state, CommState &comm, const uint8_t *data, uint16_t length)
{
	if (!validateFirmwarePacket(state, data, length)) return false;

	uint16_t ID = *(uint16_t*)(data+0);
	// 1 free byte for future use
	FirmwareStatusType type = (FirmwareStatusType)*(uint8_t*)(data+3);
	uint8_t status = *(uint8_t*)(data+4);
	uint8_t index = *(uint8_t*)(data+5);
	// 4 free bytes for future use

	if (type == FW_STATUS_UPDATE)
	{
		if (status == (uint8_t)FW_STATUS_ABORT)
		{
			printf("Received Firmware packet aborting the firmware update!\n");
			if (state.firmware.applyingUpdate)
				state.firmware.abortedUpdate = true;
			else
			 	state.firmware = {};
			return true;
		}
		if (status == (uint8_t)FW_STATUS_UPDATED && state.firmware.appliedUpdate)
		{
			printf("Received Firmware packet acknowledging completed firmware update!\n");
			state.postFirmwareActions = state.firmware.flags;
			state.firmware = {};
			return true;
		}
		return false;
	}

	if (type == FW_INQUIRE_UPDATE)
	{
		for (int i = 0; i < state.firmware.transfers.size(); i++)
			CheckFirmwareTransfer(state.firmware.transfers[i]);
		SendUpdateStatus(state, comm, CheckFirmwareUpdate(state.firmware));
		return true;
	}

	if (type == FW_INQUIRE_TRANSFER)
	{
		auto &transfer = state.firmware.transfers[index];
		SendTransferStatus(state, comm, CheckFirmwareTransfer(transfer), index);
		return true;
	}

	return false;
}
