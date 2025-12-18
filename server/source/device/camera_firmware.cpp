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

#include "camera_firmware.hpp"
#include "tracking_camera.hpp"
#include "tracking_controller.hpp"
#include "comm/packet.hpp"
#include "comm/usb.hpp"
#include "ui/shared.hpp" // Signals
#include "util/log.hpp"

#include "hash/sha256.hpp"

#include <fstream>

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;


/**
 * Firmware update structures
 */

struct FirmwareTransferStatus
{
	std::shared_ptr<TrackingCameraState> camera;
	struct BlockStatus
	{
		TimePoint_t sentTime;
		FirmwareTXStatus result = FW_TX_QUEUED;
	};
	std::vector<BlockStatus> blocks;
	int progress;
	FirmwareTXStatus code;
	TimePoint_t lastRequest;

	FirmwareTransferStatus(std::shared_ptr<TrackingCameraState> &camera)
		: camera(camera), code(FW_TX_QUEUED) {}
};

struct FirmwareTransfer
{
	std::vector<uint8_t> data;
	uint8_t sha256[SHA256::HashBytes];
	std::vector<uint8_t> sign;

	FirmwareTransferType type;
	struct {
		std::string path;
	} file;
	struct {
		VersionDesc since;
		std::string deps;
	} package;

	uint8_t index;
	uint16_t blockCount;
	std::vector<FirmwareTransferStatus> cameras;
};

struct CameraFirmwareUpdate
{
	std::shared_ptr<TrackingCameraState> camera;
	FirmwareStatus status;
	TimePoint_t lastRequest;
	TimePoint_t setupTime;
	TimePoint_t applyTime;

	CameraFirmwareUpdate(std::shared_ptr<TrackingCameraState> &camera)
		: camera(camera) {}
};

struct FirmwareUpdatePlan
{
	VersionDesc version;
	std::string system;
	uint8_t signMethod;
	uint16_t signSize;
	FirmwareUpdateFlags flags;
	std::vector<FirmwareTransfer> transfers;

	uint16_t ID;
	std::stop_token abort;
	FirmwareUpdateRef status;
	std::vector<CameraFirmwareUpdate> cameras;
	int transferIndex, blockStart;
};


/**
 * Firmware update configuration
 */

const uint16_t BLOCK_SIZE = 1000; // Limited by CTRL_TRANSFER_SIZE = 2045
const uint16_t SEND_INTERVAL_MS = 5;
const uint16_t MAX_SEND_AT_ONCE = 2;
const uint16_t MAX_WAITING_COUNT = 10;


/**
 * Camera firmware update packets
 */

static bool CameraValidState(std::shared_ptr<TrackingCameraState> &camera)
{
	if (camera->state.contextualRLock()->error.encountered)
	{
		LOG(LCameraDevice, LError, "Camera %d updating firmware failed because it is still recovering from an error!", camera->id);
		return false; // Cannot handle packet at this time, waiting for recovery
	}
	if (camera->state.contextualRLock()->commState != CommPiReady)
	{
		LOG(LFirmwareUpdate, LError, "Camera %d updating firmware failed because it has not started up yet!", camera->id);
		return false;
	}
	if (camera->isStreaming() || camera->hasSetStreaming())
	{
		LOG(LFirmwareUpdate, LError, "Camera %d updating firmware failed because it is currently streaming!", camera->id);
		return false;
	}
	return true;
}

static std::vector<int> CameraSelectFirmwareTransfers(FirmwareUpdatePlan &update, CameraFirmwareUpdate &camState)
{
	std::vector<int> transfers;
	transfers.reserve(update.transfers.size());
	for (int i = 0; i < update.transfers.size(); i++)
	{
		auto &transfer = update.transfers[i];
		// May choose to not send this transfer to camera
		if (transfer.type == FW_TX_TYPE_FILE && !transfer.file.path.empty())
			transfers.push_back(i);
		else if (transfer.type == FW_TX_TYPE_PACKAGE)
			transfers.push_back(i);
	}
	return transfers;
}

static void CameraShowTransferStatus(CameraFirmwareUpdateStatus &status, FirmwareTransferStatus &camTXStatus, FirmwareTransfer &transfer)
{
	if (camTXStatus.code == FW_TX_QUEUED)
	{
		status.text = asprintf_s("Camera does not require transfer %d!", transfer.index+1);
	}
	else if (camTXStatus.code == FW_TX_TRANSFERRING)
	{
		status.text = asprintf_s("%dKB / %dKB", camTXStatus.progress*BLOCK_SIZE/1000, (int)camTXStatus.blocks.size()*BLOCK_SIZE/1000);
	}
	else if (camTXStatus.code == FW_TX_TRANSFERRED)
	{
		status.text = asprintf_s("Firmware update transfer %d succeeded!", transfer.index+1);
	}
	else if (camTXStatus.code == FW_TX_ERROR)
	{
		status.text = asprintf_s("Firmware update transfer %d failed!", transfer.index+1);
	}
}

static bool CameraInitiateFirmwareUpdate(FirmwareUpdatePlan &update, CameraFirmwareUpdate &camState, std::vector<int> transfers)
{
	if (transfers.empty())
	{
		auto status = camState.camera->firmware->contextualLock();
		status->code = FW_STATUS_INVALID;
		return false;
	}

	// Send update plan to camera (specific to camera)
	int size = FIRMWARE_PACKET_HEADER;
	for (int i : transfers)
	{
		auto &transfer = update.transfers[i];
		size += FIRMWARE_FILE_HEADER + SHA256::HashBytes + update.signSize;
		if (transfer.type == FW_TX_TYPE_FILE)
			size += 2 + transfer.file.path.size();
		else if (transfer.type == FW_TX_TYPE_PACKAGE)
			size += 2 + transfer.package.deps.size();
	}

	std::vector<uint8_t> FWPacket(size);
	uint8_t *ptr = FWPacket.data();
	*(uint16_t*)(ptr+0) = update.ID;
	*(uint8_t*)(ptr+2) = (uint8_t)update.flags;
	*(uint8_t*)(ptr+3) = transfers.size();
	*(uint16_t*)(ptr+4) = BLOCK_SIZE;
	*(uint8_t*)(ptr+6) = update.signMethod;
	*(uint8_t*)(ptr+7) = update.signSize;
	// 2 free bytes for future use
	ptr += FIRMWARE_PACKET_HEADER;

	for (int i : transfers)
	{
		auto &transfer = update.transfers[i];
		*(uint8_t*)(ptr+0) = transfer.index;
		*(uint8_t*)(ptr+1) = (uint8_t)transfer.type;
		*(uint32_t*)(ptr+2) = transfer.data.size();
		ptr += FIRMWARE_FILE_HEADER;
		memcpy(ptr, transfer.sha256, SHA256::HashBytes);
		ptr += SHA256::HashBytes;
		if (!transfer.sign.empty())
			memcpy(ptr, transfer.sign.data(), transfer.sign.size());
		ptr += transfer.sign.size();
		if (transfer.type == FW_TX_TYPE_FILE)
		{
			*(uint16_t*)ptr = transfer.file.path.size();
			ptr += 2;
			if (!transfer.file.path.empty())
				memcpy(ptr, transfer.file.path.data(), transfer.file.path.size());
			ptr += transfer.file.path.size();
		}
		else if (transfer.type == FW_TX_TYPE_PACKAGE)
		{
			*(uint16_t*)ptr = transfer.package.deps.size();
			ptr += 2;
			if (!transfer.package.deps.empty())
				memcpy(ptr, transfer.package.deps.data(), transfer.package.deps.size());
			ptr += transfer.package.deps.size();
		}
	}

	camState.setupTime = sclock::now();
	bool success = camState.camera->sendPacket(PACKET_FW_PREPARE, FWPacket.data(), FWPacket.size());
	auto status = camState.camera->firmware->contextualLock();
	status->lastActivity = sclock::now();
	if (success)
	{
		LOG(LFirmwareUpdate, LInfo, "Requesting firmware update from camera %d...", camState.camera->id);
		status->text = asprintf_s("Requesting firmware update...");
		status->code = FW_STATUS_INITIATING;
	}
	else
	{
		LOG(LFirmwareUpdate, LWarn, "Failed to send firmware update request to camera %d!", camState.camera->id);
		status->text = asprintf_s("Failed to send firmware update request!");
		status->code = FW_STATUS_ERROR;
	}
	return success;
}

static void CamerasSendFirmwareBlock(FirmwareUpdatePlan &update, FirmwareTransfer &transfer, std::vector<int> cameras, uint16_t block)
{
	uint32_t offset = block*BLOCK_SIZE;
	uint16_t size = std::min<std::size_t>(BLOCK_SIZE, transfer.data.size()-offset);
	LOG(LFirmwareUpdate, LDebug, "Sending firmware transfer %d block %d of size %d to %d cameras\n", transfer.index, block, size, (int)cameras.size());

	std::vector<uint8_t> FWPacket(FIRMWARE_PACKET_HEADER+size+PACKET_CHECKSUM_SIZE);
	*(uint16_t*)(FWPacket.data()+0) = update.ID;
	// 1 free byte for future use
	*(uint8_t*)(FWPacket.data()+3) = transfer.index;
	*(uint16_t*)(FWPacket.data()+4) = block;
	*(uint16_t*)(FWPacket.data()+6) = size;
	// 2 free bytes for future use
	memcpy(FWPacket.data()+FIRMWARE_PACKET_HEADER, transfer.data.data()+offset, size);
	calculateForwardPacketChecksum(FWPacket.data(), FIRMWARE_PACKET_HEADER+size, FWPacket.data()+FIRMWARE_PACKET_HEADER+size);

	// USB bandwidth is currently the limiting factor since we use control transfers
	// So sending blocks to multiple cameras at once
	std::map<std::shared_ptr<TrackingControllerState>, int> mapping;
	for (int c : cameras)
	{ // Accumulate cameras by controller
		auto &camera = transfer.cameras[c].camera;
		if (camera->controller && CameraValidState(camera))
			mapping[camera->controller] |= 1<<camera->port;
	}
	for (auto &cont : mapping)
	{ // Send packet to all cameras connected to a controller at once
		bool success = comm_submit_control_data(cont.first->comm, COMMAND_OUT_SEND_PACKET,
			(uint16_t)PACKET_FW_BLOCK, (uint16_t)cont.second, FWPacket.data(), (uint16_t)FWPacket.size()) >= 0;
		if (!success)
			LOG(LFirmwareUpdate, LWarn, "Failed to send firmware block with offset %d to controller %d!", offset, cont.first->id);
	}
}

static bool CameraApplyFirmwareUpdate(FirmwareUpdatePlan &update, CameraFirmwareUpdate &camState)
{
	std::vector<uint8_t> FWPacket(FIRMWARE_PACKET_HEADER);
	*(uint16_t*)(FWPacket.data()+0) = update.ID;
	// 8 free bytes for future use

	camState.applyTime = sclock::now();
	camState.lastRequest = sclock::now();
	bool success = camState.camera->sendPacket(PACKET_FW_APPLY, FWPacket.data(), FWPacket.size());
	auto status = camState.camera->firmware->contextualLock();
	if (success)
	{
		status->text = "Applying firmware update...!";
		status->code = FW_STATUS_REQAPPLY;
	}
	else
	{
		LOG(LFirmwareUpdate, LWarn, "Camera %d failed to send firmware apply packet!", camState.camera->id);
		status->text = "Failed to send final packet to apply firmware update!";
		status->code = FW_STATUS_ERROR;
	}
	return success;
}

static bool CameraSendFirmwareStatus(FirmwareUpdatePlan &update, TrackingCameraState &camera, FirmwareStatusType type, uint8_t status = 0, uint8_t transfer = 0)
{
	std::vector<uint8_t> FWPacket(FIRMWARE_PACKET_HEADER);
	*(uint16_t*)(FWPacket.data()+0) = update.ID;
	// 1 free bytes for future use
	*(uint8_t*)(FWPacket.data()+3) = (uint8_t)type;
	*(uint8_t*)(FWPacket.data()+4) = status;
	*(uint8_t*)(FWPacket.data()+5) = transfer;
	// 4 free bytes for future use

	bool success = camera.sendPacket(PACKET_FW_STATUS, FWPacket.data(), FWPacket.size());
	if (!success)
	{
		LOG(LFirmwareUpdate, LWarn, "Camera %d failed to send status request packet!", camera.id);
		auto status = camera.firmware->contextualLock();
		status->text = "Failed to send status request packet!";
		status->code = FW_STATUS_ERROR;
	}
	return success;
}

static bool CameraSendFirmwareStatus(FirmwareUpdatePlan &update, CameraFirmwareUpdate &camState, FirmwareStatusType type, uint8_t status = 0, uint8_t transfer = 0)
{
	camState.lastRequest = sclock::now();
	return CameraSendFirmwareStatus(update, *camState.camera, type, status, transfer);
}

static bool CameraSendFirmwareStatus(FirmwareUpdatePlan &update, FirmwareTransferStatus &camTXState, uint8_t transfer = 0)
{
	camTXState.lastRequest = sclock::now();
	return CameraSendFirmwareStatus(update, *camTXState.camera, FW_INQUIRE_TRANSFER, 0, transfer);
}

static bool CameraReceiveFirmwareStatus(FirmwareUpdatePlan &update, CameraFirmwareUpdate &camState, CameraFirmwareUpdateStatus &camStatus, std::vector<uint8_t> &packet)
{
	uint16_t ID = *(uint16_t*)(packet.data()+0);
	// 1 free byte for future use
	FirmwareStatusType type = *(FirmwareStatusType*)(packet.data()+3);
	uint8_t status = *(uint8_t*)(packet.data()+4);
	uint8_t index = *(uint8_t*)(packet.data()+5);
	uint16_t block = *(uint16_t*)(packet.data()+6);
	// 2 free bytes for future use

	if (ID != update.ID)
	{
		LOG(LFirmwareUpdate, LWarn, "Received status packet with invalid firmware ID %d != %d!", ID, update.ID);
		return true;
	}

	if (type == FW_STATUS_UPDATE)
	{
		switch ((FirmwareStatus)status)
		{
		case FW_STATUS_INITIATED:
		{ // Camera accepted firmware update request and is ready to receive transfers
			LOG(LFirmwareUpdate, LInfo, "Camera %d accepted firmware update request, ready to transfer!", camState.camera->id);
			camStatus.text = asprintf_s("Transferring firmware update...");
			camStatus.code = FW_STATUS_INITIATED;
			return true;
		}
		case FW_STATUS_TRANSFERRING:
		{ // Camera sent status (on request) that some transfers are not complete yet
			LOG(LFirmwareUpdate, LInfo, "Camera %d sent status that some transfers are still requiring blocks!", camState.camera->id);
			camStatus.code = FW_STATUS_TRANSFERRING;
			return true;
		}
		case FW_STATUS_TRANSFERRED:
		{ // Camera acknowledged it received all transfers successfully
			LOG(LFirmwareUpdate, LInfo, "Camera %d finished all firmware transfers!", camState.camera->id);
			camStatus.text = asprintf_s("Firmware update transfer finished!");
			camStatus.code = FW_STATUS_TRANSFERRED;
			return true;
		}
		case FW_STATUS_ABORT:
		{ // Camera rejected update during transfer (perhaps user intervention?)
			LOG(LFirmwareUpdate, LWarn, "Camera %d aborted the firmware update!", camState.camera->id);
			// Send acknowledgement back so it can safely clear data
			CameraSendFirmwareStatus(update, camState, FW_STATUS_UPDATE, (uint8_t)FW_STATUS_ABORT);
			if (camStatus.code == FW_STATUS_INITIATING)
				camStatus.text = asprintf_s("Update request was denied!");
			else if (camStatus.code == FW_STATUS_TRANSFERRING)
				camStatus.text = asprintf_s("Update was aborted during transfer!");
			else if (camStatus.code == FW_STATUS_UPDATING)
				camStatus.text = asprintf_s("Update was aborted when applying!");
			else
				camStatus.text = asprintf_s("Update was unexpectedly aborted during state %d!", camStatus.code);
			camStatus.code = FW_STATUS_ABORT;
			return false;
		}
		case FW_STATUS_ERROR:
		case FW_STATUS_ISSUE:
		{ // Camera encountered an error during transfer (perhaps wrong transfer SHA256 despite correct blocks)
			LOG(LFirmwareUpdate, LWarn, "Camera %d encountered an error during firmware update!", camState.camera->id);
			if (camStatus.code == FW_STATUS_INITIATING)
				camStatus.text = asprintf_s("Update request encountered an error!");
			else if (camStatus.code == FW_STATUS_TRANSFERRING)
				camStatus.text = asprintf_s("Update encountered an error during transfer!");
			else if (camStatus.code == FW_STATUS_UPDATING)
				camStatus.text = asprintf_s("Applying the firmware update failed!");
			else
				camStatus.text = asprintf_s("Update encountered an unexpected error!");
			camStatus.code = FW_STATUS_ERROR;
			return false;
		}
		case FW_STATUS_UPDATING:
		{ // Camera confirms it is currently applying the update
			LOG(LFirmwareUpdate, LInfo, "Camera %d is applying the firmware update...", camState.camera->id);
			camStatus.text = asprintf_s("Update is being applied...");
			camStatus.code = FW_STATUS_UPDATING;
			return true;
		}
		case FW_STATUS_UPDATED:
		{ // Camera confirms it has applied the update to disk (but may still be flashing the MCU or rebooting)
			LOG(LFirmwareUpdate, LInfo, "Camera %d successfully applied firmware update!", camState.camera->id);
			// Send acknowledgement back so it can safely clear data and restart
			CameraSendFirmwareStatus(update, camState, FW_STATUS_UPDATE, (int)FW_STATUS_UPDATED);
			camStatus.text = asprintf_s("Update applied!");
			camStatus.code = FW_STATUS_UPDATED;
			return true;
		}
		default:
			LOG(LFirmwareUpdate, LWarn, "Camera %d received an unknown firmware update status code %d!", camState.camera->id, status);
			return false;
		}
	}

	if (index < 0 || index >= update.transfers.size())
	{
		LOG(LFirmwareUpdate, LWarn, "Received status packet for invalid transfer!");
		return true;
	}
	auto &transfer = update.transfers[index];
	int c;
	for (c = 0; c < transfer.cameras.size(); c++)
		if (transfer.cameras[c].camera == camState.camera) break;
	if (c >= transfer.cameras.size())
	{
		LOG(LFirmwareUpdate, LWarn, "Received status packet for transfer that has not been set up!");
		return true;
	}
	auto &camTXStatus = transfer.cameras[c];
	FirmwareTXStatus txStatus = (FirmwareTXStatus)status;

	if (type == FW_STATUS_TRANSFER)
	{ // Transfer
		if (txStatus == FW_TX_ERROR)
		{
			LOG(LFirmwareUpdate, LError, "Failed transfer %d!", index);
			camTXStatus.code = txStatus;
			camStatus.code = FW_STATUS_FAILEDTRANSFER;
			camStatus.text = asprintf_s("Firmware update transfer %d failed!", index+1);
			return false;
		}
		else if (txStatus == FW_TX_TRANSFERRED)
		{
			LOG(LFirmwareUpdate, LInfo, "Confirmed successful transfer %d!", index);
			camTXStatus.code = txStatus;
			camStatus.text = asprintf_s("Firmware update transfer %d succeeded!", transfer.index+1);
			return true;
		}
		LOG(LFirmwareUpdate, LWarn, "Received unknown status %d for transfer %d!!", status, index);
		return true;
	}

	if (type == FW_STATUS_BLOCK)
	{ // Transfer block
		if (block < 0 || block >= transfer.blockCount)
		{
			LOG(LFirmwareUpdate, LWarn, "Received status packet for invalid block!");
			return true;
		}
		if (camTXStatus.blocks[block].result != FW_TX_TRANSFERRED)
		{
			if (txStatus == FW_TX_TRANSFERRED)
				LOG(LFirmwareUpdate, LTrace, "Confirmed successful transfer of block %d for transfer %d! aftter %.1fms!",
					block, index,  dtMS(camTXStatus.blocks[block].sentTime, sclock::now()));
			else if (txStatus == FW_TX_ERROR)
				LOG(LFirmwareUpdate, LDarn, "Failed to transfer block %d for transfer %d, need to retransmit!", block, index);
			else
				LOG(LFirmwareUpdate, LWarn, "Received unknown status %d for block %d of transfer %d!!", status, block, index);
			camTXStatus.blocks[block].result = txStatus; 
			if (txStatus == FW_TX_TRANSFERRED)
				camTXStatus.progress++;
			CameraShowTransferStatus(camStatus, camTXStatus, transfer);
		}
		else
		{ // Status packet might have gotten extremely delayed
			LOG(LFirmwareUpdate, LDarn, "Received redundant status %d for block %d of transfer %d!!", status, block, index);
		}
		return true;
	}

	LOG(LFirmwareUpdate, LWarn, "Received invalid status type %d with status %d, block %d, transfer %d!!", type, status, block, index);
	return true;
}

static void UpdateCameraStatus(FirmwareUpdatePlan &update, CameraFirmwareUpdate &camState)
{
	auto camStatus = camState.camera->firmware->contextualLock();
	bool abort = false;
	for (auto &packet : camStatus->packets)
	{ // Read and apply status messages from camera
		if (!CameraReceiveFirmwareStatus(update, camState, *camStatus, packet))
		{
			abort = true;
			break;
		}
		camStatus->lastActivity = sclock::now();
		camState.lastRequest = sclock::now();
	}
	camStatus->packets.clear();
	if (!abort)
	{ // Check for activity timeout
		abort = true;
		if (camStatus->code == FW_STATUS_INITIATING && dtMS(camStatus->lastActivity, sclock::now()) > 500)
			camStatus->text = asprintf_s("Firmware update request timed out!");
		else if (camStatus->code == FW_STATUS_TRANSFERRING && dtMS(camStatus->lastActivity, sclock::now()) > 1000)
			camStatus->text = asprintf_s("Firmware update timed out during transfer!");
		else if (camStatus->code == FW_STATUS_REQAPPLY && dtMS(camStatus->lastActivity, sclock::now()) > 1000)
			camStatus->text = asprintf_s("Firmware update timed out requesting to apply the update!");
		else if (camStatus->code == FW_STATUS_UPDATING && dtMS(camStatus->lastActivity, sclock::now()) > 20000)
			camStatus->text = asprintf_s("Firmware update timed out applying the update!");
		else
			abort = false;
		if (abort) camStatus->code = FW_STATUS_ERROR;
	}
	if (!abort)
	{ // Re-send messages in case a status packet from camera got dropped
		camState.status = camStatus->code;
		auto unlock = camStatus.scopedUnlock();
		if (camState.status == FW_STATUS_INITIATING && dtMS(camState.setupTime, sclock::now()) > 100)
			abort = !CameraInitiateFirmwareUpdate(update, camState, CameraSelectFirmwareTransfers(update, camState));
		else if (camState.status == FW_STATUS_TRANSFERRING && dtMS(camState.lastRequest, sclock::now()) > 200)
			abort = !CameraSendFirmwareStatus(update, camState, FW_INQUIRE_UPDATE);
		else if (camState.status == FW_STATUS_REQAPPLY && dtMS(camState.applyTime, sclock::now()) > 100)
			abort = !CameraApplyFirmwareUpdate(update, camState);
		else if (camState.status == FW_STATUS_UPDATING && dtMS(camState.lastRequest, sclock::now()) > 2000)
			abort = !CameraSendFirmwareStatus(update, camState, FW_INQUIRE_UPDATE);
		else if (camState.status != FW_STATUS_NONE && camState.status != FW_STATUS_INITIATING && dtMS(camState.lastRequest, sclock::now()) > 2000)
		{ // This one is not required, just to prevent a timeout in cameras that are waiting to apply the update if some cameras take significantly longer
			abort = !CameraSendFirmwareStatus(update, camState, FW_INQUIRE_UPDATE);
		}
	}
	if (abort)
	{
		LOG(LFirmwareUpdate, LWarn, "Camera %d: %s", camState.camera->id, camStatus->text.c_str());
		for (int i = 0; i < update.transfers.size(); i++)
			std::erase_if(update.transfers[i].cameras, [&](auto&camStat){ return camStat.camera == camState.camera; });
	}
	camState.status = camStatus->code;
}


/**
 * General firmware update process
 */

static void ReportFirmwareUpdateError(FirmwareUpdateRef &updateStatus, std::string error)
{
	LOG(LFirmwareUpdate, LError, "Firmware Update failed: %s", error.c_str());
	auto status = updateStatus->contextualLock();
	status->text = error;
	status->code = FW_STATUS_ERROR;
}

static void SwitchToTransfer(FirmwareUpdatePlan &update, FirmwareTransfer &transfer)
{
	for (auto &camTXStatus : transfer.cameras)
	{
		auto camStatus = camTXStatus.camera->firmware->contextualLock();
		CameraShowTransferStatus(*camStatus, camTXStatus, transfer);
	}
	auto status = update.status->contextualLock();
	LOG(LFirmwareUpdate, LInfo, "Starting firmware transfer %d/%d", transfer.index+1, (int)update.transfers.size());
	status->text = asprintf_s("Firmware transfer %d/%d", transfer.index+1, (int)update.transfers.size());
	status->code = FW_STATUS_TRANSFERRING;
}

static bool UpdateTransferStatus(FirmwareUpdatePlan &update, FirmwareTransfer &transfer)
{
	bool needsTransferring = false;
	for (auto &camTXStatus : transfer.cameras)
	{ // Restart failed transfers incase we ever allow revisiting them
		if (camTXStatus.code == FW_TX_TRANSFERRING)
		{ // Waiting for camera to finish a firmware transfer
			if (dtMS(camTXStatus.lastRequest, sclock::now()) > 100)
				CameraSendFirmwareStatus(update, camTXStatus, transfer.index);
			needsTransferring = true;
		}
		if (camTXStatus.code == FW_TX_QUEUED)
		{ // May have been an error before (once we allow restarting from erroneous transfers), so clear everything
			LOG(LFirmwareUpdate, LInfo, "Starting to transmit transfer %d with %d blocks to camera %d!", transfer.index, transfer.blockCount, camTXStatus.camera->id);
			camTXStatus.code = FW_TX_TRANSFERRING;
			camTXStatus.progress = 0;
			camTXStatus.blocks.clear();
			camTXStatus.blocks.resize(transfer.blockCount);
			needsTransferring = true;
		}
	}
	return needsTransferring;
}

static bool InitFirmwareUpdatePlan(FirmwareUpdatePlan &update)
{
	if (!update.status) return false;

	update.ID = std::max(1, rand()%(256*256));

	if (update.transfers.empty())
	{
		ReportFirmwareUpdateError(update.status, asprintf_s("Nothing to apply in firmware file!"));
		return false;
	}
	for (int i = 0; i < update.transfers.size(); i++)
	{
		auto &transfer = update.transfers[i];
		transfer.index = i;
		transfer.blockCount = (transfer.data.size()+BLOCK_SIZE-1)/BLOCK_SIZE;
		if (transfer.data.size() > FIRMWARE_MAX_TRANSFER_SIZE)
		{
			ReportFirmwareUpdateError(update.status, asprintf_s("Firmware file %d of size %dB exceeds limit!", i+1, (int)transfer.data.size()));
			return false;
		}
		if (transfer.sign.size() != update.signSize)
		{
			ReportFirmwareUpdateError(update.status, asprintf_s("Firmware file %d has invalid signature size of %d (expected %d)!", i+1, (int)transfer.sign.size(), update.signSize));
			return false;
		}
		// TODO: Verify SHA256
	}
	return true;
}

static void HandleFirmwareTransfer(FirmwareUpdatePlan &update, FirmwareTransfer &transfer)
{
	thread_local std::vector<int> camerasSendTX;
	camerasSendTX.clear();
	int sentCount = 0, waitingCount = 0;
	int earliestBlock = transfer.blockCount;
	for (int b = update.blockStart; b < transfer.blockCount; b++)
	{
		bool awaitingBlockStatus = false; // At least one camera has not sent a status for block yet
		for (int i = 0; i < transfer.cameras.size(); i++)
		{
			auto &camTXStatus = transfer.cameras[i];
			if (camTXStatus.code != FW_TX_TRANSFERRING) continue;
			auto &block = camTXStatus.blocks[b];
			if (block.result == FW_TX_TRANSFERRED) continue;
			earliestBlock = std::min(earliestBlock, b);
			if (block.result == FW_TX_TRANSFERRING && dtMS(block.sentTime, sclock::now()) < 200)
			{ // Still waiting for block status
				awaitingBlockStatus = true;
				continue;
			}
			if (block.result == FW_TX_TRANSFERRING)
				LOG(LFirmwareUpdate, LDarn, "Re-transmiting transfer %d's block %d to camera %d after no response for %.2fms!", transfer.index, b, camTXStatus.camera->id, dtMS(block.sentTime, sclock::now()));
			else
			 	LOG(LFirmwareUpdate, LTrace, "Transmiting transfer %d's block %d to camera %d!", transfer.index, b, camTXStatus.camera->id);
			// Send block to camera
			camerasSendTX.push_back(i);
			camTXStatus.lastRequest = sclock::now();
			block.sentTime = sclock::now();
			block.result = FW_TX_TRANSFERRING;
		}
		if (awaitingBlockStatus)
			waitingCount++;
		if (!camerasSendTX.empty())
		{ // Send block to one or more cameras
			CamerasSendFirmwareBlock(update, transfer, camerasSendTX, b);
			camerasSendTX.clear();
			sentCount++;
		}
		if (sentCount >= MAX_SEND_AT_ONCE) break;
		if (waitingCount >= MAX_WAITING_COUNT) break;
	}
	update.blockStart = earliestBlock;
}

static void ExecuteFirmwareUpdatePlan(FirmwareUpdatePlan &update)
{
	LOG(LFirmwareUpdate, LInfo, "Updating firmware of %d cameras!", (int)update.cameras.size());

	while (!update.abort.stop_requested())
	{
		// Update camera transfer status with newest communication status
		// Will read camera status packets and handle timeouts
		std::array<int, FW_STATUS_MAX> cameraStatus{};
		for (auto &camState : update.cameras)
		{
			UpdateCameraStatus(update, camState);
			cameraStatus[camState.status]++;
		}

		if (cameraStatus[FW_STATUS_NONE])
		{ // Initiate firmware update by sending setup packet to cameras
			for (auto &camState : update.cameras)
			{
				if (camState.status == FW_STATUS_NONE)
					CameraInitiateFirmwareUpdate(update, camState, CameraSelectFirmwareTransfers(update, camState));
			}
			auto status = update.status->contextualLock();
			status->text = "Setting up cameras...";
			status->code = FW_STATUS_INITIATING;
		}
		else if (cameraStatus[FW_STATUS_INITIATING])
		{ // Waiting for all cameras to initiate the firmware update
		}
		else if (cameraStatus[FW_STATUS_INITIATED])
		{ // Start transferring data to all cameras (that initiated the firmware update) at once
			for (auto &camState : update.cameras)
			{
				auto status = camState.camera->firmware->contextualLock();
				if (camState.status != FW_STATUS_INITIATED) continue;
				LOG(LFirmwareUpdate, LInfo, "Setting up firmware transfers for camera %d!", camState.camera->id);
				status->code = FW_STATUS_TRANSFERRING;
				status->lastActivity = sclock::now();
				// Enter transfers that camera needs
				for (int i : CameraSelectFirmwareTransfers(update, camState))
					update.transfers[i].cameras.emplace_back(camState.camera);
			}
			update.transferIndex = 0;
			update.blockStart = 0;
			SwitchToTransfer(update, update.transfers[update.transferIndex]);
		}
		else if (cameraStatus[FW_STATUS_TRANSFERRING])
		{ // Transfer data to all cameras in lockstep (to optimise data transfer through controller)
			if (!UpdateTransferStatus(update, update.transfers[update.transferIndex]))
			{ // Transfer is done
				update.transferIndex = (update.transferIndex++) % update.transfers.size();
				update.blockStart = 0;
				if (!UpdateTransferStatus(update, update.transfers[update.transferIndex]))
				{ // May be waiting only for update status packet, with all individual transfers done
					std::this_thread::sleep_for(std::chrono::milliseconds(50));
					continue;
				}
				SwitchToTransfer(update, update.transfers[update.transferIndex]);
			}
			HandleFirmwareTransfer(update, update.transfers[update.transferIndex]);
		}
		else if (cameraStatus[FW_STATUS_TRANSFERRED])
		{ // No camera is transferring anymore, tell cameras that finished their transfers to apply the firmware update
			// NOTE: May still have individual transfers whose transferred status packets got lost, but this is not a problem in practice
			for (auto &camState : update.cameras)
			{
				if (camState.status == FW_STATUS_TRANSFERRED)
				{
					LOG(LFirmwareUpdate, LDebug, "Camera %d has completed all transfers, applying firmware update!\n", camState.camera->id);
					CameraApplyFirmwareUpdate(update, camState);
				}
			}
			LOG(LFirmwareUpdate, LInfo, "Applying firmware update...");
			auto status = update.status->contextualLock();
			status->text = "Applying firmware update...";
			status->code = FW_STATUS_UPDATING;
		}
		else if (cameraStatus[FW_STATUS_REQAPPLY])
		{ // Wait for all cameras to respond to the firmware application request
		}
		else if (cameraStatus[FW_STATUS_UPDATING])
		{ // Wait for all cameras to finish applying the firmware update
		}
		else if (cameraStatus[FW_STATUS_UPDATED])
		{ // At least some cameras succeeded updating
			auto status = update.status->contextualLock();
			status->text = "Firmware update finished!";
			status->code = FW_STATUS_UPDATED;
			break;
		}
		else
		{ // Every camera has either updated, or failed (error, abort, invalid, failedtransfer, etc.)
			auto status = update.status->contextualLock();
			status->text = "Firmware update concluded.";
			status->code = FW_STATUS_ERROR;
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(SEND_INTERVAL_MS));
	}

	// Notify cameras that firmware update is ending so they may clean up
	for (auto &camState : update.cameras)
	{ // Skip the only two outcomes where we already send a status update acknowledging them
		if (camState.status == FW_STATUS_UPDATED) continue;
		if (camState.status == FW_STATUS_ABORT) continue;
		CameraSendFirmwareStatus(update, camState, FW_STATUS_UPDATE, (uint8_t)FW_STATUS_ABORT);
	}

	if (update.abort.stop_requested())
	{
		LOG(LFirmwareUpdate, LError, "Firmware Update aborted by the user!");
		auto status = update.status->contextualLock();
		status->text = "Firmware Update has been aborted!";
		status->code = FW_STATUS_ABORT;
	}

	LOG(LFirmwareUpdate, LInfo, "Exited firmware update thread!");
}


/**
 * Different firmware update plans
 */

FirmwareUpdateRef CamerasFlashFirmwareFile(std::vector<std::shared_ptr<TrackingCameraState>> &cameras, std::string firmware)
{
	FirmwareUpdateRef updateStatus = std::make_shared<Synchronised<FirmwareUpdateStatus>>();

	std::ifstream fs(firmware, std::ios::binary);
	if (!fs.is_open())
	{
		ReportFirmwareUpdateError(updateStatus, asprintf_s("Cannot read firmware file '%s'!", firmware.c_str()));
		return updateStatus;
	}
	fs.seekg(0, std::ios::end);
	std::size_t fwSize = fs.tellg();
	if (fwSize > 1000000000)
	{
		ReportFirmwareUpdateError(updateStatus, asprintf_s("Firmware file '%s' of size %dMB exceeds limit of 1GB!", firmware.c_str(), (int)(fwSize/1000000)));
		return updateStatus;
	}

	// Begin to build update plan
	FirmwareUpdatePlan update = {};
	update.status = updateStatus;
	update.abort = updateStatus->contextualLock()->abort.get_token();
	update.flags = FW_FLAGS_NONE;

	if (firmware.ends_with(".zip"))
	{ // TODO: Read zipped folder with json describing full update plan
		ReportFirmwareUpdateError(updateStatus, asprintf_s("Uploading update packages is not yet supported!"));
		return updateStatus;
	}
	else
	{
		// Setup file metadata
		update.transfers.emplace_back();
		FirmwareTransfer &transfer = update.transfers.back();
		if (firmware.ends_with(".tgz") || firmware.ends_with(".tar.gz"))
		{
			transfer.type = FW_TX_TYPE_FILE;
			transfer.file.path = "/mnt/mmcblk0p2/tce/mydata.tgz";
			update.flags = (FirmwareUpdateFlags)(update.flags | FW_REQUIRE_REBOOT);
		}
		else if (firmware.ends_with(".bin"))
		{
			transfer.type = FW_TX_TYPE_FILE;
			transfer.file.path = "/mnt/mmcblk0p2/tce/TrackingCameraMCU.bin";
			update.flags = (FirmwareUpdateFlags)(update.flags | FW_FLASH_MCU);
		}
		else if (firmware.ends_with(".tcz"))
		{
			ReportFirmwareUpdateError(updateStatus, asprintf_s("Uploading system packages is not yet supported!"));
			return updateStatus;
			transfer.type = FW_TX_TYPE_PACKAGE;
		}

		// Read file data
		transfer.data.resize(fwSize);
		fs.seekg(0);
		fs.read((char*)transfer.data.data(), transfer.data.size());
		if (fs.fail())
		{
			ReportFirmwareUpdateError(updateStatus, asprintf_s("Failed to read full data of firmware file: %s (%d)!", strerror(errno), errno));
			return updateStatus;
		}

		// Calculate SHA256 of file
		SHA256 sha;
		sha.add(transfer.data.data(), transfer.data.size());
		sha.getHash(transfer.sha256);
		LOG(LFirmwareUpdate, LDebug, "Calculated hash for firmware file as %s", sha.getHash().c_str());
	}

	// Check and init update plan
	if (!InitFirmwareUpdatePlan(update))
		return updateStatus;

	// Add cameras to update plan
	for (int i = 0; i < cameras.size(); i++)
	{
		if (!CameraValidState(cameras[i])) continue;
		if (cameras[i]->firmware) continue;
		cameras[i]->firmware = std::make_shared<Synchronised<CameraFirmwareUpdateStatus>>();
		auto camStatus = cameras[i]->firmware->contextualLock();
		camStatus->ID = update.ID;
		camStatus->lastActivity = sclock::now();
		update.cameras.push_back(cameras[i]);
	}

	if (update.cameras.empty())
	{
		ReportFirmwareUpdateError(updateStatus, asprintf_s("No camera was ready to receive a firmware update!"));
		return updateStatus;
	}

	threadPool.push([](int id, FirmwareUpdatePlan &update)
	{
		ExecuteFirmwareUpdatePlan(update);
	}, std::move(update));

	auto status = updateStatus->contextualLock();
	status->text = "Starting firmware update...";
	status->code = FW_STATUS_INVALID;
	return updateStatus;
}