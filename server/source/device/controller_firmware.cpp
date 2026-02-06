
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

#include "controller_firmware.hpp"
#include "tracking_controller.hpp"
#include "comm/usb.hpp"

#include "util/log.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include "libusb/libusb.h"

#include <fstream>
#include <cstdio>
#include <cstring>
#include <cerrno>

static void ReportFirmwareUpdateError(ControllerFirmwareUpdateRef &updateStatus, std::string error)
{
	LOG(LFirmwareUpdate, LError, "Firmware Update failed: %s", error.c_str());
	auto status = updateStatus->contextualLock();
	status->text = error;
	status->success = false;
	status->complete = true;
}

static void FlashCH32V307(ControllerFirmwareUpdateRef updateStatus, std::vector<uint8_t> &firmwareFile);

ControllerFirmwareUpdateRef ControllerFlashFirmwareFile(std::shared_ptr<TrackingControllerState> controller, std::string firmware)
{
	// TODO: Ask controller via USB to automatically switch to Bootloader 1/3
	// and perhaps ask user for button confirmation first

	// Assume now this (and only this) controller is already in bootloader

	ControllerFirmwareUpdateRef updateStatus = std::make_shared<Synchronised<ControllerFirmwareUpdateStatus>>();

	std::ifstream fs(firmware, std::ios::binary);
	if (!fs.is_open())
	{
		ReportFirmwareUpdateError(updateStatus, asprintf_s("Cannot read firmware file '%s'!", firmware.c_str()));
		return updateStatus;
	}
	fs.seekg(0, std::ios::end);
	std::size_t fwSize = fs.tellg();

	std::vector<uint8_t> codeFile;
	codeFile.resize(fwSize + 8 - (fwSize%8), 0xFF);
	fs.seekg(0);
	fs.read((char*)codeFile.data(), fwSize);
	fs.close();
	if (fs.fail())
	{
		ReportFirmwareUpdateError(updateStatus, asprintf_s("Failed to read full data of firmware file: %s (%d)!", strerror(errno), errno));
		return updateStatus;
	}

	threadPool.push([](int id, ControllerFirmwareUpdateRef &updateStatus, std::vector<uint8_t> &codeFile)
	{
		FlashCH32V307(updateStatus, codeFile);
	}, updateStatus, std::move(codeFile));


	auto status = updateStatus->contextualLock();
	status->text = "Starting update...";
	status->size = fwSize;

	return updateStatus;
}

/**
 * MIT License
 * 
 * Original (https://github.com/NgoHungCuong/vnproch551):
 * Copyright (c) 2019 Ngo Hung Cuong
 * 
 * Adaptations for CH32V307 and Integration into codebase (device/controller_firmware.cpp):
 * Copyright (c) 2025 Seneral
 * 
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

static bool Command(libusb_device_handle *dev, uint8_t *wrData, uint8_t *rdData, uint8_t rdLen)
{
	uint8_t wrLen = wrData[1] + 3;
	int len;
	if (libusb_bulk_transfer(dev, 0x02, (unsigned char*)wrData, wrLen, &len, 5000) != 0)
		return false;
	if (libusb_bulk_transfer(dev, 0x82, (unsigned char*)rdData, rdLen, &len, 5000) != 0)
		return false;
	return true;
}

static void FlashCH32V307(ControllerFirmwareUpdateRef updateStatus, std::vector<uint8_t> &firmwareFile)
{
	const int MAX_FLASH_SIZE = 196608;
	if (firmwareFile.size()%8 != 0)
		return ReportFirmwareUpdateError(updateStatus, asprintf_s("Firmware file size %d is not a multiple of 8!", (int)firmwareFile.size()));
	if (firmwareFile.size() > MAX_FLASH_SIZE)
		return ReportFirmwareUpdateError(updateStatus, asprintf_s("Firmware file size %d is larger than maximum flash size of %d 8!", (int)firmwareFile.size(), MAX_FLASH_SIZE));
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");

	libusb_context *context;
	libusb_init(&context);
	libusb_device_handle *dev = libusb_open_device_with_vid_pid(context, 0x4348, 0x55e0);
	if (dev == NULL)
		return ReportFirmwareUpdateError(updateStatus, "Found no WinChipHead Bootloader USB!");
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");
	libusb_set_auto_detach_kernel_driver(dev, 1);
	libusb_claim_interface(dev, 0);
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");

	uint8_t resp[64];

	/* Detect MCU */
	uint8_t detectCmd[] = {
		0xA1, 0x12, 0x00,
		0x70, 0x17,
		// "MCU ISP & WCH.CN"
		0x4D, 0x43, 0x55, 0x20, 0x49, 0x53, 0x50, 0x20,
		0x26, 0x20, 0x57, 0x43, 0x48, 0x2e, 0x43, 0x4e
	};
	if (!Command(dev, detectCmd, resp, 6))
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Detect Command Failed!");
	uint8_t type = resp[4], family = resp[5];
	if (type != 0x70 && family != 0x17)
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Not a CH32V307!");
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");

	/* Read Config */
	uint8_t readConfigCmd[] = {
		0xA7, 0x02, 0x00,
		0x1F, 0x00
	};
	if (!Command(dev, readConfigCmd, resp, 30))
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Failed to read config!");
	uint8_t config[12];
	memcpy(config, resp+6, 12);
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");
	uint8_t &RDPR = config[0];
	uint8_t &USER = config[2];

	/* Print Info */
	LOG(LFirmwareUpdate, LDebug, "Bootloader: %d.%d.%d", resp[19], resp[20], resp[21]);
	LOG(LFirmwareUpdate, LDebug, "Chip ID: %02X %02X %02X %02X %02X %02X %02X %02X",
		resp[22], resp[23], resp[24], resp[25], resp[26], resp[27], resp[28], resp[29]);
	/* Print Config */
	std::stringstream ss;
	printBuffer(ss, config, sizeof(config));
	LOG(LFirmwareUpdate, LDebug, "Config: %s", ss.str().c_str());
	LOG(LFirmwareUpdate, LDebug, "    RDPR: %.2x", config[0]);
	LOG(LFirmwareUpdate, LDebug, "    IWDG_SW: %d", (USER >> 0)&0b1);
	LOG(LFirmwareUpdate, LDebug, "    STOP_RST: %d", (USER >> 1)&0b1);
	LOG(LFirmwareUpdate, LDebug, "    STANDBY_RST: %d", (USER >> 2)&0b1);
	LOG(LFirmwareUpdate, LDebug, "    SRAM_CODE_MODE: %d", (USER >> 5)&0b111);

	/* Check bootloader version */
	uint32_t bootloader = resp[19] << 16 | resp[20] << 8 | resp[21];
	if (bootloader < 0x020301)
		return ReportFirmwareUpdateError(updateStatus, asprintf_s("Bootloader: Version %d.%d.%d is not supported!", resp[19], resp[20], resp[21]));

	/* Calc XOR Mask from Chip ID */
	const uint8_t CHIP_ID_LEN = 8; // For CH32V307
	uint8_t u8Sum = 0;
	for (int i = 0; i < CHIP_ID_LEN; i++)
		u8Sum += resp[22+i];
	uint8_t u8Mask[8];
	for (int i = 0; i < 8; ++i)
		u8Mask[i] = u8Sum;
	u8Mask[7] += type;
	uint8_t keyChecksum = 0;
	for (int i = 0; i < 8; ++i)
		keyChecksum += u8Mask[i];

	// Configure
	RDPR = 0xA5; // Remove read protection
	USER = (USER & ~(0b111 << 5)) | (0b001 << 5); // Set SRAM_CODE_MODE to 00x: CODE-192KB + RAM-128KB

	/* Write config back */
	uint8_t writeConfigCmd[3+2+sizeof(config)] = {
		0xA8, 0x0E, 0x00,
		0x07, 0x00
	};
	memcpy(writeConfigCmd+3+2, config, sizeof(config));
	if (!Command(dev, writeConfigCmd, resp, 6))
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Failed to write config!");
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");

	/* Set Flash Address to 0 */
	uint8_t setAddressCmd[3+0x1E] = {
		0xA3, 0x1E, 0x00
	};
	if (!Command(dev, setAddressCmd, resp, 6))
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Failed to set base address!");
	if (resp[4] != keyChecksum)
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Key is invalid!");
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");

	/* Erase Flash Pages */
	uint8_t eraseKiB = std::max<uint8_t>(8, (firmwareFile.size() + 1023) / 1024);
	uint8_t u8EraseCmd[] = {
		0xA4, 0x01, 0x00, eraseKiB
	};
	if (!Command(dev, u8EraseCmd, resp, 6))
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Failed to erase flash!");
	if (resp[4] != 0 || resp[5] != 0)
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Could not erase flash!");
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");

	// Encode data
	for (int i = 0; i < firmwareFile.size(); ++i)
		firmwareFile[i] ^= u8Mask[i % 8];

	/* Write Block Command */
	uint8_t writeBlockCmd[64] = {
		0xA5, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		/* byte 4 Low Address (first = 1) */
		/* byte 5 High Address */
	};

	/* Write all blocks */
	const uint8_t BLOCK_LEN = 56;
	uint32_t blockCount = (firmwareFile.size()+BLOCK_LEN-1) / BLOCK_LEN;
	for (int b = 0; b < blockCount; ++b)
	{
		// Block
		uint32_t offset = b * BLOCK_LEN;
		uint8_t blockLen = std::min<uint32_t>(BLOCK_LEN, firmwareFile.size() - offset);
		// Write length and offset
		writeBlockCmd[1] = 3+2 + blockLen;
		*(uint32_t*)(writeBlockCmd+3) = offset;
		// Write data
		memmove(&writeBlockCmd[8], &firmwareFile[offset], blockLen);
		if (!Command(dev, writeBlockCmd, resp, 6))
			return ReportFirmwareUpdateError(updateStatus, asprintf_s("Bootloader: Failed to write block %d/%d of size %d to flash!", b, blockCount, blockLen));
		if (resp[4] != 0 || resp[5] != 0)
			return ReportFirmwareUpdateError(updateStatus, asprintf_s("Bootloader: Could not write to flash block %d at offset %d!", b, offset));

		auto status = updateStatus->contextualLock();
		status->progress = b*BLOCK_LEN;
		status->text = asprintf_s("Flashing block %d/%d!", b, blockCount);

		// Probably shouldn't allow aborting when flashing already started...
		//if (updateStatus->contextualRLock()->abort.stop_requested())
		//	return ReportFirmwareUpdateError(updateStatus, "Aborted!");
	}

	/* Write last Zero-Length-Block */
	writeBlockCmd[1] = 5;
	*(uint32_t*)(writeBlockCmd+3) = firmwareFile.size();
	if (!Command(dev, writeBlockCmd, resp, 6))
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Failed to write last Zero-Length-Block!");
	if (resp[4] != 0 || resp[5] != 0)
		return ReportFirmwareUpdateError(updateStatus, "Bootloader: Could not write last Zero-Length-Block!");
	if (updateStatus->contextualRLock()->abort.stop_requested())
		return ReportFirmwareUpdateError(updateStatus, "Aborted!");

	/* Reset and Run */
	uint8_t resetCmd[64] = {
		0xA2, 0x01, 0x00, 0x01 /* if 0x00 not run, 0x01 run*/
	};
	Command(dev, resetCmd, resp, 6);

	LOG(LFirmwareUpdate, LInfo, "Successfully flashed firmware file!");

	auto status = updateStatus->contextualLock();
	status->success = true;
	status->complete = true;
	status->text = "Successfully flashed firmware file!";
}