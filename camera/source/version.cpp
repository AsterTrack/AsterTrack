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

#include "version.hpp"
#include "mcu/mcu.hpp"
#include "comm/comm.hpp"
#include "util/system.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstring>
#include <thread>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

CameraID cameraID = 0;
bool isStored = false, isOverwritten = false;
std::string id_file = "/mnt/mmcblk0p2/config/id";

// Version and build number
const VersionDesc sbcFWVersion(0, 1, 0);

// Optional text descriptor
#ifndef FIRMWARE_DESCRIPTOR
#define FIRMWARE_DESCRIPTOR "dev"
#endif
const std::string sbcFWDescriptor = FIRMWARE_DESCRIPTOR;

CameraStoredInfo storedInfo;

int numCPUCores = 0;

std::string sbcModel, sbcSerialString;
uint32_t sbcRevisionCode, sbcSerialNumber;

static void gatherSBCInfo()
{
	std::ifstream modelIS("/proc/device-tree/model");
	std::ifstream serialIS("/proc/device-tree/serial-number");
	std::string revision = exec("cat /proc/cpuinfo | awk '/Revision/ {print $3}'");

	std::stringstream ssM;
	ssM << modelIS.rdbuf();
	sbcModel = ssM.str();
	std::stringstream ssS;
	ssS << serialIS.rdbuf();
	sbcSerialString = ssS.str();
	if (sbcSerialString.size() == 17)
		sbcSerialNumber = std::stoul(&sbcSerialString[8], nullptr, 16);
	else if (sbcSerialString.size() == 9) // Just in case
		sbcSerialNumber = std::stoul(sbcSerialString, nullptr, 16);
	else sbcSerialNumber = 0;
	sbcRevisionCode = std::stoul(revision, nullptr, 16);
	printf("Model %s, Revision: %x, Serial Number %.8x\n", sbcModel.c_str(), sbcRevisionCode, sbcSerialNumber);
	if (sbcModel.find("Raspberry") != std::string::npos)
		sbcRevisionCode |= 0xFF << 24; // Mark it as Raspberry Pi

	numCPUCores = std::thread::hardware_concurrency();
}

static void loadCameraID(CameraID overrideID)
{
	if (overrideID != 0)
	{ // Passed in by argument
		cameraID = overrideID;
		isOverwritten = true;
	}
	else if (std::filesystem::exists(id_file))
	{ // Read from SD card storage
		std::ifstream idFS(id_file, std::ios::in | std::ios::binary);
		idFS.seekg(0, std::ios::end);
		std::size_t size = idFS.tellg();
		if (size == 4)
		{
			idFS.seekg(0);
			idFS.read((char*)&cameraID, 4);
			isStored = true;
			printf("Read ID %u from config!\n", cameraID);
		}
		else
		{
			printf("ID config had invalid size %d!\n", size);
		}
	}
	
	if (cameraID == 0)
		cameraID = rand();
}

void gatherInfo(CameraID overrideID)
{
	gatherSBCInfo();
	loadCameraID(overrideID);
	storedInfo.sbcFWVersion = sbcFWVersion;
	storedInfo.sbcFWDescriptor = sbcFWDescriptor;
	storedInfo.sbcRevisionCode = sbcRevisionCode;
	storedInfo.sbcSerialNumber = sbcSerialNumber;
}

void receivedInfoFromMCU(CameraStoredInfo &&info)
{
	printf("Received Camera Serial %.8x%.8x%.8x, Camera MCU FW v.%d.%d.%d.%d\n",
		info.hardwareSerial.serial32[0], info.hardwareSerial.serial32[1], info.hardwareSerial.serial32[2],
		info.mcuFWVersion.major, info.mcuFWVersion.minor, info.mcuFWVersion.patch, info.mcuFWVersion.build);
	printf("    Additionally received %d subpart serial numbers, and MCU Unique ID #%.8x%.8x%.8x\n",
		info.subpartSerials.size(), info.mcuUniqueID[0], info.mcuUniqueID[1], info.mcuUniqueID[2]);
	if (!info.mcuHWDescriptor.empty())
		printf("    MCU HW Descriptor: %s\n", info.mcuHWDescriptor.c_str());
	if (!info.mcuFWDescriptor.empty())
		printf("    MCU FW Descriptor: %s\n", info.mcuFWDescriptor.c_str());

	// Update stored info
	storedInfo = std::move(info);
	storedInfo.sbcFWVersion = sbcFWVersion;
	storedInfo.sbcFWDescriptor = sbcFWDescriptor;
	storedInfo.sbcRevisionCode = sbcRevisionCode;
	storedInfo.sbcSerialNumber = sbcSerialNumber;

	sendInfoPacket(largeDataAff);
}

/* Returns whether MCU should be instructed to update its own stored ID */
bool receivedConfigFromMCU(CameraStoredConfig &&config)
{
	if (config.cameraID == 0 || config.cameraID == (CameraID)-1)
	{ // Update ID stored in MCU with current stored/generated
		printf("MCU had no ID stored, updating MCUs ID!\n");
		return true;
	}
	else if (cameraID != config.cameraID)
	{ // IDs differ, perhaps SD card got reflashed / exchanged
		/* if (isStored)
			printf("Have different ID stored, updating MCUs ID!\n");
		else if (isOverwritten)
			printf("Had different overwritten ID, updating MCUs ID!\n");
		else */
		// Adopting MCUs ID makes things easier for the server/controller, which already got the ID from the MCU
		{ // Else adopt MCUs ID
			printf("Adopting MCUs ID #%u, replacing previous ID #%u!\n", config.cameraID, cameraID);
			cameraID = config.cameraID;
			std::ofstream id_stream(id_file, std::ios::binary);
			id_stream.write((char*)&cameraID, sizeof(CameraID));
			return false; // Don't update MCU ID
		}
		return true; // Update MCU ID
	}
	return false;
}

void sendInfoPacket(CommMedium medium)
{
	int sbcFWDesc = storedInfo.sbcFWDescriptor.size() & 0xFFFF;
	int mcuFWDesc = storedInfo.mcuFWDescriptor.size() & 0xFFFF;
	int mcuHWDesc = storedInfo.mcuHWDescriptor.size() & 0xFFFF;
	int mcuSubparts = storedInfo.subpartSerials.size()*sizeof(uint64_t);

	std::vector<uint8_t> infoPacket(CAMERA_INFO_BASE_LENGTH + sbcFWDesc + mcuFWDesc + mcuHWDesc + mcuSubparts);
	static_assert(CAMERA_INFO_BASE_LENGTH == 52);

	infoPacket[0] = 1; // Packet version
	infoPacket[1] = storedInfo.mcuOTPVersion;
	infoPacket[2] = 0; // Reserved
	infoPacket[3] = 0; // Reserved

	static_assert(sizeof(VersionDesc) == 4);
	static_assert(sizeof(HardwareSerial) == 12);
	memcpy(&infoPacket[4], &storedInfo.sbcFWVersion.num, 4);
	memcpy(&infoPacket[8], &storedInfo.mcuFWVersion.num, 4);
	memcpy(&infoPacket[12], &storedInfo.hardwareSerial.serial, 12);
	memcpy(&infoPacket[24], &storedInfo.mcuUniqueID, 12);
	memcpy(&infoPacket[36], &storedInfo.sbcRevisionCode, 4);
	memcpy(&infoPacket[40], &storedInfo.sbcSerialNumber, 4);

	infoPacket[44] = sbcFWDesc >> 8;
	infoPacket[45] = sbcFWDesc & 0xFF;
	infoPacket[46] = mcuFWDesc >> 8;
	infoPacket[47] = mcuFWDesc & 0xFF;
	infoPacket[48] = mcuHWDesc >> 8;
	infoPacket[49] = mcuHWDesc & 0xFF;
	infoPacket[50] = mcuSubparts >> 8;
	infoPacket[51] = mcuSubparts & 0xFF;

	uint8_t *ptr = infoPacket.data() + CAMERA_INFO_BASE_LENGTH;
	memcpy(ptr, storedInfo.sbcFWDescriptor.data(), sbcFWDesc);
	ptr += sbcFWDesc;
	memcpy(ptr, storedInfo.mcuFWDescriptor.data(), mcuFWDesc);
	ptr += mcuFWDesc;
	memcpy(ptr, storedInfo.mcuHWDescriptor.data(), mcuHWDesc);
	ptr += mcuHWDesc;
	memcpy(ptr, storedInfo.subpartSerials.data(), mcuSubparts);
	ptr += mcuSubparts;

	comm_send(medium, PacketHeader(PACKET_CAMERA_INFO, infoPacket.size()), std::move(infoPacket));
}
