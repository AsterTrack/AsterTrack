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
#include "comm/commands.h"

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
VersionDesc sbcFWVersion(0, 1, 0);

// Optional text descriptor
#ifndef FIRMWARE_DESCRIPTOR
#define FIRMWARE_DESCRIPTOR "dev"
#endif
std::string sbcFWDescriptor = FIRMWARE_DESCRIPTOR;

MCU_StoredInfo mcuStoredInfo;

int numCPUCores = 0;

std::string modelSBC, serialNumberSBC;

static void gatherSBCInfo()
{
	std::ifstream modelIS("/proc/device-tree/model");
	std::ifstream serialIS("/proc/device-tree/serial-number");

	std::stringstream ss;
	ss.clear();
	ss << modelIS.rdbuf();
	modelSBC = ss.str();
	ss.clear();
	ss << serialIS.rdbuf();
	serialNumberSBC = ss.str();

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
}

/* Returns whether MCU should be instructed to update its own stored ID */
bool receivedInfoFromMCU(MCU_StoredInfo &&info)
{
	printf("Received Camera Serial %.8x%.8x%.8x, Camera ID #%u, Camera MCU FW v.%d.%d.%d.%d\n",
		info.hardwareSerial.serial32[0], info.hardwareSerial.serial32[1], info.hardwareSerial.serial32[2], info.cameraID,
		info.mcuFWVersion.major, info.mcuFWVersion.minor, info.mcuFWVersion.patch, info.mcuFWVersion.build);
	printf("    Additionally received %d subpart serial numbers, and MCU Unique ID #%.8x%.8x%.8x\n",
		info.subpartSerials.size(), info.mcuUniqueID[0], info.mcuUniqueID[1], info.mcuUniqueID[2]);
	if (!info.mcuHWDescriptor.empty())
		printf("    MCU HW Descriptor: %s\n", info.mcuHWDescriptor.c_str());
	if (!info.mcuFWDescriptor.empty())
		printf("    MCU FW Descriptor: %s\n", info.mcuFWDescriptor.c_str());
	mcuStoredInfo = std::move(info);
	if (mcuStoredInfo.cameraID == 0 || mcuStoredInfo.cameraID == (CameraID)-1)
	{ // Update ID stored in MCU with current stored/generated
		printf("MCU had no ID stored, updating MCUs ID!\n");
		return true;
	}
	else if (cameraID != mcuStoredInfo.cameraID)
	{ // IDs differ, perhaps SD card got reflashed / exchanged
		if (isStored)
			printf("Have different ID stored, updating MCUs ID!\n");
		else if (isOverwritten)
			printf("Had different overwritten ID, updating MCUs ID!\n");
		else
		{ // Else adopt MCUs ID
			printf("Adopting MCUs ID!\n");
			cameraID = mcuStoredInfo.cameraID;
			std::ofstream id_stream(id_file, std::ios::binary);
			id_stream.write((char*)&cameraID, sizeof(CameraID));
			return false; // Don't update MCU ID
		}
		return true; // Update MCU ID
	}
	return false;
}