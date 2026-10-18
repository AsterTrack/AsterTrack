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

#include "info.hpp"
#include "mcu/mcu.hpp"
#include "comm/comm.hpp"
#include "camera/gcs.hpp"
#include "util/system.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstring>
#include <thread>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "version.h"

CameraID cameraID = 0;
bool isStored = false, isOverwritten = false;

// Version and build number
const VersionDesc sbcFWVersion(FW_MAJOR, FW_MINOR, FW_PATCH, FW_BUILD);

// Text descriptors
#if !defined(FIRMWARE_DESCRIPTOR) || !defined(FIRMWARE_COMPATIBILITY)
#error "Script did not prepare version file properly!"
#endif
const std::string sbcFWDescriptor = FIRMWARE_DESCRIPTOR;
const std::string sbcFWCompatibility = FIRMWARE_COMPATIBILITY;

CameraStoredInfo storedInfo;

int numCPUCores = 0;

std::string sbcModel, sbcSerialString;
uint32_t sbcRevisionCode, sbcSerialNumber;

CameraSBCHardware sbcHWDetection;
bool cameraSensorExists = false;
std::string cameraSensorID;

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

void loadCameraID(CameraID overrideID)
{
	if (overrideID != 0)
	{ // Passed in by argument
		cameraID = overrideID;
		isOverwritten = true;
		isStored = false;
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
			isOverwritten = false;
			printf("Read ID %u from config!\n", cameraID);
		}
		else
		{
			printf("ID config had invalid size %d!\n", size);
		}
	}
	
	if (cameraID == 0)
		isStored = isOverwritten = false;
	while (cameraID == 0)
		cameraID = rand();
}

void storeCameraID()
{
	std::ofstream id_stream(id_file, std::ios::binary);
	id_stream.write((char*)&cameraID, sizeof(CameraID));
}

static void updateInfo(CameraStoredInfo &info)
{
	info.sbcFWVersion = sbcFWVersion;
	info.sbcFWDescriptor = sbcFWDescriptor;
	info.sbcFWCompatibility = sbcFWCompatibility;
	info.sbcRevisionCode = sbcRevisionCode;
	info.sbcSerialNumber = sbcSerialNumber;

	info.sbcHWDetection = (CameraSBCHardware)(0
		| (mcu_exists? 0 : SBC_HW_NO_MCU)
		| (cameraSensorExists? 0 : SBC_HW_NO_SENSOR)
		| (!comms.medium[COMM_MEDIUM_UART] || comms.medium[COMM_MEDIUM_UART]->relyOnExternal? 0 : SBC_HW_UNRELIABLE_WIRED)
	);
	info.cameraSensor = cameraSensorID;
}

void gatherInfo(CameraID overrideID)
{
	gatherSBCInfo();
	loadCameraID(overrideID);

	// Detect camera sensor
	SENSOR_NUM sensor = gcs_probeSensor();
	if (sensor == SENSOR_ERROR)
	{
		printf("Failed to open camera I2C!\n");
		cameraSensorID = "Error";
	}
	else if (sensor == SENSOR_INVALID || sensor >= SENSOR_MAX)
	{
		printf("Failed to identify camera sensor: %d!\n", sensor);
		cameraSensorID = "Invalid";
	}
	else
	{
		cameraSensorExists = true;
		cameraSensorID = SENSOR_ID[sensor];
		printf("Detected camera sensor %s!\n", cameraSensorID.c_str());
	}

	storedInfo = {};
	updateInfo(storedInfo);
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
	if (!info.mcuFWCompatibility.empty())
		printf("    MCU FW Compatibility: %s\n", info.mcuFWCompatibility.c_str());

	// Update stored info
	updateInfo(info);
	storedInfo = std::move(info);

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
		if (isStored)
			printf("Have different ID stored, updating MCUs ID!\n");
		else if (isOverwritten)
			printf("Had different overwritten ID, updating MCUs ID!\n");
		else
		{ // Else adopt MCUs ID
			printf("Adopting MCUs ID #%u, replacing previous ID #%u!\n", config.cameraID, cameraID);
			cameraID = config.cameraID;
			storeCameraID();
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
	int sensorID = storedInfo.cameraSensor.size() & 0xFFFF;
	int sbcFWCompat = storedInfo.sbcFWCompatibility.size() & 0xFFFF;
	int mcuFWCompat = storedInfo.mcuFWCompatibility.size() & 0xFFFF;

	std::vector<uint8_t> infoPacket(CAMERA_INFO_BASE_LENGTH + sbcFWDesc + mcuFWDesc + mcuHWDesc + mcuSubparts + sensorID + sbcFWCompat + mcuFWCompat);
	const int headerLen = 8, data32Len = 40, strLensLen = 14;
	static_assert(CAMERA_INFO_BASE_LENGTH == headerLen+data32Len+strLensLen);

	infoPacket[0] = SBC_INFO_VERSION; // Packet version
	infoPacket[1] = storedInfo.mcuOTPVersion;
	infoPacket[2] = storedInfo.mcuHWDetection << 8;
	infoPacket[3] = storedInfo.mcuHWDetection & 0xFF;
	infoPacket[4] = storedInfo.sbcHWDetection << 8;
	infoPacket[5] = storedInfo.sbcHWDetection & 0xFF;
	infoPacket[6] = 0; // Unused
	infoPacket[7] = 0; // Unused
	static_assert(headerLen == 8);

	uint8_t *data32 = infoPacket.data()+headerLen;
	static_assert(sizeof(VersionDesc) == 4);
	static_assert(sizeof(HardwareSerial) == 12);
	memcpy(data32+0, &storedInfo.sbcFWVersion.num, 4);
	memcpy(data32+4, &storedInfo.mcuFWVersion.num, 4);
	memcpy(data32+8, &storedInfo.hardwareSerial.serial, 12);
	memcpy(data32+20, &storedInfo.mcuUniqueID, 12);
	memcpy(data32+32, &storedInfo.sbcRevisionCode, 4);
	memcpy(data32+36, &storedInfo.sbcSerialNumber, 4);
	static_assert(data32Len == 40);

	uint8_t *strLens = infoPacket.data()+headerLen+data32Len;
	strLens[0] = sbcFWDesc >> 8;
	strLens[1] = sbcFWDesc & 0xFF;
	strLens[2] = mcuFWDesc >> 8;
	strLens[3] = mcuFWDesc & 0xFF;
	strLens[4] = mcuHWDesc >> 8;
	strLens[5] = mcuHWDesc & 0xFF;
	strLens[6] = mcuSubparts >> 8;
	strLens[7] = mcuSubparts & 0xFF;
	strLens[8] = sensorID >> 8;
	strLens[9] = sensorID & 0xFF;
	strLens[10] = sbcFWCompat >> 8;
	strLens[11] = sbcFWCompat & 0xFF;
	strLens[12] = mcuFWCompat >> 8;
	strLens[13] = mcuFWCompat & 0xFF;
	static_assert(strLensLen == 14);

	uint8_t *strData = infoPacket.data()+headerLen+data32Len+strLensLen;
	memcpy(strData, storedInfo.sbcFWDescriptor.data(), sbcFWDesc);
	strData += sbcFWDesc;
	memcpy(strData, storedInfo.mcuFWDescriptor.data(), mcuFWDesc);
	strData += mcuFWDesc;
	memcpy(strData, storedInfo.mcuHWDescriptor.data(), mcuHWDesc);
	strData += mcuHWDesc;
	memcpy(strData, storedInfo.subpartSerials.data(), mcuSubparts);
	strData += mcuSubparts;
	memcpy(strData, storedInfo.cameraSensor.data(), sensorID);
	strData += sensorID;
	memcpy(strData, storedInfo.sbcFWCompatibility.data(), sbcFWCompat);
	strData += sbcFWCompat;
	memcpy(strData, storedInfo.mcuFWCompatibility.data(), mcuFWCompat);
	strData += mcuFWCompat;

	comm_send(medium, PacketHeader(PACKET_CAMERA_INFO, infoPacket.size()), std::move(infoPacket));
}
