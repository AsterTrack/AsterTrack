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

#include "serial.hpp"
#include "comm/commands.h"

#include "util/util.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

uint32_t parseMCUInfoPacket(CameraStoredInfo &info, CameraStoredConfig &config, const uint8_t *packet, uint32_t length)
{ // Packet received by MCU_FETCH_INFO either via I2C or PACKET_READ_MCU_INFO
	uint8_t version = packet[0]; // Received info packet version
	if (version != MCU_INFO_VERSION)
	{ // Keep support for older versions in here!
		printf("Fetch info packet version %d is unsupported, expected %d!\n", version, MCU_INFO_VERSION);
		return 0;
	}
	uint32_t versionedSize = 8+4+4+12+12;
	if (length < versionedSize) return 0;

	info.mcuOTPVersion = packet[1]; // Should not concern us too much, but may be of interest in interpreting the data
	info.mcuHWDetection = (CameraHWDetection)packet[3];

	info.subpartSerials.resize(packet[2]);
	info.mcuHWDescriptor.resize((packet[4] << 8) | packet[5]);
	info.mcuFWDescriptor.resize((packet[6] << 8) | packet[7]);

	const uint8_t *ptr = packet+8;
	memcpy(&config.cameraID, ptr, sizeof(CameraID));
	ptr += sizeof(CameraID);
	memcpy(&info.mcuFWVersion, ptr, sizeof(VersionDesc));
	ptr += sizeof(VersionDesc);
	memcpy(&info.hardwareSerial, ptr, sizeof(HardwareSerial));
	ptr += sizeof(HardwareSerial);
	memcpy(&info.mcuUniqueID, ptr, 3*sizeof(uint32_t));
	ptr += 3*sizeof(uint32_t);

	return versionedSize;
}

std::string describeHardwareSerial(HardwareSerial hwSerial)
{
	return asprintf_s("%.1X %.1X %.2X %.2X %.2X %.4X %.2X %.2X %.8X",
		(uint8_t)hwSerial.header, (uint8_t)hwSerial.type, hwSerial.manufacturer, hwSerial.product, hwSerial.revision,
		hwSerial.config, hwSerial.batch, hwSerial.run, hwSerial.ID);
}

std::vector<std::string> describeCameraInfo(const CameraStoredInfo &info)
{
	std::vector<std::string> desc;
	bool sbc = info.sbcFWVersion.num != 0, mcu = info.mcuFWVersion.num != 0;
	if (!sbc && !mcu) return desc;
	if (sbc)
		desc.push_back(asprintf_s("SBC Firmware v%d.%d.%d (Build %.2x - %s)",
			info.sbcFWVersion.major, info.sbcFWVersion.minor, info.sbcFWVersion.patch, info.sbcFWVersion.build, info.sbcFWDescriptor.c_str()));
	else desc.push_back("No info on SBC.");
	if (mcu)
		desc.push_back(asprintf_s("MCU Firmware v%d.%d.%d (Build %.2x - %s)",
			info.mcuFWVersion.major, info.mcuFWVersion.minor, info.mcuFWVersion.patch, info.mcuFWVersion.build, info.mcuFWDescriptor.c_str()));
	else desc.push_back("No info on MCU.");
	if (sbc)
	{
		desc.push_back(asprintf_s("SBC: %s", info.sbcHWDescriptor.c_str()));
		desc.push_back(asprintf_s("SBC Revision Code %x, Serial Number: %.8x",
			info.sbcRevisionCode & 0xFFFFFF, info.sbcSerialNumber));
	}
	if (mcu && !info.mcuHWDescriptorParts.empty())
	{
		desc.push_back(asprintf_s("Hardware Descriptor: %s", info.mcuHWDescriptorParts.front().c_str()));
		for (int i = 1; i < info.mcuHWDescriptorParts.size(); i++)
			desc.push_back(asprintf_s("    Appended: %s", info.mcuHWDescriptorParts[i].c_str()));
	}
	if (mcu)
	{
	#define BYTE_BIT(byte,bit) ((byte & (1 << bit)) ? '1' : '0')
	#define BYTE_BIT_LIST(byte) BYTE_BIT(byte,7), BYTE_BIT(byte,6), BYTE_BIT(byte,5), BYTE_BIT(byte,4), BYTE_BIT(byte,3), BYTE_BIT(byte,2), BYTE_BIT(byte,1), BYTE_BIT(byte,0)
		desc.push_back(asprintf_s("Detected Hardware Bitmask: %c%c%c%c%c%c%c%c", BYTE_BIT_LIST(info.mcuHWDetection)));
		desc.push_back(asprintf_s("Hardware Serial Number: %s", describeHardwareSerial(info.hardwareSerial).c_str()));
		for (int i = 0; i < info.subpartSerials.size(); i++)
			//if (info.subpartSerials[i] != (uint64_t)-1)
			desc.push_back(asprintf_s("    Subpart %d: %.8x%.8x", i, (uint32_t)(info.subpartSerials[i] >> 32), (uint32_t)info.subpartSerials[i]));
		desc.push_back(asprintf_s("MCU Unique ID: %.8x%.8x%.8x",
			info.mcuUniqueID[0], info.mcuUniqueID[1], info.mcuUniqueID[2]));
	}
	return desc;
}
