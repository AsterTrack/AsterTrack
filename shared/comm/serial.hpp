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

#ifndef SERIAL_H
#define SERIAL_H

#include "comm/packet.h"

#include <cstdint>
#include <string>
#include <vector>


#define SBC_INFO_VERSION		4

union HardwareSerial
{
	enum class Class : uint8_t
    {
		Controller,
		SyncInput,
		SyncBeacon,
		Camera,
		Max = 15
    };

    enum class Type : uint8_t
    {
		Official,	// manufacturer signifies production facility
		ThirdParty,	// manufacturer signifies company sign
		DIY_Reg_1,	// manufacturer signifies registered seller ID
		DIY_Free_1,	// manufacturer signifies DIY sign (free-for-all)
		Max = 15
    };

	struct Camera
	{
		enum class Interface : uint8_t
		{
			RS422_WIFI				= 0x1,
			RS422_WIFI_NRF24		= 0x2,
			Max						= 0xF
		};
		enum class Sensor : uint8_t
		{
			OV9281					= 0x1,
			Max						= 0xF
		};
		enum class Lens : uint8_t
		{
			HSW_9D4024				= 0x1,
			Max						= 0xF
		};
		enum class Filter : uint8_t
		{ // For simplicity, a specific number implies pass-filter
			Filter_None				= 0x1,
			Filter_IRCut			= 0x2,
			Filter_850nm			= 0x3,
			Switched_850nm_IRCut	= 0x4,
			Max						= 0xF
		};
	};

	struct Controller
	{
		enum class Interface : uint8_t
		{
			USB2HS_RS422			= 0x1,
			USB2HS_RS422_NRF24		= 0x2,
			Max						= 0xF
		};
	};

	uint8_t serial[12];
	uint32_t serial32[3];
	struct
	{
		Class header : 4;
		// Manufacturer:
		Type type : 4;
		uint8_t manufacturer;
		// Product:
		uint8_t product;			// Different product versions / major iterations
		uint8_t revision;			// Revision of a specific product, may include fixes or minor feature additions, but not change the product itself
		union
		{ // Mainly for (initial) configurations external to the PCB itself
			uint16_t config;
			struct
			{
				Camera::Interface interface : 4;
				Camera::Sensor sensor : 4;
				Camera::Lens lens : 4;
				Camera::Filter filter : 4;
			} camera;
			struct
			{
				Controller::Interface interface : 4;
				uint8_t ports : 4;
				uint8_t resv_1 : 4;
				uint8_t resv_2 : 4;
			} controller;
		};
		// Production:
		uint8_t batch; 				// Different Batches of product (per production facility)
		uint8_t run;				// Different shifts/people, different batch of components, etc. - basically increase whenever a major factor changes
		uint32_t ID;				// Unique within run, random and centrally allocated
	};
};

enum CameraSBCHardware : uint16_t
{
	SBC_HW_NO_MCU				= 1 << 0,
	SBC_HW_NO_SENSOR			= 1 << 1,
	SBC_HW_UNRELIABLE_WIRED		= 1 << 2,
};

struct CameraStoredInfo
{
	VersionDesc sbcFWVersion;			// SBC Firmware Version, embedded in firmware
	std::string sbcFWDescriptor;		// SBC Firmware Descriptor, embedded in firmware
	std::string sbcFWCompatibility;		// SBC Firmware Compatibility, embedded in firmware
	VersionDesc mcuFWVersion;			// MCU Firmware Version, embedded in firmware
	std::string mcuFWDescriptor;		// MCU Firmware Descriptor, embedded in firmware
	std::string mcuFWCompatibility;		// MCU Firmware Compatibility, embedded in firmware

	uint8_t mcuOTPVersion;				// Indicates revision in how OTP is formatted, potentially even affecting the format of the following members
	HardwareSerial hardwareSerial = {};	// Camera Serial Number (96Bit) stored in MCU OTP, bound to hardware & production
    std::vector<uint64_t> subpartSerials; // Optional serial numbers of subparts
	std::string mcuHWDescriptor;		// MCU Hardware Descriptor, written to OTP, may be composed of successively written texts separated by MCU_MULTI_TEXT_SEP

	CameraMCUHardware mcuHWDetection;	// Flags of features or defects detected by MCU, not necessarily encoded in OTP
	CameraSBCHardware sbcHWDetection;	// Flags of features or defects detected by SBC
	std::string cameraSensor;			// ID of camera sensor detected, if any

	uint32_t sbcRevisionCode;			// SBC Revision Code identifying Raspberry Pi Model
	uint32_t sbcSerialNumber;			// SBC Serial Number, 32Bit for Raspberry Pi
	uint32_t mcuUniqueID[3];			// MCU Chip Unique ID, 96Bit for STM32

	std::string sbcHWDescriptor;		// Inferred from sbcRevisionCode, NOT SENT
	std::vector<std::string> mcuHWDescriptorParts; // Split mcuHWDescriptor, NOT SENT
};

struct CameraStoredConfig
{
	CameraID cameraID; // Camera ID (32bit), stored in MCU Flash, used for unique ID in software
	// May add further config here, stored either on the MCU Flash or in the SBC storage
};


uint32_t parseMCUInfoPacket(CameraStoredInfo &info, CameraStoredConfig &config, const uint8_t *packet, uint32_t length);
std::string describeHardwareSerial(HardwareSerial hwSerial);
std::vector<std::string> describeCameraInfo(const CameraStoredInfo &info);

#endif // SERIAL_H