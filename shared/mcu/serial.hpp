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

#ifndef SERIAL_H
#define SERIAL_H

#include <cstdint>


union HardwareSerial
{
	enum class Class : uint8_t
    {
		Controller,
		Camera,
		Tracker
    };

    enum class Type : uint8_t
    {
		Official,	// manufacturer signifies production facility
		ThirdParty,	// manufacturer signifies company sign
		DIY_Reg_1,	// manufacturer signifies registered seller ID
		DIY_Free_1,	// manufacturer signifies DIY sign (free-for-all)
    };

	struct Camera
	{
		enum class Interface : uint8_t
		{
			RS422_WIFI_NRF24 = 0x1
		};
		enum class Sensor : uint8_t
		{
			OV9281 = 0x1
		};
		enum class Lens : uint8_t
		{
			HSW_9D4024 = 0x1
		};
		enum class Filter : uint8_t
		{ // For simplicity, a specific number implies pass-filter
			Filter_None = 0x1,
			Filter_IRCut = 0x2,
			Filter_850nm = 0x3,
			Switched_850nm_IRCut = 0x4
		};
	};

	struct Controller
	{
		enum class Interface : uint8_t
		{
			USB2HS_RS422_NRF24
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
		{ // Mainly for configurations external to the PCB itself
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
		uint32_t ID;				// Unique within run, centrally allocated, to make it harder to fake serial numbers
	};
};

#endif // SERIAL_H