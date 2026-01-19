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

#include "otp.h"
#include "rgbled.h"
#include "comm/commands.h"
#include "config_impl.h"

#if defined(STM32G0)
#include "stm32g030xx.h"
#endif

#include "compat.h"

uint8_t OTP_Version; // Assume OTP not written, disable parsing
uint8_t OTP_LenMainData, OTP_MaxSubParts, OTP_MaxHWString; // Lengths in 64Bit blocks
uint8_t OTP_HwStringData[100*8]; // Ensure >= OTP_MaxHWString
uint16_t OTP_HwStringLength;
uint8_t OTP_NumSubParts;

void otp_read()
{
	uint64_t OTP_HEADER = ((uint64_t)OTP[0] << 32) | OTP[1];
	if (OTP_HEADER != (uint64_t)-1)
	{ // OTP Header has been written
		OTP_Version = OTP_HEADER >> 56; // Version Format of OTP
		// Remaining 56bit header reserved for future format info
	}
	else OTP_Version = 0;

	switch (OTP_Version)
	{
		case 1:
			OTP_LenMainData = 8;	// Header, HW Serial, 5 Blocks Reserved
			OTP_MaxSubParts = 8;	// 8 64-Bit Serial Numbers for sub-parts
			OTP_MaxHWString = 100;	// 100 Blocks maximum for HW Descriptor String
			break;
		default: // Unsupported, should not happen, assume 0 sizes and don't parse
			OTP_LenMainData = 0;
			OTP_MaxSubParts = 0;
			OTP_MaxHWString = 0;
			break;
	}
	// assert(sizeof(OTP_HwStringData) >= OTP_MaxHWString*8);

	if (OTP_MaxHWString > 0)
	{
		// Parse Hardware Descriptor String made up of 64Bit-Blocks, with 8 characters each
		// There may be multiple strings written to OTP at different time points
		// E.g. to allow for product description, and documentation of hardware modifications done after the fact
		// So any of those strings NEED to be terminated by at least 1 byte of NULL (a full 64Bit of 0s if need be)
		// Upon encountering a block that has not yet been written, string is terminated again
		uint8_t otpTextStart = (OTP_LenMainData + OTP_MaxSubParts) * 2;
		OTP_HwStringLength = 0;
		for (int b = 0; b < OTP_MaxHWString; b++)
		{
			uint8_t otpPtr32 = otpTextStart + b*2;
			uint64_t block = ((uint64_t)OTP[otpPtr32 + 1] << 32) | OTP[otpPtr32 + 0];
			// Check if OTP is written - this should only fail if there is no string at all
			if (block == (uint64_t)-1) break;
			// Check chars for termination and copy one by one
			for (int c = 0; c < 8; c++)
			{
				if ((block & 0xFF) == 0)
				{ // Insert text separator
					OTP_HwStringData[OTP_HwStringLength++] = MCU_MULTI_TEXT_SEP;
					break;
				}
				OTP_HwStringData[OTP_HwStringLength++] = block & 0xFF;
				block >>= 8;
			}
		}
		// Clear trailing separator - but retain the separators between individual texts
		if (OTP_HwStringLength > 0 && OTP_HwStringLength < OTP_MaxHWString*8 && OTP_HwStringData[OTP_HwStringLength-1] == MCU_MULTI_TEXT_SEP)
			OTP_HwStringLength--;
	}
	else OTP_HwStringLength = 0;

	// These 64Bit-Blocks are optional sub-part IDs
	OTP_NumSubParts = 0;
	for (int s = 0; s < OTP_MaxSubParts; s++)
	{
		uint8_t otpIndex = (OTP_LenMainData + s) * 2;
		if (!(OTP[otpIndex+0] == (uint32_t)-1 && OTP[otpIndex+1] == (uint32_t)-1))
			OTP_NumSubParts = s+1; // Update count of explicitly specified sub-parts
	}
}

uint8_t otp_get_subparts(uint32_t *target)
{
	// These 64Bit-Blocks are optional sub-part IDs
	// Need to be continuous, their index is their identifier
	for (int s = 0; s < OTP_NumSubParts; s++)
	{
		uint8_t otpIndex = (OTP_LenMainData + s) * 2;
		// Write sub-part serial number (no bits set for explicitly unused sub-parts, all bits set for implicitly unused sub-parts)
		uint32_t a = OTP[otpIndex+0], b = OTP[otpIndex+1];
		memcpy(target + s*2 + 0, &a, sizeof(uint32_t));
		memcpy(target + s*2 + 1, &b, sizeof(uint32_t));
	}
}