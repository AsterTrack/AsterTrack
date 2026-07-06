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

#ifndef OTP_H
#define OTP_H

#include <stdint.h>

#define OTP_HW_STRING_PREPEND	9   // max(MCU_MAX_LEADING_BYTES + 16bit size, UART_PRE_OVERHEAD_SEND + 8bit request)
#define OTP_HW_STRING_APPEND	9   // UART_POST_OVERHEAD_SEND
#define MAX_HW_STRING_BLOCKS	100
#define MAX_HW_STRING_SIZE		(MAX_HW_STRING_BLOCKS*sizeof(uint64_t))
#define HW_STRING_BUFFER_SIZE	(OTP_HW_STRING_PREPEND + OTP_HW_STRING_APPEND + MAX_HW_STRING_SIZE)
#define MAX_SUBPART_BLOCKS		8
#define MAX_SUBPART_SIZE		(MAX_SUBPART_BLOCKS*sizeof(uint64_t))

extern uint8_t OTP_Version;
extern uint8_t OTP_HwStringData[HW_STRING_BUFFER_SIZE];
extern uint16_t OTP_HwStringLength;
extern uint8_t OTP_NumSubParts, OTP_NumHWStringBlocks;

bool otp_read();
void otp_get_subparts(uint32_t *target);

uint16_t otp_program_main(uint32_t *mainData, uint16_t length);
uint16_t otp_set_subparts(uint32_t *subparts, uint16_t length);
uint16_t otp_append_hw_string(uint32_t *string, uint16_t length);

#endif // OTP_H