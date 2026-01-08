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

extern uint8_t OTP_Version;
extern uint8_t OTP_HwStringData[];
extern uint16_t OTP_HwStringLength;
extern uint8_t OTP_NumSubParts;

void otp_read();
uint8_t otp_get_subparts(uint32_t *target, uint16_t maxCount);

#endif // OTP_H