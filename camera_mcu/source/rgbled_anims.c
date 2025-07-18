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

 #include "rgbled.h"

/**
 * LED States
 * Button Order:
 * 1 2
 * 4 3
 */

#define COL_STANDBY 0x22, 0x55, 0x00

uint8_t LED_STANDBY[RGBLED_COUNT*3] = {
	COL_STANDBY,
	COL_STANDBY,
	COL_STANDBY,
	COL_STANDBY,
};

#define COL_ACTIVE 0x55, 0xBB, 0x00

uint8_t LED_ACTIVE[RGBLED_COUNT*3] = {
	COL_ACTIVE,
	COL_ACTIVE,
	COL_ACTIVE,
	COL_ACTIVE,
};

uint8_t LED_ALL_OFF[RGBLED_COUNT*3] = {
	0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
};

#define COL_INFRARED_ON 0xFF, 0x00, 0xFF
#define COL_INFRARED_OFF 0x2A, 0x00, 0x2A
#define COL_VISIBLE_ON 0x00, 0xFF, 0x88
#define COL_VISIBLE_OFF 0x00, 0x33, 0x22

uint8_t LED_FILTER_INFRARED[RGBLED_COUNT*3] = {
	COL_INFRARED_ON,
	COL_INFRARED_ON,
	COL_VISIBLE_OFF,
	COL_VISIBLE_OFF,
};

uint8_t LED_FILTER_VISIBLE[RGBLED_COUNT*3] = {
	COL_INFRARED_OFF,
	COL_INFRARED_OFF,
	COL_VISIBLE_ON,
	COL_VISIBLE_ON,
};