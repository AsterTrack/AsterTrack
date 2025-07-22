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


#define COL_FLASH_DEBUG_SWD 0x00, 0x11, 0xFF
#define COL_FLASH_BOOT0_PI 0x00, 0xFF, 0x11

uint8_t LED_FLASH_DEBUG_SWD[RGBLED_COUNT*3] = {
	COL_FLASH_DEBUG_SWD,
	COL_FLASH_DEBUG_SWD,
	COL_FLASH_DEBUG_SWD,
	COL_FLASH_DEBUG_SWD,
};

uint8_t LED_FLASH_BOOT0_PI[RGBLED_COUNT*3] = {
	COL_FLASH_BOOT0_PI,
	COL_FLASH_BOOT0_PI,
	COL_FLASH_BOOT0_PI,
	COL_FLASH_BOOT0_PI,
};

#define COL_BOOTLOADER 0xFF, 0x00, 0x55

uint8_t LED_BOOTLOADER[RGBLED_COUNT*3] = {
	COL_BOOTLOADER,
	COL_BOOTLOADER,
	COL_BOOTLOADER,
	COL_BOOTLOADER,
};

#define COL_FLASH_BAD_1 0xFF, 0x00, 0x00
#define COL_FLASH_BAD_2 0x33, 0x11, 0x00

uint8_t LED_FLASH_BAD_PHASE_1[RGBLED_COUNT*3] = {
	COL_FLASH_BAD_1,
	COL_FLASH_BAD_1,
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_2,
};

uint8_t LED_FLASH_BAD_PHASE_2[RGBLED_COUNT*3] = {
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_1,
	COL_FLASH_BAD_1,
};

struct LED_Animation LED_ANIM_FLASH_BAD = {
	.count = 4,
	.repetitions = -1,
	.transitions = {
		{
			.leds = LED_FLASH_BAD_PHASE_1,
			.time = 300*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		},
		{
			.leds = LED_FLASH_BAD_PHASE_1,
			.time = 200*TICKS_PER_MS,
			.mode = INTER_IMMEDIATE
		},
		{
			.leds = LED_FLASH_BAD_PHASE_2,
			.time = 300*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		},
		{
			.leds = LED_FLASH_BAD_PHASE_2,
			.time = 200*TICKS_PER_MS,
			.mode = INTER_IMMEDIATE
		}
	}
};

uint8_t LED_FLASH_CHARGE_INIT[RGBLED_COUNT*3] = {
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_2,
};

uint8_t LED_FLASH_CHARGE_TOP_BOOT0_PI[RGBLED_COUNT*3] = {
	COL_FLASH_BOOT0_PI,
	COL_FLASH_BOOT0_PI,
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_2,
};

uint8_t LED_FLASH_CHARGE_BOT_DEBUG_SWD[RGBLED_COUNT*3] = {
	COL_FLASH_BAD_2,
	COL_FLASH_BAD_2,
	COL_FLASH_DEBUG_SWD,
	COL_FLASH_DEBUG_SWD,
};

uint8_t LED_FLASH_CHARGE_FAIL[RGBLED_COUNT*3] = {
	COL_FLASH_BAD_1,
	COL_FLASH_BAD_1,
	COL_FLASH_BAD_1,
	COL_FLASH_BAD_1,
};

struct LED_Animation LED_ANIM_FLASH_CHARGE_TOP_BOOT0_PI = {
	.count = 4,
	.repetitions = 1,
	.transitions = {
		{
			.leds = LED_FLASH_CHARGE_INIT,
			.time = 100*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		},
		{
			.leds = LED_FLASH_CHARGE_TOP_BOOT0_PI,
			.time = (CHARGE_TIME_MIN_MS-100)*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		},
		{
			.leds = LED_FLASH_CHARGE_TOP_BOOT0_PI,
			.time = (CHARGE_TIME_MAX_MS-CHARGE_TIME_MIN_MS)/2*TICKS_PER_MS,
			.mode = INTER_IMMEDIATE
		},
		{
			.leds = LED_FLASH_CHARGE_FAIL,
			.time = (CHARGE_TIME_MAX_MS-CHARGE_TIME_MIN_MS)*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		}
	}
};

struct LED_Animation LED_ANIM_FLASH_CHARGE_BOT_DEBUG_SWD = {
	.count = 4,
	.repetitions = 1,
	.transitions = {
		{
			.leds = LED_FLASH_CHARGE_INIT,
			.time = 100*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		},
		{
			.leds = LED_FLASH_CHARGE_BOT_DEBUG_SWD,
			.time = (CHARGE_TIME_MIN_MS-100)*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		},
		{
			.leds = LED_FLASH_CHARGE_BOT_DEBUG_SWD,
			.time = (CHARGE_TIME_MAX_MS-CHARGE_TIME_MIN_MS)/2*TICKS_PER_MS,
			.mode = INTER_IMMEDIATE
		},
		{
			.leds = LED_FLASH_CHARGE_FAIL,
			.time = (CHARGE_TIME_MAX_MS-CHARGE_TIME_MIN_MS)*TICKS_PER_MS,
			.mode = INTER_LERP_LINEAR
		}
	}
};
