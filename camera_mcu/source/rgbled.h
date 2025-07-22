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

#ifndef __RGBLED_H
#define __RGBLED_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "stdbool.h"

#include "util.h"

#define RGBLED_COUNT 4

// Update interval to use for smooth animations and transitions
#define RGB_UPDATE_INTERVAL_MS 10

enum LED_INTERPOLATION {
	INTER_UNDEFINED = 0,
	INTER_IMMEDIATE,
	INTER_LERP_LINEAR
};

struct LED_Transition
{
	uint8_t *leds;
	TimeSpan time;
	enum LED_INTERPOLATION mode;
};

struct LED_Animation
{
	int repetitions;
	uint8_t count;
	struct LED_Transition transitions[];
};

extern float brightness;

void rgbled_init();

// Start a transition to the given state, overwriting any current animation
void rgbled_transition(uint8_t rgb[RGBLED_COUNT*3], int timeMS);

// Check if rgbled is currently transitioning to a static state (not animating)
bool rgbled_transitioning();

// Animate the following animation (until repetitions are reached or until overwritten by transition or animation)
void rgbled_animation(struct LED_Animation *anim);

// Check if rgbled is currently animating. If anim != NULL, will check for the given animation specifically
bool rgbled_animating(struct LED_Animation *anim);


/**
 * LED States & Animations
 */

extern uint8_t LED_STANDBY[RGBLED_COUNT*3];
extern uint8_t LED_ACTIVE[RGBLED_COUNT*3];
extern uint8_t LED_ALL_OFF[RGBLED_COUNT*3];
extern uint8_t LED_FILTER_INFRARED[RGBLED_COUNT*3];
extern uint8_t LED_FILTER_VISIBLE[RGBLED_COUNT*3];
extern uint8_t LED_FLASH_DEBUG_SWD[RGBLED_COUNT*3];
extern uint8_t LED_FLASH_BOOT0_PI[RGBLED_COUNT*3];
extern uint8_t LED_BOOTLOADER[RGBLED_COUNT*3];

extern struct LED_Animation LED_ANIM_FLASH_BAD;

// These timings are critical as they influence behaviour and the animation needs to convey that behaviour to the user
#define CHARGE_TIME_MIN_MS 1000
#define CHARGE_TIME_MAX_MS 3000
extern struct LED_Animation LED_ANIM_FLASH_CHARGE_BOT_DEBUG_SWD;
extern struct LED_Animation LED_ANIM_FLASH_CHARGE_TOP_BOOT0_PI;

#ifdef __cplusplus
}
#endif

#endif /* __RGBLED_H */
