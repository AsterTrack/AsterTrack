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
#include "config_impl.h"
#include "util.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

float brightness = 0.1f;

static uint8_t buffer_source[RGBLED_COUNT*3];
static uint8_t buffer_latest[RGBLED_COUNT*3];
static uint8_t buffer_target[RGBLED_COUNT*3];

struct LED_Animation * volatile current_animation = NULL;
volatile int anim_transition;
volatile int anim_repetitions;

volatile enum LED_INTERPOLATION transition = INTER_UNDEFINED;
volatile TimePoint transition_start, transition_end;


/* Driver Functions */

void rgbled_init_driver();

bool rgbled_set(uint8_t rgb[RGBLED_COUNT*3]);

bool rgbled_set_and_delay(uint8_t rgb[RGBLED_COUNT*3], TimePoint earliest);

bool rgbled_ready();

bool rgbled_abort_wakeup_waiting();

void rgbled_lock();
void rgbled_unlock();


/* Functions */

void rgbled_ready_callback();

void rgbled_init()
{
	rgbled_init_driver();
}

static void copy(uint8_t *source, uint8_t *output)
{
	for (int i = 0; i < RGBLED_COUNT*3; i++)
		output[i] = source[i];
}

static void process(uint8_t *source, uint8_t *output, float factor)
{
	for (int i = 0; i < RGBLED_COUNT*3; i++)
		output[i] = (int)((float)source[i] * factor);
}

static void lerp_process(uint8_t *source, uint8_t *target, uint8_t *output, float value, float factor)
{
	if (value > 1) value = 1;
	if (value < 0) value = 0;
	// Some optimisation to ensure -O0 doesn't take even longer
	float fac1 = (1-value) * factor;
	float fac2 = value * factor;
	for (int i = 0; i < RGBLED_COUNT*3; i++)
		output[i] = (int)((float)source[i] * fac1 + (float)target[i] * fac2);
	// TODO: These 26 float multiplications take ~135us in total (~5us per FLOP)
}

static float get_lerp()
{
	return (float)(GetTimePoint()-transition_start) / (float)(transition_end-transition_start);
}

static void pickup_from_transition(enum LED_INTERPOLATION prevTransition)
{
	if (prevTransition == INTER_LERP_LINEAR)  // Interrupting a lerp, resample at current time
		lerp_process(buffer_source, buffer_target, buffer_source, get_lerp(), 1);
	else // Start lerp from latest
		copy(buffer_target, buffer_source);
}

void rgbled_transition(uint8_t rgb[RGBLED_COUNT*3], int timeMS)
{
	rgbled_lock();

	// Reset state
	enum LED_INTERPOLATION prevTransition = transition;
	current_animation = NULL;
	transition = INTER_UNDEFINED;

	if (timeMS == 0)
	{ // Switch immediately
		copy(rgb, buffer_target);
		transition_start = transition_end = GetTimePoint();
		transition = INTER_IMMEDIATE;
	}
	else
	{ // Transition over time
		// Set source for transition as current (potentially interrupted) state
		pickup_from_transition(prevTransition);
		copy(rgb, buffer_target);
		transition_start = GetTimePoint();
		transition_end = transition_start + timeMS*TICKS_PER_MS;
		transition = INTER_LERP_LINEAR;
	}

	if (rgbled_ready() || rgbled_abort_wakeup_waiting())
		rgbled_ready_callback(); // Can set immediately

	rgbled_unlock();
}

bool rgbled_transitioning()
{
	return current_animation == NULL && transition != INTER_UNDEFINED;
}

static void set_anim_transition()
{
	struct LED_Transition *trans = &current_animation->transitions[anim_transition];
	copy(trans->leds, buffer_target);
	transition_start = GetTimePoint();
	transition_end = transition_start + trans->time;
	transition = trans->mode;
}

void rgbled_animation(struct LED_Animation *anim)
{
	if (anim->count == 0) return;
	if (current_animation == anim) return;
	rgbled_lock();

	// Reset state
	enum LED_INTERPOLATION prevTransition = transition;
	transition = INTER_UNDEFINED;
	current_animation = NULL;

	// Set source for first transition as current (potentially interrupted) state
	pickup_from_transition(prevTransition);

	// Set animation
	anim_transition = 0;
	anim_repetitions = 0;
	current_animation = anim;
	set_anim_transition();

	if (rgbled_ready() || rgbled_abort_wakeup_waiting())
		rgbled_ready_callback(); // Can set immediately

	rgbled_unlock();
}

bool rgbled_animating(struct LED_Animation *anim)
{
	if (anim == NULL) return current_animation != NULL;
	return current_animation == anim;
}

void rgbled_ready_callback()
{
	if (transition == INTER_UNDEFINED)
		return;

	TimePoint next_update = GetTimePoint();
	if (next_update + (RGB_UPDATE_INTERVAL_MS*TICKS_PER_MS)/10 > transition_end)
	{ // At end of transition
		struct LED_Animation *anim = current_animation;
		if (anim == NULL)
		{ // Finished individual transition
			transition = INTER_UNDEFINED;
			process(buffer_target, buffer_latest, brightness);
		}
		if (anim && ++anim_transition >= anim->count)
		{ // Finished animation
			if (++anim_repetitions >= anim->repetitions && anim->repetitions >= 0)
			{ // Finished all repetitions
				anim = current_animation = NULL;
				transition = INTER_UNDEFINED;
				process(buffer_target, buffer_latest, brightness);
			}
			else
			{ // Repeat animation
				anim_transition = 0;
			}
		}
		if (anim)
		{ // Continue animation
			copy(buffer_target, buffer_source);
			set_anim_transition();
		}
	}

	if (transition == INTER_LERP_LINEAR)
	{
		lerp_process(buffer_source, buffer_target, buffer_latest, get_lerp(), brightness);
		// Update at desired frequency
		next_update += RGB_UPDATE_INTERVAL_MS * TICKS_PER_MS;
		if (next_update > transition_end)
			next_update = transition_end;
	}
	else if (transition == INTER_IMMEDIATE)
	{
		process(buffer_target, buffer_latest, brightness);
		// No need to update until transition ends
		next_update = transition_end;
	}

	rgbled_set_and_delay(buffer_latest, next_update);
}