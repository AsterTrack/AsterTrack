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

#include "util.h"

// Events
#if EVENTLOG

volatile uint_fast16_t eventHead = 0, eventTail = 0, eventSending = 0;
uint8_t *eventBuffer = NULL;
uint8_t eventFilter[CONTROLLER_EVENT_MAX];
uint8_t eventLogClass = 0;

#endif

// Debug
#if LOGGING

volatile uint_fast16_t debugHead = 0, debugTail = 0, debugSending = 0;
uint8_t *debugBuffer = NULL;

#endif
