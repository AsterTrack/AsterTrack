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

#include "comm/packet.h"

#define USB_HEADROOM 	(USB_PACKET_HEADER+BLOCK_HEADER_SIZE+USB_PACKET_ALIGNMENT+2)
// The +2 is only the have the headroom be a multiple of 4, so the buffer start is also 4-byte aligned
// That's requried for event code which is writing full uint32_t

#if USB_HEADROOM % 4 != 0
#error "Debug or Event Buffers not properly aligned!"
#endif

// Events
#if defined(ENABLE_EVENTS)

volatile uint_fast16_t eventHead = 0, eventTail = 0, eventSending = 0;
__attribute__((aligned(4))) uint8_t eventAlloc[EVENT_BUFFER_SIZE+USB_HEADROOM];
uint8_t *eventBuffer = eventAlloc+USB_HEADROOM;
uint8_t eventFilter[CONTROLLER_EVENT_MAX];
uint8_t eventLogClass = 0;

#endif

// Debug
#if defined(ENABLE_LOG)

volatile uint_fast16_t debugHead = 0, debugTail = 0, debugSending = 0;
__attribute__((aligned(4))) uint8_t debugAlloc[DEBUG_BUFFER_SIZE+USB_HEADROOM];
uint8_t *debugBuffer = debugAlloc+USB_HEADROOM;

#endif

#if defined(ENABLE_LOG) && defined(LOG_USE_SDI)

#ifdef CH32V307
#define DEBUG_DATA0_ADDRESS  ((volatile uint32_t*)0xE0000380)
#define DEBUG_DATA1_ADDRESS  ((volatile uint32_t*)0xE0000384)
#define DEBUG_STATUS_SENTINEL  ((volatile uint32_t*)0xE0000388) // 0 if debugger is attached

#else
#error "SDI not supported!"
#endif

void SDI_Enable()
{
	*(DEBUG_DATA0_ADDRESS) = 0;
	delayMS(1);
}

inline bool SDI_ProbeAttached()
{
	return *DEBUG_STATUS_SENTINEL == 0;
}

inline bool SDI_Busy()
{
	return *DEBUG_DATA0_ADDRESS != 0;
}

bool SDI_Write(uint8_t *buf, uint_fast16_t size)
{
	if (!SDI_ProbeAttached())
		return false;

	uint_fast16_t i = 0;
	do
	{
		while (SDI_Busy());

		uint_fast8_t batchSize = size > 7? 7 : size;

		*(DEBUG_DATA1_ADDRESS) = (*(buf+i+3)) | (*(buf+i+4)<<8) | (*(buf+i+5)<<16) | (*(buf+i+6)<<24);
		*(DEBUG_DATA0_ADDRESS) = batchSize | (*(buf+i)<<8) | (*(buf+i+1)<<16) | (*(buf+i+2)<<24);

		i += batchSize;
		size -= batchSize;
	} while (size);

	return true;
}

#endif
