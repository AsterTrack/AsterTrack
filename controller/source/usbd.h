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

#ifndef __USBD_H
#define __USBD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "util.h"

// Standard definitions from libusb_stm32
#include "usb_std.h"

// USB HS driver specific to the CH32V307
#include "usb_driver.h"


/* Utilities */

// Mutex to access USB hardware from main loop or other non-USB callbacks
static inline void DisableUSBInterrupts ()
{
	NVIC_DisableIRQ(USBHS_IRQn);
}
static inline void EnableUSBInterrupts ()
{
	NVIC_EnableIRQ(USBHS_IRQn);
}
static inline void EnterUSBZone ()
{
	// Does NOT support nesting
	ATOMIC_SINGLE(DisableUSBInterrupts(););
}
static inline void LeaveUSBZone ()
{
	// Does NOT support nesting
	ATOMIC_SINGLE(EnableUSBInterrupts(););
}


/* Structures */

typedef struct {
	TimePoint lastFrame;
} USBPortState;

typedef struct {
	USBPortState *ep;
	uint_fast8_t epCount;
	uint_fast16_t interval;
} USBPortInterface;


/* Exposed variables */

extern USBPortInterface *usbd_interface;
extern usbd_device hUSB;


/* Functions */

void usbd_init();

bool usbd_is_configured();

#ifdef __cplusplus
}
#endif

#endif /* __USBD_H */
