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

#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "usb_std.h"

// Freely provided VID/PID under conditions of unique identification. Thanks! https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
#define USBD_VID					5824		// 0x16C0
#define USBD_PID					1500		// 0x05DC

#define WCID_VENDOR_CODE			0x20		// Anything non-zero should work

#define USBD_CONFIGURATION_ID		1			// Number of default configuration (must be over 0)
#define USBD_INTERFACE_ID			0			// Number of default interface (only one)

// Control Endpoint
#define USBD_EP_CTRL_SIZE			64			// Control Endpoint number 0 (max 64, but it's double buffered)
#define USBD_CTRL_MAX_PACKET_SIZE	2048		// Control maximum packet size (assembled from multiple transfers)

// Interrupt Endpoints - direction IN (Device to host)
#define USBD_EP_INT_IN_ADDR_BASE	(0x80 | 1)	// Start endpoint address
// USB 2.0 FS
#define USBD_FS_EP_INT_IN_NUM		5			// Maximum number of interrupt endpoints, depends on configuration
#define USBD_FS_EP_INT_IN_SIZE		64 			// Individual packet sizes, maximum for USB FullSpeed Spec
#define USBD_FS_EP_INT_INTERVAL     1000        // Should match bInterval*1000
// USB 2.0 HS
#define USBD_HS_EP_INT_IN_NUM		1			// Maximum number of interrupt endpoints, depends on configuration
// More than one adds uncertainty in timing, which time sync relies on. Either switch to isochronous or keep one for now.
#define USBD_HS_EP_INT_IN_SIZE		1024		// Individual packet sizes, maximum for USB HighSpeed Spec
#define USBD_HS_EP_INT_INTERVAL     125         // Should match bInterval*125

#ifdef __cplusplus
}
#endif

#endif // __USBD_CONF_H