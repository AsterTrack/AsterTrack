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
#define USBD_EP_CTRL_SIZE			64			// Max transfer size of Control Endpoint (max 64)
#define USBD_CTRL_MAX_PACKET_SIZE	1024		// Max packet size of Control Endpoint (assembled)
// WARNING: BAD VALUES MAY CAUSE A LINUX KERNEL CRASH ON HOST!
// Just setting USBHSD->UEP0_MAX_LEN to a bad value yields this behaviour, so not my code?
// Known bad values: 2048-2 to 2048+7. 2048-3 is fine, 2048+8 is fine.
// BUG: kernel NULL pointer dereference, address: 0000000000000030
// xhci_hcd 0000:c1:00.3: Timeout while waiting for setup device command
// #PF: supervisor read access in kernel mode
// #PF: error_code(0x0000) - not-present page

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