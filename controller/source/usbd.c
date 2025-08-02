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

#include "compat.h" // __IRQ;

#include "usbd.h"
// Configuration and descriptors of USB device
#include "usbd_desc.h"

/* How it works (USB 2.0 HS on CH32V307):
One or more interrupt endpoints (1024 bytes each)
Parts of large packets are sent directly from the UART RX Buffer
Smaller packets are copied to a few shared buffers, in a simpler implementation than that of USB FS
Then packets are queued, and endpoints set to DMA directly from these buffers
Packets are only queued in full once they are completed, unless they are large (> 1024), in which case they are sent in 1024-byte blocks

In the future, it might make sense to switch to isochronous transfers
*/


// State of all endpoints (for time-keeping mainly)
USBPortState usb_fs_int_eps[USBD_FS_EP_INT_IN_NUM];
USBPortState usb_hs_int_eps[USBD_HS_EP_INT_IN_NUM];

/* Custom configuration of endpoints */
#define INT_FS_EP_DEF(C) { \
	.ep = usb_fs_int_eps, \
	.epCount = C, \
	.interval = 1000, \
}
#define INT_HS_EP_DEF(C) { \
	.ep = usb_hs_int_eps, \
	.epCount = C, \
	.interval = 125, \
}
// See USBD_Config_Desc_FS
USBPortInterface usbd_fs_interfaces_arr[] = {
	INT_FS_EP_DEF(0), INT_FS_EP_DEF(1), INT_FS_EP_DEF(2), INT_FS_EP_DEF(4), INT_FS_EP_DEF(5)
};
// See USBD_Config_Desc_HS
USBPortInterface usbd_hs_interfaces_arr[] = {
	INT_HS_EP_DEF(0),
	INT_HS_EP_DEF(1),
	/* INT_HS_EP_DEF(2) */
};

// Exposed configuration
uint8_t usbd_interface_index = -1;
USBPortInterface *usbd_interface = NULL;

// State
usbd_device hUSB;

// Used for holding received packets from control endpoint
__attribute__((aligned(4))) static uint8_t ctl_buffer[USBD_CTRL_MAX_PACKET_SIZE+8];

// Directly link against usbd user implementation
void usbd_close_interface();
void usbd_set_interface(uint8_t interface);
usbd_respond usbd_control_respond(usbd_device *usbd, usbd_ctlreq *req);
usbd_respond usbd_control_receive(usbd_device *usbd, usbd_ctlreq *req);
void usbd_control_resolution(usbd_device *usbd, usbd_ctlreq *req, bool success);

/* USB class implementation functions */
usbd_respond class_impl_getdesc(usbd_device *usbd, usbd_ctlreq *req, void **address, uint16_t *length)
{
	switch (req->wValue >> 8)
	{
	/* Standard USB descriptors, implementation-specific */
	case USB_DTYPE_DEVICE:
		*address = (void*)&USBD_Device_Desc;
		*length = sizeof(USBD_Device_Desc);
		return usbd_ack;
	case USB_DTYPE_CONFIGURATION:
		if (usbd_get_speed() == USB_SPEED_FULL)
		{ // Send relevant configuration for FS Mode
			*address = (void*)&USBD_Config_Desc_FS;
			*length = sizeof(USBD_Config_Desc_FS);
		}
		else
		{ // Send relevant configuration for HS Mode
			*address = (void*)&USBD_Config_Desc_HS;
			*length = sizeof(USBD_Config_Desc_HS);
		}
		return usbd_ack;
	case USB_DTYPE_STRING:
		DEBUG_CHARR('+', 'S', 'T', 'R', INT99_TO_CHARR(req->wValue & 0xFF));
		if ((req->wValue & 0xFF) == 0xEE)
		{ // Windows WCID descriptor, this initiates the WCID process after Windows gets a response
			*address = (void*)&str_desc_wcid;
			*length = str_desc_wcid.bLength;
			return usbd_ack;
		}
		if ((req->wValue & 0xFF) >= sizeof(str_desc_table) / sizeof(struct usb_string_descriptor *))
			return usbd_fail; // No such string descriptor
		*address = (void*)str_desc_table[req->wValue & 0xFF];
		*length = str_desc_table[req->wValue & 0xFF]->bLength;
		return usbd_ack;
	case USB_DTYPE_QUALIFIER:
		if (usbd_get_speed() == USB_SPEED_FULL)
		{ // Send relevant configuration if device was in HS mode
			*address = (void*)&USBD_Qualifier_Desc_HS;
			*length = sizeof(USBD_Qualifier_Desc_HS);
		}
		else
		{ // Send relevant configuration if device was in FS mode
			*address = (void*)&USBD_Qualifier_Desc_FS;
			*length = sizeof(USBD_Qualifier_Desc_FS);
		}
		return usbd_ack;

	// get usb other-speed descriptor
	case USB_DTYPE_OTHER:
		return usbd_fail;
		/* if (usbd_get_speed() == USBHS_SPEED_FULL)
		{ // Send relevant configuration if device was in HS mode
			*address = (void*)&USBD_Qualifier_Desc_HS;
			*length = sizeof(USBD_Qualifier_Desc_HS);
		}
		else
		{ // Send relevant configuration if device was in FS mode
			*address = (void*)&USBD_Qualifier_Desc_FS;
			*length = sizeof(USBD_Qualifier_Desc_FS);
		} */
		// TODO: Only descriptor type is modified from normal descriptors HS/FS descriptors, likely not needed
		/* if (USBHS_DevSpeed == USBHS_SPEED_HIGH)
		{
			// High speed mode
			memcpy(&TAB_USB_HS_OSC_DESC[2], &MyCfgDescr_FS[2], DEF_USBD_CONFIG_FS_DESC_LEN - 2);
			*address = (uint8_t *)&TAB_USB_HS_OSC_DESC[0];
			*length = DEF_USBD_CONFIG_FS_DESC_LEN;
		}
		else if (USBHS_DevSpeed == USBHS_SPEED_FULL)
		{
			// Full speed mode
			memcpy(&TAB_USB_FS_OSC_DESC[2], &MyCfgDescr_HS[2], DEF_USBD_CONFIG_HS_DESC_LEN - 2);
			*address = (uint8_t *)&TAB_USB_FS_OSC_DESC[0];
			*length = DEF_USBD_CONFIG_HS_DESC_LEN;
		}*/
		return usbd_ack;

	/* Class-specific descriptors are usually targetted at interface (handled in control) */
	default:
		return usbd_fail;
	}
}

static void class_impl_closeInterface(usbd_device *usbd)
{
	if (usbd_interface != NULL)
	{
		for (int j = 0; j < usbd_interface->epCount; j++)
		{
			uint8_t ep = USBD_EP_INT_IN_ADDR_BASE+j;
			usbd_ep_deconfig(usbd, ep);
		}
		usbd_close_interface();
	}
	usbd_interface_index = -1;
	usbd_interface = NULL;
}

static usbd_respond class_impl_setinterface(usbd_device *usbd, uint8_t interface)
{
	USBPortInterface *sel_interface = NULL;
	if (usbd_get_speed() == USB_SPEED_FULL && interface >= 0 && interface < sizeof(usbd_fs_interfaces_arr)/sizeof(USBPortInterface))
		sel_interface = &usbd_fs_interfaces_arr[interface];
	if (usbd_get_speed() == USB_SPEED_HIGH && interface >= 0 && interface < sizeof(usbd_hs_interfaces_arr)/sizeof(USBPortInterface))
		sel_interface = &usbd_hs_interfaces_arr[interface];
	if (sel_interface == NULL)
	{
		class_impl_closeInterface(usbd);
		return usbd_fail;
	}
	if (sel_interface != usbd_interface)
	{
		class_impl_closeInterface(usbd);
		usbd_interface_index = interface;
		usbd_interface = sel_interface;
		for (int j = 0; j < usbd_interface->epCount; j++)
		{ // Setup default interrupt endpoints and callback
			uint8_t ep = USBD_EP_INT_IN_ADDR_BASE+j;
			// Barely needs configuration, on TX it just DMAs from any random buffer
			usbd_ep_tx_config(usbd, ep);
		}
		usbd_set_interface(interface);
	}
	return usbd_ack;
}

/*
	Incoming request on control pipeline (endpoint 0 OUT)
	Handle standard class requests and all non-standard requests
	Set callback to be notified once a DEVTOHOST response has been fully send (potentially over multiple frames)
*/
usbd_respond class_impl_control(usbd_device *usbd, usbd_ctlreq *req)
{
	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_STANDARD
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_INTERFACE
		&& req->bRequest == USB_STD_GET_INTERFACE
		&& req->wIndex == USBD_INTERFACE_ID)
	{ // Standard Get Interface Request (for alternative interrupt transfer)
		usbd->status.data_ptr = &usbd_interface_index;
		usbd->status.data_count = 1;
		USBD_STR("+GetInterface");
		return usbd_ack;
	}

	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_STANDARD
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_INTERFACE
		&& req->bRequest == USB_STD_SET_INTERFACE
		&& req->wIndex == USBD_INTERFACE_ID)
	{ // Standard Set Interface Request (to set to alternative interrupt transfer)
		USBD_STR("+SetInterface");
		return class_impl_setinterface(usbd, req->wValue);
	}

	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_VENDOR
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_DEVICE
		&& req->bRequest == WCID_VENDOR_CODE
		&& req->wIndex == 0x0004)
	{ // WCID compatible ID request (part of WinUSB support for windows)
		USBD_STR("+WCID");
		usbd->status.data_ptr = (void*)&WCID_CompatID_Desc;
		usbd->status.data_count = sizeof(WCID_CompatID_Desc);
		return usbd_ack;
	}

	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_VENDOR
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_OTHER)
	{ // Vendor-specific requests (can't use interface recipient as windows enforces index==USBD_INTERFACE_ID for that...)
		if ((req->bmRequestType & USB_REQ_DIRECTION) == USB_REQ_DEVTOHOST)
			return usbd_control_respond(usbd, req);
		else
			return usbd_control_receive(usbd, req);
	}

	return usbd_fail;
}

void class_impl_control_resolution(usbd_device *usbd, usbd_ctlreq *req, bool success)
{
	// Standard _class_ request doesn't need to handle this
	// If descriptor wasn't sent, who cares
	// The only standard request that cares is the device request SetAddress, which is handled by the driver

	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_VENDOR
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_OTHER)
	{ // Vendor-specific requests might need to handle it
		// E.g. if some data wasn't sent, it might have to be kept for future requests
		return usbd_control_resolution(usbd, req, success);
	}
}

usbd_respond class_impl_setconf(usbd_device *usbd, uint8_t cfg)
{
	switch (cfg)
	{
	case 0: // Deconfigure
		class_impl_closeInterface(usbd);
		return usbd_ack;
	case 1: // Configuration 1, Default interface
		class_impl_setinterface(usbd, 0);
		return usbd_ack;
	default:
		return usbd_fail;
	}
}

void usbd_init()
{
	usbd_interface_index = -1;
	usbd_interface = NULL;

	// Init with hardware driver usbd_hw and control endpoint setup
	usbd_driver_init(&hUSB, USBD_EP_CTRL_SIZE, ctl_buffer, sizeof(ctl_buffer));
	usbd_enable(&hUSB, true);
	// Setup interrupt
	NVIC_SetPriority(USBHS_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
	NVIC_EnableIRQ(USBHS_IRQn);
}

bool usbd_is_configured()
{
	return hUSB.status.device_cfg == usbd_state_configured;
}

void USBHS_IRQHandler(void) __IRQ;
void USBHS_IRQHandler(void)
{
	LOG_EVT_INT(CONTROLLER_INTERRUPT_USB, true);
	usbd_poll(&hUSB);
	LOG_EVT_INT(CONTROLLER_INTERRUPT_USB, false);
}
