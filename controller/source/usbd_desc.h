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

#ifndef __USBD_DESC_H
#define __USBD_DESC_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "usb_std.h"
#include "usbd_conf.h"


/* USB string descriptors */
static const struct usb_string_descriptor str_desc_wcid = USB_ARRAY_DESC('M', 'S', 'F', 'T', '1', '0', '0', WCID_VENDOR_CODE);
static const struct usb_string_descriptor str_desc_lang = USB_ARRAY_DESC(USB_LANGID_ENG_UK);
static const struct usb_string_descriptor str_desc_manufacturer = USB_STRING_DESC("Seneral seneral.dev");

// ATTENTION: Had issues before with string length over 30 (new arch linux install, old ubuntu install didn't have this issue)
static const struct usb_string_descriptor str_desc_product = USB_STRING_DESC("AsterTrack Controller V4");
static const struct usb_string_descriptor *const str_desc_table[] = {
	&str_desc_lang,
	&str_desc_manufacturer,
	&str_desc_product,
};

/* USB device descriptor */
static const struct usb_device_descriptor USBD_Device_Desc = {
	.bLength = sizeof(struct usb_device_descriptor),
	.bDescriptorType = USB_DTYPE_DEVICE,
	.bcdUSB = VERSION_BCD(2, 0, 0),
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = USBD_EP_CTRL_SIZE,
	.idVendor = USBD_VID,
	.idProduct = USBD_PID,
	.bcdDevice = VERSION_BCD(1, 0, 0),
	.iManufacturer = 1, // String index
	.iProduct = 2, // String index
	.iSerialNumber = INTSERIALNO_DESCRIPTOR,
	.bNumConfigurations = 1,
};
/* USB qualifier descriptor describing device if it were the other speed (FS->HS and vice versa)*/
static const struct usb_qualifier_descriptor USBD_Qualifier_Desc_FS = {
	.bLength = sizeof(struct usb_qualifier_descriptor),
	.bDescriptorType = USB_DTYPE_QUALIFIER,
	.bcdUSB = VERSION_BCD(2, 0, 0),
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = USBD_EP_CTRL_SIZE,
	.bNumConfigurations = 1,
};
static const struct usb_qualifier_descriptor USBD_Qualifier_Desc_HS = {
	.bLength = sizeof(struct usb_qualifier_descriptor),
	.bDescriptorType = USB_DTYPE_QUALIFIER,
	.bcdUSB = VERSION_BCD(2, 0, 0),
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = USBD_EP_CTRL_SIZE,
	.bNumConfigurations = 1,
};


/* Structure of custom device configuration descriptor */
struct usb_config_fs
{
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface_0_0_int;
	struct usb_interface_descriptor interface_0_1_int;
		struct usb_endpoint_descriptor ep_int_in_1_0;
	struct usb_interface_descriptor interface_0_2_int;
		struct usb_endpoint_descriptor ep_int_in_2_0;
		struct usb_endpoint_descriptor ep_int_in_2_1;
	struct usb_interface_descriptor interface_0_4_int;
		struct usb_endpoint_descriptor ep_int_in_4_0;
		struct usb_endpoint_descriptor ep_int_in_4_1;
		struct usb_endpoint_descriptor ep_int_in_4_2;
		struct usb_endpoint_descriptor ep_int_in_4_3;
	struct usb_interface_descriptor interface_0_5_int;
		struct usb_endpoint_descriptor ep_int_in_5_0;
		struct usb_endpoint_descriptor ep_int_in_5_1;
		struct usb_endpoint_descriptor ep_int_in_5_2;
		struct usb_endpoint_descriptor ep_int_in_5_3;
		struct usb_endpoint_descriptor ep_int_in_5_4;
} __attribute__((packed));

struct usb_config_hs
{
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface_0_0_int;
	struct usb_interface_descriptor interface_0_1_int;
		struct usb_endpoint_descriptor ep_int_in_1_0;
/* 	struct usb_interface_descriptor interface_1_2_int;
		struct usb_endpoint_descriptor ep_int_in_2_0;
		struct usb_endpoint_descriptor ep_int_in_2_1; */
} __attribute__((packed));

/* Descriptors of interrupt interfaces alternate settings */
#define INT_ALT_DESC(A,N) { \
	.bLength = sizeof(struct usb_interface_descriptor), \
	.bDescriptorType = USB_DTYPE_INTERFACE, \
	.bInterfaceNumber = USBD_INTERFACE_ID, \
	.bAlternateSetting = A, \
	.bNumEndpoints = N, \
	.bInterfaceClass = USB_CLASS_VENDOR, \
	.bInterfaceSubClass = USB_SUBCLASS_NONE, \
	.bInterfaceProtocol = USB_PROTO_NONE, \
	.iInterface = NO_DESCRIPTOR, \
}

/* Descriptors of interrupt endpoints */
// All the same with different adresses, reused across multiple alternate settings
#define INT_FS_EP_CFG(I) { \
	.bLength = sizeof(struct usb_endpoint_descriptor), \
	.bDescriptorType = USB_DTYPE_ENDPOINT, \
	.bEndpointAddress = USBD_EP_INT_IN_ADDR_BASE+I, \
	.bmAttributes = USB_EPTYPE_INTERRUPT, \
	.wMaxPacketSize = USBD_FS_EP_INT_IN_SIZE, \
	.bInterval = 1, \
}

/* Descriptors of interrupt endpoints */
// All the same with different adresses, reused across multiple alternate settings
#define INT_HS_EP_DESC(I) { \
	.bLength = sizeof(struct usb_endpoint_descriptor), \
	.bDescriptorType = USB_DTYPE_ENDPOINT, \
	.bEndpointAddress = USBD_EP_INT_IN_ADDR_BASE+I, \
	.bmAttributes = USB_EPTYPE_INTERRUPT, \
	.wMaxPacketSize = USBD_HS_EP_INT_IN_SIZE, \
	.bInterval = 1, \
}

/* USB configuration descriptor */
static const struct usb_config_fs USBD_Config_Desc_FS = {
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),
		.bDescriptorType = USB_DTYPE_CONFIGURATION,
		.wTotalLength = sizeof(struct usb_config_fs),
		.bNumInterfaces = 1,
		.bConfigurationValue = USBD_CONFIGURATION_ID,
		.iConfiguration = NO_DESCRIPTOR,
		.bmAttributes = USB_CFG_ATTR_RESERVED,
		.bMaxPower = USB_CFG_POWER_MA(100),
	},
	.interface_0_0_int = INT_ALT_DESC(0,1),
	.interface_0_1_int = INT_ALT_DESC(1,1),
	.ep_int_in_1_0 = INT_FS_EP_CFG(0),
	.interface_0_2_int = INT_ALT_DESC(2,2),
	.ep_int_in_2_0 = INT_FS_EP_CFG(0),
	.ep_int_in_2_1 = INT_FS_EP_CFG(1),
	.interface_0_4_int = INT_ALT_DESC(3,4),
	.ep_int_in_4_0 = INT_FS_EP_CFG(0),
	.ep_int_in_4_1 = INT_FS_EP_CFG(1),
	.ep_int_in_4_2 = INT_FS_EP_CFG(2),
	.ep_int_in_4_3 = INT_FS_EP_CFG(3),
	.interface_0_5_int = INT_ALT_DESC(4,5),
	.ep_int_in_5_0 = INT_FS_EP_CFG(0),
	.ep_int_in_5_1 = INT_FS_EP_CFG(1),
	.ep_int_in_5_2 = INT_FS_EP_CFG(2),
	.ep_int_in_5_3 = INT_FS_EP_CFG(3),
	.ep_int_in_5_4 = INT_FS_EP_CFG(4),
};

static const struct usb_config_hs USBD_Config_Desc_HS = {
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),
		.bDescriptorType = USB_DTYPE_CONFIGURATION,
		.wTotalLength = sizeof(struct usb_config_hs),
		.bNumInterfaces = 1,
		.bConfigurationValue = USBD_CONFIGURATION_ID,
		.iConfiguration = NO_DESCRIPTOR,
		.bmAttributes = USB_CFG_ATTR_RESERVED,
		.bMaxPower = USB_CFG_POWER_MA(100),
	},
	.interface_0_0_int = INT_ALT_DESC(0,0),
	.interface_0_1_int = INT_ALT_DESC(1,1),
	.ep_int_in_1_0 = INT_HS_EP_DESC(0),
/* 	.interface_1_2_int = INT_ALT_DESC(1,2),
	.ep_int_in_2_0 = INT_HS_EP_DESC(0),
	.ep_int_in_2_1 = INT_HS_EP_DESC(1) */
};

/*
	WCID Compat ID descriptor
*/
struct usb_compatid_header
{
	uint32_t dwLength;
	uint16_t bcdVersion;
	uint16_t wIndex;
	uint8_t bCount;
	uint8_t bReserved0[7];
} __attribute__((packed));
struct usb_compatid_section
{
	uint8_t bFirstInterfaceNumber;
	uint8_t bReserved1;
	uint8_t bCompatibleID[8];
	uint8_t bSubCompatibleID[8];
	uint8_t bReserved0[6];
} __attribute__((packed));
struct usb_compatid_descriptor
{
	struct usb_compatid_header header;
	struct usb_compatid_section section0;
} __attribute__((packed));
static const struct usb_compatid_descriptor WCID_CompatID_Desc = {
	.header = {
		.dwLength = sizeof(struct usb_compatid_descriptor),
		.bcdVersion = VERSION_BCD(1, 0, 0),
		.wIndex = 0x0004, // Index of compatID descriptor defined by WCID spec
		.bCount = 0x01,	// number of sections to follow
		.bReserved0 = {0}
	},
	.section0 = {
		.bFirstInterfaceNumber = USBD_INTERFACE_ID, // Interface (feature group) to load driver for
		.bReserved1 = 0x01,
		.bCompatibleID = "WINUSB", // This tells windows to load the WinUSB drivers for this feature group
		.bSubCompatibleID = {0},
		.bReserved0 = {0}
	}
};

#ifdef __cplusplus
}
#endif

#endif // __USBD_DESC_H