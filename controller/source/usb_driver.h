/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef __USB_DRIVER_H
#define __USB_DRIVER_H

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "util.h"

#include "ch32v30x.h"
#include "ch32v30x_usb.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Structures */

/**
 * COPY from usbd_core.h from libusb_stm32
 */

/**\brief Represents generic USB control request.*/
typedef struct
{
	uint8_t bmRequestType; /**<\brief This bitmapped field identifies the characteristics of
							* the specific request.*/
	uint8_t bRequest;		/**<\brief This field specifies the particular request.*/
	uint16_t wValue;		/**<\brief It is used to pass a parameter to the device, specific to
							* the request.*/
	uint16_t wIndex;		/**<\brief It is used to pass a parameter to the device, specific to
							* the request.*/
	uint16_t wLength;		/**<\brief This field specifies the length of the data transferred
							* during the second phase of the control transfer.*/
	uint8_t data[];			/**<\brief Data payload.*/
} usbd_ctlreq;

#define usbd_evt_reset		0	/**<\brief Reset.*/
#define usbd_evt_sof		1	/**<\brief Start of frame.*/
#define usbd_evt_susp		2	/**<\brief Suspend.*/
#define usbd_evt_wkup		3	/**<\brief Wakeup.*/
#define usbd_evt_eptx		4	/**<\brief Data packet transmitted*/
#define usbd_evt_eprx		5	/**<\brief Data packet received.*/
#define usbd_evt_epsetup	6	/**<\brief Setup packet received.*/
#define usbd_evt_error		7	/**<\brief Data error.*/
#define usbd_evt_count		8

/**\brief Reporting status results.*/
typedef enum _usbd_respond {
	usbd_fail,					/**<\brief Function has an error, STALLPID will be issued.*/
	usbd_ack,					/**<\brief Function completes request accepted ZLP or data will be send.*/
	usbd_nak,					/**<\brief Function is busy. NAK handshake.*/
} usbd_respond;

/**\brief USB device machine states.*/
enum usbd_machine_state {
	usbd_state_disabled,
	usbd_state_disconnected,
	usbd_state_default,			/**< Default.*/
	usbd_state_addressed,		/**< Addressed.*/
	usbd_state_configured,		/**< Configured.*/
};

/**\brief USB device control endpoint machine state.*/
enum usbd_ctl_state {
	usbd_ctl_idle,				/**<\brief Idle stage. Awaiting for SETUP packet.*/
	usbd_ctl_rxdata,			/**<\brief RX stage. Receiving DATA-OUT payload.*/
	usbd_ctl_txdata,			/**<\brief TX stage. Transmitting DATA-IN payload.*/
	usbd_ctl_ztxdata,			/**<\brief TX stage. Transmitting DATA-IN payload. Zero length
								* packet maybe required..*/
	usbd_ctl_lastdata,			/**<\brief TX stage. Last DATA-IN packed passed to buffer. Awaiting
								* for the TX completion.*/
	usbd_ctl_statusin,			/**<\brief STATUS-IN stage.*/
	usbd_ctl_statusout,			/**<\brief STATUS-OUT stage.*/
};

typedef struct {} usbd_rqc_callback; // Not implemented

/** END COPY */

typedef struct {
	uint8_t		*data_buf;		/**<\brief Pointer to data buffer used for control requests.*/
	uint8_t		*data_ptr;		/**<\brief Pointer to current data for control request.*/
	uint16_t	data_count;		/**<\brief Count remained data for control request.*/
	uint16_t	data_sending;	/**<\brief Holds data sent in last packet for control request.*/
	uint16_t	data_count_rx;	/**<\brief Count remained data for control request.*/
	uint16_t	data_maxsize;	/**<\brief Size of the data buffer for control requests.*/
	uint8_t		ep0size;		/**<\brief Size of the control endpoint.*/
	uint8_t		device_cfg;		/**<\brief Current device configuration number.*/
	uint8_t		device_state;	/**<\brief Current \ref usbd_machine_state.*/
} usbd_status;

typedef struct
{
	usbd_status status;
	enum usbd_ctl_state state;
	usbd_ctlreq ctl_setup;
	bool ep_set[16];
} usbd_device;


/* Functions */

void usbd_driver_init(usbd_device *dev, const uint8_t ep0size, uint8_t *buffer, const uint16_t bsize);
void usbd_enable(usbd_device *dev, bool enable);
void usbd_ep_rx_config(usbd_device *dev, uint8_t ep, uint8_t *dma_buf, uint16_t epsize);
void usbd_ep_tx_config(usbd_device *dev, uint8_t ep);
void usbd_ep_deconfig(usbd_device *dev, uint8_t ep);
bool usbd_ep_set_dma(usbd_device *dev, uint8_t ep, uint8_t *buf, uint16_t len);
void usbd_ep_reset_dma(usbd_device *dev, uint8_t ep);
void usbd_poll(usbd_device *dev);

static inline void usbd_connect(usbd_device *dev, bool connect)
{
	if (connect)
		USBHSD->CONTROL |= USBFS_UC_DEV_PU_EN;
	else
		USBHSD->CONTROL &= ~USBFS_UC_DEV_PU_EN;
}

// Not implemented, always same callbacks usbd_EP_TX/RX_CB
static inline bool usbd_reg_endpoint(usbd_device *dev, uint8_t ep, void *callback) { return true; }

static inline uint8_t usbd_get_speed() { return USBHSD->SPEED_TYPE & 0b11; }

#ifdef __cplusplus
}
#endif

#endif /* __USB_DRIVER_H */

