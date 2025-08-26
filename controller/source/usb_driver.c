/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"
#include "compat.h"

#include "usb_driver.h"
#include "usb_std.h"
#include "util.h"


/* Defines */

#define EPx_RX_DMA(ep) (*(&USBHSD->UEP1_RX_DMA + ((ep)-1)))
#define EPx_TX_DMA(ep) (*(&USBHSD->UEP1_TX_DMA + ((ep)-1)))
#define EP_MAX_LEN(ep) (*(&USBHSD->UEP0_MAX_LEN + ((ep)*2)))
#define EP_TX_LEN(ep) (*(&USBHSD->UEP0_TX_LEN + ((ep)*2)))
#define EP_TX_CTRL(ep) (*(&USBHSD->UEP0_TX_CTRL + ((ep)*4)))
#define EP_RX_CTRL(ep) (*(&USBHSD->UEP0_RX_CTRL + ((ep)*4)))


/* Functions */

// Directly link against usbd class implementation
void usbd_SOF_CB(usbd_device *usbd);
void usbd_EP_TX_CB(usbd_device *usbd, uint8_t ep, bool success);
void usbd_EP_RX_CB(usbd_device *usbd, uint8_t ep, uint16_t len);
usbd_respond class_impl_getdesc(usbd_device *usbd, usbd_ctlreq *req, void **address, uint16_t *length);
usbd_respond class_impl_control(usbd_device *usbd, usbd_ctlreq *req);
void class_impl_control_resolution(usbd_device *usbd, usbd_ctlreq *req, bool success);
usbd_respond class_impl_setconf(usbd_device *usbd, uint8_t cfg);

void usbd_driver_init(usbd_device *dev, const uint8_t ep0size, uint8_t *buffer, const uint16_t bsize)
{
	// Initialise Clocks

	RCC->CFGR2 &= 0x00FFFFFF;

	// Configure USB HS PLL to generate 480Mhz from reference of PLL (HSE)
	// HSE is 8MHz, either divide by 2 so reference is 4Mhz, or take the 8Mhz as reference directly
	//RCC->CFGR2 |= (RCC_USBHSPLLCLKSource_HSE << 27) | (RCC_USBPLL_Div2 << 24) | (RCC_USBHSPLLCKREFCLK_4M << 28);
	RCC->CFGR2 |= (RCC_USBHSPLLCLKSource_HSE << 27) | (RCC_USBPLL_Div1 << 24) | (RCC_USBHSPLLCKREFCLK_8M << 28);

	// Two ways to generate the 48MHz OTG FS CLK
	// Either by dividing 144MHz System Clock by 3 (USBCLK) => HSE 8MHz * 18 / 3
	// Or from USB HS PLL which also generates the 480MHz from HSE => 8Mhz * 60 / 10
	// Which one is the better source? IDK
	// I assume USBCLK is intended for when only FS is used so USB HS PLL can be disabled

	// Set USBCLK = 144MHz/3 = 48MHz
	//RCC->CFGR0 |= 0b10 << RCC_USBPRE;
	// Select USBCLK and enable it
	//RCC->CFGR2 |= (RCC_USBCLK48MCLKSource_USBPHY << 31) | (1 << 30);

	// Select output from USB HS PLL (HSE 8Mhz * 60 / 10 since it also generates the 480MHz)
	RCC->CFGR2 |= (RCC_USBCLK48MCLKSource_PLLCLK << 31);

	// Enable USB HS Peripheral clock
	RCC->AHBPCENR |= RCC_AHBPeriph_USBHS;

	// Setup EP config
	dev->status.data_buf = buffer;
	dev->status.data_maxsize = bsize-8; // remove header
	dev->status.ep0size = ep0size;
}

void usbd_ep0_config(usbd_device *dev)
{
	dev->status.device_cfg = 0;
	dev->status.device_state = usbd_state_default;
	dev->state = usbd_ctl_idle;

	// Disable all but EP0
	USBHSD->ENDP_CONFIG = 0;

	USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
	USBHSD->UEP0_MAX_LEN = dev->status.data_maxsize;
	USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
	USBHSD->UEP0_TX_LEN = 0;
	USBHSD->UEP0_DMA = (uint16_t)(intptr_t)dev->status.data_buf;
}

void usbd_enable(usbd_device *dev, bool enable)
{
	USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
	// Recommended to prevent issues once connected to USB PHY 
	GPIO_CFG(GPIOB, GPIO_PIN_6, GPIO_AF_PP_OUT | GPIO_Speed_2MHz);
	GPIO_CFG(GPIOB, GPIO_PIN_7, GPIO_AF_PP_OUT | GPIO_Speed_2MHz);
	delayUS(10);
	USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;
	if (enable)
	{
		USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
		USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_SPEED_HIGH | USBHS_UC_INT_BUSY;
		USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_SOF_ACT | USBHS_UIE_TRANSFER
			 | USBHS_UIE_BUS_RST | USBHS_UIE_SUSPEND | USBHS_UIE_FIFO_OV; // USBHS_UIE_DEV_NAK // USBHS_UIE_ISO_ACT

		usbd_ep0_config(dev);

		// Signal to PC that we are connected
		USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
	}
	else
	{
		USBHSD->CONTROL &= ~USBHS_UC_DEV_PU_EN;
	}
}

void usbd_ep_rx_config(usbd_device *dev, uint8_t ep, uint8_t *dma_buf, uint16_t epsize)
{
	ep = ep&0xF;
	EP_RX_CTRL(ep) = USBHS_UEP_R_RES_ACK;
	EPx_RX_DMA(ep) = (uint32_t)dma_buf;
	EP_MAX_LEN(ep) = epsize;
	USBHSD->ENDP_CONFIG |= 0x00010000 << ep;
}

void usbd_ep_tx_config(usbd_device *dev, uint8_t ep)
{
	ep = ep&0xF;
	EP_TX_CTRL(ep) = USBHS_UEP_T_RES_NAK;
	EP_TX_LEN(ep) = 0;
	dev->ep_set[ep] = false;
	USBHSD->ENDP_CONFIG |= 0x00000001 << ep;

	// TODO: Consider using isochronous transfers with synchronous buffer mode with:
	//USBHSD->ENDP_TYPE |= 0x00000001 << ep; // Mark as synchronous (e.g. isochronous)
	//USBHSD->BUF_MODE |= 0x00010000 << ep;
	// R32_UEP_BUF_MOD, R32_UEP_TYPE, R32_UEP_CONFIG in docs
	// This allows double buffered communication, so that RX buffer (DMA + MaxLen) is copied to TX buffer (DMA+TXLen) after each SOF (microframe)
	// Increases reaction time from 125us to 250us for USB HS
}

void usbd_ep_deconfig(usbd_device *dev, uint8_t ep)
{
	ep = ep&0xF;
	EP_TX_CTRL(ep) = USBHS_UEP_T_RES_STALL;
	EP_TX_LEN(ep) = 0;
	EPx_TX_DMA(ep) = 0;
	EP_RX_CTRL(ep) = USBHS_UEP_R_RES_STALL;
	EPx_RX_DMA(ep) = 0;
	EP_MAX_LEN(ep) = 0;
	USBHSD->ENDP_CONFIG &= ~(1 << ep) & ~(1 << (ep+8));
}

bool usbd_ep_set_dma(usbd_device *dev, uint8_t ep, uint8_t *buf, uint16_t len)
{
	ep = ep&0xF;
	if (dev->ep_set[ep])
	{
		ERR_STR("\n#EpSet");
		return false;
	}
	LOG_EVT_USB(CONTROLLER_EVENT_USB_DATA_TX, true);
	EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
	EPx_TX_DMA(ep) = (uint32_t)buf;
	EP_TX_LEN(ep) = len;
	EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
	return true;
}

void usbd_ep_reset_dma(usbd_device *dev, uint8_t ep)
{
	ep = ep&0xF;
	EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
	EPx_TX_DMA(ep) = 0;
	EP_TX_LEN(ep) = 0;
	dev->ep_set[ep] = false;
}


// Copied following serial number generator from 96-bit GUID from libusb_stm32

static uint32_t fnv1a32_turn (uint32_t fnv, uint32_t data ) {
	for (int i = 0; i < 4 ; i++) {
		fnv ^= (data & 0xFF);
		fnv *= 16777619;
		data >>= 8;
	}
	return fnv;
}

static uint16_t get_serialno_desc(void *buffer) {
	struct usb_string_descriptor *dsc = buffer;
	uint16_t *str = dsc->wString;
	uint32_t fnv = 2166136261;
	fnv = fnv1a32_turn(fnv, ESIG->UNIID1);
	fnv = fnv1a32_turn(fnv, ESIG->UNIID2);
	fnv = fnv1a32_turn(fnv, ESIG->UNIID3);
	for (int i = 28; i >= 0; i -= 4 ) {
		uint16_t c = (fnv >> i) & 0x0F;
		c += (c < 10) ? '0' : ('A' - 10);
		*str++ = c;
	}
	dsc->bDescriptorType = USB_DTYPE_STRING;
	dsc->bLength = 18;
	return 18;
}

static usbd_respond std_control(usbd_device *dev, usbd_ctlreq *req)
{
	usbd_respond res = usbd_fail;
	uint8_t ep = dev->ctl_setup.wIndex&0x7;
	switch (dev->ctl_setup.bmRequestType & (USB_REQ_TYPE | USB_REQ_RECIPIENT))
	{
	case USB_REQ_STANDARD | USB_REQ_DEVICE:
	{
		switch (dev->ctl_setup.bRequest)
		{
		case USB_STD_GET_DESCRIPTOR:
			USBD_STR("+Desc:");
			USBD_CHARR(UI8_TO_HEX_ARR(dev->ctl_setup.wValue>>8));
			if (dev->ctl_setup.wValue == ((USB_DTYPE_STRING << 8) | INTSERIALNO_DESCRIPTOR ))
			{
				dev->status.data_count = get_serialno_desc(dev->status.data_buf);
				dev->status.data_ptr = dev->status.data_buf;
				if (dev->status.data_count <= dev->status.data_maxsize)
					res = usbd_ack; // Should always happen, returns 18 - if not it already overwrote memory out of bounds
				else
					ERR_CHARR('/', 'D', 'C', 'S', INT99999_TO_CHARR(dev->status.data_count));
			}
			else
			{
				res = class_impl_getdesc(dev, &dev->ctl_setup, (void**)&dev->status.data_ptr, &dev->status.data_count);
				if (res == usbd_fail)
				{
					USBD_STR("!Failed");
				}
				//else
				//	USBD_STR("+Handled");
			}
			break;
		case USB_STD_GET_CONFIG:
			dev->status.data_buf[0] = dev->status.device_cfg;
			dev->status.data_ptr = dev->status.data_buf;
			dev->status.data_count = 1;
			res = usbd_ack;
			USBD_STR("+GetConfig");
			break;
		case USB_STD_SET_CONFIG:
			res = class_impl_setconf(dev, dev->ctl_setup.wValue & 0xFF);
			if (res == usbd_ack)
			{
				dev->status.device_cfg = (uint8_t)(dev->ctl_setup.wValue & 0xFF);
				dev->status.device_state = dev->status.device_cfg? usbd_state_configured : usbd_state_addressed;
				USBD_STR("+SetConfig");
			}
			else
				USBD_STR("!FailedSetConfig");
			break;
		case USB_STD_GET_STATUS:
			dev->status.data_buf[0] = 0;
			dev->status.data_buf[1] = 0;
			dev->status.data_ptr = dev->status.data_buf;
			dev->status.data_count = 2;
			res = usbd_ack;
			USBD_STR("+DevGetStatus");
			break;
		case USB_STD_SET_ADDRESS:
			if (dev->status.device_state != usbd_state_addressed && dev->status.device_state != usbd_state_configured)
			{
				USBD_STR("+SetAddress");
				res = usbd_ack; // TODO: Accept always?
			}
			else
			{
				USBD_STR("\n!RepeatAddress");
			}
			// Set later once transfer is acknowledged
			break;
		case USB_STD_SET_DESCRIPTOR:
			/* should be externally handled */
			USBD_STR("!SetDesc");
			break;
		case USB_STD_CLEAR_FEATURE:
			/* not yet supported */
			USBD_STR("!ClearFeature");
			break;
		case USB_STD_SET_FEATURE:
			/* not yet supported */
			USBD_STR("!SetFeature");
			break;
		default:
			USBD_STR("!UnknownDevReq:");
			USBD_CHARR(INT999_TO_CHARR(dev->ctl_setup.bRequest));
			break;
		}
		break;
	}
	case USB_REQ_STANDARD | USB_REQ_INTERFACE:
		if (dev->ctl_setup.bRequest == USB_STD_GET_STATUS)
		{
			dev->status.data_buf[0] = 0;
			dev->status.data_buf[1] = 0;
			dev->status.data_ptr = dev->status.data_buf;
			dev->status.data_count = 2;
			res = usbd_ack;
			USBD_STR("+IntGetStatus:");
		}
		else
		{
			USBD_STR("!UnknownStdIntReq:");
			USBD_CHARR(INT999_TO_CHARR(dev->ctl_setup.bRequest));
		}
		break;
	case USB_REQ_STANDARD | USB_REQ_ENDPOINT:
		switch (dev->ctl_setup.bRequest)
		{
		case USB_STD_SET_FEATURE:
			if ((dev->ctl_setup.wValue & 0xFF) == USB_FEAT_ENDPOINT_HALT)
			{
				if (dev->ctl_setup.wIndex & USB_EPDIR_IN)
					EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_STALL;
				else
					EP_TX_CTRL(ep) = (EP_RX_CTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_STALL;
				USBD_STR("+EPSetHalt:");
				USBD_CHARR(INT999_TO_CHARR(ep));
			}
			else
			{
				USBD_STR("!UnknownEPSetFeature:");
				USBD_CHARR(INT999_TO_CHARR(dev->ctl_setup.wValue & 0xFF));
			}
			res = usbd_ack;
			break;
		case USB_STD_CLEAR_FEATURE:
			if ((dev->ctl_setup.wValue & 0xFF) == USB_FEAT_ENDPOINT_HALT)
			{
				if (dev->ctl_setup.wIndex & USB_EPDIR_IN)
					EP_RX_CTRL(dev->ctl_setup.wIndex&0x7) = USBHS_UEP_R_RES_NAK;
				else
					EP_TX_CTRL(dev->ctl_setup.wIndex&0x7) = USBHS_UEP_T_RES_NAK;
				USBD_STR("+EPClearHalt:");
				USBD_CHARR(INT999_TO_CHARR(ep));
			}
			else
			{
				USBD_STR("!UnknownEPClearFeature:");
				USBD_CHARR(INT999_TO_CHARR(dev->ctl_setup.wValue & 0xFF));
			}
			res = usbd_ack;
			break;
		case USB_STD_GET_STATUS:
			USBD_STR("+EPGetStatus:");
				USBD_CHARR(INT999_TO_CHARR(ep));
			if (dev->ctl_setup.wIndex & USB_EPDIR_IN)
				dev->status.data_buf[0] = (EP_RX_CTRL(dev->ctl_setup.wIndex&0x7) & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_STALL? 1 : 0;
			else
				dev->status.data_buf[0] = (EP_TX_CTRL(dev->ctl_setup.wIndex&0x7) & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_STALL? 1 : 0;
			dev->status.data_buf[1] = 0;
			dev->status.data_ptr = dev->status.data_buf;
			dev->status.data_count = 2;
			res = usbd_ack;
			break;
		default:
			USBD_STR("!UnknownEPReq:");
			USBD_CHARR(INT999_TO_CHARR(dev->ctl_setup.bRequest));
			break;
			break;
		}
		break;
	default:
		USBD_STR("!UnknownReqType:");
		USBD_CHARR(INT999_TO_CHARR(dev->ctl_setup.bmRequestType));
		break;
	}
	return res;
}

void usbd_poll(usbd_device *dev)
{
	if (USBHSD->INT_FG & USBHS_UIF_TRANSFER)
	{
		uint8_t usbStatus = USBHSD->INT_ST;
		switch (usbStatus & USBHS_UIS_TOKEN_MASK)
		{
		// Data IN - Device To Host
		case USBHS_UIS_TOKEN_IN:
		{
			uint8_t ep = usbStatus & USBHS_UIS_ENDP_MASK;
			if (ep == 0)
			{
				USBD_STR("\nSend:");
				USBD_CHARR(INT9999_TO_CHARR(USBHSD->UEP0_TX_LEN));
				if (dev->status.data_count == 0 && dev->status.data_sending != dev->status.ep0size)
				{ // Done with Control IN transfer (TX)

					if (dev->state == usbd_ctl_statusout)
					{ // Sent out a ZLP as positive statusout packet
						USBD_STR("+ACK");

						if ((dev->ctl_setup.bmRequestType & USB_REQ_TYPE) == USB_REQ_STANDARD
						 && (dev->ctl_setup.bmRequestType & USB_REQ_RECIPIENT) == USB_REQ_DEVICE
						 && (dev->ctl_setup.bRequest == USB_STD_SET_ADDRESS))
						{
							USBHSD->DEV_AD = dev->ctl_setup.wValue & 0xFF;
							dev->status.device_state = USBHSD->DEV_AD? usbd_state_addressed : usbd_state_default;
						}
						// usbd_ctl_statusout never gets a resolution except for set_address

						// Prepare RX for setup
						dev->state = usbd_ctl_idle;
						LOG_EVT_USB(CONTROLLER_EVENT_USB_CONTROL, false);
					}
					else if (dev->state == usbd_ctl_txdata)
					{ // Send out a data packet
						USBD_STR("+Done");
						USBD_CHARR(':', UI8_TO_HEX_ARR(usbStatus));
						dev->state = usbd_ctl_statusin; // Expecting OUT token for statusin phase, receiving a valid ZLP would be a positive status packet
					}
					else if (dev->state == usbd_ctl_statusin)
					{ // Duplicate event after the last TXDone whenever the transfer involved multiple data packets
						KERR_STR("\n!RepeatTX");
						KERR_CHARR(':', UI8_TO_HEX_ARR(usbStatus));
					}
					else
					{ // Invalid state
						ERR_STR("\n#TX0What:");
						ERR_CHARR(UI8_TO_HEX_ARR(dev->state));
						USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_STALL;
						break;
					}

					// Not expecting any further IN tokens
					USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;

					// Prepare RX for setup (or respond with ACK to statusin packet)
					USBHSD->UEP0_DMA = (uint16_t)(intptr_t)dev->status.data_buf;
					USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
					USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
				}
				else
				{ // Continue ongoing Control IN transfer (TX), either with more data or a last zero-length-packet
					if ((USBHSD->UEP0_TX_CTRL & USBHS_UEP_T_RES_MASK) != USBHS_UEP_T_RES_ACK)
					{ // Not sure what this means
						// TODO: May have to retransmit the last packet?
						// We know how much to retrace from dev->status.data_sending
						ERR_STR("\n#TNAK");
						USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_STALL;
						break;
					}

					if (dev->state != usbd_ctl_txdata)
					{ // Invalid State
						ERR_STR("\n#TXWhat:");
						USBD_CHARR(UI8_TO_HEX_ARR(dev->state));
						USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_STALL;
						break;
					}

					if (dev->status.data_count == 0)
					{ // Need to send another ZLP to signal that this is the end
						// Since (dev->status.data_sending == dev->status.ep0size)
						// Just sent a full packet so host needs to know if this is the end of it
						USBD_STR("+ZLP:");
					}
					else
						USBD_STR("+Cont:");

					uint16_t len = dev->status.data_count >= dev->status.ep0size? dev->status.ep0size : dev->status.data_count;
					USBHSD->UEP0_DMA = (uint16_t)(intptr_t)dev->status.data_ptr;
					USBHSD->UEP0_TX_LEN = len;
					USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
					dev->status.data_ptr += len;
					dev->status.data_count -= len;
					dev->status.data_sending = len;
					USBD_CHARR(INT99_TO_CHARR(len));
				}
			}
			else
			{
				if (USBHSD->ENDP_CONFIG & (1 << ep))
				{
					if ((usbStatus & USBHS_UIS_IS_NAK) == USBHS_UIS_IS_NAK)
					{
						KERR_STR("\n!IntTxNAK");
						LOG_EVT_USB(CONTROLLER_EVENT_USB_DATA_TX, false);
						EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
						EP_TX_CTRL(ep) ^= USBHS_UEP_T_TOG_DATA1;
						dev->ep_set[ep] = false;
						usbd_EP_TX_CB(dev, ep, false);
					}
					else
					{
						LOG_EVT_USB(CONTROLLER_EVENT_USB_DATA_TX, false);
						EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
						EP_TX_CTRL(ep) ^= USBHS_UEP_T_TOG_DATA1;
						dev->ep_set[ep] = false;
						usbd_EP_TX_CB(dev, ep, true);
					}
				}
				else
				{
					ERR_STR("\n#SentEPInvalid:");
					USBD_CHARR(INT99_TO_CHARR(ep));
					if (ep > 0 && ep < 16)
						EP_TX_CTRL(ep) = (EP_TX_CTRL(ep) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_STALL;
				}
			}
			break;
		}
		// Data OUT - Host To Device
		case USBHS_UIS_TOKEN_OUT:
		{
			uint8_t ep = usbStatus & USBHS_UIS_ENDP_MASK;
			uint16_t rxLen = USBHSD->RX_LEN;
			if (ep == 0)
			{
				USBD_STR("\nRecv");
				if ((usbStatus & USBHS_UIS_IS_NAK) == USBHS_UIS_IS_NAK)
				{ // TODO: Figure out what NAK during receive means - it's usually accompannied by bad TOG_OK
					// May have to do with time spent in usbd_poll? Adding logging (even via SDI) causes many more to appear
					ERR_STR("\n#RNAK");
					//break; // Stopping here breaks control flow hardware expects
				}
				if (rxLen == 0)
				{
					if (dev->state != usbd_ctl_statusin)
					{
						ERR_STR("\n#RX0What:");
						USBD_CHARR(UI8_TO_HEX_ARR(dev->state));
						USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_STALL;
						break;
					}

					// Got positive statusin packet, our TX transfer was successful
					USBD_STR("+ACK");
					class_impl_control_resolution(dev, &dev->ctl_setup, true);

					// Prepare RX for setup
					USBHSD->UEP0_DMA = (uint16_t)(intptr_t)dev->status.data_buf;
					USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
					USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
					dev->state = usbd_ctl_idle;
					LOG_EVT_USB(CONTROLLER_EVENT_USB_CONTROL, false);
				}
				else
				{
					USBD_CHARR(':', INT999_TO_CHARR(rxLen));

					if ((usbStatus & USBHS_UIS_TOG_OK) == 0)
					{ // TODO: Figure out how to avoid this - usually happens after a RNAK
						ERR_STR("\n#NOTOK:");
						ERR_CHARR(INT999_TO_CHARR(rxLen), '/', INT9999_TO_CHARR(dev->status.data_count_rx), '\n');
						// USB HW should have already responded with a NAK, packet will be retried later
						//break; // Stopping here breaks control flow hardware expects
					}

					if (dev->state != usbd_ctl_rxdata)
					{
						ERR_STR("\n#RXWhat:");
						USBD_CHARR(UI8_TO_HEX_ARR(dev->state), ':', INT999_TO_CHARR(rxLen), '/', INT9999_TO_CHARR(dev->status.data_count_rx));
						break;
					}

					if (dev->status.data_count_rx < rxLen)
					{
						ERR_STR("\n#RXOverLength");
						dev->status.data_count_rx = 0;
					}
					else
						dev->status.data_count_rx -= rxLen;
					if (dev->status.data_count_rx == 0)
					{ // Finished RX transfer
						USBD_STR("+Done");
						usbd_respond res = class_impl_control(dev, (usbd_ctlreq *)dev->status.data_buf);

						if (res == usbd_fail)
						{ // Stall in response to statusout to signal RX had an issue
							ERR_STR("\n#Invalid");
							USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL; // Signals error occurred
							//USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK; // Signals still processing
							// TODO: Test
							USBHSD->UEP0_RX_CTRL = USBHS_UEP_T_RES_STALL;

							// Prepare RX for setup
							USBHSD->UEP0_DMA = (uint16_t)(intptr_t)dev->status.data_buf;
							USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
							dev->state = usbd_ctl_idle;
							LOG_EVT_USB(CONTROLLER_EVENT_USB_CONTROL, false);
							// TODO: Prepare for setup here?
						}
						else 
						{ // Send zero-length packet in response to statusout to signal RX was ok
							USBD_STR("+ZLP");
							USBHSD->UEP0_TX_LEN = 0;
							USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK; // Status always uses DATA1
							dev->state = usbd_ctl_statusout; // Expecting IN token for statusout phase, send ZLP as positive status packet

							// Not expecting any further OUT tokens
							USBHSD->UEP0_RX_CTRL = USBHS_UEP_T_RES_STALL;
						}
					}
					else
					{ // Continue writing to end of existing data
						USBHSD->UEP0_DMA += rxLen;
						USBHSD->UEP0_RX_CTRL ^= USBHS_UEP_R_TOG_DATA1;
						USBHSD->UEP0_RX_CTRL = (USBHSD->UEP0_RX_CTRL & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
						dev->state = usbd_ctl_rxdata; // Expecting more OUT tokens

						USBD_STR("+Cont:");
						USBD_CHARR(INT9999_TO_CHARR(dev->status.data_count_rx));
					}
				}
			}
			else
			{
				USBD_STR("\nTokenOUT:");
				if (USBHSD->ENDP_CONFIG & (1 << (ep+8)))
				{
					//USBD_CHARR('+', 'R', 'X', 'E', 'P', INT99_TO_CHARR(ep));
					if ((usbStatus & USBHS_UIS_TOG_OK) == USBHS_UIS_TOG_OK)
					{
						EP_RX_CTRL(ep) ^= USBHS_UEP_R_TOG_DATA1;
						EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
						// Starts at beginning of buffer next - might need to set later DMA pos to continue
						//EPx_RX_DMA(ep) = ...
						usbd_EP_RX_CB(dev, ep, rxLen);
					}
					else
					{
						KERR_STR("\n#IsoRXNotOK");
						if (ep > 0 && ep < 16)
							EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_STALL;
					}
				}
				else
				{
					ERR_STR("\n#RecvEPNotExist:");
					USBD_CHARR(INT99_TO_CHARR(ep));
					if (ep > 0 && ep < 16)
						EP_RX_CTRL(ep) = (EP_RX_CTRL(ep) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_STALL;
				}
			}
			break;
		}
		case USBHS_UIS_TOKEN_SOF:
		{
			LOG_EVT_USB(CONTROLLER_EVENT_USB_SOF, true);
			usbd_SOF_CB(dev);
			break;
		}
		default:
		{
			ERR_STR("\n!UTK");
			ERR_CHARR(':', UI8_TO_HEX_ARR(usbStatus));
			break;
		}
		}
		USBHSD->INT_FG = USBHS_UIF_TRANSFER; // Clear flag
	}
	if (USBHSD->INT_FG & USBHS_UIF_SETUP_ACT)
	{ // Setup token on ep0 - initiating a new control transfer

		if (dev->state != usbd_ctl_idle)
		{ // Another control transfer must've gotten interrupted, have to accept this new one
			// This should not happen anymore, if this pops up again, something broke
			ERR_STR("\n#CtrlInt");
			ERR_CHARR(':', INT9_TO_CHARR((uint8_t)dev->state));

			if (dev->state != usbd_ctl_statusout && dev->state != usbd_ctl_statusin)
			{ // Set negative resolution for still ongoing transfers
				class_impl_control_resolution(dev, &dev->ctl_setup, false);
			}

			if (USBHSD->UEP0_DMA != ((uint16_t)(intptr_t)dev->status.data_buf))
			{ // Retractively fix, copy into start of buffer and fix DMA to it
				USBD_STR("+FixDMA");
				memcpy(dev->status.data_buf, (uint8_t*)(0x20000000+USBHSD->UEP0_DMA), sizeof(usbd_ctlreq));
				USBHSD->UEP0_DMA = (uint16_t)(intptr_t)dev->status.data_buf;
			}
		}
		dev->status.data_count = 0;
		dev->status.data_sending = 0;
		dev->status.data_count_rx = 0;

		LOG_EVT_USB(CONTROLLER_EVENT_USB_CONTROL, true);
		USBD_STR("\nSetup");

		// Reset state
		USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
		USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_NAK;

		// Read header
		dev->ctl_setup = *(usbd_ctlreq *)dev->status.data_buf;

		usbd_respond res = usbd_fail;
		if ((dev->ctl_setup.bmRequestType & USB_REQ_DIRECTION) == USB_REQ_HOSTTODEV)
		{ // Host to Device, always accept and handle once all data is there

			USBD_STR("+OUT");

			if (dev->ctl_setup.wLength == 0)
			{ // No data, just setup header
				USBD_STR("+Header:");
				USBD_CHARR(INT999_TO_CHARR(dev->ctl_setup.bRequest));
				res = class_impl_control(dev, (usbd_ctlreq *)dev->status.data_buf);

				if (res == usbd_ack)
				{
					USBHSD->UEP0_TX_LEN = 0;
					USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
					dev->state = usbd_ctl_statusout; // Expecting IN token for statusout phase, send ZLP as positive status packet
					USBD_STR("+ZLP");
				}
				else
				{
					KERR_STR("\n#FailedReqToDev");
					KERR_CHARR('+', UI8_TO_HEX_ARR(dev->ctl_setup.bmRequestType));
					//USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_STALL; // Signals error occured
					//USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK; // Signals still processing
					//dev->state = usbd_ctl_idle;
					// TODO: Stall doesn't seem to work, send ZLP as statusout (ACK) instead for now
					USBHSD->UEP0_TX_LEN = 0;
					USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
					dev->state = usbd_ctl_statusout; // Expecting IN token for statusout phase, send ZLP as "positive" status packet
				}
			}
			else if (dev->ctl_setup.wLength > dev->status.data_maxsize)
			{ // Can't read full data, stall
				ERR_STR("\n#RXTooLong:");
				USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
				USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
				dev->state = usbd_ctl_idle;
				LOG_EVT_USB(CONTROLLER_EVENT_USB_CONTROL, false);
			}
			else 
			{ // Ready to receive data of Control OUT transfer
				USBD_STR("+RX:");
				USBD_CHARR(INT9999_TO_CHARR(dev->ctl_setup.wLength));

				USBHSD->UEP0_DMA += sizeof(usbd_ctlreq); // Write data after header
				USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
				dev->status.data_count_rx = dev->ctl_setup.wLength;
				dev->state = usbd_ctl_rxdata; // Expect OUT tokens to receive data
			}
		}
		else 
		{ // Device to Host, handle setup header to gather data to send, or deny

			USBD_STR("+IN");

			// Check custom control handling first
			res = class_impl_control(dev, (usbd_ctlreq *)dev->status.data_buf);

			if (res == usbd_fail)
			{ // Handle standard requests in driver
				res = std_control(dev, (usbd_ctlreq *)dev->status.data_buf);
			}

			if (res == usbd_fail)
			{ // Request not supported, stall TX
				ERR_STR("\n#FailStall:");
				USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;

				// Prepare RX for setup
				USBHSD->UEP0_DMA = (uint16_t)(intptr_t)dev->status.data_buf;
				USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
				//USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
				dev->state = usbd_ctl_idle;
				LOG_EVT_USB(CONTROLLER_EVENT_USB_CONTROL, false);
			}
			else if (dev->status.data_count == 0)
			{ // No data to send, can either send zero-length-packet or NAK to signal no data is available
				USBD_STR("+TX0");
				USBHSD->UEP0_TX_LEN = 0;
				USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
				dev->state = usbd_ctl_txdata; // Expecting IN token to send zero-length-packet as data
				// TODO: Test if NAK also works, should
				//USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
				//dev->state = usbd_ctl_idle;
			}
			else
			{ // Transmit data in response to future IN tokens
				USBD_STR("+TX");
				USBD_CHARR(':', INT9999_TO_CHARR(dev->status.data_count));

				// Limit size if required
				uint16_t orig = dev->status.data_count;
				if (dev->status.data_count > dev->ctl_setup.wLength)
					dev->status.data_count = dev->ctl_setup.wLength;
				if (dev->status.data_count > dev->status.data_maxsize)
					dev->status.data_count = dev->status.data_maxsize;
				if (orig != dev->status.data_count)
					USBD_CHARR('!', INT9999_TO_CHARR(dev->status.data_count));
				
				// USB DMA can only access addresses in SRAM and needs them to be 4-byte aligned
				/* if (((uint32_t)dev->status.data_ptr)&USB_PACKET_ALIGNMENT)
					ERR_CHARR('+', 'A', 'L', 'G', 'N'); // Needs to be 4-byte aligned
				if ((((uint32_t)dev->status.data_ptr)&0x20000000) != 0x20000000)
					ERR_CHARR('+', 'C', 'N', 'S', 'T'); // Must be stored in SRAM, not flash (e.g. constant buffers)*/
				
				// So just default to copying to buffer
					// Descriptors are in flash and need to be copied (or set to non-const)
					// Debug needs to be copied anyway and is generally unaligned
					// Some responses are setup in EP0 buffer already, don't copy those
				if (dev->status.data_buf != dev->status.data_ptr)
				{
					memcpy(dev->status.data_buf, dev->status.data_ptr, dev->status.data_count);
				}
				dev->status.data_ptr = dev->status.data_buf;
				
				// Send as much as EP0 allows
				uint16_t len = (dev->status.data_count > dev->status.ep0size)? dev->status.ep0size : dev->status.data_count;
				USBHSD->UEP0_TX_LEN = len;
				USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
				dev->status.data_ptr += len;
				dev->status.data_count -= len;
				dev->status.data_sending = len;
				if (dev->status.data_count > 0)
					USBD_STR("+Cont");

				dev->state = usbd_ctl_txdata; // Expect IN tokens to send that data
			}
		}
		USBHSD->INT_FG = USBHS_UIF_SETUP_ACT; // Clear flag
	}
	if (USBHSD->INT_FG & USBHS_UIF_ISO_ACT)
	{
		USBHSD->INT_FG = USBHS_UIF_ISO_ACT; // Clear flag
		// TODO: Isochronous starts sending, once isochronous transfers are used
	}
	if (USBHSD->INT_FG & USBHS_UIF_BUS_RST)
	{ // Reset
		USBHSD->INT_FG = USBHS_UIF_BUS_RST; // Clear flag
		USBD_STR("/Reset");
		USBHSD->DEV_AD = 0;
		usbd_ep0_config(dev);
	}
	if (USBHSD->INT_FG & USBHS_UIF_SUSPEND)
	{ // Suspend is not supported
		USBHSD->INT_FG = USBHS_UIF_SUSPEND; // Clear flag
		USBD_STR("/Suspend");
	}
	if (USBHSD->INT_FG & USBHS_UIF_FIFO_OV)
	{ // Fifo Overflow is not supported
		USBHSD->INT_FG = USBHS_UIF_FIFO_OV; // Clear flag
		ERR_STR("\n!FifoOverflow");
	}
	/* if (USBHSD->INT_FG)
	{ // Next interrupt was already loaded
		USBD_STR("^Recursion");
		usbd_poll(dev);
	} */
}