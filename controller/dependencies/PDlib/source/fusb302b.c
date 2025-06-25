/*
 * PD Buddy Firmware Library - USB Power Delivery for everyone
 * Copyright 2017-2018 Clayton G. Hobbs
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "fusb302b.h"
#include "fusb302_defines.h"
#include <pd.h>
#ifdef PD_DEBUG_OUTPUT
#include "stdio.h"
#endif

#include "util.h"


I2CFunc I2CRead;
I2CFunc I2CWrite;
TimestampFunc getTimeStamp;
DelayFunc osDelay;

/*
 * Read a single byte from the FUSB302B
 *
 * cfg: The FUSB302B to communicate with
 * addr: The memory address from which to read
 *
 * Returns the value read from addr.
 */
static inline uint8_t fusb_read_byte(FUSB302 *fusb, const uint8_t addr) {
	uint8_t data[1];
	if (!I2CRead(fusb->DeviceAddress, addr, 1, (uint8_t*)data))
		return 0;
	return data[0];
}

/*
 * Write a single byte to the FUSB302B
 *
 * cfg: The FUSB302B to communicate with
 * addr: The memory address to which we will write
 * byte: The value to write
 */
static inline bool fusb_write_byte(FUSB302 *fusb, const uint8_t addr, const uint8_t byte)
{
	return I2CWrite(fusb->DeviceAddress, addr, 1, (uint8_t*)&byte);
}

void fusb_send_message(FUSB302 *fusb, const pd_msg *msg)
{
	// Token sequences for the FUSB302B
	static uint8_t       sop_seq[5] = {FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP1, FUSB_FIFO_TX_SOP2, FUSB_FIFO_TX_PACKSYM};
	static const uint8_t eop_seq[4] = {FUSB_FIFO_TX_JAM_CRC, FUSB_FIFO_TX_EOP, FUSB_FIFO_TX_TXOFF, FUSB_FIFO_TX_TXON};

	// Get the length of the message: a two-octet header plus NUMOBJ four-octet data objects
	uint8_t msg_len = 2 + 4 * PD_NUMOBJ_GET(msg);

	// Set the number of bytes to be transmitted in the packet
	sop_seq[4] = FUSB_FIFO_TX_PACKSYM | msg_len;

	// Write all three parts of the message to the TX FIFO
	bool result = I2CWrite(fusb->DeviceAddress, FUSB_FIFOS, 5, sop_seq);
	if (!result) {
#ifdef PD_DEBUG_OUTPUT
		printf("I2CWrite failed 1\r\n");
#endif
	}
	result = I2CWrite(fusb->DeviceAddress, FUSB_FIFOS, msg_len, (uint8_t *)msg->bytes);
	if (!result) {
#ifdef PD_DEBUG_OUTPUT
		printf("I2CWrite failed 2\r\n");
#endif
	}

	result = I2CWrite(fusb->DeviceAddress, FUSB_FIFOS, 4, (uint8_t *)eop_seq);
	if (!result) {
#ifdef PD_DEBUG_OUTPUT
		printf("I2CWrite failed 3\r\n");
#endif
	}
}

bool fusb_rx_pending(FUSB302 *fusb)
{
	return (fusb_read_byte(fusb, FUSB_STATUS1) & FUSB_STATUS1_RX_EMPTY) != FUSB_STATUS1_RX_EMPTY;
}

uint8_t fusb_read_message(FUSB302 *fusb, pd_msg *msg)
{
	static uint8_t garbage[4];
	uint8_t numobj;

	// Read the header. If its not a SOP we dont actually want it at all
	// But on some revisions of the fusb if you dont both pick them up and read
	// them out of the fifo, it gets stuck
	// TODO this might need a tad more testing about how many bites we throw out, but believe it is correct
	uint8_t returnValue = 0;
	if ((fusb_read_byte(fusb, FUSB_FIFOS) & FUSB_FIFO_RX_TOKEN_BITS) != FUSB_FIFO_RX_SOP)
		returnValue = 1;

	// Read the message header into msg
	I2CRead(fusb->DeviceAddress, FUSB_FIFOS, 2, msg->bytes);
	// Get the number of data objects
	numobj = PD_NUMOBJ_GET(msg);
	// If there is at least one data object, read the data objects
	if (numobj > 0)
		I2CRead(fusb->DeviceAddress, FUSB_FIFOS, numobj * 4, msg->bytes + 2);
	// Throw the CRC32 in the garbage, since the PHY already checked it.
	I2CRead(fusb->DeviceAddress, FUSB_FIFOS, 4, garbage);
	USBPD_CHARR('/', 'P', 'K', 'T');
	ERR_HEX(msg->bytes, 2+(numobj*4));
	USBPD_CHARR('+', 'C', 'R', 'C');
	ERR_HEX(garbage, 4);

	return returnValue;
}

void fusb_send_hardrst(FUSB302 *fusb)
{
	fusb_write_byte(fusb, FUSB_CONTROL3, 0x07 | FUSB_CONTROL3_SEND_HARD_RESET);
}

bool fusb_setup(FUSB302 *fusb)
{
	// Fully reset the FUSB302B
	//if (!fusb_write_byte(fusb, FUSB_RESET, FUSB_RESET_SW_RES))
	//	return false;
	//osDelay(10000);
	uint8_t tries = 0;
	while (!fusb_read_id(fusb))
	{
		osDelay(1000);
		tries++;
		if (tries > 5)
			return false; // Welp :(
	}

	// Turn on all power if it isn't already
	if (!fusb_write_byte(fusb, FUSB_POWER, 0x0F))
		return false;

	// Set interrupt masks to 0 so interrupts are allowed
	if (!fusb_write_byte(fusb, FUSB_MASK1, 0x01)) // TODO: Changed to mask I_BC_LVL
		return false;
	if (!fusb_write_byte(fusb, FUSB_MASKA, 0x00))
		return false;
	if (!fusb_write_byte(fusb, FUSB_MASKB, 0x00))
		return false;
	if (!fusb_write_byte(fusb, FUSB_CONTROL0, 0b11 << 2)) // Set HOST_CUR to High Current Mode (3A) (why?)
		return false;

	// Enable automatic retransmission
	if (!fusb_write_byte(fusb, FUSB_CONTROL3, 0x07))
		return false;
	// set defaults
	//if (!fusb_write_byte(fusb, FUSB_CONTROL2, 0x00))
	//	return false;
	// Flush the RX buffer
	//if (!fusb_write_byte(fusb, FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH))
	//	return false; // TODO: This the reason it doesn't autoconnect when a PD was already connected? Later in fusb_reset, too

	// Check if it's already initialised (e.g. after reset)
	//uint8_t measureCC = fusb_read_byte(fusb, FUSB_SWITCHES0) & (FUSB_SWITCHES0_MEAS_CC1 | FUSB_SWITCHES0_MEAS_CC2);
	uint8_t txCC = fusb_read_byte(fusb, FUSB_SWITCHES1) & (FUSB_SWITCHES1_TXCC1 | FUSB_SWITCHES1_TXCC2);
	// Setup AUTO_CRC|SPECREV0 and potential existing selected CC line
	if (!fusb_write_byte(fusb, FUSB_SWITCHES1, 0x24 | txCC))
		return false;
	
	// Measure and select CC line and set Auto CRC
	//if (fusb_runCCLineSelection(fusb) < 0)
	//	return false; // Might still result in inconclusive CC measurements (returns 1)

	//if (!fusb_reset(fusb))
	//	return false;

	return true;
}

int fusb_runCCLineSelection(FUSB302 *fusb)
{
	// Measure CC1
	if (!fusb_write_byte(fusb, FUSB_SWITCHES0, 0x07))
		return -1;
	osDelay(1000); // 1MS default
	uint8_t cc1 = fusb_read_byte(fusb, FUSB_STATUS0) & FUSB_STATUS0_BC_LVL;

	// Measure CC2
	if (!fusb_write_byte(fusb, FUSB_SWITCHES0, 0x0B))
		return -1;
	osDelay(1000); // 1MS default
	uint8_t cc2 = fusb_read_byte(fusb, FUSB_STATUS0) & FUSB_STATUS0_BC_LVL;

	// Select the correct CC line for BMC signaling; also enable AUTO_CRC
	if (cc1 > cc2)
	{
		USBPD_CHARR('/', 'C', 'C', hex[cc1], ':', hex[cc2], '>', '1');
		// TX_CC1|AUTO_CRC|SPECREV0
		if (!fusb_write_byte(fusb, FUSB_SWITCHES1, 0x25))
			return -1;
		// PWDN1|PWDN2|MEAS_CC1
		if (!fusb_write_byte(fusb, FUSB_SWITCHES0, 0x07))
			return -1;
	}
	else if (cc1 < cc2)
	{
		USBPD_CHARR('/', 'C', 'C', hex[cc1], ':', hex[cc2], '>', '2');
		// TX_CC2|AUTO_CRC|SPECREV0
		if (!fusb_write_byte(fusb, FUSB_SWITCHES1, 0x26))
			return -1;
		// PWDN1|PWDN2|MEAS_CC2
		if (!fusb_write_byte(fusb, FUSB_SWITCHES0, 0x0B))
			return -1;
	}
	else
	{
		USBPD_CHARR('/', 'C', 'C', hex[cc1], ':', hex[cc2], '?');
		return 1;
	}

	return 0;
}

bool fusb_isVBUSConnected(FUSB302 *fusb)
{
	return fusb_checkVBUS(fusb, 100); // effectively checks if it's over 0.42V
}

bool fusb_checkVBUS(FUSB302 *fusb, uint16_t minVBUS_mv)
{
	// So we want to set MEAS_VBUS to enable measuring the VBus signal
	// Then check the status
	uint8_t measureBackup  = fusb_read_byte(fusb, FUSB_MEASURE);
	uint8_t switchesBackup = fusb_read_byte(fusb, FUSB_SWITCHES0);
	// clear MEAS_CCx bits
	fusb_write_byte(fusb, FUSB_SWITCHES0, switchesBackup & 0b11110011);
	uint8_t measure = minVBUS_mv/420; // Rounds so that it will need at least that much to return true
	fusb_write_byte(fusb, FUSB_MEASURE, 0b01000000 | measure);
	osDelay(1000);
	uint8_t status = fusb_read_byte(fusb, FUSB_STATUS0);
	// Write back original value
	fusb_write_byte(fusb, FUSB_MEASURE, measureBackup);
	fusb_write_byte(fusb, FUSB_SWITCHES0, switchesBackup);
	return (status & (0b00100000)) != 0;
}

bool fusb_hasCCLineSelection(FUSB302 *fusb)
{
	return (fusb_read_byte(fusb, FUSB_SWITCHES1) & (FUSB_SWITCHES1_TXCC1 | FUSB_SWITCHES1_TXCC2)) != 0;
}

bool fusb_get_status(FUSB302 *fusb, fusb_status *status)
{
	return I2CRead(fusb->DeviceAddress, FUSB_STATUS0A, 7, status->bytes);
}

enum fusb_typec_current fusb_get_typec_current(FUSB302 *fusb)
{
	return (enum fusb_typec_current)(fusb_read_byte(fusb, FUSB_STATUS0) & FUSB_STATUS0_BC_LVL);
}

bool fusb_reset(FUSB302 *fusb)
{
	// Flush the TX buffer
	if (!fusb_write_byte(fusb, FUSB_CONTROL0, 0x44))
		return false;
	// Flush the RX buffer
	if (!fusb_write_byte(fusb, FUSB_CONTROL1, FUSB_CONTROL1_RX_FLUSH))
		return false;
	// Reset the PD logic
	if (!fusb_write_byte(fusb, FUSB_RESET, FUSB_RESET_PD_RESET))
		return false;
	return true;
}

bool fusb_read_id(FUSB302 *fusb)
{
	// Return true if read of the revision ID is sane
	uint8_t version = 0;
	bool res = I2CRead(fusb->DeviceAddress, FUSB_DEVICE_ID, 1, &version);
	if (!res)
		return res;
	if (version == 0 || version == 0xFF)
		return false;
	return true;
}
