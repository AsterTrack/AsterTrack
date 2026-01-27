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

#include "pd_driver.h"
#include "config.h"
#include "util.h"

/* #include "uartd.h"

#include "ch32v30x_usart.h"
#include "ch32v30x_dma.h" */
#include "ch32v30x_i2c.h"
#include "ch32v30x_wwdg.h"

#include "fusb302b.h"
#include "fusb302_defines.h"
#include "policy_engine.h"

bool i2c_read(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf);
bool i2c_write(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf);
void pd_delayUS(uint32_t us)
{
	TimePoint tgt = GetTimePoint();
	tgt += us * TICKS_PER_US;
	USBPD_CHARR('/', 'D', INT9999_TO_CHARR(us));
	while (GetTimePoint() < tgt)
		WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);
	// Or just
	//delayUS(us);
}
uint64_t timestampUS()
{
	return GetTimePointUS();
}

bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request);
void pdbs_dpm_get_sink_capability(pd_msg *cap, const bool isPD3);

static FUSB302 fusb;
static PolicyEngine pe;

volatile bool handleEvents = false;

int max_req_voltage = 15000;
int min_req_voltage = 10000;

inline static void log_pe_state(char conn)
{
#if USBPD_LOG
	USBPD_CHARR(conn);
	const char *stateName = pe_getStateName(pe.state);
	USBPD_BUF(stateName, strlen(stateName));
	if (pe.state == PEWaitingEvent)
	{
		USBPD_CHARR('(');
		stateName = pe_getStateName(pe.postNotificationEvalState);
		USBPD_BUF(stateName, strlen(stateName));
		USBPD_CHARR(')');
	}
#endif
}

void pd_init()
{
	pd_os_init(timestampUS, pd_delayUS, i2c_read, i2c_write);
	fusb.DeviceAddress = FUSB302B_ADDR;
	pe_init(&pe, &fusb, pdbs_dpm_get_sink_capability, pdbs_dpm_evaluate_capability, NULL, 0);

	USBPD_STR("\nPDSTP");

	// TODO: Might have to call TimersCallback every 5-10s to keep voltage coming (for PPS specifically)

	// TODO: Consider renegotiating when controller is reset instead of just assuming the PD state is correct

	// TODO: Verify PD voltage (and external power in voltage) with ADC circuit

	// TODO: Make this async somehow, it blocks everything whenever pd_delayUS is used rn
	// fine for now as it's only used at startup but worse if PD stuff needs to happen regularly / during streaming

	if (fusb_setup(&fusb))
	{ // Connected and set up FUSB, without resetting possible prior state
		if (fusb_checkVBUS(&fusb, 500))
		{ // Can attempt to select CC lines to communicate
			USBPD_STR("+VBUS_CONN");

			if (fusb_checkVBUS(&fusb, min_req_voltage))
			{ // Already set up VBUS, likely had reset - forcibly set to ready state
				USBPD_STR("+VBUS_RDY>");

				if (!fusb_hasCCLineSelection(&fusb))
				{
					USBPD_STR("+SEL_CC");
					int ccSel = fusb_runCCLineSelection(&fusb);
					if (ccSel == 0)
					{
						USBPD_STR("+CCSEL");
						pe.state = PESinkReady;
					}
					else if (ccSel < 0)
					{
						USBPD_STR("+CCFDT");
						pe.state = PESinkSendSoftReset;
					}
					else
					{
						USBPD_STR("+CCNDT");
						pe.state = PESinkSendSoftReset;
					}
				}
				else
				{
					USBPD_STR("+HAS_CC");
					pe.state = PESinkReady;
				}
			}
			else
			{
				USBPD_STR("+VBUS_CONN");
				pe.state = PESinkDiscovery;
			}

			if (pe.state == PESinkReady)
			{ // Ww can immediately renegotiate without dropping power
				USBPD_STR("+RENEGOTIATE");
				// Get status and handle any prior states
				handleEvents = pe_IRQ_occured(&pe);
				while (handleEvents)
					pd_handleAll();
				// Initiate a renegotiation
				pe_renegotiate(&pe);
				// Handle
				handleEvents = true;
				pd_handleAll();
			}
			else
			{ // There is a PD Source connected, but communications are uncertain
				// E.g. PD Source sent capabilities and timed out while controller was still powered off
				// Or controller failed negotiations before a reset
				// Either way, simulate a real re-plug
				USBPD_STR("+REPLUG");
				pe_forceReplug(&pe);
			}
		}
		else
		{ // Else wait for interrupt
			USBPD_STR("+NO_VBUS");
			pe.state = PESinkStartup;
		}

		log_pe_state('=');
	}
	else
		USBPD_STR("!PDSTP_FAILED");
}

void pd_poll()
{
	USBPD_STR("\nPDINT");
	TimePoint now = GetTimePoint();
	handleEvents = pe_IRQ_occured(&pe);
	TimeSpan lag = GetTimeSinceUS(now);
	WARN_CHARR('-', '-', INT9999_TO_CHARR(lag), '\n');
}

void pd_handleAll()
{
	if (!handleEvents && !isWaitingOnTimer(&pe))
		return;
	while (pe_thread(&pe))
		log_pe_state('+');
	if (pe.state == PEWaitingEvent && handleEvents)
		log_pe_state('+');
	handleEvents = false;
}

void pd_handleOne()
{
	if (!handleEvents && !isWaitingOnTimer(&pe))
		return;
	bool cont = pe_thread(&pe);
	if (handleEvents || cont)
		log_pe_state('+');
	handleEvents = cont;
}

void pd_renegotiate()
{
	ERR_STR("\nRenegotiating PD!");
	pe_renegotiate(&pe);
	handleEvents = true;
}


// The current draw when the output is disabled
#define DPM_MIN_CURRENT PD_MA2PDI(10)

bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request)
{
	// Get the number of PDOs
	uint8_t numobj = PD_NUMOBJ_GET(capabilities);

	PDCAP_STR("\nPDEV");
	PDCAP_CHARR(INT99_TO_CHARR(numobj));

	// Get whether or not the power supply is constrained

	// Make sure we have configuration
	// Look at the PDOs to see if one matches our desires
	// Look against USB_PD_Desired_Levels to select in order of preference
	uint8_t bestIndex = 0xFF;
	int bestIndexVoltage = 0;
	int bestIndexCurrent = 0;
	bool bestIsPPS = false, bestIsVariable = false;
	for (uint8_t i = 0; i < numobj; i++)
	{
		// If we have a fixed PDO, its V equals our desired V, and its I is at least our desired I
		if ((capabilities->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_FIXED)
		{
			// This is a fixed PDO entry
			// Evaluate if it can produve sufficient current based on the
			// tipResistance (ohms*10) V=I*R -> V/I => minimum resistance, if our tip
			// resistance is >= this then we can use this supply

			int voltage_mv = PD_PDV2MV(PD_PDO_SRC_FIXED_VOLTAGE_GET(capabilities->obj[i])); // voltage in mV units
			int current_ma = PD_PDI2MA(PD_PDO_SRC_FIXED_CURRENT_GET(capabilities->obj[i])); // current in 10mA units
			//printf("PD slot %d -> %d mV; %d mA\r\n", i, voltage_mv, current_ma);
			PDCAP_CHARR('+', 'F', INT999_TO_CHARR(voltage_mv/100), ':', INT999_TO_CHARR(current_ma/100));

			if ((voltage_mv > bestIndexVoltage || bestIndex == 0xFF) && voltage_mv <= max_req_voltage)
			{
				// Higher voltage and valid, select this instead
				bestIndex = i;
				bestIndexVoltage = voltage_mv;
				bestIndexCurrent = current_ma;
				bestIsPPS = false;
				bestIsVariable = false;
			}
		}
		else if ((capabilities->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED
				&& (capabilities->obj[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS)
		{
			// If this is a PPS slot, calculate the max voltage in the PPS range that
			// can we be used and maintain
			int max_voltage = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(capabilities->obj[i]));
			int min_voltage = PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(capabilities->obj[i]));
			int max_current = PD_PAI2MA(PD_APDO_PPS_CURRENT_GET(capabilities->obj[i])); // max current in mA units
			//printf("PD PDO slot %d -> %d mV; %d mA\r\n", i, max_voltage, max_current * 10);
			PDCAP_CHARR('+', 'P', INT999_TO_CHARR(min_voltage/100), '-', INT999_TO_CHARR(max_voltage/100), ':', INT999_TO_CHARR(max_current/100));
			// Using the current and tip resistance, calculate the ideal max voltage
			// if this is range, then we will work with this voltage
			// if this is not in range; then max_voltage can be safely selected

			// PPS requires frequent interrups (10s or so) to keep alive, which tends to fail due to overtemperature signal eventually
			/* if ((max_voltage > bestIndexVoltage && bestIndexVoltage != max_req_voltage) || bestIndex == 0xFF)
			{
				bestIndex = i;
				bestIndexVoltage = max_voltage > max_req_voltage? max_req_voltage : max_voltage;
				bestIndexCurrent = max_current;
				bestIsPPS = true;
				bestIsVariable = false;
			} */
		}
		else if ((capabilities->obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_VARIABLE)
		{
			{
				// If this is a variable PDO slot, get maximum voltage in range that we can use
				int max_voltage = PD_PDV2MV(PD_PDO_SRC_VARIABLE_MAX_VOLTAGE_GET(capabilities->obj[i]));
				int min_voltage = PD_PDV2MV(PD_PDO_SRC_VARIABLE_MIN_VOLTAGE_GET(capabilities->obj[i]));
				int max_current = PD_PAI2MA(PD_PDO_SRC_VARIABLE_CURRENT_GET(capabilities->obj[i])); // max current in mA
				//printf("PD PDO slot %d -> %d mV; %d mA\r\n", i, max_voltage, max_current);
				PDCAP_CHARR('+', 'V', INT999_TO_CHARR(min_voltage/100), '-', INT999_TO_CHARR(max_voltage/100), ':', INT999_TO_CHARR(max_current/100), '?', UI32_TO_HEX_ARR(capabilities->obj[i]));
				// Using the current and tip resistance, calculate the ideal max voltage
				// if this is range, then we will work with this voltage
				// if this is not in range; then max_voltage can be safely selected

				// No charger to test this on, except the Anker Nano II 45W which outputs a bogus variable-type PDO instead of it's advertised 21V PPS.
				// If that is any indication of the state of variable-type PDO usage then this is not reliable if it works at all
				/* if ((max_voltage > bestIndexVoltage && bestIndexVoltage != max_req_voltage) || bestIndex == 0xFF)
				{
					bestIndex = i;
					bestIndexVoltage = max_voltage > max_req_voltage? max_req_voltage : max_voltage;
					bestIndexCurrent = max_current > 3000? 2000 : max_current; // Attempt to request even when PDO is obviously faulty
					bestIsPPS = false;
					bestIsVariable = true;
				} */
			}
			/* {
				// If this is a PPS slot, calculate the max voltage in the PPS range that
				// can we be used and maintain
				int max_voltage = PD_PAV2MV(PD_APDO_PPS_MAX_VOLTAGE_GET(capabilities->obj[i]));
				int min_voltage = PD_PAV2MV(PD_APDO_PPS_MIN_VOLTAGE_GET(capabilities->obj[i]));
				int max_current = PD_PAI2MA(PD_APDO_PPS_CURRENT_GET(capabilities->obj[i])); // max current in mA units
				//printf("PD PDO slot %d -> %d mV; %d mA\r\n", i, max_voltage, max_current * 10);
				PDCAP_CHARR('+', '?', INT999_TO_CHARR(min_voltage/100), '-', INT999_TO_CHARR(max_voltage/100), ':', INT999_TO_CHARR(max_current/100));
				// Using the current and tip resistance, calculate the ideal max voltage
				// if this is range, then we will work with this voltage
				// if this is not in range; then max_voltage can be safely selected
			} */
		}
		else
			PDCAP_CHARR('+', UI32_TO_HEX_ARR(capabilities->obj[i]), '?');
	}

	if (bestIndex != 0xFF)
	{
		//printf("Found desired capability at index  %d, %d mV, %d mA\r\n",
		//		(int) bestIndex, bestIndexVoltage, bestIndexCurrent);

		// We got what we wanted, so build a request for that
		request->hdr = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
		if (bestIsPPS)
		{
			request->obj[0] = PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1) | 
				PD_RDO_PROG_CURRENT_SET(PD_MA2PAI(bestIndexCurrent)) | 
				PD_RDO_PROG_VOLTAGE_SET(PD_MV2PRV(bestIndexVoltage));
			PDCAP_CHARR('=', 'P', '0'+bestIndex, ':', INT999_TO_CHARR(bestIndexVoltage/100), ':', INT999_TO_CHARR(bestIndexCurrent/100), '\n');
		}
		else
		{ // Fixed and Variable share same RDO format
			request->obj[0] = PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1) | 
				PD_RDO_FV_MAX_CURRENT_SET(PD_MA2PDI(bestIndexCurrent)) | 
				PD_RDO_FV_CURRENT_SET(PD_MA2PDI(bestIndexCurrent));
			PDCAP_CHARR('=', bestIsVariable? 'V' : 'F', '0'+bestIndex, ':', INT999_TO_CHARR(bestIndexVoltage/100), ':', INT999_TO_CHARR(bestIndexCurrent/100), '\n');
		}
		// USB Data
		request->obj[0] |= PD_RDO_USB_COMMS;
	}
	else
	{
		PDCAP_CHARR('=', '!', '\n');
		return false;
	}
	/* else
	{
		// Nothing matched (or no configuration), so get 5 V at low current
		request->hdr = PD_MSGTYPE_REQUEST | PD_NUMOBJ(1);
		request->obj[0] = PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(1) | 
			PD_RDO_FV_MAX_CURRENT_SET(DPM_MIN_CURRENT) | 
			PD_RDO_FV_CURRENT_SET(DPM_MIN_CURRENT);
		// If the output is enabled and we got here, it must be a capability mismatch.
		if (false)
		{ // TODO: Check if you have already negotiated
			request->obj[0] |= PD_RDO_CAP_MISMATCH;
		}
		// USB Data
		request->obj[0] |= PD_RDO_USB_COMMS;
	} */
	// Even if we didnt match, we return true as we would still like to handshake on 5V at the minimum
	return true;
}

void pdbs_dpm_get_sink_capability(pd_msg *cap, const bool isPD3)
{
	// Keep track of how many PDOs we've added
	int numobj = 0;
	USBPD_STR("\nPDCP");

	// Must always have a PDO object for vSafe5V, indicate the bare minimum power required
	// Minimum current, 5 V, and higher capability.
	cap->obj[numobj++] = PD_PDO_TYPE_FIXED | 
		PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(5000)) | 
		PD_PDO_SNK_FIXED_CURRENT_SET(DPM_MIN_CURRENT);

	if (true)
	{ // If requesting more than 5V
		// Get the current we want
		uint16_t voltage = 15 * 1000; // in mV
		uint16_t current = 1 * 100; // In 10mA

		// Add a PDO for the desired power.
		cap->obj[numobj++] = PD_PDO_TYPE_FIXED | 
			PD_PDO_SNK_FIXED_VOLTAGE_SET(PD_MV2PDV(voltage)) | 
			PD_PDO_SNK_FIXED_CURRENT_SET(current);

		// If we want more than 5 V, set the Higher Capability flag
		//if (PD_MV2PDV(voltage) != PD_MV2PDV(5000))
		{
			cap->obj[0] |= PD_PDO_SNK_FIXED_HIGHER_CAP;
		}
		// If we're using PD 3.0, add a PPS APDO for our desired voltage
		/* if (isPD3)
		{
			cap->obj[numobj++] = PD_PDO_TYPE_AUGMENTED | PD_APDO_TYPE_PPS | 
				PD_APDO_PPS_MAX_VOLTAGE_SET(PD_MV2PAV(voltage)) | 
				PD_APDO_PPS_MIN_VOLTAGE_SET(PD_MV2PAV(voltage)) | 
				PD_APDO_PPS_CURRENT_SET(PD_CA2PAI(current));
		} */
	}
	// Set the USB communications capable flag.
	cap->obj[0] |= PD_PDO_SNK_FIXED_USB_COMMS;
	// if this device is unconstrained, set the flag
	//cap->obj[0] |= PD_PDO_SNK_FIXED_UNCONSTRAINED;

	// Set the Sink_Capabilities message header
	cap->hdr = PD_MSGTYPE_SINK_CAPABILITIES | PD_NUMOBJ(numobj);
}

#define WAIT_FOR_STAR1(FLAG, TIMEOUT_US, MSG) {\
	TimePoint start = GetTimePoint();\
	TimePoint limit = start + TIMEOUT_US * TICKS_PER_US;\
	while (!(I2C1->STAR1&FLAG));\
	if (GetTimePoint() > limit)\
	{\
		ERR_STR(MSG);\
		ERR_CHARR(':', UINT999999_TO_CHARR(GetTimeSpanUS(start, GetTimePoint())));\
	}\
}

bool i2c_read(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf)
{
	I2C1->CTLR1 = I2C_CTLR1_PE;
	while (I2C1->CTLR1 == 0);

	WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);

	I2C1->CTLR1 |= I2C_CTLR1_START;
	WAIT_FOR_STAR1(I2C_STAR1_SB, 10, "!I2CSBR");
	I2C1->DATAR = deviceAddr | I2C_Direction_Transmitter;
	WAIT_FOR_STAR1(I2C_STAR1_ADDR, 35, "!I2CAD");
	volatile uint16_t star2 = I2C1->STAR2; // Read after STAR1 to clear ADDR bit

	I2C1->DATAR = registerAdd;
	WAIT_FOR_STAR1(I2C_STAR1_TXE, 2, "!I2CTX");

	I2C1->CTLR1 |= I2C_CTLR1_START;
	WAIT_FOR_STAR1(I2C_STAR1_SB, 35, "!I2CSBT");
	I2C1->DATAR = deviceAddr | I2C_Direction_Receiver;
	WAIT_FOR_STAR1(I2C_STAR1_ADDR, 35, "!I2CAD");
	star2 = I2C1->STAR2; // Read after STAR1 to clear ADDR bit

	// Ensure that acks are send when reading data
	I2C1->CTLR1 |= I2C_CTLR1_ACK;
	bool lostData = false;
	for (int i = 0; i < size; i++)
	{
		if (i == size-1)
			I2C1->CTLR1 = (I2C1->CTLR1&~I2C_CTLR1_ACK) | I2C_CTLR1_STOP;
		WAIT_FOR_STAR1(I2C_STAR1_RXNE, 35, "!I2CRX");
		bool btf = I2C1->STAR1&I2C_STAR1_BTF; // Could happen since PD handling can be interrupted (low priority interrupt)
		buf[i] = I2C1->DATAR;
		if (btf)
		{
			ERR_STR("!I2CLD");
			lostData = true;
		}
	}

	I2C1->CTLR1 = 0;

	WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);
	return !lostData;
}

bool i2c_write(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf)
{
	I2C1->CTLR1 = I2C_CTLR1_PE;
	while (I2C1->CTLR1 == 0);

	WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);

	I2C1->CTLR1 |= I2C_CTLR1_START;
	WAIT_FOR_STAR1(I2C_STAR1_SB, 10, "!I2CSBT");
	I2C1->DATAR = deviceAddr | I2C_Direction_Transmitter;
	WAIT_FOR_STAR1(I2C_STAR1_ADDR, 35, "!I2CAD");
	volatile uint16_t star2 = I2C1->STAR2; // Read after STAR1 to clear ADDR bit

	I2C1->DATAR = registerAdd;
	WAIT_FOR_STAR1(I2C_STAR1_TXE, 35, "!I2CTX");

	for (int i = 0; i < size; i++)
	{
		I2C1->DATAR = buf[i];
		WAIT_FOR_STAR1(I2C_STAR1_TXE, 35, "!I2CTX");
	}
	WAIT_FOR_STAR1(I2C_STAR1_BTF, 35, "!I2CBT");

	I2C1->CTLR1 |= I2C_CTLR1_STOP;

	I2C1->CTLR1 = 0;

	WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);
	return true;
}