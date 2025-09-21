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
#include "util.h"

/* #include "uartd.h"

#include "ch32v30x_usart.h"
#include "ch32v30x_dma.h" */
#include "ch32v30x_i2c.h"

#include "fusb302b.h"
#include "fusb302_defines.h"
#include "policy_engine.h"

#if USBPD_LOG || PD_CAPS
#define PDCAP_CHARR LOG_CHARR
#else
#define PDCAP_CHARR(...) {}
#endif

bool i2c_read(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf);
bool i2c_write(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf);
void pd_delayUS(uint32_t us)
{
	TimePoint tgt = GetTimePoint();
	tgt += us * TICKS_PER_US;
	USBPD_CHARR('/', 'D', INT9999_TO_CHARR(us));
	while (GetTimePoint() < tgt);
	// Or just
	//delayUS(us);
}
uint32_t timestampMS()
{
	return GetMS(GetTimePoint());
}

bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request);
void pdbs_dpm_get_sink_capability(pd_msg *cap, const bool isPD3);

static FUSB302 fusb;
static PolicyEngine pe;

int max_req_voltage = 15000;
int min_req_voltage = 10000;

inline static void log_pe_state(char conn)
{
#if USBPD_LOG
	USBPD_CHARR('!');
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
	pd_os_init(timestampMS, pd_delayUS, i2c_read, i2c_write);
	fusb.DeviceAddress = FUSB302B_ADDR;
	pe_init(&pe, &fusb, pdbs_dpm_get_sink_capability, pdbs_dpm_evaluate_capability, NULL, 0);

	USBPD_CHARR('\n', '/', 'P', 'D', 'S', 'T', 'P');

	// TODO: Might have to call TimersCallback every 5-10s to keep voltage coming (for PPS specifically)

	// TODO: Consider renegotiating when controller is reset instead of just assuming the PD state is correct

	// TODO: Verify PD voltage (and external power in voltage) with ADC circuit

	// TODO: Make this async somehow, it blocks everything whenever pd_delayUS is used rn
	// fine for now as it's only used at startup but worse if PD stuff needs to happen regularly / during streaming

	if (fusb_setup(&fusb))
	{ // Connected and set up FUSB, without resetting possible prior state
		//pe_IRQ_occured(&pe);
		if (fusb_isVBUSConnected(&fusb))
		{ // Can attempt to select CC lines to communicate
			if (fusb_checkVBUS(&fusb, min_req_voltage) && fusb_hasCCLineSelection(&fusb))
			{ // Already set up VBUS, likely had reset - forcibly set to ready state
				USBPD_CHARR('+', 'V', 'B', 'S', 'R', 'D', 'Y');
				//pe.state = PESinkReady;
				pe.state = PESinkSendSoftReset;
			}
			else
			{
				USBPD_CHARR('+', 'V', 'B', 'S', 'C', 'N', 'C');
				int ccSel = fusb_runCCLineSelection(&fusb);
				if (ccSel == 0)
				{
					pe.state = PESinkSendSoftReset;
					USBPD_CHARR('+', 'C', 'C', 'S', 'E', 'L');
				}
				else if (ccSel < 0)
				{
					USBPD_CHARR('+', 'C', 'C', 'F', 'D', 'T');
				}
				else
				{
					USBPD_CHARR('+', 'C', 'C', 'N', 'D', 'T');
				}
			}
		}
		else
			USBPD_CHARR('+', 'V', 'B', 'S', 'D', 'C', 'N');
		// Else wait for interrupt
	}

	log_pe_state('=');
}

void pd_poll()
{
	USBPD_CHARR('\n', '/', 'P', 'D', 'I', 'N', 'T');

	pe_IRQ_occured(&pe);

	log_pe_state('!');
	while (pe_thread(&pe))
		log_pe_state('+');
	log_pe_state('+');
}


void pd_timer()
{
	//if (pe.PPSTimerEnabled)
	{
		USBPD_CHARR('\n', '/', 'T', 'M');

		TimersCallback(&pe);

		log_pe_state('!');
		while (pe_thread(&pe))
			log_pe_state('+');
		log_pe_state('+');
	}
}


// The current draw when the output is disabled
#define DPM_MIN_CURRENT PD_MA2PDI(10)

bool pdbs_dpm_evaluate_capability(const pd_msg *capabilities, pd_msg *request)
{
	// Get the number of PDOs
	uint8_t numobj = PD_NUMOBJ_GET(capabilities);

	PDCAP_CHARR('\n', '/', 'P', 'D', 'E', 'V', INT99_TO_CHARR(numobj));

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
			PDCAP_CHARR('=', 'P', '0'+bestIndex, ':', INT999_TO_CHARR(bestIndexVoltage/100), ':', INT999_TO_CHARR(bestIndexCurrent/100));
		}
		else
		{ // Fixed and Variable share same RDO format
			request->obj[0] = PD_RDO_NO_USB_SUSPEND | PD_RDO_OBJPOS_SET(bestIndex + 1) | 
				PD_RDO_FV_MAX_CURRENT_SET(PD_MA2PDI(bestIndexCurrent)) | 
				PD_RDO_FV_CURRENT_SET(PD_MA2PDI(bestIndexCurrent));
			PDCAP_CHARR('=', bestIsVariable? 'V' : 'F', '0'+bestIndex, ':', INT999_TO_CHARR(bestIndexVoltage/100), ':', INT999_TO_CHARR(bestIndexCurrent/100));
		}
		// USB Data
		request->obj[0] |= PD_RDO_USB_COMMS;
	}
	else
	{
		PDCAP_CHARR('=', '!');
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
	USBPD_CHARR('\n', '/', 'P', 'D', 'C', 'P');

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

bool i2c_read(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf)
{
	uint32_t timeout;
	I2C1->CTLR1 |= I2C_CTLR1_START;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_SB) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'S', 'B'); return false; }
	I2C1->CTLR1 &= ~I2C_CTLR1_START;
	I2C1->DATAR = deviceAddr | I2C_Direction_Transmitter;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_ADDR) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'A', 'D'); return false; }
	volatile uint16_t star2 = I2C1->STAR2;

	I2C1->DATAR = registerAdd;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_TXE) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'T', 'X'); return false; }

	I2C1->CTLR1 |= I2C_CTLR1_START;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_SB) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'S', 'B'); return false; }
	I2C1->CTLR1 &= ~I2C_CTLR1_START;
	I2C1->DATAR = deviceAddr | I2C_Direction_Receiver;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_ADDR) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'A', 'D'); return false; }
	star2 = I2C1->STAR2;

	// Ensure that acks are send when reading data
	I2C1->CTLR1 |= I2C_CTLR1_ACK;
	bool lostData = false;
	for (int i = 0; i < size; i++)
	{
		if (i == size-1)
			I2C1->CTLR1 &= ~I2C_CTLR1_ACK;
		timeout = 10000;
		while (!(I2C1->STAR1&I2C_STAR1_RXNE) && timeout-- > 0);
		if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'R', 'X'); return false; }
		bool btf = I2C1->STAR1&I2C_STAR1_BTF; // Could happen since PD handling can be interrupted (low priority interrupt)
		buf[i] = I2C1->DATAR;
		if (btf)
		{
			ERR_CHARR('/', 'I', '2', 'C', 'L', 'D');
			lostData = true;
		}
	}

	I2C1->CTLR1 |= I2C_CTLR1_STOP;
	//while (!(I2C1->STAR1&I2C_STAR1_SB));

	return !lostData;
}

bool i2c_write(const uint8_t deviceAddr, const uint8_t registerAdd, const uint8_t size, uint8_t *buf)
{
	uint32_t timeout;
	I2C1->CTLR1 |= I2C_CTLR1_START;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_SB) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'S', 'B'); return false; }
	I2C1->CTLR1 &= ~I2C_CTLR1_START;
	I2C1->DATAR = deviceAddr | I2C_Direction_Transmitter;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_ADDR) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'A', 'D'); return false; }
	volatile uint16_t star2 = I2C1->STAR2;

	I2C1->DATAR = registerAdd;
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_TXE) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'T', 'X'); return false; }

	for (int i = 0; i < size; i++)
	{
		I2C1->DATAR = buf[i];
		timeout = 10000;
		while (!(I2C1->STAR1&I2C_STAR1_TXE) && timeout-- > 0);
		if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'T', 'X'); return false; }
	}
	timeout = 10000;
	while (!(I2C1->STAR1&I2C_STAR1_BTF) && timeout-- > 0);
	if (!timeout) { ERR_CHARR('/', 'I', '2', 'C', 'B', 'T'); return false; }

	I2C1->CTLR1 |= I2C_CTLR1_STOP;
	//while (!(I2C1->STAR1&I2C_STAR1_SB));

	return true;
}