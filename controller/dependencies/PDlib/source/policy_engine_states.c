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
#include "policy_engine.h"
#include <pd.h>
#include <stdbool.h>
#ifdef PD_DEBUG_OUTPUT
#include "stdio.h"
#endif

#include "util.h"

policy_engine_state pe_sink_startup(PolicyEngine *pe)
{
	/* No need to reset the protocol layer here.  There are three ways into this
	 * state: startup, CC line lost, and exiting hard reset.
	 * On startup, the protocol layer is reset by the startup procedure.
	 * When CC line is lost, it is reset in the IRQ that detected it.
	 * When exiting hard reset, the it is reset by the hard reset state machine.
	 * Since it's already done somewhere else, there's no need to do it again here. */
	if ((pe->status.status0 & FUSB_STATUS0_VBUSOK) != FUSB_STATUS0_VBUSOK)
		return PESinkStartup;
	return PESinkDiscovery;
}

policy_engine_state pe_sink_discovery(PolicyEngine *pe)
{
	/* Wait for VBUS.  Since it's our only power source, we already know that
	 * we have it, so just move on.
	 * If this was not true, we would want to wait and then re-run CC line selection
	 * fusb_runCCLineSelection();
	 */
	int stat = 0;
	while((stat = fusb_runCCLineSelection(pe->fusb)) != 0)
	{
		if (stat < 0) // Failed to write
			return PESinkDiscovery;
		if (stat > 0) // Could wait here, or wait for next event (high VBUS)
			return PESinkDiscovery;
	}
	return PESinkSetupWaitCap;
}
policy_engine_state pe_sink_setup_wait_cap(PolicyEngine *pe)
{
	pe->_explicit_contract = false;
	pe->PPSTimerEnabled    = false;
	pe->currentEvents      = 0;

	pe->timestampNegotiationsStarted = getTimeStamp();
	// Wait for cap timeout
	return pe_waitForEvent(pe, PESinkWaitCap, (uint32_t)NOTIF_MSG_RX | (uint32_t)NOTIF_I_OVRTEMP | (uint32_t)NOTIF_RESET, PD_T_TYPEC_SINK_WAIT_CAP);
}
policy_engine_state pe_sink_wait_cap(PolicyEngine *pe)
{
	// Fetch a message from the protocol layer
	uint32_t evt = pe->currentEvents;
	pe_clearEvents(pe, evt);
#ifdef PD_DEBUG_OUTPUT
	printf("Wait Cap Event %04X\r\n", (int)evt);
#endif

	// If we're too hot, we shouldn't negotiate power yet
	if (evt & (uint32_t)NOTIF_I_OVRTEMP)
		return PESinkSetupWaitCap;

	// Get the message
	while (pdb_msg_getOccupied(&pe->incomingMessages))
	{
		pdb_msg_pop(&pe->incomingMessages, &pe->tempMessage);
		// If we got a Source_Capabilities message, read it.
		if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_SOURCE_CAPABILITIES && PD_NUMOBJ_GET(&pe->tempMessage) > 0)
		{
#ifdef PD_DEBUG_OUTPUT
			printf("Source Capabilities message RX\r\n");
#endif

			// First, determine what PD revision we're using
			if ((pe->hdr_template & PD_HDR_SPECREV) == PD_SPECREV_1_0)
			{
				if ((pe->tempMessage.hdr & PD_HDR_SPECREV) >= PD_SPECREV_3_0)
				{ // If the other end is using at least version 3.0, we'll use version 3.0.
					pe->hdr_template |= PD_SPECREV_3_0;
					USBPD_STR("+V3");
				}
				else
				{ // Otherwise, use 2.0.  Don't worry about the 1.0 case because we don't have hardware for PD 1.0 signaling.
					pe->hdr_template |= PD_SPECREV_2_0;
					USBPD_STR("+V2");
				}
			}
			return PESinkEvalCap;
		}
	}

	// If we failed to get a message, wait longer
	return PESinkSetupWaitCap;
}

policy_engine_state pe_sink_eval_cap(PolicyEngine *pe)
{
	/* If we have a Source_Capabilities message, remember the index of the
	 * first PPS APDO so we can check if the request is for a PPS APDO in
	 * PE_SNK_Select_Cap. */
	/* Start by assuming we won't find a PPS APDO (set the index greater
	 * than the maximum possible) */
	pe->_pps_index = 0xFF;
	/* New capabilities also means we can't be making a request from the
	 * same PPS APDO */
	/* Search for the first PPS APDO */
	for (int i = 0; i < PD_NUMOBJ_GET(&pe->tempMessage); i++)
	{
		if ((pe->tempMessage.obj[i] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED && (pe->tempMessage.obj[i] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS)
		{
			pe->_pps_index = i + 1;
			break;
		}
	}
	pe->_unconstrained_power = pe->tempMessage.obj[0] & PD_PDO_SRC_FIXED_UNCONSTRAINED;
	pe->sourceIsEPRCapable   = pe->tempMessage.obj[0] & PD_PDO_SRC_FIXED_EPR_CAPABLE;

	// Ask the DPM what to request
	if (pe->pdbs_dpm_evaluate_capability(&pe->tempMessage, &pe->_last_dpm_request))
	{
		pe->_last_dpm_request.hdr |= pe->hdr_template;
		// If we're using PD 3.0
		if ((pe->hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0)
		{ // If the request was for a PPS, start time callbacks if not started
			uint32_t pdoPos = PD_RDO_OBJPOS_GET(&pe->_last_dpm_request);
			if (pdoPos <= 7 && pdoPos >= pe->_pps_index)
				pe->PPSTimerEnabled = true;
			else
				pe->PPSTimerEnabled = false;
		}
		return PESinkSelectCapTx;
	}

	return PESinkWaitCap;
}
policy_engine_state pe_sink_select_cap_tx(PolicyEngine *pe)
{
	// Transmit the request
	// pe_clearEvents(0xFFFFFF); // clear all pending incase of an rx while prepping

#ifdef PD_DEBUG_OUTPUT
	printf("Sending desired capability\r\n");
#endif
	return pe_start_message_tx(pe, PESinkSelectCap, PESinkHardReset, &pe->_last_dpm_request);
}
policy_engine_state pe_sink_select_cap(PolicyEngine *pe)
{
	// Have transmitted the selected cap, transition to waiting for the response
	pe_clearEvents(pe, 0xFFFFFF);
	// wait for a response
	return pe_waitForEvent(pe, PESinkWaitCapResp, (uint32_t)NOTIF_MSG_RX | (uint32_t)NOTIF_RESET | (uint32_t)NOTIF_TIMEOUT, PD_T_SENDER_RESPONSE);
}

policy_engine_state pe_sink_wait_cap_resp(PolicyEngine *pe)
{
	// Wait for a response
	pe_clearEvents(pe, 0xFFFFFF);

	while (pdb_msg_getOccupied(&pe->incomingMessages))
	{ // Get the response message
		pdb_msg_pop(&pe->incomingMessages, &pe->tempMessage);
		if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_ACCEPT)
		{ // If the source accepted our request, wait for the new power message

			pe->is_epr = (PD_NUMOBJ_GET(&pe->_last_dpm_request) == 2);
			if (pe->is_epr)
				pe->EPRTimeLastEvent = getTimeStamp();
			return pe_waitForEvent(pe, PESinkTransitionSink, (uint32_t)NOTIF_MSG_RX | (uint32_t)NOTIF_RESET, PD_T_PS_TRANSITION);
		}
		else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_SOFT_RESET)
		{ // If the message was a Soft_Reset, do the soft reset procedure
			return PESinkHandleSoftReset;
		}
		else if ((PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_REJECT || PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_WAIT))
		{ // If the message was Wait or Reject
#ifdef PD_DEBUG_OUTPUT
			printf("Requested Capabilities Rejected\r\n");
#endif
			if (!pe->_explicit_contract)
				// If we don't have an explicit contract, wait for capabilities
				return PESinkSetupWaitCap;
			else // If we do have an explicit contract, go to the ready state
				return pe_waitForEvent(pe, PESinkReady, (uint32_t)NOTIF_ALL, TICK_MAX_DELAY);
		}
	}
	return pe_waitForEvent(pe, PESinkWaitCapResp, (uint32_t)NOTIF_MSG_RX | (uint32_t)NOTIF_RESET | (uint32_t)NOTIF_TIMEOUT, PD_T_SENDER_RESPONSE);
}

policy_engine_state pe_sink_transition_sink(PolicyEngine *pe)
{
	/* Wait for the PS_RDY message */
	pe_clearEvents(pe, 0xFFFFFF);
	/* If we received a message, read it */
	while (pdb_msg_getOccupied(&pe->incomingMessages))
	{
		pdb_msg_pop(&pe->incomingMessages, &pe->tempMessage);

		/* If we got a PS_RDY, handle it */
		if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_PS_RDY)
		{
			/* We just finished negotiating an explicit contract */
			/* Negotiation finished */
			pe->negotiationOfEPRInProgress = false;
			if (pe->sourceIsEPRCapable && (pe->device_epr_wattage > 0) && !pe->is_epr)
			{
				// We have entered into an SPR contract, but we support EPR and the supply does too
				//  Request entering EPR mode
				pe->negotiationOfEPRInProgress = true;
				notify(pe, NOTIF_REQUEST_EPR);
			}
			pe->_explicit_contract = true;

			return PESinkReady;
		}
		else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_SOURCE_CAPABILITIES)
		{
			return PESinkEvalCap;
		}
	}
	// Timeout
	return PESinkSendSoftReset;
}

policy_engine_state pe_sink_ready(PolicyEngine *pe)
{
	uint32_t evt = pe->currentEvents;
	pe_clearEvents(pe, evt);
	/* If SinkPPSPeriodicTimer ran out, send a new request */
	if (evt & (uint32_t)NOTIF_PPS_REQUEST) {
		return PESinkSelectCapTx;
	}

	/* If we overheated, send a hard reset */
	if (evt & (uint32_t)NOTIF_I_OVRTEMP) {
		return PESinkHardReset;
	}
	/* If the DPM wants us to, send a Get_Source_Cap message */
	if (evt & (uint32_t)NOTIF_GET_SOURCE_CAP) {
		return PESinkGetSourceCap;
	}
	/* Request the source sends us its current capabilities again */
	if (evt & (uint32_t)NOTIF_NEW_POWER) {
		return PESinkGetSourceCap;
	}

	if (evt & (uint32_t)NOTIF_REQUEST_EPR) {
		return PESinkRequestEPR;
	}

	if (evt & (uint32_t)NOTIF_EPR_KEEPALIVE) {
		return PESinkSendEPRKeepAlive;
	}

	/* If we received a message */
	if (evt & (uint32_t)NOTIF_MSG_RX) {
		while (pdb_msg_getOccupied(&pe->incomingMessages)) {

			pdb_msg_pop(&pe->incomingMessages, &pe->tempMessage);

			if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_VENDOR_DEFINED && PD_NUMOBJ_GET(&pe->tempMessage) > 0) {
				// return pe_waitForEvent(PESinkReady, (uint32_t)NOTIF_ALL, TICK_MAX_DELAY);
				/* Ignore Ping messages */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_PING && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				// return pe_waitForEvent(PESinkReady, (uint32_t)NOTIF_ALL, TICK_MAX_DELAY);
				/* DR_Swap messages are not supported */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_DR_SWAP && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				return PESinkSendNotSupported;
				/* Get_Source_Cap messages are not supported */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_GET_SOURCE_CAP && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				return PESinkSendNotSupported;
				/* PR_Swap messages are not supported */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_PR_SWAP && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				return PESinkSendNotSupported;
				/* VCONN_Swap messages are not supported */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_VCONN_SWAP && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				return PESinkSendNotSupported;
				/* Request messages are not supported */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_REQUEST && PD_NUMOBJ_GET(&pe->tempMessage) > 0) {
				return PESinkSendNotSupported;
				/* Sink_Capabilities messages are not supported */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_SINK_CAPABILITIES && PD_NUMOBJ_GET(&pe->tempMessage) > 0) {
				return PESinkSendNotSupported;
				/* Handle GotoMin messages */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_GOTOMIN && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				return PESinkSendNotSupported;
				/* Evaluate new Source_Capabilities */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_SOURCE_CAPABILITIES && PD_NUMOBJ_GET(&pe->tempMessage) > 0) {
				return PESinkEvalCap;
				/* Give sink capabilities when asked */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_GET_SINK_CAP && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				return PESinkGiveSinkCap;
				/* If the message was a Soft_Reset, do the soft reset procedure */
			} else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&pe->tempMessage) == 0) {
				return PESinkHandleSoftReset;
				/* PD 3.0 messges */
			}
			else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_EPR_MODE && PD_NUMOBJ_GET(&pe->tempMessage) > 0)
			{
				if (pe->tempMessage.bytes[0] == 3)
				{ // We start off from here, but let the message read loop run until all are read
					pe->is_epr = true;
					// return PESinkReady;
				}
				else if (pe->tempMessage.bytes[0] == 4)
				{ // We attempted to enter EPR and failed, no need to renegotiate
					pe->is_epr = false;
					return PESinkReady;
					
				}
				else if (pe->tempMessage.bytes[0] == 5)
				{ // We exited EPR so now need to renegotiate an SPR contract
					pe->is_epr = false;
					return PESinkWaitCap;
				}
			}
			else if ((pe->hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0)
			{
				if ((pe->tempMessage.hdr & PD_HDR_EXT) && (PD_DATA_SIZE_GET(&pe->tempMessage) >= PD_MAX_EXT_MSG_LEGACY_LEN))
				{ // If the message is a multi-chunk extended message
					if ((PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_EPR_SOURCE_CAPABILITIES))
						return PESinkHandleEPRChunk; // We can support _some_ chunked messages but not all
					else // Tell the DPM a message we sent got a response of Not_Supported.
						return PESinkSendNotSupported;
					
				}
				else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_NOT_SUPPORTED && PD_NUMOBJ_GET(&pe->tempMessage) == 0)
					return PESinkNotSupportedReceived;
				else // If we got an unknown message, Send Not Supported back
					return PESinkSendNotSupported;
			}
		}
	}

	return pe_waitForEvent(pe, PESinkReady, (uint32_t)NOTIF_ALL, TICK_MAX_DELAY);
}

policy_engine_state pe_sink_get_source_cap(PolicyEngine *pe)
{
	// Get a message object
	pd_msg *get_source_cap = &pe->tempMessage;
	// Make a Get_Source_Cap message
	get_source_cap->hdr = pe->hdr_template | PD_MSGTYPE_GET_SOURCE_CAP | PD_NUMOBJ(0);
	// Transmit the Get_Source_Cap
	// On fail -> hard reset, on send -> Sink Ready
	return pe_start_message_tx(pe, PESinkReady, PESinkHardReset, get_source_cap);
}

policy_engine_state pe_sink_give_sink_cap(PolicyEngine *pe)
{
	// Get a message object
	pd_msg *snk_cap = &pe->tempMessage;
	// Get our capabilities from the DPM
	pe->pdbs_dpm_get_sink_capability(snk_cap, ((pe->hdr_template & PD_HDR_SPECREV) >= PD_SPECREV_3_0));
	// Transmit our capabilities
	return pe_start_message_tx(pe, PESinkReady, PESinkHardReset, snk_cap);
}

policy_engine_state pe_sink_hard_reset(PolicyEngine *pe)
{
	// If we've already sent the maximum number of hard resets, assume the source is unresponsive.

#ifdef PD_DEBUG_OUTPUT
	printf("Sending hard reset\r\n");
#endif
	if (pe->_hard_reset_counter > PD_N_HARD_RESET_COUNT) {
		return PESinkSourceUnresponsive;
	}
	// So, we could send a hardreset here; however that will cause a power cycle
	// on the PSU end.. Which will then reset this MCU So therefore we went get
	// anywhere :)
	// Increment HardResetCounter
	pe->_hard_reset_counter++;

	return PESinkTransitionDefault;
}

policy_engine_state pe_sink_transition_default(PolicyEngine *pe)
{
	// There is no local hardware to reset.
	// Since we never change our data role from UFP, there is no reason to set it here.

	return PESinkStartup;
}

policy_engine_state pe_sink_soft_reset(PolicyEngine *pe)
{
	// Soft reset message is received
	// No need to explicitly reset the protocol layer here.
	// It resets itself when a Soft_Reset message is received.

	// Get a message object
	pd_msg accept;
	// Make an soft reset message
	accept.hdr = pe->hdr_template | PD_MSGTYPE_SOFT_RESET | PD_NUMOBJ(0);
	// Transmit the Accept
	return pe_start_message_tx(pe, PESinkSetupWaitCap, PESinkHardReset, &accept);
}
policy_engine_state pe_sink_send_soft_reset(PolicyEngine *pe)
{
	// No need to explicitly reset the protocol layer here.
	// It resets itself just before a Soft_Reset message is transmitted.

#ifdef PD_DEBUG_OUTPUT
	printf("Sending soft reset\r\n");
#endif
	// Get a message object
	pd_msg *softrst = &pe->tempMessage;
	// Make a Soft_Reset message
	softrst->hdr = pe->hdr_template | PD_MSGTYPE_SOFT_RESET | PD_NUMOBJ(0);
	// Transmit the soft reset
	return pe_start_message_tx(pe, PESinkSendSoftResetTxOK, PESinkHardReset, softrst);
}
policy_engine_state pe_sink_send_soft_reset_tx_ok(PolicyEngine *pe)
{
	if (pe->status.status0a & 0x10)
	{ // SOFTFAIL
		return PESinkHardReset;
	}
	// Transmit is good, wait for response event
	return pe_waitForEvent(pe, PESinkSendSoftResetResp, (uint32_t)NOTIF_TIMEOUT | (uint32_t)NOTIF_MSG_RX | (uint32_t)NOTIF_RESET, PD_T_SENDER_RESPONSE);
}
policy_engine_state pe_sink_send_soft_reset_resp(PolicyEngine *pe)
{
	// Wait for a response
	pe_clearEvents(pe, 0xFFFFFF);

	// Get the response message
	if (pdb_msg_getOccupied(&pe->incomingMessages))
	{
		pdb_msg_pop(&pe->incomingMessages, &pe->tempMessage);
		if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_ACCEPT && PD_NUMOBJ_GET(&pe->tempMessage) == 0)
		{ // If the source accepted our soft reset, wait for capabilities.
			return PESinkSetupWaitCap;
		}
		else if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&pe->tempMessage) == 0)
		{ // If the message was a Soft_Reset, do the soft reset procedure
			return PESinkHandleSoftReset;
		}
		else // Otherwise, send a hard reset
			return PESinkHardReset;
	}
	return PESinkHardReset;
}

policy_engine_state pe_sink_send_not_supported(PolicyEngine *pe)
{
#ifdef PD_DEBUG_OUTPUT
	printf("Sending not supported\r\n");
#endif
	if ((pe->hdr_template & PD_HDR_SPECREV) == PD_SPECREV_2_0)
	{ // Make a Reject message
		pe->tempMessage.hdr = pe->hdr_template | PD_MSGTYPE_REJECT | PD_NUMOBJ(0);
	}
	else if ((pe->hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0)
	{ // Make a Not_Supported message
		pe->tempMessage.hdr = pe->hdr_template | PD_MSGTYPE_NOT_SUPPORTED | PD_NUMOBJ(0);
	}

	// Transmit the message
	return pe_start_message_tx(pe, PESinkReady, PESinkSendSoftReset, &pe->tempMessage);
}

policy_engine_state pe_sink_wait_epr_chunk(PolicyEngine *pe)
{
	uint32_t evt = pe->currentEvents;
	pe_clearEvents(pe, evt);
	// If we received a message
	if (evt & (uint32_t)NOTIF_MSG_RX)
	{
		while (pdb_msg_getOccupied(&pe->incomingMessages))
		{
			pdb_msg_pop(&pe->incomingMessages, &pe->tempMessage);

			if ((pe->hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0)
			{
				if ((pe->tempMessage.hdr & PD_HDR_EXT) && (PD_DATA_SIZE_GET(&pe->tempMessage) >= PD_MAX_EXT_MSG_LEGACY_LEN))
				{ // The message is a multi-chunk extended message
					if ((PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_EPR_SOURCE_CAPABILITIES))
						return PESinkHandleEPRChunk; // We can support _some_ chunked messages but not all
					else // Tell the DPM a message we sent got a response of Not_Supported.
						return PESinkSendNotSupported;
				}
			}
		}
	}

	return pe_waitForEvent(pe, PESinkWaitForHandleEPRChunk, (uint32_t)NOTIF_ALL, TICK_MAX_DELAY);
}

policy_engine_state pe_sink_handle_epr_chunk(PolicyEngine *pe)
{
	if (pe->tempMessage.exthdr & PD_EXTHDR_REQUEST_CHUNK)
		return pe_waitForEvent(pe, PESinkWaitForHandleEPRChunk, (uint32_t)NOTIF_ALL, TICK_MAX_DELAY);
	uint8_t chunk_index = PD_CHUNK_NUMBER_GET(&pe->tempMessage);

	if (chunk_index == 0)
	{ // Copy first message directly over the object to set header,ext-header + start of PDO's
		memcpy(&pe->recent_epr_capabilities, &pe->tempMessage.bytes, sizeof(pe->tempMessage.bytes));
	} 
	else
	{
		memcpy(&(pe->recent_epr_capabilities.data[chunk_index * PD_MAX_EXT_MSG_CHUNK_LEN]), &(pe->tempMessage.data), 2 + (4 * (PD_NUMOBJ_GET(&pe->tempMessage) - 1)));
	}
	uint32_t receivedLength = (PD_MAX_EXT_MSG_CHUNK_LEN * chunk_index) /*Bytes Implicit by chunk index*/ 
			+ 2 /*half PDO*/ 
			+ (4 * (PD_NUMOBJ_GET(&pe->tempMessage) - 1) /* Data in this message*/);

	if ((receivedLength) >= PD_DATA_SIZE_GET(&pe->recent_epr_capabilities))
		return PESinkEPREvalCap;
	memset(pe->tempMessage.data,0,sizeof(pe->tempMessage.data));
	pe->tempMessage.hdr    = pe->hdr_template | (pe->tempMessage.hdr & PD_HDR_MSGTYPE) | PD_NUMOBJ(1) | PD_HDR_EXT;
	pe->tempMessage.exthdr = ((chunk_index + 1) << PD_EXTHDR_CHUNK_NUMBER_SHIFT) | PD_EXTHDR_REQUEST_CHUNK | PD_EXTHDR_CHUNKED;
	return pe_start_message_tx(pe, PESinkWaitForHandleEPRChunk, PESinkHardReset, &pe->tempMessage);
}

policy_engine_state pe_sink_not_supported_received(PolicyEngine *pe)
{
	// Inform the Device Policy Manager that we received a Not_Supported message.
	return pe_waitForEvent(pe, PESinkReady, (uint32_t)NOTIF_ALL, TICK_MAX_DELAY);
}

policy_engine_state pe_sink_source_unresponsive(PolicyEngine *pe)
{
	// Sit and chill, as PD is not working
	pe->_explicit_contract = false;
	return PESinkSourceUnresponsive;
}

policy_engine_state pe_sink_wait_event(PolicyEngine *pe)
{
	// Check timeout
	if (getTimeStamp() > pe->waitingEventsTimeout)
		notify(pe, NOTIF_TIMEOUT);
	if (pe->currentEvents & (uint32_t)NOTIF_TIMEOUT)
	{
		pe_clearEvents(pe, 0xFFFFFF);
		if (pe->postNotificationEvalState >= PESinkHandleSoftReset && pe->postNotificationEvalState <= PESinkSendSoftResetResp)
			return PESinkStartup; // Timeout in soft reset, so reset state machine
		return PESinkSendSoftReset;
	}
	if (pe->currentEvents & (uint32_t)NOTIF_RESET)
		return PESinkTransitionDefault;

	if (pe->currentEvents & pe->waitingEventsMask)
		return pe->postNotificationEvalState;
	return PEWaitingEvent;
}

policy_engine_state pe_sink_wait_good_crc(PolicyEngine *pe)
{
	pe_clearEvents(pe, 0xFFFFFF);

	while (pdb_msg_getOccupied(&pe->incomingMessages))
	{ // Wait for the Good CRC
		pd_msg goodcrc;
		pdb_msg_pop(&pe->incomingMessages, &goodcrc);
		// Check that the message is correct
		if (PD_MSGTYPE_GET(&goodcrc) == PD_MSGTYPE_GOODCRC && PD_NUMOBJ_GET(&goodcrc) == 0 && PD_MESSAGEID_GET(&goodcrc) == pe->_tx_messageidcounter)
		{
			// Increment MessageIDCounter
			pe->_tx_messageidcounter = (pe->_tx_messageidcounter + 1) % 8;
			notify(pe, NOTIF_TX_DONE);
			return pe->postSendState;
		}
	}
	notify(pe, NOTIF_TX_ERR);
	return pe->postSendFailedState;
}
policy_engine_state pe_sink_wait_send_done(PolicyEngine *pe)
{
	// Waiting for response
	uint32_t evt = pe->currentEvents;
	pe_clearEvents(pe, evt);

	// If the message was sent successfully 
	if ((uint32_t)evt & (uint32_t)NOTIF_I_TXSENT)
	{
		if (pdb_msg_getOccupied(&pe->incomingMessages))
			return pe_sink_wait_good_crc(pe);
		else // No Good CRC has arrived, these should _normally_ come really fast (100us), but users implementation may be lagging
			// Setup a callback for this state
			return pe_waitForEvent(pe, PEWaitingMessageGoodCRC, (uint32_t)NOTIF_MSG_RX, 120);
	}
	// If the message failed to be sent
	if ((uint32_t)evt & (uint32_t)NOTIF_I_RETRYFAIL)
	{
		notify(pe, NOTIF_TX_ERR);
		return pe->postSendFailedState;
	}

	// Silence the compiler warning
	notify(pe, NOTIF_TX_ERR);
	return pe->postSendFailedState;
}

policy_engine_state pe_sink_epr_eval_cap(PolicyEngine *pe)
{
	pe->EPRTimeLastEvent = getTimeStamp();
	if (pe->pdbs_dpm_epr_evaluate_capability(&pe->recent_epr_capabilities, &pe->_last_dpm_request))
	{
		uint32_t pps_index  = PD_RDO_OBJPOS_GET(&pe->_last_dpm_request);
		pe->PPSTimerEnabled = (pe->recent_epr_capabilities.obj[pps_index - 1] & PD_PDO_TYPE) == PD_PDO_TYPE_AUGMENTED && (pe->recent_epr_capabilities.obj[pps_index - 1] & PD_APDO_TYPE) == PD_APDO_TYPE_PPS;
		pe->_last_dpm_request.hdr |= pe->hdr_template;
		return PESinkSelectCapTx;
	} 
	else
		return PESinkWaitCap;
}

policy_engine_state pe_sink_request_epr(PolicyEngine *pe)
{
	pe->EPRTimeLastEvent = getTimeStamp();
	pd_msg *epr_mode = &pe->tempMessage;
	epr_mode->hdr = pe->hdr_template | PD_MSGTYPE_EPR_MODE | PD_NUMOBJ(1);
	epr_mode->obj[0] = (0x01 << PD_EPR_MODE_ACTION_SHIFT) | (pe->device_epr_wattage << PD_EPR_MODE_DATA_SHIFT);
	return pe_start_message_tx(pe, PESinkReady, PESinkHardReset, epr_mode);
}

policy_engine_state pe_sink_send_epr_keep_alive(PolicyEngine *pe)
{
	while (pdb_msg_getOccupied(&pe->incomingMessages))
		pdb_msg_pop(&pe->incomingMessages, NULL);
	pe->negotiationOfEPRInProgress = true;
	pe->tempMessage.hdr = PD_HDR_EXT | pe->hdr_template | PD_NUMOBJ(1) | PD_MSGTYPE_EXTENDED_CONTROL;
	pe->tempMessage.exthdr = (PD_EXTHDR_DATA_SIZE & 2) << PD_EXTHDR_DATA_SIZE_SHIFT | PD_EXTHDR_CHUNKED;
	pe->tempMessage.data[0] = PD_EXTENDED_CONTROL_TYPE_EPR_KEEPALIVE;
	pe->tempMessage.data[1] = PD_EXTENDED_CONTROL_DATA_UNUSED;
	return pe_start_message_tx(pe, PESinkWaitEPRKeepAliveAck, PESinkReady, &pe->tempMessage);
}

policy_engine_state pe_sink_wait_epr_keep_alive_ack(PolicyEngine *pe)
{
	// We want to wait for an ACK for the epr message
	while (pdb_msg_getOccupied(&pe->incomingMessages))
	{
		pdb_msg_pop(&pe->incomingMessages, &pe->tempMessage);
		if (PD_MSGTYPE_GET(&pe->tempMessage) == PD_MSGTYPE_EXTENDED_CONTROL && pe->tempMessage.data[0] == PD_EXTENDED_CONTROL_TYPE_EPR_KEEPALIVE_ACK)
		{
			pe->negotiationOfEPRInProgress = false;
			pe->EPRTimeLastEvent = getTimeStamp();
			return PESinkReady;
		}
	}
	// Retry for ack
	return PESinkWaitEPRKeepAliveAck;
}