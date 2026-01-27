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
#include "policy_engine.h"
#include "fusb302_defines.h"
#include "fusb302b.h"
#include "pd.h"
#include <stdbool.h>
#ifdef PD_DEBUG_OUTPUT
#include "stdio.h"
#endif

#include "util.h"

const char* pe_getStateName(uint8_t state)
{
	const char *names[] = {"PEWaitingEvent",
							"PEWaitingMessageTx",
							"PEWaitingMessageGoodCRC",
							"PESinkStartup",
							"PESinkDiscovery",
							"PESinkSetupWaitCap",
							"PESinkWaitCap",
							"PESinkEvalCap",
							"PESinkSelectCapTx",
							"PESinkSelectCap",
							"PESinkWaitCapResp",
							"PESinkTransitionSink",
							"PESinkReady",
							"PESinkGetSourceCap",
							"PESinkGiveSinkCap",
							"PESinkHardReset",
							"PESinkTransitionDefault",
							"PESinkSoftReset",
							"PESinkSendSoftReset",
							"PESinkSendSoftResetTxOK",
							"PESinkSendSoftResetResp",
							"PESinkSendNotSupported",
							"PESinkHandleEPRChunk",
							"PESinkWaitForHandleEPRChunk",
							"PESinkNotSupportedReceived",
							"PESinkSourceUnresponsive",
							"PESinkEPREvalCap",
							"PESinkRequestEPR",
							"PESinkSendEPRKeepAlive",
							"PESinkWaitEPRKeepAliveAck",
							"PESinkMeasureCC1",
							"PESinkMeasureCC2",
	};
	const char* unknown = "Unknown";
	if (state >= sizeof(names))
		return unknown;
	return names[state];
}

bool pe_thread(PolicyEngine *pe)
{
	policy_engine_state stateEnter = pe->state;
	switch (pe->state) {

	case PESinkStartup:
		pe->state = pe_sink_startup(pe);
		break;
	case PESinkDiscovery:
		pe->state = pe_sink_discovery(pe);
		break;
	case PESinkMeasureCC1:
		pe->state = pe_sink_measure_cc1(pe);
		break;
	case PESinkMeasureCC2:
		pe->state = pe_sink_measure_cc2(pe);
		break;
	case PESinkSetupWaitCap:
		pe->state = pe_sink_setup_wait_cap(pe);
		break;
	case PESinkWaitCap:
		pe->state = pe_sink_wait_cap(pe);
		break;
	case PESinkEvalCap:
		pe->state = pe_sink_eval_cap(pe);
		break;
	case PESinkSelectCapTx:
		pe->state = pe_sink_select_cap_tx(pe);
		break;
	case PESinkSelectCap:
		pe->state = pe_sink_select_cap(pe);
		break;
	case PESinkWaitCapResp:
		pe->state = pe_sink_wait_cap_resp(pe);
		break;
	case PESinkTransitionSink:
		pe->state = pe_sink_transition_sink(pe);
		break;
	case PESinkReady:
		pe->state = pe_sink_ready(pe);
		break;
	case PESinkGetSourceCap:
		pe->state = pe_sink_get_source_cap(pe);
		break;
	case PESinkGiveSinkCap:
		pe->state = pe_sink_give_sink_cap(pe);
		break;
	case PESinkHardReset:
		pe->state = pe_sink_hard_reset(pe);
		break;
	case PESinkTransitionDefault:
		pe->state = pe_sink_transition_default(pe);
		break;
	case PESinkHandleSoftReset:
		pe->state = pe_sink_soft_reset(pe);
		break;
	case PESinkSendSoftReset:
		pe->state = pe_sink_send_soft_reset(pe);
		break;
	case PESinkSendSoftResetTxOK:
		pe->state = pe_sink_send_soft_reset_tx_ok(pe);
		break;
	case PESinkSendSoftResetResp:
		pe->state = pe_sink_send_soft_reset_resp(pe);
		break;
	case PESinkSendNotSupported:
		pe->state = pe_sink_send_not_supported(pe);
		break;
	case PESinkWaitForHandleEPRChunk:
		pe->state = pe_sink_wait_epr_chunk(pe);
		break;
	case PESinkHandleEPRChunk:
		pe->state = pe_sink_handle_epr_chunk(pe);
		break;
	case PESinkSourceUnresponsive:
		pe->state = pe_sink_source_unresponsive(pe);
		break;
	case PESinkNotSupportedReceived:
		pe->state = pe_sink_not_supported_received(pe);
		break;
	case PEWaitingEvent:
		pe->state = pe_sink_wait_event(pe);
		break;
	case PEWaitingMessageTx:
		pe->state = pe_sink_wait_send_done(pe);
		break;
	case PEWaitingMessageGoodCRC:
		pe->state = pe_sink_wait_good_crc(pe);
		break;
	case PESinkEPREvalCap:
		pe->state = pe_sink_epr_eval_cap(pe);
		break;
	case PESinkRequestEPR:
		pe->state = pe_sink_request_epr(pe);
		break;
	case PESinkSendEPRKeepAlive:
		pe->state = pe_sink_send_epr_keep_alive(pe);
		break;
	case PESinkWaitEPRKeepAliveAck:
		pe->state = pe_sink_wait_epr_keep_alive_ack(pe);
		break;
	default:
		pe->state = PESinkStartup;
		break;
	}
#ifdef PD_DEBUG_OUTPUT
	if (pe->state != PEWaitingEvent)
	printf("Current state - %s\r\n", pe_getStateName(pe->state));
#endif
	return (pe->state != stateEnter) && (pe->state != PEWaitingEvent);
}

void TimersCallback(PolicyEngine *pe)
{
	if (pe->PPSTimerEnabled)
	{ // Have to periodically re-send to keep the voltage level active
		if ((getTimeStamp() - pe->PPSTimeLastEvent) > (1000))
		{ // Send a new PPS message
			notify(pe, NOTIF_PPS_REQUEST);
			pe->PPSTimeLastEvent = getTimeStamp();
		}
	}
	if (pe->is_epr)
	{ // We need to engage in _some_ PD communication to stay in EPR mode
		if ((getTimeStamp() - pe->EPRTimeLastEvent) > (200))
			notify(pe, NOTIF_EPR_KEEPALIVE);
	}
}
policy_engine_state pe_start_message_tx(PolicyEngine *pe, policy_engine_state postTxState, policy_engine_state txFailState, pd_msg *msg)
{
#ifdef PD_DEBUG_OUTPUT
	printf("Starting message Tx - %02X\r\n", PD_MSGTYPE_GET(msg));
#endif
	if (PD_MSGTYPE_GET(msg) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(msg) == 0)
	{ // Clear MessageIDCounter
		pe->_tx_messageidcounter = 0;
		return postTxState; // Message is "done"
	}
	pe->postSendFailedState = txFailState;
	pe->postSendState = postTxState;
	msg->hdr &= ~PD_HDR_MESSAGEID;
	msg->hdr |= (pe->_tx_messageidcounter % 8) << PD_HDR_MESSAGEID_SHIFT;

	/* PD 3.0 collision avoidance */
	// if (isPD3_0())
	// { // If we're starting an AMS, wait for permission to transmit
	//   while (fusb.fusb_get_typec_current() != fusb_sink_tx_ok)
	//     osDelay(1000);
	// }
	/* Send the message to the PHY */
	fusb_send_message(pe->fusb, msg);
#ifdef PD_DEBUG_OUTPUT
	printf("Message queued to send\r\n");
#endif

	// Setup waiting for notification
	return pe_waitForEvent(pe, PEWaitingMessageTx, (uint32_t)NOTIF_RESET | (uint32_t)NOTIF_MSG_RX | (uint32_t)NOTIF_I_TXSENT | (uint32_t)NOTIF_I_RETRYFAIL, TICK_MAX_DELAY);
}

void pe_clearEvents(PolicyEngine *pe, uint32_t notification)
{
	pe->currentEvents &= ~notification;
}

policy_engine_state pe_waitForEvent(PolicyEngine *pe, policy_engine_state evalState, uint32_t notification, TICK_TYPE timeout)
{
	// Record the new state, and the desired notifications mask, then schedule the waiter state
	pe->waitingEventsMask = notification;
#ifdef PD_DEBUG_OUTPUT
	printf("Waiting for events %04X\r\n", (int)notification);
#endif

	// If notification is already present, we can continue straight to eval state
	if (pe->currentEvents & pe->waitingEventsMask) {
		return evalState;
	}
	// If waiting for message rx, but one is in the buffer, jump to eval
	if ((pe->waitingEventsMask & (uint32_t)NOTIF_MSG_RX) == (uint32_t)NOTIF_MSG_RX) {
		if (pdb_msg_getOccupied(&pe->incomingMessages) > 0) {
			pe->currentEvents |= (uint32_t)NOTIF_MSG_RX;
			return evalState;
		}
	}
	pe->postNotificationEvalState = evalState;
	if (timeout == TICK_MAX_DELAY) {
		pe->waitingEventsTimeout = TICK_MAX_DELAY;
	} else {
		pe->waitingEventsTimeout = getTimeStamp() + timeout;
	}
	return PEWaitingEvent;
}

void readPendingMessage(PolicyEngine *pe)
{
	bool pending = (pe->status.status1 & FUSB_STATUS1_RX_EMPTY) != FUSB_STATUS1_RX_EMPTY;
	while (pending)
	{
		// Read the message
		if (fusb_read_message(pe->fusb, &pe->irqMessage) == 0)
		{
			USBPD_CHARR(':', 'M', 'S', 'G', UI8_TO_HEX_ARR(PD_MSGTYPE_GET(&pe->irqMessage)));
			// If it's a Soft_Reset, go to the soft reset state
			if (PD_MSGTYPE_GET(&pe->irqMessage) == PD_MSGTYPE_SOFT_RESET && PD_NUMOBJ_GET(&pe->irqMessage) == 0)
			{ // PE transitions to its reset state
				USBPD_STR("=RST");
				notify(pe, NOTIF_RESET);
			}
			else
			{ // Pass the message to the policy engine.
				pdb_msg_push(&pe->incomingMessages, &pe->irqMessage);
				notify(pe, NOTIF_MSG_RX);
			}
		}
		else
		{
			USBPD_STR(":SOP");
		} // Invalid message or SOP'

		pending = fusb_rx_pending(pe->fusb);
	}
}

bool pe_IRQ_occured(PolicyEngine *pe)
{
	bool returnValue = false;
	// Read the FUSB302B status and interrupt registers
	if (fusb_get_status(pe->fusb, &pe->status))
	{
		USBPD_CHARR('+', UI8_TO_HEX_ARR(pe->status.status0a), UI8_TO_HEX_ARR(pe->status.status1a), UI8_TO_HEX_ARR(pe->status.interrupta), UI8_TO_HEX_ARR(pe->status.interruptb), UI8_TO_HEX_ARR(pe->status.status0), UI8_TO_HEX_ARR(pe->status.status1), UI8_TO_HEX_ARR(pe->status.interrupt));


		if ((pe->status.status0 & FUSB_STATUS0_CRC_CHK))
		{
			USBPD_STR(">CRCK");
		}

		// If the I_GCRCSENT flag is set, tell the Protocol RX thread
		// This means a message was received with a good CRC
		if (pe->status.interruptb & FUSB_INTERRUPTB_I_GCRCSENT)
		{
			USBPD_STR(">GCRC");
			readPendingMessage(pe);
			returnValue = true;
		}

		// If the RX_EMPTY flag is set, tell the Protocol RX thread
		// This means a message was received (not necessarily with good CRC?)
		if (!(pe->status.status1 & FUSB_STATUS1_RX_EMPTY))
		{
			USBPD_STR(">PMSG");
			//readPendingMessage(pe);
			//returnValue = true;
		}

		// If the I_TXSENT or I_RETRYFAIL flag is set, tell the Protocol TX thread
		if (pe->status.interrupta & FUSB_INTERRUPTA_I_TXSENT)
		{
			USBPD_STR(">TXST");
			notify(pe, NOTIF_I_TXSENT);
			returnValue = true;
		}
		if (pe->status.interrupta & FUSB_INTERRUPTA_I_RETRYFAIL)
		{
			USBPD_STR(">RTYF");
			notify(pe, NOTIF_I_RETRYFAIL);
			returnValue = true;
		}

		// If the I_OCP_TEMP and OVRTEMP flags are set, tell the Policy Engine thread
		if ((pe->status.interrupta & FUSB_INTERRUPTA_I_OCP_TEMP) && (pe->status.status1 & FUSB_STATUS1_OVRTEMP))
		{
			USBPD_STR(">OVTP");
			notify(pe, NOTIF_I_OVRTEMP);
			returnValue = true;
		}

		if ((pe->status.status0 & FUSB_STATUS0_BC_LVL) == 0 && (pe->status.status0 & FUSB_STATUS0_VBUSOK) == 0 && pe->state != PESinkDiscovery && pe->state != PESinkStartup)
		{ // Lost VBUS voltage on selected CC line, likely disconnected source - go into startup->discovery to rediscover CC line
			USBPD_STR("!CCLOST");
			pe->state = PESinkStartup;
			if (!fusb_reset(pe->fusb))
				return false;
			return false;
		}
	}
	return returnValue;
}