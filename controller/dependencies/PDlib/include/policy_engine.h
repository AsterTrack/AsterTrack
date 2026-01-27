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

#ifndef PDB_POLICY_ENGINE_H
#define PDB_POLICY_ENGINE_H
#include "fusb302b.h"
#include "pdb_msg.h"
#include "pdb_msg_queue.h"
#include <string.h>
#include <stdint.h>

#define EVENT_MASK(x) (1 << x)

typedef enum {
	PEWaitingEvent              = 0,  // Meta state: waiting for event or timeout
	PEWaitingMessageTx          = 1,  // Meta state: waiting for message tx to confirm
	PEWaitingMessageGoodCRC     = 2,  // We have sent a message, waiting for a GoodCRC to come back
	PESinkStartup               = 3,  // Start of state machine
	PESinkDiscovery             = 4,  // initiating selection of CC lines as source sends capability packets
	PESinkSetupWaitCap          = 5,  // Setup events wanted by waitCap
	PESinkWaitCap               = 6,  // Waiting for source
	PESinkEvalCap               = 7,  // Evaluating the source provided capabilities message
	PESinkSelectCapTx           = 8,  // Send cap selected
	PESinkSelectCap             = 9,  // Wait send ok
	PESinkWaitCapResp           = 10, // Wait response message
	PESinkTransitionSink        = 11, // Transition to sink mode
	PESinkReady                 = 12, // Normal operational state, all is good
	PESinkGetSourceCap          = 13, // Request source capabilities
	PESinkGiveSinkCap           = 14, // Device has been requested for its capabilities
	PESinkHardReset             = 15, // Send a hard reset
	PESinkTransitionDefault     = 16, // Transition to reset
	PESinkHandleSoftReset       = 17, // Soft reset received
	PESinkSendSoftReset         = 18, // Send soft reset (comms resync)
	PESinkSendSoftResetTxOK     = 19, // Sending soft reset, waiting message tx
	PESinkSendSoftResetResp     = 20, // Soft reset waiting for response
	PESinkSendNotSupported      = 21, // Send a NACK message
	PESinkHandleEPRChunk        = 22, // A chunked EPR message received
	PESinkWaitForHandleEPRChunk = 23, // A chunked EPR message received
	PESinkNotSupportedReceived  = 24, // One of our messages was not supported
	PESinkSourceUnresponsive    = 25, // A resting state for a source that doesnt talk (aka no PD)
	PESinkEPREvalCap            = 26, // Evaluating the source's provided EPR capabilities message
	PESinkRequestEPR            = 27, // We're requesting the EPR capabilities
	PESinkSendEPRKeepAlive      = 28, // Send the EPR Keep Alive packet
	PESinkWaitEPRKeepAliveAck   = 29, // wait for the Source to acknowledge the keep alive
	PESinkMeasureCC1            = 30, // measure CC lines 1 before making a selection
	PESinkMeasureCC2            = 31, // measure CC lines 2 before making a selection
	PESinkWaitReplug            = 32, // wait for a timeout before attempting to replug
} policy_engine_state;

typedef enum {
	NOTIF_RESET          = EVENT_MASK(0),  // 1
	NOTIF_MSG_RX         = EVENT_MASK(1),  // 2
	NOTIF_TX_DONE        = EVENT_MASK(2),  // 4
	NOTIF_TX_ERR         = EVENT_MASK(3),  // 8
	NOTIF_HARD_SENT      = EVENT_MASK(4),  // 10
	NOTIF_I_OVRTEMP      = EVENT_MASK(5),  // 20
	NOTIF_PPS_REQUEST    = EVENT_MASK(6),  // 40
	NOTIF_GET_SOURCE_CAP = EVENT_MASK(7),  // 80
	NOTIF_NEW_POWER      = EVENT_MASK(8),  // 100
	NOTIF_I_TXSENT       = EVENT_MASK(9),  // 200
	NOTIF_I_RETRYFAIL    = EVENT_MASK(10), // 400
	NOTIF_TIMEOUT        = EVENT_MASK(11), // 800 Internal notification for timeout waiting for an event
	NOTIF_REQUEST_EPR    = EVENT_MASK(12), // 1000
	NOTIF_EPR_KEEPALIVE  = EVENT_MASK(13), // 2000
	NOTIF_ALL            = (EVENT_MASK(14) - 1),
} Notifications;

// Functions required to be created by the user for their end application
/*
	* Create a Request message based on the given Source_Capabilities message. If
	* capabilities is NULL, the last non-null Source_Capabilities message passes
	* is used.  If none has been provided, the behavior is undefined.
	*
	* Returns true if sufficient power is available, false otherwise.
	*/
typedef bool (*EvaluateCapabilityFunc)(const pd_msg *capabilities, pd_msg *request);
typedef bool (*EPREvaluateCapabilityFunc)(const epr_pd_msg *capabilities, pd_msg *request);
/*
	* Create a Sink_Capabilities message for our current capabilities.
	*/
typedef void (*SinkCapabilityFunc)(pd_msg *cap, const bool isPD3);

typedef struct
{
	FUSB302 *fusb;
	SinkCapabilityFunc pdbs_dpm_get_sink_capability;
	EvaluateCapabilityFunc pdbs_dpm_evaluate_capability;
	EPREvaluateCapabilityFunc pdbs_dpm_epr_evaluate_capability;

	int current_voltage_mv; // The current voltage PD is expecting
	int _requested_voltage; // The voltage the unit wanted to requests
	bool _unconstrained_power; // If the source is unconstrained
	uint8_t _tx_messageidcounter; // Counter for messages sent to be packed into messages sent
	uint16_t hdr_template; // PD message header template

	// Whether or not we have an explicit contract
	bool _explicit_contract;
	bool negotiationOfEPRInProgress;
	// The number of hard resets we've sent
	int8_t _hard_reset_counter;
	// The index of the first PPS APDO
	uint8_t _pps_index;

	// Event group
	// Temp messages for storage
	pd_msg tempMessage;
	pdb_msg_queue incomingMessages;
	pd_msg irqMessage; // irq will unpack recieved message to here
	pd_msg _last_dpm_request;
	policy_engine_state state;
	// Read a pending message into the temp message
	bool PPSTimerEnabled;
	TICK_TYPE PPSTimeLastEvent, EPRTimeLastEvent;
	epr_pd_msg recent_epr_capabilities;
	uint8_t device_epr_wattage;
	bool sourceIsEPRCapable;
	bool is_epr;

	fusb_status status;

	policy_engine_state postNotificationEvalState;
	policy_engine_state postSendState;
	policy_engine_state postSendFailedState;
	uint32_t waitingEventsMask;
	TICK_TYPE waitingEventsTimeout;
	uint32_t currentEvents;
	TICK_TYPE timestampNegotiationsStarted;
	uint8_t ccMeasurement;
} PolicyEngine;

static inline void pe_init(PolicyEngine *pe,
			FUSB302 *fusb,
			SinkCapabilityFunc sinkCapabilities,
			EvaluateCapabilityFunc evalFunc,
			EPREvaluateCapabilityFunc eprEvalFunc,
			const uint8_t device_max_epr_wattage)
{
	pe->fusb = fusb;
	pe->pdbs_dpm_get_sink_capability = sinkCapabilities;
	pe->pdbs_dpm_evaluate_capability = evalFunc;
	pe->pdbs_dpm_epr_evaluate_capability = eprEvalFunc;
	pe->device_epr_wattage = device_max_epr_wattage;
	pe->hdr_template = PD_DATAROLE_UFP | PD_POWERROLE_SINK;
	pe->_pps_index = 0xFF;
	pe->state = PESinkStartup;
}

// Runs the internal thread, returns true if should re-run again immediately if possible
bool pe_thread(PolicyEngine *pe);

// Returns true if headers indicate PD3.0 compliant
static inline bool isPD3_0(PolicyEngine *pe)
{
	return (pe->hdr_template & PD_HDR_SPECREV) == PD_SPECREV_3_0;
}
static inline bool hasExplicitContract(PolicyEngine *pe)
{
	return pe->_explicit_contract;
}
static inline bool inReadyState(PolicyEngine *pe)
{
	return pe->state == PESinkReady || (pe->state == PEWaitingEvent && pe->postNotificationEvalState == PESinkReady);
}
static inline bool isWaitingOnTimer(PolicyEngine *pe)
{
	return pe->state == PESinkMeasureCC1 || pe->state == PESinkMeasureCC2 || pe->state == PESinkWaitReplug;
}
static inline bool NegotiationTimeoutReached(PolicyEngine *pe, uint8_t timeout)
{
	// Check if have been waiting longer than timeout without finishing
	// If so force state into the failed state and return true

	// Timeout is in 100ms increments
	if (timeout)
	{ // If the system ticks is greater than the specified timeout then we call it all off
		if ((getTimeStamp() - pe->timestampNegotiationsStarted) > (timeout * 100))
		{
			// state = PESinkSourceUnresponsive;
			return true;
		}
	}
	return false;
}
static inline bool setupCompleteOrTimedOut(PolicyEngine *pe, uint8_t timeout)
{
	if (pe->negotiationOfEPRInProgress)
		return false;
	if (pe->_explicit_contract)
		return true;
	if (pe->state == PESinkSourceUnresponsive)
		return true;
	if (inReadyState(pe))
		return true;
	if (NegotiationTimeoutReached(pe, timeout))
		return true;
	return false;
}
// Has pd negotiation completed
static inline bool pdHasNegotiated(PolicyEngine *pe)
{
	if (pe->state == PESinkSourceUnresponsive)
		return false;
	return pe->negotiationOfEPRInProgress || pe->_explicit_contract;
}

// Call this periodically, by the spec at least once every 10 seconds for PPS. <5 is recommended
// If in EPR should be called every 4-400 milliseconds
void TimersCallback(PolicyEngine *pe);

bool pe_IRQ_occured(PolicyEngine *pe);
const char *pe_getStateName(uint8_t state);
// Useful for debug reading out
static inline int currentStateCode(PolicyEngine *pe, const bool noWait)
{
	if (noWait && (pe->state == PEWaitingEvent))
		return (int)pe->postNotificationEvalState;
	return (int)pe->state;
}

// Send a notification
static inline void notify(PolicyEngine *pe, Notifications notification)
{
	pe->currentEvents |= (uint32_t)notification;
}
void pe_clearEvents(PolicyEngine *pe, uint32_t notification);
policy_engine_state pe_waitForEvent(PolicyEngine *pe, policy_engine_state evalState, uint32_t notification, TICK_TYPE timeout);

static inline bool pe_renegotiate(PolicyEngine *pe)
{
	if (!inReadyState(pe))
		return false;
	notify(pe, NOTIF_NEW_POWER);
	return true;
}

void pe_forceReplug(PolicyEngine *pe);

policy_engine_state pe_sink_startup(PolicyEngine *pe);
policy_engine_state pe_sink_discovery(PolicyEngine *pe);
policy_engine_state pe_sink_measure_cc1(PolicyEngine *pe);
policy_engine_state pe_sink_measure_cc2(PolicyEngine *pe);
policy_engine_state pe_sink_setup_wait_cap(PolicyEngine *pe);
policy_engine_state pe_sink_wait_cap(PolicyEngine *pe);
policy_engine_state pe_sink_eval_cap(PolicyEngine *pe);
policy_engine_state pe_sink_select_cap(PolicyEngine *pe);
policy_engine_state pe_sink_select_cap_tx(PolicyEngine *pe);
policy_engine_state pe_sink_wait_cap_resp(PolicyEngine *pe);
policy_engine_state pe_sink_transition_sink(PolicyEngine *pe);
policy_engine_state pe_sink_ready(PolicyEngine *pe);
policy_engine_state pe_sink_get_source_cap(PolicyEngine *pe);
policy_engine_state pe_sink_give_sink_cap(PolicyEngine *pe);
policy_engine_state pe_sink_hard_reset(PolicyEngine *pe);
policy_engine_state pe_sink_transition_default(PolicyEngine *pe);
policy_engine_state pe_sink_soft_reset(PolicyEngine *pe);
policy_engine_state pe_sink_send_soft_reset_resp(PolicyEngine *pe);
policy_engine_state pe_sink_send_soft_reset_tx_ok(PolicyEngine *pe);
policy_engine_state pe_sink_send_soft_reset(PolicyEngine *pe);
policy_engine_state pe_sink_send_not_supported(PolicyEngine *pe);
policy_engine_state pe_sink_handle_epr_chunk(PolicyEngine *pe);
policy_engine_state pe_sink_wait_epr_chunk(PolicyEngine *pe);
policy_engine_state pe_sink_not_supported_received(PolicyEngine *pe);
policy_engine_state pe_sink_source_unresponsive(PolicyEngine *pe);
policy_engine_state pe_sink_wait_event(PolicyEngine *pe);
policy_engine_state pe_sink_wait_send_done(PolicyEngine *pe);
policy_engine_state pe_sink_wait_good_crc(PolicyEngine *pe);
policy_engine_state pe_sink_epr_eval_cap(PolicyEngine *pe);
policy_engine_state pe_sink_request_epr(PolicyEngine *pe);
policy_engine_state pe_sink_send_epr_keep_alive(PolicyEngine *pe);
policy_engine_state pe_sink_wait_epr_keep_alive_ack(PolicyEngine *pe);
// Sending messages, starts send and returns next state
policy_engine_state pe_start_message_tx(PolicyEngine *pe, policy_engine_state postTxState, policy_engine_state txFailState, pd_msg *msg);
policy_engine_state pe_sink_wait_replug(PolicyEngine *pe);

#endif /* PDB_POLICY_ENGINE_H */
