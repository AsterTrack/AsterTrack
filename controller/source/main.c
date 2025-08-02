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

#include "ch32v30x.h"
#include "ch32v30x_gpio.h"
#include "compat.h"

#include "util.h"
#include "packetHub.h"
#include "uartd.h"
#include "uart_driver.h"
#include "usbd.h"
#include "usbd_conf.h"
#include "config.h"


/* Defines */

// Timing (how often the intervals/timeouts are checked themselves depends on their position within the main loop)
#define UART_COMM_TIMEOUT		350*TICKS_PER_MS		// Comm timeout at which a camera is considered disconnected
#define UART_PING_INTERVAL		100*TICKS_PER_MS		// Interval in which Tracking Cameras are pinged when idle
#define UART_TIME_SYNC_INTERVAL	10*TICKS_PER_MS			// Minimum interval used for time sync with Tracking Cameras (doesn't need to be super accurate)

#define USB_TIME_SYNC_INTERVAL	1*TICKS_PER_MS			// Minimum interval used for time sync with host (needs to be as accurate as possible)
#define USB_COMM_TIMEOUT		200*TICKS_PER_MS		// Comm timeout at which the host is considered disconnected
#define USB_COMM_TIMEOUT_STR	20*TICKS_PER_MS			// Comm timeout at which the host is considered disconnected when timesync is enabled

#define SYNC_PULSE_WIDTH_US		10

#define WWDG_TIMEOUT			(0x40 | 10)


/* Function Prototypes */

// Packet Hub
static void InitPacketHub();
static void SetupPacketHubSinks();

/* Variables */

union VersionDesc version; // Initialised at the beginning

// PacketHub is responsible for distributing data from UART sources to USB sinks
static Hub packetHub;

// Stages to start streaming and controller/camera lifetime
// Stage 1: Open Channels
//  - Host sets up configuration and interrupt channel - see usbd_set_interface
//  - At this point, debug should be expected to be sent over these channels, as well as packets from the camera (if any)
//  - If host stops asking for packets, the controller will assume a dropped connection (can't deconfigure itself)
// Stage 2: Time Sync
//  - Continued communication with the host and cameras for timesync purposes
//  - At this point, controller sends packets regularly for timing purposes and expects host to receive them
//  - If host stops asking for packets, the controller will assume a dropped connection and return to Open Channel
// Stage 3: Setup Sync Source
//  - Setup parameters for sync, e.g. external, or generated internally
//  - Technically optional if all cameras will be running free (masked)
// Stage 4: Tell cameras to start streaming mode
//  - This stage should not need controller to know
//  - Eventually, the camera will send a packet to notify host that it is ready for streaming (or an error)
// Stage 5: Setup sync mask for cameras
//  - Host tells the controller to start sending sync pulses and packets to specific cameras
//  - Goal is that these cameras are already setup to immediately process a frame, and with a good timesync with the controller
// Stage 6: Error
//  - If a camera has an error, the sync pulses to the camera should be stopped
//  - Either the controller reacts itself, or the host instructs the controller
// Stage 7: Recovery
//  - As soon as it recovers, the host can instruct the camera to start streaming mode again (see Stage 4)
//  - And eventually also to start streaming again by sending sync pulses to the camera (see Stage 5)


// Controller state
static volatile bool commChannelsOpen; // Stage 1
static volatile bool enforceTimeSync; // Stage 2
//	bool PortState::cameraEnabled; // Stage 4
//	bool PortState::frameSyncEnabled; // Stage 5
static volatile uint32_t enabledSyncPinsGPIOD; // Stage 5

// Camera state
typedef struct
{
	enum ControllerCommState comm;
	struct IdentPacket identity;

	// Streaming state
	bool cameraEnabled; // TODO: Needed? Probably not. But nice to know
	bool frameSyncEnabled;
} CameraState;
CameraState camStates[UART_PORT_COUNT];

// Sync source setup (Stage 3)
enum FrameSignalSource { FrameSignal_None, FrameSignal_Generating, FrameSignal_External };
static volatile enum FrameSignalSource frameSignalSource; // Stage 3
static volatile uint32_t framerate;
static volatile uint32_t frametimeUS;

// Streaming state
static volatile TimePoint curSOF = 0; // NOT USB frame, but camera frame time
static volatile uint32_t curFrameID = 0; // More than half a year before it lapses at current max FPS of ~210
static volatile TimePoint cachedSOF = 0;
static volatile uint32_t cachedFrameID = 0;

// Times for supervision
static TimePoint startup = 0;
static TimePoint lastPing = 0;
static TimePoint lastUSBSOF;
static volatile TimePoint lastUSBPacket = 0;
static volatile TimePoint lastIntPacket = 0;
//static StatValue packetLatency, packetDiff;

// Fixed UART Messages
static struct IdentPacket ownIdent;
static uint8_t ownIdentPacket[UART_PACKET_OVERHEAD_SEND+IDENT_PACKET_SIZE];
static uint8_t rcvIdentPacket[IDENT_PACKET_SIZE];

// For sending event and debug logs over interrupt transfers
#ifdef DEBUG_USE_INT
PacketRef debugUSBPacket;
#endif
#ifdef EVENT_USE_INT
PacketRef eventUSBPacket;
#endif
// For sending occasional packets from camera to host via control transfers when comm channels are not open
volatile uint8_t packetSendCounter = 0;
volatile PacketRef *packetUSBPacket = NULL;

volatile PacketRef *lastSOFPacket = NULL;
volatile TimePoint lastUARTActivity;

/* Functions */

static void uart_set_identification()
{ // This is theoretically constant, but requires code to initialise nicely
	UARTPacketRef *uartIdentPacket = (UARTPacketRef*)ownIdentPacket;
	ownIdent = (struct IdentPacket){ .device = DEVICE_TRCONT, .id = 0, .type = INTERFACE_UART, .version = version };
	storeIdentPacket(ownIdent, uartIdentPacket->data);
	finaliseUARTPacket(uartIdentPacket, (struct PacketHeader){ .tag = PACKET_IDENT, .length = IDENT_PACKET_SIZE });
}

static void sendSOFPackets(uint32_t frameID, TimePoint SOF);
static void cleanSendingState();

int main()//(uint16_t after, uint16_t before, uint16_t start)
{
	// The Flash+SRAM configuration in link_CH32V307VCTx.ld
	// NEEDS to be configured first
	// Either by code (effect after reset) or in windows tool
	// https://programmer.ink/think/risc-v-mcu-development-tutorial-ch32v307-configuring-flash-and-ram.html
	// Figure out detail and use that if applicable
	// EDIT: Implemented a flashable configurator, but only works with Debug Restart (see details in main_configurator)

	/* __asm volatile(
	"lui a0, 0x1ffff\n"
	"li a1, 0x300\n");
	__asm volatile(
	"sh a1, 0x1b0(a0)\n");
	__asm volatile(
"1:  lui s2, 0x40022\n"
	"lw a0, 0xc(s2)\n"
	"andi a0, a0, 1\n"
	"bnez a0, 1b\n"); */

	/* uint16_t *ptr = *((uint16_t*)0x1ffff1b0);

	uint16_t val1 = *ptr;
	bool busy1 = FLASH->STATR&1;

	*ptr = 0x0300;

	uint16_t val2 = *ptr;
	bool busy2 = FLASH->STATR&1;
	
	int it = 0;
	while(FLASH->STATR&1) it++;

	uint16_t val3 = *ptr;
	bool busy3 = FLASH->STATR&1; */
	
	//ERR_CHARR('/', 'B', UI32_TO_HEX_ARR(before), 'A', UI32_TO_HEX_ARR(after), 's', UI32_TO_HEX_ARR(start));

	//SystemInit(); // Already called in startup code
	SysTick->CNT = 0; // Reset SysTick to 0 by writing anything

	// Base setup
	Setup_Peripherals();
	
	//uint32_t corecfgr, corecfgr2;
	//__asm volatile ( "mv %0," "t2" : "=r" (corecfgr) );
	//__asm volatile ( "mv %0," "t3" : "=r" (corecfgr2) );
	//ERR_CHARR('/', 'C', UI32_TO_HEX_ARR(corecfgr), ':', UI32_TO_HEX_ARR(corecfgr2));

	DEBUG_STR("/START");

	// Startup sequence
	startup = GetTimePoint();

	// Initialise version
	version = GetVersion(0, 0, 0);

#if defined(ENABLE_EVENTS)
	for (int i = 0; i < CONTROLLER_EVENT_MAX; i++)
		eventFilter[i] = 0; // Disable all by default
#endif

	// Init Packet hub (distributing incoming UART packages to USB endpoints)
	InitPacketHub();

	// Init USB device stack
	usbd_init();

	// Init UART device
	uartd_init();

	// System reset
	//PFIC->SCTLR = 0x8000; // Doesn't work
	//PFIC->CFGR = NVIC_KEY3 | 0x0080; // Works

	// Initialise Identification
	uart_set_identification();

	/* RCC->APB1PRSTR |= RCC_APB1Periph_WWDG;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_WWDG;
	RCC->APB1PCENR |= RCC_APB1Periph_WWDG;
	WWDG->CFGR |= WWDG_CFGR_EWI | (WWDG_TIMEOUT & WWDG_CFGR_W);
	//WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);
	WWDG->CTLR = WWDG_CTLR_WDGA (WWDG_TIMEOUT & WWDG_CTLR_T); */
	//RCC->APB1PRSTR |= RCC_APB1Periph_WWDG;
	//RCC->APB1PRSTR &= ~RCC_APB1Periph_WWDG;

	// Start LED interval timer
	StartTimer(TIM6, 10); // 100us, just to wait

	// Start of main loop
	TimePoint lastLoopIT = GetTimePoint();
	TimePoint lastTimeCheck = GetTimePoint();

	// TODO: Use USBHS_UMS_SIE_FREE in USBHSD->MIS_ST to know if a transfer is currently ongoing?
	while (1)
	{
		TimePoint now = GetTimePoint();
		TimeSpan loopDiff = GetTimeSpanUS(lastLoopIT, now);
#if __OPTIMIZE__ == 1
		if (loopDiff > 100)
#else
		if (loopDiff > 200)
#endif
			WARN_CHARR('/', 'L', 'A', 'G', INT99999999_TO_CHARR(loopDiff));
		lastLoopIT = now;

		WWDG->CTLR = (WWDG_TIMEOUT & WWDG_CTLR_T);
		__enable_irq();

		/* for (int p = 0; p < UART_PORT_COUNT; p++)
		{
			uartd_process_port(p);
		} */

		if (frameSignalSource != FrameSignal_None && cachedFrameID != curFrameID)
		{ // Send SOF packet to both host and cameras
			// This is not super time-critical as it is properly timestamped, so do in main loop not interrupt

			int diff;
			ATOMIC_SINGLE(
				diff = curFrameID-cachedFrameID;
				cachedSOF = curSOF;
				cachedFrameID = curFrameID;
			)

			if (diff != 1)
			{
				ERR_STR("#DROPPED_SOF:");
				ERR_CHARR(INT99_TO_CHARR(diff-1));
			}

			EnterPacketHubZone();
			sendSOFPackets(cachedFrameID, cachedSOF);
			LeavePacketHubZone();
		}

		now = GetTimePoint();
		static TimePoint lastTimeMicroCheck = 0;
		if (now-lastTimeMicroCheck < 100*TICKS_PER_US)
			continue;
		lastTimeMicroCheck = now;

		/* // Test if any debugs are dropped
		static int counter = 0;
		if (commChannelsOpen)
		{
			counter++;
			if (counter%10 == 0)
				ERR_CHARR('&', INT99_TO_CHARR(counter/10));
		} */

		if (enforceTimeSync)
		{ // Enforce a certain density of time syncs with both host and connected cameras

			// Check USB endpoint timeouts
			// Need accurate data on when USB endpoints are send, else errors may occur
			// This changes constantly due to drift, so occasionally packages have to be send
			// Especially on low framerates or when a lot of endpoints are configured but rarely used
			for (uint_fast8_t i = 0; i < packetHub.sinkCount; i++)
			{
				TimeSpan timeSinceLastComm = now - packetHub.sinks[i].lastSentEnd;
				static TimeSpan lastOverflow;
				if (timeSinceLastComm > USB_TIME_SYNC_INTERVAL && !packetHub.sinks[i].sending)
				{ // Set sending. Empty package still gives a sent callback for timing
					EnterPacketHubZone();
					if (setSinkSending(&packetHub, &packetHub.sinks[i], NULL))
						lastOverflow = timeSinceLastComm;
					LeavePacketHubZone();
				}
				else if (timeSinceLastComm - lastOverflow > USB_TIME_SYNC_INTERVAL)
				{ // Set sending. Empty package still gives a sent callback for timing
					lastOverflow = timeSinceLastComm;
					KERR_CHARR('/', 'S', 'T', 'L', '0'+i, packetHub.sinks[i].sending? 'S' : 'N');
				}
			}
		}

		now = GetTimePoint();
		static TimePoint lastTimeSendCheck = 0;
		if (now-lastTimeSendCheck < 1000 * TICKS_PER_US)
			continue;
		lastTimeSendCheck = now;

		if (enforceTimeSync)
		{ // Enforce a certain density of time syncs with both host and connected cameras
			
			// Send Sync Packets to cameras
			const struct PacketHeader header = { .tag = PACKET_SYNC, .length = SYNC_PACKET_SIZE, .frameID = curFrameID };
			UARTPacketRef *packet = allocateUARTPacket(header.length);
			const struct SyncPacket syncPacket = { GetTimePointUS() };
			storeSyncPacket(syncPacket, packet->data);
			finaliseUARTPacket(packet, header);
			for (uint_fast8_t i = 0; i < UART_PORT_COUNT; i ++)
			{
				if (camStates[i].comm != CommPiReady) continue;
				TimeSpan timeSinceLastTimeSync = now - portStates[i].lastTimeSync;
				if (timeSinceLastTimeSync > UART_TIME_SYNC_INTERVAL)
				{
					uartd_send(i, packet, UART_PACKET_OVERHEAD_SEND+SYNC_PACKET_SIZE, false);
					portStates[i].lastTimeSync = now;
				}
			}
		}

#if defined(ENABLE_LOG) && defined(DEBUG_USE_INT)
		if (commChannelsOpen)
		{ // Setup real-time transfers to use
			EnterPacketHubZone();
			uint8_t *dbgDataPtr = &debugBuffer[debugTail];
			int size = debugHead - debugTail;
			if (size != 0 && debugTail == debugSending)
			{
				debugSending = debugSending+1; // "Lock" as quickly as possible
				if (size < 0) size = DEBUG_BUFFER_SIZE-debugTail;
				int prependSize = USB_PACKET_HEADER + BLOCK_HEADER_SIZE;
				uint8_t alignment = (uint32_t)(dbgDataPtr - prependSize) & USB_PACKET_ALIGNMENT;
				prependSize += alignment;
				debugUSBPacket.buffer = dbgDataPtr - prependSize;
				
				int maxLength = USBD_HS_EP_INT_IN_SIZE - prependSize;
				if (size > maxLength) size = maxLength;
				debugUSBPacket.size = (uint_fast16_t)(prependSize + size);

				const struct BlockHeader header = { .isSignal = true, .signal = SIGNAL_DEBUG, .skip = alignment, .size = size };
				storeBlockHeader(header, debugUSBPacket.buffer+USB_PACKET_HEADER);

				if (!queuePacket(&packetHub, &debugUSBPacket))
				{
					ERR_STR("#FailedDebugQueue");
					debugSending = debugTail;
				}
				else
				{
					debugSending = (debugTail+size) % DEBUG_BUFFER_SIZE;
				}
			}
			LeavePacketHubZone();
		}
#endif
#if defined(ENABLE_EVENTS) && defined(EVENT_USE_INT)
		if (commChannelsOpen)
		{ // Setup real-time transfers to use
			EnterPacketHubZone();
			uint8_t *evtDataPtr = &eventBuffer[eventTail];
			int size = eventHead - eventTail;
			if (size != 0 && eventTail == eventSending)
			{
				eventSending = eventSending+1; // "Lock" as quickly as possible
				if (size < 0) size = EVENT_BUFFER_SIZE-eventTail;
				int prependSize = USB_PACKET_HEADER + BLOCK_HEADER_SIZE;
				uint8_t alignment = (uint32_t)(evtDataPtr - prependSize) & USB_PACKET_ALIGNMENT;
				prependSize += alignment; // Alignment should be 1, since evtDataPtr should already be aligned
				eventUSBPacket.buffer = evtDataPtr - prependSize;
				
				int maxLength = USBD_HS_EP_INT_IN_SIZE - prependSize;
				if (size > maxLength) size = maxLength;
				size -= size % EVENT_BYTE_SIZE; // Only send full events
				eventUSBPacket.size = (uint_fast16_t)(prependSize + size);

				const struct BlockHeader header = { .isSignal = true, .signal = SIGNAL_EVENT, .skip = alignment, .size = size };
				storeBlockHeader(header, eventUSBPacket.buffer+USB_PACKET_HEADER);

				if (!queuePacket(&packetHub, &eventUSBPacket))
				{
					ERR_STR("#FailedEventQueue");
					eventSending = eventTail;
				}
				else
				{
					eventSending = (eventTail+size) % EVENT_BUFFER_SIZE;
				}
				eventSending = (eventTail+size) % EVENT_BUFFER_SIZE;
			}
			LeavePacketHubZone();
		}
#endif

		now = GetTimePoint();
		if (now-lastTimeCheck < 20*TICKS_PER_MS)
			continue;
		lastTimeCheck = now;

		// Check all kinds of timeouts

		// Check UART timeouts
		for (uint_fast8_t i = 0; i < UART_PORT_COUNT; i++)
		{
			if (camStates[i].comm == CommNoCon) continue;
			TimeSpan timeSinceLastComm = now - portStates[i].lastComm;
			if (timeSinceLastComm > UART_COMM_TIMEOUT)
			{ // Reset Comm after silence
				EnterPacketHubZone(); // Mostly for the debug
				WARN_STR("!CommTimeout:");
				WARN_CHARR(INT9_TO_CHARR(i), ':', INT9999_TO_CHARR(timeSinceLastComm/TICKS_PER_MS));
				uartd_nak_int(i);
				uartd_reset_port_int(i);
				LeavePacketHubZone();
				// Configurator is notified by its camera iteration check
			}
		}

		// Check ping
		if (now-lastPing > UART_PING_INTERVAL)
		{ // Send ping occasionally
			for (uint_fast8_t i = 0; i < UART_PORT_COUNT; i ++)
			{
				if ((camStates[i].comm & CommReady) != CommReady)
					continue; // Comms not set up, doesn't need ping
				if ((camStates[i].comm == CommPiReady) && (enforceTimeSync || frameSignalSource == FrameSignal_Generating))
					continue; // Already sending packets regularly, doesn't need ping
				uartd_send(i, msg_ping, sizeof(msg_ping), true);
			}
			lastPing = now;
		}

		if (commChannelsOpen)
		{
			TimeSpan inactive = now-lastUSBPacket;
			if (inactive > USB_COMM_TIMEOUT || (enforceTimeSync && inactive > USB_COMM_TIMEOUT_STR))
			{ // Not connected anymore, Configurator possibly crashed / stalled
				// -> In streaming mode, interrupt endpoints are used
				// -> If it was still connected, at least USB hardware would check and accept packets
				EnterPacketHubZone();

				// TODO: Properly reverse all streaming stages. Should be fine, but still unsure.

				// Stage 4&5: Ensure cameras are notified and stop streaming
				const struct PacketHeader header = { .tag = PACKET_CFG_MODE, .length = 1 };
				UARTPacketRef *packet = allocateUARTPacket(header.length);
				packet->data[0] = TRCAM_STANDBY;
				finaliseUARTPacket(packet, header);
				for (int i = 0; i < UART_PORT_COUNT; i++)
				{
					if (camStates[i].comm == CommPiReady && camStates[i].cameraEnabled)
						uartd_send(i, packet, UART_PACKET_OVERHEAD_SEND+1, false);
					camStates[i].frameSyncEnabled = false;
				}

				// Stage 3: Disable frame sync
				curFrameID = 0;
				enabledSyncPinsGPIOD = 0;
				SYNC_Reset();
				if (frameSignalSource == FrameSignal_Generating)
					StopTimer(TIM3);
				frameSignalSource = FrameSignal_None;

				// Stage 2: Disable time sync
				enforceTimeSync = false;

				// Stage 1: Close comm channels
				//commChannelsOpen = false; // Cannot disable, still set by interface
				
				// This won't be reset if interface isn't unset first, so clear now
				cleanSendingState();
				resetPacketHub(&packetHub);

				LeavePacketHubZone();
				ERR_CHARR('/', 'U', 'C', 'L'); // USB Connection Lost
			}
		}

		/* static TimePoint lastPDTimer = 0;
		if (now-lastPDTimer > 8000*TICKS_PER_MS)
		{
			now = GetTimePoint();
			pd_timer();
			lastPDTimer = now;

			TimeSpan lag = GetTimeSinceUS(now);
			if (lag > 100)
			{
				WARN_STR("!PDtimLag:");
				WARN_CHARR(INT9999_TO_CHARR(lag));
			}
		} */

		/* static TimePoint lastStatTimer = 0;
		if (now-lastStatTimer > 10000*TICKS_PER_MS)
		{
			lastStatTimer = now;
			int devL = (int)sqrtf(packetLatency.var)/TICKS_PER_US * 3;
			int devD = (int)sqrtf(packetDiff.var)/TICKS_PER_US * 3;
			KERR_CHARR('^', 'T', 
			'L', INT9999_TO_CHARR((int)(packetLatency.min/TICKS_PER_US)), '>', INT9999_TO_CHARR((int)(packetLatency.avg/TICKS_PER_US-devL)), '-', INT9999_TO_CHARR((int)(packetLatency.avg/TICKS_PER_US)), '-', INT9999_TO_CHARR((int)(packetLatency.avg/TICKS_PER_US+devL)), '<', INT9999_TO_CHARR((int)(packetLatency.max/TICKS_PER_US)), 
			'D', INT9999_TO_CHARR((int)(packetDiff.min/TICKS_PER_US)), '>', INT9999_TO_CHARR((int)(packetDiff.avg/TICKS_PER_US-devD)), '-', INT9999_TO_CHARR((int)(packetDiff.avg/TICKS_PER_US)), '-', INT9999_TO_CHARR((int)(packetDiff.avg/TICKS_PER_US+devD)), '<', INT9999_TO_CHARR((int)(packetDiff.max/TICKS_PER_US)));
			stats_reset(&packetLatency);
			stats_reset(&packetDiff);
		} */
	}
}

static void sendSOFPackets(uint32_t frameID, TimePoint SOF)
{
	struct SOFPacket sofPacket = { frameID, GetTimestampUS(SOF) };

	if (lastSOFPacket)
	{
		ERR_STR("#NotSentSOF:");
		ERR_CHARR(INT999_TO_CHARR(frameID-1));
	}

	SOFD_CHARR('<', 'S', 'O', 'F', ':', INT999_TO_CHARR(frameID));

	// Allocate space in a shared USB packet
	SharedBuffer *buffer;
	uint8_t *ptr;
	uint_fast16_t allocated = allocateSharedUSBSpace(&packetHub, BLOCK_HEADER_SIZE+SOF_PACKET_SIZE, &buffer, &ptr);
	if (allocated)
	{ // Successfully allocated
		//assert(allocated == BLOCK_HEADER_SIZE+SOF_PACKET_SIZE);

		const struct BlockHeader header = { .isSignal = true, .signal = SIGNAL_SOF, .size = SOF_PACKET_SIZE };
		storeBlockHeader(header, ptr);
		storeSOFPacket(sofPacket, ptr+BLOCK_HEADER_SIZE);

		unlockPacket(&buffer->packet);
		buffer->writeTime = GetTimePoint();
		lastSOFPacket = &buffer->packet;

		LOG_EVT_SET(CONTROLLER_EVENT_USB_QUEUE_SOF, true);

		if (!buffer->packet.queued)
		{
			if (!queuePacket(&packetHub, &buffer->packet))
			{
				ERR_STR("#FailedSOFQueue");
			}
			else
			{
				if (buffer->packet.queued)
				{
					DEBUG_STR("+Q");
				}
				else
				{
					DEBUG_STR("+S");
				}
			}
		}
		else
			DEBUG_STR("+SOFAlrQueued");
	}
	else
		ERR_STR("#FailedSOFAlloc");

	{ // Send SOF Packet to cameras
		const struct PacketHeader header = { .tag = PACKET_SOF, .length = SOF_PACKET_SIZE, .frameID = frameID };
		UARTPacketRef *packet = allocateUARTPacket(header.length);
		storeSOFPacket(sofPacket, packet->data);
		finaliseUARTPacket(packet, header);
		for (uint_fast8_t i = 0; i < UART_PORT_COUNT; i ++)
		{
			if (camStates[i].comm == CommPiReady)
			{
				uartd_send(i, packet, UART_PACKET_OVERHEAD_SEND+SOF_PACKET_SIZE, false);
				portStates[i].lastTimeSync = SOF;
			}
		}
	}
}

static void cleanSendingState()
{
	// Reset packet-specific sending states - they expect a callback and would get stuck otherwise
#if defined(ENABLE_LOG) && defined(DEBUG_USE_INT)
	if (debugUSBPacket.queued || debugUSBPacket.writeSet)
		debugSending = debugTail;
	resetPacketRef(&debugUSBPacket);
#endif
#if defined(ENABLE_EVENTS) && defined(EVENT_USE_INT)
	if (eventUSBPacket.queued || eventUSBPacket.writeSet)
		eventSending = eventTail;
	resetPacketRef(&eventUSBPacket);
#endif
}


/* ------ Setup Packet Hub ------ */

static void InitPacketHub()
{
	// Setup shared packets for smaller packets and signals from controller to host
	memset(packetHub.shared, 0, sizeof(SharedBuffer) * SHARED_BUF_COUNT);
	for (int i = 0; i < SHARED_BUF_COUNT; i++)
	{
		packetHub.shared[i].packet.shared = &packetHub.shared[i];
		packetHub.shared[i].packet.buffer = packetHub.shared[i].buffer;
		packetHub.shared[i].packet.isDMASource = false;
	}

	// Setup a source structure for each UART port
	memset(packetHub.sources, 0, sizeof(Source) * UART_PORT_COUNT);
	for (int i = 0; i < UART_PORT_COUNT; i++)
	{
		packetHub.sources[i].id = i;
		for (int j = 0; j < UART_TX_BLOCKS; j++)
			packetHub.sources[i].packets[j].isDMASource = true;
	}

	// Can't setup sinks yet, follows USB endpoints
	packetHub.sinkCount = 0;

	resetPacketHub(&packetHub);
}

static void SetupPacketHubSinks()
{
	packetHub.sinkCount = 0;

	if (usbd_interface == NULL)
		return;

	packetHub.sinkCount = usbd_interface->epCount > MAX_EP_COUNT? MAX_EP_COUNT : usbd_interface->epCount;
	memset(packetHub.sinks, 0, sizeof(Sink) * packetHub.sinkCount);
	for (int i = 0; i < packetHub.sinkCount; i++)
	{
		packetHub.sinks[i].ep = USBD_EP_INT_IN_ADDR_BASE+i;
		packetHub.sinks[i].nullPacket.size = USB_PACKET_HEADER;
		packetHub.sinks[i].nullPacket.buffer = packetHub.sinks[i].nullPacketBuffer;
	}
}


/* ------ USB Behaviour ------ */

void usbd_SOF_CB(usbd_device *usbd)
{ // Direct callback from driver
	lastUSBSOF = GetTimePoint();
	// TODO: Use difference from expected to adjust upcoming dueTimes
}

void usbd_EP_TX_CB(usbd_device *usbd, uint8_t ep, bool success)
{ // Direct callback from driver
	TimePoint now = GetTimePoint();
	uint8_t epIndex = ep-1;
	USBPortState *port = &usbd_interface->ep[epIndex];
	Sink *sink = &packetHub.sinks[epIndex];

	/* {
		TimeSpan diff = GetTimeSpanUS(lastIntPacket, now);
		diff = (diff+USBD_HS_EP_INT_INTERVAL/2) % USBD_HS_EP_INT_INTERVAL - USBD_HS_EP_INT_INTERVAL/2;
		stats_update(&packetDiff, diff);
	} */

	lastUSBPacket = now;
	lastIntPacket = now;
	port->lastFrame = now;
	sink->lastSentStart = now - sink->sending->size*TICKS_PER_US*USB_BYTE_TIME_US - 10;
	sink->lastSentEnd = now;

	bool hadTX = sink->sending->size > USB_PACKET_HEADER; // != nullPacket
	if (hadTX)
	{
		LOG_EVT_STR(CONTROLLER_EVENT_DATA_OUT, true);

		/* TimeSpan latency = GetTimeSpanUS(sink->sending->latencyTime, now);
		stats_update(&packetLatency, latency); */

		LOG_EVT_SET(CONTROLLER_EVENT_USB_SENDING_PACKET, false);
	}
	else
	{
		LOG_EVT_SET(CONTROLLER_EVENT_USB_SENDING_NULL, false);
	}

	if (sink->sending == lastSOFPacket)
	{
		lastSOFPacket = NULL;
	}

#if defined(ENABLE_LOG) && defined(DEBUG_USE_INT)
	if (sink->sending == &debugUSBPacket)
	{
		if (success)
			debugTail = debugSending;
		else
			debugSending = debugTail;
	}
#endif
#if defined(ENABLE_EVENTS) && defined(EVENT_USE_INT)
	if (sink->sending == &eventUSBPacket)
	{
		if (success)
			eventTail = eventSending;
		else
			eventSending = eventTail;
	}
#endif

	TimeSpan timeToSentUS = GetTimeSpanUS(sink->sending->sendingTime, sink->lastSentStart);
	if (timeToSentUS > USBD_HS_EP_INT_INTERVAL*2)
	{
		KERR_STR("!UsbEpStall:");
		KERR_CHARR(INT99999_TO_CHARR(timeToSentUS));
	}

	if (sink->sending->locked)
		ERR_STR("#SENT_WAS_LOCKED");
	resetPacketRef(sink->sending);

	// Update sink state
	sink->sending = NULL;
	sink->counter++;

	// Select next packet from queue
	bool sending = false;
	for (int i = 0; i < packetHub.packetQueueSz; i++)
	{
		if (setSinkSending(&packetHub, sink, packetHub.packetQueue[i]))
		{ // Packet was ready to be set sending next
			//TEMP_CHARR('/', 'D', 'Q', 'D', INT9999_TO_CHARR(packetHub.packetQueue[i]->size));
			dequeuePacket(&packetHub, i);
			sending = true;
			break;
		}
	}
	if (!sending && packetHub.packetQueueSz > 0)
	{
		WARN_STR("!NoSend-QueueHas:");
		WARN_CHARR(INT9_TO_CHARR(packetHub.packetQueueSz));
	}
}

void usbd_EP_RX_CB(usbd_device *usbd, uint8_t ep, uint16_t len)
{ // Direct callback from driver
	// No EP other than EP0 is setup to receive
}

void usbd_set_interface(uint8_t interface)
{ // Callback from usb class implementation
	CMDD_STR("+SetInterface:");
	CMDD_CHARR(INT9_TO_CHARR(interface));
	// Stage 1: Open Channels
	commChannelsOpen = interface > 0;
	// Clear sinks and set them up again
	SetupPacketHubSinks();
}

void usbd_close_interface()
{ // Callback from usb class implementation
	CMDD_STR("+CloseInterface");
	commChannelsOpen = false;
	cleanSendingState();
	resetSinkStates(&packetHub);
	SetupPacketHubSinks();
}

/* Request from host expecting response on control endpoint (vendor-defined on the single interface) */
usbd_respond usbd_control_respond(usbd_device *usbd, usbd_ctlreq *req)
{  // Callback from usb class implementation
	// Check req->bRequest, req->wIndex and req->wValue for handling
	lastUSBPacket = GetTimePoint();

	if (req->bRequest == COMMAND_IN_DEBUG)
	{ // Request for debug data
#if defined(ENABLE_LOG)
		USBD_STR("+Debug");
		if (debugTail == debugSending)
		{
			debugSending = debugSending+1; // "Lock" as quickly as possible
			// No need for atomics, see util.h, can't be interrupted by code that debugs
			usbd->status.data_ptr = (void*)&debugBuffer[debugTail];
			int size = debugHead - debugTail;
			if (size < 0) size = DEBUG_BUFFER_SIZE-debugTail;
			if (size > req->wLength) size = req->wLength;
			if (size > usbd->status.data_maxsize) size = usbd->status.data_maxsize;
			usbd->status.data_count = size;
			debugSending = (debugTail+size) % DEBUG_BUFFER_SIZE;
		}
		else
		{ // A debug packet is already sending. Can not be another control transfer. Mostly here to allow future simultaneous channels.
			usbd->status.data_ptr = (void*)0;
			usbd->status.data_count = 0;
		}
#else
		usbd->status.data_ptr = (void*)0;
		usbd->status.data_count = 0;
#endif
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_IN_STATUS)
	{ // Request to report controller status
		USBD_STR("+Iterate");
		uint8_t *buf = (uint8_t*)&usbd->status.data_buf[sizeof(usbd_ctlreq)];
	
		// Controller status
		const int CONTROLLER_STATUS_SIZE = 4;
		uint8_t *controllerStatus = buf + 0;
		controllerStatus[0] = ((commChannelsOpen? 1:0) << 0)
			| ((enforceTimeSync? 1:0) << 1)
			| ((frameSignalSource == FrameSignal_External? 1:0) << 2)
			| ((frameSignalSource != FrameSignal_Generating? 1:0) << 3);
		// Controller timestamp (not really necessary)
		TimeSpan timeSinceStartupMS = GetTimeSinceMS(startup);
		controllerStatus[1] = (timeSinceStartupMS >> 0)&0xFF;
		controllerStatus[2] = (timeSinceStartupMS >> 1)&0xFF;
		controllerStatus[3] = (timeSinceStartupMS >> 2)&0xFF;

		// Reporting port states
		uint8_t *portStatus = buf + CONTROLLER_STATUS_SIZE;
		const int PORT_STATUS_SIZE = 7;
		const int PORTS_STATUS_SIZE = 1+UART_PORT_COUNT*PORT_STATUS_SIZE;
		portStatus[0] = UART_PORT_COUNT;
		for (int i = 0; i < UART_PORT_COUNT; i++)
		{
			int base = 1+i*PORT_STATUS_SIZE;
			portStatus[base+0] = camStates[i].comm;
			INFO_CHARR(UI8_TO_HEX_ARR(camStates[i].comm), ':');
			uint8_t *ID = (uint8_t*)&camStates[i].identity.id;
			portStatus[base+1] = ID[0];
			portStatus[base+2] = ID[1];
			portStatus[base+3] = ID[2];
			portStatus[base+4] = ID[3];
			portStatus[base+5] = camStates[i].cameraEnabled;
			portStatus[base+6] = camStates[i].frameSyncEnabled;
			// Because this does not work on RISC-V... target address is not aligned
			//*((int32_t*)&buf[base+1]) = camStates[i].identity.id;
		}
		usbd->status.data_ptr = buf;
		usbd->status.data_count = CONTROLLER_STATUS_SIZE+PORTS_STATUS_SIZE;
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_IN_EVENTS)
	{ // Request for event data
		USBD_STR("+Event");
#if defined(ENABLE_EVENTS)
		if (eventTail == eventSending)
		{
			eventSending = eventSending+1; // "Lock" as quickly as possible
			usbd->status.data_ptr = (void*)&eventBuffer[eventTail];
			int size = eventHead - eventTail;
			if (size < 0) size = EVENT_BUFFER_SIZE-eventTail;
			if (size > req->wLength) size = req->wLength;
			if (size > usbd->status.data_maxsize) size = usbd->status.data_maxsize;
			size -= size % EVENT_BYTE_SIZE; // Only send full events
			usbd->status.data_count = size;
			eventSending = (eventTail+size) % EVENT_BUFFER_SIZE;
		}
		else
		{ // An event packet is already sending. Can not be another control transfer. Mostly here to allow future simultaneous channels.
			usbd->status.data_ptr = (void*)0;
			usbd->status.data_count = 0;
		}
#else
		usbd->status.data_ptr = (void*)0;
		usbd->status.data_count = 0;
#endif
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_IN_PACKETS)
	{ // Request for camera packets (stored in shared buffers)
		USBD_STR("+ReqPackets");
		for (int i = 0; i < SHARED_BUF_COUNT; i++)
		{
			SharedBuffer *buf = &packetHub.shared[i];
			if (buf->packet.queued || buf->packet.writeSet || buf->packet.locked)
				continue;
			if (buf->packet.size <= USB_PACKET_HEADER)
				continue;
			USE_LOCKS();
			LOCK();
			if (buf->packet.queued || buf->packet.writeSet || buf->packet.locked)
			{
				UNLOCK();
				continue;
			}
			buf->packet.writeSet = true;
			UNLOCK();
			storeUSBPacketHeader((struct USBPacketHeader){ packetSendCounter, 0 }, buf->buffer);
			usbd->status.data_ptr = (void*)buf->buffer;
			usbd->status.data_count = buf->packet.size;
			packetUSBPacket = &buf->packet;
			return usbd_ack;
		}
		usbd->status.data_ptr = (void*)0;
		usbd->status.data_count = 0;
		return usbd_ack;
	}

	return usbd_fail;
}

/* Request from host not expecting a response on control endpoint (vendor-defined on the single interface) */
usbd_respond usbd_control_receive(usbd_device *usbd, usbd_ctlreq *req)
{ // Callback from usb class implementation
	// Handle data in req->data of length req->wLength
	lastUSBPacket = GetTimePoint();

	if (req->bRequest == COMMAND_OUT_TIME_SYNC)
	{ // Stage 2; Request to set time sync enforcement
		CMDD_STR("+TimeSync");
		CMDD_CHARR(INT9_TO_CHARR(req->wValue));
		enforceTimeSync = req->wValue != 0;
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_OUT_SYNC_RESET)
	{ // Stage 3: Request to disable all sync sources
		CMDD_STR("+SyncRst");
		SYNC_Reset();
		if (frameSignalSource == FrameSignal_Generating)
			StopTimer(TIM3);
		frameSignalSource = FrameSignal_None;
		// Reset Sync Mask implicitly
		for (int i = 0; i < UART_PORT_COUNT; i++)
			camStates[i].frameSyncEnabled = false;
		enabledSyncPinsGPIOD = 0;
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_OUT_SYNC_EXTERNAL)
	{ // Stage 3: Request to setup external sync source
		CMDD_STR("+SyncExt");
		SYNC_Reset();
		if (frameSignalSource == FrameSignal_Generating)
			StopTimer(TIM3);
		SYNC_Input_Init();
		frameSignalSource = FrameSignal_External;
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_OUT_SYNC_GENERATE)
	{ // Stage 3: Request to setup internal sync source
		CMDD_STR("+SyncGen:");
		CMDD_CHARR(INT999_TO_CHARR(req->wValue));
		framerate = req->wValue;
		frametimeUS = 1000000/framerate;
		if (frametimeUS > UINT16_MAX)
		{ // TODO: This limits fps to be over 15.26fps. Consider changing timer to be less precise (currently 1us steps)
			frametimeUS = UINT16_MAX;
		}
		StartTimer(TIM3, frametimeUS);
		SYNC_Reset();
		SYNC_Output_Init();
		curFrameID = 0; // TODO: Should we really reset frameID every time streaming starts?
		frameSignalSource = FrameSignal_Generating;
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_OUT_SYNC_MASK)
	{ // Request to set sync mask for cameras
		CMDD_STR("+SyncMsk:");
		CMDD_CHARR(UI8_TO_HEX_ARR(req->wIndex));
		enabledSyncPinsGPIOD = 0;
		for (int i = 0; i < UART_PORT_COUNT; i++)
		{
			camStates[i].frameSyncEnabled = req->wIndex >> i;
			if (camStates[i].frameSyncEnabled)
				enabledSyncPinsGPIOD |= GPIOD_SYNC_PIN[i];
		}
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_OUT_SEND_PACKET)
	{ // Request to send a packet to cameras
		CMDD_STR("+SendPkt:");
		CMDD_CHARR(INT99_TO_CHARR(req->wValue));
		UARTPacketRef *packet = allocateUARTPacket(req->wLength);
		memcpy(packet->data, req->data, req->wLength);
		finaliseUARTPacket(packet, (struct PacketHeader){ .tag = req->wValue, .length = req->wLength });
		for (int i = 0; i < UART_PORT_COUNT; i++)
		{
			if (!((req->wIndex >> i)&1) || (camStates[i].comm & CommReady) != CommReady) continue;
			uartd_send_int(i, packet, UART_PACKET_OVERHEAD_SEND+req->wLength, false);
			if (req->wValue == PACKET_CFG_MODE && req->wLength > 0)
			{ // Snoop in to check if camera/streaming is enabled
				camStates[i].cameraEnabled = req->data[0]&TRCAM_FLAG_STREAMING;
			}
		}
		return usbd_ack;
	}
	else if (req->bRequest == COMMAND_OUT_TEST)
	{ // Test signals
		CMDD_STR("+Test");
		int powerTest = -1;
		switch (req->wValue)
		{
			case 0: case 1:
				uartd_init();
				break;
			case 2:
				powerTest = 0;
				break;
			case 3:
				powerTest = 1;
				break;
			case 4:
				powerTest = 2;
				break;
			case 5:
				powerTest = 3;
				break;
			case 6:
				powerTest = 4;
				break;
			case 7:
				powerTest = 5;
				break;
			case 8:
				powerTest = 6;
				break;
			case 9:
				// System Reset
				PFIC->CFGR = NVIC_KEY3 | 0x0080; // Works
				break;
		}
		const int delayTest[] = { 1000, 2000, 5000, 7500, 10000, 50000, 100000 };
		if (powerTest >= 0)
		{
			GPIO_RESET(GPIOE, GPIO_PIN_7);
			delayUS(delayTest[powerTest]);
			GPIO_SET(GPIOE, GPIO_PIN_7);
		}
		return usbd_ack;
	}
#if defined(ENABLE_EVENTS)
	else if (req->bRequest == COMMAND_OUT_EVENTS)
	{ // Set events to log
		CMDD_STR("+EventLog");
		eventLogClass = req->wIndex;
		eventTail = eventHead = eventSending = 0;
		return usbd_ack;
	}
#endif
	return usbd_nak;
}

/* Request either was successfull (response sent and received ack) */
void usbd_control_resolution(usbd_device *usbd, usbd_ctlreq *req, bool success)
{  // Callback from usb class implementation
	// Called for ALL control transfers where:
	// a) usbd_control_** function has been called on previously and was NOT a zero-length host-to-device packet
	// b) that function returned usbd_ack
	// c) the control transfer ended somehow (returning usbd_device::state back to usbd_ctl_idle)
	// if in c), the control transfer was interrupted by a new one, success will be set to false
	// this can take a long time until the next control transfer is sent by the host

	// req is NOT the full packet, just the header

	if ((req->bmRequestType & USB_REQ_DIRECTION) == USB_REQ_DEVTOHOST)
	{
		if (req->bRequest == COMMAND_IN_DEBUG)
		{ // Request for debug data
	#if defined(ENABLE_LOG)
			if (success)
				debugTail = debugSending;
			else
				debugSending = debugTail;
	#endif
		}
		else if (req->bRequest == COMMAND_IN_STATUS)
		{ // Request to iterate connected Tracking Cameras
			// Doesn't matter
		}
		else if (req->bRequest == COMMAND_IN_EVENTS)
		{ // Request for event data
	#if defined(ENABLE_EVENTS)
			if (success)
			{
				eventTail = eventSending;
				USBD_STR("+EVT_Success");
			}
			else
			{
				eventSending = eventTail;
				USBD_STR("!EVT_Failure");
			}
	#endif
		}
		else if (req->bRequest == COMMAND_IN_PACKETS)
		{ // Request for packet data
			if (packetUSBPacket)
			{
				if (success)
				{
					resetPacketRef(packetUSBPacket);
					packetSendCounter++;
					USBD_STR("+PKT_Success");
				}
				else
				{
					packetUSBPacket->writeSet = false;
					USBD_STR("!PKT_Failure");
				}
			}
			packetUSBPacket = NULL;
		}
	}
	else
	{

	}
}


/* ------ UART Behaviour ------ */

uartd_respond uartd_handle_header(uint_fast8_t port)
{
	TimePoint now = GetTimePoint();
	PortState *state = &portStates[port];
	CameraState *cam = &camStates[port];
	Source *source = &packetHub.sources[port];

	if ((cam->comm & CommReady) != CommReady)
	{ // Identification phase
		if (state->header.tag == PACKET_IDENT)
		{ // Identification
			if (state->header.length == IDENT_PACKET_SIZE)
				return uartd_accept;
			ERR_STR("#IdentWrongSize:");
			ERR_CHARR(INT9_TO_CHARR(state->dataPos), '+', INT9999_TO_CHARR(state->header.length));
			uartd_nak_int(port);
			return uartd_reset;
		}
		else if (state->header.tag == PACKET_ACK)
		{
			if ((cam->comm&CommReady) == CommID)
			{ // Received acknowledgement of own identity
				cam->comm |= CommACK;
				COMM_STR("/Connected:");
				COMM_CHARR(INT9_TO_CHARR(port));
				return uartd_accept;
			}
			else
			{
				COMM_STR("!PrematureACK:");
				COMM_CHARR(INT9_TO_CHARR(port));
				uartd_nak_int(port);
				return uartd_reset;
			}
		}
		else if (state->header.tag == PACKET_NAK)
		{
			COMM_STR("!IdentNAKed:"); 
			COMM_CHARR(INT9_TO_CHARR(port));
			return uartd_reset;
		}
		else
		{ // Received an unexpected packet before comm handshake
			COMM_STR("!Unexpected:");
			COMM_CHARR(INT9_TO_CHARR(port), '+', UI8_TO_HEX_ARR(state->header.tag));
			// Very likely, this controller was reset/restarted
			// Reset comms and wait for camera to send an ident packet again
			uartd_nak_int(port);
			return uartd_reset;
		}
	}

	if (state->header.tag >= PACKET_HOST_COMM)
	{ // Packet designated for Host
		/* TEMP_CHARR('<', INT9_TO_CHARR(port), 'R', 'C', 'V',
			'+', INT99_TO_CHARR(state->header.tag), ':', INT9999_TO_CHARR(state->header.length),
			//'+', 'T', INT99_TO_CHARR(state->lastComm.ms), ':', INT999_TO_CHARR(state->lastComm.us)
		); */

		if (state->header.tag == PACKET_FRAME_SIGNAL)
		{
			if (state->header.frameID != (state->lastAnnounceID+1)%256)
				DEBUG_CHARR('/', INT9_TO_CHARR(port), 'S', 'F', 'S', INT99_TO_CHARR(state->lastAnnounceID), ':', INT99_TO_CHARR(state->header.frameID));
			state->lastAnnounceID = state->header.frameID;
		}
		else if (state->header.tag == PACKET_BLOB)
		{
			if (state->header.frameID != (state->lastStreamID+1)%256)
				DEBUG_CHARR('/', INT9_TO_CHARR(port), 'S', 'S', 'P', INT99_TO_CHARR(state->lastStreamID), ':', INT99_TO_CHARR(state->header.frameID));
			state->lastStreamID = state->header.frameID;
		}
		else if (state->header.tag == PACKET_ERROR)
		{ // Snoop in, and stop streaming already
			// If the error is serious, camera software will send a NAK and restart anyways, in which case this is not necessary
			cam->cameraEnabled = false;
			cam->frameSyncEnabled = false;
			enabledSyncPinsGPIOD &= ~(GPIOD_SYNC_PIN[port]);
		}

		// Set flag on next block header that a new packet starts
		initSourcePackets(source);
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_PING)
	{ // Ping response, last comm time is updated
		return uartd_ignore;
	}
	else if (state->header.tag == PACKET_NAK)
	{
		COMM_STR("!CommReset:"); 
		COMM_CHARR(INT9_TO_CHARR(port));
		return uartd_reset;
	}
	else if (state->header.tag == PACKET_ACK)
	{ // ACK for something completely different?
		COMM_STR("!StrayACK:");
		COMM_CHARR(INT9_TO_CHARR(port));
		return uartd_ignore;
	}
	else if (state->header.tag == PACKET_IDENT)
	{ // Identification - not expected unless camera software crashed and quickly restarted or similar
		COMM_STR("!StrayIdent:");
		COMM_CHARR(INT9_TO_CHARR(port));
		COMM_STR("+Reset");
		return uartd_reset;
	}
	else
	{
		return uartd_unknown;
	}
	TimeSpan dT = GetTimeSinceUS(now);
	if (dT > 100)
		TERR_STR("!UART_HH");
}

uartd_respond uartd_handle_data(uint_fast8_t port, uint8_t* ptr, uint_fast16_t size)
{
	PortState *state = &portStates[port];
	CameraState *cam = &camStates[port];
	TimePoint now = GetTimePoint();
	if (state->header.tag >= PACKET_HOST_COMM)
	{ // Packet designated for Host
		//TEMP_CHARR('+', 'D', INT999_TO_CHARR(size));
		if (state->dataPos == 0)
		{ // Beginning of buffer, make sure header is prepended
			// - might not be the case if data starts at beginning of RX buffer (header partly at end of buffer)
			// also erases the header checksum
			state->queuedPos = ptr-state->bufferPtr - PACKET_HEADER_SIZE;
			memcpy(state->bufferPtr+state->queuedPos, state->headerRaw, PACKET_HEADER_SIZE);
		}
		TimeSpan dT = GetTimeSinceUS(now);
		if (dT > 100)
			TERR_STR("!UART_HD_0");
		uint_fast16_t end = (ptr-state->bufferPtr)+size;
		uint16_t remSize = end - state->queuedPos;
		bool completed = state->dataPos+size == state->header.length;
		bool endOfBuf = end == UART_RX_BUFFER_SIZE;
		//TEMP_CHARR('!', INT99999_TO_CHARR(state->queuedPos));
		if (endOfBuf)
			TEMP_CHARR('/', 'E', 'O', 'B');
		if (!completed)
			TEMP_CHARR('/', 'I', 'N', 'C');
		if (remSize != state->dataPos+size+PACKET_HEADER_SIZE)
		{
			TEMP_CHARR('/', 'C', 'N', 'T', INT9999_TO_CHARR(remSize), ':', INT99999_TO_CHARR(state->queuedPos), '-', INT99999_TO_CHARR(end));
		}
		bool finishData = completed || endOfBuf;
		uint_fast16_t fullPacketSize = USBD_PACKET_SIZE-USB_PACKET_HEADER-BLOCK_HEADER_SIZE;
		int iterations = 0;
		while (remSize > 0 && (finishData || remSize >= fullPacketSize))
		{
			if (iterations > 0)
			{
				TEMP_CHARR('+', 'T', 'D', INT9999_TO_CHARR(remSize));
			}
			uint16_t handled = remSize;
			handleSourceData(&packetHub, &packetHub.sources[port], state->bufferPtr+state->queuedPos, &remSize);
			handled -= remSize;
			//TEMP_CHARR('+', 'B', INT999_TO_CHARR(handled), '/', INT999_TO_CHARR(remSize+handled));
			if (handled == 0)
			{ // Failed to queue data, can't handle packet
				ERR_CHARR('!', 'O', 'V', 'F');
				/* ERR_CHARR('+', packetHub.sinks[0].sending? 'S' : 'F');
				if (packetHub.sinks[0].sending)
					ERR_CHARR(':', INT9999_TO_CHARR(packetHub.sinks[0].sending->size));
				ERR_CHARR('+', 'S', 'H', 'R', 'D');
				for (int i = 0; i < SHARED_BUF_COUNT; i++)
				{
					SharedBuffer *buf = &packetHub.shared[i];
					TimeSpan latency = GetTimeSinceUS(buf->packet.latencyTime);
					ERR_CHARR('-', UI8_TO_HEX_ARR((uint8_t)&buf->packet), 'S', INT9999_TO_CHARR(buf->packet.size), 'L', INT999_TO_CHARR(latency/100), '=', buf->packet.writeSet? 'W' : 'N', buf->packet.queued? 'Q' : 'N');

				} */
				return uartd_ignore;
			}
			state->queuedPos += handled;
			iterations++;
			//TEMP_CHARR('+', 'R', INT99999_TO_CHARR(state->queuedPos));
		}
		//TimeSpan dtUS = GetTimeSinceUS(state->lastComm);
		//TEMP_CHARR('+', 'T', INT99_TO_CHARR(dtUS), '>');

		if (endOfBuf)
		{
			state->queuedPos = 0;
			//TEMP_CHARR('+', 'R', 'S', 'T', 'Q');
		}

		dT = GetTimeSinceUS(now);
		if (dT > 100)
			TERR_STR("!UART_HD_1");
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_IDENT)
	{ // Identification
		memcpy(rcvIdentPacket+state->dataPos, ptr, size);
		if (state->dataPos+size == IDENT_PACKET_SIZE)
		{ // Received full identification, check
			struct IdentPacket ident = parseIdentPacket(rcvIdentPacket);
			if ((ident.device != DEVICE_TRCAM && ident.device != DEVICE_TRCAM_MCU) || ident.type != INTERFACE_UART)
			{ // Ident packet was plain wrong
				WARN_STR("!IdentFail:");
				WARN_CHARR(INT9_TO_CHARR(port));
				WARN_CHARR('+', UI8_TO_HEX_ARR(ident.device), '+', UI8_TO_HEX_ARR(ident.type));
				uartd_nak_int(port);
				return uartd_reset;
			}
			if (ident.version.major != ownIdent.version.major || ident.version.minor != ownIdent.version.minor)
			{ // TODO: Deal with versions, controller should be able to update older versions anyway
				WARN_STR("!VersionMismatch:");
				WARN_CHARR(INT9_TO_CHARR(port));
				WARN_STR("+Cont:v");
				WARN_CHARR(INT99_TO_CHARR(ownIdent.version.major), '.', INT99_TO_CHARR(ownIdent.version.minor));
				WARN_STR("+Cam:v");
				WARN_CHARR(INT99_TO_CHARR(ident.version.major), '.', INT99_TO_CHARR(ident.version.minor));
				uartd_nak_int(port);
				return uartd_reset;
			}
			// Idenfication verified
			cam->comm = CommID;
			cam->identity = ident;
			if (ident.device == DEVICE_TRCAM)
				cam->comm |= CommPi;
			else if (ident.device == DEVICE_TRCAM_MCU)
				cam->comm |= CommMCU;
			COMM_STR("/Identified:");
			COMM_CHARR(INT9_TO_CHARR(port));
			COMM_CHARR('+', UI8_TO_HEX_ARR(ident.device));
			// Send own identification in response
			uartd_send_int(port, ownIdentPacket, sizeof(ownIdentPacket), true);
		}

		TimeSpan dT = GetTimeSinceUS(now);
		if (dT > 100)
			TERR_STR("!UART_HD_I");
		return uartd_accept;
	}
	else
	{ // Shouldn't happen
		return uartd_unknown;
	}
}

void uartd_handle_packet(uint_fast8_t port, uint_fast16_t endPos)
{

}

void uartd_handle_reset(uint_fast8_t port)
{
	memset(&camStates[port], 0, sizeof(CameraState));
}

/* ------ INIT Behaviour ------ */

void TIM3_IRQHandler(void) __IRQ;
void TIM3_IRQHandler()
{
	LOG_EVT_INT(CONTROLLER_INTERRUPT_SYNC_GEN, true);

	// Reset IRQ flag
	TIM_SR(TIM3) = 0;
	if (frameSignalSource != FrameSignal_Generating)
	{
		LOG_EVT_INT(CONTROLLER_INTERRUPT_SYNC_GEN, false);
		return;
	}

	// Generate External Trigger Signal on output and camera pins
	GPIO_SET(GPIOB, GPIOB_SYNC_IO_PINS);
	GPIO_SET(GPIOD, enabledSyncPinsGPIOD);

	// Update Sync SOF
	curSOF = GetTimePoint();
	curFrameID++;

	// Done in main thread, not critical enough to do in interrupt
	//sendSOFPackets(curFrameID, curSOF);
	//ERR_CHARR(':', INT999_TO_CHARR(curFrameID%0xFF));

	// Wait until pulse end
	TimePoint tgt = curSOF + SYNC_PULSE_WIDTH_US * TICKS_PER_US;
	while (GetTimePoint() < tgt);

	// Set camera & output sync pulse low
	GPIO_RESET(GPIOB, GPIOB_SYNC_IO_PINS);
	GPIO_RESET(GPIOD, enabledSyncPinsGPIOD);
	LOG_EVT_INT(CONTROLLER_INTERRUPT_SYNC_GEN, false);
	LOG_EVT_STR(CONTROLLER_EVENT_SYNC, true);
}

void EXTI3_IRQHandler(void) __IRQ;
void EXTI3_IRQHandler()
{
	LOG_EVT_INT(CONTROLLER_INTERRUPT_SYNC_INPUT, true);
	if (EXTI->INTFR & GPIOE_SYNC_EXTI_LINES)
	{ // Interrupt pending

		// Mirror External Trigger Signal
		GPIO_SET(GPIOD, enabledSyncPinsGPIOD);

		// Update Sync SOF
		curSOF = GetTimePoint();
		curFrameID++;

		// Done in main thread, not critical enough to do in interrupt
		//sendSOFPackets(curFrameID, curSOF);

		// Wait until pulse end
		TimePoint tgt = curSOF + SYNC_PULSE_WIDTH_US * TICKS_PER_US;
		while (GetTimePoint() < tgt);

		// Reset IRQ flag
		EXTI->INTFR = GPIOE_SYNC_EXTI_LINES;

		// Set camera sync pulse low
		GPIO_RESET(GPIOD, enabledSyncPinsGPIOD);

		LOG_EVT_STR(CONTROLLER_EVENT_SYNC, true);
	}
	LOG_EVT_INT(CONTROLLER_INTERRUPT_SYNC_INPUT, false);
}

/**
 * LED Update state interrupt. Could either be:
 * - standard blink update
 */
void TIM6_IRQHandler(void) __IRQ;
void TIM6_IRQHandler()
{
	if (commChannelsOpen)
		LOG_EVT_INT(CONTROLLER_INTERRUPT_LED_UPDATE, true);
	StopTimer(TIM6);
	TIM_SR(TIM6) = 0;
	static uint_fast8_t updateState = 0;
	static uint32_t blinkState = 0;

	if (updateState == 0)
	{ // Update LED state and write
		TimePoint now = GetTimePoint();
		uint16_t LEDState = 0;
		for (int i = 0; i < UART_PORT_COUNT; i++)
		{
			uint8_t camLEDState = 0b00;
			if ((camStates[i].comm & CommReady) == CommReady)
			{
				camLEDState |= 0b01;
			}
			if (camStates[i].comm & CommReady)
			{
				TimeSpan timeSinceLastComm = now - portStates[i].lastComm;
				TimeSpan timeoutBlink = (enforceTimeSync? 50 : 800)*TICKS_PER_MS;
				if (timeSinceLastComm > timeoutBlink)
				{
					camLEDState |= blinkState&0b10;
				}
				else
					camLEDState |= 0b10;
			}
			LEDState |= camLEDState << (i<<1);
		}
		SPI2->DATAR = LEDState;
		blinkState++;
		updateState = 1;
		StartTimer(TIM6, 2); // 20us, Write time until Latch
	}
	else if (updateState == 1)
	{ // Data is sent. Enable Latch
		GPIO_SET(GPIOB, GPIO_PIN_12);
		updateState = 2;
		StartTimer(TIM6, 2); // 20us, Latch pulse width
	}
	else if (updateState == 2)
	{ // Disable Latch
		GPIO_RESET(GPIOB, GPIO_PIN_12);
		updateState = 0;
		StartTimer(TIM6, 10000-4); // 100ms, Update interval
	}
	if (commChannelsOpen)
		LOG_EVT_INT(CONTROLLER_INTERRUPT_LED_UPDATE, false);
}