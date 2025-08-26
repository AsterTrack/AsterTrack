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

#ifndef __PACKETHUB_H
#define __PACKETHUB_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <string.h>

#include "util.h"
#include "uartd.h"			// Dis/EnableUARTInterrupts, UART_BAUD_RATE, UART_TX_BLOCKS, USBD_PACKET_SIZE
#include "usbd.h" 			// *USBZone, hUSB

#include "usbd_conf.h"
#include "usb_driver.h"		// Driver specific functions


/* Defines */

#define PACKET_BLOCK_SZ				USBD_PACKET_SIZE
#define MAX_QUEUED_PACKETS			32

#define SHARED_BUF_COUNT			20

#define USB_BYTE_TIME_US 			8/480 // 8Bit / 480Bit/us

/* Utilities */

// Mutex to access packetHub, UART and USB hardware from main loop
// Does NOT support nesting
static inline void EnterPacketHubZone ()
{
	ATOMIC_SINGLE( // Does NOT support nesting
		DisableUARTInterrupts();
		DisableUSBInterrupts();
	);
}
// Does NOT support nesting
static inline void LeavePacketHubZone ()
{
	ATOMIC_SINGLE(
		EnableUARTInterrupts();
		EnableUSBInterrupts();
	);
}


/* Structures */

typedef struct SharedBuffer SharedBuffer;
typedef struct Source Source;

typedef struct
{
	// To measure performance
	TimePoint latencyTime;
	TimePoint sendingTime;

	// State
	volatile bool writeSet;
	volatile bool queued;
	volatile bool locked;

	// To setup DMA from buffer
	uint8_t *buffer;
	volatile uint_fast16_t size;

	// Source: Shared Buffer
	SharedBuffer *shared;

	// Source: DMA from UART Buffer
	bool isDMASource;
	volatile struct BlockHeader writeBlockHeader; // used only if from DMA source

	// int staticPriority = (streaming? 10 : 1) * (completed? 5 : (size/128));
	// int dynamicPriority = (now-latencyTime) / 4096; // 1 every 227us assuming TICKS_PER_US = 18
	// int priority = staticPriority * dynamicPriority;
} PacketRef;

struct SharedBuffer
{
	TimePoint writeTime;

	int index;
	PacketRef packet;
	__attribute__((aligned(4))) uint8_t buffer[PACKET_BLOCK_SZ];
};

struct Source
{
	uint8_t id;					// ID of source itself
	volatile uint8_t blockID;	// Increasing ID to sort blocks from one source
	bool isNewPacket;

	PacketRef packets[UART_TX_BLOCKS]; // Space for full blocks of UART data
};

typedef struct
{
	int ep;

	volatile uint8_t counter; 	// Counter to help host determine continuity per sink
	TimePoint lastSentStart;	// Timepoint at which it last started sending, updated after it finished sending
	TimePoint lastSentEnd;		// Timepoint at which it last finished sending
	TimePoint dueTime;			// Timepoint at which it is expected to start sending next

	uint8_t nullPacketBuffer[USB_PACKET_HEADER];
	PacketRef nullPacket;
	volatile PacketRef *sending;
	//PacketRef *queued; // Unused until double-buffering is used
} Sink;

typedef struct
{
	__attribute__((aligned(4))) SharedBuffer shared[SHARED_BUF_COUNT];

	__attribute__((aligned(4))) Source sources[UART_PORT_COUNT];

	__attribute__((aligned(4))) Sink sinks[MAX_EP_COUNT];
	uint16_t sinkCount;

	PacketRef *packetQueue[MAX_QUEUED_PACKETS];
	uint8_t packetQueueSz;

	int sharedStallTimeout;
} Hub;


/* Functions */

/**
 * Lock sink to prevent it from sending
 * WARNING: Does nothing if it's already being send, would result in invalid state
 */

static inline bool lockPacket_int(Hub *hub, PacketRef *packet)
{
	bool doLock = !packet->writeSet && !packet->locked && (!packet->queued || hub->packetQueue[0] != packet);
	if (doLock)
		packet->locked = 1;
	return doLock;
}

/**
 * Lock sink to prevent it from sending
 * WARNING: Does nothing if it's already being send, would result in invalid state
 */
static inline bool lockPacket(Hub *hub, PacketRef *packet)
{
	USE_LOCKS();
	LOCK();
	bool doLock = lockPacket_int(hub, packet);
	UNLOCK();
	return doLock;
}

/**
 * Unlock sink to allow it to send
 */
static inline void unlockPacket(PacketRef *packet)
{
	packet->locked = 0;
}

/**
 * Dequeue packet from packet queue - only call from interrupts that can't be preempted by other PacketHub interrupts
 */
static inline void dequeuePacket(Hub *hub, int index)
{
	if (index >= hub->packetQueueSz)
	{
		ERR_STR("#DequeueOOR");
		return;
	}

	if (hub->packetQueue[index]->shared)
	{
		USBP_STR("+D");
		USBP_CHARR(INT9_TO_CHARR(hub->packetQueue[index]->shared->index));
	}

	USE_LOCKS();
	LOCK();
	hub->packetQueue[index]->queued = false;
	hub->packetQueueSz--;
	for (; index < hub->packetQueueSz; index++)
		hub->packetQueue[index] = hub->packetQueue[index+1];
	UNLOCK();
}

static uint_fast16_t allocateSharedUSBSpace(Hub *hub, uint_fast16_t size, SharedBuffer **buffer, uint8_t **data)
{
	for (int i = 0; i < SHARED_BUF_COUNT; i++)
	{
		// Calculate timing of current sink
		SharedBuffer *buf = &hub->shared[i];
		// TODO: revert setSinkSending if "expected" time is still away
		if (!lockPacket(hub, &buf->packet))
			continue;

		// Make sure we got space to write at least the minimum
		uint_fast16_t space = PACKET_BLOCK_SZ - buf->packet.size;
		if (space < size)
		{
			DEBUG_CHARR('+', '0'+i, 'F', INT999_TO_CHARR(space));
			// TODO: Mark as completed if that expedites sending?
			unlockPacket(&buf->packet);
			continue; // Buffer is full
		}

		// Allocate space in shared buffer
		*buffer = buf;
		*data = buf->buffer + buf->packet.size;
		buf->packet.size += size;
		return size;
	}

	if (hub->sharedStallTimeout == 0)
	{ // Debug why we couldn't allocate shared buffer, this should not happen during normal use
		ERR_STR("\n#SharedFull(");
		ERR_CHARR(INT999_TO_CHARR(size), ')');
		for (int i = 0; i < SHARED_BUF_COUNT; i++)
		{
			SharedBuffer *buf = &hub->shared[i];
			ERR_CHARR('+', INT9_TO_CHARR(i));
			if (buf->packet.queued)
			{
				ERR_STR(":Queued");
				bool inQueue = false;
				for (int i = 0; i < hub->packetQueueSz; i++)
				{
					PacketRef *next = hub->packetQueue[i];
					if (next == &buf->packet)
						inQueue = true;
				}
				if (!inQueue)
					ERR_STR("!Missing");
			}
			if (!lockPacket(hub, &buf->packet))
			{
				if (buf->packet.writeSet)
				{
					ERR_STR(":Writing");
				}
				else if (buf->packet.locked)
				{
					ERR_STR(":Locked");
				}
				else
				{
					ERR_STR(":Next");
				}
				continue;
			}
			ERR_CHARR('+', INT9_TO_CHARR(i), ':', INT999_TO_CHARR(buf->packet.size));
			ERR_STR("=Full");
			if (!buf->packet.queued)
				ERR_STR("!NotQueued");
			unlockPacket(&buf->packet);
		}
	}
	else ERR_STR("+Full");
	hub->sharedStallTimeout = 20;

	// Could not allocate shared buffer space
	*buffer = NULL;
	*data = NULL;
	return 0;
}

static inline bool canSinkSend(Sink *sink)
{
	PacketRef *sending = (PacketRef *)sink->sending; // Discard volatile, only read once
	return !sending;// || sending == &sink->nullPacket;
	// Can't overwrite anything, may already be sending rn
	// Then next callback would assume the new packet has already been sent
}

static inline bool setSinkSending(Hub *hub, Sink *sink, PacketRef *packet)
{
	if (!canSinkSend(sink))
	{ // Attempted
		return false;
	}
	if (packet == NULL)
	{
		packet = &sink->nullPacket;
	}
	else
	{ // Validate packet
		if (packet->locked)
		{ // May theoretically happen if shared packet is re-locked to put additional data in despite being queued (but not in first position)
			ERR_STR("!SENDING_PACKET_LOCKED");
			return false;
		}
		/* if (packet->queued)
		{ // Should not happen
			ERR_STR("!SENDING_STILL_QUEUED");
			return false;
		} */
		if (packet->writeSet)
		{ // Should not happen
			ERR_STR("!SENDING_ALREADY_SET");
			return false;
		}
	}
	// Prepend packet header with recent information
	// This allows host to exactly time the last sent packet
	int counter = sink->counter;
	storeUSBPacketHeader((struct USBPacketHeader){ counter, GetTimestampUS(sink->lastSentEnd) }, packet->buffer);
	// If deferred due to overwriting concerns, write block header now
	if (packet->isDMASource)
		storeBlockHeader(packet->writeBlockHeader, packet->buffer+USB_PACKET_HEADER);

	// Make sure again, in barrier, that we can send
	if (packet == &sink->nullPacket)
	{ // Is null packet being set
		LOG_EVT_SET(CONTROLLER_EVENT_USB_SENDING_NULL, true);
	}
	else
	{ // NOrmal packet is being set
		LOG_EVT_SET(CONTROLLER_EVENT_USB_SENDING_PACKET, true);
	}
	USE_LOCKS();
	LOCK();
	if (packet->locked)
	{ // Race Condition
		UNLOCK();
		ERR_STR("!SEND_LOCK");
		return false;
	}
	if (!canSinkSend(sink))
	{ // Race Condition
		UNLOCK();
		WARN_STR("!SEND_OW");
		return false;
	}
	// Mark packet buffer to be sent
	usbd_ep_set_dma(&hUSB, sink->ep, packet->buffer, packet->size);
	packet->writeSet = true;
	sink->sending = packet;
	packet->sendingTime = GetTimePoint();
	UNLOCK();

	USBP_STR("+S");
	if (packet->shared)
	{
		USBP_CHARR(INT9_TO_CHARR(packet->shared->index));
	}

	//ERR_CHARR('>', INT999_TO_CHARR(packet->buffer[0]), ':', INT999_TO_CHARR(packet->size));
	if (sink->counter != counter)
	{
		WARN_STR("!CounterAdv");
	}
	//sink->dueTime = lastSOF+XXX
	return true;
}

static bool queuePacket(Hub *hub, PacketRef *packet)
{
	if (packet->queued)
	{ // Should not happen (race condition possible)
		WARN_STR("!QUEUE_ALREADY_QUEUED");
		return true;
	}
	if (packet->writeSet)
	{ // Should not happen
		WARN_STR("!QUEUE_ALREADY_WRITING");
		return true;
	}

	packet->latencyTime = GetTimePoint();

	// Set sending immediately if a sink is free
	for (int i = 0; i < hub->sinkCount; i++)
	{
		if (setSinkSending(hub, &hub->sinks[i], packet))
			return true;
	}

	if (packet->isDMASource)
	{
		TEMP_CHARR('+', 'Q', 'U', 'D', UI32_TO_HEX_ARR((uint32_t)packet), ':', INT9999_TO_CHARR(packet->size));
	}

	// Enqueue packet
	USE_LOCKS();
	LOCK();
	if (packet->queued)
	{ // Race Condition
		UNLOCK();
		ERR_STR("#QUEUE_ALREADY_QUEUED");
		return true;
	}
	if (hub->packetQueueSz >= MAX_QUEUED_PACKETS)
	{ // Cannot queue further packets
		UNLOCK();
		ERR_STR("#QUEUE_FULL");
		return false;
	}
	packet->queued = true;
	hub->packetQueue[hub->packetQueueSz] = packet;
	hub->packetQueueSz++;
	UNLOCK();

	if (packet->shared)
	{
		USBP_STR("+Q");
		USBP_CHARR(INT9_TO_CHARR(packet->shared->index));
	}

	return true;
}

static PacketRef* handleSourceData(Hub *hub, Source *source, uint8_t *data, uint16_t *size)
{
	LOG_EVT_STR(CONTROLLER_EVENT_DATA_IN, true);
	// Write block header
	struct BlockHeader blockHeader;
	blockHeader.isSignal = false;
	blockHeader.blockID = source->blockID;
	blockHeader.isFirstPacketBlock = source->isNewPacket;
	blockHeader.portNr = source->id;

	uint_fast16_t packetSz = *size+BLOCK_HEADER_SIZE;
	const int MaxSharedSize = (USBD_PACKET_SIZE-USB_PACKET_HEADER)/4;
	if (packetSz <= MaxSharedSize || hub->sinkCount == 0)
	{ // Allocate space in shared buffer among other smaller data
		// If no sink exists, we are not streaming, and we have to put data into shared packets for sending over control
		SharedBuffer *buf;
		uint8_t *ptr;
		uint_fast16_t allocated = allocateSharedUSBSpace(hub, packetSz, &buf, &ptr);

		if (allocated)
		{
			//assert(allocated == packetSz);
			blockHeader.size = *size;
			blockHeader.skip = 0;
			storeBlockHeader(blockHeader, ptr);
			//if (dataSz < 32) // For now, always memcpy
			{ // memcpy
				memcpy(ptr+BLOCK_HEADER_SIZE, data, *size);
			}
			//else
			{ // Inititate DMA to shared buffer
			}
			unlockPacket(&buf->packet);
			buf->writeTime = GetTimePoint();

			USBP_STR("+W");
			USBP_CHARR(INT9_TO_CHARR(buf->index));
			if (!buf->packet.queued)
			{ // Queue shared buffer packet
				if (!queuePacket(hub, &buf->packet))
					ERR_STR("#FailedSourceSharedQueue");
			}

			// Update source state after block
			source->isNewPacket = false; // Clear flag
			source->blockID++;
			if (source->blockID == BLOCK_ID_SIGNAL) source->blockID++;

			*size = 0;
			return &buf->packet;
		}
		else
		{
			if (buf)
				unlockPacket(&buf->packet);
			// Continue queueing as direct DMA
		}
	}

	// Queue packet to DMA from UART buffer

	PacketRef *ref = NULL;
	/// Not needed since this is the only place that uses source->packets
	///USE_LOCKS();
	///LOCK();
	for (int i = 0; i < UART_TX_BLOCKS; i++)
	{
		///if (lockPacket_int(hub, &source->packets[i]))
		if (!source->packets[i].queued && !source->packets[i].writeSet)
		{
			ref = &source->packets[i];
			break;
		}
	}
	///UNLOCK();
	if (ref == NULL)
	{
		ERR_STR("!HANDLE_PACKETS_USED");
		return NULL;
	}

	// Decide on data size to handle
	uint_fast16_t prependSize = BLOCK_HEADER_SIZE+USB_PACKET_HEADER;
	uint8_t alignment = (uint32_t)(data-prependSize) & USB_PACKET_ALIGNMENT;
	prependSize += alignment;

	packetSz = *size+prependSize;
	if (packetSz > USBD_PACKET_SIZE)
		packetSz = USBD_PACKET_SIZE;
	uint_fast16_t dataSz = packetSz-prependSize;

	uint8_t *ptr = data-prependSize;
	blockHeader.size = dataSz;
	blockHeader.skip = alignment;

	// DO NOT write header now, previous direct-DMA packet might still be waiting to be sent
	// Instead, write block header (raw even) in ref, and mark to be written to beginning right before it's sent
	// With multiple sinks, might have to be delayed until last got sent off
	//storeBlockHeader(blockHeader, ptr+USB_PACKET_HEADER);
	ref->writeBlockHeader = blockHeader;

	ref->buffer = ptr;
	ref->size = packetSz;

	///unlockPacket(ref);

	TEMP_CHARR('/', 'D', 'M', INT9999_TO_CHARR(packetSz), ':', INT9999_TO_CHARR(dataSz));
	TEMP_CHARR('+', UI32_TO_HEX_ARR((uint32_t)data), ':', UI32_TO_HEX_ARR((uint32_t)(data+*size)));

	USBP_STR("+P");
	if (!queuePacket(hub, ref))
		ERR_STR("#FailedSourceQueue");

	// Update source state after block
	source->isNewPacket = false; // Clear flag
	source->blockID++;
	if (source->blockID == BLOCK_ID_SIGNAL) source->blockID++;

	*size -= dataSz;
	return ref;
}

static inline void initSourcePackets(Source *source)
{
	source->isNewPacket = true;
}

static inline void resetPacketRef(volatile PacketRef *packet)
{ // Clear sending state only, not fields related to origin
	packet->queued = false;
	packet->writeSet = false;
	packet->locked = false;
	packet->size = USB_PACKET_HEADER;
}

static inline void resetSourceState(Source *source)
{
	source->blockID = 0;
	source->isNewPacket = false;
	for (int i = 0; i < UART_TX_BLOCKS; i++)
		resetPacketRef(&source->packets[i]);
}

static inline void unstallSink(Sink *sink)
{
	// Clear send buffers and sending state
	usbd_ep_reset_dma(&hUSB, sink->ep);
	if (sink->sending)
		sink->sending->writeSet = false;
	sink->sending = NULL;
}

static inline void resetSinkState(Sink *sink)
{
	// Clear send buffers and sending state
	usbd_ep_reset_dma(&hUSB, sink->ep);
	if (sink->sending)
		sink->sending->writeSet = false;
	sink->sending = NULL;
	sink->counter = 0;
	resetPacketRef(&sink->nullPacket);
}

static inline void resetSinkStates(Hub *hub)
{
	for (int i = 0; i < hub->sinkCount; i++)
		resetSinkState(&hub->sinks[i]);
}

static inline void resetPacketHub(Hub *hub)
{
	// Reset sinks and sending state
	resetSinkStates(hub);
	// Reset sources and queued packets
	for (int i = 0; i < UART_PORT_COUNT; i++)
		resetSourceState(&hub->sources[i]);
	for (int i = 0; i < SHARED_BUF_COUNT; i++)
	{
		resetPacketRef(&hub->shared[i].packet);
		hub->shared[i].index = i;
	}
	hub->packetQueueSz = 0;
	hub->sharedStallTimeout = 0;
}


#ifdef __cplusplus
}
#endif

#endif /* __PACKETHUB_H */
