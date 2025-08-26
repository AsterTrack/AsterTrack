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

#if defined(STM32G0)
#include "stm32g0xx_ll_dma.h"
#elif defined(CH32V3)
#include "ch32v30x_dma.h"
#endif
#include "compat.h"

#include "util.h"
#include "uartd.h"
#include "uart_driver.h"


/* Variables */

// UART State
PortState portStates[UART_PORT_COUNT];

// Predefined message
uint8_t msg_ack[UART_PACKET_OVERHEAD_SEND], msg_nak[UART_PACKET_OVERHEAD_SEND], msg_ping[UART_PACKET_OVERHEAD_SEND];

// Temporary UART Buffers
static uint8_t UARTPacketBuffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t UARTPacketBufferPos = 0;

// Hardware mapping
struct UART_IO_State UART_IO[UART_PORT_COUNT] = {};

#pragma GCC push_options
#pragma GCC optimize ("unroll-loops")
// Mutex to access portStates and UART hardware from non-UART callbacks
inline void DisableUARTInterrupts()
{
	for (int i = 0; i < UART_PORT_COUNT; i++)
	{
		NVIC_DisableIRQ(UART[i].uartIRQ_RX);
		NVIC_DisableIRQ(UART[i].dmaIRQ_RX);
	}
}
inline void EnableUARTInterrupts()
{
	for (int i = 0; i < UART_PORT_COUNT; i++)
	{
		NVIC_EnableIRQ(UART[i].uartIRQ_RX);
		NVIC_EnableIRQ(UART[i].dmaIRQ_RX);
	}
}
inline void EnterUARTPortZone(uint8_t port)
{
	// Does NOT support nesting
	ATOMIC_SINGLE(
		NVIC_DisableIRQ(UART[port].uartIRQ_RX);
		NVIC_DisableIRQ(UART[port].dmaIRQ_RX);
	);
}
inline void LeaveUARTPortZone(uint8_t port)
{
	// Does NOT support nesting
	ATOMIC_SINGLE(
		NVIC_EnableIRQ(UART[port].uartIRQ_RX);
		NVIC_EnableIRQ(UART[port].dmaIRQ_RX);
	);
}
#pragma GCC pop_options


/* Application Functions */

uartd_respond uartd_handle_header(uint_fast8_t port);
uartd_respond uartd_handle_data(uint_fast8_t port, uint8_t* ptr, uint_fast16_t size);
uartd_respond uartd_handle_packet(uint_fast8_t port, uint_fast16_t endPos);
void uartd_handle_reset(uint_fast8_t port);


/* Functions */

/** UART Initialization Function */
void uartd_init()
{
	// Init predefined messages
	finaliseDirectUARTPacket(msg_ping, (struct PacketHeader){ .tag = PACKET_PING, .length = 0 });
	finaliseDirectUARTPacket(msg_nak, (struct PacketHeader){ .tag = PACKET_NAK, .length = 0 });
	finaliseDirectUARTPacket(msg_ack, (struct PacketHeader){ .tag = PACKET_ACK, .length = 0 });

	// Init port states
	memset(portStates, 0, sizeof(portStates));
	for (int i = 0; i < UART_PORT_COUNT; i++)
	{
		UART_IO[i].rx_buffer = (uint8_t*)((uint32_t)UART_IO[i].rx_alloc + UART_HEADROOM);
		portStates[i].bufferPtr = UART_IO[i].rx_buffer;
		uartd_handle_reset(i);
	}

	uart_driver_init();
}

/** Send data over UART port */

/**
 * Use from within a UART interrupt or an interrupt that can't be preempted by a UART interrupt
 */
void uartd_send_int(uint_fast8_t port, const void* data, uint_fast16_t len, bool isOwner)
{
	if (!UART_IO[port].uart_tx)
	{ // Send off immediately
		UART_IO[port].uart_tx = true;
		uart_send_dma(port, data, len);
		return;
	}
	// Still transferring last presumably, queue packet for sending
	int p = UART_IO[port].tx_queue_pos;
	for (int i = 0; i < SZ_TX_QUEUE; i++)
	{
		if (!UART_IO[port].tx_queue[p].valid)
		{
			UART_IO[port].tx_queue[p].addr = (uint32_t)data;
			UART_IO[port].tx_queue[p].len = len;
			UART_IO[port].tx_queue[p].valid = true;
			UART_STR("+TXQueue");
			return;
		}
		p = (p + 1) % SZ_TX_QUEUE;
	}
	// Could not queue, loosing packet
	ERR_STR("#UartTXStall");
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_TX)->CONTROL &= ~DMA_CONTROL_ENABLE;
	UART_IO[port].uart_tx = false; // TODO: Leaving without sending anything, that ok?
}
/**
 * Do NOT call from within a UART interrupt
 */
void uartd_send(uint_fast8_t port, const void* data, uint_fast16_t len, bool isOwner)
{
	EnterUARTPortZone(port);
	uartd_send_int(port, data, len, isOwner);
	LeaveUARTPortZone(port);
}

inline void uartd_reset_port_int(uint_fast8_t port)
{
	// Stop TX & RX DMA
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_RX)->CONTROL &= ~DMA_CONTROL_ENABLE;
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_TX)->CONTROL &= ~DMA_CONTROL_ENABLE;
	while (DMA_CH(UART[port].DMA, UART[port].DMA_CH_RX)->CONTROL & DMA_CONTROL_ENABLE);
	// Reset port state
	memset(&portStates[port], 0, sizeof(PortState));
	portStates[port].resetTimer = GetTimePoint();
	portStates[port].bufferPtr = UART_IO[port].rx_buffer;
	// Reset UART_IO
	for (int i = 0; i < SZ_TX_QUEUE; i++)
		UART_IO[port].tx_queue[i].valid = false;
	UART_IO[port].tx_queue_pos = 0;
	UART_IO[port].uart_tx = false;
	// Reset application state
	uartd_handle_reset(port);
	// Restart RX DMA
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_RX)->COUNTER = UART_RX_BUFFER_SIZE;
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_RX)->CONTROL |= DMA_CONTROL_ENABLE;
}
void uartd_reset_port(uint_fast8_t port)
{
	EnterUARTPortZone(port);
	uartd_reset_port_int(port);
	LeaveUARTPortZone(port);
}

static inline void skipToEnd(PortState *state)
{
	state->searchingEnd = true;
	state->inHeader = false;
	state->inData = false;
	state->numTrailing = 0;
	state->numLeading = 0;
}

/** Process received data over UART */
extern volatile TimePoint lastUARTActivity;
static uartd_respond uartd_process_data(uint_fast8_t port, uint_fast16_t begin, uint_fast16_t end)
{
	TimePoint start = GetTimePoint();
	// TODO: Log fracturing of packets when spearate comm_write are used instead of a block-write. Not optimal.
	UARTTR_CHARR('+', 'B', INT999_TO_CHARR(begin), '>', INT999_TO_CHARR(end));

	lastUARTActivity = GetTimePoint();
	PortState *state = &portStates[port];
	static int noiseData = 0;
	bool received = false;
	uartd_respond resp = uartd_ignore;
	int iterations = 0;
	uint_fast16_t pos = begin;
	while (pos < end)
	{
		if (state->inData)
		{ // Handle data in blocks
			UARTTR_STR("+D");
			state->lastComm = start;
			TimePoint now = GetTimePoint();
			uint16_t missingSize = state->header.length + PACKET_CHECKSUM_SIZE - state->dataPos;
			uint16_t blockSize = end - pos;
			bool complete = blockSize >= missingSize;
			if (complete) blockSize = missingSize;
			if (!state->ignoreData)
			{
				resp = uartd_handle_data(port, state->bufferPtr+pos, blockSize);
				if (resp == uartd_reset || resp == uartd_reset_nak)
				{
					WARN_STR("!UartDataBad");
					break;
				}
				state->ignoreData = resp == uartd_ignore;
			}
			pos += blockSize;
			state->dataPos += blockSize;
			if (complete && !state->ignoreData)
			{
				resp = uartd_handle_packet(port, pos);
				if (resp == uartd_reset || resp == uartd_reset_nak)
				{
					WARN_STR("!UartPacketBad");
					break;
				}
			}
			if (complete)
				skipToEnd(state);
			TimeSpan dT = GetTimeSinceUS(now);
			if (dT > 100)
			{
				TERR_STR("!UART_PD_D");
			}
		}
		else if (state->inHeader)
		{ // Continue to read header
			UARTTR_STR("P");
			state->lastComm = start;
			TimePoint now = GetTimePoint();
			while (state->headerPos < PACKET_HEADER_SIZE+HEADER_CHECKSUM_SIZE && pos < end)
				state->headerRaw[state->headerPos++] = state->bufferPtr[pos++];
			if (state->headerPos == PACKET_HEADER_SIZE+HEADER_CHECKSUM_SIZE)
			{ // Found header of data, handle data next
				UARTTR_CHARR('+', 'H', ':', UI32_TO_HEX_ARR(*(uint32_t*)state->headerRaw));
				// TODO: Handle parsing failure due to error more gracefully (e.g. ACK/NAK packets)
				if (!verifyHeaderChecksum(state->headerRaw))
				{ // Don't trust packet header, skip to end
					WARN_STR("!HeaderChecksumWrong");
					skipToEnd(state);
					continue;
				}
				state->inHeader = false;
				state->header = parsePacketHeader(state->headerRaw);
				UART_CHARR('\n', '<', INT9_TO_CHARR(port), 'R', 'C', 'V',
					'+', INT99_TO_CHARR(state->header.tag), ':', INT9999_TO_CHARR(state->header.length),
					//'+', 'T', INT99_TO_CHARR(state->lastComm.ms), ':', INT999_TO_CHARR(state->lastComm.us)
				);
				if (state->header.length > 100000)
				{ // Don't trust packet header, skip to end
					WARN_STR("!UartHeaderOverLength:");
					WARN_CHARR(INT99999999_TO_CHARR(state->header.length));
					skipToEnd(state);
					continue;
				}
				received = true;

				resp = uartd_handle_header(port);
				if (resp == uartd_reset)
				{ // Bad state, reset port
					WARN_STR("!UartHeaderBad");
					WARN_CHARR(':', INT99_TO_CHARR(state->header.tag), '+', INT9999_TO_CHARR(state->header.length));
					break;
				}
				else if (resp == uartd_unknown)
				{ // Don't trust packet header, skip to end
					WARN_STR("!UartHeaderUnknown");
					WARN_CHARR(':', INT99_TO_CHARR(state->header.tag), '+', INT9999_TO_CHARR(state->header.length));
					skipToEnd(state);
					continue;
				}
				else
				{
					UARTTR_STR("=");
					state->dataPos = 0;
					state->ignoreData = resp == uartd_ignore;
					if (state->header.length > 0)
						state->inData = true;
					else if (!state->ignoreData) // Handle empty packets
						uartd_handle_data(port, state->bufferPtr+pos, 0);
				}
			}
			TimeSpan dT = GetTimeSinceUS(now);
			if (dT > 100)
				TERR_STR("!UART_PD_H");
		}
		else if (state->searchingEnd)
		{ // Searching for trailing bytes that mark a packet end
			if (state->numTrailing < UART_TRAILING_SEND && state->bufferPtr[pos] == UART_TRAILING_BYTE)
			{
				state->numTrailing++;
				pos++;
			}
			else if (state->numTrailing < UART_TRAILING_RECV)
			{ // Not enough, random trailing byte?
				state->numTrailing = 0;
				pos++;
			}
			else
			{ // Found end
				state->numTrailing = 0;
				state->searchingEnd = false;
				UARTTR_STR("$");
			}
		}
		else
		{ // Searching for leading bytes that mark a packet header
			if (state->bufferPtr[pos] == UART_LEADING_BYTE)
			{ // First header byte cannot be leading byte, so search until we find one
				state->numLeading++;
				pos++;
			}
			else if (state->numLeading < UART_LEADING_RECV)
			{ // Not enough, random leading byte?
				state->numLeading = 0;
				pos++;
				noiseData++;
			}
			else
			{ // Found first header byte after leading bytes
				state->numLeading = 0;
				state->inHeader = true;
				state->headerPos = 0;
				state->headerPtr = state->bufferPtr+pos;
				state->lastPacketTime = start; // For timesync purposes
				UARTTR_STR("#");
			}
		}
		iterations++;
	}
	if (received)
	{ // Debug parsing time
		TimeSpan dtUS = GetTimeSinceUS(state->lastComm);
		DEBUG_CHARR('+', 'T', INT999_TO_CHARR(dtUS), '>');
	}
	if (noiseData > 0)// && (received || state->inData || state->inHeader))
	{ // At the end of a noise packet, debug noise
		INFO_CHARR('/', 'N', 'S', 'E', INT999_TO_CHARR(noiseData));
		noiseData = 0;
	}
	TimeSpan dT = GetTimeSinceUS(start);
	if (dT > 100)
	{
		TERR_STR("!UART_PD");
		TERR_CHARR('+', 'I', 'T', INT99999999_TO_CHARR(iterations));
	}
	if (iterations > 100)
	{
		TERR_CHARR('/', 'I', 'T', INT99999999_TO_CHARR(iterations));
	}
	return resp;
}

void uartd_process_port(uint_fast8_t port, uint_fast16_t tail)
{
	PortState *state = &portStates[port];
	if (GetTimeSinceMS(state->resetTimer) < UART_RESET_TIMEOUT_MS)
	{
		state->parsePos = tail;
		return;
	}

	UARTTR_STR("/RX:");
	UARTTR_CHARR(UI16_TO_HEX_ARR(state->parsePos), ':', UI16_TO_HEX_ARR(tail));
	uint32_t len = 0;
	uartd_respond resp = uartd_ignore;
	if (state->parsePos < tail)
	{ // One continuous range
		len = tail-state->parsePos;

		resp = uartd_process_data(port, state->parsePos, tail);
	}
	else if (state->parsePos > tail)
	{ // Two ranges, at start and end of ringbuffer
		len = UART_RX_BUFFER_SIZE+tail-state->parsePos;

		resp = uartd_process_data(port, state->parsePos, UART_RX_BUFFER_SIZE);

		TimeSpan dT = GetTimeSinceUS(state->lastComm);
		if (resp != uartd_reset && resp != uartd_reset_nak && tail > 0)
		{
			TimePoint now = GetTimePoint();
			state->parsePos = 0;
			resp = uartd_process_data(port, 0, tail);
		}
	}
	if (resp == uartd_reset)
		uartd_reset_port_int(port);
	else if (resp == uartd_reset_nak)
	{
		uartd_reset_port_int(port); // Interrupts and clears TX
		delayUS(10);
		uart_send_dma(port, msg_nak, sizeof(msg_nak));
	}
	else
	{
		if (tail == UART_RX_BUFFER_SIZE)
			state->parsePos = 0;
		else
			state->parsePos = tail;
	}

	TimeSpan dT = GetTimeSinceUS(state->lastComm);
	if (dT > 200 || (dT > 50 && dT*5 > len) )
	{
		TERR_STR(")Proc");
		TERR_CHARR(':', UINT9999_TO_CHARR(len), ':', UINT9999_TO_CHARR(dT));
	}
}

UARTPacketRef* allocateUARTPacket(uint16_t packetLength)
{ // Return a part that hasn't been used recently (might still be used for DMA TX, no hard reinforcement)
	packetLength += UART_PACKET_OVERHEAD_SEND;
	USE_LOCKS();
	LOCK();
	UARTPacketRef *addr;
	uint8_t align = (UARTPacketBufferPos+sizeof(UARTPacketRef)) & 0b11;
	UARTPacketBufferPos += (4-align) & 0b11;
	if (sizeof(UARTPacketBuffer)-UARTPacketBufferPos > packetLength)
	{
		addr = (UARTPacketRef*)&UARTPacketBuffer[UARTPacketBufferPos];
		UARTPacketBufferPos += packetLength;
	}
	else
	{
		align = (4 - (sizeof(UARTPacketRef) & 0b11)) & 0b11;
		addr = (UARTPacketRef*)&UARTPacketBuffer[align];
		UARTPacketBufferPos = align+packetLength;
	}
	//assert(((intptr_t)addr->data & 0b11) == 0);
	UNLOCK();
	return addr;
}