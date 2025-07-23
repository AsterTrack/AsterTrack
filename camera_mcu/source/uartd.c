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

#include "util.h"
#include "uartd.h"
#include "uart_driver.h"
#include "rgbled.h"

#if defined(STM32G0)
#include "stm32g0xx_ll_dma.h"
#endif

// Definitions of GPIO Pins / DMA Channels and access so that some descriptive structures (UART_DMA_Setup UART) can be shared
#include "compat.h"

#include <stdint.h>
#include <string.h>


/* Variables */

// UART State
PortState portStates[UART_PORT_COUNT];

// Predefined message
uint8_t msg_ack[1+PACKET_HEADER_SIZE], msg_nak[1+PACKET_HEADER_SIZE], msg_ping[1+PACKET_HEADER_SIZE];

// UART Callbacks
static uartd_callbacks impl_callbacks;

// Hardware mapping
struct UART_IO_State UART_IO[UART_PORT_COUNT] = {};
const struct UART_DMA_Setup UART[UART_PORT_COUNT] = {
	{ // UART 1
	#if defined(BOARD_OLD)
		.uart = USART2,
	#else
		.uart = USART1,
	#endif
		.DMA = DMA1,
		.DMA_CH_RX = DMA_CHANNEL_2,
		.DMA_CH_TX = DMA_CHANNEL_3,
	#if defined(BOARD_OLD)
		.uartIRQ_RX = USART2_IRQn,
	#else
		.uartIRQ_RX = USART1_IRQn,
	#endif
		.dmaIRQ_RX = DMA1_Channel2_3_IRQn,
		.dmaIRQ_TX = DMA1_Channel2_3_IRQn,
	}
};

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
inline void EnterUARTZone()
{
	// Does NOT support nesting
	ATOMIC_SINGLE(DisableUARTInterrupts(););
}
inline void LeaveUARTZone()
{
	// Does NOT support nesting
	ATOMIC_SINGLE(EnableUARTInterrupts(););
}
#pragma GCC pop_options


/* Functions */

/** UART Initialization Function */
void uartd_init(uartd_callbacks callbacks)
{
	// Init predefined messages
	msg_ping[0] = UART_LEADING_BYTE;
	struct PacketHeader header = { .tag = PACKET_PING, .length = 0 };
	storePacketHeader(header, msg_ping+1);
	msg_nak[0] = UART_LEADING_BYTE;
	header.tag = PACKET_NAK;
	storePacketHeader(header, msg_nak+1);
	msg_ack[0] = UART_LEADING_BYTE;
	header.tag = PACKET_ACK;
	storePacketHeader(header, msg_ack+1);

	// Init port states
	memset(portStates, 0, sizeof(portStates));
	for (int i = 0; i < UART_PORT_COUNT; i++)
	{
		if (UART_IO[i].rx_alloc)
			free(UART_IO[i].rx_alloc);
		UART_IO[i].rx_alloc = malloc(RX_HEADROOM+UART_RX_BUFFER_SIZE);
		UART_IO[i].rx_buffer = (uint8_t*)UART_IO[i].rx_alloc;
		portStates[i].bufferPtr = UART_IO[i].rx_buffer;
	}

	impl_callbacks = callbacks;

	uart_driver_init();
}

/** Send data over UART port */

/**
 * Use from within a UART interrupt or an interrupt that can't be preempted by a UART interrupt
 */
void uartd_send_int(uint_fast8_t port, const uint8_t* data, uint_fast16_t len, bool isOwner)
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
void uartd_send(uint_fast8_t port, const uint8_t* data, uint_fast16_t len, bool isOwner)
{
	EnterUARTZone();
	uartd_send_int(port, data, len, isOwner);
	LeaveUARTZone();
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
	// Restart RX DMA
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_RX)->COUNTER = UART_RX_BUFFER_SIZE;
	DMA_CH(UART[port].DMA, UART[port].DMA_CH_RX)->CONTROL |= DMA_CONTROL_ENABLE;
}
void uartd_reset_port(uint_fast8_t port)
{
	EnterUARTZone();
	uartd_reset_port_int(port);
	LeaveUARTZone();
}

/** Process received data over UART */
extern volatile TimePoint lastUARTActivity;
static bool uartd_process_data(uint_fast8_t port, uint_fast16_t begin, uint_fast16_t end)
{
	TimePoint start = GetTimePoint();
	// TODO: Log fracturing of packets when spearate comm_write are used instead of a block-write. Not optimal.
	UART_CHARR('+', 'B', INT999_TO_CHARR(begin), '>', INT999_TO_CHARR(end));

	lastUARTActivity = GetTimePoint();
	PortState *state = &portStates[port];
	static int noiseData = 0;
	bool received = false, success = true;
	int iterations = 0;
	uint_fast16_t pos = begin;
	while (pos < end)
	{
		if (state->inData)
		{ // Handle data in blocks
			UARTTR_STR("+D");
			state->lastComm = start;
			TimePoint now = GetTimePoint();
			uint16_t missingSize = state->header.length - state->dataPos;
			uint16_t blockSize = end - pos;
			bool complete = blockSize >= missingSize;
			if (complete) blockSize = missingSize;
			if (!state->ignoreData && impl_callbacks.uartd_handle_data)
			{
				uartd_respond resp = impl_callbacks.uartd_handle_data(port, state->bufferPtr+pos, blockSize);
				if (resp == uartd_reset)
				{
					WARN_STR("!UartDataBad");
					success = false;
					break;
				}
				state->ignoreData = resp == uartd_unknown || resp == uartd_ignore;
			}
			pos += blockSize;

			if (complete && impl_callbacks.uartd_handle_packet)
				impl_callbacks.uartd_handle_packet(port, pos);
			state->dataPos += blockSize;
			state->inData = !complete;
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
			while (state->headerPos < 4 && pos < end)
				state->headerRaw[state->headerPos++] = state->bufferPtr[pos++];
			if (state->headerPos == 4)
			{ // Found header of data, handle data next
				UARTTR_CHARR('+', 'H', ':', UI32_TO_HEX_ARR(*(uint32_t*)state->headerRaw));
				state->inHeader = false;
				state->header = parsePacketHeader(state->headerRaw);
				UART_CHARR('<', INT9_TO_CHARR(port), 'R', 'C', 'V',
					'+', INT99_TO_CHARR(state->header.tag), ':', INT9999_TO_CHARR(state->header.length),
					//'+', 'T', INT99_TO_CHARR(state->lastComm.ms), ':', INT999_TO_CHARR(state->lastComm.us)
				);
				if (state->header.length > 20000)
				{ // Just assuming this is an error. CAN happen if wifi on camera is enabled (more UART errors?)
					WARN_STR("!UartHeaderOverLength:");
					WARN_CHARR(INT99999999_TO_CHARR(state->header.length));
					return false;
				}
				received = true;

				uartd_respond resp = impl_callbacks.uartd_handle_header(port);
				if (resp == uartd_reset)
				{ // Bad state, reset port
					WARN_STR("!UartHeaderBad");
					WARN_CHARR(':', INT99_TO_CHARR(state->header.tag), '+', INT9999_TO_CHARR(state->header.length));
					success = false;
					break;
				}
				else
				{
					UARTTR_STR("=");
					state->dataPos = 0;
					state->ignoreData = resp == uartd_unknown || resp == uartd_ignore;
					if (state->header.length > 0)
						state->inData = true;
					else if (!state->ignoreData && impl_callbacks.uartd_handle_data) // Handle empty packets
						impl_callbacks.uartd_handle_data(port, state->bufferPtr+pos, 0);
					if (resp == uartd_unknown)  // HeaDer Unknown
						WARN_CHARR('/', 'H', 'D', 'U', INT99_TO_CHARR(state->header.tag));
				}
			}
			TimeSpan dT = GetTimeSinceUS(now);
			if (dT > 100)
				TERR_STR("!UART_PD_H");
		}
		else if (state->bufferPtr[pos] == UART_LEADING_BYTE)
		{
			state->inHeader = true;
			state->headerPos = 0;
			pos++;
			state->headerPtr = state->bufferPtr+pos;
			state->lastPacketTime = start; // For timesync purposes
			UARTTR_STR("#");
		}
		else if (state->bufferPtr[pos] == UART_TRAILING_BYTE)
		{ // Trailing bytes are added so that dropped bytes in the packet don't cause them to consume the start of the next packet
			UARTTR_STR("$");
			pos++;
		}
		else
		{
			pos++;
			noiseData++;
		}
		if (noiseData > 0 && (received || state->inData || state->inHeader))
		{ // At the end of a noise packet, debug noise
			INFO_CHARR('/', 'N', 'S', 'E', INT999_TO_CHARR(noiseData));
			noiseData = 0;
		}
		iterations++;
	}
	if (received)
	{ // Debug parsing time
		TimeSpan dtUS = GetTimeSinceUS(state->lastComm);
		DEBUG_CHARR('+', 'T', INT999_TO_CHARR(dtUS), '>');
	}
	else if (noiseData > 0)
	{ // At the end of a noise packet, debug noise
		INFO_CHARR('/', 'N', 'S', 'E', INT999_TO_CHARR(noiseData));
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
	if (!success)
	{
		WARN_STR("+CommReset:");
		WARN_CHARR(INT9_TO_CHARR(port));
		rgbled_transition(LED_UART_ERROR, 0);
		ReturnToDefaultLEDState(UART_RESET_TIMEOUT_MS);
	}
	return success;
}

void uartd_process_port(uint_fast8_t port, uint_fast16_t tail)
{
	PortState *state = &portStates[port];
	if (GetTimeSinceMS(state->resetTimer) < UART_RESET_TIMEOUT_MS)
	{
		state->parsePos = tail;
		return;
	}

	UART_STR("/RX:");
	UART_CHARR(UI16_TO_HEX_ARR(state->parsePos), ':', UI16_TO_HEX_ARR(tail));
	if (state->parsePos < tail)
	{ // One continuous range
		if (!uartd_process_data(port, state->parsePos, tail))
			uartd_reset_port_int(port);

		TimeSpan dT = GetTimeSinceUS(state->lastComm);
		if (dT > 100)
			TERR_STR("!UART_PP_1");
	}
	else if (state->parsePos > tail)
	{ // Two ranges, at start and end of ringbuffer
		if (!uartd_process_data(port, state->parsePos, UART_RX_BUFFER_SIZE))
			uartd_reset_port_int(port);

		TimeSpan dT = GetTimeSinceUS(state->lastComm);
		if (dT > 100)
			TERR_STR("!UART_PP_2.1");
		if (tail > 0)
		{
			TimePoint now = GetTimePoint();
			state->parsePos = 0;
			if (!uartd_process_data(port, 0, tail))
				uartd_reset_port_int(port);

			dT = GetTimeSinceUS(now);
			if (dT > 100)
				TERR_STR("!UART_PP_2.2");
		}
	}
	if (tail == UART_RX_BUFFER_SIZE)
		state->parsePos = 0;
	else
		state->parsePos = tail;

	TimeSpan dT = GetTimeSinceUS(state->lastComm);
	if (dT > 100)
		TERR_STR("!UART_PP");
}