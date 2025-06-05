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
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_rcc.h"
//#include "stm32g0xx_ll_wwdg.h"
#endif

#include "main.h"
#include "util.h"
#include "uartd.h"

#include "compat.h"
#include "config_impl.h"


/* Defines */

// UART
#define UART_IDENT_INTERVAL		500		// Interval in which ports are probed for cameras with an ident packet
#define UART_PING_INTERVAL		500		// Interval in which Tracking Cameras are pinged when idle
#define UART_TIME_SYNC_INTERVAL	10		// Minimum interval used for time sync with Tracking Cameras (doesn't need to be super accurate)
#define UART_COMM_TIMEOUT		5000	// Comm timeout at which a camera is considered disconnected

#define FSIN_PULSE_WIDTH_US		10

#define WWDG_TIMEOUT			0		// (Timeout+1)*113.77us


/* Function Prototypes */

// UART Device callbacks
static uartd_respond uartd_handle_header(uint_fast8_t port);
static uartd_respond uartd_handle_data(uint_fast8_t port, uint8_t* ptr, uint_fast16_t size);

/* Variables */

union VersionDesc version; // Initialised at the beginning

// Temp Send Buffer
static uint8_t sharedSendBuffer[1024];
static volatile uint16_t sendBufferPos = 0;
static uint8_t* getSendBuffer(uint8_t len)
{ // Return a part that hasn't been used recently (might still be used for DMA TX, no hard reinforcement)
	USE_LOCKS();
	LOCK();
	uint8_t *addr;
	if (sizeof(sharedSendBuffer)-sendBufferPos > len)
	{
		addr = &sharedSendBuffer[sendBufferPos];
		sendBufferPos += len;
	}
	else
	{
		sendBufferPos = len;
		addr = sharedSendBuffer;
	}
	UNLOCK();
	return addr;
}

// Times for supervision
TimePoint startup = 0;
static TimePoint lastPing = 0;
static TimePoint lastIdent = 0;

// Fixed UART Messages
static struct IdentPacket ownIdent;
static uint8_t ownIdentPacket[1+PACKET_HEADER_SIZE+IDENT_PACKET_SIZE];
static uint8_t rcvIdentPacket[IDENT_PACKET_SIZE];

// Filter switcher state
enum FilterSwitchCommand filterSwitcherState = FILTER_SWITCH_INFRARED;

// Synced camera_pi state
bool isStreaming = false;

// camera_mcu state
volatile TimePoint lastUARTActivity = 0;


/* Functions */

static void uart_set_identification()
{ // This is theoretically constant, but requires code to initialise nicely
	const struct PacketHeader header = { .tag = PACKET_IDENT, .length = IDENT_PACKET_SIZE };
	ownIdentPacket[0] = UART_LEADING_BYTE;
	storePacketHeader(header, ownIdentPacket+1);
	ownIdent = (struct IdentPacket){ .device = DEVICE_TRCAM_MCU, .id = 0, .type = INTERFACE_UART, .version = version };
	storeIdentPacket(ownIdent, ownIdentPacket+(1+PACKET_HEADER_SIZE));
}

static void UpdateFilterSwitcher(enum FilterSwitchCommand state);

static void SetupUARTEXTI();

int main(void)
{
	// Base setup
	Setup_Peripherals();

#if !defined(USE_UART)
	SetupUARTEXTI();
#endif

	// Initialise
	GPIO_RESET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
	GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
	GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
	lastUARTActivity = GetTimePoint();

#if !defined(BOARD_OLD) && defined(USE_UART)
	// Route UART to this STM32
	//GPIO_RESET(UARTSEL_GPIO_X, UARTSEL_PIN);

	// Route UART to Pi
	GPIO_SET(UARTSEL_GPIO_X, UARTSEL_PIN);
#endif

	DEBUG_STR("/START");

	// Startup sequence
	startup = GetTimePoint();

	// Initialise version
	version = GetVersion(0, 0, 0);

#if defined(USE_UART)
	// Init UART device
	uartd_init((uartd_callbacks){ uartd_handle_header, uartd_handle_data, NULL });
#endif

	while(1)
	{
		/* // Automatic switching of filter switcher for testing
		UpdateFilterSwitcher(FILTER_SWITCH_VISIBLE);
		delayUS(2000000);
		UpdateFilterSwitcher(FILTER_SWITCH_INFRARED);
		delayUS(10000000); */

		// Allow toggling of filter switcher via buttons
		if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN))
			UpdateFilterSwitcher(FILTER_SWITCH_VISIBLE);
		else if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN))
			UpdateFilterSwitcher(FILTER_SWITCH_INFRARED);
		else
			UpdateFilterSwitcher(FILTER_KEEP);

		/* Flash respective LED if button has been pressed (visual only) */
		if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN))
		{
			GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
			GPIO_SET(RJLED_GPIO_X, RJLED_GREEN_PIN);
			delayUS(50000);
			GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
			delayUS(50000);
			GPIO_SET(RJLED_GPIO_X, RJLED_GREEN_PIN);
			delayUS(50000);
			GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
		}
		else if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN))
		{
			GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
			delayUS(50000);
			GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
			delayUS(50000);
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
			delayUS(50000);
			GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		}

		/* Indicate UART activity on green LED */
		if (GetTimeSinceMS(lastUARTActivity) < 20)
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		else
			GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);

		// Simple blinking of green LED to indicate activity
		static TimePoint lastOrangeLEDToggle = 0;
		static bool orangeLEDToggle = false;
		if (GetTimeSinceMS(lastOrangeLEDToggle) > 500)
		{
			lastOrangeLEDToggle = GetTimePoint();
			orangeLEDToggle = !orangeLEDToggle;
			if (orangeLEDToggle)
			{
				GPIO_SET(RJLED_GPIO_X, RJLED_GREEN_PIN);
			#if !defined(BOARD_OLD) && !defined(USE_UART)
				GPIO_SET(GPIOA, GPIO_PIN_9);
			#endif
			} else {
				GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
			#if !defined(BOARD_OLD) && !defined(USE_UART)
				GPIO_RESET(GPIOA, GPIO_PIN_9);
			#endif
			}
		}
	}
}


/* ------ UART Behaviour ------ */

static uartd_respond uartd_handle_header(uint_fast8_t port)
{
	PortState *state = &portStates[port];
	if (state->header.tag >= PACKET_MAX_ID_POSSIBLE)
		return uartd_unknown;
	if (state->header.tag == PACKET_PING)
	{
		return uartd_ignore;
	}
	else if (state->header.tag == PACKET_NAK)
	{
		WARN_CHARR('/', '0'+port, 'N', 'A', 'K');
		state->commInit = CommNoCon;
		uartd_reset_port(port);
		return uartd_reset;
	}
	else if (state->header.tag == PACKET_ACK)
	{
		if ((state->commInit & CommReady) != CommReady)
		{ // Received acknowledgement of own identity
			state->commInit |= CommACK;
			COMM_CHARR('/', '0'+port, 'A', 'C', 'K');
			// Check if comm setup is finished
			if ((state->commInit & CommReady) == CommReady)
				COMM_CHARR('/', '0'+port, 'R', 'D', 'Y');
		}
		else
			COMM_CHARR('/', '0'+port, 'U', 'A', 'K');
		return uartd_ignore;
	}
	else if (state->header.tag == PACKET_IDENT)
	{ // Identification
		if (state->header.length == IDENT_PACKET_SIZE)
			return uartd_accept;
		WARN_CHARR('/', '0'+port, 'I', 'N', 'K');
		uartd_nak(port);
		return uartd_reset;
	}
	else
	{
		return uartd_unknown;
	}
}

static uartd_respond uartd_handle_data(uint_fast8_t port, uint8_t* ptr, uint_fast16_t size)
{
	PortState *state = &portStates[port];

	if (state->header.tag == PACKET_SOF)
	{
		GPIO_SET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
		delayUS(FSIN_PULSE_WIDTH_US);
		GPIO_RESET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
	}
	else if (state->header.tag == PACKET_SYNC)
	{ // Received sync packet
	}
	else if (state->header.tag == PACKET_IDENT)
	{ // Identification
		memcpy(rcvIdentPacket+state->dataPos, ptr, size);
		//if (!(state->commInit & CommID))
		if (state->dataPos+size == IDENT_PACKET_SIZE)
		{ // Received identification, check
			// TODO:
		}
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_CFG_MODE)
	{
		if (size >= 1)
		{
			uint8_t modePacket = ptr[0];
			bool streaming = modePacket&TRCAM_FLAG_STREAMING;
			if (!isStreaming && streaming)
			{ // Requested to enter streaming mode
				GPIO_SET(RJLED_GPIO_X, RJLED_GREEN_PIN);
				delayUS(100000);
				GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
				delayUS(100000);
				GPIO_SET(RJLED_GPIO_X, RJLED_GREEN_PIN);
				delayUS(100000);
				GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
				delayUS(100000);
			}
			if (isStreaming && !streaming)
			{ // Requested to leave streaming mode
				GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
				delayUS(100000);
				GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
				delayUS(100000);
				GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
				delayUS(100000);
				GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
				delayUS(100000);
			}
			isStreaming = streaming;
			// TODO: Get this update from camera_pi directly instead over I2C?
			// E.g. Errors from camera_pi cannot be read here
		}
	}
	else if (state->header.tag == PACKET_CFG_FILTER)
	{
		if (size >= 1)
		{
			uint8_t filterState = (enum FilterSwitchCommand)ptr[0];
			if (filterState == FILTER_SWITCH_VISIBLE || filterState == FILTER_SWITCH_INFRARED)
				UpdateFilterSwitcher(filterState);
		}
	}
	else if (state->header.tag == PACKET_CFG_SIGNAL)
	{
		// TODO: Set state for LEDs to signal, e.g. focus/selection in host UI, or calibration quality, etc.
	}
	else
	{
		return uartd_unknown;
	}
}

static void UpdateFilterSwitcher(enum FilterSwitchCommand state)
{
	if (state != FILTER_KEEP)
		filterSwitcherState = state;
	if (state == FILTER_SWITCH_VISIBLE)
	{
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN);
		GPIO_SET(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN);
		GPIO_SET(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP);
	}
	else if (state == FILTER_SWITCH_INFRARED)
	{
		GPIO_SET(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN);
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN);
		GPIO_SET(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP);
	}
	else if (state == FILTER_KEEP)
	{
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN);
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN);
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP);
	}
	// May assume control over LEDs, too
	/* if (filterSwitcherState == FILTER_SWITCH_VISIBLE)
	{
		GPIO_SET(RJLED_GPIO_X, RJLED_GREEN_PIN);
		GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
	}
	else
	{
		GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
		GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
	} */
}

void SetupUARTEXTI()
{
#ifdef BOARD_OLD

	// Init GPIO pins
	// RX: Floating input
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_15, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_15, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_PIN_15, LL_GPIO_SPEED_FREQ_VERY_HIGH);

	// Setup NVIC interrupt handler for EXTI line 9
	NVIC_SetPriority(EXTI4_15_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0)); // Same priority as SOF timer
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Select PB (0x01) for EXTI Line 9 - so PB9
	EXTI->EXTICR[3] = (EXTI->EXTICR[3] & ~EXTI_EXTICR4_EXTI15_Msk) | (0x00 << EXTI_EXTICR4_EXTI15_Pos);

	// Setup interrupts for EXTI line 9
	EXTI->RTSR1 |= LL_EXTI_LINE_15; // Set to trigger on rising edge
	EXTI->IMR1 |= LL_EXTI_LINE_15; // Enable interrupt generation

#else
	// AFIO
	RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP | SYSCFG_CFGR1_PA12_RMP;

	// Init GPIO pins
	// RX: Floating input
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, GPIO_PIN_10, LL_GPIO_PULL_DOWN);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);

	// Setup NVIC interrupt handler for EXTI line 9
	NVIC_SetPriority(EXTI4_15_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0)); // Same priority as SOF timer
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	// Select PB (0x01) for EXTI Line 9 - so PB9
	EXTI->EXTICR[2] = (EXTI->EXTICR[2] & ~EXTI_EXTICR3_EXTI10_Msk) | (0x00 << EXTI_EXTICR3_EXTI10_Pos);

	// Setup interrupts for EXTI line 9
	EXTI->RTSR1 |= LL_EXTI_LINE_10; // Set to trigger on rising edge
	EXTI->IMR1 |= LL_EXTI_LINE_10; // Enable interrupt generation

	// Camera FSIN output
	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOA, GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);

#endif
}

void EXTI4_15_IRQHandler(void) __IRQ;
void EXTI4_15_IRQHandler()
{
#if defined(BOARD_OLD)
	if (EXTI->RPR1 & LL_EXTI_LINE_9)
	{ // Interrupt pending

		GPIO_SET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
		delayUS(FSIN_PULSE_WIDTH_US);
		GPIO_RESET(FSIN_GPIO_X, CAMERA_FSIN_PIN);

		// Reset IRQ flag
		EXTI->RPR1 = LL_EXTI_LINE_9;
	}
#endif

#if !defined(USE_UART) && !defined(BOARD_OLD)
	if (EXTI->RPR1 & LL_EXTI_LINE_10)
	{ // Interrupt pending

		lastUARTActivity = GetTimePoint();

		// Reset IRQ flag
		EXTI->RPR1 = LL_EXTI_LINE_10;
	}
#endif
#if !defined(USE_UART) && defined(BOARD_OLD)
	if (EXTI->RPR1 & LL_EXTI_LINE_15)
	{ // Interrupt pending

		lastUARTActivity = GetTimePoint();

		// Reset IRQ flag
		EXTI->RPR1 = LL_EXTI_LINE_15;
	}
#endif
}