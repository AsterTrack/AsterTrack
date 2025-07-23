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
#include "stm32g030xx.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_rcc.h"
//#include "stm32g0xx_ll_wwdg.h"
#endif

#include "util.h"
#include "uartd.h"
#include "rgbled.h"
#include "comm/commands.h"

#include "compat.h"
#include "config_impl.h"


/* Defines */

// UART
#define UART_COMM_TIMEOUT_MS		250		// Controller sends a ping every 100ms when not already streaming
#define UART_IDENT_INTERVAL_MS		500		// Interval at which the ident packet is sent to the controller

#define FSIN_PULSE_WIDTH_US		10
#define FILTER_SWITCHER_COIL_PULSE_MS	100

#define WWDG_TIMEOUT			0		// (Timeout+1)*113.77us


/* Function Prototypes */

// UART Device callbacks
static uartd_respond uartd_handle_header(uint_fast8_t port);
static uartd_respond uartd_handle_data(uint_fast8_t port, uint8_t* ptr, uint_fast16_t size);

// I2C Driver
void i2c_driver_init();

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

// Fixed UART Messages
static struct IdentPacket ownIdent;
static uint8_t ownIdentPacket[1+PACKET_HEADER_SIZE+IDENT_PACKET_SIZE];
static uint8_t rcvIdentPacket[IDENT_PACKET_SIZE];

// camera_mcu state
volatile TimePoint lastFilterSwitch;
volatile TimePoint lastUARTActivity = 0;
volatile TimePoint lastMarker = 0;
volatile bool piIsBooted;
volatile bool piHasUARTControl;
volatile bool piWantsBootloader;
volatile bool piIsStreaming = false;
volatile enum FilterSwitchCommand filterSwitcherState;
volatile enum CameraMCUFlashConfig mcuFlashConfig = MCU_FLASH_UNKNOWN;


/* Functions */

static void uart_set_identification()
{ // This is theoretically constant, but requires code to initialise nicely
	const struct PacketHeader header = { .tag = PACKET_IDENT, .length = IDENT_PACKET_SIZE };
	ownIdentPacket[0] = UART_LEADING_BYTE;
	storePacketHeader(header, ownIdentPacket+1);
	ownIdent = (struct IdentPacket){ .device = DEVICE_TRCAM_MCU, .id = 0, .type = INTERFACE_UART, .version = version };
	storeIdentPacket(ownIdent, ownIdentPacket+(1+PACKET_HEADER_SIZE));
}

static bool UpdateFilterSwitcher(enum FilterSwitchCommand state);

static enum CameraMCUFlashConfig InterpretUserFlashConfiguration();

static void SetupUARTEXTI();

// This flag (setup in linker script to be ontop of stack) persists across software resets
extern int _bflag;
uint32_t * const BOOTLOADER_FLAG = (uint32_t*) (&_bflag);
const uint32_t BOOTLOADER_KEY = 0xB7E283F2;

int main(void)
{
	if (*BOOTLOADER_FLAG == BOOTLOADER_KEY)
	{ // Want to switch to bootloader
		*BOOTLOADER_FLAG = 0;
		// Switching now right after boot is a lot easier than mid-program
		SwitchToBootloader();
		// We should never get here
		rgbled_animation(&LED_ANIM_FLASH_BAD);
		while (true);
	}
	*BOOTLOADER_FLAG = 0;

	// Base setup
	Setup_Peripherals();

	// Initialise
	startup = GetTimePoint();
	GPIO_RESET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
	GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
	GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
	piHasUARTControl = true;
	lastUARTActivity = GetTimePoint();
	GPIO_SET(UARTSEL_GPIO_X, UARTSEL_PIN); // Route UART to Pi

	// Init LEDs to default state
	rgbled_init();
	rgbled_transition(LED_ALL_OFF, 0);

	// Enable configuration of boot/flashing configuration
	mcuFlashConfig = ReadFlashConfiguration();
	if (mcuFlashConfig == MCU_FLASH_UNKNOWN || mcuFlashConfig == MCU_FLASH_ERROR)
	{ // Invalid state, signal error state, and allow user to select a state to enter
		bool waitClearButtons = false;
		while (true)
		{ // Give user the option to pick boot/flashing configuration
			if (GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) && GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN))
				waitClearButtons = false;
			if (!waitClearButtons)
			{
				enum CameraMCUFlashConfig config = InterpretUserFlashConfiguration();
				if (config == MCU_FLASH_USER_ABORTED)
					waitClearButtons = true;
				else if (config != MCU_FLASH_KEEP)
				{
					SetFlashConfiguration(config);
					mcuFlashConfig = config;
					delayMS(1000);
					break;
				}
			}

			if (!rgbled_animating(&LED_ANIM_FLASH_BAD))
				rgbled_animation(&LED_ANIM_FLASH_BAD);

			delayMS(1);
		}
	}
	else
	{
		// Read buttons on boot to give user options to enable/disable SWD for debug
		enum CameraMCUFlashConfig config = InterpretUserFlashConfiguration();
		if (config == MCU_FLASH_USER_ABORTED)
		{ // Flash animation if button is kept held from boot, but don't give another chance to change config this boot
			rgbled_animation(&LED_ANIM_FLASH_BAD);
			while (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) || !GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN));
		}
		else if (config != MCU_FLASH_KEEP)
		{
			SetFlashConfiguration(config);
			mcuFlashConfig = config;
			delayMS(1000);
		}

	}
	if (true)
	{ // Flash current flash config
		if (mcuFlashConfig == MCU_FLASH_DEBUG_SWD)
			rgbled_transition(LED_FLASH_DEBUG_SWD, 10);
		else if (mcuFlashConfig == MCU_FLASH_BOOT0_PI)
			rgbled_transition(LED_FLASH_BOOT0_PI, 10);
		delayMS(100);
	}
	rgbled_transition(LED_STANDBY, 10);

	DEBUG_STR("/START");

	// Initialise version
	version = GetVersion(0, 0, 0);

#if !defined(BOARD_OLD)/*  && defined(USE_UART)
	// Route UART to this STM32
	GPIO_RESET(UARTSEL_GPIO_X, UARTSEL_PIN);
	piHasUARTControl = false;
#elif !defined(BOARD_OLD) && !defined(USE_UART) */
	// Route UART to Pi
	GPIO_SET(UARTSEL_GPIO_X, UARTSEL_PIN);
	piHasUARTControl = true;
#else
	piHasUARTControl = true;
#endif

#if defined(USE_UART)
	// Init UART device
	uartd_init((uartd_callbacks){ uartd_handle_header, uartd_handle_data, NULL });
	// Prepare identification to be sent out over UART
	uart_set_identification();
#else
	SetupUARTEXTI();
#endif

#if defined(USE_I2C)
	// Init I2C device
	i2c_driver_init();
#endif

	// Bring filter switcher into default position
	UpdateFilterSwitcher(FILTER_SWITCH_INFRARED);

	// Start of main loop
	TimePoint lastLoopIT = GetTimePoint();
	while (1)
	{
		delayUS(10);

		TimePoint now = GetTimePoint();
		TimeSpan loopDiff = GetTimeSpanUS(lastLoopIT, now);
		if (loopDiff > 120)
		{
			WARN_CHARR('/', 'L', 'A', 'G', INT99999_TO_CHARR(loopDiff));
			if (loopDiff > 1000)
			{
				WARN_CHARR('+', 'P', INT9_TO_CHARR(GetMS(now)), ':', INT999_TO_CHARR(GetUS(now)), 'D', INT9999_TO_CHARR(GetTimeSpanUS(now, lastLoopIT)));
			}
		}
		lastLoopIT = now;

		if (piWantsBootloader)
		{
			delayMS(100);
			rgbled_transition(LED_ACTIVE, 500);
			// Set flag that persists the reset to switch to bootloader
			*BOOTLOADER_FLAG = BOOTLOADER_KEY;
			NVIC_SystemReset();
			// Should never get here
			rgbled_animation(&LED_ANIM_FLASH_BAD);
			while (true);
		}

		/* // Automatic switching of filter switcher for testing
		UpdateFilterSwitcher(FILTER_SWITCH_VISIBLE);
		delayUS(2000000);
		UpdateFilterSwitcher(FILTER_SWITCH_INFRARED);
		delayUS(10000000); */

		{
			// Allow switching of filter via buttons
			enum FilterSwitchCommand target = FILTER_KEEP;
			if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN))
				target = FILTER_SWITCH_INFRARED;
			else if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN))
				target = FILTER_SWITCH_VISIBLE;

			// Update status of filter and LEDs showing status
			static enum FilterSwitchCommand showingFSStatus = FILTER_KEEP;
			static TimePoint lastFSStatus;
			if (target != FILTER_KEEP)
			{ // Wish to switch filter
				UpdateFilterSwitcher(target);
				if (showingFSStatus != target)
					rgbled_transition(target == FILTER_SWITCH_INFRARED? LED_FILTER_INFRARED : LED_FILTER_VISIBLE, 200);
				showingFSStatus = target;
				lastFSStatus = GetTimePoint();
			}
			else
			{ // No change to filter desired
				UpdateFilterSwitcher(FILTER_KEEP);
				if (showingFSStatus != FILTER_KEEP &&
					GetTimeSinceMS(lastFSStatus) > 500 &&
					GetTimeSinceMS(lastFilterSwitch) > 1000)
				{
					rgbled_transition(piIsBooted? LED_ACTIVE : LED_STANDBY, 500);
					showingFSStatus = FILTER_KEEP;
				}
			}
		}

		/* if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) && brightness > 0.01f)
		{
			brightness = brightness * 0.8;
			rgbled_transition_to(RGB_DEFAULT_ON, 0);
		}
		else if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN) && brightness < 0.8)
		{
			brightness = brightness * 1.25f;
			rgbled_transition_to(RGB_DEFAULT_ON, 0);
		} */

		now = GetTimePoint();

		/* Indicate UART activity on orange LED */
		if (GetTimeSpanMS(lastMarker, now) < 20)
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
		else
			GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);

		// Simple blinking of green LED to indicate activity
		static TimePoint lastPowerLEDToggle = 0;
		static bool powerLEDToggle = false;
		if (GetTimeSpanMS(lastPowerLEDToggle, now) > 500)
		{
			lastPowerLEDToggle = GetTimePoint();
			powerLEDToggle = !powerLEDToggle;
			if (powerLEDToggle)
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

		// TODO: Inconsistent check once lastTimeCheck.ms lapses, will check every 100us within last 10ms, but will not happen normally
		now = GetTimePoint();
		static TimePoint lastTimeCheck = 0;
		if (GetTimeSpanMS(lastTimeCheck, now) < 10)
			continue;
		lastTimeCheck = now;
		// Check all kinds of timeouts >= 50 ms

#if defined(USE_UART)
#if !defined(BOARD_OLD)
		// Check identification send timeout
		static TimePoint lastIdent = 0;
		if (!piHasUARTControl && GetTimeSpanMS(lastIdent, now) > UART_IDENT_INTERVAL_MS)
		{ // Send identification packet occasionally 
			lastIdent = now;
			uartd_send(0, ownIdentPacket, sizeof(ownIdentPacket), true);
		}
#endif

		// Check UART timeouts
		if (portStates[0].ready)
		{
			TimeSpan timeSinceLastComm = GetTimeSpanMS(portStates[0].lastComm, lastTimeCheck);
			if (timeSinceLastComm > UART_COMM_TIMEOUT_MS || timeSinceLastComm < -1)
			{ // Reset Comm after silence
				EnterUARTZone(); // Mostly for the debug
				if (!piHasUARTControl)
					uartd_nak_int(0);
				uartd_reset_port_int(0);
				// Configurator is notified by its camera iteration check
				WARN_CHARR('/', 'T', 'M', 'O'); // TiMeOut
				LeaveUARTZone();
			}
		}
#endif
	}
}


/* ------ UART Behaviour ------ */

static uartd_respond uartd_handle_header(uint_fast8_t port)
{
	PortState *state = &portStates[port];
	if (state->header.tag >= PACKET_MAX_ID_POSSIBLE)
		return uartd_unknown;

	if (!state->ready)
	{
		if (state->header.tag == PACKET_IDENT)
		{ // Redundant identification
			if (state->header.length == IDENT_PACKET_SIZE)
			{ // Correct size, receive and check fully
				lastMarker = GetTimePoint();
				return uartd_accept;
			}
			WARN_CHARR('/', 'I', 'N', 'K');
			if (!piHasUARTControl)
				uartd_nak_int(port);
			return uartd_reset;
		}
		else if (!piHasUARTControl)
		{ // Invalid packet before identification - only complain when we're in control - otherwise, just accept packets even without identification
			WARN_CHARR('/', 'I', 'P', 'K', '+', UI8_TO_HEX_ARR(state->header.tag));
			uartd_nak_int(port);
			return uartd_reset;
		}
	}

	if (state->header.tag == PACKET_SOF)
	{ // Received sof packet
		lastMarker = GetTimePoint();

#if !defined(USE_SYNC)
		GPIO_SET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
		delayUS(FSIN_PULSE_WIDTH_US);
		GPIO_RESET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
#endif

		return uartd_accept;
	}
	else if (state->header.tag == PACKET_SYNC)
	{ // Received sync packet
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_PING)
	{
		if (!piHasUARTControl)
		{ // Answer ping to notify of existence
			uartd_send_int(0, msg_ping, sizeof(msg_ping), true);
		}
		lastMarker = GetTimePoint();
		return uartd_ignore;
	}
	else if (state->header.tag == PACKET_NAK)
	{
		WARN_CHARR('/', 'N', 'A', 'K');
		state->ready = false;
		uartd_reset_port(port);
		return uartd_reset;
	}
	else if (state->header.tag == PACKET_ACK)
	{ // Redundant ACK
		WARN_CHARR('/', 'A', 'C', 'K');
		return uartd_ignore;
	}
	else if (state->header.tag == PACKET_IDENT)
	{ // Redundant identification
		WARN_CHARR('/', 'I', 'R', 'D');
		if (!piHasUARTControl)
			uartd_nak_int(port);
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
		//state->lastPacketTime;
	}
	else if (state->header.tag == PACKET_SYNC)
	{ // Received sync packet
		//state->lastPacketTime;
	}
	else if (state->header.tag == PACKET_IDENT)
	{ // Identification
		memcpy(rcvIdentPacket+state->dataPos, ptr, size);
		//assert(!state->ready);
		if (state->dataPos+size == IDENT_PACKET_SIZE)
		{ // Received identification, check
			struct IdentPacket ident = parseIdentPacket(rcvIdentPacket);
			if (ident.device != DEVICE_TRCONT || ident.type != INTERFACE_UART)
			{ // Ident packet was plain wrong
				WARN_STR("!IdentFail:");
				WARN_CHARR(INT9_TO_CHARR(port));
				WARN_CHARR('+', UI8_TO_HEX_ARR(ident.device), '+', UI8_TO_HEX_ARR(ident.type));
				if (!piHasUARTControl)
					uartd_nak_int(port);
				return uartd_reset;
			}
			if (ident.version.major != ownIdent.version.major || ident.version.minor != ownIdent.version.minor)
			{ // TODO: Deal with versions
				WARN_STR("!VersionMismatch:");
				WARN_CHARR(INT9_TO_CHARR(port));
				WARN_STR("+Cont:v");
				WARN_CHARR(INT99_TO_CHARR(ownIdent.version.major), '.', INT99_TO_CHARR(ownIdent.version.minor));
				WARN_STR("+Cam:v");
				WARN_CHARR(INT99_TO_CHARR(ident.version.major), '.', INT99_TO_CHARR(ident.version.minor));
				if (!piHasUARTControl)
					uartd_nak_int(port);
				return uartd_reset;
			}
			// Idenfication verified
			state->ready = true;
			COMM_CHARR('/', 'I', 'D', 'S');
			// Send acknowledgement
			if (!piHasUARTControl)
				uartd_ack_int(port);
		}
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_CFG_MODE)
	{
		if (size >= 1)
		{
			uint8_t modePacket = ptr[0];
			bool streaming = modePacket&TRCAM_FLAG_STREAMING;
			if (!piIsStreaming && streaming)
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
			if (piIsStreaming && !streaming)
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
			piIsStreaming = streaming;
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
	return uartd_accept;
}


/* ------ I2C Behaviour ------ */

bool i2cd_handle_command(enum CameraMCUCommand command, uint8_t *data, uint8_t len)
{
	piIsBooted = true;
	rgbled_transition(LED_ACTIVE, 500);

	switch (command)
	{
		case MCU_REQUEST_UART:
			piHasUARTControl = true;
			GPIO_SET(UARTSEL_GPIO_X, UARTSEL_PIN);
			return true;
		case MCU_RELEASE_UART:
			piHasUARTControl = false;
			GPIO_RESET(UARTSEL_GPIO_X, UARTSEL_PIN);
			return true;
		case MCU_SWITCH_BOOTLOADER:
			piWantsBootloader = true;
			rgbled_transition(LED_BOOTLOADER, 0);
			return true;
		case MCU_PING:
			// Nothing to do
			return true;
			break;
		default:
			return false;
			break;
	}
}

uint8_t i2cd_prepare_response(enum CameraMCUCommand command, uint8_t *data, uint8_t len, uint8_t response[256])
{
	piIsBooted = true;
	rgbled_transition(LED_ACTIVE, 500);

	switch (command)
	{
		case MCU_REG_ID:
			response[0] = MCU_I2C_ID;
			return 1;
		default:
			// TODO: Error?
			return 0;
	}
}


/* ------ Functional Behaviour ------ */

static bool UpdateFilterSwitcher(enum FilterSwitchCommand state)
{
	bool switched = state != FILTER_KEEP && filterSwitcherState != state;
	if (switched)
	{ // Need to actuate the motor to switch
		filterSwitcherState = state;
		lastFilterSwitch = GetTimePoint();
	}
	int timeSince = GetTimeSinceMS(lastFilterSwitch);
	bool actuateMotor = timeSince < FILTER_SWITCHER_COIL_PULSE_MS; // Need 10-50ms to switch
	/* if (timeSince > 10000)
	{ // Actuate motor regularly just in case a vibration knocked it out of place
	 	// Sadly it's kind of audible so don't do for now
		actuateMotor = true;
		lastFilterSwitch = GetTimePoint();
	} */
	if (actuateMotor)
	{
		if (filterSwitcherState == FILTER_SWITCH_VISIBLE)
		{
			GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN);
			GPIO_SET(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN);
			GPIO_SET(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP);
		}
		else if (filterSwitcherState == FILTER_SWITCH_INFRARED)
		{
			GPIO_SET(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN);
			GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN);
			GPIO_SET(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP);
		}
	}
	else
	{
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_INFRARED_PIN);
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_VISIBLE_PIN);
		GPIO_RESET(FILTERSW_GPIO_X, FILTERSW_PIN_SLEEP);
	}
	return switched;
}

static enum CameraMCUFlashConfig InterpretUserFlashConfiguration()
{
	TimePoint check = GetTimePoint();
	if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) && GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN))
	{
		rgbled_animation(&LED_ANIM_FLASH_CHARGE_BOT_DEBUG_SWD);
		while (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) && GetTimeSinceMS(check) < CHARGE_TIME_MAX_MS);
		if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) || !GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN))
			return MCU_FLASH_USER_ABORTED; // A button is (still) pressed after check concluded
		if (GetTimeSinceMS(check) < CHARGE_TIME_MIN_MS)
			return MCU_FLASH_USER_ABORTED; // Button was pressed too short - be very conservative here
		rgbled_transition(LED_FLASH_DEBUG_SWD, 50);
		return MCU_FLASH_DEBUG_SWD;
	}
	else if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN) && GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN))
	{
		rgbled_animation(&LED_ANIM_FLASH_CHARGE_TOP_BOOT0_PI);
		while (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN) && GetTimeSinceMS(check) < CHARGE_TIME_MAX_MS);
		if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN) || !GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN))
			return MCU_FLASH_USER_ABORTED; // A button is (still) pressed after check concluded
		if (GetTimeSinceMS(check) < CHARGE_TIME_MIN_MS)
			return MCU_FLASH_USER_ABORTED; // Button was pressed too short - be very conservative here
		rgbled_transition(LED_FLASH_BOOT0_PI, 50);
		return MCU_FLASH_BOOT0_PI;
	}
	return MCU_FLASH_KEEP;
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
#if defined(BOARD_OLD) && defined(USE_SYNC)
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