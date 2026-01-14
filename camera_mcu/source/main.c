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
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_wwdg.h"
#endif
#include "compat.h"

#include "util.h"
#include "uartd.h"
#include "rgbled.h"
#include "comm/commands.h"
#include "config_impl.h"
#include "i2c_driver.h"
#include "otp.h"
#include "mcu/timesync.h"


/* Defines */

#define FSIN_PULSE_WIDTH_US		10
#define FILTER_SWITCHER_COIL_PULSE_MS	100

typedef enum
{
	UART_None,
	UART_CamMCU,
	UART_CamPi
} UART_STATE;

/* Function Prototypes */

void version_init();

/* Variables */

// Firmware Version Information initialised in version_init
extern union VersionDesc firmwareVersion;
extern const char* firmwareDescriptor;
extern const uint32_t firmwareDescriptorLength;

// Times for supervision
TimePoint startup = 0;

// Fixed UART Messages
static struct IdentPacket ownIdent;
static uint8_t ownIdentPacket[UART_PACKET_OVERHEAD_SEND+IDENT_PACKET_SIZE];

// UART receive state
PortState * const state = &portStates[0];
struct {
	uint8_t packetBuffer[UART_TEMP_PACKET_BUF];
	uint16_t packetSize;
} receive;

// camera_mcu state
volatile TimePoint lastFilterSwitch;
volatile TimePoint lastUARTActivity = 0;
volatile TimePoint lastMarker = 0;
volatile TimePoint lastPiComm = 0;
struct IdentPacket controllerIdentity;
volatile UART_STATE uartState;
// TODO: Toggle Pi Power after UART negotiation with CamMCU
volatile bool piHasPower = true;
volatile bool piIsBooted;
volatile bool piWantsBootloader;
volatile bool piIsStreaming = false;
volatile enum FilterSwitchCommand filterSwitcherState;
volatile enum CameraMCUFlashConfig mcuFlashConfig = MCU_FLASH_UNKNOWN;
volatile CameraID cameraIDToWrite = 0;

// Frame and Time sync
struct time_sync timesync;	// Current estimate of controller-MCU-time relationship
TimePoint lastTimeSync;		// Synced time of last packet with time sync
uint32_t lastFrameID;		// ID of last frame (received or generated)
TimePoint lastFrameTime;	// Intended synced time of last frame
TimePoint lastFrameSync;	// Actual local time of last frame (FSIN signal)


/* Functions */

static void uart_set_identification()
{ // This is theoretically constant, but requires code to initialise nicely
	UARTPacketRef *uartIdentPacket = (UARTPacketRef*)ownIdentPacket;
	ownIdent = (struct IdentPacket){ .device = DEVICE_TRCAM_MCU, .id = PERSISTENT_CONFIG[0], .type = INTERFACE_UART, .version = firmwareVersion };
	storeIdentPacket(ownIdent, uartIdentPacket->data);
	finaliseDirectUARTPacket(uartIdentPacket, (struct PacketHeader){ .tag = PACKET_IDENT, .length = IDENT_PACKET_SIZE });
}

static bool UpdateFilterSwitcher(enum FilterSwitchCommand state);

static bool ApplyUserFlashConfiguration();

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
	lastUARTActivity = GetTimePoint();
	uartState = UART_None;
	GPIO_RESET(UARTSEL_GPIO_X, UARTSEL_PIN); // Route UART to MCU
	GPIO_SET(I2C_INT_GPIO_X, I2C_INT_PIN); // Disable Interrupt

	// Init LEDs to default state
	rgbled_init();
	rgbled_transition(LED_ALL_OFF, 0);

	// Enable configuration of boot/flashing configuration
	mcuFlashConfig = ReadFlashConfiguration();
	if (mcuFlashConfig == MCU_FLASH_UNKNOWN || mcuFlashConfig == MCU_FLASH_ERROR)
	{ // Invalid state, signal error state, and allow user to select a state to enter
		while (true)
		{ // Give user the option to pick boot/flashing configuration
			ApplyUserFlashConfiguration();
			// Will restart on successful change, so loop 

			if (!rgbled_animating(&LED_ANIM_FLASH_BAD))
				rgbled_animation(&LED_ANIM_FLASH_BAD);

			delayMS(1);
		}
	}
	else
	{ // Check for a button pressed on boot
		ApplyUserFlashConfiguration();
		// May restart if proper button procedure was followed
	}
	if (mcuFlashConfig == MCU_FLASH_DEBUG_SWD)
	{ // Flash LED to signal non-standard flash config
		rgbled_transition(LED_FLASH_DEBUG_SWD, 10);
		delayMS(200);
	}
	rgbled_transition(LED_INITIALISING, 10);

	DEBUG_STR("/START");

	// Initialise version
	version_init();

#if defined(USE_UART)
	// Prepare identification to be sent out over UART
	uart_set_identification();
	// Init UART device
	uartd_init();
#endif

#if !defined(USE_UART) || !defined(USE_I2C)
	// Route UART to Pi
	GPIO_SET(UARTSEL_GPIO_X, UARTSEL_PIN);
	uartState = UART_CamPi;
#endif

#if defined(USE_I2C)
	// Init I2C device
	i2c_driver_init();
#endif

	// Start continuously measuring power supply voltage
	EnableADC();

	// Bring filter switcher into default position
	UpdateFilterSwitcher(FILTER_SWITCH_INFRARED);

	// Start watchdog
	EnableWatchdog();

	// Start of main loop
	TimePoint lastLoopIT = GetTimePoint();
	while (1)
	{
		delayUS(10);
		LL_WWDG_SetCounter(WWDG, WWDG_TIMEOUT);

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
			GPIO_RESET(UARTSEL_GPIO_X, UARTSEL_PIN); // Route UART to MCU
			rgbled_transition(LED_BOOTLOADER, 0);
			SafeDelayMS(100);
			// Set flag that persists the reset to switch to bootloader
			*BOOTLOADER_FLAG = BOOTLOADER_KEY;
			NVIC_SystemReset();
			// Should never get here
			rgbled_animation(&LED_ANIM_FLASH_BAD);
			while (true);
		}

		if (cameraIDToWrite)
		{
			CameraID cameraIDWrite = cameraIDToWrite;
			cameraIDToWrite = 0;
			// Signal write state
			rgbled_transition(LED_BOOTLOADER, 0);
			SafeDelayMS(100);
			// Write config
			uint32_t config[] = { cameraIDWrite, 0 };
			uint16_t error = EraseAndProgramFlash(PERSISTENT_CONFIG, config, sizeof(config)/sizeof(uint32_t));
			// Signal result
			if (error == 1)
				rgbled_transition(LED_UART_ERROR, 0);
			else if (error)
				rgbled_transition(LED_ERROR_3, 0);
			else if (config[0] != PERSISTENT_CONFIG[0] && config[1] != PERSISTENT_CONFIG[1])
				rgbled_transition(LED_ERROR_4, 0);
			else
				rgbled_transition(LED_STANDBY, 0);
			ReturnToDefaultLEDState(500);
			// Update identification packet to use new ID
			uart_set_identification();
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
				if (UpdateFilterSwitcher(target))
					GPIO_RESET(I2C_INT_GPIO_X, I2C_INT_PIN);
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
					ReturnToDefaultLEDState(500);
					showingFSStatus = FILTER_KEEP;
				}
			}
		}

		/* if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) && brightness > 0.01f)
		{
			brightness = brightness * 0.8;
			ReturnToDefaultLED();
		}
		else if (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN) && brightness < 0.8)
		{
			brightness = brightness * 1.25f;
			ReturnToDefaultLED();
		} */

		now = GetTimePoint();

		// Simple blinking of green LED to indicate activity
		static TimePoint lastPowerLEDToggle = 0;
		static bool powerLEDToggle = false;
		if (GetTimeSpanMS(lastPowerLEDToggle, now) > 500)
		{
			lastPowerLEDToggle = GetTimePoint();
			powerLEDToggle = !powerLEDToggle;
			if (powerLEDToggle)
				GPIO_SET(RJLED_GPIO_X, RJLED_GREEN_PIN);
			else
				GPIO_RESET(RJLED_GPIO_X, RJLED_GREEN_PIN);
		}

		// TODO: Inconsistent check once lastTimeCheck.ms lapses, will check every 100us within last 10ms, but will not happen normally
		now = GetTimePoint();
		static TimePoint lastTimeCheck = 0;
		if (GetTimeSpanMS(lastTimeCheck, now) < 10)
			continue;
		lastTimeCheck = now;
		// Check all kinds of timeouts >= 50 ms

#if defined(USE_UART)
		// Check identification send timeout
		static TimePoint lastIdent = 0;
		if (uartState == UART_None && 
			GetTimeSpanMS(startup, now) > UART_IDENT_STARTUP_DELAY_MS &&
			GetTimeSpanMS(state->resetTimer, now) > UART_RESET_TIMEOUT_MS &&
			(GetTimeSpanMS(lastIdent, now) > UART_IDENT_INTERVAL_MS || lastIdent == 0))
		{ // Send identification packet occasionally
			lastIdent = now;
			rgbled_transition(LED_CONNECTING, 0);
			ReturnToDefaultLEDState(UART_IDENT_INTERVAL_MS);
			uartd_send(0, ownIdentPacket, sizeof(ownIdentPacket), true);
		}

		// Check UART timeouts
		if (uartState == UART_CamMCU)
		{
			TimeSpan timeSinceLastComm = GetTimeSinceMS(state->lastComm);
			if (timeSinceLastComm > UART_COMM_TIMEOUT_MS || timeSinceLastComm < -1)
			{ // Reset Comm after silence
				EnterUARTPortZone(0); // Mostly for the debug
				uartd_reset_port_int(0);
				delayUS(10);
				uartd_nak_int(0);
				WARN_CHARR('/', 'T', 'M', 'O'); // TiMeOut
				LeaveUARTPortZone(0);
			}
		}
#endif

#if defined(USE_I2C)
		if (piIsBooted && GetTimeSpanMS(lastPiComm, now) > MCU_COMM_TIMEOUT_MS)
			piIsBooted = false;
		if (uartState == UART_CamPi && !piIsBooted)
		{ // Pi stopped corresponding, clear booted flag and take back UART control
			EnterUARTPortZone(0);
			uartState = UART_None;

			// Grab UART control from SBC, and notify SBC
			GPIO_RESET(UARTSEL_GPIO_X, UARTSEL_PIN);
			GPIO_RESET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
			GPIO_RESET(I2C_INT_GPIO_X, I2C_INT_PIN);
			delayUS(100);

			// Properly notify controller with a NAK
			uartd_reset_port_int(0);
			delayUS(10);
			uartd_nak_int(0);
			// TODO: Switching UART Control - Properly notify controller of switch
			//uartd_send_int(0, msg_sw_mcu, sizeof(msg_sw_mcu), true);

			LeaveUARTPortZone(0);
			ReturnToDefaultLEDState(100);
		}
		else if (uartState != UART_CamPi && piIsBooted)
		{ // Pi started corresponding, hand over UART control automatically
			EnterUARTPortZone(0);
			uartState = UART_CamPi;

			// Properly notify controller with a NAK, whether comms were ready or not
			uartd_reset_port_int(0);
			delayUS(10);
			uartd_nak_int(0);

			LeaveUARTPortZone(0);

			SafeDelayMS(1);

			// Pass UART control to SBC during reset timeout, and notify SBC
			GPIO_SET(UARTSEL_GPIO_X, UARTSEL_PIN);
			GPIO_SET(RJLED_GPIO_X, RJLED_ORANGE_PIN);
			GPIO_RESET(I2C_INT_GPIO_X, I2C_INT_PIN);

			ReturnToDefaultLEDState(100);
		}
#endif
	}
}


/* ------ UART Behaviour ------ */

uartd_respond uartd_handle_header(uint_fast8_t port)
{
	if (state->header.tag >= PACKET_MAX_ID_POSSIBLE)
		return uartd_ignore;

	if (uartState == UART_None)
	{
		if (state->header.tag == PACKET_IDENT)
		{ // Received identification
			if (state->header.length == IDENT_PACKET_SIZE)
			{ // Correct size, receive and check fully
				lastMarker = GetTimePoint();
				return uartd_accept;
			}
			ERR_STR("#IdentWrongSize:");
			return uartd_reset_nak;
		}
		else if (state->header.tag == PACKET_NAK)
		{
			WARN_STR("!IdentNAKed:"); 
			return uartd_reset;
		}
		else
		{ // Invalid packet before identification
			COMM_STR("!Unexpected:");
			COMM_CHARR(INT9_TO_CHARR(port), '+', UI8_TO_HEX_ARR(state->header.tag));
			return uartd_reset_nak;
		}
	}
	else if (uartState == UART_CamMCU)
	{
		if (state->header.tag == PACKET_PING)
		{ // Answer ping to notify of existence
			uartd_send_int(0, msg_ping, sizeof(msg_ping), true);
			lastMarker = GetTimePoint();
			return uartd_ignore;
		}
		else if (state->header.tag == PACKET_NAK)
		{
			WARN_CHARR('/', 'N', 'A', 'K');
			uartState = UART_None;
			return uartd_reset;
		}
		else if (state->header.tag == PACKET_IDENT)
		{ // Redundant identification
			WARN_CHARR('/', 'I', 'R', 'D');
			return uartd_reset_nak;
		}
		else if (state->header.tag == PACKET_SYNC)
		{ // Received sync packet
			return uartd_accept;
		}
		return uartd_unknown;
	}
	// else assert(uartState == UART_CamPi);

	if (state->header.tag == PACKET_PING)
	{ // Pi will answer ping to notify of existence
		lastMarker = GetTimePoint();
		return uartd_ignore;
	}
	else if (state->header.tag == PACKET_SYNC)
	{ // Received sync packet
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_SOF)
	{ // Received packet indicating an immediate Start of Frame
		lastMarker = GetTimePoint();

#if !defined(BOARD_OLD)
		GPIO_SET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
		lastFrameSync = GetTimePoint();
#endif

		return uartd_accept;
	}
	else if (state->header.tag == PACKET_CFG_MODE)
	{
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_CFG_FILTER)
	{
		return uartd_accept;
	}
	else
	{ // May have been intended for camera_pi
		return uartd_ignore;
	}
}

uartd_respond uartd_handle_data(uint_fast8_t port, uint8_t* ptr, uint_fast16_t size)
{
	// Copy packet (which may be split by UART receive buffer end) to temporary buffer
	// The MCU only ever receives small packets that are not addressed to Host
	if (state->header.length+PACKET_CHECKSUM_SIZE > UART_TEMP_PACKET_BUF || state->dataPos+size > UART_TEMP_PACKET_BUF)
		return uartd_ignore;	
	memcpy(receive.packetBuffer+state->dataPos, ptr, size);
	receive.packetSize = state->dataPos+size;
}

uartd_respond uartd_handle_packet(uint_fast8_t port, uint_fast16_t endPos)
{
	PortState *state = &portStates[port];

	// Verify direct checksum
	receive.packetSize -= PACKET_CHECKSUM_SIZE;
	uint8_t checksum[PACKET_CHECKSUM_SIZE];
	calculateDirectPacketChecksum(receive.packetBuffer, receive.packetSize, checksum);
	bool correctChecksum = true;
	for (int i = 0; i < PACKET_CHECKSUM_SIZE; i++)
	{
		if (checksum[i] != receive.packetBuffer[receive.packetSize+i])
		{
			WARN_STR("!PacketChecksum:");
			WARN_CHARR(INT9_TO_CHARR(port), '+', UI8_TO_HEX_ARR(state->header.tag), '+', INT999_TO_CHARR(state->header.length));
			correctChecksum = false;
			rgbled_transition(LED_UART_ERROR, 0);
			ReturnToDefaultLEDState(UART_RESET_TIMEOUT_MS);
			break;
		}
	}

	if (state->header.tag == PACKET_IDENT)
	{ // Identification
		if (uartState != UART_None)
		{ // Why are we here?
			return uartd_ignore;
		}
		if (!correctChecksum)
		{
			return uartd_reset_nak;
		}
		if (receive.packetSize < IDENT_PACKET_SIZE)
		{
			WARN_STR("!IdentSmall:");
			WARN_CHARR(INT9_TO_CHARR(port), '+', INT99_TO_CHARR(state->header.length), '+', INT99_TO_CHARR(receive.packetSize));
			return uartd_reset_nak;
		}
		// Received full identification, check
		struct IdentPacket ident = parseIdentPacket(receive.packetBuffer);
		if (ident.device != DEVICE_TRCONT || ident.type != INTERFACE_UART)
		{ // Ident packet was plain wrong
			WARN_STR("!IdentFail:");
			WARN_CHARR(INT9_TO_CHARR(port), '+', UI8_TO_HEX_ARR(ident.device), '+', UI8_TO_HEX_ARR(ident.type));
			return uartd_reset_nak;
		}
		if (ident.version.major != ownIdent.version.major)
		{ // TODO: Handle differing versions - ideally, try to connect anyway to allow for updating
			WARN_STR("!VersionMismatch:");
			WARN_CHARR(INT9_TO_CHARR(port));
			WARN_STR("+Cont:v");
			WARN_CHARR(INT99_TO_CHARR(ownIdent.version.major), '.', INT99_TO_CHARR(ownIdent.version.minor));
			WARN_STR("+Cam:v");
			WARN_CHARR(INT99_TO_CHARR(ident.version.major), '.', INT99_TO_CHARR(ident.version.minor));
		}
		// Idenfication verified
		uartState = UART_CamMCU;
		controllerIdentity = ident;
		COMM_CHARR('/', 'I', 'D', 'S');
		ReturnToDefaultLEDState(100);
		// Send acknowledgement
		uartd_ack_int(port);
		return uartd_accept;
	}

	if (!correctChecksum)
		return uartd_ignore;

	if (state->header.tag == PACKET_SYNC)
	{ // Received sync packet
		if (state->header.length < SYNC_PACKET_SIZE)
			return uartd_ignore;
		struct SyncPacket packet = parseSyncPacket(receive.packetBuffer);
		// Update time sync
		uint64_t controllerTimeUS = rebase_timestamp(&timesync, packet.timeUS, 1 << 24, (1 << 21)*3); // bias 75% newer, 25% older 
		lastTimeSync = update_time_synced(&timesync, controllerTimeUS, state->lastPacketTime);
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_SOF)
	{ // Received packet indicating an immediate Start of Frame
		if (state->header.length < SOF_PACKET_SIZE)
			return uartd_ignore;
		struct SOFPacket packet = parseSOFPacket(receive.packetBuffer);
		// Update time sync
		uint64_t controllerTimeUS = rebase_timestamp(&timesync, packet.timeUS, 1 << 24, (1 << 21)*3); // bias 75% newer, 25% older 
		lastTimeSync = update_time_synced(&timesync, controllerTimeUS, state->lastPacketTime / TICKS_PER_US);
		// Record ID and time of last frame
		if (packet.frameID != (lastFrameID+1)%256)
			WARN_CHARR('/', 'S', 'F', 'S', INT99_TO_CHARR(lastFrameID), ':', INT99_TO_CHARR(packet.frameID));
		lastFrameID = packet.frameID;
		lastFrameTime = lastTimeSync;
		// Tell SBC to fetch status packet with new frame ID
		GPIO_RESET(I2C_INT_GPIO_X, I2C_INT_PIN);
		// Reset FSIN after desired pulse width
#if !defined(BOARD_OLD)
		TimePoint frameSyncEnd = lastFrameSync + FSIN_PULSE_WIDTH_US * TICKS_PER_US;
		while (GetTimePoint() < frameSyncEnd);
		GPIO_RESET(FSIN_GPIO_X, CAMERA_FSIN_PIN);
#endif
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_CFG_MODE)
	{
		if (receive.packetSize >= 1)
		{
			uint8_t modePacket = receive.packetBuffer[0];
			piIsStreaming = modePacket&TRCAM_FLAG_STREAMING;
			ReturnToDefaultLEDState(500);
			// TODO: Get this update from camera_pi directly instead over I2C?
			// E.g. Errors from camera_pi cannot be read here
		}
		return uartd_accept;
	}
	else if (state->header.tag == PACKET_CFG_FILTER)
	{
		if (receive.packetSize >= 1)
		{
			uint8_t filterState = (enum FilterSwitchCommand)receive.packetBuffer[0];
			if (filterState == FILTER_SWITCH_VISIBLE || filterState == FILTER_SWITCH_INFRARED)
				UpdateFilterSwitcher(filterState);
		}
		return uartd_accept;
	}

	// Shouldn't happen
	return uartd_unknown;
}

void uartd_handle_reset(uint_fast8_t port)
{
	rgbled_transition(LED_UART_ERROR, 0);
	ReturnToDefaultLEDState(UART_RESET_TIMEOUT_MS);
}


/* ------ I2C Behaviour ------ */

bool i2cd_handle_command(enum CameraMCUCommand command, uint8_t *data, uint8_t len)
{
	if (command == MCU_BOOT_FIRST)
	{
		// TODO: Notify user that Pi is initialising and may take longer to boot
		return true;
	}
	else if (command == MCU_BOOT)
	{
		// TODO: If Pi doesn't connect shortly after, notify user
		// May indicate program crashes or otherwise can't connect via I2C
		return true;
	}

	lastPiComm = GetTimePoint();
	piIsBooted = true;

	switch (command)
	{
		case MCU_SWITCH_BOOTLOADER:
			piWantsBootloader = true;
			rgbled_transition(LED_BOOTLOADER, 0);
			return true;
		case MCU_PING:
			// Nothing to do
			return true;
		case MCU_UPDATE_ID:
		{
			if (len != sizeof(CameraID))
			{
				rgbled_transition(LED_ERROR_1, 0);
				ReturnToDefaultLEDState(1000);
				return false;
			}
			CameraID id;
			memcpy(&id, data, sizeof(CameraID));
			if (id == 0)
			{
				rgbled_transition(LED_ERROR_2, 0);
				ReturnToDefaultLEDState(1000);
				return true;
			}
			else if (PERSISTENT_CONFIG[0] == id)
			{
				rgbled_transition(LED_STANDBY, 0);
				ReturnToDefaultLEDState(500);
			}
			else
			{ // Tell main thread to write to flash
				cameraIDToWrite = id;
			}
			return true;
		}
		default:
			return false;
	}
}


uint8_t i2cd_prepare_response(enum CameraMCUCommand command, uint8_t *data, uint8_t len, uint8_t response[256])
{
	lastPiComm = GetTimePoint();
	piIsBooted = true;

	switch (command)
	{
		case MCU_REG_ID:
			response[0] = MCU_I2C_ID;
			return 1;
		case MCU_GET_STATUS:
		{
			// Reset interrupt flag
			GPIO_SET(I2C_INT_GPIO_X, I2C_INT_PIN);
			// 16bits of various states
			uint16_t states = ((filterSwitcherState & 0b11) << 14);
			response[0] = states >> 8;
			response[1] = states & 0xFF;
			// Latest supply voltage reading from ADC 
			uint16_t powerMV = GetMillivolts();
			response[2] = powerMV >> 8;
			response[3] = powerMV & 0xFF;
			// Timestamp for timesync
			TimePoint timestamp = GetTimePoint();
			uint16_t timestampUS = (timestamp / TICKS_PER_US) & 0xFFFF;
			response[4] = timestampUS >> 8;
			response[5] = timestampUS & 0xFF;
			// Last frame info for framesync (using timestamp as reference to more easily encode when frametime was too long ago)
			response[6] = 0; // last frame ID
			TimeSpan usSinceFrame = 0; // time since frame sync
			response[7] = usSinceFrame >> 8;
			response[8] = usSinceFrame & 0xFF;
			// State of queued packets from controller intended for SBC
			response[9] = 0; // Queue size
			uint16_t packetSize = 0; // might not need 16bit, but to be safe
			response[10] = packetSize >> 8;
			response[11] = packetSize & 0xFF;
			return MCU_STATUS_LENGTH;
		}
		case MCU_FETCH_INFO:
		{
			otp_read();

			// This is a critical packet, so make sure misinterpretations can not happen
			uint8_t Packet_Version = 1;
			if (len > 0 && data[0] != 0 && data[0] < Packet_Version)
				Packet_Version = data[0];
			// TODO: KEEP BACKWARDS COMPATIBLE TO LOWER Packet_Version requested by SBC!!

			// Write packet header
			response[0] = Packet_Version;		// Fetch Info packet version
			response[1] = OTP_Version; 			// OTP Format Version
			response[2] = OTP_NumSubParts; 		// Sub-Part Count
			response[3] = 0; 					// Reserved
			
			// Write string lengths for separate request
			response[4] = OTP_HwStringLength >> 8;
			response[5] = OTP_HwStringLength & 0xFF;
			response[6] = firmwareDescriptorLength >> 8;
			response[7] = firmwareDescriptorLength & 0xFF;

			uint32_t *packet32 = ((uint32_t*)response);
			packet32[2] = PERSISTENT_CONFIG[0];	// 32-Bit Camera ID
			packet32[3] = firmwareVersion.num;	// MCU Firmware Version
			packet32[4] = OTP[2];				// 64-Bit Camera Serial Number Word 1
			packet32[5] = OTP[3];				// 64-Bit Camera Serial Number Word 2
			packet32[6] = LL_GetUID_Word0();	// 96-Bit Unique MCU ID Word 1
			packet32[7] = LL_GetUID_Word1();	// 96-Bit Unique MCU ID Word 2
			packet32[8] = LL_GetUID_Word2();	// 96-Bit Unique MCU ID Word 3

			static_assert(MCU_INFO_MAX_LENGTH == (9 * sizeof(uint32_t)));
			return MCU_INFO_MAX_LENGTH;
		}
		case MCU_GET_HW_STR:
		{
			// We assume OTP has already been read - otherwise, the Pi would not know the length to read
			response[0] = OTP_HwStringLength >> 8;
			response[1] = OTP_HwStringLength & 0xFF;
			memcpy(response+2, OTP_HwStringData, OTP_HwStringLength);
			return 2 + OTP_HwStringLength;
		}
		case MCU_GET_FW_STR:
		{
			response[0] = firmwareDescriptorLength >> 8;
			response[1] = firmwareDescriptorLength & 0xFF;
			memcpy(response+2, firmwareDescriptor, firmwareDescriptorLength);
			return 2 + firmwareDescriptorLength;
		}
		case MCU_GET_PARTS:
		{
			response[0] = 0;
			response[1] = otp_get_subparts((uint32_t*)(response+2), OTP_NumSubParts);
			return 2 + response[1]*sizeof(uint64_t);
		}
		default:
			// TODO: Error?
			return 0;
	}
}


/* ------ Functional Behaviour ------ */

void ReturnToDefaultLEDState(int timeMS)
{
	if (piIsStreaming) // Implies piHasPower && piIsBooted && uartState == UART_CamPi
		rgbled_animation(&LED_ANIM_STREAMING);
	else if (uartState == UART_CamPi) // Implies piHasPower && piIsBooted
		rgbled_transition(LED_ACTIVE, timeMS);
	else if (uartState == UART_CamMCU && piHasPower)
		rgbled_animation(&LED_ANIM_BOOTING);
	else if (uartState == UART_CamMCU)
		rgbled_transition(LED_STANDBY, timeMS);
	else
		rgbled_transition(LED_INITIALISING, timeMS);
}

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

static bool ApplyUserFlashConfiguration()
{
	enum CameraMCUFlashConfig config = InterpretUserFlashConfiguration();
	if (config == MCU_FLASH_USER_ABORTED)
	{
		rgbled_animation(&LED_ANIM_FLASH_BAD);
		while (!GPIO_READ(BUTTONS_GPIO_X, BUTTON_BOTTOM_PIN) || !GPIO_READ(BUTTONS_GPIO_X, BUTTON_TOP_PIN));
	}
	else if (config != MCU_FLASH_KEEP)
	{
		enum CameraMCUFlashConfig status = SetFlashConfiguration(config);
		if (status != MCU_FLASH_KEEP)
		{ // Option Bytes changed, gotta restart
			if (status == MCU_FLASH_ERROR)
				rgbled_animation(&LED_ANIM_FLASH_BAD);
			else if (status == MCU_FLASH_BOOT0_PI)
				rgbled_animation(&LED_ANIM_FLASH_SWITCH_BOOT0_PI);
			else if (status == MCU_FLASH_DEBUG_SWD)
				rgbled_animation(&LED_ANIM_FLASH_SWITCH_DEBUG_SWD);
			delayMS(2000);

			// Normal restart, but this won't actually reload option bytes for next boot
			//SCB->AIRCR = (0x05FA << SCB_AIRCR_VECTKEY_Pos) | (1 << SCB_AIRCR_SYSRESETREQ_Pos);

			// Restart and load option bytes from flash back to register
			FLASH->CR = FLASH_CR_OBL_LAUNCH;
		}
		mcuFlashConfig = config;
		return true;
	}
	return false;
}