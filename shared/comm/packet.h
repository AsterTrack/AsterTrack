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

#ifndef PACKET_H
#define PACKET_H

/**
 * This file describes the common packet interface and is shared between all projects
 * Thus it needs to be kept in sync and working with all compilers (C style)!
*/

#include <stdint.h>
#include <stdbool.h>

enum TrCamMode
{
	TRCAM_STANDBY					= 0b00000000,
	TRCAM_FLAG_STREAMING			= 0b10000000,
	TRCAM_MODE_MASK					= 0b01111111,

	TRCAM_MODE_SIM					= 0b00010000,
	TRCAM_OPT_SIM_MSK				= 0b00001111,
	TRCAM_OPT_SIM_LVL				= 0b00000111,
	TRCAM_OPT_SIM_PROC				= 0b00001000,
	TRCAM_MODE_BLOB					= 0b00001000,
	TRCAM_OPT_BLOB_MSK				= 0b00000111,
	TRCAM_MODE_BGCALIB				= 0b00000100,
	TRCAM_OPT_BGCALIB_MSK			= 0b00000011,
	TRCAM_OPT_BGCALIB_ACCEPT		= 0b00000010,	
	TRCAM_OPT_BGCALIB_RESET			= 0b00000001,	
	TRCAM_MODE_VISUAL				= 0b00000010,
	TRCAM_OPT_VISUAL_MSK			= 0b00000001,
	TRCAM_MODE_IDLE					= 0, // Last bit set or none
	TRCAM_OPT_NONE					= 0,

	TRCAM_MODE_SIZE					= 0b11111111
};

enum PacketTag
{
	// All below are for (direct) Cam<->Controller comm
	PACKET_ACK = 1,			// Acknowledge identification
	PACKET_NAK,				// Interrupt all communication
	PACKET_IDENT,			// Identification packet with version
	PACKET_PING,			// NOP packet that must be returned
	PACKET_SYNC,			// Timestamped packet for TimeSync
	PACKET_SOF,				// Timestamped packet signaling SOF for frameID
	PACKET_RATE_CONFIG,		// Request to switch baudrate
	PACKET_RATE_VERIFY,		// Verification data with checksums to test baudrate

	// All below are for (forwarded) Cam<->Host comm
	PACKET_HOST_COMM = 24,
	PACKET_CFG_SETUP = 24,	// Set base camera configuration
	PACKET_CFG_MODE,		// Set mode/status configuration - response PACKET_CONFIG, toggles
	PACKET_CFG_WIFI,		// Set wireless configuration & state - response PACKET_WIRELESS
	PACKET_CFG_IMAGE,		// Set streaming configuration - enables/disables PACKET_IMAGE
	PACKET_CFG_VIS,			// Set HDMI visualisation configuration
	PACKET_MODE,			// Report mode change status (response to PACKET_CFG_MODE)
	PACKET_ERROR,			// Report error (failed configuration change or runtime error)
	PACKET_STAT,			// Report streaming statistics
	PACKET_WIRELESS,		// Report status of wireless connection
	PACKET_FRAME_SIGNAL,	// Signal acceptance of frame
	PACKET_BLOB,			// Send blob data of frame
	PACKET_VISUAL,			// Send visual debug data of frame
	PACKET_IMAGE,			// Send camera frame image
	PACKET_BGTILES,			// Send updated background tiles
	PACKET_CFG_FILTER,		// Configure filter switcher state
	PACKET_CFG_SIGNAL,		// Configure state for LED to signal
	PACKET_MAX_ID_POSSIBLE = 63
};

enum DeviceTag
{
	DEVICE_SERVER					= 1<<2,
	DEVICE_TRCAM					= 1<<4,
	DEVICE_TRCAM_MCU				= 1<<5,
	DEVICE_TRCONT					= 1<<6,
};

enum InterfaceTag
{
	INTERFACE_UART					= 0b10101010,
	INTERFACE_SERVER				= 0b01010101
};

enum SignalTag
{
	SIGNAL_INVALID = 0,
	SIGNAL_DEBUG,
	SIGNAL_EVENT,
	SIGNAL_SOF,
	SIGNAL_MAX = 0b1111
};

enum USBCommand
{
	COMMAND_OUT_BASE = 0,
	COMMAND_OUT_TIME_SYNC,
	COMMAND_OUT_SYNC_RESET,
	COMMAND_OUT_SYNC_EXTERNAL,
	COMMAND_OUT_SYNC_GENERATE,
	COMMAND_OUT_SYNC_MASK,
	COMMAND_OUT_SEND_PACKET,
	COMMAND_OUT_TEST,
	COMMAND_OUT_EVENTS,

	// Requests
	COMMAND_IN_BASE = 0,
	COMMAND_IN_STATUS,
	COMMAND_IN_DEBUG,
	COMMAND_IN_EVENTS,
	COMMAND_IN_PACKETS
};

enum ControllerEventID
{
	// Low-level Interrupts
	CONTROLLER_INTERRUPT_USB,
	CONTROLLER_INTERRUPT_UART,
	CONTROLLER_INTERRUPT_SYNC_GEN,
	CONTROLLER_INTERRUPT_SYNC_INPUT,
	CONTROLLER_INTERRUPT_LED_UPDATE,
	CONTROLLER_INTERRUPT_PD_INT,
	CONTROLLER_INTERRUPT_FLASH_BUTTON,
	CONTROLLER_INTERRUPT_FLASH_TIMER,

	// Low-Level USB Events
	CONTROLLER_EVENT_USB_SOF,
	CONTROLLER_EVENT_USB_CONTROL,
	CONTROLLER_EVENT_USB_DATA_TX,

	// High-Level USB Sending Events
	CONTROLLER_EVENT_USB_SENDING_NULL,
	CONTROLLER_EVENT_USB_SENDING_PACKET,
	CONTROLLER_EVENT_USB_QUEUE_SOF,

	// High-Level Streaming Events
	CONTROLLER_EVENT_SYNC,
	CONTROLLER_EVENT_DATA_IN,
	CONTROLLER_EVENT_DATA_OUT,

	CONTROLLER_EVENT_MAX
};

enum ErrorTag
{
	ERROR_NONE = 0,
	ERROR_UNKNOWN,
	ERROR_GCS_NO_I2C,
	ERROR_GCS_NO_SENSOR,
	ERROR_GCS_CREATE,
	ERROR_INIT_BD,
	ERROR_QPU_ENABLE,
	ERROR_GCS_START,
	ERROR_GCS_TIMEOUT,
	ERROR_GCS_RETURN,
	ERROR_GCS_REQUEST,
	ERROR_MEM_ACCESS,
	ERROR_QPU_STALL_CPY,
	ERROR_QPU_STALL_MSK,
	ERROR_EXCEPTION_UNKNOWN,
	ERROR_EXCEPTION_ILLEGAL,
	ERROR_EXCEPTION_BUS,
	ERROR_EXCEPTION_FPE,
	ERROR_EXCEPTION_SEGFAULT,
	ERROR_EXCEPTION_PIPE,
	ERROR_MAX
};

static const char *ErrorTag_String[ERROR_MAX] = 
{
	"No Error",
	"Unknown Error",
	"Failed to find camera I2C - system broken",
	"Failed to find camera sensor - check connection",
	"Camera subsystem failed to start",
	"Error during Blob Detection initialisation",
	"Failed to enable QPU",
	"Failed to start GCS",
	"No camera frames (missing sync?)",
	"Failed to return image frame",
	"Failed to request image frame",
	"Failed to access image frame",
	"QPU stalled during copying",
	"QPU stalled during mask thresholding",
	"Unknown camera program exception",
	"Camera program exception (illegal instruction)",
	"Camera program exception (bus error)",
	"Camera program exception (floating point error)",
	"Camera program exception (segfault)",
	"Camera program exception (pipe)"
};

enum ControllerCommState {
	CommNoCon = 0,
	CommID = 1,
	CommACK = 2,
	CommMCU = 4,
	CommPi = 8,
	CommReady = CommID | CommACK,
	CommMCUReady = CommMCU | CommReady,
	CommPiReady = CommPi | CommReady,
};

enum FilterSwitchCommand {
	FILTER_KEEP = 0,
	FILTER_SWITCH_VISIBLE = 1,
	FILTER_SWITCH_INFRARED = 2,
};

enum CameraMCUFlashConfig {
	MCU_FLASH_KEEP = 0,
	MCU_FLASH_UNKNOWN = 0,
	MCU_FLASH_BOOT0_PI = 1,
	MCU_FLASH_DEBUG_SWD = 2,
	MCU_FLASH_ERROR = 10,
	MCU_FLASH_USER_ABORTED = 10,
};


/**
 * Header for full packets (for controller-camera and server-camera communication)
 */
struct PacketHeader
{
	enum PacketTag tag; // : 6
	uint8_t frameID; // : 8
	uint32_t length; // : 18

#ifdef __cplusplus
	PacketHeader(){}
	PacketHeader(enum PacketTag Tag, uint32_t Length, uint8_t FrameID = 0) : tag(Tag), frameID(FrameID), length(Length) {}

	inline bool isStreamPacket() const { return tag == PACKET_BLOB; }
#endif
};
#ifndef __cplusplus
static inline bool isStreamPacket(struct PacketHeader header) { return header.tag == PACKET_BLOB; }
#endif
#define PACKET_HEADER_SIZE 4

static inline struct PacketHeader parsePacketHeader(uint8_t data[PACKET_HEADER_SIZE])
{
	struct PacketHeader header;
	header.tag = (enum PacketTag)((data[0] >> 2) & 0x3F);
	header.frameID = ((data[0] & 0x3) << 6) | ((data[1] >> 2) & 0x3F);
	header.length = ((data[1] & 0x3) << 16) | (data[2] << 8)  | data[3];
	return header;
};

static inline void storePacketHeader(const struct PacketHeader header, uint8_t data[PACKET_HEADER_SIZE])
{
	data[0] = ((header.tag & 0x3F) << 2) | ((header.frameID & 0xC0) >> 6);
	data[1] = ((header.frameID & 0x3F) << 2) | ((header.length & 0x30000) >> 16);
	data[2] = ((header.length & 0x0FF00) >> 8);
	data[3] = ((header.length & 0x000FF));
};

/**
 * Checksum used for packets, appended at the end
 */
#define CHECKSUM_SIZE 				1
#if CHECKSUM_SIZE == 1
typedef uint8_t checksum_t;
#elif CHECKSUM_SIZE == 2
typedef uint16_t checksum_t;
#elif CHECKSUM_SIZE == 4
typedef uint32_t checksum_t;
#elif CHECKSUM_SIZE == 8
typedef uint64_t checksum_t;
#else
#error Invalid checksum size!
#endif

/**
 * Header for blocks within USB packets to assign them to the correct port (camera)
 * Signals are from controller
 * Normal blocks are (part of) a packet from ports (cameras)
 */
struct BlockHeader
{
	uint8_t blockID; // : 8;
	bool isSignal; // blockID == 255
	// If block:
	bool isFirstPacketBlock; // : 1;
	uint8_t portNr; // : 3;
	// If signal:
	enum SignalTag signal; // : 4;
	// Payload:
	uint8_t skip; // : 2;
	uint16_t size; // : 10;
};
#define BLOCK_HEADER_SIZE 			3

// Special blockID to designate signals (not from ports, but from controller itself)
#define BLOCK_ID_SIGNAL				255

static inline struct BlockHeader parseBlockHeader(uint8_t data[BLOCK_HEADER_SIZE])
{
	struct BlockHeader header;
	header.blockID = data[0];
	header.isSignal = header.blockID == BLOCK_ID_SIGNAL;
	if (header.isSignal)
	{
		header.signal = (enum SignalTag)((data[1] >> 4) & 0b1111);
	}
	else
	{
		header.isFirstPacketBlock = data[1] >> 7;
		header.portNr = (data[1] >> 4) & 0b111;
	}
	header.skip = ((data[1] >> 2) & 0b11);
	header.size = (data[1] & 0b11) << 8 | data[2];
	return header;
};

static inline void storeBlockHeader(const struct BlockHeader header, uint8_t data[BLOCK_HEADER_SIZE])
{
	if (header.isSignal)
	{
		data[0] = BLOCK_ID_SIGNAL;
		data[1] = ((header.signal&0b1111) << 4) | ((header.skip&0b11) << 2) | ((header.size >> 8)&0b11);
		data[2] = header.size&0xFF;
	}
	else
	{
		//assert(header.blockID != BLOCK_ID_SIGNAL);
		data[0] = header.blockID;
		data[1] = ((header.isFirstPacketBlock&0b1) << 7) | ((header.portNr&0b111) << 4) | ((header.skip&0b11) << 2) | ((header.size >> 8)&0b11);
		data[2] = header.size&0xFF;
	}
};


/**
 * Header for USB packets (containing multiple blocks of packets and signals)
 * Contains controller timestamp of last packet (counter-1), so that host can use them for timesync
 * Current packet timestamps COULD be sent, but are not as reliable as the last packets timestamp since unexpected stalls do happen
 */

struct USBPacketHeader
{
	uint8_t counter; // : 8; - Running counter to detect missing packets per sink
	uint32_t lastTimestamp; // : 24; - Enough to keep 16s in us units
};
#define USB_PACKET_HEADER 			4

static inline struct USBPacketHeader parseUSBPacketHeader(uint8_t data[USB_PACKET_HEADER])
{
	struct USBPacketHeader header;
	header.counter = data[0];
	header.lastTimestamp = (data[1]<<16) | (data[2]<<8) | data[3];
	return header;
};
static inline void storeUSBPacketHeader(struct USBPacketHeader header, uint8_t data[USB_PACKET_HEADER])
{
	data[0] = header.counter;
	data[1] = (header.lastTimestamp >> 16) & 0xFF;
	data[2] = (header.lastTimestamp >> 8) & 0xFF;
	data[3] = (header.lastTimestamp >> 0) & 0xFF;
};

#define USB_PACKET_SIZE				1024	// USB 2.0 HS Max packet size for Interrupt/Isochronous transfers
// USB transfer buffers might need to be aligned
#define USB_PACKET_ALIGNMENT 		(4-1)	// Bytes to allow start address to be 4-byte alignment (e.g. for HW DMA)
// Packet size that can be sent to host optimally without needless fragmentation
#define OPT_PACKET_SIZE				(USB_PACKET_SIZE - (USB_PACKET_HEADER + BLOCK_HEADER_SIZE + USB_PACKET_ALIGNMENT + PACKET_HEADER_SIZE + CHECKSUM_SIZE))


/**
 * Simple hash utility
 */
static const uint32_t val_const = 0x9ab4fb1e;
static const uint32_t prime_const = 0x7e9a3c6d;
static inline uint32_t HashStrConst(const char* const str, const uint32_t value)
{
	return !*str? value : HashStrConst(str+1, (value ^ (uint32_t)(*str)) * prime_const);
}


/**
 * Version Descriptor for each device
 */
union VersionDesc
{
	struct {
		uint8_t major;
		uint8_t minor;
		uint8_t patch;
		uint8_t build;
	};
	uint32_t num;

#ifdef __cplusplus
	VersionDesc() {}
	VersionDesc(uint8_t verMajor, uint8_t verMinor, uint8_t verPatch) : major(verMajor), minor(verMinor), patch(verPatch)
	{
		build = 0; //(uint8_t)(HashStrConst(__DATE__ __TIME__, val_const)%256);
	}
#endif
};

#ifndef __cplusplus
static inline union VersionDesc GetVersion(uint8_t verMajor, uint8_t verMinor, uint8_t verPatch)
{
	union VersionDesc version;
	version.major = verMajor;
	version.minor = verMinor;
	version.patch = verPatch;
	version.build = 0; //(uint8_t)(HashStrConst(__DATE__ __TIME__, val_const)%256);
	return version;
}
#endif


/**
 * Identification packet used for inter-device communication
 */
struct IdentPacket
{
	enum DeviceTag device; // What type of device/interface
	enum InterfaceTag type; // What type of device/interface
	int id; // Device ID (only relevant for camera rn)
	union VersionDesc version;

#ifdef __cplusplus
	IdentPacket() {}
	IdentPacket(enum DeviceTag Device, enum InterfaceTag Type, union VersionDesc Version) : device(Device), type(Type), id(0), version(Version) {}
	IdentPacket(int id, enum DeviceTag Device, enum InterfaceTag Type, union VersionDesc Version) : device(Device), type(Type), id(id), version(Version) {}
#endif
};

#define IDENT_PACKET_SIZE			10

static inline struct IdentPacket parseIdentPacket(uint8_t data[IDENT_PACKET_SIZE])
{
	struct IdentPacket ident;
	ident.device = (enum DeviceTag)data[0];
	ident.type = (enum InterfaceTag)data[1];
	ident.id = (data[2] << 24) | (data[3] << 16) | (data[4] << 8) | (data[5]);
	ident.version.major = data[6];
	ident.version.minor = data[7];
	ident.version.patch = data[8];
	ident.version.build = data[9];
	return ident;
};

static inline void storeIdentPacket(struct IdentPacket ident, uint8_t data[IDENT_PACKET_SIZE])
{
	data[0] = ident.device;
	data[1] = ident.type;
	data[2] = (ident.id >> 24) & 0xFF;
	data[3] = (ident.id >> 16) & 0xFF;
	data[4] = (ident.id >> 8) & 0xFF;
	data[5] = (ident.id >> 0) & 0xFF;
	data[6] = ident.version.major;
	data[7] = ident.version.minor;
	data[8] = ident.version.patch;
	data[9] = ident.version.build;
};


/**
 * Sync packet for controller-camera timesync
 */
struct SyncPacket
{
	uint32_t timeUS;

#ifdef __cplusplus
	SyncPacket() {}
	SyncPacket(uint32_t SOFus) : timeUS(SOFus) {}
#endif
};
#define SYNC_PACKET_SIZE			3

static inline struct SyncPacket parseSyncPacket(uint8_t data[SYNC_PACKET_SIZE])
{
	struct SyncPacket sync;
	sync.timeUS = (data[0] << 16) | (data[1] << 8) | (data[2]);
	return sync;
};

static inline void storeSyncPacket(struct SyncPacket sync, uint8_t data[SYNC_PACKET_SIZE])
{
	data[0] = (sync.timeUS >> 16) & 0xFF;
	data[1] = (sync.timeUS >> 8) & 0xFF;
	data[2] = (sync.timeUS >> 0) & 0xFF;
};


/**
 * SOF (Start Of Frame) packet for controller-camera and controller-server broadcast of new frame information
 */
struct SOFPacket
{
	uint32_t frameID;
	uint32_t timeUS;

#ifdef __cplusplus
	SOFPacket() {}
	SOFPacket(uint32_t FrameID, uint32_t SOFus) : frameID(FrameID), timeUS(SOFus) {}
#endif
};
#define SOF_PACKET_SIZE				7

static inline struct SOFPacket parseSOFPacket(uint8_t data[SOF_PACKET_SIZE])
{
	struct SOFPacket sof;
	sof.timeUS = (data[0] << 16) | (data[1] << 8) | (data[2]);
	sof.frameID = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | (data[6]);
	return sof;
};

static inline void storeSOFPacket(struct SOFPacket sof, uint8_t data[SOF_PACKET_SIZE])
{
	data[0] = (sof.timeUS >> 16) & 0xFF;
	data[1] = (sof.timeUS >> 8) & 0xFF;
	data[2] = (sof.timeUS >> 0) & 0xFF;
	data[3] = (sof.frameID >> 24) & 0xFF;
	data[4] = (sof.frameID >> 16) & 0xFF;
	data[5] = (sof.frameID >> 8) & 0xFF;
	data[6] = (sof.frameID >> 0) & 0xFF;
};

#endif // PACKET_H