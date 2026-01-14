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

#include "mcu.hpp"
#include "comm/timesync.hpp"
#include "comm/commands.h"
#include "stm32_bootloader.hpp"
#include "version.hpp"

#include "util/util.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <errno.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <fstream>

#include <gpiod.h>

#include <sys/prctl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

const char *i2c_port = "/dev/i2c-1";
unsigned int i2c_fd = -1;

const char *gpio_chipname = "/dev/gpiochip0";
unsigned int PIN_BOOT0 = 27;
unsigned int PIN_NRST = 18;
unsigned int PIN_INT = 17;

gpiod_chip *gpio_chip;
gpiod_request_config *req_config;
gpiod_line_config *line_config_out, *line_config_in;
gpiod_line_settings *line_boot0, *line_reset, *line_int;
gpiod_line_request *line_request_out, *line_request_in;
gpiod_edge_event_buffer *edge_events;

std::string mcu_flash_file = "/mnt/mmcblk0p2/tce/TrackingCameraMCU.bin";

std::mutex mcu_mutex;
std::atomic<bool> stop_thread;
std::thread *mcu_comm_thread;
TimePoint_t lastPing;

TimeSync timesync;

volatile bool mcu_exists;
volatile bool mcu_active;
volatile bool mcu_intentional_bootloader;


static bool i2c_init();
static bool i2c_probe();
static void i2c_cleanup();

static bool gpio_init();
static void gpio_cleanup();

static bool mcu_send_ping();

static void mcu_thread();


bool mcu_initial_connect()
{
	if (!mcu_init())
		return false;
	
	std::unique_lock lock(mcu_mutex);
	if (mcu_probe())
	{ // MCU responded
		return true;
	}

	// Perhaps MCU is in bootloader mode or an invalid state, reset it and attempt to contact it again
	mcu_reconnect();

	if (!mcu_active)
	{ // MCU is still not responding, it is either not available in hardware, or bricked
		if (!mcu_probe_bootloader() && !mcu_switch_bootloader())
		{ // Could not reboot it into bootloader even via GPIO - likely not available in hardware
			printf("Could not find MCU!\n");
		}
		else if (!mcu_verify_program(mcu_flash_file))
		{ // MCU firmware is just bricked, and we have a differing firmware to try
			printf("MCU is bricked with differing firmware, will re-flash!\n");
			if (mcu_flash_program(mcu_flash_file))
				printf("Successfully re-flashed MCU in attempt to recover it!\n");
			mcu_reconnect();
		}
		else
		{ // MCU firmware is likely bricked (did not connect after reset)
			printf("MCU is bricked, but no differing firmware is available to attempt a recovery with!\n");
			mcu_reconnect();
		}
	}

	return mcu_active;
}

bool mcu_init()
{
	mcu_active = false;
	mcu_exists = false;
	bool i2c = i2c_init();
	bool gpio = gpio_init();
	return i2c && gpio;
}

bool mcu_probe()
{
	if (i2c_fd < 0) return false;

	if (i2c_probe())
	{
		if (!mcu_exists)
		{
			printf("Verified existance of MCU!\n");
			mcu_exists = true;
			mcu_monitor();
		}
		if (!mcu_active)
		{
			printf("Connected with MCU!\n");
			mcu_active = true;
			mcu_sync_info();
		}
	}
	else mcu_active = false;
	return mcu_active;
}

bool mcu_reconnect()
{
	// Whether we reflashed it or not, reset it into normal mode and try again
	mcu_reset();
	if (mcu_probe())
		return true;
	printf("Failed to reconnect to the MCU!\n");
	if (mcu_probe_bootloader())
		printf("MCU is still in the bootloader!\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	mcu_reset();
	i2c_init();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	if (mcu_probe())
		return true;
	printf("Still can't reconnect to the MCU after further reset!\n");
	return false;
}

void mcu_monitor()
{
	lastPing = sclock::now();
	if (!mcu_comm_thread)
	{
		stop_thread = false;
		mcu_comm_thread = new std::thread(mcu_thread);
	}
}

void mcu_cleanup()
{
	stop_thread = true;
	if (mcu_comm_thread && mcu_comm_thread->joinable())
		mcu_comm_thread->join();
	delete mcu_comm_thread;
	mcu_comm_thread = nullptr;

	if (gpio_chip)
		gpio_cleanup();

	if (i2c_fd >= 0)
		i2c_cleanup();
}

static bool handle_i2c_error()
{
	mcu_active = false;
	if (errno == EBADF)
	{
		i2c_init();
	}
	return false;
}

static void mcu_thread()
{
	prctl(PR_SET_NAME, "MCU_Monitor");

	while (!stop_thread.load())
	{
		if (!mcu_active)
		{
			if (dtMS(lastPing, sclock::now()) > MCU_PROBE_INTERVAL_MS && !mcu_intentional_bootloader)
			{
				std::unique_lock lock(mcu_mutex);
				if (mcu_active || mcu_intentional_bootloader)
					continue;
				lastPing = sclock::now();
				mcu_active = i2c_probe();
				if (mcu_active)
				{ // Reconnected, exchange info again (might have changed)
					printf("Reconnected with MCU!\n");
					mcu_sync_info();
				}
				else if (mcu_probe_bootloader())
				{ // Since we locked the mutex, no firmware update is ongoing - reset to exit bootloader?
					printf("MCU is in the bootloader!\n");
					mcu_reset();
				}
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			continue;
		}

		// Read interrupt line directly, just in case we missed it
		int events = 0;
		gpiod_line_value value_int[1];
		if (gpiod_line_request_get_values(line_request_in, value_int))
			printf("Failed to get interrupt GPIO pin state! %d: %s\n", errno, strerror(errno));
		else if (value_int[0] == GPIOD_LINE_VALUE_INACTIVE)
			events = 1; // Interrupt line is pulled low

		if (events == 0)
		{ // Wait for interrupt line to be pulled low
			events = gpiod_line_request_wait_edge_events(line_request_in, 50*1000*1000);
			if (events < 0)
			{
				printf("Failed to detect interrupts from MCU!\n");
				continue;
			}
			else if (events > 0)
			{ // Detected edge events, clear event buffer
				events = gpiod_line_request_read_edge_events(line_request_in, edge_events, 16);
				if (events < 0) printf("Failed to read interrupts from MCU!\n");
				//else printf("Detected %d interrupt events!\n", events);
			}
		}
		if (!mcu_active)
		{ // MCU disconnected while in the loop
			continue;
		}
		
		if (events > 0)
		{ // Interrupt line signals events available
			std::unique_lock lock(mcu_mutex);
			if (!mcu_active) continue;
			mcu_get_status();
			lastPing = sclock::now();
		}

		if (events <= 0 && dtMS(lastPing, sclock::now()) > MCU_PING_INTERVAL_MS)
		{
			std::unique_lock lock(mcu_mutex);
			if (!mcu_active) continue;
			mcu_send_ping();
			lastPing = sclock::now();
		}
	}
	printf("Stopping MCU monitor!\n");
}

void mcu_reset()
{
	if (!gpio_chip) return;

	printf("Resetting MCU...\n");

	mcu_active = false;
	mcu_intentional_bootloader = false;

	// BOOT0, RESET
	gpiod_line_value values_reset[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_ACTIVE };
	if (gpiod_line_request_set_values(line_request_out, values_reset))
		printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// BOOT0, RESET
	gpiod_line_value values_normal[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE };
	if (gpiod_line_request_set_values(line_request_out, values_normal))
		printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool mcu_probe_bootloader()
{
	if (i2c_fd < 0) return false;

	if (bootloaderGet() == RES_OK)
	{
		printf("MCU bootloader was found!\n");
		mcu_exists = true;
		mcu_active = false;
		return true;
	}
	return false;
}

bool mcu_switch_bootloader()
{
	if (i2c_fd >= 0 && mcu_active)
	{
		printf("Requesting MCU to switch to bootloader...\n");
		mcu_active = false;
		unsigned char REG_ID[] = { MCU_SWITCH_BOOTLOADER };
		struct i2c_msg I2C_MSG[] = {
			{ MCU_I2C_ADDRESS, 0, sizeof(REG_ID), REG_ID },
		};
		struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
		if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
		{
			printf("Failed to send I2C message to MCU (MCU_SWITCH_BOOTLOADER)! %d: %s\n", errno, strerror(errno));
			handle_i2c_error();
			return false;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		if (bootloaderGet() == RES_OK && bootloaderVersion() == RES_OK && bootloaderId() == RES_OK)
		{
			printf("Successfully switched to and queried the MCUs bootloader!\n");
			mcu_exists = true;
			mcu_intentional_bootloader = true;
			return true;
		}
		else
			printf("Failed to switch or query the bootloader via I2C message! %d: %s\n", errno, strerror(errno));
	}

	if (gpio_chip)
	{
		printf("Resetting MCU with BOOT0 high to switch to bootloader...\n");
		mcu_active = false;

		// BOOT0, RESET
		gpiod_line_value values_reset[] = { GPIOD_LINE_VALUE_ACTIVE, GPIOD_LINE_VALUE_ACTIVE };
		if (gpiod_line_request_set_values(line_request_out, values_reset))
			printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		// BOOT0, RESET
		gpiod_line_value values_boot[] = { GPIOD_LINE_VALUE_ACTIVE, GPIOD_LINE_VALUE_INACTIVE };
		if (gpiod_line_request_set_values(line_request_out, values_boot))
			printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

		std::this_thread::sleep_for(std::chrono::milliseconds(20));

		// BOOT0, RESET
		gpiod_line_value values_normal[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE };
		if (gpiod_line_request_set_values(line_request_out, values_normal))
			printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

		if (bootloaderGet() == RES_OK && bootloaderVersion() == RES_OK && bootloaderId() == RES_OK)
		{
			printf("Successfully switched to and queried the MCUs bootloader!\n");
			mcu_exists = true;
			mcu_intentional_bootloader = true;
			return true;
		}
		else
		{
			printf("Failed to switch or query the bootloader via GPIO pins!\n");
			printf("Perhaps flash configuration (option bytes) were wrong, may need an ST Link to flash it!\n");
		}
	}
	return false;
}

bool mcu_flash_program(std::string filename)
{
	if (i2c_fd < 0) return false;

	//if (bootloaderReleaseMemProtect() == RES_FAIL) return false;

	std::ifstream fs(filename);
	if (!fs.is_open()) return false;
	pRESULT ret = RES_OK;

	fs.seekg(0, std::ios::end);
	std::size_t size = fs.tellg();
	fs.seekg(0);
	int pages = (size+2047)/2048; // 2KB Page size of STM32G0

	for (int i = 0; i < pages; i++)
	{
		ret = bootloaderErasePages(i, 1);
		if (ret == RES_FAIL)
		{
			printf("MCU Flash: Failed to erase page %d\n", i);
			return false;
		}
		printf("MCU Flash: MCU IAP: Erased page %d\n", i);
	}

	uint8_t loadAddress[4] = {0x08, 0x00, 0x00, 0x00};
	uint8_t block[256] = {0};
	int curr_block = 0, bytes_read = 0;
	while (!fs.eof())
	{
		fs.read((char*)block, sizeof(block));
		bytes_read = fs.gcount();
		if (bytes_read == 0) break;
		curr_block++;
		printf("MCU Flash: Writing block %d of size %d\n", curr_block, bytes_read);

		ret = flashPage(loadAddress, block, bytes_read);
		if (ret == RES_FAIL)
		{
			printf("MCU Flash: Failed to write block %d / %d, size %d!\n", curr_block, (size+sizeof(block)-1) / sizeof(block), bytes_read);
			break;
		}

		ret = verifyPage(loadAddress, block, bytes_read);
		if (ret == RES_FAIL)
		{
			printf("MCU Flash: Failed to verify block %d / %d, size %d!\n", curr_block, (size+sizeof(block)-1) / sizeof(block), bytes_read);
			break;
		}

		incrementAddress(loadAddress, bytes_read);
		memset(block, 0xff, bytes_read);
	}
	if (ret != RES_FAIL)
		printf("MCU Flash: Successfully flashed MCU program!\n");
	return ret != RES_FAIL;
}

bool mcu_verify_program(std::string filename)
{
	if (i2c_fd < 0) return false;
	std::ifstream fs(filename);
	if (!fs.is_open()) return false;

	uint8_t loadAddress[4] = {0x08, 0x00, 0x00, 0x00};
	uint8_t block[256] = {0};
	int curr_block = 0, bytes_read = 0;
	pRESULT ret = RES_OK;
	while (!fs.eof())
	{
		fs.read((char*)block, sizeof(block));
		bytes_read = fs.gcount();
		if (bytes_read == 0) break;
		curr_block++;
		printf("MCU Flash: Verifying block %d of size %d\n", curr_block, bytes_read);

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		ret = verifyPage(loadAddress, block, bytes_read);
		if (ret == RES_FAIL)
		{
			printf("MCU Flash: Block %d of size %d differs!\n", curr_block, bytes_read);
			break;
		}

		incrementAddress(loadAddress, bytes_read);
		memset(block, 0xff, bytes_read);
	}
	if (ret != RES_FAIL)
		printf("MCU Flash: Successfully verified MCU program!\n");
	return ret != RES_FAIL;
}

static bool mcu_send_ping()
{
	if (i2c_fd < 0) return false;
	unsigned char REG_ID[] = { MCU_PING };
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(REG_ID), REG_ID },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to send I2C message to MCU (ping)! %d: %s\n", errno, strerror(errno));
		handle_i2c_error();
		return false;
	}
	return true;
}

static bool mcu_fetch_descriptor(std::string &descriptor, uint8_t stringID, uint16_t length)
{
	if (i2c_fd < 0) return false;
	descriptor.clear();
	if (length == 0) return true;

	std::vector<uint8_t> INFO_DATA(MCU_LEADING_BYTES+2+length);
	unsigned char FETCH_CMD[] = { stringID };
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(FETCH_CMD), FETCH_CMD },
		{ MCU_I2C_ADDRESS, I2C_M_RD, (uint16_t)INFO_DATA.size(), INFO_DATA.data() },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to send I2C message to MCU (get descriptor)! %d: %s\n", errno, strerror(errno));
		handle_i2c_error();
		return false;
	}

	uint8_t *packet = INFO_DATA.data()+MCU_LEADING_BYTES;
	uint16_t received = (packet[0] << 8) | packet[1];
	if (received != length)
		printf("Requested string %d of length %d but received string of length %d!\n", stringID, length, received);
	uint16_t strlen = std::min(received, length);
	descriptor = std::string((char*)packet+2, strlen);
	return true;
}

static bool mcu_fetch_subparts(std::vector<uint64_t> &subparts, uint8_t count)
{
	if (i2c_fd < 0) return false;
	subparts.clear();
	if (count == 0) return true;

	std::vector<uint8_t> INFO_DATA(MCU_LEADING_BYTES+2+count*sizeof(uint64_t));
	unsigned char FETCH_CMD[] = { MCU_GET_PARTS };
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(FETCH_CMD), FETCH_CMD },
		{ MCU_I2C_ADDRESS, I2C_M_RD, (uint16_t)INFO_DATA.size(), INFO_DATA.data() },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to send I2C message to MCU (get subparts)! %d: %s\n", errno, strerror(errno));
		handle_i2c_error();
		return false;
	}

	uint8_t *packet = INFO_DATA.data()+MCU_LEADING_BYTES;
	uint8_t received = packet[1];
	if (received != count)
		printf("Requested %d subpart serial IDs but received %d!\n", count, received);
	received = std::min(received, count);
	subparts.resize(received);
	memcpy(subparts.data(), packet+2, received*sizeof(uint64_t));
	return true;
}

bool mcu_fetch_info(MCU_StoredInfo &info)
{
	if (i2c_fd < 0) return false;

	uint8_t INFO_DATA[MCU_LEADING_BYTES+MCU_INFO_MAX_LENGTH];
	uint8_t FETCH_VERSION = 1; // Request up to this version, may receive lower version
	unsigned char FETCH_CMD[] = { MCU_FETCH_INFO, FETCH_VERSION };
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(FETCH_CMD), FETCH_CMD },
		{ MCU_I2C_ADDRESS, I2C_M_RD, sizeof(INFO_DATA), INFO_DATA },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to send I2C message to MCU (fetch info)! %d: %s\n", errno, strerror(errno));
		handle_i2c_error();
		return false;
	}

	uint8_t *packet = INFO_DATA+MCU_LEADING_BYTES;
	FETCH_VERSION = packet[0]; // Actually received version
	if (FETCH_VERSION != 1)
	{
		printf("Fetch info packet version %d is unsupported!\n", FETCH_VERSION);
		return false;
	}
	info.mcuOTPVersion = packet[1]; // Should not concern us too much, but may be of interest in interpreting the data

	uint8_t *ptr = packet+8;
	memcpy(&info.cameraID, ptr, sizeof(CameraID));
	ptr += sizeof(CameraID);
	memcpy(&info.mcuFWVersion, ptr, sizeof(VersionDesc));
	ptr += sizeof(VersionDesc);
	memcpy(&info.hardwareSerial, ptr, sizeof(HardwareSerial));
	ptr += sizeof(HardwareSerial);
	memcpy(&info.mcuUniqueID, ptr, 3*sizeof(uint32_t));
	ptr += 3*sizeof(uint32_t);

	uint8_t subpartCount = packet[2];
	if (!mcu_fetch_subparts(info.subpartSerials, subpartCount))
		return false;

	uint16_t hwStrLen = (packet[4] << 8) | packet[5];
	if (!mcu_fetch_descriptor(info.mcuHWDescriptor, MCU_GET_HW_STR, hwStrLen))
		return false;

	uint16_t fwStrLen = (packet[6] << 8) | packet[7];
	if (!mcu_fetch_descriptor(info.mcuFWDescriptor, MCU_GET_FW_STR, fwStrLen))
		return false;

	return true;
}

bool mcu_update_id(CameraID cameraID)
{
	if (i2c_fd < 0) return false;

	printf("Updating MCU CameraID to #%u!\n", cameraID);

	uint8_t *ID_ARR = (uint8_t*)&cameraID;
	unsigned char UPDATE_CMD[] = { MCU_UPDATE_ID, ID_ARR[0], ID_ARR[1], ID_ARR[2], ID_ARR[3] };
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(UPDATE_CMD), UPDATE_CMD },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to send I2C message to MCU (update ID)! %d: %s\n", errno, strerror(errno));
		handle_i2c_error();
		return false;
	}
	return true;
}

void mcu_sync_info()
{
	MCU_StoredInfo info;
	if (mcu_fetch_info(info))
	{ // Successfully fetche info from MCU
		if (receivedInfoFromMCU(std::move(info)))
		{ // Had differing camera IDs and need to tell MCU to update it
			mcu_update_id(cameraID);
		}
	}
}

bool mcu_get_status()
{
	if (i2c_fd < 0) return false;

	uint8_t STATUS_DATA[MCU_LEADING_BYTES+MCU_STATUS_LENGTH];
	unsigned char FETCH_CMD[] = { MCU_GET_STATUS };
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(FETCH_CMD), FETCH_CMD },
		{ MCU_I2C_ADDRESS, I2C_M_RD, sizeof(STATUS_DATA), STATUS_DATA },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
	TimePoint_t requestTime = sclock::now();
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to send I2C message to MCU (get status)! %d: %s\n", errno, strerror(errno));
		handle_i2c_error();
		return false;
	}
	TimePoint_t receiveTime = sclock::now();
	long roundtripTimeUS = dtUS(requestTime, receiveTime);
	const int observedRoundtripUS = 430;
	const int // Just estimate of fixed delays, they don't exactly add up to the observed roundtrip time
		transmitUS = ((sizeof(FETCH_CMD)+1)*9 + 2)*1000/400,
		receiveUS = (sizeof(STATUS_DATA)*9 + 2)*1000/400 + 10;
	TimePoint_t estSendTime = receiveTime - std::chrono::microseconds(receiveUS);
	bool timingReliable = std::abs(roundtripTimeUS - observedRoundtripUS) < 20;

	uint8_t *packet = STATUS_DATA+MCU_LEADING_BYTES;
	
	uint16_t states = (packet[0] << 8) | packet[1];

	static enum FilterSwitchCommand pastFilterState = FILTER_KEEP;
	enum FilterSwitchCommand filterState = (enum FilterSwitchCommand)((states >> 14) & 0b11);
	if (pastFilterState != filterState)
	{
		pastFilterState = filterState;
		if (filterState == FILTER_SWITCH_INFRARED)
			printf("Filter set to show infrared light!\n");
		else if (filterState == FILTER_SWITCH_VISIBLE)
			printf("Filter set to show visible light!\n");
		else
			printf("Filter switcher in unknown state!\n");
	}

	static TimePoint_t lastVoltageReadTime = sclock::now();
	uint16_t powerMV = (packet[2] << 8) | packet[3];
	if (powerMV < 10000 || powerMV > 22000 || dtMS(lastVoltageReadTime, sclock::now()) > 1000)
	{
		lastVoltageReadTime = sclock::now();
		printf("Voltage is %.4fV\n", powerMV/1000.0f);
	}

	uint16_t timestampUS = (packet[4] << 8) | packet[5];
	if (dtUS(timesync.lastTime, sclock::now()) > 1000000)
		ResetTimeSync(timesync);
	TimePoint_t sendTime;
	if (timingReliable)
		sendTime = UpdateTimeSync(timesync, timestampUS, 1<<16, estSendTime);
	else sendTime = GetTimeSynced(timesync, timestampUS, 1<<16);
	long responseTimeUS = dtUS(sendTime, receiveTime);

	static uint8_t lastReceivedFrameID = 0;
	uint8_t lastFrameID = packet[6];
	uint16_t usSinceFrameID = (packet[7] << 8) | packet[8];
	if (usSinceFrameID < 0xFFFF)
	{
		if (lastReceivedFrameID != lastFrameID && ((lastReceivedFrameID+1)%256) != lastFrameID)
			printf("Received odd frame ID %d %.2fms ago - previous one %d!\n", lastFrameID, usSinceFrameID/1000.0f, lastReceivedFrameID); 
		lastReceivedFrameID = lastFrameID;
	}

	uint8_t queueSize = packet[9];
	uint8_t packetSize = (packet[10] << 8) | packet[11];
	if (queueSize > 0)
		printf("%d packets are queued, next of size %d!\n", queueSize, packetSize);

	return true;
}

static bool i2c_init()
{
	if (i2c_fd >= 0)
		close(i2c_fd);
	i2c_fd = open(i2c_port, O_RDWR);
	if (i2c_fd < 0)
	{
		printf("Failed to open MCU I2C FD! %d: %s \n", errno, strerror(errno));
		i2c_fd = -1;
		return false;
	}
	return true;
}

static bool i2c_probe()
{
	if (i2c_fd < 0) return false;

	unsigned char REG_ID[] = { MCU_REG_ID };
	uint8_t MCU_ID[MCU_LEADING_BYTES+1];
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(REG_ID), REG_ID },
		{ MCU_I2C_ADDRESS, I2C_M_RD, sizeof(MCU_ID), MCU_ID },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, sizeof(I2C_MSG)/sizeof(i2c_msg) };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to read MCU ID register! %d: %s\n", errno, strerror(errno));
		handle_i2c_error();
		return false;
	}

	if (MCU_ID[MCU_LEADING_BYTES] != MCU_I2C_ID)
	{
		printf("Failed verify MCU ID %x against expected ID %x!\n", MCU_ID[MCU_LEADING_BYTES], MCU_I2C_ID);
		return false;
	}

	return true;
}

static void i2c_cleanup()
{
	if (i2c_fd >= 0)
		close(i2c_fd);
	i2c_fd = -1;
}

static bool gpio_init()
{
	gpio_chip = gpiod_chip_open(gpio_chipname);
	if (!gpio_chip)
	{
		printf("Failed open GPIO Chip 0! %d: %s\n", errno, strerror(errno));
		return false;
	}

	req_config = nullptr;
	//req_config = gpiod_request_config_new();

	line_config_out = gpiod_line_config_new();
	line_config_in = gpiod_line_config_new();
	if (!line_config_out || !line_config_in)
	{
		printf("Failed create line config! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	line_boot0 = gpiod_line_settings_new();
	line_reset = gpiod_line_settings_new();
	line_int = gpiod_line_settings_new();

	if (!line_boot0 || !line_reset || !line_int)
	{
		printf("Failed create line settings! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	gpiod_line_settings_set_direction(line_boot0, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(line_boot0, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_active_low(line_boot0, false);
	gpiod_line_settings_set_output_value(line_boot0, GPIOD_LINE_VALUE_INACTIVE);

	gpiod_line_settings_set_direction(line_reset, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(line_reset, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_active_low(line_reset, false);
	gpiod_line_settings_set_output_value(line_reset, GPIOD_LINE_VALUE_INACTIVE);

	gpiod_line_settings_set_direction(line_int, GPIOD_LINE_DIRECTION_INPUT);
	gpiod_line_settings_set_bias(line_int, GPIOD_LINE_BIAS_PULL_UP); // Technically we already have a pull-up in hardware
	gpiod_line_settings_set_edge_detection(line_int, GPIOD_LINE_EDGE_FALLING);

	if (gpiod_line_config_add_line_settings(line_config_out, &PIN_BOOT0, 1, line_boot0) ||
		gpiod_line_config_add_line_settings(line_config_out, &PIN_NRST, 1, line_reset) ||
		gpiod_line_config_add_line_settings(line_config_in, &PIN_INT, 1, line_int))
	{
		printf("Failed to add line setting to GPIO line config! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	line_request_out = gpiod_chip_request_lines(gpio_chip, req_config, line_config_out);
	line_request_in = gpiod_chip_request_lines(gpio_chip, req_config, line_config_in);
	if (!line_request_out || !line_request_in)
	{
		printf("Failed to request lines for GPIO! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	edge_events = gpiod_edge_event_buffer_new(16);
	if (!edge_events)
	{
		printf("Failed to request GPIO edge event buffer! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	return true;
}

static void gpio_cleanup()
{
	if (edge_events) gpiod_edge_event_buffer_free(edge_events);

	if (line_request_out) gpiod_line_request_release(line_request_out);
	if (line_request_in) gpiod_line_request_release(line_request_in);

	if (line_boot0) gpiod_line_settings_free(line_boot0);
	if (line_reset) gpiod_line_settings_free(line_reset);
	if (line_int) gpiod_line_settings_free(line_int);

	if (line_config_out) gpiod_line_config_free(line_config_out);
	if (line_config_in) gpiod_line_config_free(line_config_in);

	if (req_config) gpiod_request_config_free(req_config);

	if (gpio_chip) gpiod_chip_close(gpio_chip);

	edge_events = nullptr;
	line_request_out = nullptr;
	line_request_in = nullptr;
	line_boot0 = nullptr;
	line_reset = nullptr;
	line_int = nullptr;
	line_config_out  = nullptr;
	line_request_in = nullptr;
	req_config = nullptr;
	gpio_chip = nullptr;
}