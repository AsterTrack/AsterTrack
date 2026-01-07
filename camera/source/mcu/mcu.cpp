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
gpiod_line_config *line_config;
gpiod_line_settings *line_boot0, *line_reset;
gpiod_line_request *line_request;

std::string mcu_flash_file = "/mnt/mmcblk0p2/tce/TrackingCameraMCU.bin";

std::mutex mcu_mutex;
std::atomic<bool> stop_thread;
std::thread *mcu_comm_thread;
TimePoint_t lastPing;

bool mcu_exists;
bool mcu_active;
bool mcu_intentional_bootloader;


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
	{ // MCU responded, start monitoring it
		mcu_monitor();
		return true;
	}
	// Could not contact MCU, probe bootloader and check if recovery is required

	if (mcu_probe_bootloader())
	{ // If MCU is in bootloader mode for some reason, reset it and attempt to contact it again
		mcu_reconnect();
	}

	if (!mcu_active)
	{ // MCU is still not responding, it is either not available in hardware, or bricked
		if (!mcu_switch_bootloader())
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

	if (mcu_exists)
	{ // Start monitoring MCU only if its available in hardware (e.g. we were able to contact it or its bootloader)
		mcu_monitor();
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
			printf("Verified existance of MCU!\n");
		if (!mcu_active)
		{
			printf("Connected with MCU!\n");
			mcu_sync_info();
		}
		mcu_active = true;
		mcu_exists = true;
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
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	if (mcu_probe())
		return true;
	printf("Still can't reconnect to the MCU after further reset!\n");
	return false;
}

void mcu_monitor()
{
	lastPing = sclock::now();
	stop_thread = false;
	if (!mcu_comm_thread)
		mcu_comm_thread = new std::thread(mcu_thread);
}

void mcu_cleanup()
{
	stop_thread = true;
	if (mcu_comm_thread && mcu_comm_thread->joinable())
		mcu_comm_thread->join();
	delete mcu_comm_thread;

	if (gpio_chip)
		gpio_cleanup();

	if (i2c_fd >= 0)
		i2c_cleanup();
}

static bool handle_i2c_error()
{
	if (errno == EBADF)// || errno == ETIMEDOUT)
	{
		mcu_active = false;
		i2c_init();
		return true;
	}
	return false;
}

static void mcu_thread()
{
	while (!stop_thread.load())
	{
		if (mcu_active && dtMS(lastPing, sclock::now()) > MCU_PING_INTERVAL_MS)
		{
			std::unique_lock lock(mcu_mutex);
			if (!mcu_active)
				continue;
			mcu_send_ping();
			lastPing = sclock::now();
		}
		if (!mcu_active && dtMS(lastPing, sclock::now()) > MCU_PROBE_INTERVAL_MS && !mcu_intentional_bootloader)
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

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void mcu_reset()
{
	if (!gpio_chip) return;

	printf("Resetting MCU...\n");

	// BOOT0, RESET
	gpiod_line_value values_reset[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_ACTIVE };
	if (gpiod_line_request_set_values(line_request, values_reset))
		printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	// BOOT0, RESET
	gpiod_line_value values_normal[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE };
	if (gpiod_line_request_set_values(line_request, values_normal))
		printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

	std::this_thread::sleep_for(std::chrono::milliseconds(10));

	mcu_intentional_bootloader = false;
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
		if (gpiod_line_request_set_values(line_request, values_reset))
			printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		// BOOT0, RESET
		gpiod_line_value values_boot[] = { GPIOD_LINE_VALUE_ACTIVE, GPIOD_LINE_VALUE_INACTIVE };
		if (gpiod_line_request_set_values(line_request, values_boot))
			printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

		std::this_thread::sleep_for(std::chrono::milliseconds(20));

		// BOOT0, RESET
		gpiod_line_value values_normal[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE };
		if (gpiod_line_request_set_values(line_request, values_normal))
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

	if (packet[2] <= 8)
	{
		info.subpartSerials.resize(packet[2]);
		memcpy(info.subpartSerials.data(), ptr, packet[2]*sizeof(uint64_t));
	}

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

	line_config = gpiod_line_config_new();
	if (line_config)
		line_boot0 = gpiod_line_settings_new();
	if (line_boot0)
		line_reset = gpiod_line_settings_new();

	if (!line_config || !line_boot0 || !line_reset)
	{
		printf("Failed open GPIO Chip 0! %d: %s\n", errno, strerror(errno));
		gpiod_chip_close(gpio_chip);
	}

	gpiod_line_settings_set_direction(line_boot0, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(line_boot0, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_active_low(line_boot0, false);
	gpiod_line_settings_set_output_value(line_boot0, GPIOD_LINE_VALUE_INACTIVE);

	gpiod_line_settings_set_direction(line_reset, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(line_reset, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_active_low(line_reset, false);
	gpiod_line_settings_set_output_value(line_reset, GPIOD_LINE_VALUE_INACTIVE);

	if (gpiod_line_config_add_line_settings(line_config, &PIN_BOOT0, 1, line_boot0))
	{
		printf("Failed to add line setting for BOOT0 to GPIO! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	if (gpiod_line_config_add_line_settings(line_config, &PIN_NRST, 1, line_reset))
	{
		printf("Failed to add line setting for NRST to GPIO! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	line_request = gpiod_chip_request_lines(gpio_chip, req_config, line_config);
	if (!line_request)
	{
		printf("Failed to request lines for GPIO! %d: %s\n", errno, strerror(errno));
		gpio_cleanup();
		return false;
	}

	return true;
}

static void gpio_cleanup()
{
	if (line_request)
		gpiod_line_request_release(line_request);

	gpiod_line_settings_free(line_boot0);
	gpiod_line_settings_free(line_reset);
	gpiod_line_config_free(line_config);

	if (req_config)
		gpiod_request_config_free(req_config);

	gpiod_chip_close(gpio_chip);

	line_request = nullptr;
	line_boot0 = nullptr;
	line_reset = nullptr;
	line_config  = nullptr;
	req_config = nullptr;
	gpio_chip = nullptr;
}