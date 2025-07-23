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
unsigned int i2c_fd;

const char *gpio_chipname = "/dev/gpiochip0";
unsigned int PIN_BOOT0 = 27;
unsigned int PIN_NRST = 18;
unsigned int PIN_INT = 17;

gpiod_chip *gpio_chip;
gpiod_request_config *req_config;
gpiod_line_config *line_config;
gpiod_line_settings *line_boot0, *line_reset;
gpiod_line_request *line_request;

TimePoint_t lastPing;

std::atomic<bool> stop_thread;
std::thread *mcu_comm_thread;

static bool i2c_init();
static bool i2c_probe();
static void i2c_cleanup();

static bool gpio_init();
static void gpio_cleanup();

static void mcu_send_ping();

static void mcu_thread();

bool mcu_init()
{
	bool i2c = i2c_init();
	bool gpio = gpio_init();
	return i2c && gpio;
}

bool mcu_probe()
{
	if (i2c_fd < 0) return false;

	if (!i2c_probe()) return false;

	lastPing = sclock::now();
	stop_thread = false;
	mcu_comm_thread = new std::thread(mcu_thread);

	return true;
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

static void mcu_thread()
{
	while (!stop_thread.load())
	{
		if (dtMS(lastPing, sclock::now()) > MCU_PING_INTERVAL_MS)
		{
			mcu_send_ping();
			lastPing = sclock::now();
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void mcu_reset()
{
	if (!gpio_chip) return;

	// BOOT0, RESET
	gpiod_line_value values_reset[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_ACTIVE };
	if (gpiod_line_request_set_values(line_request, values_reset))
		printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

	std::this_thread::sleep_for(std::chrono::milliseconds(1));

	// BOOT0, RESET
	gpiod_line_value values_normal[] = { GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE };
	if (gpiod_line_request_set_values(line_request, values_normal))
		printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));
}

void mcu_switch_bootloader()
{
	if (i2c_fd >= 0)
	{
		unsigned char REG_ID[1] = { MCU_SWITCH_BOOTLOADER };
		struct i2c_msg I2C_MSG[] = {
			{ MCU_I2C_ADDRESS, 0, sizeof(REG_ID), REG_ID },
		};
		struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, 1 };
		if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
			printf("Failed to send I2C message to MCU (MCU_SWITCH_BOOTLOADER)! %d: %s\n", errno, strerror(errno));

		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		if (bootloaderGet() == RES_OK && bootloaderVersion() == RES_OK && bootloaderId() == RES_OK)
		{
			printf("Successfully switched to and queried the MCUs bootloader!\n");
			return;
		}
		else
			printf("Failed to switch or query the bootloader via I2C message! %d: %s\n", errno, strerror(errno));
	}

	if (gpio_chip)
	{
		// BOOT0, RESET
		gpiod_line_value values_reset[] = { GPIOD_LINE_VALUE_ACTIVE, GPIOD_LINE_VALUE_ACTIVE };
		if (gpiod_line_request_set_values(line_request, values_reset))
			printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

		std::this_thread::sleep_for(std::chrono::milliseconds(1));

		// BOOT0, RESET
		gpiod_line_value values_normal[] = { GPIOD_LINE_VALUE_ACTIVE, GPIOD_LINE_VALUE_INACTIVE };
		if (gpiod_line_request_set_values(line_request, values_normal))
			printf("Failed to set output of GPIO! %d: %s\n", errno, strerror(errno));

		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		if (bootloaderGet() == RES_OK && bootloaderVersion() == RES_OK && bootloaderId() == RES_OK)
		{
			printf("Successfully switched to and queried the MCUs bootloader!\n");
			return;
		}
		else
			printf("Failed to switch or query the bootloader via GPIO pins! %d: %s\n", errno, strerror(errno));
	}
}

bool mcu_flash_program(std::string filename)
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
		printf("Slave MCU IAP: Writing block: %d,block size: %d\n", curr_block, bytes_read);

		ret = flashPage(loadAddress, block, bytes_read);
		if (ret == RES_FAIL)
			break;

		ret = verifyPage(loadAddress, block, bytes_read);
		if (ret == RES_FAIL)
			break;

		incrementAddress(loadAddress, bytes_read);
		memset(block, 0xff, bytes_read);
	}
	fs.close();
	if (ret != RES_FAIL)
		printf("Successfully flashed MCU program!\n");
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
		printf("Slave MCU IAP: Verifying block: %d,block size: %d\n", curr_block, bytes_read);

		ret = verifyPage(loadAddress, block, bytes_read);
		if (ret == RES_FAIL)
			break;

		incrementAddress(loadAddress, bytes_read);
		memset(block, 0xff, bytes_read);
	}
	fs.close();
	if (ret != RES_FAIL)
		printf("Successfully verified MCU program!\n");
	return ret != RES_FAIL;
}

static void mcu_send_ping()
{
	if (i2c_fd < 0) return;
	unsigned char REG_ID[1] = { MCU_PING };
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(REG_ID), REG_ID },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, 1 };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
		printf("Failed to send I2C message to MCU (ping)! %d: %s\n", errno, strerror(errno));
}

static bool i2c_init()
{
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

	unsigned char REG_ID[1] = { MCU_REG_ID };
	uint8_t MCU_ID;
	struct i2c_msg I2C_MSG[] = {
		{ MCU_I2C_ADDRESS, 0, sizeof(REG_ID), REG_ID },
		{ MCU_I2C_ADDRESS, I2C_M_RD, 1, &MCU_ID },
	};
	struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG, 2 };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
	{
		printf("Failed to read MCU ID register! %d: %s\n", errno, strerror(errno));
		close(i2c_fd);
		return false;
	}

	if (MCU_ID != MCU_I2C_ID)
	{
		printf("Failed verify MCU ID %x against expected ID %x!\n", MCU_ID, MCU_I2C_ID);
		close(i2c_fd);
		return false;
	}

	printf("Verified existance of MCU!\n");
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