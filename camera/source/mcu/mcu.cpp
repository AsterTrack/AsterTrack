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

#include "util/util.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <errno.h>
#include <chrono>
#include <thread>
#include <atomic>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

const char *i2c_port = "/dev/i2c-1";
unsigned int i2c_fd;

TimePoint_t lastPing;

std::atomic<bool> stop_thread;
std::thread *mcu_comm_thread;

static bool i2c_init();
static bool i2c_probe();
static void i2c_cleanup();

static void mcu_send_ping();

static void mcu_thread();

bool mcu_init()
{
	return i2c_init();
}

bool mcu_probe()
{
	if (i2c_fd < 0) return false;

	if (!i2c_probe()) return false;

	stop_thread = false;
	//mcu_comm_thread = new std::thread(mcu_thread);

	return true;
}

void mcu_cleanup()
{
	stop_thread = true;
	if (mcu_comm_thread && mcu_comm_thread->joinable())
		mcu_comm_thread->join();
	delete mcu_comm_thread;

	if (i2c_fd >= 0)
		i2c_cleanup();
}

static void mcu_thread()
{
	while (!stop_thread.load())
	{
		if (dtMS(lastPing, sclock::now()) < 1000)
		{
			mcu_send_ping();
			lastPing = sclock::now();
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
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