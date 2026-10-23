/**
AsterTrack Optical Tracking System
Copyright (C)  2026 Seneral <seneral@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include <cstdint>
#include <cstring>
#include <errno.h>
#include <stdio.h>

#include <sys/ioctl.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <linux/videodev2.h>

static int32_t Read16Bit(unsigned int fd, uint16_t reg)
{
	unsigned char REG_HIGH[2] = { (uint8_t)(reg>>8), (uint8_t)(reg&0xFF) }; // Assume low is even
	unsigned char REG_LOW[2] = { (uint8_t)(reg>>8), (uint8_t)((reg&0xFF)+1) };
	uint16_t value;
	uint8_t *val_msg = (uint8_t*)&value;

	// Warning: BCM2835 I2C driver specifically does not support two reads merged in one ioctl

	struct i2c_msg I2C_MSG_HIGH[] = {
		{ 0x60, 0, sizeof(REG_HIGH), REG_HIGH },
		{ 0x60, I2C_M_RD, 1, val_msg+1 },
	};
	struct i2c_rdwr_ioctl_data I2C_READ_HIGH = { I2C_MSG_HIGH, 2 };
	if (ioctl(fd, I2C_RDWR, &I2C_READ_HIGH) < 0)
	{
		printf("Failed to read 16bit high register! %d: %s\n", errno, strerror(errno));
		return -1;
	}

	struct i2c_msg I2C_MSG_LOW[] = {
		{ 0x60, 0, sizeof(REG_LOW), REG_LOW },
		{ 0x60, I2C_M_RD, 1, val_msg+0 },
	};
	struct i2c_rdwr_ioctl_data I2C_READ_LOW = { I2C_MSG_LOW, 2 };
	if (ioctl(fd, I2C_RDWR, &I2C_READ_LOW) < 0)
	{
		printf("Failed to read 16bit low register! %d: %s\n", errno, strerror(errno));
		return -1;
	}

	return value;
}

static int16_t Read8Bit(unsigned int fd, uint16_t reg)
{
	uint8_t value;
	unsigned char REG[2] = { (uint8_t)(reg>>8), (uint8_t)(reg&0xFF) };
	struct i2c_msg I2C_MSG[] = {
		{ 0x60, 0, sizeof(REG), REG },
		{ 0x60, I2C_M_RD, 1, &value },
	};
	struct i2c_rdwr_ioctl_data I2C_READ = { I2C_MSG, 2 };
	if (ioctl(fd, I2C_RDWR, &I2C_READ) < 0)
	{
		printf("Failed to read 8bit register! %d: %s\n", errno, strerror(errno));
		return -1;
	}
	return value;
}
