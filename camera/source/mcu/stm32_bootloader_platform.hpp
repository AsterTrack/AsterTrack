/*
 * STM32 bootloader platform implementation for Linux SBCs
 *
 * Assumes bootloader is externally entered and left before calling bootloader functions
 * And the I2C bus is available through i2c_fd
 *
 * Author: Seneral
 */
#ifndef _BOOTLOADER_PLATFORM_H_
#define _BOOTLOADER_PLATFORM_H_

#include "stm32_bootloader.hpp"

#include <cstdio>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <thread>
#include <chrono>

#define BOOTLOADER_I2C 0
#define BOOTLOADER_UART 1

#define BOOTLOADER_PORT BOOTLOADER_I2C
#define DEVICE_I2C_ADDRESS 0x56 // check STM32 AN2606 for the I2C address for each device model ("Target 7-bit address")
extern unsigned int i2c_fd;

#define USE_NO_STRETCH_COMMANDS	// Choose commands that 

#define ENABLE_DEBUG_LOG
#ifdef ENABLE_DEBUG_LOG
static void LogDebugInfo(const char *log) { printf("%s\n", log); }
static void LogDebugInfoHEX(const uint8_t log) { printf("%x\n", log); }
#else
static inline void LogDebugInfo(const char *log) {}
static inline void LogDebugInfoHEX(const uint8_t log) {}
#endif

/**
 * @brief  Receive in Master I2C mode/ UART
 * @param  reg       7 bit I2C address, 0x0xxx xxxx
 * @param  bufp      buffer to receive data
 * @param  len       data length
 * @retval result 0 = RES_OK  -1 = RES_FAIL
 */
static pRESULT platform_read(uint8_t *bufp, uint16_t len)
{
	// TODO: Timeout not respected
	struct i2c_msg I2C_MSG[] = {
		{ DEVICE_I2C_ADDRESS, I2C_M_RD, len, bufp }
	};
	struct i2c_rdwr_ioctl_data I2C_READ = { I2C_MSG, 1 };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_READ) < 0)
	{
		printf("Failed to read I2C register! %s\n", strerror(errno));
		return RES_FAIL;
	}
	return RES_OK;
}

/**
 * @brief  Transmit in Master I2C mode  / UART
 * @param  reg       7 bit I2C address, 0x0xxx xxxx
 * @param  bufp      data buffer to send
 * @param  len       data length
 * @retval result 0 = RES_OK  1 = RES_FAIL
 */
static pRESULT platform_write(const uint8_t *bufp, uint16_t len)
{
	struct i2c_msg I2C_MSG[] = {
		{ DEVICE_I2C_ADDRESS, 0, len, (uint8_t*)bufp }
	};
	struct i2c_rdwr_ioctl_data I2C_WRITE = { I2C_MSG, 1 };
	if (ioctl(i2c_fd, I2C_RDWR, &I2C_WRITE) < 0)
	{
		printf("Failed to write I2C data! %s\n", strerror(errno));
		return RES_FAIL;
	}
	return RES_OK;
}

/**
 * @brief  delay some time in milliseconds
 * @param  time_ms
 * @retval
 */
static void platform_delay_ms(uint32_t time_ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(time_ms));
}

#endif //_BOOTLOADER_PLATFORM_H_
