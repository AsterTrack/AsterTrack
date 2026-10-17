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

#include "camera/gcs.hpp"

#include "i2c.inl"

#include <cmath>

static bool ov9281_getSensorArea(const GCS_CameraParams *params, uint16_t &sensorX, uint16_t &sensorY, uint16_t &minX, uint16_t &minY, uint16_t &maxX, uint16_t &maxY)
{
	minX = minY = 0;
	sensorX = maxX = 1280;
	sensorY = maxY = 800;
	switch (params->height)
	{
		case 800: // Full frame
		case 400: // Subsampled from full frame
			return true;
		case 720: // Cropped vertically
			minY = 40;
			maxY = 760;
			return true;
		case 480: // Cropped,but not supported by driver
			minX = 320;
			maxX = 960;
			minY = 160;
			maxY = 640;
			return true;
		default: // Should not happen
			printf("Invalid mode %d x %d for OV9281!\n", params->width, params->height);
			return false;
	}
}

static void ov9281_deconfigure(unsigned int i2c_fd, const GCS_CameraParams *params)
{
	// Default config for most parameters
	unsigned char SC_CTRL_06[3] = { 0x30, 0x06, 0x06 };			// Enable ILPWM and VSYNC output (as opposed to STROBE)
	unsigned char ANA_CORE_6[3] = { 0x36, 0x66, 0x0A };			// Don't use FSIN
	unsigned char SC_LP_CTRL4[3] = { 0x30, 0x30, 0x00 };		// Disable FSIN to wake camera from sleep
	unsigned char TIMING_REG23[3] = { 0x38, 0x23, 0x00 };		// Default toggles for timing & sync

	const int I2C_MSG_CNT = 4;
	struct i2c_msg I2C_MSG[I2C_MSG_CNT] = {
		{ 0x60, 0, sizeof(SC_CTRL_06), SC_CTRL_06 },
		{ 0x60, 0, sizeof(ANA_CORE_6), ANA_CORE_6 },
		{ 0x60, 0, sizeof(SC_LP_CTRL4), SC_LP_CTRL4 },
		{ 0x60, 0, sizeof(TIMING_REG23), TIMING_REG23 },
	};
	for (int i = 0; i < I2C_MSG_CNT; i++)
	{
		struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG+i, 1 };
		if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
			printf("Failed to write deconfig register %d! %d: %s\n", i, errno, strerror(errno));
	}
}

static void ov9281_configure(unsigned int i2c_fd, const GCS_CameraParams *params)
{
	// Default config for free, unsynchronised streaming
	unsigned char PSV_CTRL[3] = { 0x4f, 0x00, 0x08 };			// Disable PSV - Power Saving in vblank - completely
	unsigned char SC_MODE_SELECT[3] = { 0x01, 0x00, 0x01 };		// Keep camera in streaming mode
	unsigned char SC_CTRL_06[3] = { 0x30, 0x06, 0x06 };			// Enable ILPWM and VSYNC output (as opposed to STROBE)
	unsigned char ANA_CORE_6[3] = { 0x36, 0x66, 0x0A };			// Don't use FSIN
	unsigned char SC_LP_CTRL4[3] = { 0x30, 0x30, 0x00 };		// Disable FSIN to wake camera from sleep
	unsigned char SC_CTRL_3F[3] = { 0x30, 0x3F, 0x00 };			// Number of frames to trigger in burst
	unsigned char TIMING_REG23[3] = { 0x38, 0x23, 0x00 };		// Default toggles for timing & sync

	// TODO: When switching to external-trigger mode after leaving it before, it will run at 55Hz completely desynced
	// E.g. switching frame sync -> ext trigger works fine, but not ext trigger -> frame sync -> ext trigger
	if (params->sync == 1)
	{ // Configure external trigger mode (max 120Hz) - FSIN as frame trigger
		SC_MODE_SELECT[2] = 0x00;		// Put camera in standby
		SC_CTRL_06[2] = 0x04;			// Enable FSIN input (not VSYNC output)
		ANA_CORE_6[2] = 0x00;			// Use FSIN pin for frame sync/trigger
		SC_LP_CTRL4[2] = 0x04;			// Enable FSIN to wake camera and trigger frames
		SC_CTRL_3F[2] = 0x01;			// Set number of frames to trigger to one
	}
	else if (params->sync == 2)
	{ // Configure frame sync mode (max 144Hz) - FSIN as frame sync
		SC_CTRL_06[2] = 0x04;			// Enable FSIN input (not VSYNC output)
		ANA_CORE_6[2] = 0x00;			// Use FSIN pin for frame sync/trigger
		TIMING_REG23[2] = 0x20;			// Set ext_vs_en to lock vertical sync with FSIN
	}

	if (params->strobe)
	{
		SC_CTRL_06[2] |= 0x08; // Enable STROBE output

		// Strobe silicon is not aware of mode, and WILL just follow strobe pattern
		// E.g. in external trigger, it will start a next strobe even before the next trigger, according to configured fps
		// But we do need full strobe pattern for all other modes, or else it will not strobe for those frames
		uint8_t strobe_pattern = params->sync == 1? 0x80 : 0xFF;

		// Set strobe_frame_pattern within the triggered frame burst - default 0xA5
		// Set strobe_frame_shift [0-31] (signed offset after SOF) - default 5
		// Set strobe_frame_span [0-31] (pulse width) in steps - default 26
		uint8_t offset_low = (uint8_t)(std::abs(params->strobeOffset) & 0xFF);
		uint8_t offset_high = (uint8_t)((std::abs(params->strobeOffset) >> 8) & 0xFF);
		uint8_t offset_sign = (uint8_t)(params->strobeOffset < 0? 0x80 : 0x00);
		uint8_t width_low = (uint8_t)(params->strobeLength & 0xFF);
		uint8_t width_high = (uint8_t)((params->strobeLength >> 8) & 0xFF);
		unsigned char PWM_CTRL_20_X[] = { 0x39, 0x20,
			strobe_pattern,
			offset_sign, 0x00, offset_high, offset_low,
			0x00, 0x00, width_high, width_low
		};

		const int I2C_MSG_CNT = 1;
		struct i2c_msg I2C_MSG[I2C_MSG_CNT] = {
			{ 0x60, 0, sizeof(PWM_CTRL_20_X), PWM_CTRL_20_X }
		};
		struct i2c_rdwr_ioctl_data I2C_DATA_STROBE = { I2C_MSG, I2C_MSG_CNT };
		if (params->strobe && ioctl(i2c_fd, I2C_RDWR, &I2C_DATA_STROBE) < 0)
			printf("Failed to write strobe registers! %s\n", strerror(errno));
	}

	const int I2C_MSG_CNT = 7;
	struct i2c_msg I2C_MSG[I2C_MSG_CNT] = {
		{ 0x60, 0, sizeof(PSV_CTRL), PSV_CTRL },
		{ 0x60, 0, sizeof(SC_MODE_SELECT), SC_MODE_SELECT },
		{ 0x60, 0, sizeof(SC_CTRL_06), SC_CTRL_06 },
		{ 0x60, 0, sizeof(ANA_CORE_6), ANA_CORE_6 },
		{ 0x60, 0, sizeof(SC_LP_CTRL4), SC_LP_CTRL4 },
		{ 0x60, 0, sizeof(SC_CTRL_3F), SC_CTRL_3F },
		{ 0x60, 0, sizeof(TIMING_REG23), TIMING_REG23 },
	};
	for (int i = 0; i < I2C_MSG_CNT; i++)
	{
		struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG+i, 1 };
		if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
			printf("Failed to write sync register %d! %d: %s\n", i, errno, strerror(errno));
	}
}