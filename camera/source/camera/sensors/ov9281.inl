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
	// Disable external trigger mode so that a sync pulse without this camera set to stream will not make the camera take frames
	unsigned char PSV_CTRL[3] = { 0x4f, 0x00, 0x00 };		// Prevent FSIN from waking camera and exposing

	struct i2c_msg I2C_MSG[] = {
		{ 0x60, 0, sizeof(PSV_CTRL), PSV_CTRL }
	};
	struct i2c_rdwr_ioctl_data I2C_DATA_TRIG = { I2C_MSG+0, 1 };
	if (params->extTrig && ioctl(i2c_fd, I2C_RDWR, &I2C_DATA_TRIG) < 0)
		printf("Failed to write extTrig registers! %s\n", strerror(errno));
}

static void ov9281_configure(unsigned int i2c_fd, const GCS_CameraParams *params)
{
	if (params->extTrig == 1)
	{
		// Switch to external trigger mode (max 120Hz) - FSIN as frame trigger
		unsigned char SC_MODE_SELECT[3] = { 0x01, 0x00, 0x00 };		// Put camera in standby
		unsigned char PSV_CTRL[3] = { 0x4f, 0x00, 0x08 };			// Disable PSV - Power Saving in vblank - completely
		unsigned char SC_CTRL_06[3] = { 0x30, 0x06, 0x04 };			// Disable strobe (set that later), ensure FSIN input (not VSYNC output)
		unsigned char ANA_CORE_6[3] = { 0x36, 0x66, 0x00 };			// Use FSIN pin for frame sync/trigger (set by driver)
		unsigned char SC_LP_CTRL4[3] = { 0x30, 0x30, 0x04 };		// Enable FSIN to wake camera and trigger frames
		unsigned char SC_CTRL_3F[3] = { 0x30, 0x3F, 0x01 };			// Set number of frames to trigger on wakeup to one
		unsigned char AUTO_SLEEP_CTRL[3] = { 0x4f, 0x01, 0x00 };	// Disable tc_sof_sync_en (default)

		const int I2C_MSG_CNT = 7;
		struct i2c_msg I2C_MSG[I2C_MSG_CNT] = {
			{ 0x60, 0, sizeof(SC_MODE_SELECT), SC_MODE_SELECT },
			{ 0x60, 0, sizeof(PSV_CTRL), PSV_CTRL },
			{ 0x60, 0, sizeof(SC_CTRL_06), SC_CTRL_06 },
			{ 0x60, 0, sizeof(ANA_CORE_6), ANA_CORE_6 },
			{ 0x60, 0, sizeof(SC_LP_CTRL4), SC_LP_CTRL4 },
			{ 0x60, 0, sizeof(SC_CTRL_3F), SC_CTRL_3F },
			{ 0x60, 0, sizeof(AUTO_SLEEP_CTRL), AUTO_SLEEP_CTRL }
		};
		for (int i = 0; i < I2C_MSG_CNT; i++)
		{
			struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG+i, 1 };
			if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
				printf("Failed to write extTrig register %d! %d: %s\n", i, errno, strerror(errno));
		}
	}
	/* else if (params->extTrig == 2)
	{
		// Switch to frame sync mode (max 144Hz) - FSIN as frame sync
		unsigned char SC_MODE_SELECT[3] = { 0x01, 0x00, 0x01 };		// Keep camera in streaming mode
		unsigned char PSV_CTRL[3] = { 0x4f, 0x00, 0x08 };			// Disable PSV - Power Saving in vblank - completely
		unsigned char SC_CTRL_06[3] = { 0x30, 0x06, 0x04 };			// Disable strobe (set that later), ensure FSIN input (not VSYNC output)
		unsigned char ANA_CORE_6[3] = { 0x36, 0x66, 0x00 };			// Use FSIN pin for frame sync/trigger (set by driver)
		unsigned char SC_LP_CTRL4[3] = { 0x30, 0x30, 0x00 };		// Disable FSIN to wake camera from sleep (camera is not sleeping)
		unsigned char SC_CTRL_3F[3] = { 0x30, 0x3F, 0x01 };			// Number of frames to trigger (irrelevant)
		unsigned char AUTO_SLEEP_CTRL[3] = { 0x4f, 0x01, 0x08 };	// Enable tc_sof_sync_en

		// Unsure what they do, but sound like something
		//unsigned char TIMING_REG23[3] = { 0x38, 0x23, 0x40 };	// ext_vs_re
		//unsigned char TIMING_REG23[3] = { 0x38, 0x23, 0x20 };	// ext_vs_en
		//unsigned char FC_CTRL0[3] = { 0x42, 0x40, 0x04 };	// sof_sel
		//unsigned char FC_CTRL3[3] = { 0x42, 0x43, 0x02 };	// sof_mask_dis

		// TODO: Figure out if sync mode exists and how to use it instead of external trigger mode
		// With external trigger, the sensor goes to sleep between each frame, which limits framerate to 120Hz
		// But in free-running mode, 144Hz is possible for 1280x800 @ 8bit
		// So if the "sync" mode, hinted at in documentation, exists so that it doesn't go to sleep every time, that'd be great

		const int I2C_MSG_CNT = 7;
		struct i2c_msg I2C_MSG[I2C_MSG_CNT] = {
			{ 0x60, 0, sizeof(SC_MODE_SELECT), SC_MODE_SELECT },
			{ 0x60, 0, sizeof(PSV_CTRL), PSV_CTRL },
			{ 0x60, 0, sizeof(SC_CTRL_06), SC_CTRL_06 },
			{ 0x60, 0, sizeof(ANA_CORE_6), ANA_CORE_6 },
			{ 0x60, 0, sizeof(SC_LP_CTRL4), SC_LP_CTRL4 },
			{ 0x60, 0, sizeof(SC_CTRL_3F), SC_CTRL_3F },
			{ 0x60, 0, sizeof(AUTO_SLEEP_CTRL), AUTO_SLEEP_CTRL }
		};
		for (int i = 0; i < I2C_MSG_CNT; i++)
		{
			struct i2c_rdwr_ioctl_data I2C_DATA = { I2C_MSG+i, 1 };
			if (ioctl(i2c_fd, I2C_RDWR, &I2C_DATA) < 0)
				printf("Failed to write extTrig register %d! %d: %s\n", i, errno, strerror(errno));
		}
	} */

	if (params->strobe)
	{
		// Set strobe and ILPWM to output, leave FSIN as input
		unsigned char SC_CTRL_06[3] = { 0x30, 0x06, 0b00001100 };
		// Set strobe_frame_pattern - default 0xA5
		// Set strobe_frame_shift [0-7] (offset after SOF) - default 5
		// Set strobe_frame_shift [24-30], shift direction [31] (offset after SOF) - default 0
		// Set strobe_frame_span [0-7] (pulse width) in steps - default 26
		uint8_t offset_low = (uint8_t)std::min(255, std::abs(params->strobeOffset));
		uint8_t offset_high = (uint8_t)((params->strobeOffset<0? 0x80 : 0x00) + (std::abs(params->strobeOffset)>>8));
		uint8_t width_low = (uint8_t)(params->strobeLength&0xFF);
		uint8_t width_high = (uint8_t)((params->strobeLength>>8)&0xFF);
		unsigned char PWM_CTRL_20_X[] = { 0x39, 0x20, 0xFF, offset_high, 0x00, 0x00, offset_low, 0x00, 0x00, width_high, width_low,
		// Tests with other fields:
			0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x0C }; // Custom
//				0x01, 0xB4, 0x00, 0x10, 0x05, 0xF2, 0x40 }; // Default
//				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 };
		// First 4 fields don't matter. Rest makes start of strobe earlier by 12us (yay)
		// But with this, it needs strobe_frame_span=45 to get to maximum brightness at 118us width (end at 834us after trig)
		// Before strobe_frame_span=13 was enough 

		struct i2c_msg I2C_MSG[] = {
			{ 0x60, 0, sizeof(SC_CTRL_06), SC_CTRL_06 },
			{ 0x60, 0, sizeof(PWM_CTRL_20_X), PWM_CTRL_20_X }
		};
		struct i2c_rdwr_ioctl_data I2C_DATA_STROBE = { I2C_MSG, 2 };
		if (params->strobe && ioctl(i2c_fd, I2C_RDWR, &I2C_DATA_STROBE) < 0)
			printf("Failed to write strobe registers! %s\n", strerror(errno));
	}
}