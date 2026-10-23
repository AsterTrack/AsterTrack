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
#include "ov9281.inl"

static SENSOR_NUM sensor_probe(unsigned int i2c_fd)
{
	SENSOR_NUM sensor = SENSOR_INVALID;

	// Identify OV9281
	int32_t CHIP_ID = Read16Bit(i2c_fd, 0x300A);		
	if (CHIP_ID == 0x9281)
		sensor = SENSOR_OV9281;

	return sensor;
}

static bool sensor_isGrayscale(SENSOR_NUM sensor, const GCS_CameraParams &params)
{
	if (sensor == SENSOR_OV9281)
		return true;
	// False currently means YUV420
	return false;
}

static bool sensor_getSensorArea(SENSOR_NUM sensor, const GCS_CameraParams &params, const GCS_CameraConfig &config, uint16_t &sensorX, uint16_t &sensorY, uint16_t &minX, uint16_t &minY, uint16_t &maxX, uint16_t &maxY)
{
	if (sensor == SENSOR_OV9281)
		return ov9281_getSensorArea(params, config, sensorX, sensorY, minX, minY, maxX, maxY);
	return false;
}

static double sensor_getVBlankScale(SENSOR_NUM sensor, const GCS_CameraParams &params, const GCS_CameraConfig &config)
{ // Unsure why this is needed
	if (sensor == SENSOR_OV9281)
		return 0.96;
	return 1.0;
}

static uint32_t sensor_getExposure(SENSOR_NUM sensor, const GCS_CameraParams &params, const GCS_CameraConfig &config)
{
	if (sensor == SENSOR_OV9281)
		return params.sensor.exposureUS/OV9281_TIME_FACTOR + 1;
	return 0;
}

static void sensor_configure(SENSOR_NUM sensor, unsigned int i2c_fd, const GCS_CameraParams &params, const GCS_CameraConfig &config)
{
	if (sensor == SENSOR_OV9281)
		ov9281_configure(i2c_fd, params, config);
}

static void sensor_deconfigure(SENSOR_NUM sensor, unsigned int i2c_fd, const GCS_CameraParams &params, const GCS_CameraConfig &config)
{
	if (sensor == SENSOR_OV9281)
		ov9281_deconfigure(i2c_fd, params, config);
}
