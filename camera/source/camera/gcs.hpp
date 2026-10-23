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

#ifndef GCS_H
#define GCS_H

#include <inttypes.h>

#include "vcsm/vcsm.hpp"
#include "util/camera.hpp"

/* GPU Camera Stream
	Simple camera stream interface
	CameraParams covers all possible cameras, backends choose which parameters to support
*/

/* Input camera parameters */
struct GCS_CameraParams
{
	char *devName;
	CameraSensorConfig sensor;

	// Padding control
	//uint32_t padL, padR, padT; // TODO: Possible to implement horizontal padding?
	uint32_t padB;
	uint32_t padValue;
};

/* Actually used camera config */
struct GCS_CameraConfig
{
	uint32_t width;
	uint32_t height;
	uint32_t stride;
};

enum SENSOR_NUM
{
	SENSOR_ERROR = -1,
	SENSOR_INVALID = 0,
	SENSOR_OV9281,
	// Insert here, update ID below
	SENSOR_MAX
};

static const char* SENSOR_ID[SENSOR_MAX]
{
	"Invalid",
	"OV9281"
};


/* Opaque GPU Camera Stream structure */
struct GCS;

/* Returns SENSOR_NUM indicating a valid sensor, no valid sensor found, or a system error (e.g. no camera I2C) */
SENSOR_NUM gcs_probeSensor();

/* Creates a GCS (camera stream) instance. Uses params for sensor configuration, and immediately writes actually used width, height and stride into config. */
GCS *gcs_create(const GCS_CameraParams &params, GCS_CameraConfig &config);

/* Destroys GCS */
void gcs_destroy(GCS *gcs);

/* Start GCS (camera stream) */
uint8_t gcs_start(GCS *gcs);

/* Stop GCS (camera output) */
void gcs_stop(GCS *gcs);

/* Returns 1 for valid sensor, 0 for no valid sensor found, -1 for system error (no camera I2C) */
SENSOR_NUM gcs_getSensor(GCS *gcs);

/* Get the sensor area of current mode. Returns false if not streaming or sensor is not recognised. */
bool gcs_getSensorArea(GCS *gcs, uint16_t &sensorX, uint16_t &sensorY, uint16_t &minX, uint16_t &minY, uint16_t &maxX, uint16_t &maxY);

/* Returns error flag and resets it */
uint8_t gcs_readErrorFlag(GCS *gcs);

/* Updates given parameters, except for width and height, which are locked along with stride in gcs_create. */
void gcs_updateParameters(GCS *gcs, const GCS_CameraParams &params);

/* Returns once a new camera frame is available */
bool gcs_waitForFrameBuffer(GCS *gcs, uint32_t waitUS = 200000);

/* Returns whether there is a new camera frame available */
uint8_t gcs_hasFrameBuffer(GCS *gcs);

/* Returns the next camera frame. If no camera frame is available yet, blocks until there is. */
void* gcs_requestFrameBuffer(GCS *gcs);

/* Returns the most recent camera frame. If no camera frame is available yet, blocks until there is. */
void* gcs_requestLatestFrameBuffer(GCS *gcs, unsigned int *num);

/* Returns the VCSM buffer data of the given framebuffer. */
VCSM_BUFFER& gcs_getFrameBufferData(void *framebuffer);

/* Return requested Frame Buffer after processing is done. */
bool gcs_returnFrameBuffer(GCS *gcs, void *bufferHeader);

#endif
