/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

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

/* GPU Camera Stream
	Simple camera stream interface
	CameraParams covers all possible cameras, backends choose which parameters to support
	Has V4L2 (OV9281 specific) backend
*/

/* Camera parameters passed for MMAL camera set up */
struct GCS_CameraParams
{
	char *devName;
	uint16_t width;
	uint16_t height;
	uint32_t stride;
	uint16_t fps;
	uint32_t shutterSpeed;
	uint32_t iso;
	uint8_t digitalGain;
	uint8_t analogGain;
	uint8_t grayscale;
	uint8_t extTrig;
	uint8_t strobe;
	int32_t strobeOffset;
	uint32_t strobeLength;
	uint8_t disableEXP;
	uint8_t disableAWB;
	uint32_t disableISPBlocks;
};

/* Disable ISP Blocks */
// https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=175711
// (1 << 2); // Black Level Compensation
// (1 << 3); // Lens Shading
// (1 << 5); // White Balance Gain
// (1 << 7); // Defective Pixel Correction
// (1 << 9); // Crosstalk
// (1 << 18); // Gamma
// (1 << 22); // Sharpening
// (1 << 24); // Some Color Conversion

/* Opaque GPU Camera Stream structure */
struct GCS;

void gcs_init();

/* Creates a GCS (camera stream) instance. Requires cameraParams to stay allocated until gcs_destroy is called. It is used to update actual parameters, such as width, and stride, as well as update parameters for gcs_updateParameters */
GCS *gcs_create(GCS_CameraParams *cameraParams);

/* Destroys GCS */
void gcs_destroy(GCS *gcs);

/* Start GCS (camera stream). Enables MMAL camera and starts watchdog */
uint8_t gcs_start(GCS *gcs);

/* Stop GCS (camera output). Stops watchdog and disabled MMAL camera */
void gcs_stop(GCS *gcs);

/* Returns error flag and resets it */
uint8_t gcs_readErrorFlag(GCS *gcs);

/* Updates parameters from linked GCS_CameraParams struct */
void gcs_updateParameters(GCS *gcs);

/* Returns once a new camera frame is available */
uint8_t gcs_waitForFrameBuffer(GCS *gcs, uint32_t waitUS = 200000);

/* Returns whether there is a new camera frame available */
uint8_t gcs_hasFrameBuffer(GCS *gcs);

/* Returns the next camera frame. If no camera frame is available yet, blocks until there is. */
void* gcs_requestFrameBuffer(GCS *gcs);

/* Returns the most recent camera frame. Assumes gcs_waitForFrameBuffer or gcs_hasFrameBuffer has returned true (frameWaiting is true). */
void* gcs_requestLatestFrameBuffer(GCS *gcs, unsigned int *num);

/* Returns the VCSM buffer data of the given framebuffer. */
VCSM_BUFFER& gcs_getFrameBufferData(void *framebuffer);

/* Return requested Frane Buffer after processing is done.
 * Has to be called before a new frame buffer can be requested. */
bool gcs_returnFrameBuffer(GCS *gcs, void *bufferHeader);

#endif
