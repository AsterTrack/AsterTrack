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

#include "gcs.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/time.h>
#include <math.h>
#include <algorithm>
#include <mutex>

#include "sensors/sensors.inl"

#define CHECK_STATUS(STATUS, MSG, ERRHANDLER) \
	if (STATUS) { \
		printf("%s (%d): %s\n", MSG, errno, strerror(errno)); \
		goto ERRHANDLER; \
	}

/* How many buffers the camera has to work with.
 * 3 minimum, but might introduce some latency as only 2 can be used alternatingly in the background while one processes.
 * For the v4l2 implementation, there are two options:
 * - take 2, and when n frames are skipped, n+1 will actually be skipped as it waits for a new most recent frame
 * - take n, and when m <= n-2 frames are skipped, it will slowly skip over them (0.1-0.2ms for each) until it caught up
 * 		but if n-1 or more are skipped, it will skip and THEN have to wait for the new most recent frame */
#define GCS_SIMUL_BUFFERS 10

struct GCS_FrameBuffer
{
	int index;
	VCSM_BUFFER vcsm;
	int32_t DMAFD = -1;
};

struct GCS
{
	// Flags
	uint8_t error = 0;
	bool started = false;
	bool frameWaiting = false;

	SENSOR_NUM sensor;

	// Camera parameters
	GCS_CameraParams params = {};
	GCS_CameraConfig config;

	int fd = -1;
	int bufferCount = 0;
	int processingBufferIndex = 0;
	struct GCS_FrameBuffer buffers[GCS_SIMUL_BUFFERS];

	std::mutex accessMutex;
};

int gcs_setParameter(GCS *gcs, uint32_t id, uint32_t value, uint32_t max)
{
	struct v4l2_queryctrl ctrlQuery;
	memset(&ctrlQuery, 0, sizeof ctrlQuery);
	ctrlQuery.id = id;
	int status = ioctl(gcs->fd, VIDIOC_QUERYCTRL, &ctrlQuery);
	if (status) return status;

	struct v4l2_control ctrl;
	memset(&ctrl, 0, sizeof ctrl);
	ctrl.id = id;
	if (max == 0)
		ctrl.value = ctrlQuery.maximum > value? value : ctrlQuery.maximum;
	else
		ctrl.value = value*ctrlQuery.maximum/max;
	if (ctrl.value < ctrlQuery.minimum)
		ctrl.value = ctrlQuery.minimum;
	status = ioctl(gcs->fd, VIDIOC_S_CTRL, &ctrl);
	return status;
}

int gcs_getParameterRange(GCS *gcs, uint32_t id, uint32_t &min, uint32_t &max)
{
	struct v4l2_queryctrl ctrlQuery;
	memset(&ctrlQuery, 0, sizeof ctrlQuery);
	ctrlQuery.id = id;
	int status = ioctl(gcs->fd, VIDIOC_QUERYCTRL, &ctrlQuery);
	min = ctrlQuery.minimum;
	max = ctrlQuery.maximum;
	return status;
}

int gcs_getParameterValue(GCS *gcs, uint32_t id, uint32_t &value)
{
	struct v4l2_control ctrl;
	memset(&ctrl, 0, sizeof ctrl);
	ctrl.id = id;
	int status = ioctl(gcs->fd, VIDIOC_G_CTRL, &ctrl);
	value = ctrl.value;
	return status;
}

int gcs_readPixelRate(GCS *gcs, int64_t &pixelRate)
{
	struct v4l2_ext_control ctrl = {};
	ctrl.id = V4L2_CID_PIXEL_RATE;
	struct v4l2_ext_controls ctrlQuery = {};
	ctrlQuery.ctrl_class = V4L2_CID_IMAGE_PROC_CLASS;
	ctrlQuery.which = V4L2_CTRL_WHICH_CUR_VAL;
	ctrlQuery.count = 1;
	ctrlQuery.controls = &ctrl;
	int status = ioctl(gcs->fd, VIDIOC_G_EXT_CTRLS, &ctrlQuery);
	pixelRate = ctrl.value64;
	return status;
}

SENSOR_NUM gcs_probeSensor()
{
	// Tell OV9281 camera to enable strobe (to force LEDs off as a temporary hardware fix)
	unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
	if (i2c_fd < 0)
	{
		printf("Failed to address camera over I2C! %s\n", strerror(errno));
		return SENSOR_ERROR;
	}
	SENSOR_NUM sensor = sensor_probe(i2c_fd);
	close(i2c_fd);
	return sensor;
}

GCS *gcs_create(const GCS_CameraParams &params, GCS_CameraConfig &config)
{
	int status;
	int o = 0;

	GCS *gcs = new GCS();
	gcs->params = params;
	gcs->processingBufferIndex = -1;

	gcs->sensor = gcs_probeSensor();

	gcs->fd = open(gcs->params.devName, O_RDWR);
	CHECK_STATUS(gcs->fd < 0, "Failed to open camera fd!", error_open);

//	printf("IOCTL %d: MSG: %d VIDIOC_LOG_STATUS, CONTENT: %d\n", gcs->fd, VIDIOC_LOG_STATUS, 0);
	//ioctl(gcs->fd, VIDIOC_LOG_STATUS);

	//printf("Successfully opened Camera %s\n", gcs->params.devName);

	struct v4l2_format format;
	memset(&format, 0, sizeof format);
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = gcs->params.sensor.width;
	format.fmt.pix.height = gcs->params.sensor.height;
	format.fmt.pix.pixelformat = sensor_isGrayscale(gcs->sensor, gcs->params)? V4L2_PIX_FMT_GREY : V4L2_PIX_FMT_YUV420;
	status = ioctl(gcs->fd, VIDIOC_TRY_FMT, &format);
	CHECK_STATUS(status, "Failed to test camera format!", error_format);
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//	printf("IOCTL %d: MSG: %d VIDIOC_S_FMT, CONTENT: %dx%d, %d, %d\n", gcs->fd, VIDIOC_S_FMT, format.fmt.pix_mp.width, format.fmt.pix_mp.height, format.fmt.pix_mp.pixelformat, format.fmt.pix_mp.field);
	status = ioctl(gcs->fd, VIDIOC_S_FMT, &format);
	CHECK_STATUS(status, "Failed to set camera format!", error_format);

	struct v4l2_format getFormat;
	memset(&getFormat, 0, sizeof getFormat);
	getFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//	printf("IOCTL %d: MSG: %d VIDIOC_G_FMT, CONTENT: %d\n", gcs->fd, VIDIOC_LOG_STATUS, getFormat.type);
	status = ioctl(gcs->fd, VIDIOC_G_FMT, &getFormat);
	CHECK_STATUS(status, "Failed to get camera format!", error_format);

	printf("Format: %ux%u, stride %u, size %u\n",
		getFormat.fmt.pix.width, getFormat.fmt.pix.height,
		getFormat.fmt.pix.bytesperline, getFormat.fmt.pix.sizeimage);
	gcs->config = config = {
		.width = getFormat.fmt.pix.width,
		.height = getFormat.fmt.pix.height,
		.stride = getFormat.fmt.pix.bytesperline,
	};

	// Notify V4L2 we'll allocate buffers externally (through VCSM) and import them as DMABUFs
	struct v4l2_requestbuffers reqBuf;
	memset(&reqBuf, 0, sizeof reqBuf);
	reqBuf.count = GCS_SIMUL_BUFFERS;
	reqBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqBuf.memory = V4L2_MEMORY_DMABUF;
//	printf("IOCTL %d: MSG: %d VIDIOC_REQBUFS, CONTENT: %dcnt, %dtype, %dmem\n", gcs->fd, VIDIOC_REQBUFS, reqBuf.count, reqBuf.type, reqBuf.memory);
	status = ioctl(gcs->fd, VIDIOC_REQBUFS, &reqBuf);
	CHECK_STATUS(status, "Failed to allocate frame buffers!", error_buffers);

	gcs->bufferCount = reqBuf.count;
//	printf("%u buffers requested, V4L2 returned %u bufs, type %d (exp %d).\n", GCS_SIMUL_BUFFERS, reqBuf.count, reqBuf.type, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	int i; // Used to track progess in error handling
	for (i = 0; i < gcs->bufferCount; i++)
	{
		// Query buffer to get exact size
		struct v4l2_buffer buffer;
		memset(&buffer, 0, sizeof buffer);
		buffer.index = i;
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_DMABUF;
//		printf("IOCTL %d: MSG: %d VIDIOC_QUERYBUF, CONTENT: (%d, %d, %d, %d, %p)\n", gcs->fd, VIDIOC_QUERYBUF, 
//			buffer.index, buffer.type, buffer.memory, buffer.length, buffer.m.planes);
		status = ioctl(gcs->fd, VIDIOC_QUERYBUF, &buffer);
		CHECK_STATUS(status, "Failed to query buffer!", error_bufquery);

		gcs->buffers[i].index = i;

		uint32_t desiredBufferSize = (gcs->config.height+gcs->params.padB) * gcs->config.stride;
		gcs->buffers[i].vcsm = vcsm_malloc(std::max(buffer.length, desiredBufferSize));
		CHECK_STATUS(gcs->buffers[i].vcsm.fd < 0, "Failed to allocate vcsm memory!", error_bufmalloc);

		gcs->buffers[i].DMAFD = dup(gcs->buffers[i].vcsm.fd);
	}

	for (int j = 0; j < gcs->bufferCount; j++)
	{
		vcsm_lock(gcs->buffers[j].vcsm);
		uint8_t *ptr = (uint8_t*)gcs->buffers[j].vcsm.mem;
		memset(ptr + gcs->config.height * gcs->config.stride, gcs->params.padValue, gcs->params.padB * gcs->config.stride);
		vcsm_unlock(gcs->buffers[j].vcsm);
//		printf("Deallocate %d. \n", gcs->buffers[j].vcsm.fd);
	}

////	printf("Set up GCS!\n");

	return gcs;

error_bufmalloc:
error_bufquery:
	// Free already allocated VC memory buffers
	for (int j = 0; j < i; j++)
	{
		// Close the exported DMABUF FD
		close(gcs->buffers[i].DMAFD);
		// Free the VCSM memory (closes the original VCSM FD)
		vcsm_free(gcs->buffers[i].vcsm);
		// With both DMABUF and VCSM FD freed, memory will be deallocated
	}
	reqBuf.count = 0;
//	printf("IOCTL %d: MSG: %d VIDIOC_REQBUFS, CONTENT: %dcnt, %dtype, %dmem\n", gcs->fd, VIDIOC_REQBUFS, reqBuf.count, reqBuf.type, reqBuf.memory);
	status = ioctl(gcs->fd, VIDIOC_REQBUFS, &reqBuf);
error_buffers:
error_format:
	close(gcs->fd);
error_open:
	delete gcs;
error_alloc:
	return NULL;
}

void gcs_destroy(GCS *gcs)
{
	if (!gcs) return;

	// Stop gcs
	gcs_stop(gcs);

	// Deallocate DMABUF structures in V4L2
	struct v4l2_requestbuffers reqBuf;
	memset(&reqBuf, 0, sizeof reqBuf);
	reqBuf.count = 0;
	reqBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqBuf.memory = V4L2_MEMORY_DMABUF;
//	printf("IOCTL %d: MSG: %d VIDIOC_REQBUFS, CONTENT: %dcnt, %dtype, %dmem\n", gcs->fd, VIDIOC_REQBUFS, reqBuf.count, reqBuf.type, reqBuf.memory);
	int status = ioctl(gcs->fd, VIDIOC_REQBUFS, &reqBuf);
	if (status == -1)
		printf("Failed to deallocate frame buffers! Received error %d: %s\n", errno, strerror(errno));

	// Free VC memory buffers
	for (int i = 0; i < gcs->bufferCount; i++)
	{
		// Close the exported DMABUF FD
		close(gcs->buffers[i].DMAFD);
		// Free the VCSM memory (closes the original VCSM FD)
		vcsm_free(gcs->buffers[i].vcsm);
		// With both DMABUF and VCSM FD freed, memory will be deallocated
	}

	// Close camera
	close(gcs->fd);

	// Free allocated resources
	delete gcs;
}

uint8_t gcs_start(GCS *gcs)
{
	// Ensure GCS is stopped first
	gcs_stop(gcs);

	// Init
	std::unique_lock lock(gcs->accessMutex);
	gcs->error = 0;
	gcs->started = true;

	// Enqueue all buffers
	int i;
	for (i = 0; i < gcs->bufferCount; i++)
	{
		struct v4l2_buffer buffer;
		memset(&buffer, 0, sizeof buffer);
		buffer.index = i;
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_DMABUF;
		buffer.m.fd = gcs->buffers[i].DMAFD;
		if (ioctl(gcs->fd, VIDIOC_QBUF, &buffer) >= 0)
			continue;

		printf("gcs_start: Failed to enqueue buffer!");
		for (int j = 0; j < i; j++)
		{
			struct v4l2_buffer buffer;
			memset(&buffer, 0, sizeof buffer);
			buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buffer.memory = V4L2_MEMORY_MMAP;
			if (ioctl(gcs->fd, VIDIOC_DQBUF, &buffer))
				printf("Failed to dequeue buffer on error handling: %s!\n", strerror(errno));
		}
		gcs->error = 1;
		gcs->started = false;
		return 1;
	}

	// Start streaming
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//	printf("IOCTL %d: MSG: %d VIDIOC_STREAMON, CONTENT: x\n", gcs->fd, VIDIOC_STREAMON);
	if (ioctl(gcs->fd, VIDIOC_STREAMON, &type) < 0)
	{
		printf("gcs_start: Failed to start streaming!");
		gcs->error = 1;
		gcs->started = false;
		return 2;
	}

	// Attempt to set parameters
	gcs_updateParameters(gcs, gcs->params);

	return 0;
}

void gcs_stop(GCS *gcs)
{
	std::unique_lock lock(gcs->accessMutex);
	// Do not clear gcs->error

	//printf("Stop Streaming in GCS!\n");
	// Dequeue all buffers
	/*for (int i = 0; i < gcs->bufferCount; i++)
	{
		struct v4l2_buffer buffer;
		memset(&buffer, 0, sizeof buffer);
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
		if (ioctl(gcs->fd, VIDIOC_DQBUF, &buffer))
			printf("Failed to dequeue buffer: %s\n", strerror(errno));
		//else
		//	printf("Dequeue buffer %d!\n", buffer.index);
	}*/

	// Stop streaming
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // Doesn't do anything afaik
	//printf("IOCTL %d: MSG: %d VIDIOC_STREAMOFF, CONTENT: x\n", gcs->fd, VIDIOC_STREAMOFF);
	if (ioctl(gcs->fd, VIDIOC_STREAMOFF, &type) < 0)
	{
		printf("gcs_stop: streamoff ioctl failed!");
		gcs->error = 1;
		return;
	}

	if (!gcs->started)
		return;
	gcs->started = false;

	// Directly access camera I2C (i2c-vs)
	unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
	if (i2c_fd < 0)
		printf("Failed to open camera I2C fd! %s \n", strerror(errno));
	else
	{
	 	sensor_deconfigure(gcs->sensor, i2c_fd, gcs->params, gcs->config);
		close(i2c_fd);
	}
}

SENSOR_NUM gcs_getSensor(GCS *gcs)
{
	return gcs->sensor;
}

bool gcs_getSensorArea(GCS *gcs, uint16_t &sensorX, uint16_t &sensorY, uint16_t &minX, uint16_t &minY, uint16_t &maxX, uint16_t &maxY)
{
	if (!gcs->started)
	{ // Since we're relying on V4L2 driver to select closest existing mode
		printf("Cannot get used sensor area before stream start!\n");
		return false;
	}
	if (gcs->sensor == SENSOR_INVALID)
	{
		printf("No sensor detected to determine area!\n");
		return false;
	}
	return sensor_getSensorArea(gcs->sensor, gcs->params, gcs->config, sensorX, sensorY, minX, minY, maxX, maxY);
}

uint8_t gcs_readErrorFlag(GCS *gcs)
{
	uint8_t flag = gcs->error;
	gcs->error = 0;
	return flag;
}

void gcs_updateParameters(GCS *gcs, const GCS_CameraParams &params)
{
	gcs->params = params;

	// Directly access camera I2C (i2c-vs)
	unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
	if (i2c_fd < 0)
		printf("Failed to open camera I2C fd! %s \n", strerror(errno));
	else
	{
	 	sensor_configure(gcs->sensor, i2c_fd, gcs->params, gcs->config);
		close(i2c_fd);
	}

	int status;
	if (gcs->params.sensor.sync != SENSOR_EXTERNAL_TRIGGER)
	{
		uint32_t minHBlank, maxHBlank;
		if ((status = gcs_getParameterRange(gcs, V4L2_CID_HBLANK, minHBlank, maxHBlank)))
			printf("Failed to get hblank range!\n");

		uint32_t minVBlank, maxVBlank;
		if ((status = gcs_getParameterRange(gcs, V4L2_CID_VBLANK, minVBlank, maxVBlank)))
			printf("Failed to get vblank range!\n");

		int64_t pixelRate;
		if ((status = gcs_readPixelRate(gcs, pixelRate)))
			printf("Failed to get pixel rate!\n");

		// Calculate vblank to achieve the desired FPS
		double magicScaleFactor = sensor_getVBlankScale(gcs->sensor, gcs->params, gcs->config);
		int64_t desiredVBlank = magicScaleFactor * (pixelRate * gcs->params.sensor.frameTimeNS)
			/ ((uint64_t)1000000000 * (gcs->config.width+minHBlank))
			- gcs->config.height;
		uint32_t setVBlank = std::clamp<int64_t>(desiredVBlank, minVBlank, maxVBlank);
		float predFrameTimeNS = (uint64_t)((gcs->config.width+minHBlank) * (gcs->config.height+setVBlank)) * 1000000000
			/ (magicScaleFactor * pixelRate);
		printf("Setting vblank to %d (desired %lld) to get FPS %.2f (desired %.2f)!\n",
			setVBlank, desiredVBlank, 1000000000.0/predFrameTimeNS, 1000000000.0/gcs->params.sensor.frameTimeNS);
		gcs->params.sensor.frameTimeNS = predFrameTimeNS;

		// Set hblank (extra time between lines)
		if ((status = gcs_setParameter(gcs, V4L2_CID_HBLANK, minHBlank, 0)))
			printf("Failed to set hblank to minimum!\n");

		// Set vblank (time between last frame and new frame - used to set FPS)
		if ((status = gcs_setParameter(gcs, V4L2_CID_VBLANK, setVBlank, 0)))
			printf("Failed to set vblank to %d!\n", setVBlank);
	}
	else
	{
		// Set hblank (extra time between lines)
		if ((status = gcs_setParameter(gcs, V4L2_CID_HBLANK, 0, 0)))
			printf("Failed to set hblank to minimum!\n");

		// Set vblank (time between last frame and new frame - used to set FPS)
		if ((status = gcs_setParameter(gcs, V4L2_CID_VBLANK, 0, 0)))
			printf("Failed to set vblank to minimum!\n");
	}

	if ((status = gcs_setParameter(gcs, V4L2_CID_ANALOGUE_GAIN, gcs->params.sensor.analogGain, 16)))
		printf("Failed to set analog camera gain!\n");
	if ((status = gcs_setParameter(gcs, V4L2_CID_EXPOSURE, sensor_getExposure(gcs->sensor, gcs->params, gcs->config), 0)))
		printf("Failed to set camera exposure / shutter speed!\n");
}

/* Returns once a new camera frame is available */
bool gcs_waitForFrameBuffer(GCS *gcs, uint32_t waitUS)
{
	if (gcs->frameWaiting)
		return gcs->frameWaiting;

	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = waitUS;

	// Return values for select indicating events in camera file descriptor
	fd_set readFD;
	FD_ZERO(&readFD);
	FD_SET(gcs->fd, &readFD);

	std::unique_lock lock(gcs->accessMutex);

	// Wait for changes from camera file descriptor
	if (select(gcs->fd + 1, &readFD, NULL, NULL, &timeout) < 0)
	{
		printf("gcs_waitForFrameBuffer: select ioctl failed!");
		gcs->error = 1;
		return 0;
	}

	gcs->frameWaiting = FD_ISSET(gcs->fd, &readFD);
	return gcs->frameWaiting;
}

uint8_t gcs_hasFrameBuffer(GCS *gcs)
{
	return gcs_waitForFrameBuffer(gcs, 0);
}

void* gcs_requestFrameBuffer(GCS *gcs)
{
	if (!gcs->frameWaiting && !gcs_waitForFrameBuffer(gcs))
		return NULL;

	std::unique_lock lock(gcs->accessMutex);

	if (!gcs->frameWaiting)
		return NULL;
	gcs->frameWaiting = false;

	struct v4l2_buffer buffer;
	memset(&buffer, 0, sizeof buffer);
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_DMABUF;

	if (ioctl(gcs->fd, VIDIOC_DQBUF, &buffer) < 0)
	{
		printf("gcs_waitForFrameBuffer: Failed to dequeue buffer!");
		gcs->error = 1;
		return NULL;
	}
	if (buffer.index >= GCS_SIMUL_BUFFERS)
	{
		printf("gcs_waitForFrameBuffer: Dequeue returned invalid buffer!");
		gcs->error = 1;
		return NULL;
	}

	return &gcs->buffers[buffer.index];
}

void* gcs_requestLatestFrameBuffer(GCS *gcs, unsigned int *num)
{
	if (!gcs->frameWaiting && !gcs_waitForFrameBuffer(gcs))
		return NULL;

	std::unique_lock lock(gcs->accessMutex);

	if (!gcs->frameWaiting)
		return NULL;
	gcs->frameWaiting = false;

	bool waiting = true;
	void* bufferHeader = NULL;
	int status;
	*num = 0;

	// Prepare V4L2 buffer deque
	struct v4l2_buffer buffer;
	memset(&buffer, 0, sizeof buffer);
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_DMABUF;

	// Prepare select call
	struct timeval timeout;
	fd_set readFD;

	while (waiting)
	{
		// Deque a buffer
		status = ioctl(gcs->fd, VIDIOC_DQBUF, &buffer);
		CHECK_STATUS(status, "Failed to dequeue buffer!", error_abort);
		CHECK_STATUS(buffer.index >= GCS_SIMUL_BUFFERS || buffer.index < 0, "Dequeue returned invalid buffer!", error_abort);
		bufferHeader = &gcs->buffers[buffer.index];
		*num += 1;

		// Check for further buffers
		timeout.tv_sec = 0;
		timeout.tv_usec = 0;
		FD_ZERO(&readFD);
		FD_SET(gcs->fd, &readFD);
		status = select(gcs->fd + 1, &readFD, NULL, NULL, &timeout);
		CHECK_STATUS(status == -1, "Failed select!", error_abort);
		waiting = FD_ISSET(gcs->fd, &readFD);

		if (waiting)
		{
			status = ioctl(gcs->fd, VIDIOC_QBUF, &buffer);
			CHECK_STATUS(status, "Failed to requeue frame buffer when next one is already waiting!", error_abort);
		}
	}

	return bufferHeader;

error_abort:
	gcs->error = 1;
	return NULL;
}

VCSM_BUFFER& gcs_getFrameBufferData(void *framebuffer)
{
	return ((struct GCS_FrameBuffer*)framebuffer)->vcsm;
}

bool gcs_returnFrameBuffer(GCS *gcs, void *bufferHeader)
{
	if (!bufferHeader) return true;
	std::unique_lock lock(gcs->accessMutex);
	struct v4l2_buffer buffer;
	memset(&buffer, 0, sizeof buffer);
	buffer.index = ((struct GCS_FrameBuffer*)bufferHeader)->index;
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_DMABUF;
	buffer.m.fd = gcs->buffers[buffer.index].DMAFD;
	if (ioctl(gcs->fd, VIDIOC_QBUF, &buffer) < 0)
	{
		printf("gcs_returnFrameBuffer: Failed to requeue frame buffer after processing!");
		gcs->error = 1;
		return false;
	}
	return true;
}
