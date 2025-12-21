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

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <linux/videodev2.h>

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
	uint8_t started = 0; // Specifies that stream have started
	uint8_t error = 0;
	uint8_t frameWaiting = 0;

	// Camera parameters
	GCS_CameraParams *cameraParams = NULL;

	int fd = -1;
	int bufferCount = 0;
	int processingBufferIndex = 0;
	struct GCS_FrameBuffer buffers[GCS_SIMUL_BUFFERS];

	std::mutex accessMutex;
};

int gcs_setParameter(GCS *gcs, uint32_t id, uint32_t value, uint32_t max)
{
	int status;

	struct v4l2_queryctrl ctrlQuery;
	memset(&ctrlQuery, 0, sizeof ctrlQuery);
	ctrlQuery.id = id;
	status = ioctl(gcs->fd, VIDIOC_QUERYCTRL, &ctrlQuery);
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

/* Returns 1 for valid sensor, 0 for no valid sensor found, -1 for system error (no camera I2C) */
int gcs_findCamera()
{
	// Tell OV9281 camera to enable strobe (to force LEDs off as a temporary hardware fix)
	unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
	if (i2c_fd < 0)
	{
		printf("Failed to address camera over I2C! %s\n", strerror(errno));
		return -1;
	}
	
	bool validSensor = false;

	// Identify OV9281
	int32_t CHIP_ID = Read16Bit(i2c_fd, 0x300A);		
	if (CHIP_ID == 0x9281)
		validSensor = true;
	else
		printf("Read wrong chip ID! ID is %x (expected %x)\n", CHIP_ID, 0x9281);
	
	close(i2c_fd);
	return validSensor? 1 : 0;
}


GCS *gcs_create(GCS_CameraParams *cameraParams)
{
	int status;
	int o = 0;

	GCS *gcs = new GCS();
	gcs->cameraParams = cameraParams;
	gcs->processingBufferIndex = -1;

	gcs->fd = open(gcs->cameraParams->devName, O_RDWR);
	CHECK_STATUS(gcs->fd < 0, "Failed to open camera fd!", error_open);

//	printf("IOCTL %d: MSG: %d VIDIOC_LOG_STATUS, CONTENT: %d\n", gcs->fd, VIDIOC_LOG_STATUS, 0);
	//ioctl(gcs->fd, VIDIOC_LOG_STATUS);

	//printf("Successfully opened Camera %s\n", gcs->cameraParams->devName);

	struct v4l2_format format;
	memset(&format, 0, sizeof format);
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = gcs->cameraParams->width;
	format.fmt.pix.height = gcs->cameraParams->height;
	format.fmt.pix.pixelformat = gcs->cameraParams->grayscale? V4L2_PIX_FMT_GREY : V4L2_PIX_FMT_YUV420;
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
	gcs->cameraParams->width = getFormat.fmt.pix.width;
	gcs->cameraParams->height = getFormat.fmt.pix.height;
	gcs->cameraParams->stride = getFormat.fmt.pix.bytesperline;

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

		gcs->buffers[i].vcsm = vcsm_malloc(buffer.length);
		CHECK_STATUS(gcs->buffers[i].vcsm.fd < 0, "Failed to allocate vcsm memory!", error_bufmalloc);

		gcs->buffers[i].DMAFD = dup(gcs->buffers[i].vcsm.fd);
	}

////	printf("Set up GCS!\n");

	return gcs;

error_bufimport:
error_bufexport:
	o = 1;
error_bufmalloc:
	for (int j = 0; j < i+o; j++)
	{
		vcsm_free(gcs->buffers[i].vcsm);
//		printf("Deallocate %d. \n", gcs->buffers[i].vcsm.fd);
	}
error_bufquery:
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

/* Start GCS (camera stream) */
uint8_t gcs_start(GCS *gcs)
{
	// Ensure GCS is stopped first
	gcs_stop(gcs);

	// Init
	std::unique_lock lock(gcs->accessMutex);
	gcs->error = 0;
	gcs->started = 1;

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
		gcs->started = 0;
		return 1;
	}

	// Start streaming
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//	printf("IOCTL %d: MSG: %d VIDIOC_STREAMON, CONTENT: x\n", gcs->fd, VIDIOC_STREAMON);
	if (ioctl(gcs->fd, VIDIOC_STREAMON, &type) < 0)
	{
		printf("gcs_start: Failed to start streaming!");
		gcs->error = 1;
		gcs->started = 0;
		return 2;
	}

	// Attempt to set parameters
	gcs_updateParameters(gcs);

	return 0;
}

/* Stop GCS (camera output). Stops watchdog and disabled MMAL camera */
void gcs_stop(GCS *gcs)
{
	if (gcs->started == 0) return;
	std::unique_lock lock(gcs->accessMutex);
	gcs->started = 0;
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

	// Directly access camera I2C (i2c-vs)
	if (gcs->cameraParams->extTrig)
	{ // TODO: Make sure that OV9281 driver is loaded
		unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
		if (i2c_fd < 0)
			printf("Failed to open camera I2C fd! %s \n", strerror(errno));
		else
		{ // Now modify camera streaming behaviour

			// Disable external trigger mode so that a sync pulse without this camera set to stream will not make the camera take frames
			unsigned char PSV_CTRL[3] = { 0x4f, 0x00, 0x00 };		// Enabling FSIN to wake camera and expose

			struct i2c_msg I2C_MSG[] = {
				{ 0x60, 0, sizeof(PSV_CTRL), PSV_CTRL }
			};
			struct i2c_rdwr_ioctl_data I2C_DATA_TRIG = { I2C_MSG+0, 1 };
			if (gcs->cameraParams->extTrig && ioctl(i2c_fd, I2C_RDWR, &I2C_DATA_TRIG) < 0)
				printf("Failed to write extTrig registers! %s\n", strerror(errno));
			close(i2c_fd);
		}
	}
	
	// Stop streaming
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // Doesn't do anything afaik
	//printf("IOCTL %d: MSG: %d VIDIOC_STREAMOFF, CONTENT: x\n", gcs->fd, VIDIOC_STREAMOFF);
	if (ioctl(gcs->fd, VIDIOC_STREAMOFF, &type) < 0)
	{
		printf("gcs_stop: streamoff ioctl failed!");
		gcs->error = 1;
		return;
	}
}

/* Returns error flag and resets it */
uint8_t gcs_readErrorFlag(GCS *gcs)
{
	uint8_t flag = gcs->error;
	gcs->error = 0;
	return flag;
}

/* Updates internal parameters if possible */
void gcs_updateParameters(GCS *gcs)
{
	int status;
	unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
	if (i2c_fd < 0)
		printf("Failed to open camera I2C fd! %s \n", strerror(errno));

	if (i2c_fd >= 0)
	{
		// Set hblank (extra time between lines)
		// TODO: What does this even mean for a global shutter sensor?
		// These are the minimums of the OV9282 driver, and what the OV9281 set as constant
		#define MIN_HBLANK_1280x800 176 // HTS of (176+1280)/2
		#define MIN_HBLANK_640x400 816 // HTS of (816+640)/2
		if ((status = gcs_setParameter(gcs, V4L2_CID_HBLANK, 0, 0)))
			printf("Failed to set hblank to minimum!\n");

		// Set vblank (time between last frame and new frame - used to set FPS)
		// TODO: respect gcs->cameraParams->fps if NOT gcs->cameraParams->extTrig
		// These are the minimums of the OV9282 driver, and what the OV9281 set as constant
		#define MIN_VBLANK_1280x800 110 // VTS of 110+800
		#define MIN_VBLANK_640x400 22 // VTS of 22+400
		if ((status = gcs_setParameter(gcs, V4L2_CID_VBLANK, 0, 0)))
			printf("Failed to set vblank to minimum!\n");
	}

	// Digital camera gain not supported by OV9281 camera / driver
	//if ((status = gcs_setParameter(gcs, V4L2_CID_GAIN, gcs->cameraParams->digitalGain, 16)))
	//	printf("Failed to set digital camera gain!\n");
	if ((status = gcs_setParameter(gcs, V4L2_CID_ANALOGUE_GAIN, gcs->cameraParams->analogGain, 16)))
		printf("Failed to set analog camera gain!\n");
	if ((status = gcs_setParameter(gcs, V4L2_CID_EXPOSURE, gcs->cameraParams->shutterSpeed, 0)))
		printf("Failed to set camera exposure / shutter speed!\n");

	// Directly access camera I2C (i2c-vs)
	if (i2c_fd >= 0)
	{ // Now modify camera streaming behaviour

		if (gcs->cameraParams->extTrig)
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
		/* else
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

		if (gcs->cameraParams->strobe)
		{
			// Set strobe and ILPWM to output, leave FSIN as input
			unsigned char SC_CTRL_06[3] = { 0x30, 0x06, 0b00001100 };
			// Set strobe_frame_pattern - default 0xA5
			// Set strobe_frame_shift [0-7] (offset after SOF) - default 5
			// Set strobe_frame_shift [24-30], shift direction [31] (offset after SOF) - default 0
			// Set strobe_frame_span [0-7] (pulse width) in steps - default 26
			uint8_t offset_low = (uint8_t)std::min(255, std::abs(gcs->cameraParams->strobeOffset));
			uint8_t offset_high = (uint8_t)((gcs->cameraParams->strobeOffset<0? 0x80 : 0x00) + (std::abs(gcs->cameraParams->strobeOffset)>>8));
			uint8_t width_low = (uint8_t)(gcs->cameraParams->strobeLength&0xFF);
			uint8_t width_high = (uint8_t)((gcs->cameraParams->strobeLength>>8)&0xFF);
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
			if (gcs->cameraParams->strobe && ioctl(i2c_fd, I2C_RDWR, &I2C_DATA_STROBE) < 0)
				printf("Failed to write strobe registers! %s\n", strerror(errno));
		}

		close(i2c_fd);
	}
}

/* Returns once a new camera frame is available */
uint8_t gcs_waitForFrameBuffer(GCS *gcs, uint32_t waitUS)
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

/* Returns whether there is a new camera frame available */
uint8_t gcs_hasFrameBuffer(GCS *gcs)
{
	return gcs_waitForFrameBuffer(gcs, 0);
}

/* Returns the next camera frame. If no camera frame is available yet, blocks until there is. */
void* gcs_requestFrameBuffer(GCS *gcs)
{
	if (!gcs->frameWaiting && !gcs_waitForFrameBuffer(gcs))
		return NULL;

	std::unique_lock lock(gcs->accessMutex);

	if (!gcs->frameWaiting)
		return NULL;
	gcs->frameWaiting = 0;

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

/* Returns the most recent camera frame. Assumes gcs_waitForFrameBuffer or gcs_hasFrameBuffer has returned true (frameWaiting is true). */
void* gcs_requestLatestFrameBuffer(GCS *gcs, unsigned int *num)
{
	if (!gcs->frameWaiting && !gcs_waitForFrameBuffer(gcs))
		return NULL;

	std::unique_lock lock(gcs->accessMutex);

	if (!gcs->frameWaiting)
		return NULL;
	gcs->frameWaiting = 0;

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

/* Returns the VCSM buffer data of the given framebuffer. */
VCSM_BUFFER& gcs_getFrameBufferData(void *framebuffer)
{
	return ((struct GCS_FrameBuffer*)framebuffer)->vcsm;
}

/* Return requested Franme Buffer after processing is done. */
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
