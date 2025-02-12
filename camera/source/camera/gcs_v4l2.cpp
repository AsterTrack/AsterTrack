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

#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <linux/videodev2.h>
#include "interface/vcsm/user-vcsm.h"
#include "interface/vcos/vcos_mutex.h"

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
	uint32_t size;
	void *mem;
	int32_t DMAFD;
	uint32_t vcsmHandle;
	uint32_t vcsmVCHandle;
	uint32_t vcsmVCMem;
};
struct GCS
{
	// Flags
	uint8_t started; // Specifies that stream have started
	uint8_t error;
	uint8_t frameWaiting;

	// Camera parameters
	GCS_CameraParams *cameraParams;

	int fd;
	int bufferCount;
	int processingBufferIndex;
	struct GCS_FrameBuffer buffers[GCS_SIMUL_BUFFERS];

	VCOS_MUTEX_T accessMutex;
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

	struct i2c_msg I2C_MSG[] = {
		{ 0x60, 0, sizeof(REG_HIGH), REG_HIGH },
		{ 0x60, I2C_M_RD, 1, val_msg+1 },
		{ 0x60, 0, sizeof(REG_LOW), REG_LOW },
		{ 0x60, I2C_M_RD, 1, val_msg+0 },
	};
	struct i2c_rdwr_ioctl_data I2C_READ = { I2C_MSG, 4 };
	if (ioctl(fd, I2C_RDWR, &I2C_READ) < 0)
		printf("Failed to read register! %s\n", strerror(errno));
	else
		return value;
	return -1;
}

void gcs_init()
{
	// Tell OV9281 camera to enable strobe (to force LEDs off as a temporary hardware fix)
	unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
	if (i2c_fd < 0)
		printf("Failed to adress camera over I2C! %s \n", strerror(errno));
	else
	{ // Now modify camera streaming behaviour

		int32_t CHIP_ID = Read16Bit(i2c_fd, 0x300A);		
		if (CHIP_ID != 0x9281)
			printf("Read wrong chip ID! ID is %d (expected %d)", CHIP_ID, 0x9281);

		close(i2c_fd);
	}
}


GCS *gcs_create(GCS_CameraParams *cameraParams)
{
	int status;
	int o = 0;

	GCS *gcs = new GCS();
	memset(gcs, 0, sizeof *gcs);
	gcs->cameraParams = cameraParams;
	gcs->processingBufferIndex = -1;

	// Access mutex
	status = vcos_mutex_create(&gcs->accessMutex, "gcs-mutex");
	CHECK_STATUS(status != VCOS_SUCCESS, "Failed to create mutex", error_mutex);

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

	// Init VCSM to exchange buffers with
	vcsm_init_ex(1, -1);
//	vcsm_init();

	// Allocate buffers through VCSM directly
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
		gcs->buffers[i].size = buffer.length;

		gcs->buffers[i].vcsmHandle = vcsm_malloc(buffer.length, "Buf");
		CHECK_STATUS(gcs->buffers[i].vcsmHandle == 0, "Failed to allocate vcsm memory!", error_bufmalloc);

		gcs->buffers[i].vcsmVCHandle = vcsm_vc_hdl_from_hdl(gcs->buffers[i].vcsmHandle);
		CHECK_STATUS(gcs->buffers[i].vcsmVCHandle == 0, "Failed to change VCSM user handle to VSCM VC handle!", error_bufimport);

		gcs->buffers[i].vcsmVCMem = vcsm_vc_addr_from_hdl(gcs->buffers[i].vcsmHandle);
		CHECK_STATUS(gcs->buffers[i].vcsmVCMem == 0, "Failed to change VCSM user handle to VSCM VC memory!", error_bufimport);

		gcs->buffers[i].mem = vcsm_usr_address(gcs->buffers[i].vcsmHandle);
		CHECK_STATUS(gcs->buffers[i].mem == 0, "Failed to get VCSM user-space address!", error_bufexport);

		gcs->buffers[i].DMAFD = vcsm_export_dmabuf(gcs->buffers[i].vcsmHandle);
		CHECK_STATUS(gcs->buffers[i].DMAFD == 0, "Failed to export DMABUF from VSCM!", error_bufexport);
	}

	/*// Allocate buffers through V4L2
	struct v4l2_requestbuffers reqBuf;
	memset(&reqBuf, 0, sizeof reqBuf);
	reqBuf.count = GCS_SIMUL_BUFFERS;
	reqBuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqBuf.memory = V4L2_MEMORY_MMAP;
//	printf("IOCTL %d: MSG: %d VIDIOC_REQBUFS, CONTENT: %dcnt, %dtype, %dmem\n", gcs->fd, VIDIOC_REQBUFS, reqBuf.count, reqBuf.type, reqBuf.memory);
	status = ioctl(gcs->fd, VIDIOC_REQBUFS, &reqBuf);
	CHECK_STATUS(status, "Failed to allocate frame buffers!", error_buffers);

	gcs->bufferCount = reqBuf.count;
//	printf("%u buffers requested, V4L2 returned %u bufs, type %d (exp %d).\n", GCS_SIMUL_BUFFERS, reqBuf.count, reqBuf.type, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	int i; // Used to track progess in error handling
	for (i = 0; i < gcs->bufferCount; i++)
	{
		struct v4l2_plane planes[8];
		memset(planes, 0, sizeof planes);
		struct v4l2_buffer buffer;
		memset(&buffer, 0, sizeof buffer);
		buffer.index = i;
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
//		printf("IOCTL %d: MSG: %d VIDIOC_QUERYBUF, CONTENT: (%d, %d, %d, %d, %p)\n", gcs->fd, VIDIOC_QUERYBUF, 
//			buffer.index, buffer.type, buffer.memory, buffer.length, buffer.m.planes);
		status = ioctl(gcs->fd, VIDIOC_QUERYBUF, &buffer);
		CHECK_STATUS(status, "Failed to query buffer!", error_bufquery);

		gcs->buffers[i].index = i;
		gcs->buffers[i].size = buffer.length;
		gcs->buffers[i].mem = mmap(0, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, gcs->fd, buffer.m.offset);
		CHECK_STATUS(gcs->buffers[i].mem == MAP_FAILED, "Failed to mmap buffer!", error_bufmmap);

		struct v4l2_exportbuffer expBuffer;
		memset(&expBuffer, 0, sizeof(expBuffer));
		expBuffer.index = i;
		expBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//		printf("IOCTL %d: MSG: %d VIDIOC_EXPBUF, CONTENT: (%d, %d)\n", gcs->fd, VIDIOC_EXPBUF, 
//			expBuffer.index, expBuffer.type);
		status = ioctl(gcs->fd, VIDIOC_EXPBUF, &expBuffer);
		CHECK_STATUS(status, "Failed to export buffer!", error_bufexport);

		gcs->buffers[i].DMAFD = expBuffer.fd;
		gcs->buffers[i].vcsmHandle = vcsm_import_dmabuf(expBuffer.fd, "V4L2 buf");
		CHECK_STATUS(gcs->buffers[i].vcsmHandle == 0 || gcs->buffers[i].vcsmHandle == (uint32_t)-1, "Failed to import DMABUF to VSCM!", error_bufimport);
//		printf("Successfully imported into VCSM, user handle: %u\n", gcs->buffers[i].vcsmHandle);
		gcs->buffers[i].vcsmVCHandle = vcsm_vc_hdl_from_hdl(gcs->buffers[i].vcsmHandle);
		CHECK_STATUS(gcs->buffers[i].vcsmVCHandle == 0 || gcs->buffers[i].vcsmVCHandle == (uint32_t)-1, "Failed to change VCSM user handle to VSCM VC handle!", error_bufimport);
//		printf("Converted to VC-space handle: %u\n", gcs->buffers[i].vcsmVCHandle);
	}*/

////	printf("Set up GCS!\n");

	return gcs;

error_bufimport:
error_bufexport:
	o = 1;
error_bufmalloc:
	for (int j = 0; j < i+o; j++)
	{
		vcsm_free(gcs->buffers[i].vcsmHandle);
//		printf("Deallocate %d. \n", gcs->buffers[i].vcsmHandle);
	}
/*error_bufmmap:
	for (int j = 0; j < i+o; j++)
		munmap(gcs->buffers[i].mem, gcs->buffers[i].size);*/
error_bufquery:
	reqBuf.count = 0;
//	printf("IOCTL %d: MSG: %d VIDIOC_REQBUFS, CONTENT: %dcnt, %dtype, %dmem\n", gcs->fd, VIDIOC_REQBUFS, reqBuf.count, reqBuf.type, reqBuf.memory);
	status = ioctl(gcs->fd, VIDIOC_REQBUFS, &reqBuf);
error_buffers:
	vcsm_exit();
error_format:
	close(gcs->fd);
error_open:
	vcos_mutex_delete(&gcs->accessMutex);
error_mutex:
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
		vcsm_free(gcs->buffers[i].vcsmHandle);
		// With both DMABUF and VCSM FD freed, memory will be deallocated
	}

	// Exit VCSM
	vcsm_exit();

	// Close camera
	close(gcs->fd);

	// Delete access mutex
	vcos_mutex_delete(&gcs->accessMutex);

	// Free allocated resources
	delete gcs;
}

/* Start GCS (camera stream) */
uint8_t gcs_start(GCS *gcs)
{
	int status;

	// Ensure GCS is stopped first
	gcs_stop(gcs);
	gcs->error = 0;
	gcs->started = 1;
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

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
		status = ioctl(gcs->fd, VIDIOC_QBUF, &buffer);
		CHECK_STATUS(status, "Failed to enqueue buffer!", error_bufqueue);
	}

	// Start streaming
//	printf("IOCTL %d: MSG: %d VIDIOC_STREAMON, CONTENT: x\n", gcs->fd, VIDIOC_STREAMON);
	status = ioctl(gcs->fd, VIDIOC_STREAMON, &type);
	CHECK_STATUS(status, "Failed to start streaming!", error_streamon);


	// Directly access camera I2C (i2c-vs)
	if (gcs->cameraParams->extTrig)
	{ // TODO: Make sure that OV9281 driver is loaded
		unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
		if (i2c_fd < 0)
			printf("Failed to open camera I2C fd! %s \n", strerror(errno));
		else
		{ // Now modify camera streaming behaviour

			// Switch to external trigger mode (max 120Hz)
			unsigned char SC_MODE_SELECT[3] = { 0x01, 0x00, 0x00 };	// Put camera in standby
			unsigned char PSV_CTRL[3] = { 0x4f, 0x00, 0x0C };	// Shutoff SCLK to use FSIN to trigger
			unsigned char AUTO_SLEEP_CTRL[3] = { 0x4f, 0x01, 0x00 };	// tc_sof_sync_en
			unsigned char SC_LP_CTRL0_3[] = { 0x30, 0x2C, 0x00, 0x00, 0x00, 0x00 }; // r_sleep_period
			unsigned char SC_LP_CTRL4[3] = { 0x30, 0x30, 0x04 };	// Enable FSIN to wake camera (already set)
			unsigned char SC_CTRL_3F[3] = { 0x30, 0x3F, 0x01 };	// r_frame_on_num (already set)
			unsigned char TIMING_REG23[3] = { 0x38, 0x23, 0x00 };	// ?? (already set)
			unsigned char TIMING_VTS[] = { 0x38, 0x0E, 0x03, 0x8E };	// ext_vs_en

			// Switch to freerunning FSIN mode (max 120Hz)
			/* unsigned char SC_MODE_SELECT[3] = { 0x01, 0x00, 0x01 };	// Put camera in streaming mode
			unsigned char PSV_CTRL[3] = { 0x4f, 0x00, 0x09 };	// Shutoff SCLK to use FSIN to trigger
			unsigned char AUTO_SLEEP_CTRL[3] = { 0x4f, 0x01, 0x08 };	// tc_sof_sync_en
			unsigned char SC_LP_CTRL0_3[] = { 0x30, 0x2C, 0x00, 0x00, 0x00, 0x00 }; // r_sleep_period
			unsigned char SC_LP_CTRL4[3] = { 0x30, 0x30, 0x00 };	// Enable FSIN to wake camera (already set)
			unsigned char SC_CTRL_3F[3] = { 0x30, 0x3F, 0x01 };	// r_frame_on_num (already set)
			unsigned char TIMING_REG23[3] = { 0x38, 0x23, 0x40 };	// ext_vs_en
			uint16_t vts = 0x038E * gcs->cameraParams->fps/144; 
			unsigned char TIMING_VTS[] = { 0x38, 0x0E, (uint8_t)(vts>>8), (uint8_t)(vts&0xFF) }; */

			// Potentially set VSYNC_DELAY2 to lower delay and decrease transmission time?

			// Switch to external Frame Sync IN mode
			//unsigned char ANA_CORE_6[3] = { 0x36, 0x66, 0x00 };			// Enable FSIN

			// TODO: Figure out if sync mode exists and how to use it instead of external trigger mode
			// With external trigger, the sensor goes to sleep between each frame, which limits framerate to 120Hz
			// But in free-running mode, 144Hz is possible for 1280x800 @ 8bit
			// So if the "sync" mode, hinted at in documentation, exists so that it doesn't go to sleep every time, that'd be great

			struct i2c_msg I2C_MSG[] = {
				{ 0x60, 0, sizeof(PSV_CTRL), PSV_CTRL },
				{ 0x60, 0, sizeof(SC_MODE_SELECT), SC_MODE_SELECT },
				{ 0x60, 0, sizeof(AUTO_SLEEP_CTRL), AUTO_SLEEP_CTRL },
				{ 0x60, 0, sizeof(SC_LP_CTRL0_3), SC_LP_CTRL0_3 },
				{ 0x60, 0, sizeof(SC_LP_CTRL4), SC_LP_CTRL4 },
				{ 0x60, 0, sizeof(SC_CTRL_3F), SC_CTRL_3F },
				{ 0x60, 0, sizeof(TIMING_REG23), TIMING_REG23 },
				{ 0x60, 0, sizeof(TIMING_VTS), TIMING_VTS }
			};
			struct i2c_rdwr_ioctl_data I2C_DATA_TRIG = { I2C_MSG+0, 8 };
			if (gcs->cameraParams->extTrig && ioctl(i2c_fd, I2C_RDWR, &I2C_DATA_TRIG) < 0)
				printf("Failed to write extTrig registers! %s\n", strerror(errno));
			close(i2c_fd);
		}
	}

	// Attempt to set parameters
	gcs_updateParameters(gcs);

	return 0;

error_bufqueue:
	for (int j = 0; j < i; j++)
	{
		struct v4l2_buffer buffer;
		memset(&buffer, 0, sizeof buffer);
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;
		if (ioctl(gcs->fd, VIDIOC_DQBUF, &buffer))
			printf("Failed to dequeue buffer on error handling: %s!\n", strerror(errno));
	}
error_streamon:
	gcs->error = 1;
	gcs->started = 0;
	return -1;
}

/* Stop GCS (camera output). Stops watchdog and disabled MMAL camera */
void gcs_stop(GCS *gcs)
{
	if (gcs->started)
	{
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
		int status = ioctl(gcs->fd, VIDIOC_STREAMOFF, &type);
		CHECK_STATUS(status < 0, "Failed to stop streaming!", error_streamoff);
	}

	gcs->started = 0;
	return;

error_streamoff:
	gcs->started = 0;
	gcs->error = 1;
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
	/* {
		// Cannot set on OV9281, driver doesn't support it
		// Need to manually set up if setting framerate without external sync is desired
		struct v4l2_streamparm parm;
		memset(&parm, 0, sizeof parm);
		parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		status = ioctl(gcs->fd, VIDIOC_G_PARM, &parm);
		if (status == 0)
		{ // Camera can set parameters (not supported on some custom drivers)
			parm.parm.capture.timeperframe.numerator = gcs->cameraParams->fps;
			parm.parm.capture.timeperframe.denominator = 1;
			status = ioctl(gcs->fd, VIDIOC_S_PARM, &parm);
			printf("Successfully set framerate!\n");
		}
		else
			printf("Failed to get/set framerate!\n");
	} */


	// Directly access camera I2C (i2c-vs)
	if (gcs->cameraParams->strobe)
	{ // TODO: Make sure that OV9281 driver is loaded
		unsigned int i2c_fd = open("/dev/i2c-10", O_RDWR);
		if (i2c_fd < 0)
			printf("Failed to open camera I2C fd! %s \n", strerror(errno));
		else
		{ // Now modify camera streaming behaviour

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
			close(i2c_fd);
		}
	}

	// Digital camera gain not supported by OV9281 camera / driver
	//if ((status = gcs_setParameter(gcs, V4L2_CID_GAIN, gcs->cameraParams->digitalGain, 16)))
	//	printf("Failed to set digital camera gain!\n");
	if ((status = gcs_setParameter(gcs, V4L2_CID_ANALOGUE_GAIN, gcs->cameraParams->analogGain, 16)))
		printf("Failed to set analog camera gain!\n");
	if ((status = gcs_setParameter(gcs, V4L2_CID_EXPOSURE, gcs->cameraParams->shutterSpeed, 0)))
		printf("Failed to set camera exposure / shutter speed!\n");
	// Not used on OV9281
	/* if (gcs->cameraParams->disableAWB)
	{
		if ((status = gcs_setParameter(gcs, V4L2_CID_AUTO_WHITE_BALANCE, !gcs->cameraParams->disableAWB, 1)))
			printf("Failed to set auto white balance!\n");
	}
	if (gcs->cameraParams->disableEXP)
	{
		if ((status = gcs_setParameter(gcs, V4L2_CID_AUTOGAIN, !gcs->cameraParams->disableEXP, 1)))
			printf("Failed to set auto exposure!\n");
	} */
}

/* Returns once a new camera frame is available */
uint8_t gcs_waitForFrameBuffer(GCS *gcs, uint32_t waitUS)
{
	if (gcs->frameWaiting)
		return gcs->frameWaiting;

	vcos_mutex_lock(&gcs->accessMutex);
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = waitUS;

	// Return values for select indicating events in camera file descriptor
	fd_set readFD;
	FD_ZERO(&readFD);
	FD_SET(gcs->fd, &readFD);

	// Wait for changes from camera file descriptor
	int status = select(gcs->fd + 1, &readFD, NULL, NULL, &timeout);
	CHECK_STATUS(status == -1, "Failed select!", error_select);

	gcs->frameWaiting = FD_ISSET(gcs->fd, &readFD);
	vcos_mutex_unlock(&gcs->accessMutex);
	return gcs->frameWaiting;

error_select:
	gcs->error = 1;
	vcos_mutex_unlock(&gcs->accessMutex);
	return 0;
}

/* Returns whether there is a new camera frame available */
uint8_t gcs_hasFrameBuffer(GCS *gcs)
{
	return gcs_waitForFrameBuffer(gcs, 0);
}

/* Returns the next camera frame. If no camera frame is available yet, blocks until there is. */
void* gcs_requestFrameBuffer(GCS *gcs)
{
	vcos_mutex_lock(&gcs->accessMutex);
	if (gcs->frameWaiting || gcs_waitForFrameBuffer(gcs))
	{
		gcs->frameWaiting = 0;

		struct v4l2_buffer buffer;
		memset(&buffer, 0, sizeof buffer);
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_DMABUF;

		int status = ioctl(gcs->fd, VIDIOC_DQBUF, &buffer);
		CHECK_STATUS(status, "Failed to dequeue buffer!", error_dequeue);
		CHECK_STATUS(buffer.index >= GCS_SIMUL_BUFFERS || buffer.index < 0, "Dequeue returned invalid buffer!", error_dequeue);

		void* bufferHeader = &gcs->buffers[buffer.index];
		vcos_mutex_unlock(&gcs->accessMutex);
		return bufferHeader;
	}
	vcos_mutex_unlock(&gcs->accessMutex);
	return NULL;

error_dequeue:
	gcs->error = 1;
	vcos_mutex_unlock(&gcs->accessMutex);
	return NULL;
}

/* Returns the most recent camera frame. Assumes gcs_waitForFrameBuffer or gcs_hasFrameBuffer has returned true (frameWaiting is true). */
void* gcs_requestLatestFrameBuffer(GCS *gcs, unsigned int *num)
{
	if (!gcs->frameWaiting && !gcs_waitForFrameBuffer(gcs)) return NULL;
	gcs->frameWaiting = 0;

	bool waiting = true;
	void* bufferHeader = NULL;
	int status;
	*num = 0;

	vcos_mutex_lock(&gcs->accessMutex);

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
			CHECK_STATUS(status, "Failed to requeue frame buffer after processing!", error_abort);
		}
	}

	vcos_mutex_unlock(&gcs->accessMutex);
	return bufferHeader;

error_abort:
	gcs->error = 1;
	vcos_mutex_unlock(&gcs->accessMutex);
	return NULL;
}

/* Returns the user-space data of the given framebuffer. Use after gcs_requestFrameBuffer. */
void* gcs_getFrameBufferData(void *framebuffer)
{
	return ((struct GCS_FrameBuffer*)framebuffer)->mem;
}

/* Returns the user-space VCSM handle of the given framebuffer. Use after gcs_requestFrameBuffer. */
uint32_t gcs_getFrameBufferVCSMUserHandle(void *framebuffer)
{
	return ((struct GCS_FrameBuffer*)framebuffer)->vcsmHandle;
}

/* Returns the VC-space VCSM handle of the given framebuffer. Use after gcs_requestFrameBuffer. */
uint32_t gcs_getFrameBufferVCSMVCHandle(void *framebuffer)
{
	return ((struct GCS_FrameBuffer*)framebuffer)->vcsmVCHandle;
}

/* Returns the VC-space address of the given framebuffer. Use after gcs_requestFrameBuffer. */
uint32_t gcs_getFrameBufferVCPtr(void *framebuffer)
{
	return ((struct GCS_FrameBuffer*)framebuffer)->vcsmVCMem;
}

/* Return requested Franme Buffer after processing is done. */
bool gcs_returnFrameBuffer(GCS *gcs, void *bufferHeader)
{
	if (!bufferHeader) return true;
	vcos_mutex_lock(&gcs->accessMutex);
	struct v4l2_buffer buffer;
	memset(&buffer, 0, sizeof buffer);
	buffer.index = ((struct GCS_FrameBuffer*)bufferHeader)->index;
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_DMABUF;
	buffer.m.fd = gcs->buffers[buffer.index].DMAFD;
	int status = ioctl(gcs->fd, VIDIOC_QBUF, &buffer);
	CHECK_STATUS(status, "Failed to requeue frame buffer after processing!", error_requeue);
	vcos_mutex_unlock(&gcs->accessMutex);
	return true;

error_requeue:
	gcs->error = 1;
	vcos_mutex_unlock(&gcs->accessMutex);
	return false;
}
