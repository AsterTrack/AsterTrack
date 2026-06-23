/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <math.h>
#include <sched.h>
#include <execinfo.h>
#include <unistd.h>
#include <sys/prctl.h>

#include "util/log.hpp"

#include "processing.hpp"
#include "processing/masking.hpp"
#include "processing/framesync.hpp"
#include "blob/detection.hpp"
#include "blob/blob.hpp"
#include "camera/gcs.hpp"
#include "visualisation.hpp"

#include "state.hpp"
#include "comm/comm.hpp"

#include "ctpl/ctpl.hpp"

#define RUN_CAMERA	// Have the camera supply frames (else: emulate camera buffers)
//#define EMUL_VCSM	// Use VCSM for Emulation buffers instead of Mailbox allocated QPU buffers
//#define LOG_QPU
#define LOG_DROPS
#define LOG_TIMING

// Number of emulated buffers to iterate over if RUN_CAMERA is not defined
const int emulBufCnt = 4;

static ctpl::thread_pool threadPool(6);
static std::atomic<bool> isVisualising, isPreparingFrame;
static GCS *gcs = NULL;
// Keeping track of frame buffers
static std::mutex frameAccess;

extern int numCPUCores; // version.hpp

struct GCSWrapper
{
	GCSWrapper(GCS_CameraParams *cameraParams)
	{
		gcs = gcs_create(cameraParams);
	}
	~GCSWrapper()
	{
		std::unique_lock lock(frameAccess);
		gcs_destroy(gcs);
		gcs = nullptr;
		printf("-- Camera Stream Cleaned --\n");
	}
	operator bool() { return gcs != nullptr; }
};
FrameBuffer::~FrameBuffer()
{
#ifdef RUN_CAMERA // Return camera buffer to camera
	if (!gcs) return;
	std::unique_lock lock(frameAccess);
	if (gcs && !gcs_returnFrameBuffer(gcs, header))
	{ // TODO: Properly abort with camAbort
		//error = ERROR_GCS_RETURN;
		printf("Error returning frame buffer during FrameBuffer destructor!\n");
	}
#endif
}
struct StreamingState
{
	StreamingState()
	{
		printf("== Entering camera streaming mode! ==\n");
		std::unique_lock lock(framesync.access);
		framesync.frameSOFs = {};
		framesync.SOF2RecvDelay.reset();
	}
	~StreamingState()
	{
		printf("== Left camera streaming mode! ==\n");
		state.imageRequests = {};
	}
};

// Stats for continuous operation
inline static void addDeltaTimes(StatPacket::Times &base, StatPacket::Times &sample, uint16_t avgNum)
{
	for (int i = 0; i < sizeof(base.data)/sizeof(base.data[0]); i++)
		base.data[i] = (uint16_t)((sample.data[i] + (uint32_t)base.data[i]*(avgNum-1))/avgNum);
}
inline static void printDeltaTimes(StatPacket::Times &time)
{
	printf("Latency %.2fms (qpu %.2f, fetch %.2f, CCL %.2f, post %.2f, TX %.2f)",
		time.latency/1000.0f, time.qpu/1000.0f, time.fetch/1000.0f, time.ccl/1000.0f, time.post/1000.0f, time.send/1000.0f);
}
// Stats for occurences
inline static void recordIncident(StatPacket::Incidents::Stat &stat, uint16_t value)
{ 
	uint32_t tmp = (value+(uint32_t)stat.avg*stat.occurences); stat.avg = (uint16_t)(tmp / (++stat.occurences)); stat.max = std::max(stat.max, value);
}
inline static void recordIncident(StatPacket::Incidents::Stat &stat, uint16_t value, uint16_t limit)
{
	if (value > limit) recordIncident(stat, value);
}
inline static void printIncidents(StatPacket::Incidents::Stat &stat, const char *label)
{
	if (stat.occurences) printf("%dx %s %dus, max %dus; ", stat.occurences, label, stat.avg, stat.max);
}

bool ProcessingStage(TrackingCameraState &state, QPU_BASE &base)
{
	TimePoint_t time_start = sclock::now();
	TimePoint_t time_lastStatCheck = sclock::now();

	StreamingState enterStreaming = {};

	// ---- Setup camera stream ----

#ifdef RUN_CAMERA

	// Init camera subsystem
	SENSOR_NUM gcsSensor = gcs_findCamera();
	if (gcsSensor == SENSOR_ERROR)
	{
		printf("Failed to open camera I2C!\n");
		if (handleError(ERROR_GCS_NO_I2C))
			return false;
	}
	else if (gcsSensor == SENSOR_INVALID)
	{
		printf("Failed to identify camera sensor!\n");
		if (handleError(ERROR_GCS_NO_SENSOR))
			return false;
	}

	// Create GCS (static) in RAII. Not nice, but ensures deconstruction in-order
	GCSWrapper camera(&state.camera);
	if (!camera)
	{
		printf("Failed to initialise camera!\n");
		if (handleError(ERROR_GCS_CREATE))
			return false;
	}
#else // Camera emulation buffers
#ifdef EMUL_VCSM
	VCSM_BUFFER camEmulBuf[emulBufCnt];
#else
	QPU_BUFFER camEmulBuf[emulBufCnt];
#endif
#endif


	// ---- Setup blob processing ----

	// Layout QPU program instances across camera frame according to cores used
	ProgramLayout layout = SetupProgramLayout(state.camera.width, state.camera.height, state.qpuCores.getUsed());

	BlobDetection detect(layout.maskSize, layout.maskOffset, numCPUCores);
	if (!detect)
	{ // Shouldn't actually fail, just basic init
		printf("-- Failed to initialize blob detection! --\n");
		if (handleError(ERROR_INIT_BD))
			return false;
	}

	MaskingProgram masking(base, layout, state.codeFile);
	if (!masking)
	{
		printf("-- Failed to setup QPU masking program! --\n");
		if (handleError(ERROR_INIT_BD))
			return false;
	}
	masking.SetParameters(state.thresholds.absolute, state.thresholds.edge);

	ExclusiveQPU qpu(base, state.qpuCores, layout.instances, !state.writeStatLogs);
	if (!qpu)
	{
		printf("QPU enable failed!\n");
		if (handleError(ERROR_QPU_ENABLE))
			return false;
	}

#ifdef RUN_CAMERA
	uint32_t srcStride = state.camera.stride;
#else // Camera emulation buffers
	uint32_t srcStride = (state.camera.width+31)/32*32;
#endif

	// ---- Start camera stream ----

	// Start GPU camera stream
#ifdef RUN_CAMERA
	if (gcs_start(gcs) != 0)
	{
		printf("-- Failed to start camera stream! --\n");
		if (handleError(ERROR_GCS_START))
			return false;
	}
	printf("-- Camera Stream started --\n");
#else

	// Allocate emulated buffers
#ifdef EMUL_VCSM
	for (int i = 0; i < emulBufCnt; i++)
	{ // Allocate only grayscale buffer
		camEmulBuf[i] = vcsm_malloc(srcStride*state.camera.height);
		if (camEmulBuf[i].fd >= 0) continue;
		printf("Failed to allocate vcsm buffer!\n");
		for (int j = i-1; j >= 0; j--)
			vcsm_free(camEmulBuf[j]);
		return false;
	}
#else
	for (int i = 0; i < emulBufCnt; i++)
	{ // Allocate only grayscale buffer
		int ret = qpu_allocBuffer(&camEmulBuf[i], &base, srcStride*state.camera.height, 4096);
		if (!ret) continue;
		printf("Failed to allocate buffer %d for emulation: %d!\n", i, ret);
		for (int j = i-1; j >= 0; j--)
			qpu_releaseBuffer(&camEmulBuf[j]);
		return false;
	}
#endif

	// Generate random test frames
	for (int i = 0; i < emulBufCnt; i++)
	{
#ifdef EMUL_VCSM
		if (!vcsm_lock(camEmulBuf[i]))
		{
			printf("Failed to lock vcsm buffer!");
			return false;
		}
		uint8_t *YUVFrameData = (uint8_t*)camEmulBuf[i].mem;
#else
		qpu_lockBuffer(&camEmulBuf[i]);
		uint8_t *YUVFrameData = (uint8_t*)camEmulBuf[i].ptr.arm.vptr;
#endif
		for (int y = 0; y < state.camera.height; y++)
			for (int x = 0; x < state.camera.width; x++)
				YUVFrameData[srcStride*state.camera.height*0 + y*srcStride + x] = 0;
		// Write test blobs
		for (int c = 0; c < 5; c++)
		{
			float size = 40.0f;
			int posX = state.camera.width/2-50+100*(i&1);
			int posY = state.camera.height/2-50+100*((i&2)>>1);
//			float size = (rand()%255) / 255.0f * 5.0f + 1.0f;
//			int posX = rand()%state.camera.width;
//			int posY = rand()%state.camera.height;
			int dots = 0;
			for (int y = -size; y < size; y++)
			{
				for (int x = -size; x < size; x++)
				{
					if (x*x + y*y <= size*size)
					{
						int pX = posX+x, pY = posY+y;
						if (pX >= 0 && pY >= 0 && pX < state.camera.width && pY < state.camera.height)
						{
							YUVFrameData[pY*srcStride + pX] = 255;
							dots++;
						}
					}
				}
			}
			printf("Blob at %d, %d has %d dots!\n", posX, posY, dots);
		}
#ifdef EMUL_VCSM
		vcsm_unlock(camEmulBuf[i]);
#else
		qpu_unlockBuffer(&camEmulBuf[i]);
#endif
	}
#endif


	// ---- Shared Variables ----

	// Performance counters
	uint32_t numFrames = 0;
	StatPacket::Times avgTimes = {}, curTimes = {};
	int skip_count_trigger = 0, skip_count_qpu = 0, skip_count_cpu = 0;

	// Prediction when a new frame will arrive through V4L2
	float avgInterval = 0.0f;
	TimePoint_t time_frameRecv;
	
	// Setup incident statistics
	StatPacket::Incidents incidents = {};
	// Limits achievable on Zero 1 and Zero 2 without overclocking, with some spikes
	uint16_t awaitLimit = 1000000/state.camera.fps,
		skipLimit = 30,
		accessLimit = 10,
		procLimit = 2500,
		handleLimit = 50,
		cpuwaitLimit = 1000000/state.camera.fps;

	// Buffers
	std::vector<Cluster> clusters;
	std::vector<uint8_t> packetBuffer;
	packetBuffer.reserve(4096);
	int bytesSentAccum = 0;


	// ---- Setup QPU thread ----

	std::shared_ptr<FrameBuffer> frameExchange; // Would like this to be either volatile or std::atomic, but alas, latter is C++20
	std::mutex frameReady;
	frameReady.lock();
	std::atomic<bool> abortStreaming = { false };
	std::thread *qpuThread = new std::thread([&]()
	{
		prctl(PR_SET_NAME, "Thread_QPU");
		nice(-18); // Very high priority
		/*cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(2, &set);
		sched_setaffinity(0, sizeof(set), &set);*/

		int qpu_it = 0; // Detached frame count, doesn't need to match numFrames
		uint32_t frameID = -1;
		int cumulFrameNoTrigger = 0, cumulFramePassedQPU = 0, cumulFramePassedCPU = 0;
		time_frameRecv = sclock::now();

		while (!abortStreaming)
		{
			qpu_it++;

			TimePoint_t t0 = sclock::now();

			if (state.updateSetupQPU.exchange(false))
			{ // Got a new setup packet to read
				acceptQPUConfig(state);

				// Update camera parameters (previously linked to gcs)
			#ifdef RUN_CAMERA
				gcs_updateParameters(gcs);
			#endif

				// Update QPU parameters in uniform memory
				masking.SetParameters(state.thresholds.absolute, state.thresholds.edge);
			}

#ifdef RUN_CAMERA
			auto waitForFrame = [&]() -> bool
			{
				// Wait for a frame to be received in the VideoCore
				TimePoint_t waitStart = sclock::now();
				bool sentMinorTimeout = false;
				int minorWait = qpu_it < 10? 100 : 10, seriousWait = 500;
				while (gcs_waitForFrameBuffer(gcs, 1000) == 0 && !abortStreaming)
				{
					if (!state.noComms && !comm_anyReady(comms))
					{
						printf("== Lost comms! Stopping streaming! ==\n");
						state.curMode.streaming = false;
						state.curMode.mode = TRCAM_STANDBY;
						state.curMode.opt = TRCAM_OPT_NONE;
						return true;
					}
					long waitTimeUS = dtUS(waitStart, sclock::now());
					long waitFrames = waitTimeUS*state.camera.fps/1000000;
					if ((waitFrames >= minorWait && !sentMinorTimeout) || waitFrames >= seriousWait)
					{ // Unsuccessfully waited a few frames (or more if just after streaming start) for next frame
						printf("== %.2fms: QPU: Waited for %ldus / %d frames - way over expected frame interval! ==\n",
							dtMS(time_start, sclock::now()), waitTimeUS, (int)(waitTimeUS * state.camera.fps / 1000000));
						sentMinorTimeout = true;
						// Notify host
						if (handleError(ERROR_GCS_TIMEOUT, waitFrames >= seriousWait))
							return true;
					}
				}
				return false;
			};
			if (waitForFrame())
				break;

			if (frameReady.try_lock())
			{ // Return previous frame if it wasn't claimed already
				std::shared_ptr<FrameBuffer> pastFrame = std::move(frameExchange);
#ifdef LOG_DROPS
				if (pastFrame)
					printf("%lldms: CPU FRAME DROP ID %u\n",
						std::chrono::duration_cast<std::chrono::milliseconds>(sclock::now() - time_start).count(), pastFrame->ID);
#endif
				cumulFramePassedCPU++;
			}
			else
			{ // CPU accepted last frame, reset state
				cumulFrameNoTrigger = 0;
				cumulFramePassedQPU = 0;
				cumulFramePassedCPU = 0;
				masking.bitmskSwitch = (masking.bitmskSwitch+1) % masking.BitmaskCount;
			}

			TimePoint_t t1 = sclock::now();

			// Get most recent frame
			unsigned int obsAdvance = 0;
			void *frameHeader = gcs_requestLatestFrameBuffer(gcs, &obsAdvance);
			if (abortStreaming)
				break;
			if (frameHeader == NULL || obsAdvance == 0)
			{
				printf("== GCS returned NULL frame with error flag %d! ==\n", gcs_readErrorFlag(gcs));
				if (handleError(ERROR_GCS_REQUEST))
					break;
			}

			TimePoint_t time_cur = sclock::now();
#ifdef LOG_DROPS
			if (obsAdvance > 1)
				printf("%.2fms: QPU THREAD MISSED %d+ FRAMES AFTER ID %d\n", dtMS(time_start, time_cur), obsAdvance-1, frameID);
#endif

			float newInterval = dtMS(time_frameRecv, time_cur);
			float expInterval = 1000.0f/state.camera.fps;
			float minInterval = expInterval;
			int advanceFrames = obsAdvance;

			// Consider waiting a bit more for next frame to reset (accumulated) delay
			/* float timeDelay = newInterval - expInterval*advanceFrames;
			float maxDelay = expInterval - avgTimes.cpu/1000.0f;
			// NOTE: Here should be the average time of the single largest bottleneck - currently, CPU
			if (timeDelay > maxDelay)
			{ // Skip current frame as well, or else future frames will be delayed, too
				printf("-- Skipping one frame deliberately because frame delay %fms (%f - %f) took longer than %fms!\n",
					timeDelay, newInterval, expInterval*advanceFrames, maxDelay);
				if (!gcs_returnFrameBuffer(gcs, frameHeader))
				{
					printf("== GCS failed to return frame buffer with error flag %d while skipping delayed! ==\n", gcs_readErrorFlag(gcs));
					if (handleError(ERROR_GCS_RETURN))
						break;
				}
				TimePoint_t waitStart = sclock::now();
				if (waitForFrame())
					break;
				frameHeader = gcs_requestFrameBuffer(gcs);
				if (frameHeader == NULL)
				{
					printf("== GCS returned NULL frame with error flag %d after skipping delayed! ==\n", gcs_readErrorFlag(gcs));
					if (handleError(ERROR_GCS_REQUEST))
						break;
				}
				// Update values
				advanceFrames++;
				time_cur = sclock::now();
#ifdef LOG_DROPS
				printf("%.2fms: QPU: SKIPPED %.2fms TO CURRENT FRAME, new interval %ldus\n",
					dtMS(time_start, time_cur), dtMS(waitStart, time_cur), dtUS(time_frameRecv, time_cur));
				printf("%.2fms: QPU: Reason: time_diff %.2fms, time_delay %.2fms, advance frames %d, avg processing %.2fms, max delay %.2fms\n",
					dtMS(time_start, time_cur), newInterval, timeDelay, advanceFrames, avgTimes.processing/1000.0f, maxDelay);
#endif
				newInterval = dtMS(time_frameRecv, time_cur);
			} */

			// Update frameID with advanced frames
			float expFrameIntervals = newInterval / expInterval;
			int expAdvance = std::max(1, (int)(expFrameIntervals+0.2f));
			bool likelyDropped = std::abs(expFrameIntervals - obsAdvance) > 1;
			if (likelyDropped)
			{
				printf("May have dropped frames! %.1f frame times passed, but only got %d new frames!\n", expFrameIntervals, obsAdvance);
			}

			if (getFramesyncActive() && state.camera.extTrig)
			{ // Match current frame with frame SOFs from PACKET_SOF - can be assumed to be continuous
				uint32_t frameIDNext = correctFromFrameSync(framesync, time_cur, frameID, obsAdvance, expAdvance, likelyDropped);
				int unexpectedDiff = frameIDNext - (frameID + obsAdvance);
				cumulFramePassedQPU += obsAdvance-1;
				cumulFrameNoTrigger += unexpectedDiff;
				advanceFrames += unexpectedDiff;
				frameID = frameIDNext;
			}
			else
			{
				if (likelyDropped && advanceFrames == 1)
				{
					cumulFramePassedQPU += obsAdvance-1;
					cumulFrameNoTrigger += expAdvance - obsAdvance;
					advanceFrames = expAdvance;
					printf("    Determined to likely have skipped %d frames with no trigger!\n", advanceFrames);
				}
				frameID += advanceFrames;
			}

			const int gracePeriod = 100;
			if (qpu_it < gracePeriod)
			{ // Accept SOF as it is in the beginning
				time_frameRecv = time_cur;
			}
			else if (advanceFrames > 1)
			{ // Advance SOF prediction
				time_frameRecv += std::chrono::microseconds((int)(expInterval*advanceFrames * 1000));
			}
			else if (std::abs(newInterval-expInterval) < 400)
			{ // Update SOF prediction
				const int expWeight = 10, newWeight = 100;
				int avgWeight = std::min(50, qpu_it-gracePeriod);
				// Update prediction and average
				float avg = avgWeight*avgInterval;
				float advInterval = (expInterval*expWeight + newInterval*newWeight + avg) / (expWeight + avgWeight + newWeight);
				avgInterval = (avg + newInterval)/(avgWeight+1);
				time_frameRecv += std::chrono::microseconds((int)(advInterval * 1000));
			}
			else if (std::abs(advanceFrames-expAdvance) > 1)
			{
				time_frameRecv = time_cur;
				printf("Received frame interval is WAY off from expectation at %.2fms, average %.2fms, expected %.2fms! Adopting new rythm.\n",
					newInterval, avgInterval, expInterval);
			}
			else
			{ // Advance SOF prediction, too different to use for update
				time_frameRecv += std::chrono::microseconds((int)(expInterval*advanceFrames * 1000));
#ifdef LOG_TIMING
				printf("Current frame interval is %.2fms, average %.2fms, expected %.2fms, indicating QPU thread is not as realtime as it should be!\n",
					newInterval, avgInterval, expInterval);
#endif
			}

			TimePoint_t t2 = sclock::now();

			VCSM_BUFFER &frameBuffer = gcs_getFrameBufferData(frameHeader);

			// Lock VCSM buffer (doesn't seem to be needed on a pi)
			bool locked = true;
			//locked = vcsm_lock(frameBuffer);
			//locked = mem_lock(base.mb, frameBuffer.VCHandle) != 0;
			if (!locked)
			{
				printf("== %.2fms: QPU: Failed to access frame! ==\n", dtMS(time_start, sclock::now()));
				if (handleError(ERROR_MEM_ACCESS))
					break;
			}

			uint8_t *framePtrARM = (uint8_t*)frameBuffer.mem;
			uint32_t framePtrVC = frameBuffer.VCMem; 

#else
			// Somewhat emulate framerate
			usleep(std::max(0,(int)(1.0f/state.camera.fps*1000*1000)-4000));

			TimePoint_t t1 = sclock::now();
			TimePoint_t t2 = sclock::now();

			if (frameReady.try_lock())
			{ // Return previous frame if it wasn't claimed already
				std::shared_ptr<FrameBuffer> pastFrame = std::move(frameExchange);
			}
			else
			{ // CPU accepted last frame
				masking.bitmskSwitch = (masking.bitmskSwitch+1) % masking.BitmaskCount;
			}

			void *frameHeader = NULL;

			// Use prepared testing frames
	#ifdef EMUL_VCSM
			VCSM_BUFFER &frameBuffer = camEmulBuf[qpu_it%emulBufCnt];

			// Lock VCSM buffer (doesn't seem to be needed on a pi)
			bool locked = true;
			//locked = vcsm_lock(frameBuffer);
			//locked = mem_lock(base.mb, frameBuffer.VCHandle) != 0;
			if (!locked)
			{
				printf("== %.2fms: QPU: Failed to access frame! ==\n", dtMS(time_start, sclock::now()));
				if (handleError(ERROR_MEM_ACCESS))
					break;
			}

			uint8_t *framePtrARM = (uint8_t*)frameBuffer.mem;
			uint32_t framePtrVC = frameBuffer.VCMem; 
	#else
			QPU_BUFFER &frameBuffer = camEmulBuf[qpu_it%emulBufCnt];

			// Lock QPU buffer (needed?)
			qpu_lockBuffer(&frameBuffer);
			uint8_t *framePtrARM = (uint8_t*)frameBuffer.ptr.arm.cptr;
			uint32_t framePtrVC = frameBuffer.ptr.vc;
	#endif
#endif

			TimePoint_t t3 = sclock::now();

			// ---- Masking on QPU ----

			int code = masking.Execute(base, qpu.perf, srcStride, framePtrVC);
			QPU_BUFFER &bitmskBuf = masking.bitmskBuffer[masking.bitmskSwitch];

#ifdef LOG_QPU	// Only relevant during development of QPU programs
			qpu_logErrors(&base);
#endif

			if (code != 0)
			{
				printf("== %.2fms: QPU: Failed to process frame! ==\n", dtMS(time_start, sclock::now()));
				if (handleError(ERROR_QPU_STALL_MSK))
					break;
			}

			TimePoint_t t4 = sclock::now();

#ifdef RUN_CAMERA
			// Unlock VCSM buffer
			//vcsm_unlock(frameBuffer);
			//mem_unlock(base.mb, frameBuffer.VCHandle);
#else
	#ifdef EMUL_VCSM
			// Unlock VCSM buffer
			//vcsm_unlock(frameBuffer);
			//mem_unlock(base.mb, frameBuffer.VCHandle);
	#else
			qpu_unlockBuffer(&frameBuffer);
	#endif
#endif

			// Signal main thread and update resources for the new frame
			std::shared_ptr<FrameBuffer> newFrame = std::make_shared<FrameBuffer>();
			newFrame->header = frameHeader;
			newFrame->memory = framePtrARM;
			newFrame->stride = srcStride;
			newFrame->bitmsk = &bitmskBuf;
			newFrame->rcv = t3;
			newFrame->ID = frameID;
			newFrame->time_qpu = dtUS(t3, t4);
			newFrame->skippedTrigger = cumulFrameNoTrigger;
			newFrame->skippedQPU = cumulFramePassedQPU;
			newFrame->skippedCPU = cumulFramePassedCPU;
			frameExchange = std::move(newFrame);
			frameReady.unlock();

			TimePoint_t t5 = sclock::now();
			//printf("%.2fms: QPU: Finished QPU processing after %.2fms, including sending off %.2fms\n", dtMS(time_start, t6), dtMS(t1, t5), dtMS(t1, t6));

			// In case of only one CPU core: Switch to CPU thread to process frame
			sched_yield();

			// Record delays
			recordIncident(incidents.await, dtUS(t0, t1), awaitLimit);
			recordIncident(incidents.skip, dtUS(t1, t2), skipLimit);
			recordIncident(incidents.access, dtUS(t2, t3), accessLimit);
			recordIncident(incidents.proc, dtUS(t3, t4), procLimit);
			recordIncident(incidents.handle, dtUS(t4, t5), handleLimit);
		}
		abortStreaming = true;
		static_cast<void>(frameReady.try_lock());
		frameReady.unlock();
	});


	// ---- Init Initial Mode ----

	if (state.curMode.mode == TRCAM_MODE_BGCALIB)
	{
		if (state.curMode.opt == TRCAM_OPT_NONE)
			detect.initBackgroundCalibration();
	}


	// ---- Processing Loop ----

	// Increase clock rate after boot (likely has to be done through linux CPU governor, not directly)
//		setClockRate(base.mb, 3, 1000000000); // 1000Mhz fixed
//		printf("Set ARM Core clock rate to fixed %dMhz!\n", getClockRate(base.mb, 3)/1000000);

	nice(-15); // High priority

	time_lastStatCheck = sclock::now();
	time_start = sclock::now();
	int time_debug_interval = 200;
	int frameNum_lastStatCheck = 0;

	bool abortProgram = false;
	while (!abortProgram && state.curMode.streaming)
	{
		numFrames++;

		if (!state.noComms && !comm_anyReady(comms))
		{
			printf("Lost comms! Stopping streaming!\n");
			state.curMode.streaming = false;
			state.curMode.mode = TRCAM_STANDBY;
			state.curMode.opt = TRCAM_OPT_NONE;
			break;
		}

		if (!handleConsoleInputStreaming())
			break;

		if (state.updateSetupCPU.exchange(false))
		{ // Got a new setup packet to read
			acceptCPUConfig(state);
		}

		if (state.updateMode.exchange(false))
		{ // Switch to specified mode
			TrackingCameraMode newMode = state.newMode;
			if (newMode.mode == TRCAM_MODE_BGCALIB)
			{
				if (newMode.opt == TRCAM_OPT_NONE)
				{ // Reset temp mask
					printf("Init Background Calibration!\n");
					detect.initBackgroundCalibration();
				}
				else if (newMode.opt == TRCAM_OPT_BGCALIB_RESET)
				{ // Reset current mask (and temp mask)
					if (state.curMode.mode == TRCAM_MODE_BGCALIB)
					{
						printf("Resetting Background Calibration!\n");
						detect.resetBackgroundCalibration();
					}
				}
				else if (newMode.opt == TRCAM_OPT_BGCALIB_RETRY)
				{ // Reset current mask (and temp mask)
					if (state.curMode.mode == TRCAM_MODE_BGCALIB)
					{
						printf("Resetting ongoing Background Calibration!\n");
						detect.retryBackgroundCalibration();
					}
				}
				else if (newMode.opt == TRCAM_OPT_BGCALIB_DISCARD)
				{ // Accept temp mask as current mask
					if (state.curMode.mode == TRCAM_MODE_BGCALIB)
					{
						printf("Discarding Background Calibration!\n");
						detect.discardBackgroundCalibration();
					}
					newMode.mode = TRCAM_MODE_BLOB;
					newMode.opt = TRCAM_OPT_NONE;
				}
				else if (newMode.opt == TRCAM_OPT_BGCALIB_ACCEPT)
				{ // Accept temp mask as current mask
					if (state.curMode.mode == TRCAM_MODE_BGCALIB)
					{
						printf("Accepting Background Calibration!\n");
						detect.acceptBackgroundCalibration();
					}
					newMode.mode = TRCAM_MODE_BLOB;
					newMode.opt = TRCAM_OPT_NONE;
				}
			}
			printf("New mode: streaming: %d, mode: %d, opt: %d\n", newMode.streaming, newMode.mode, newMode.opt);
			state.curMode = newMode;

			// Notify host of mode change
			static_assert(TRCAM_MODE_SIZE == std::numeric_limits<uint8_t>::max());
			uint8_t mode = (uint8_t)(newMode.streaming? TRCAM_FLAG_STREAMING : 0) | newMode.mode | newMode.opt;
			comm_send(realTimeAff, PacketHeader(PACKET_MODE, 1), &mode);

			if (!state.curMode.streaming)
			{
				printf("== Received command to leave streaming mode! ==\n");
				break;
			}
		}


		// ---- QPU Thread Interfacing ----

		std::shared_ptr<FrameBuffer> curFrame;
		curTimes = {};
		{ // Wait for a new bitmask from the QPU processing thread
			TimePoint_t waitStart = sclock::now();
			frameReady.lock();
			curFrame = std::move(frameExchange);
			recordIncident(incidents.cpuwait, dtUS(waitStart, sclock::now()), cpuwaitLimit);
			if (!curFrame || abortStreaming) // Encountered error in QPU thread
				break;
		}

		/* if (uartComms.ready)
		{ // Verify the large packet interleaving works by checking the TX queue size
			// Mostly interested in UART for now

			int txQueue = uart_getTXQueue(uartComms.port);
			if (txQueue > 100)
			{
				printf("Failed to constrain large packet blocks to idle times, have %d bytes in TX buffer before sending SOF!\n", txQueue);
			}
		} */

		{ // Announce to controller that frame is being processed
			CommState* rtComm = comms.get(realTimeAff);
			if (comm_packet(rtComm, PacketHeader(PACKET_FRAME_SIGNAL, 0, curFrame->ID&0xFF)))
				comm_submit(rtComm);
		}

		// Allow sending from now until processing is expected to end, at which point we need to send the results with low latency
		bytesSentAccum += sendInterleavedQueuedPackets(avgTimes.cpu-200);

		// Increase clock rate after boot (likely has to be done through linux CPU governor, not directly)
		//printf("======\nHad ARM Core clock rate of %dMhz!\n", getClockRate(base.mb, 3)/1000000);
		//setClockRate(base.mb, 3, 1000000000); // 1000Mhz fixed
		//printf("Set ARM Core clock rate to fixed %dMhz!\n", getClockRate(base.mb, 3)/1000000);


		// ---- CPU Processing ----

		TimePoint_t time_cpu_begin = sclock::now();

		if (state.curMode.mode == TRCAM_MODE_SIM && !(state.curMode.opt&TRCAM_OPT_SIM_PROC))
		{ // Simulation mode, no processing
			int simBlobCnt = 2<<(state.curMode.opt&TRCAM_OPT_SIM_LVL); // Range: 2 - 256

			if (clusters.size() != simBlobCnt)
			{
				clusters.resize(simBlobCnt);
				for (int i = 0; i < simBlobCnt; i++)
				{
					clusters[i].centroid.x() = (rand()%(10*state.camera.width))/10.0f;
					clusters[i].centroid.y() = (rand()%(10*state.camera.height))/10.0f;
					clusters[i].size = (rand()%255) / 255.0f * 5.0f + 1.0f;
				}
			}
			else
			{
				for (int i = 0; i < simBlobCnt; i++)
				{
					clusters[i].centroid.x() += (rand()%100)/10.0f;
					clusters[i].centroid.y() += (rand()%100)/10.0f;
					if (clusters[i].centroid.x() <= 0 || clusters[i].centroid.x() >= state.camera.width || clusters[i].centroid.y() <= 0 || clusters[i].centroid.y() >= state.camera.height)
					{
						clusters[i].centroid.x() = (rand()%(10*state.camera.width))/10.0f;
						clusters[i].centroid.y() = (rand()%(10*state.camera.height))/10.0f;
					}

				}
			}
			for (int i = 0; i < simBlobCnt; i++)
			{
				float posX = clusters[i].centroid.x(), posY = clusters[i].centroid.y(), size = clusters[i].size/2;
				int dots = 0;
				clusters[i].dots.clear();
				for (int y = -size; y < size; y++)
				for (int x = -size; x < size; x++)
				{
					if (x*x + y*y <= size*size)
					{
						int pX = posX+x, pY = posY+y;
						if (pX >= 0 && pY >= 0 && pX < state.camera.width && pY < state.camera.height)
							clusters[i].dots.emplace_back(pX, pY);
					}
				}
			}
		}
		else if (state.curMode.mode != TRCAM_MODE_IDLE)
		{ // Perform blob detection on bitmask
			TimePoint_t t0 = sclock::now();

			// Lock bitmask buffer
			//qpu_lockBuffer(curFrame->bitmsk);

			// Extract regions with blobs
			detect.fetchRegions(curFrame->bitmsk->ptr.arm.uptr);

			// Unlock bitmask buffer
			//qpu_unlockBuffer(curFrame->bitmsk);

			TimePoint_t t1 = sclock::now();

			// Perform Connected Component Labelling on regions
			clusters.clear();
			detect.performCCL(clusters);

			// Update timing
			curTimes.fetch = dtUS(t0, t1);
			curTimes.ccl = dtUS(t1, sclock::now());
		}

		if (state.curMode.mode == TRCAM_MODE_BLOB)
		{ // Blob Refinement
			TimePoint_t t0 = sclock::now();
			curTimes.blur = 0;
			curTimes.maxima = 0;
			curTimes.local = 0;
			curTimes.iter = 0;
			curTimes.check = 0;
			curTimes.reseg = 0;
			curTimes.refine = 0;

			std::vector<Cluster> oldClusters = std::move(clusters);
			std::mutex syncClusters, done;
			std::atomic<int> remaining = { (int)oldClusters.size() };

			PrecomputedKernels kernels;

			if (!oldClusters.empty())
			{ // Will submit threads
				done.lock();
				if (state.blobParams.base.blur)
					kernels = precomputeKernels(state.blobParams);
			}
			for (int i = 0; i < oldClusters.size(); i++)
			{
				threadPool.push([&](int, int index){
					prctl(PR_SET_NAME, "Worker_Blob");
					nice(-10); // Medium-High priority
					TimePoint_t t0 = sclock::now();

					int blurTimeUS = 0, maximaTimeUS = 0, localMaxTimeUS = 0, maxIterTimeUS = 0, finalCheckTimeUS = 0, resegTimeUS = 0, refineTimeUS = 0;
					std::vector<Cluster> subClusters = handleCluster(std::move(oldClusters[index]), 
						curFrame->memory, srcStride, state.camera.width, state.camera.height,
						state.blobParams, kernels,
						blurTimeUS, maximaTimeUS, localMaxTimeUS, maxIterTimeUS, finalCheckTimeUS, resegTimeUS, refineTimeUS);

					std::unique_lock lock(syncClusters);
					std::move(std::begin(subClusters), std::end(subClusters), std::back_inserter(clusters));
					curTimes.blur += blurTimeUS;
					curTimes.maxima += maximaTimeUS;
					curTimes.local += localMaxTimeUS;
					curTimes.iter += maxIterTimeUS;
					curTimes.check += finalCheckTimeUS;
					curTimes.reseg += resegTimeUS;
					curTimes.refine += refineTimeUS;
					if (--remaining == 0)
						done.unlock();
				}, i);
			}
			done.lock();

			curTimes.post = dtUS(t0, sclock::now());
		}

		TimePoint_t time_cpu_end = sclock::now();
		//printf("%.2fms: CPU: Finished CPU processing in %.2fms\n", dtMS(time_start, sclock::now()), dtMS(time_cpu_begin, time_cpu_end));


		// ---- Reporting & Comms ---

		TimePoint_t time_report_begin = sclock::now();
		bytesSentAccum = getInterleavedQueuedBytes();

		if (state.curMode.mode == TRCAM_MODE_BGCALIB)
		{
			Vector2<uint8_t> extends = (layout.validMaskRect.extends()/8).cast<uint8_t>();
			std::vector<uint8_t> bgTiles(2);
			bgTiles[0] = extends.x();
			bgTiles[1] = extends.y();
			detect.updateBackgroundCalibration(bgTiles);
			if (bgTiles.size() > 2)
			{
				//printf("Added %d new tiles to the background mask!\n", bgTiles.size()/2-1));
				TimePoint_t t0 = sclock::now();
				comm_send(largeDataAff, PacketHeader(PACKET_BGTILES, bgTiles.size()), std::move(bgTiles));
				bytesSentAccum += bgTiles.size();
				curTimes.send += dtUS(t0, sclock::now());
			}
		}

		CommState* rtComm = comms.get(realTimeAff);
		if (rtComm && !rtComm->realtimeControlled)
		{ // Ensure this comm medium is the only one flagged
			for (auto &comm : comms.medium)
				if (comm)
					comm->realtimeControlled = false;
			rtComm->realtimeControlled = true;
		}
		if (rtComm)
		{ // Create blob report
			TimePoint_t t0 = sclock::now();

			int reportLength = STREAM_PACKET_HEADER_SIZE + STREAM_PACKET_BLOB_SIZE * clusters.size();
			if (packetBuffer.size() < reportLength)
				packetBuffer.resize(reportLength*3/2);
			static_assert(STREAM_PACKET_HEADER_SIZE == 0);

			// Fill in blob data
			uint8_t *blobData = &packetBuffer[STREAM_PACKET_HEADER_SIZE];
			for (int i = 0; i < clusters.size(); i++)
			{
				Cluster *blob = &clusters[i];
				int base = i*STREAM_PACKET_BLOB_SIZE;
				uint16_t blobX = (blob->centroid.x()+0.5f) * std::numeric_limits<uint16_t>::max() / state.camera.width;
				uint16_t blobY = (blob->centroid.y()+0.5f) * std::numeric_limits<uint16_t>::max() / state.camera.height;
				uint8_t blobSz = std::min<int>(blob->size * 2, std::numeric_limits<uint8_t>::max());
				uint8_t blobVal = std::min<int>(clusters[i].value/4, std::numeric_limits<uint8_t>::max());
				*(uint16_t*)&blobData[base+0] = blobX;
				*(uint16_t*)&blobData[base+2] = blobY;
				*(uint8_t*)&blobData[base+4] = blobSz;
				*(uint8_t*)&blobData[base+5] = blobVal;
				static_assert(STREAM_PACKET_BLOB_SIZE == 6);
		//		int blobX = (int)((double)blob->centroid.x() * 16383 / state.camera.width);
		//		int blobY = (int)((double)blob->centroid.y() * 16383 / state.camera.height);
		//		int blobS = std::min((int)(blob->centroid.S * 2), 255);
		//		uint32_t data = (blobX << 0) | (blobY << 14)
		//				|  << 22
		//				| (blobColor & ((1<<4)-1)) << 28;
		//		blobData[i] = data;
			}

			if (comm_packet(rtComm, PacketHeader(PACKET_BLOB, reportLength, curFrame->ID&0xFF)))
			{
				comm_write(rtComm, packetBuffer.data(), reportLength);
				comm_submit(rtComm);
			}
			//comm_send(realTimeAff, PacketHeader(PACKET_BLOB, reportLength, curFrame->ID&0xFF), packetBuffer.data());
			bytesSentAccum += packetBuffer.size();
			curTimes.send += dtUS(t0, sclock::now());
		}

		if (state.curMode.mode == TRCAM_MODE_VISUAL)
		{ // Visual Debug mode
			// TODO: Deprecate Visual Debug. Have full blob detection emulation instead 

			// Select blob to debug
			int maxInd = -1, maxSz = 4;
			for (int i = 0; i < clusters.size(); i++)
			{
				Cluster &blob = clusters[i];
				int size = blob.bounds.size();
				if (size < 1000 && size > maxSz)
				{
					maxInd = i;
					maxSz = size;
				}
			}
			
			if (maxInd >= 0)
			{
				Cluster &blob = clusters[maxInd];

				// Refine blob
				std::vector<Vector2<float>> edgeRefined;
				std::vector<bool> edgeOutliers;
				refineCluster(blob, curFrame->memory, srcStride,
					state.blobParams.refinement, estimateHoughParameters1(blob), 10,
					edgeRefined, edgeOutliers);
				edgeRefined.clear();

				// Determine sizes needed for buffer
				auto bounds = blob.bounds;
				bounds.extendBy(Vector2<uint16_t>(2, 2));
				bounds.overlapWith(Bounds2<uint16_t>(0, 0, state.camera.width, state.camera.height));
				auto extends = bounds.extends();
				int ptSize = edgeRefined.size()*3;
				int imgSize = extends.prod();
				int bitSize = imgSize / 8;
				if (imgSize & 7) bitSize++;
				int metaSize = 2*4 + 2*2 + 2 + 2; // Bounds, Center, Size, Refinement Method + Point Count

				TimePoint_t t0 = sclock::now();

				// Write header
				int packetLength = metaSize + imgSize + bitSize + ptSize;
				if (packetBuffer.size() < packetLength)
					packetBuffer.resize(packetLength*3/2);

				// Write metadata
				uint16_t *metaData = (uint16_t*)packetBuffer.data();
				metaData[0] = bounds.min.x();
				metaData[1] = bounds.min.y();
				metaData[2] = bounds.max.x();
				metaData[3] = bounds.max.y();
				metaData[4] = (uint16_t)(((double)blob.centroid.x()+0.5) * 65536 / state.camera.width);
				metaData[5] = (uint16_t)(((double)blob.centroid.y()+0.5) * 65536 / state.camera.height);
				metaData[6] = (uint16_t)((double)blob.size * 65536 / 256);
				metaData[7] = (edgeRefined.size() & 0xFFFF);

				// Read out image data from bounds
				uint8_t *imgData = (uint8_t*)metaData + metaSize;
				for (int y = 0; y < extends.y(); y++)
				{
					int imgPos = y*extends.x();
					int ptrPos = (bounds.min.y()+y)*srcStride + bounds.min.x();
					memcpy(imgData + imgPos, curFrame->memory + ptrPos, extends.x());
				}
				
				// Write bit buf
				uint8_t *bitBuf = imgData + imgSize;
				memset(bitBuf, 0, bitSize);
				for (int i = 0; i < blob.dots.size(); i++)
				{
					Vector2<uint16_t> pt = blob.dots[i] - bounds.min;
					int pos = pt.y()*extends.x() + pt.x();
					bitBuf[pos/8] |= 1 << (pos % 8);
				}
				
				// Write points
				uint8_t *ptBuf = bitBuf + bitSize;
				/* for (int i = 0; i < edgeRefined.size(); i++)
				{
					Vector2<float> pt = edgeRefined[i] - bounds.min().cast<float>();
					pt = pt.cwiseQuotient(bounds.extends().cast<float>());
					uint32_t dat = 0;
					dat |= (uint16_t)(pt.x() * 2047) << 13;
					dat |= (uint16_t)(pt.y() * 2047) << 2;
					dat |= edgeOutliers[i]? 0x01 : 0x00;
					ptBuf[i*3+0] = (dat >> 16)&0xFF;
					ptBuf[i*3+1] = (dat >> 8)&0xFF;
					ptBuf[i*3+2] = (dat >> 0)&0xFF;
				} */

				//printf("Selected blob %d of size %d = %dx%d (%d, %d, %d, %d) with %d points, size %f, as debug target! Total report length is %d!\n",
				//	maxInd, maxSz, extends.x(), extends.y(), blob.bounds.minX, blob.bounds.minY, blob.bounds.maxX, blob.bounds.maxY, blob.ptCnt, blob.centroid.S, blobPayloadLength);

				if (comm_packet(rtComm, PacketHeader(PACKET_VISUAL, packetLength, curFrame->ID&0xFF)))
				{
					comm_write(rtComm, packetBuffer.data(), packetLength);
					comm_submit(rtComm);
				}
				//comm_send(rtComm, PacketHeader(PACKET_VISUAL, packetLength, curFrame->ID&0xFF), packetBuffer.data());
				bytesSentAccum += packetLength;
				curTimes.send += dtUS(t0, sclock::now());
			}
			else 
			{ // Write empty packet
				TimePoint_t t0 = sclock::now();
				comm_send(rtComm, PacketHeader(PACKET_VISUAL, 0, curFrame->ID&0xFF), nullptr);
				curTimes.send += dtUS(t0, sclock::now());
			}
		}


		// ---- Framebuffer debugging ----

		if (state.visualisation.enabled && state.visualisation.initialised && (numFrames % state.visualisation.interval) == 0)
		{
			if (!isVisualising.exchange(true))
			{
				static std::vector<Cluster> pastBlobs, curBlobs;
				curBlobs = clusters;
				threadPool.push([&](int, std::shared_ptr<FrameBuffer> frame){
					prctl(PR_SET_NAME, "Worker_Vis");
					nice(20); // Low priority
					TimePoint_t t0 = sclock::now();

					state.visualisation.visualise(curBlobs, pastBlobs, frame->memory, state.camera.width, state.camera.height, srcStride);
					pastBlobs.swap(curBlobs);

					recordIncident(incidents.vis, dtUS(t0, sclock::now()));
					isVisualising = false;
				}, curFrame);
			}
		}

		if (state.streaming.enabled && comm_anyReady(comms)
			&& (state.streaming.frame == 0 || (curFrame->ID % state.streaming.frame) == 0)
			&& !isPreparingFrame.exchange(true))
		{ // Send image of current frame as part of continous streaming
			threadPool.push([&](int, std::shared_ptr<FrameBuffer> frame, ImageStreamState stream){
				prctl(PR_SET_NAME, "Worker_Stream");
				nice(20); // Low priority
				TimePoint_t start = sclock::now();
				if (prepareImageStreamingPacket(*frame, stream))
				{
					recordIncident(incidents.stream, dtUS(start, sclock::now())>>8);
				}
				isPreparingFrame = false;
			}, curFrame, state.streaming);
		}
		else if (!state.imageRequests.empty() && comm_anyReady(comms)
			&& !isPreparingFrame.exchange(true))
		{ // Send image of past frame as part of a spontaneous request
			auto imageRequest = state.imageRequests.front();
			state.imageRequests.pop();

			// TODO: Properly keep some past frames around - need to increase buffers everywhere
			std::shared_ptr<FrameBuffer> &reqFrame = curFrame; // For imageRequest.frame

			threadPool.push([&](int, std::shared_ptr<FrameBuffer> frame, ImageStreamState stream){
				prctl(PR_SET_NAME, "Worker_Oneshot");
				nice(20); // Low priority
				//TimePoint_t start = sclock::now();
				if (prepareImageStreamingPacket(*frame, stream))
				{
					//recordIncident(incidents.stream, dtUS(start, sclock::now())>>8);
				}
				isPreparingFrame = false;
			}, reqFrame, imageRequest);
		}

		// Increase clock rate after boot (likely has to be done through linux CPU governor, not directly)
		//printf("Had ARM Core clock rate of %dMhz!\n", getClockRate(base.mb, 3)/1000000);
		//setClockRate(base.mb, 3, 600000000); // 1000Mhz fixed
		//printf("Set ARM Core clock rate to fixed %dMhz!\n", getClockRate(base.mb, 3)/1000000);

		// Allow sending from start of comm until next frame is passed to CPU, at which point we need to send a SOF to host software
		// Assumes near constant QPU processing time
		bytesSentAccum += sendInterleavedQueuedPackets(avgInterval*1000 - dtUS(time_cpu_begin, sclock::now()) - 500);
		// TODO: Very often doesn't send in time, something is likely off in the calculations

		TimePoint_t time_report_end = sclock::now();


		// ---- Performance and Statistics ----

		{
			int avgNum = std::min(numFrames-frameNum_lastStatCheck, (uint32_t)1000);
			skip_count_trigger += curFrame->skippedTrigger;
			skip_count_qpu += curFrame->skippedQPU;
			skip_count_cpu += curFrame->skippedCPU;
			curTimes.qpu = curFrame->time_qpu;
			curTimes.cpu = dtUS(time_cpu_begin, time_cpu_end);
			curTimes.processing = curTimes.qpu + curTimes.cpu;
			// Doesn't including reporting times ( & comms)
			[[maybe_unused]] uint16_t report = dtUS(time_report_begin, time_report_end);
			curTimes.latency = dtUS(curFrame->rcv, time_cpu_end);
			// Update average
			addDeltaTimes(avgTimes, curTimes, avgNum);
			if (numFrames > 20 && curTimes.latency*2 > avgTimes.latency*3)
			{ // Update latency records
				recordIncident(incidents.lag, curTimes.latency);
				//addDeltaTimes(lagTimes, curTimes, incidences.lag.occurences);
			}

#ifdef LOG_QPU
			if (numFrames % 10 == 0)
			{ // Detailed QPU performance gathering (every 10th frame to handle QPU performance register overflows)
				qpu_updatePerformance(&base, &qpu.perf);
			}
#endif
		}
	

		if (numFrames % time_debug_interval == 0)
		{ // Frames per second
			TimePoint_t currentTime = sclock::now();
			uint32_t deltaStatUS = dtUS(time_lastStatCheck, currentTime);

			StatPacket stats = prepareSystemStatusPacket(state, deltaStatUS);
			stats.header.frame = numFrames;
			stats.header.skipTrigger = std::min(255, skip_count_trigger);
			stats.header.skipCPU = std::min(255, skip_count_cpu);
			stats.header.skipQPU = std::min(255, skip_count_qpu);
			stats.times = avgTimes;
			stats.incidents = incidents;
			// Write byte representation
			uint8_t statPacket[STAT_PACKET_SIZE];
			storeStatPacket(stats, statPacket);
			// Send packet
			comm_send(largeDataAff, PacketHeader(PACKET_STAT, sizeof(statPacket)), statPacket);

			if (state.writeStatLogs)
			{
				float elapsedS = deltaStatUS/1000000.0f;
				float fps = time_debug_interval / elapsedS;

				printf("%d: %.1ffps, %.1f°C, %.2fV, ", numFrames, fps, stats.header.tempSOC/100.0f, stats.header.voltage/1000.0f);
				printDeltaTimes(avgTimes);
				if (state.visualisation.enabled)
					printf(", Vis: (%dus, %.0fHz)", incidents.vis.avg, incidents.vis.occurences/elapsedS);
				if (state.streaming.enabled)
					printf(", Stream: (%.2fms, %.0fHz)", (incidents.stream.avg<<8)/1000.0f, incidents.stream.occurences/elapsedS);
				printf("\n");

				if (incidents.lag.occurences > 0 || skip_count_trigger > 0 || skip_count_qpu > 0 || skip_count_cpu > 0)
				{
					printf("%d triggers dropped, %d QPU-dropped, %d CPU-dropped, %d/%d lagged",
						skip_count_trigger, skip_count_qpu, skip_count_cpu, incidents.lag.occurences, time_debug_interval);
					if (incidents.lag.occurences > 0)
					{
						printf(" (max %.2fms)", incidents.lag.max/1000.0f);
						//PrintTimes(lagTimes);
						//lagTimes = StatPacket::AvgTimes();
					}
					printf("\n");
				}
				if (incidents.await.occurences || incidents.skip.occurences || incidents.access.occurences
					|| incidents.proc.occurences || incidents.handle.occurences || incidents.cpuwait.occurences)
				{
					printf("Lag incidences: ");
					printIncidents(incidents.await, "await");
					printIncidents(incidents.skip, "skip");
					printIncidents(incidents.access, "access");
					printIncidents(incidents.proc, "proc");
					printIncidents(incidents.handle, "handle");
					printIncidents(incidents.cpuwait, "CPUwait");
					printf("\n");
				}
#ifdef LOG_QPU // Log QPU performance
				qpu_logPerformance(&qpu.perf);
				fflush(stdout);
#endif
			}

			avgTimes = {};
			incidents = {};
			skip_count_trigger = 0;
			skip_count_qpu = 0;
			skip_count_cpu = 0;

			// Update debug interval
			//if (numFrames >= 10000)
			//	time_debug_interval = 10000;
			//else 
			if (numFrames >= 1000)
				time_debug_interval = 1000;
			time_lastStatCheck = currentTime;
			frameNum_lastStatCheck = numFrames;
		}

		// Another low-priority packet, send after frame streaming is done, but not when a stat packet is sent to keep line clear
		else if (state.wireless.sendStatus)
		{ // Notify host of wireless status
			sendWirelessStatusPacket(state);
		}

		bytesSentAccum = 0;
	}

	for (auto &comm : comms.medium)
		if (comm)
			comm->realtimeControlled = false;

	printf("-- Blob Detection Stopping --\n");

	abortStreaming = true;
	if (qpuThread->joinable())
		qpuThread->join();
	printf("-- QPU Thread Stopped --\n");

	threadPool.clear_queue();
	while (threadPool.n_idle() < threadPool.size())
		sched_yield();
	printf("-- Thread Pool Done --\n");

#ifdef RUN_CAMERA
	gcs_stop(gcs);
	printf("-- Camera Stream Stopped --\n");
#else
#ifdef EMUL_VCSM
	for (int i = 0; i < emulBufCnt; i++)
		vcsm_free(camEmulBuf[i]);
#else
	for (int i = 0; i < emulBufCnt; i++)
		qpu_releaseBuffer(&camEmulBuf[i]);
#endif
#endif

	nice(0); // Normal priority

	return abortProgram;
}