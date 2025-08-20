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
#include <list>
#include <chrono>
#include <thread>
#include <atomic>
#include <string>
#include <math.h>
#include <sched.h>
#include <fstream>
#include <execinfo.h>
#include <signal.h>

// Console and UART
#include <termios.h>
#include "util/fbUtil.h"

#include "qpu/qpu_program.h"
#include "qpu/qpu_info.h"
#include "camera/gcs.hpp"
#include "blob/blob.hpp"
#include "blob/qpu_blob_tiled.hpp"

#include "state.hpp"
#include "options.hpp"
#include "visualisation.hpp"

#include "mcu/mcu.hpp"

#include "comm/comm.hpp"
#include "comm/uart.hpp"
#include "comm/server.hpp"
#include "comm/wireless.hpp"
#include "util/image.hpp"

#include "ctpl/ctpl.hpp"

#include "util/util.hpp"

#include "vcsm/vcsm.hpp"

#define RUN_CAMERA	// Have the camera supply frames (else: emulate camera buffers)
//#define EMUL_VCSM	// Use VCSM for Emulation buffers instead of Mailbox allocated QPU buffers
//#define LOG_QPU
#define LOG_SHORT
//#define LOG_DROPS
//#define LOG_TIMING

bool isZero2 = false;
int numCPUCores = 0;

// Number of emulated buffers to iterate over if RUN_CAMERA is not defined
const int emulBufCnt = 4;

// Terminal Output and Input
bool isConsole;
struct termios terminalSettings;
static void setConsoleRawMode();


// Static initialised members (for atexit)
static TrackingCameraState state = {};
static QPU_BASE base;
GCS *gcs = NULL;
CommList comms;
static std::thread *uartThread;
static std::thread *serverThread;
ErrorTag error = ERROR_NONE;
bool hasMCU;

static ctpl::thread_pool threadPool(6);
static std::atomic<bool> isVisualising, isPreparingFrame;
static struct
{
	bool sending;
	uint8_t header[22];
	std::vector<uint8_t> data;
	int sendProgress;
	uint8_t ID;
	PacketTag tag;
} largePacket;
// Keeping track of frame buffers
static std::mutex frameAccess;
struct FrameBuffer
{
	void* header = NULL;
	uint8_t* memory = NULL;
	uint16_t stride;
	QPU_BUFFER *bitmsk = NULL;
	uint32_t ID = 0;
	TimePoint_t SOF, rcv;
	uint32_t time_qpu;
	int skippedTrigger, skippedCPU, skippedQPU;

	~FrameBuffer()
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

static bool prepareImageStreamingPacket(const FrameBuffer &frame, ImageStreamState stream);

static void sendWirelessStatusPacket(TrackingCameraState &state);
static int sendLargePacketData(TrackingCameraState &state, int commTimeUS);

/* Functions */

static void EnterStreamingState()
{
	printf("== Entering camera streaming mode! ==\n");
	std::unique_lock lock(state.sync.access);
	state.sync.frameSOFs = {};
	state.sync.SOF2RecvDelay.reset();
}

static void LeaveStreamingState()
{
	printf("== Left camera streaming mode! ==\n");
	state.imageRequests = {};
}

bool handleError(bool NAK = true, std::string backtrace = "")
{
	if (error < ERROR_MAX)
		fprintf(stderr, "Encountered Error %d: %s!\n", (int)error, ErrorTag_String[(int)error]);
	if (comm_anyEnabled(comms))
	{
		bool serious = error != ERROR_GCS_TIMEOUT;
		void *packet = comm_packet(comms, PacketHeader(PACKET_ERROR, 2+backtrace.size()));
		comm_write(comms, packet, (uint8_t*)&error, 1);
		comm_write(comms, packet, (uint8_t*)&serious, 1);
		comm_write(comms, packet, (uint8_t*)backtrace.data(), backtrace.size());
		comm_submit(comms, packet);
		comm_flush(comms);
		if (NAK)
			comm_NAK(comms);
	}
	return true;
}

void crash_handler(int signal)
{
	fprintf(stderr, "Error Signal %d:\n", signal);
	void *array[32];
	size_t size = backtrace(array, 32);
	backtrace_symbols_fd(array, size, STDERR_FILENO);
	char **btcharr = backtrace_symbols(array, size);
	// This is not displaying line number, only functions.
	// TODO: Use libbacktrace for perfect backtrace
	std::string btstr;
	for (int i = 0; i < size; i++)
	{
		btstr.append(btcharr[i]);
		btstr.push_back('\0');
	}
	free(btcharr);
	switch(signal)
	{
		case SIGILL:
			error = ERROR_EXCEPTION_ILLEGAL;
			break;
		case SIGBUS:
			error = ERROR_EXCEPTION_BUS;
			break;
		case SIGFPE:
			error = ERROR_EXCEPTION_FPE;
			break;
		case SIGSEGV:
			error = ERROR_EXCEPTION_SEGFAULT;
			break;
		case SIGPIPE:
			error = ERROR_EXCEPTION_PIPE;
			break;
		default:
			error = ERROR_EXCEPTION_UNKNOWN;
			break;
	}
	if (handleError(true, btstr))
		exit(error);
}

int main(int argc, char **argv)
{
	TimePoint_t time_start = sclock::now();

	// ---- Init Application ----

	// Setup handler to catch segfaults
	struct sigaction action;
	action.sa_handler = crash_handler;
	sigaction(SIGILL, &action, NULL);
	sigaction(SIGBUS, &action, NULL);
	sigaction(SIGFPE, &action, NULL);
	sigaction(SIGSEGV, &action, NULL);
	sigaction(SIGPIPE, &action, NULL);

	// Read and validate arguments
	if (!options_read(state, argc, argv))
		return -1;

	srand((unsigned int)time(NULL));

	// Should be read from permanent, unique file in shell
	if (state.id == 0)
		state.id = rand();

	// Check MCU presence
	bool hasMCU = false;
	if (mcu_init())
	{
		hasMCU = mcu_probe();
		if (!hasMCU && mcu_probe_bootloader())
		{
			if (!mcu_verify_program(state.mcuFile))
			{
				printf("MCU is bricked with invalid firmware, will re-flash!\n");
				if (mcu_flash_program(state.mcuFile))
					printf("Successfully re-flashed MCU in attempt to recover it!\n");
			}
			else
			{
				printf("MCU is bricked, but firmware has been validated!\n");
			}
			// Whether we reflashed it or not, reset it into normal mode and try again
			mcu_reset();
			hasMCU = mcu_probe();
		}
	}
	atexit(mcu_cleanup);

	// Init VCSM
	if (!vcsm_init())
	{
		printf("Failed to init VCSM!\n");
		return -1;
	}
	atexit(vcsm_exit);

	// Modify tty to allow for non-blocking input (for console & ssh, but not background execution)
	isConsole = isatty(STDIN_FILENO);
	if (isConsole)
		setConsoleRawMode();

	// Init visualisation resources
	state.visualisation.fbfd = setupFrameBuffer(&state.visualisation.vinfo, &state.visualisation.finfo, false);
	state.visualisation.initialised = state.visualisation.fbfd != 0;
	if (!state.visualisation.initialised)
		printf("Failed to initialise visualisation with framebuffer!\n");
	atexit([]{ // Close at exit
		if (state.visualisation.fbfd != 0)
			close(state.visualisation.fbfd);
	});

	// Init QPU Base (basic information to work with QPU), e.g. VCHIQ mailbox
	if (int errCode = qpu_initBase(&base) != 0)
	{ // More a dev error, should not happen on a deployed system
		printf("Failed to init QPU Base! %d\n", errCode);
		return -1;
	}
	atexit([]{ // Close at exit
		qpu_destroyBase(&base);
	});

	{
		std::string modelName(128, 0);
		std::ifstream stream("/proc/device-tree/model");
		stream.read((char*)modelName.data(), modelName.capacity());
		modelName.resize(stream.gcount());
		isZero2 = modelName.find("Zero 2");
		numCPUCores = std::thread::hardware_concurrency();
	}

	// TODO: Proper Thread Core distribution
	// Assuming 4-core
	// All threads that are mostly idling are on CPU 0
		// To this belong:
			// Main thread - instructing other threads, writing blob reports - mostly waiting
			// Comm threads - reading/writing, updating main thread - mostly waiting
			// QPU Thread - instructing QPU, frame timing - mostly waiting, only one
		// Need to allow system threads to work
			// Either realtime and mostly sleeping
			// Or not realtime but high-priority
	// CPU blob processing is done in parallel:
		// Likely realtime and solely on CPU 1-3
			// Possibly 3 manual threads
			// Or generic task pool if those can be made to be realtime priority

	// Increase priority in normal scheduler
	//nice(-20);

	// Set as non-exemptive realtime process using FIFO scheduler
	//struct sched_param schedparam;
	//schedparam.sched_priority = sched_get_priority_max(SCHED_FIFO);
	//sched_setscheduler(0, SCHED_FIFO, &schedparam);

	// Set CPU affinity to force this thread on one specific core
	/*cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(3, &set);
	sched_setaffinity(0, sizeof(set), &set);*/

	// Increase clock rate during processing phases
	// (likely has to be done through linux CPU governor, not directly)
	//setClockRate(base.mb, 3, 1000000000); // 1000Mhz fixed
	//printf("Set ARM Core clock rate to fixed %dMhz!\n", getClockRate(base.mb, 3)/1000000);

	// ----Setup Communications ----

	comm_init();

	// TODO: update versions
	VersionDesc version(0, 0, 0);
	IdentPacket ident (state.id, DEVICE_TRCAM, (InterfaceTag)0, version);

	// UART communications
	{ // Init whether used or not, might be enabled later on
		state.uart.protocol.needsErrorScanning = true;
		state.uart.port = uart_init(state.serialName);
		state.uart.start = uart_start;
		state.uart.stop = uart_stop;
		state.uart.wait = uart_wait;
		state.uart.configure = uart_configure;
		state.uart.read = uart_read;
		state.uart.write = uart_write;
		state.uart.submit = uart_submit;
		state.uart.flush = uart_flush;
		state.uart.ownIdent = ident;
		state.uart.ownIdent.type = INTERFACE_UART;
		state.uart.expIdent.device = DEVICE_TRCONT;
		state.uart.started = false;
	}
	atexit([]{ // Close at exit
		state.uart.enabled = false;
		if (uartThread && uartThread->joinable())
			uartThread->join();
		delete uartThread;
		uart_deinit(state.uart.port);
	});
	if (state.uart.enabled)
	{
		uartThread = new std::thread(CommThread, &state.uart, &state);
		comms.arr[comms.cnt++] = &state.uart;
		printf("Initiating UART connection...\n");
	}

	// Server communications
	{ // Init whether used or not, might be enabled later on
		state.server.protocol.needsErrorScanning = false;
		state.server.port = server_init(state.server_host, state.server_port);
		state.server.start = server_start;
		state.server.stop = server_stop;
		state.server.wait = server_wait;
		state.server.configure = nullptr;
		state.server.read = server_read;
		state.server.write = server_write;
		state.server.submit = server_submit;
		state.server.flush = server_flush;
		state.server.ownIdent = ident;
		state.server.ownIdent.type = INTERFACE_SERVER;
		state.server.expIdent.device = DEVICE_SERVER;
		state.server.started = false;
	}
	atexit([]{ // Close at exit
		state.server.enabled = false;
		if (serverThread && serverThread->joinable())
			serverThread->join();
		delete serverThread;
		server_deinit(state.server.port);
	});
	if (state.server.enabled)
	{
		serverThread = new std::thread(CommThread, &state.server, &state);
		comms.arr[comms.cnt++] = &state.server;
		printf("Initiating server connection...\n");
	}

	bool needsComms = comm_anyEnabled(comms);


	// ---- Wireless ----

	initWirelessMonitor(state);
	atexit([]{ // Close at exit
		stopWirelessMonitor(state);
	});


	// ---- Start Loop ----

	bool running = true; // Only for interactive operation
	uint32_t lastErrorFrames = 0, numFrames;

	while (running)
	{

		// ---- Initialise based on communication ---

		if (!state.server.enabled && !state.uart.enabled)
		{ // Skip to streaming in interactive mode (likely on the console)
			state.curMode.streaming = true;
			state.curMode.mode = TRCAM_MODE_BLOB;
			state.curMode.opt = TRCAM_OPT_NONE;
			// If any of these modes need init, write to state.newMode and set state.updateMode = true!
		}
		else 
		{
			state.curMode.streaming = false;
			state.curMode.mode = TRCAM_STANDBY;
			state.curMode.opt = TRCAM_OPT_NONE;
		}

		while ((state.server.enabled || state.uart.enabled) && !state.curMode.streaming)
		{ // Wait for a connected communications channel to instruct a stream start
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			if (isConsole)
			{ // Check input
				char cin;
				if (read(STDIN_FILENO, &cin, 1) == 1)
				{
					if (cin == 'q')
					{ // Complete stop of program requested
						return EXIT_SUCCESS;
					}
					else if (cin == 'e')
					{
						mcu_reset();
					}
					else if (cin == 'b')
					{
						mcu_switch_bootloader();
					}
					else if (cin == 'f')
					{
						mcu_flash_program(state.mcuFile);
					}
					else if (cin == 'v')
					{
						mcu_verify_program(state.mcuFile);
					}
				}
			}

			if (state.firmware.applyingUpdate)
			{ // We're supposed to apply a firmware update
				ApplyFirmwareUpdate(state);
			}

			if (state.postFirmwareActions != FW_FLAGS_NONE)
			{ // Apply post-firmware actions like flashing MCU or rebooting
				PostFirmwareUpdateActions(state);
			}

			if (state.updateSetupCPU.exchange(false))
			{ // Got a new setup packet to read
				state.updateSetupQPU = false;
				acceptCPUConfig(state);
				acceptQPUConfig(state);
			}

			if (state.updateMode.exchange(false))
			{ // Switch to specified mode
				if (state.newMode.streaming && !state.firmware)
				{ // Perform actual mode switch (and initialisation) in stream loop
					state.curMode.streaming = true;
					state.updateMode = true;
					printf("== Received command to enter streaming mode! ==\n");
					break;
				}
				// No change, but need to notify host anyway
				static_assert(TRCAM_MODE_SIZE == std::numeric_limits<uint8_t>::max());
				uint8_t mode = (uint8_t)TRCAM_STANDBY;
				void *packet = comm_packet(comms, PacketHeader(PACKET_MODE, 1));
				comm_write(comms, packet, &mode, 1);
				comm_submit(comms, packet);
			}

			if (state.wireless.sendStatus)
			{ // Notify host of wireless status
				state.wireless.sendStatus = false;
				state.wireless.lastStatus = sclock::now();
				sendWirelessStatusPacket(state);
			}
		}

		EnterStreamingState();

		// ---- Setup camera stream ----

#ifdef RUN_CAMERA

		// Init camera subsystem
		int gcsSensor = gcs_findCamera();
		if (gcsSensor == -1)
		{
			printf("Failed to open camera I2C!\n");
			error = ERROR_GCS_NO_I2C;
			if (handleError())
				break;
		}
		else if (gcsSensor == 0)
		{
			printf("Failed to identify camera sensor!\n");
			error = ERROR_GCS_NO_SENSOR;
			if (handleError())
				break;
		}

		gcs = gcs_create(&state.camera);
		if (gcs == NULL)
		{
			printf("Failed to initialise camera!\n");
			error = ERROR_GCS_CREATE;
			if (handleError())
				break;
		}
#else // Camera emulation buffers
	#ifdef EMUL_VCSM
		VCSM_BUFFER camEmulBuf[emulBufCnt];
	#else
		QPU_BUFFER camEmulBuf[emulBufCnt];
	#endif
#endif

		auto cleanup_camera = [&]()
		{
#ifdef RUN_CAMERA
			std::unique_lock lock(frameAccess);
			gcs_destroy(gcs);
			gcs = NULL;
			printf("-- Camera Stream Cleaned --\n");
#endif
		};


		// ---- Generate tiling setup ----

		// QPU core usage
		int qpuCoresUsed = 0;
		for (int i = 0; i < 12; i++)
			qpuCoresUsed += state.enableQPU[i]? 1 : 0;

		// Layout QPU program instances across camera frame according to cores used
		ProgramLayout layout = SetupProgramLayout(state.camera.width, state.camera.height, qpuCoresUsed);

#ifdef RUN_CAMERA
		uint32_t srcStride = state.camera.stride;
#else // Camera emulation buffers
		uint32_t srcStride = ROUND_UP(state.camera.width, 32);
#endif


		// ---- Setup blob detection ----

		if (!initBlobDetection(layout.maskSize, layout.maskOffset, numCPUCores))
		{ // Shouldn't actually fail, just basic init
			printf("-- Failed to initialize blob detection! --\n");
			cleanup_camera();
			error = ERROR_INIT_BD;
			if (handleError())
				break;
		}
		if (state.curMode.mode == TRCAM_MODE_BGCALIB)
		{
			if (state.curMode.opt == TRCAM_OPT_NONE)
				initBackgroundCalibration();
		}

		auto cleanup_blob_detect = []()
		{
			cleanBlobDetection();
			printf("-- Blob Detection Cleaned --\n");
		};


		// ---- Setup QPU resources ----

		// Set up bit target, one bit per pixel
		const int bitmskCount = 2;
		QPU_BUFFER bitmskBuffer[bitmskCount];
		int bitmskSwitch = 0;
		qpu_allocBuffer(bitmskBuffer+0, &base, layout.maskSize.prod()/8, 4096);
		qpu_allocBuffer(bitmskBuffer+1, &base, layout.maskSize.prod()/8, 4096);

		// Setup main blob detection program with specified progmem sizes
		QPU_PROGRAM blobProgram;
		qpu_initProgram(&blobProgram, &base, (QPU_PROGMEM){
			.codeSize = qpu_getCodeSize(state.codeFile.c_str()),
			.uniformsSize = (uint32_t)layout.instances*numUnif,
			.messageSize = 2
		});
		qpu_loadProgramCode(&blobProgram, state.codeFile.c_str());

		// Set up uniforms of the blob QPU program
		qpu_lockBuffer(&blobProgram.progmem_buffer);
		SetupProgramUniforms(layout, blobProgram.progmem.uniforms.arm.uptr, srcStride);
		SetupProgramSettings(layout, blobProgram.progmem.uniforms.arm.uptr, state.thresholds.absolute, state.thresholds.edge);
		qpu_unlockBuffer(&blobProgram.progmem_buffer);

		auto cleanup_qpu_resources = [&]()
		{
			qpu_destroyProgram(&blobProgram);
			qpu_releaseBuffer(bitmskBuffer+0);
			qpu_releaseBuffer(bitmskBuffer+1);

			printf("-- QPU Cleaned --\n");
		};


		// ---- Enable QPU ----

		// Enable QPU - this locks the QPU for our use and prevents other use (e.g. camera AWB, OpenGL ES)
		// We could do this setup on every frame and then disable/release the QPU after we are done
		// See https://github.com/raspberrypi/firmware/issues/793
		if (qpu_enable(base.mb, 1))
		{
			printf("QPU enable failed!\n");
			cleanup_qpu_resources();
			cleanup_blob_detect();
			cleanup_camera();
			error = ERROR_QPU_ENABLE;
			if (handleError())
				break;
		}
		printf("-- QPU Enabled --\n");

		// QPU scheduler reservation
	//	for (int i = 0; i < 12; i++) // Enable only QPUs selected as parameter, disable others completely
	//		qpu_setReservationSetting(&base, i, state.enableQPU[i]? 0b1110 : 0b1111);
		for (int i = 0; i < 12; i++) // Enable only QPUs selected as parameter, allow GL shaders on others
			qpu_setReservationSetting(&base, i, state.enableQPU[i]? 0b1110 : 0b0001);
#ifdef LOG_QPU
		qpu_logReservationSettings(&base);

		// Debug QPU Hardware
		qpu_debugHW(&base);

		// VPM memory reservation
		base.peripherals[V3D_VPMBASE] = 16; // times 4 to get number of vectors; Default: 8 (32/4), Max: 16 (64/4)
		QPU_HWConfiguration hwConfig;
		qpu_getHWConfiguration(&hwConfig, &base);
		QPU_UserProgramInfo upInfo;
		qpu_getUserProgramInfo(&upInfo, &base);
		printf("Reserved %d / %d vectors of VPM memory for user programs!\n", upInfo.VPMURSV_V, hwConfig.VPMSZ_V);
#endif

		// Setup performance monitoring
		QPU_PerformanceState perfState;
		qpu_setupPerformanceCounters(&base, &perfState);
		perfState.qpusUsed = std::min(layout.instances, qpuCoresUsed);

		auto cleanup_qpu_enable = [&]()
		{
			for (int i = 0; i < 12; i++) // Reset all QPUs to be freely sheduled
				qpu_setReservationSetting(&base, i, 0b0000);

			// Disable QPU
			if (qpu_enable(base.mb, 0))
				printf("-- QPU Disable Failed --\n");
			else
				printf("-- QPU Disabled --\n");
		};


		// ---- Start camera stream ----

		// Start GPU camera stream
#ifdef RUN_CAMERA
		if (gcs_start(gcs) != 0)
		{
			printf("-- Failed to start camera stream! --\n");
			gcs_readErrorFlag(gcs); // Reset error flag
			cleanup_qpu_enable();
			cleanup_qpu_resources();
			cleanup_blob_detect();
			cleanup_camera();
			error = ERROR_GCS_START;
			if (handleError())
				break;
		}
		printf("-- Camera Stream started --\n");
#else

		// Allocate emulated buffers
	#ifdef EMUL_VCSM
		for (int i = 0; i < emulBufCnt; i++)
		{ // Allocate only grayscale buffer
			camEmulBuf[i] = vcsm_malloc(srcStride*state.camera.height);
			if (camEmulBuf[i].fd < 0)
			{
				printf("Failed to allocate vcsm buffer!\n");
				return -1;
			}
		}
	#else
		for (int i = 0; i < emulBufCnt; i++)
		{ // Allocate only grayscale buffer
			qpu_allocBuffer(&camEmulBuf[i], &base, srcStride*state.camera.height, 4096);
		}
	#endif

		// Generate random test frames
		for (int i = 0; i < emulBufCnt; i++)
		{
	#ifdef EMUL_VCSM
			if (!vcsm_lock(camEmulBuf[i]))
			{
				printf("Failed to lock vcsm buffer!");
				return -1;
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
		numFrames = 0;
		StatPacket::Times avgTimes = {}, curTimes = {};
		int skip_count_trigger = 0, skip_count_qpu = 0, skip_count_cpu = 0;

		// Prediction when a new frame will arrive through V4L2
		float avgInterval = 0.0f;
		TimePoint_t time_frameRecv;
		
		// Setup incident statistics
		StatPacket::Incidents incidents = {};
		// Limits achievable on Zero 1 and Zero 2 without overclocking, with some spikes
		uint16_t awaitLimit = 1000000/state.camera.fps,
			skipLimit = isZero2? 30 : 10, // Yes, really
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
					qpu_lockBuffer(&blobProgram.progmem_buffer);
					SetupProgramSettings(layout, blobProgram.progmem.uniforms.arm.uptr, state.thresholds.absolute, state.thresholds.edge);
					qpu_unlockBuffer(&blobProgram.progmem_buffer);
				}

#ifdef RUN_CAMERA
				auto waitForFrame = [&]() -> bool
				{
					// Wait for a frame to be received in the VideoCore
					TimePoint_t waitStart = sclock::now(), errorSent;
					bool sentTimeoutError = false;
					while (gcs_waitForFrameBuffer(gcs, 1000) == 0 && !abortStreaming)
					{
						if (needsComms && !comm_anyReady(comms))
						{
							printf("== Lost comms! Stopping streaming! ==\n");
							state.curMode.streaming = false;
							state.curMode.mode = TRCAM_STANDBY;
							state.curMode.opt = TRCAM_OPT_NONE;
							abortStreaming = true;
							break;
						}
						long waitTimeUS = dtUS(waitStart, sclock::now());
						long waitFrames = waitTimeUS*state.camera.fps/1000000;
						if (waitFrames > (qpu_it < 10? 100 : 10) && (!sentTimeoutError || dtMS(errorSent, sclock::now()) > 1000))
						{ // Unsuccessfully waited a few frames (or more if just after streaming start) for next frame
							printf("== %.2fms: QPU: Waited for %ldus / %d frames - way over expected frame interval! ==\n",
								dtMS(time_start, sclock::now()), waitTimeUS, (int)(waitTimeUS * state.camera.fps / 1000000));
							errorSent = sclock::now();
							sentTimeoutError = true;
							error = ERROR_GCS_TIMEOUT;
							handleError(false);
							if (waitFrames > 100)
							{
								abortStreaming = true;
								break;
							}
						}
					}
					return abortStreaming;
				};
				if (waitForFrame())
					break;

				if (frameReady.try_lock())
				{ // Return previous frame if it wasn't claimed already
					std::shared_ptr<FrameBuffer> pastFrame = std::move(frameExchange);
#ifdef LOG_DROPS
					if (pastFrame)
						printf("%.2fms: CPU FRAME DROP ID %d\n", dtMS(time_start, sclock::now()), pastFrame->ID);
#endif
					cumulFramePassedCPU++;
				}
				else
				{ // CPU accepted last frame, reset state
					cumulFrameNoTrigger = 0;
					cumulFramePassedQPU = 0;
					cumulFramePassedCPU = 0;
					bitmskSwitch = (bitmskSwitch+1)%bitmskCount;
				}

				TimePoint_t t1 = sclock::now();

				// Get most recent frame
				unsigned int advanceFrames = 0;
				void *frameHeader = gcs_requestLatestFrameBuffer(gcs, &advanceFrames);
				if (abortStreaming)
					break;
				if (frameHeader == NULL || advanceFrames == 0)
				{
					printf("== GCS returned NULL frame with error flag %d! ==\n", gcs_readErrorFlag(gcs));
					error = ERROR_GCS_REQUEST;
					abortStreaming = true;
					break;
				}

				TimePoint_t time_cur = sclock::now();
#ifdef LOG_DROPS
				if (advanceFrames > 1)
					printf("%.2fms: QPU THREAD MISSED %d+ FRAMES AFTER ID %d\n", dtMS(time_start, time_cur), advanceFrames-1, frameID);
#endif

				float newInterval = dtMS(time_frameRecv, time_cur);
				float expInterval = 1000.0f/state.camera.fps;

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
						error = ERROR_GCS_RETURN;
						abortStreaming = true;
						break;
					}
					TimePoint_t waitStart = sclock::now();
					if (waitForFrame())
						break;
					frameHeader = gcs_requestFrameBuffer(gcs);
					if (frameHeader == NULL)
					{
						printf("== GCS returned NULL frame with error flag %d after skipping delayed! ==\n", gcs_readErrorFlag(gcs));
						error = ERROR_GCS_REQUEST;
						abortStreaming = true;
						break;
					}
					// Update values
					advanceFrames++;
					time_cur = sclock::now();
#ifdef LOG_DROPS
					printf("%.2fms: QPU: SKIPPED %.2fms TO CURRENT FRAME, new interval %ldus\n",
						dtMS(time_start, time_cur), dtMS(waitStart, time_cur), dtUS(time_SOF, time_cur));
					printf("%.2fms: QPU: Reason: time_diff %.2fms, time_delay %.2fms, advance frames %d, avg processing %.2fms, max delay %.2fms\n",
						dtMS(time_start, time_cur), newInterval, timeDelay, advanceFrames, avgTimes.processing/1000.0f, maxDelay);
#endif
					newInterval = dtMS(time_frameRecv, time_cur);
				} */

				// Update frameID with advanced frames
				frameID += advanceFrames;
				cumulFramePassedQPU += advanceFrames-1;

				if (state.uart.started && state.camera.extTrig)
				{ // Match current frame with frame SOFs from PACKET_SOF
					// Incase frameID is non-continuous, frameID will hopefully be adopted properly anyway, but cumulFrameNoTrigger will be inaccurate

					std::unique_lock lock(state.sync.access);
					
					// 7000us at least from trigger signal to a frame being received in the camera
					// Usually 7150us, with early timesync 7080us - just to be safe from timesync wonkyness, lower
					const unsigned int MIN_TRIGGER_TO_FRAME = 6800;

					// Assume that any camera frame we received is valid (no false triggers)
					// With that, assume any SOFs with ID < predID to have been skipped
					// Do NOT assume we receive a SOF&Trigger for every frameID
					// TODO: Incase we got a trigger, but SOF was missed (e.g. UART error):
					//  - Cannot reliably know which frameID it was supposed to be when frameIDs are not continuous
					//  - Throw frames away or assume continous anyway?
					// With that, search for the next best frame fitting our frame timing model
					// Then update our frame timing model with delay from SOF packet to receiving camera frame
					// TODO: Improve. Still yields wrong result. Make use of continuity of frames?
					// Sometimes, large amounts of SOF packets are dropped (at least for pi, perhaps not for mcu)
					bool logCase = false;
					while (!state.sync.frameSOFs.empty())
					{
						auto SOF = state.sync.frameSOFs.front();
						if (SOF.first < frameID)
						{ // Skipped this frame
#ifdef LOG_TIMING
							if (state.writeStatLogs)
							{
								if (!logCase) printf("Assuming frame ID to be %d after skipping %d frames\n", frameID, advanceFrames-1);
								printf("  Skipped frame ID %d from %ldus ago\n", SOF.first, dtUS(SOF.second, time_cur));
								logCase = true;
							}
#endif
							state.sync.frameSOFs.pop();
							if (state.sync.frameSOFs.empty())
							{
#ifdef LOG_TIMING
								if (logCase)
									printf("  Ended up with initial frame ID %d!\n", frameID);
#endif
								break;
							}
							continue;
						}

						long frameTimeUS = dtUS(SOF.second, time_cur);
						if (frameTimeUS <= MIN_TRIGGER_TO_FRAME)
						{
#ifdef LOG_TIMING
							if (state.writeStatLogs || SOF.first != frameID)
							{
								if (!logCase) printf("Assuming frame ID to be %d after skipping %d frames\n", frameID, advanceFrames-1);
								if (SOF.first == frameID)
									printf("  But SOF for frame ID %d was only %ldus ago - SOF time prediction wrong? Earlier frameID failure? \n",
										SOF.first, frameTimeUS);
								else
									printf("  Next SOF for frame ID %d was only %ldus ago - missed a SOF packet for frame ID %d?\n",
										SOF.first, frameTimeUS, frameID);
								printf("  Ended up with initial frame ID %d!\n", frameID);
								logCase = false;
							}
#endif
							// Just trust our own prediction for now
							// But frameID should be considered uncertain now
							break;
						}
						// Possibly this or future SOF
						state.sync.frameSOFs.pop(); // Consume already

						// Check delay model of trigger to receiving the frame through V4L2
						auto &delayModel = state.sync.SOF2RecvDelay; // Fine without initialisation
						if (!state.sync.frameSOFs.empty() && frameTimeUS > delayModel.avg + delayModel.stdDev()*2)
						{ // Check next frame if it could be our SOF instead
							auto nextSOF = state.sync.frameSOFs.front();
							long nextFrameTimeUS = dtUS(nextSOF.second, time_cur);
							if (nextFrameTimeUS > MIN_TRIGGER_TO_FRAME &&
								nextFrameTimeUS > delayModel.avg - delayModel.stdDev())
							{ // Consider next SOF as reference instead, assume we missed previous SOF (trigger missed)
								bool bothOld = nextFrameTimeUS > delayModel.avg + delayModel.stdDev()*2;
#ifdef LOG_TIMING
								if (state.writeStatLogs || bothOld)
								{
									if (!logCase) printf("Assuming frame ID to be %d after skipping %d frames\n", frameID, advanceFrames-1);
									if (!bothOld)
										printf("  But SOF for frame ID %d was %ldus ago, next SOF for frame ID %d was only %ldus ago - received no trigger for frame ID %d?\n",
											SOF.first, frameTimeUS, nextSOF.first, nextFrameTimeUS, frameID);
									else // Both are old, something probably went wrong
										printf("  SOF for frame ID %d was %ldus ago, for frame ID %d %ldus ago - received no trigger for either?\n",
											SOF.first, frameTimeUS, nextSOF.first, nextFrameTimeUS);
									logCase = true;
								}
#endif
								cumulFrameNoTrigger += nextSOF.first-frameID;
								advanceFrames += nextSOF.first-frameID;
								frameID = nextSOF.first;
								continue; // Don't immediately take it, might not be the best one either
							}
#ifdef LOG_TIMING
							else if (logCase)
							{
								printf("  Decided against next frame ID %d from %ldus ago, took frame ID %d from %ldus ago \n",
									nextSOF.first, nextFrameTimeUS, SOF.first, frameTimeUS);
								logCase = false;
							}
#endif
						}
#ifdef LOG_TIMING
						else if (logCase)
						{
							printf("  Taking frame ID %d from %ldus ago as the best SOF! Got %d newer SOFs waiting, maximum SOF to receive delay is %d+%dus\n",
								SOF.first, frameTimeUS, (int)state.sync.frameSOFs.size(), (int)delayModel.avg, (int)delayModel.stdDev()*2);
							logCase = false;
						}
#endif
						// Take this SOF as reference
						cumulFrameNoTrigger += SOF.first-frameID;
						advanceFrames += SOF.first-frameID;
						frameID = SOF.first;
						delayModel.update(frameTimeUS);
						break;
					}
					assert(!logCase); // Final log should've been written
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
					error = ERROR_MEM_ACCESS;
					abortStreaming = true;
					break;
				}

				uint8_t *framePtrARM = (uint8_t*)frameBuffer.mem;
				uint32_t framePtrVC = frameBuffer.VCMem; 

#else
				// Somewhat emulate framerate
				usleep(std::max(0,(int)(1.0f/state.camera.fps*1000*1000)-4000));

				TimePoint_t t1 = sclock::now();
				TimePoint_t t2 = sclock::now();

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
					error = ERROR_MEM_ACCESS;
					abortStreaming = true;
					break;
				}

				uint8_t *framePtrARM = (uint8_t*)frameBuffer.mem;
				uint32_t framePtrVC = frameBuffer.VCMem; 
		#else
				// Lock QPU buffer (needed?)
				qpu_lockBuffer(&frameBuffer);
				uint8_t *framePtrARM = frameBuffer.ptr.arm.cptr;
				uint32_t framePtrVC = frameBuffer.ptr.vc;
		#endif
#endif

				TimePoint_t t3 = sclock::now();

				// ---- Uniform preparation ----

				// Set source buffer pointer in progmem uniforms
				QPU_BUFFER &bitmskBuf = bitmskBuffer[bitmskSwitch];
				qpu_lockBuffer(&blobProgram.progmem_buffer);
				SetupProgramBuffers(layout, blobProgram.progmem.uniforms.arm.uptr, srcStride, framePtrVC, bitmskBuf.ptr.vc);
				qpu_unlockBuffer(&blobProgram.progmem_buffer);

				// ---- Program execution ----

				// Lock bitmask buffer
				//qpu_lockBuffer(&bitmskBuf);

				// Execute layout.instances programs each with their own set of uniforms
				int code = qpu_executeProgramDirect(&blobProgram, &base, layout.instances, numUnif, numUnif, &perfState);

				// Unlock bitmask buffer
				//qpu_unlockBuffer(&bitmskBuf);

#ifdef LOG_QPU	// Only relevant during development of QPU programs
				qpu_logErrors(&base);
#endif

				// Process frame to bitmask
				if (code != 0)
				{
					printf("== %.2fms: QPU: Failed to process frame! ==\n", dtMS(time_start, sclock::now()));
					error = ERROR_QPU_STALL_MSK;
					abortStreaming = true;
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
				qpu_unlockBuffer(&camEmulBuf[qpu_it%emulBufCnt]);
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
			static_cast<void>(frameReady.try_lock());
			frameReady.unlock();
		});


		// ---- Processing Loop ----

		// Increase clock rate after boot (likely has to be done through linux CPU governor, not directly)
//		setClockRate(base.mb, 3, 1000000000); // 1000Mhz fixed
//		printf("Set ARM Core clock rate to fixed %dMhz!\n", getClockRate(base.mb, 3)/1000000);

		nice(-15); // High priority

		TimePoint_t time_lastStatCheck = sclock::now();
		time_start = sclock::now();
		int time_debug_interval = 100;
		int frameNum_lastStatCheck = 0;

		while (state.curMode.streaming)
		{
			numFrames++;

			if (needsComms && !comm_anyReady(comms))
			{
				printf("Lost comms! Stopping streaming!\n");
				state.curMode.streaming = false;
				state.curMode.mode = TRCAM_STANDBY;
				state.curMode.opt = TRCAM_OPT_NONE;
				break;
			}

			if (isConsole && numFrames % 10 == 0)
			{ // Check input
				char cin;
				if (read(STDIN_FILENO, &cin, 1) == 1)
				{
					if (cin == 'q')
					{ // Complete stop of program requested
						running = false;
						break;
					}
					else if (cin == 'r')
					{ // Reset/Restart, stop current blob detection and start again
						break;
					}
					else if (cin == 'e')
					{
						mcu_reset();
					}
				}
			}

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
						initBackgroundCalibration();
					}
					else if (newMode.opt == TRCAM_OPT_BGCALIB_ACCEPT)
					{ // Accept temp mask as current mask
						if (state.curMode.mode == TRCAM_MODE_BGCALIB)
						{
							printf("Accepting Background Calibration!\n");
							acceptBackgroundCalibration();
						}
					}
					else if (newMode.opt == TRCAM_OPT_BGCALIB_RESET)
					{ // Reset current mask (and temp mask)
						printf("Resetting stored Background Calibration!\n");
						resetBackgroundCalibration();
					}
				}
				state.curMode = newMode;
				printf("New mode: streaming: %d, mode: %d, opt: %d\n", state.curMode.streaming, state.curMode.mode, state.curMode.opt);

				// Notify host of mode change
				static_assert(TRCAM_MODE_SIZE == std::numeric_limits<uint8_t>::max());
				uint8_t mode = (uint8_t)state.newModeRaw;
				void *packet = comm_packet(comms, PacketHeader(PACKET_MODE, 1));
				comm_write(comms, packet, &mode, 1);
				comm_submit(comms, packet);

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


			/* if (state.uart.ready)
			{ // Verify the large packet interleaving works by checking the TX queue size
				// Mostly interested in UART for now

				int txQueue = uart_getTXQueue(state.uart.port);
				if (txQueue > 100)
				{
					printf("Failed to constrain large packet blocks to idle times, have %d bytes in TX buffer before sending SOF!\n", txQueue);
				}
			} */
			
			// Announce to controller that frame is being processed
			void *packet = comm_packet(comms, PacketHeader(PACKET_FRAME_SIGNAL, 0, curFrame->ID&0xFF));
			comm_submit(comms, packet);

			// Allow sending from now until processing is expected to end, at which point we need to send the results with low latency
			bytesSentAccum += sendLargePacketData(state, avgTimes.cpu-200);

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
				performBlobDetectionRegionsFetch(curFrame->bitmsk->ptr.arm.uptr);

				// Unlock bitmask buffer
				//qpu_unlockBuffer(curFrame->bitmsk);

				TimePoint_t t1 = sclock::now();

				// Perform Connected Component Labelling on regions
				clusters.clear();
				performBlobDetectionCPU(clusters);

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
			bytesSentAccum = 0; // Only care for the bytes sent in this section

			if (state.uart.ready)
			{ // Verify the large packet interleaving works by checking the TX queue size
				// Mostly interested in UART for now

				int txQueue = uart_getTXQueue(state.uart.port);
				/* if (txQueue > 0)
				{
					printf("Failed to constrain large packet blocks to idle times, have %d bytes in TX buffer before reporting!\n", txQueue);
				} */
				bytesSentAccum += txQueue;
			}

			if (state.curMode.mode == TRCAM_MODE_BGCALIB)
			{
				std::vector<uint8_t> bgTiles = updateBackgroundCalibration();
				if (bgTiles.size() > 0)
				{
					printf("Added %d new tiles to the background mask!\n", bgTiles.size());
					if (comm_anyReady(comms))
					{ // Create mask report
						TimePoint_t t0 = sclock::now();

						void *packet = comm_packet(comms, PacketHeader(PACKET_BGTILES, bgTiles.size()+2));
						Vector2<uint8_t> extends = (layout.validMaskRect.extends()/8).cast<uint8_t>();
						comm_write(comms, packet, extends.data(), 2);
						comm_write(comms, packet, (uint8_t*)bgTiles.data(), bgTiles.size());
						comm_submit(comms, packet);
						//comm_flush(comms);
						bytesSentAccum += 2+bgTiles.size();

						curTimes.send += dtUS(t0, sclock::now());
					}
				}
			}

			if (comm_anyReady(comms))
			{ // Create blob report
				TimePoint_t t0 = sclock::now();

				int reportLength = STREAM_PACKET_HEADER_SIZE + STREAM_PACKET_BLOB_SIZE * clusters.size();
				void *packet = comm_packet(comms, PacketHeader(PACKET_BLOB, reportLength, curFrame->ID&0xFF));

				// Send blob individually
				/* uint16_t blobData[3]; // uint32_t blobData;
				for (int i = 0; i < blobs.size(); i++)
				{
					Cluster *blob = &blobs[i];
					int blobX = (int)(((double)blob->centroid.x()+0.5) * 65536 / state.camera.width);
					int blobY = (int)(((double)blob->centroid.y()+0.5) * 65536 / state.camera.height);
					int blobS = std::min((int)(blob->centroid.S * 2), 255);
					blobData[0] = blobX;
					blobData[1] = blobY;
					blobData[2] = (blobS & 0xFF) | ((blobs[i].quality & 0xFF) << 8);
					// Alternative 32bit
					//int blobX = (int)((double)blob->centroid.x() * 16383 / state.camera.width);
					//int blobY = (int)((double)blob->centroid.y() * 16383 / state.camera.height);
					//int blobS = std::min((int)(blob->centroid.S * 2), 255);
					//blobData = (blobX << 0) | (blobY << 14)
					//		|  << 22
					//		| (blobColor & ((1<<4)-1)) << 28;
					// Write
					comm_write(comms, (uint8_t*)blobData, sizeof(blobData));
				} */

				// Individualy comm_write sadly add an idle pause that splits up packets on the controller's end.
				// Thus it is better to send one big comm_write (ignoring the comm_submit checksum bit, doesn't appear to be split ususally)

				// Fill in blob data
				packetBuffer.resize(reportLength);
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

				// Send blob report
				comm_write(comms, packet, packetBuffer.data(), packetBuffer.size());

				// Submit
				comm_submit(comms, packet);
				//comm_flush(comms);

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
					int size = (blob.bounds.maxX-blob.bounds.minX)*(blob.bounds.maxY-blob.bounds.minY);
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
					//refineBlob(blob, state.refinementMethod, curFrame->memory);
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
					void *packet = comm_packet(comms, PacketHeader(PACKET_VISUAL, packetLength, curFrame->ID&0xFF));

					// Write metadata
					uint16_t *metaData = (uint16_t*)packetBuffer.data();
					metaData[0] = bounds.minX;
					metaData[1] = bounds.minY;
					metaData[2] = bounds.maxX;
					metaData[3] = bounds.maxY;
					metaData[4] = (uint16_t)(((double)blob.centroid.x()+0.5) * 65536 / state.camera.width);
					metaData[5] = (uint16_t)(((double)blob.centroid.y()+0.5) * 65536 / state.camera.height);
					metaData[6] = (uint16_t)((double)blob.size * 65536 / 256);
					metaData[7] = (edgeRefined.size() & 0xFFFF);
					comm_write(comms, packet, (uint8_t*)metaData, metaSize);

					// Read out image data from bounds
					for (int y = 0; y < extends.y(); y++)
					{
						int imgPos = y*extends.x();
						int ptrPos = (bounds.minY+y)*srcStride + bounds.minX;
						comm_write(comms, packet, curFrame->memory + ptrPos, extends.x());
					}
					
					// Write bit buf
					uint8_t *bitBuf = packetBuffer.data();
					memset(bitBuf, 0, bitSize);
					for (int i = 0; i < blob.dots.size(); i++)
					{
						Vector2<uint16_t> pt = blob.dots[i] - bounds.min();
						int pos = pt.y()*extends.x() + pt.x();
						bitBuf[pos/8] |= 1 << (pos % 8);
					}
					comm_write(comms, packet, bitBuf, bitSize);
					
					// Write points
					uint8_t *ptBuf = packetBuffer.data();
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
					comm_write(comms, packet, ptBuf, ptSize);

					//printf("Selected blob %d of size %d = %dx%d (%d, %d, %d, %d) with %d points, size %f, as debug target! Total report length is %d!\n",
					//	maxInd, maxSz, extends.x(), extends.y(), blob.bounds.minX, blob.bounds.minY, blob.bounds.maxX, blob.bounds.maxY, blob.ptCnt, blob.centroid.S, blobPayloadLength);

					// Send blob report
					comm_submit(comms, packet);
					//comm_flush(comms);

					bytesSentAccum += metaSize + imgSize + bitSize + ptSize;
					curTimes.send += dtUS(t0, sclock::now());
				}
				else 
				{ // Write empty packet
					TimePoint_t t0 = sclock::now();
					void *packet = comm_packet(comms, PacketHeader(PACKET_VISUAL, 0, curFrame->ID&0xFF));
					comm_submit(comms, packet);
					//comm_flush(comms);
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
						nice(20); // Low priority
						TimePoint_t t0 = sclock::now();

						vis_visualise(state.visualisation, curBlobs, pastBlobs, frame->memory, state.camera.width, state.camera.height, srcStride);
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
			bytesSentAccum += sendLargePacketData(state, avgInterval*1000 - dtUS(time_cpu_begin, sclock::now()) - 500);
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
					qpu_updatePerformance(&base, &perfState);
				}
#endif
			}
		

			if (numFrames % time_debug_interval == 0)
			{ // Frames per second
				TimePoint_t currentTime = sclock::now();
				uint32_t deltaUS = dtUS(time_lastStatCheck, currentTime);
				uint32_t temperature = getTemperature(base.mb);

				if (comm_anyReady(comms))
				{
					StatPacket stats;
					stats.header.frame = numFrames;
					stats.header.deltaUS = deltaUS;
					stats.header.tempSOC = temperature/10; // Not accurate to a thousands anyway, more a tenth
					stats.header.skipTrigger = std::min(255, skip_count_trigger);
					stats.header.skipCPU = std::min(255, skip_count_cpu);
					stats.header.skipQPU = std::min(255, skip_count_qpu);
					stats.times = avgTimes;
					stats.incidents = incidents;
					// Write byte representation
					uint8_t statPacket[STAT_PACKET_SIZE];
					storeStatPacket(stats, statPacket);
					// Send packet
					void *packet = comm_packet(comms, PacketHeader(PACKET_STAT, sizeof(statPacket)));
					comm_write(comms, packet, statPacket, sizeof(statPacket));
					comm_submit(comms, packet);
				}

				if (state.writeStatLogs)
				{
					float elapsedS = deltaUS/1000000.0f;
					float fps = time_debug_interval / elapsedS;

#ifdef LOG_SHORT
					printf("%d: %.1ffps, %.1f°C, ", numFrames, fps, temperature/1000.0f);
					printDeltaTimes(avgTimes);
					if (state.visualisation.enabled)
						printf(", Vis: (%dus, %.0fHz)", incidents.vis.avg, incidents.vis.occurences/elapsedS);
					if (state.streaming.enabled)
						printf(", Stream: (%.2fms, %.0fHz)", (incidents.stream.avg<<8)/1000.0f, incidents.stream.occurences/elapsedS);
					printf("\n");
#else
					//printf("ARM Core clock rate is at %dMhz!\n", getClockRate(base.mb, 3)/1000000);
					printf("%d frames over %.2fs (%.1ffps), Temp %.1f°C, Latency %.2fms, Processing %.2fms; QPU: %.2fms, CPU: %.2fms (Fetch: %.2fms, CCL: %.2fms, Post: %.2fms, Send: %.2fms)!\n",
						time_debug_interval, elapsedS, fps, getTemperature(base.mb)/1000.0f, avgTimes.latency, avgTimes.processing, avgTimes.qpu, avgTimes.cpu, avgTimes.fetch, ccl, avgTimes.post, avgTimes.send);
#endif
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
					qpu_logPerformance(&perfState);
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
			{
				state.wireless.sendStatus = false;
				state.wireless.lastStatus = sclock::now();
				sendWirelessStatusPacket(state);
			}

			bytesSentAccum = 0;
		}

		// Handle error now, cleanup is uncertain
		bool abort = error != ERROR_NONE && handleError();

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

		cleanup_qpu_enable();
		cleanup_qpu_resources();
		cleanup_blob_detect();
		cleanup_camera();

		LeaveStreamingState();

		nice(0); // Normal priority

		if (abort)
			break;
		error = ERROR_NONE;

	} // while (running)

	printf("------------------------\n");

	return error != ERROR_NONE? error : EXIT_SUCCESS;
}

static bool prepareImageStreamingPacket(const FrameBuffer &frame, ImageStreamState stream)
{
	TimePoint_t t0 = sclock::now();

	// TODO: Should really use hardware JPEG encoder here, and not waste away CPU cycles
	// But not super easy without MMAL - does V4L2 support that? Need to research

	// Extract image data that we want (and subsample if necessary)
	int encX, encY, encStride;
	Bounds2<int> encBounds;
	uint8_t* encData = sampleImageBounds(frame.memory, state.camera.width, state.camera.height, frame.stride,
		stream.bounds, stream.subsampling, encX, encY, encStride, encBounds);
	if (!encData)
	{
		if (state.writeStatLogs)
			printf("Failed to subsample image!\n");
		return false;
	}

	TimePoint_t t1 = sclock::now();

	std::vector<uint8_t> jpegData;
	bool success = compress(jpegData, encData, encX, encY, encStride, stream.jpegQuality);

	if (!success)
	{
		if (state.writeStatLogs)
			printf("Failed to compress image!\n");
		return false;
	}

	TimePoint_t t2 = sclock::now();

	/* if (state.writeStatLogs)
	{
		printf("Encoded image %dx%d, %dKB, to %dKB with quality %d%%!\n",
			dH, dV, (dH*dV/1024), (int)jpegData.size()/1024, stream.quality);
	} */
	
	// Wait for last frame to be sent
	int it = 0;
	while (largePacket.sending || it > 20)
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

	if (largePacket.sending)
	{ // Still sending data of last frame, probably an error, so discard it
		if (state.writeStatLogs)
			printf("Failed to send previous frame!\n");
		largePacket.sending = false;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	
	// Write header
	*(uint32_t*)&largePacket.header[0] = jpegData.size();
	//*(uint32_t*)&frameImage.header[4] = blockOffset;
	//*(uint32_t*)&frameImage.header[8] = blockSize;
	*(uint8_t*)&largePacket.header[12] = stream.jpegQuality;
	*(uint8_t*)&largePacket.header[13] = stream.subsampling;
	*(uint16_t*)&largePacket.header[14] = encBounds.minX;
	*(uint16_t*)&largePacket.header[16] = encBounds.minY;
	*(uint16_t*)&largePacket.header[18] = encBounds.maxX;
	*(uint16_t*)&largePacket.header[20] = encBounds.maxY;
	// Block data in header is filled in as data is sent	

	/* if (state.server.ready && state.uart.ready)
	{ // Prefer to send larger packets over server to not disrupt low latency communications
		*(uint32_t*)&frameImage.header[4] = 0;
		*(uint32_t*)&frameImage.header[8] = frameImage.image.size();
		void *packet = comm_packet(state.server, PacketHeader(PACKET_IMAGE, sizeof(frameImage.header)+jpegData.size(), frame.ID));
		comm_write(state.server, packet, frameImage.header, sizeof(frameImage.header));
		comm_write(state.server, packet, jpegData.data(), jpegData.size());
		comm_submit(state.server, packet);
	}
	else */
	{ // Tell main thread to interleave sending large data in between frames
		largePacket.data.swap(jpegData);
		largePacket.sendProgress = 0;
		largePacket.sending = true;
		largePacket.ID = frame.ID;
		largePacket.tag = PACKET_IMAGE;
	}

	TimePoint_t t3 = sclock::now();
	/* if (state.writeStatLogs)
	{
		int subsampleUS = dtUS(t0, t1), encodeUS = dtUS(t1, t2), totalUS = dtUS(t0, t3);
		printf("Took %.2fms to subsample and %.2fms to encode image frame - in total %.2fms of CPU time!\n",
			subsampleUS/1000.0f, encodeUS/1000.0f, totalUS/1000.0f);
	} */

	return true;
}

static void sendWirelessStatusPacket(TrackingCameraState &state)
{
	if (!comm_anyReady(comms))
		return;
	std::vector<uint8_t> statusPacket;
	fillWirelessStatusPacket(state, statusPacket);
	void *packet = comm_packet(comms, PacketHeader(PACKET_WIRELESS, statusPacket.size()));
	comm_write(comms, packet, statusPacket.data(), statusPacket.size());
	comm_submit(comms, packet);
}

static int sendLargePacketData(TrackingCameraState &state, int commTimeUS)
{
	if (!largePacket.sending) return 0;
	// Interleave packet with low-latency comm to not disrupt it

	// Only one comm should be active, else it would have already been sent directly to server
	auto &comm = state.uart.ready? state.uart : state.server;

	// Calculate rough budget left to send bytes
	int budget;
	if (&comm == &state.uart)
	{ // Account for bytes still in TX queue
		budget = (int)(uart_getBitsPerUS(state.uart.port) * commTimeUS);
		budget -= uart_getTXQueue(state.uart.port);
	}
	else
	{
		budget = commTimeUS;
		printf("Server-only mode not supported!\n");
		largePacket.sending = false;
		return 0;
	}
	if (budget < 200)
		return 0;

	// Calculate how many bytes to send efficiently
	int numUSBPackets = std::max(1, (int)std::floor(budget / OPT_PACKET_SIZE));
	int numBytes = std::min(budget, numUSBPackets * OPT_PACKET_SIZE) - sizeof(largePacket.header);
	int bytesLeft = largePacket.data.size() - largePacket.sendProgress;
	if (numBytes >= bytesLeft) numBytes = bytesLeft;
	else if (bytesLeft-numBytes < 100) numBytes = bytesLeft;

	// Send as many bytes
	assert(largePacket.sendProgress < largePacket.data.size());
	*(uint32_t*)&largePacket.header[4] = largePacket.sendProgress;
	*(uint32_t*)&largePacket.header[8] = numBytes;
	void *packet = comm_packet(comm, PacketHeader(largePacket.tag, sizeof(largePacket.header)+numBytes, largePacket.ID));
	comm_write(comm, packet, largePacket.header, sizeof(largePacket.header));
	comm_write(comm, packet, largePacket.data.data()+largePacket.sendProgress, numBytes);
	comm_submit(comm, packet);
	largePacket.sendProgress += numBytes;
	if (largePacket.sendProgress == largePacket.data.size())
		largePacket.sending = false;
	return numBytes;
};

/* Sets console to raw mode which among others allows for non-blocking input, even over SSH */
static void setConsoleRawMode()
{
	tcgetattr(STDIN_FILENO, &terminalSettings);
	struct termios termSet = terminalSettings;
	atexit([]{ // Reset at exit
		tcsetattr(STDIN_FILENO, TCSANOW, &terminalSettings);
	});
	termSet.c_lflag &= ~(ECHO | ICANON);
	termSet.c_cc[VMIN] = 0;
	termSet.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &termSet);
}