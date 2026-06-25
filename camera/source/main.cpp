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
#include <chrono>
#include <thread>
#include <atomic>
#include <string>
#include <math.h>
#include <sched.h>
#include <execinfo.h>
#include <signal.h>

// Console and UART
#include <termios.h>
// Native threads
#include <sys/prctl.h>

#include "state.hpp"
#include "processing/processing.hpp"

#include "mcu/mcu.hpp"

#include "comm/comm.hpp"
#include "comm/uart.hpp"
#include "comm/server.hpp"
#include "comm/wireless.hpp"
#include "util/image.hpp"
#include "util/util.hpp"

#include "version.hpp"

#include "vcsm/vcsm.hpp"
#include "videocore/mailbox.h"

// Terminal Output and Input
bool isConsole;
struct termios terminalSettings;
static void setConsoleRawMode(bool certainTTY);
static char getConsoleChar();


// Static initialised members (for atexit)
TrackingCameraState state = {};
static VC_BASE base;
GCS *gcs = NULL;
ErrorTag errorCode = ERROR_NONE;
CommState uartComms = {};
CommState serverComms = {};

bool running = true; // Only for interactive operation

/* Functions */

bool handleError(ErrorTag error, bool serious, const char* reportBuf, int reportLen)
{
	if (error < ERROR_MAX)
		fprintf(stderr, "Encountered Error %d: %s!\n", (int)error, ErrorTag_String[(int)error]);
	else
		fprintf(stderr, "Encountered Invalid Error %d!\n", (int)error);
	if (serious)
		errorCode = error;
	for (int c = 0; c < COMM_MEDIUM_MAX; c++)
	{
		CommState* comm = comms.medium[c];
		if (!comm || !comm->ready) continue;
		if (serious && !comm_interject(comm))
		{ // Have to assume we can't wait for other threads to finish sending, so interject, if we can't give up
			printf("    Failed to interject comms %d in error handling!\n", comm->medium);
			continue;
		}
		if (comm_packet(comm, PacketHeader(PACKET_ERROR, 2+reportLen)))
		{ // Send notification of error over this comm medium
			comm_write(comm, (uint8_t*)&error, 1);
			comm_write(comm, (uint8_t*)&serious, 1);
			if (reportBuf && reportLen > 0)
				comm_write(comm, (uint8_t*)reportBuf, reportLen);
			comm_submit(comm);
			printf("    Sent error packet via comm medium %d!\n", comm->medium);
		}
		else
			printf("    Failed to send error packet via comm medium %d!\n", comm->medium);
		if (serious)
		{ // If we're going to restart the program, notify comm partner
			comm_force_close(comm);
		}
	}
	return serious;
}

bool EnableCrashHandler();

void CrashHandler(int signal, const char* reportBuf, int reportLen = 0)
{
	ErrorTag error;
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
		case SIGABRT:
			error = ERROR_EXCEPTION_ABORT;
			break;
		default:
			error = ERROR_EXCEPTION_UNKNOWN;
			break;
	}
	handleError(error, true, reportBuf, reportLen);
}

int main(int argc, char **argv)
{

	// ---- Init Application ----

	if (geteuid() != 0)
	{
		printf("Require superuser privileges!\n");
		return 0;
	}

	//srand(1111110);
	srand(time(0)); // Sufficient to get new random values every time

	prctl(PR_SET_NAME, "Thread_CPU");

	EnableCrashHandler();

	// Read and validate arguments
	if (!options_read(state, argc, argv))
		return -1;

	// Modify tty to allow for non-blocking input (for console & ssh, but not background execution)
	isConsole = true; // Unreliable: isatty(STDIN_FILENO);
	if (isConsole)
		setConsoleRawMode(isatty(STDIN_FILENO));

	srand((unsigned int)time(NULL));

	// Gather information about the firmware and hardware on this camera
	gatherInfo(state.id);

	if (!state.noMCU)
	{ // Init, detect, recover, connect with, and monitor MCU
		mcu_initial_connect(state.probeMode);
		atexit(mcu_cleanup);
	}

	// Init VCSM
	if (!vcsm_init())
	{
		printf("Failed to init VCSM!\n");
		return -1;
	}
	atexit(vcsm_exit);

	// Init VideoCore Base (basic information to work with the VideoCore IV), e.g. VCHIQ mailbox
	if (int errCode = vc_initBase(&base) != 0)
	{ // More a dev error, should not happen on a deployed system
		printf("Failed to init QPU Base! %d\n", errCode);
		return -1;
	}
	atexit([]{ // Close at exit
		vc_destroyBase(&base);
	});

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

	// UART communications
	{ // Init whether used or not, might be enabled later on
		uartComms.protocol.needsErrorScanning = true;
		uartComms.start = uart_start;
		uartComms.stop = uart_stop;
		uartComms.wait = uart_wait;
		uartComms.configure = uart_configure;
		uartComms.read = uart_read;
		uartComms.write = uart_write;
		uartComms.submit = uart_submit;
		uartComms.flush = uart_flush;
		uartComms.ownIdent = IdentPacket(cameraID, DEVICE_TRCAM, INTERFACE_UART, sbcFWVersion);
		uartComms.expIdent.device = DEVICE_TRCONT;
		uartComms.started = false;
	}
	atexit([]{ // Close at exit
		comm_disable(uartComms);
		uart_deinit(&uartComms.port);
	});
	if (state.enableUART)
	{
		uartComms.port = uart_init(state.serialName);
		comm_enable(uartComms, &state, COMM_MEDIUM_UART);
		printf("Initiating UART connection...\n");
	}

	// Server communications
	{ // Init whether used or not, might be enabled later on
		serverComms.protocol.needsErrorScanning = false;
		serverComms.start = server_start;
		serverComms.stop = server_stop;
		serverComms.wait = server_wait;
		serverComms.configure = nullptr;
		serverComms.read = server_read;
		serverComms.write = server_write;
		serverComms.submit = server_submit;
		serverComms.flush = server_flush;
		serverComms.ownIdent = IdentPacket(cameraID, DEVICE_TRCAM, INTERFACE_SERVER, sbcFWVersion);
		serverComms.expIdent.device = DEVICE_SERVER;
		serverComms.started = false;
	}
	atexit([]{ // Close at exit
		comm_disable(serverComms);
		server_deinit(&serverComms.port);
	});
	// Server will be started by initWirelessMonitor if wifi is enabled, either due to configuration or requested by console


	// ---- Wireless ----

	initWirelessMonitor(state);
	atexit([]{ // Close at exit
		stopWirelessMonitor(state);
	});


	// ---- Start Loop ----

	while (running)
	{

		// ---- Initialise based on communication ---

		if (state.noComms)
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

		TimePoint_t time_lastStatCheck = sclock::now();
		while (!state.noComms && !state.curMode.streaming)
		{ // Wait for a connected communications channel to instruct a stream start
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
			if (isConsole)
			{ // Check input
				char cin = getConsoleChar();
				if (cin == 'q')
				{ // Complete stop of program requested
					return EXIT_SUCCESS;
				}
				if (cin && !state.noMCU)
				{
					switch (cin)
					{
					case 'e':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_reconnect();
						break;
					}
					case 'b':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_switch_bootloader();
						break;
					}
					case 'f':
					{
						std::unique_lock lock(mcu_mutex);
						if (!mcu_probe_bootloader())
							printf("MCU is not in bootloader!\n");
						else
						 	mcu_flash_program(mcu_firmware_path);
						break;
					}
					case 'v':
					{
						std::unique_lock lock(mcu_mutex);
						if (!mcu_probe_bootloader())
							printf("MCU is not in bootloader!\n");
						else
						 	mcu_verify_program(mcu_firmware_path);
						break;
					}
					case 's':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_sync_info();
						break;
					}
					case 'd':
					{
						std::unique_lock lock(mcu_mutex);
						CameraStoredInfo info;
						CameraStoredConfig config;
						if (mcu_fetch_info(info, config))
							receivedInfoFromMCU(std::move(info));
						break;
					}
					case 'g':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_get_status();
						break;
					}
					case 'u':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_update_id(cameraID);
						break;
					}
					case 'i':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_update_id(35236462);
						break;
					}
					case 'x':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_disable();
						break;
					}
					case 'c':
					{
						std::unique_lock lock(mcu_mutex);
						mcu_check_disabled();
						break;
					}
					default:
						break;
					}
				}
			}

			if (state.firmware.applyingUpdate)
			{ // We're supposed to apply a firmware update
				ApplyFirmwareUpdate(state);
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
				comm_send(realTimeAff, PacketHeader(PACKET_MODE, 1), &mode);
			}

			if (state.wireless.sendStatus)
			{ // Notify host of wireless status
				sendWirelessStatusPacket(state);
			}

			long deltaStatUS = dtUS(time_lastStatCheck, sclock::now());
			if (deltaStatUS > 1000000)
			{ // Send a shortened, non-streaming stat packet every second
				time_lastStatCheck = sclock::now();
				StatPacket stats = prepareSystemStatusPacket(state, deltaStatUS);
				if (state.writeStatLogs)
					printf("%.1f°C, %.2fV\n", stats.header.tempSOC/100.0f, stats.header.voltage/1000.0f);
				// Write byte representation
				uint8_t statPacket[STAT_PACKET_HEADER];
				storeStatPacketHeader(stats, statPacket);
				// Send packet
				comm_send(largeDataAff, PacketHeader(PACKET_STAT, sizeof(statPacket)), statPacket);
			}
		}


		// ---- Processing Setup & Loop ----

		running = ProcessingStage(state, base);

		if (errorCode != ERROR_NONE)
			break;

	} // while (running)

	printf("------------------------\n");

	return errorCode != ERROR_NONE? errorCode : EXIT_SUCCESS;
}

bool getFramesyncActive()
{
	return uartComms.ready || mcu_active;
}

bool handleConsoleInputStreaming()
{
	if (!isConsole) return true;
	// Check input
	char cin = getConsoleChar();
	if (cin)
	{
		if (cin == 'q')
		{ // Complete stop of program requested
			running = false;
			return false;
		}
		else if (cin == 'r')
		{ // Reset/Restart, stop current blob detection and start again
			return false;
		}
		else if (cin == 'e' && !state.noMCU)
		{
			std::unique_lock lock(mcu_mutex);
			mcu_reconnect();
		}
		else if (cin == 'd' && !state.noMCU)
		{
			std::unique_lock lock(mcu_mutex);
			CameraStoredInfo info;
			CameraStoredConfig config;
			if (mcu_fetch_info(info, config))
				receivedInfoFromMCU(std::move(info));
		}
		else if (cin == 'g' && !state.noMCU)
		{
			std::unique_lock lock(mcu_mutex);
			mcu_get_status();
		}
	}
	return true;
}

bool prepareImageStreamingPacket(const FrameBuffer &frame, ImageStreamState stream)
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

	// Prepare large packet header
	std::vector<uint8_t> largePacketHeader(sizeof(LargePacketHeader)+10);
	LargePacketHeader *largePacket = (LargePacketHeader*)largePacketHeader.data();
	largePacket->totalSize = jpegData.size();
	largePacket->blockOffset = 0;
	largePacket->blockSize = jpegData.size();
	largePacket->data[0] = stream.jpegQuality;
	largePacket->data[1] = stream.subsampling;
	*(uint16_t*)&largePacket->data[2] = encBounds.min.x();
	*(uint16_t*)&largePacket->data[4] = encBounds.min.y();
	*(uint16_t*)&largePacket->data[6] = encBounds.max.x();
	*(uint16_t*)&largePacket->data[8] = encBounds.max.y();

	// Queue send
	comm_queue_send(comms.get(largeDataAff),
		PacketHeader(PACKET_IMAGE, 0, frame.ID&0xFF),
		std::move(jpegData), largePacketHeader);

	TimePoint_t t3 = sclock::now();
	/* if (state.writeStatLogs)
	{
		int subsampleUS = dtUS(t0, t1), encodeUS = dtUS(t1, t2), totalUS = dtUS(t0, t3);
		printf("Took %.2fms to subsample and %.2fms to encode image frame - in total %.2fms of CPU time!\n",
			subsampleUS/1000.0f, encodeUS/1000.0f, totalUS/1000.0f);
	} */

	return true;
}

void sendWirelessStatusPacket(TrackingCameraState &state)
{
	state.wireless.sendStatus = false;
	state.wireless.lastStatus = sclock::now();
	std::vector<uint8_t> statusPacket;
	fillWirelessStatusPacket(state, statusPacket);
	comm_send(realTimeAff, PacketHeader(PACKET_WIRELESS, statusPacket.size()), std::move(statusPacket));
}

StatPacket prepareSystemStatusPacket(TrackingCameraState &state, long deltaStatUS)
{
	StatPacket stats = {};
	stats.header.frame = 0;
	stats.header.deltaUS = deltaStatUS;
	stats.header.tempSOC = getTemperature(base.mb)/10; // Not accurate to a thousands anyway, more a tenth
	stats.header.voltage = floatingSupplyVoltageMV.load();
	return stats;
}

int getInterleavedQueuedBytes()
{
	if (uartComms.ready)
	{ // Verify the large packet interleaving works by checking the TX queue size
		// Mostly interested in UART for now

		int txQueue = uart_getTXQueue(uartComms.port);
		/* if (txQueue > 0)
		{
			printf("Failed to constrain large packet blocks to idle times, have %d bytes in TX buffer before reporting!\n", txQueue);
		} */
		return txQueue;
	}
	return 0;
}

int sendInterleavedQueuedPackets(int commTimeUS)
{
	// Interleave packet with low-latency comm to not disrupt it

	CommState* comm = comms.get(realTimeAff);
	if (!comm || comm->packetQueue.empty())
		return 0;

	// Calculate rough budget left to send bytes
	int budget = 0;
	if (comm == &uartComms)
	{ // Account for bytes still in TX queue
		budget = (int)(uart_getBytesPerUS(uartComms.port) * commTimeUS);
		budget -= uart_getTXQueue(uartComms.port);
	}
	else if (comm == &serverComms)
	{ // Not sure if this makes much sense for wireless
		budget = 15.0f/8 * commTimeUS; // Assume 15Mbit/s, 15-30 is realistic per device, but spectrum is shared
		budget -= server_getTXQueue(serverComms.port);
	}

	int sentTotal = 0;
	std::unique_lock lock(comm->queueMutex);
	while (budget > 10 && !comm->packetQueue.empty())
	{
		auto &packet = comm->packetQueue.front();
		if (!packet.largePacketHeader.empty())
		{ // Interleave in blocks
			// Calculate how many bytes to send efficiently
			int largeBudget = std::min(budget, PACKET_MAX_LENGTH);
			if (comm == &uartComms)
			{ // If there's a lot of data, prioritise optimising USB throughput by using multiples of OPT_PACKET_SIZE
				int numUSBPackets = std::max(1, (int)std::floor(largeBudget / OPT_PACKET_SIZE));
				largeBudget = std::min(largeBudget, numUSBPackets * OPT_PACKET_SIZE);
				// TODO: OPT_PACKET_SIZE is lower bound assuming a packet header for each USB packet, could go higher if numUSBPackets > 1
			}
			int sent = 0;
			int ret = comm_send_block(comm, packet.header, packet.data, packet.largePacketHeader, largeBudget, sent);
			if (ret > 0)
			{
				budget -= sent;
				sentTotal += sent;
				// Sent data, but not done, likely exhausting budget
				if (ret == 1) break;
				// Else done, remove packet and continue
			}
			else
			{
				// Failed to send any data, maybe budget too small
				// TODO: Consider interleaving smaller packets further down in the queue
				break;
			}
		}
		else if (comm_send_immediate(comm, packet.header, packet.data.data()))
		{ // Sent packet, substract from budget and continue
			budget -= packet.data.size();
			sentTotal += packet.data.size();
		}
		else
		{ // Failed to send packet
			printf("Failed to send queued packet interleaved with realtime data!\n");
		}
		// Remove packet and continue
		comm->packetQueue.pop();
	}

	return sentTotal;
}

/* Sets console to raw mode which among others allows for non-blocking input, even over SSH */
static void setConsoleRawMode(bool certainTTY)
{
	tcgetattr(STDIN_FILENO, &terminalSettings);
	struct termios termSet = terminalSettings;
	atexit([]{ // Reset at exit
		tcsetattr(STDIN_FILENO, TCSANOW, &terminalSettings);
	});
	termSet.c_lflag &= ~ICANON;
	if (certainTTY)
	{ // Nice-to-have, but bad if accidentally set as it affects gdb as well
		termSet.c_lflag &= ~ECHO;
	}
	termSet.c_cc[VMIN] = 0;
	termSet.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &termSet);
}

static char getConsoleChar()
{
	struct timeval timeout = { 0, 0 };
	// Set FD to STDIN
	fd_set readFD;
	FD_ZERO(&readFD);
	FD_SET(STDIN_FILENO, &readFD);
	// Check STDIN for bytes
	int status = select(STDIN_FILENO + 1, &readFD, nullptr, nullptr, &timeout);
	if (status < 0) return 0;
	if (!FD_ISSET(STDIN_FILENO, &readFD)) return 0;
	// STDIN has bytes
	TimePoint_t startWait = sclock::now();
	char cin;
	int ret = read(STDIN_FILENO, &cin, 1);
	if (ret == 1) return cin;
	if (ret < 0)
		printf("Failed to read console input: %d, %s (%c)\n", ret, strerror(errno), errno);
	return 0;
}