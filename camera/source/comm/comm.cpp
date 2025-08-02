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

#include "comm.hpp"
#include "comm/protocol_stream.hpp"
#include "timesync.hpp"
#include "wireless.hpp"

#include "util/util.hpp"

#include <thread>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <unistd.h>

// Predefined messages
static uint8_t msg_ack[1+PACKET_HEADER_SIZE], msg_nak[1+PACKET_HEADER_SIZE], msg_ping[1+PACKET_HEADER_SIZE];

// UART Identification and connection
#define UART_COMM_TIMEOUT_MS		250		// Controller sends a ping every 100ms when not already streaming
#define COMM_RESET_TIMEOUT_MS		100		// Timeout beween comm loss and next try
#define COMM_IDENT_TIMEOUT_MS		10		// Timeout between sending identification and requiring a response - should be << 1ms
#define COMM_IDENT_BACKOFF_MS		1000	// Additional backoff interval between failed identifications attempts to not spam log
#define COMM_INTERVAL_US			500

void comm_init()
{
	// Init predefined messages
	msg_ping[0] = UART_LEADING_BYTE;
	struct PacketHeader header(PACKET_PING, 0);
	storePacketHeader(header, msg_ping+1);
	msg_nak[0] = UART_LEADING_BYTE;
	header.tag = PACKET_NAK;
	storePacketHeader(header, msg_nak+1);
	msg_ack[0] = UART_LEADING_BYTE;
	header.tag = PACKET_ACK;
	storePacketHeader(header, msg_ack+1);
}

inline void comm_reset(CommState &comm)
{
	comm.ready = false;
	comm.writing = false;
	proto_clear(comm.protocol);
}

inline void comm_stop(CommState &comm)
{
	if (comm.started)
		comm.stop(comm.port);
	comm.started = comm.ready = false;
}

void comm_close(CommState &comm)
{
	comm_stop(comm);
	comm_reset(comm);
}

inline void comm_abort(CommState &comm)
{
	comm_NAK(comm);
	comm_reset(comm);
	comm.read(comm.port, comm.protocol.rcvBuf.data(), comm.protocol.rcvBuf.size());
	std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT_MS));
	comm.read(comm.port, comm.protocol.rcvBuf.data(), comm.protocol.rcvBuf.size());
	// Read and discard what came immediately after abort
	comm_reset(comm);
}

inline bool comm_write_internal(CommState &comm, const uint8_t *data, uint16_t length)
{ // All internal (protocol) writes are simple messages
	comm.writeAccess.lock();
	int ret = comm.write(comm.port, data, length);
	if (ret < 0)
	{
		comm_close(comm);
		comm.writeAccess.unlock();
		return false;
	}
	comm.submit(comm.port);
	comm.writeAccess.unlock();
	return true;
}

inline int comm_read_internal(CommState &comm, uint32_t timeoutUS)
{
	int num = 0;
	if (timeoutUS > 0 && (num = comm.wait(comm.port, timeoutUS)) <= 0)
	{ // wait returns 1 on read, negative on error, 0 on timeout
		if (num < 0)
		{ // TODO: what kind of errors can wait (select) return?
			if (errno != EAGAIN)
			{
				comm_close(comm);
				return -1;
			}
		}
		return 0;
	}
	proto_clean(comm.protocol, true);
	num = comm.read(comm.port, &comm.protocol.rcvBuf[comm.protocol.tail], comm.protocol.rcvBuf.size()-comm.protocol.tail);
	/* if (num > 0)
	{
		std::stringstream hexBuf;
		printBuffer(hexBuf, &comm.protocol.rcvBuf[comm.protocol.tail], num);
		printf("TX IN %d: %s\n", num, hexBuf.str().c_str());
	} */
	if (num < 0)
	{
		// Log error
		if (errno == ECONNRESET)
			printf("Connection reset by peer!\n");
		else if (errno == EBADF)
			printf("Bad file descriptor (EBADF)!\n");
		else if (errno != EAGAIN && errno != EWOULDBLOCK)
			printf("Received error num %d, resetting comm!\n", errno);
		// React to error
		if (errno != EAGAIN && errno != EWOULDBLOCK)
		{
			comm_close(comm);
			return -1;
		}
		return 0;
	}
	comm.protocol.tail += num;
	return num;
}

void comm_NAK(CommState &comm)
{
	if (comm_write_internal(comm, msg_nak, sizeof(msg_nak)))
		comm_flush(comm);
}

inline void comm_ACK(CommState &comm)
{
	if (comm_write_internal(comm, msg_ack, sizeof(msg_ack)))
		comm_flush(comm);
}

static void comm_identify(CommState &comm)
{
	if (!comm.started) return;
	const PacketHeader header(PACKET_IDENT, IDENT_PACKET_SIZE);
	uint8_t buffer[1+PACKET_HEADER_SIZE+IDENT_PACKET_SIZE];
	buffer[0] = UART_LEADING_BYTE;
	storePacketHeader(header, buffer+1);
	storeIdentPacket(comm.ownIdent, buffer+(1+PACKET_HEADER_SIZE));
	if (comm_write_internal(comm, buffer, sizeof(buffer)))
		comm_flush(comm);
}

void CommThread(CommState *comm_ptr, TrackingCameraState *state_ptr)
{
	CommState &comm = *comm_ptr;
	ProtocolState &proto = comm.protocol;
	TrackingCameraState &state = *state_ptr;

	TimePoint_t time_begin, time_read, time_ident, time_start;
	time_begin = sclock::now();
	int ident_backoff = 0;

	nice(-20); // Highest priority
	while (comm.enabled)
	{

		/* Starting Step */

phase_start:

		if (!comm.enabled)
			break;

		while (!comm.started)
		{
			comm.started = comm.start(comm.port);
			comm_reset(comm);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		if (comm.ready)
			goto phase_comm;

		/* Identification Step */

phase_identification:

		if (!comm.enabled || !comm.started)
			continue;

		if (ident_backoff++ > 5)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(COMM_IDENT_BACKOFF_MS));

			if (!comm.enabled || !comm.started)
				continue;
		}

		// Send own ID
		printf("Sending identification!\n");
		comm_identify(comm);
		time_ident = sclock::now();

		while (comm.enabled && comm.started)
		{ // Identification loop to make sure communication works

			fflush(stdout);

			int num = comm_read_internal(comm, COMM_INTERVAL_US);
			if (num < 0)
			{ // Error
				printf("UART error during identification!\n");
				comm_NAK(comm);
				comm_close(comm);
				goto phase_start;
			}
			bool prevInCmd = proto.isCmd;
			bool newToParse = num > 0;
			while (newToParse && proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.header.tag == PACKET_NAK)
				{ // NAK received
					if (proto_fetchCmd(proto))
					{
						printf("Identification rejected (NAK) %.2fms after sending!\n", dtMS(time_ident, sclock::now()));
						comm_reset(comm);
						std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT_MS));
						goto phase_identification;
					}
				}
				else if (proto.header.tag == PACKET_IDENT)
				{
					if (proto_fetchCmd(proto))
					{ // Received full identification command (implicit acknowledgement of own identification)
						bool correct = proto.cmdSz == IDENT_PACKET_SIZE;
						if (correct)
						{
							IdentPacket rcvIdent = parseIdentPacket(proto.rcvBuf.data()+proto.cmdPos);
							correct = (rcvIdent.device&comm.expIdent.device) != 0 
								&& rcvIdent.type == comm.ownIdent.type
								&& rcvIdent.version.major == comm.ownIdent.version.major
								&& rcvIdent.version.minor >= comm.ownIdent.version.minor;
							// TODO: Deal with versions
							if (correct)
							{ // Proper identity
								printf("Valid identification response received %.2fms after sending!\n", dtMS(time_ident, sclock::now()));
								if (!comm.started)
									break;
								comm.ready = true;
								comm_ACK(comm);
								comm.otherIdent = rcvIdent;
								proto_clear(proto);
								ident_backoff = 0;
								goto phase_comm;
							}
							else
								printf("Incorrect identification response received %.2fms after sending!\n", dtMS(time_ident, sclock::now()));
						}
						else
							printf("Invalid identification response received %.2fms after sending!\n", dtMS(time_ident, sclock::now()));
						// Wrong identity
						comm_abort(comm);
						goto phase_identification;
					}
				}
				else
				{
					printf("Received unexpected or unknown tag %d!\n", proto.header.tag);
					comm_abort(comm);
					goto phase_identification;
				}

				// Continue parsing if the current command was handled fully
				int rem = proto.tail-proto.head;
				newToParse = rem > 0 && !proto.cmdSkip && !proto.isCmd && comm.enabled && comm.started;
				num = 0;
			}

			if (dtMS(time_ident, sclock::now()) > COMM_IDENT_TIMEOUT_MS)
			{ // Identification timeout
				printf("Identification timeout exceeded, trying again!\n");
				std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT_MS));
				goto phase_identification;
			}
		}


		/* Comm phase */

phase_comm:

		if (!comm.enabled || !comm.started)
			continue;

		ResetTimeSync(state.sync.time);

		// Notify host of wireless status
		if (state.wireless.enabled && !state.wireless.updating)
			state.wireless.needsStatusPacket = true;

		time_start = sclock::now();

		time_read = sclock::now();
		while (comm.enabled && comm.started)
		{
			int num = comm_read_internal(comm, COMM_INTERVAL_US);
			if (num < 0)
			{ // Error
				comm_NAK(comm);
				comm_close(comm);
				goto phase_start;
			}
			else if (num > 0) time_read = sclock::now();
			bool prevInCmd = proto.isCmd;
			bool newToParse = num > 0;
			while (newToParse && proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.header.tag == PACKET_NAK && proto_fetchCmd(proto))
				{ // NAK received
					printf("NAK Received, resetting comm!\n");
					comm_reset(comm);
					std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT_MS));
					goto phase_identification;
				}
				else if (proto.header.tag == PACKET_ACK && proto_fetchCmd(proto))
				{ // ACK received
					printf("Redundant ACK Received!\n");
				}
				else if (proto.header.tag == PACKET_PING)
				{ // Received ping, answer ping
					if (proto_fetchCmd(proto))
						comm_write_internal(comm, msg_ping, sizeof(msg_ping));
				}
				else if (proto.header.tag == PACKET_IDENT)
				{ // Received redundant identification packet
					comm_abort(comm);
					goto phase_identification;
				}
				else if (proto.header.tag == PACKET_SYNC)
				{ // Received sync packet
					if (proto_fetchCmd(proto) && proto.cmdSz >= SYNC_PACKET_SIZE)
					{
						struct SyncPacket sync = parseSyncPacket(&proto.rcvBuf[proto.cmdPos]);
						std::unique_lock lock(state.sync.access);
						if (dtUS(state.sync.time.lastTime, sclock::now()) > 1000000)
							ResetTimeSync(state.sync.time);
						UpdateTimeSync(state.sync.time, sync.timeUS, 1<<24, time_read);
					}
				}
				else if (proto.header.tag == PACKET_SOF)
				{ // Received sof packet
					if (proto_fetchCmd(proto) && proto.cmdSz >= SOF_PACKET_SIZE)
					{
						struct SOFPacket sync = parseSOFPacket(&proto.rcvBuf[proto.cmdPos]);
						std::unique_lock lock(state.sync.access);
						if (dtUS(state.sync.time.lastTime, sclock::now()) > 1000000)
							ResetTimeSync(state.sync.time);
						TimePoint_t SOF = UpdateTimeSync(state.sync.time, sync.timeUS, 1<<24, time_read);
						state.sync.frameSOFs.emplace(sync.frameID, SOF);
						//printf("Corrected SOF read %lldus ago to be %.2fms in the past!\n", std::chrono::duration_cast<std::chrono::microseconds>(sclock::now()-time_read).count(), std::chrono::duration_cast<std::chrono::microseconds>(sclock::now()-SOF).count()/1000.0f);
					}
				}
				/* else if (proto.header.tag == PACKET_RATE_CONFIG)
				{ // Received request to change baud rate
					// TODO: Read baudrate, verfication_blocks, and timeout
					old_baudrate = old? UART_BAUD_RATE_SAFE? idk
					if (comm.configure)
						comm.configure(comm, baudrate);
					else
						comm_NAK(comm);
					time_rate_timeout = sclock::now() + std::chrono::milliseconds(timeout);
					in_rate_config = true;
					required_verified = verfication_blocks
					successful_verified = 0;
					unsuccessful_verified = 0;
				}
				else if (proto.header.tag == PACKET_RATE_VERIFY)
				{ // Received request to verify packet
					bool correctChecksum = true;
					if (correctChecksum)
					{
						comm_ACK(comm);
						successful_verified++;
					}
					else
					{
						comm_NAK(comm);
						unsuccessful_verified++;
					}

				}
				else if (in_rate_config && sclock::now() > time_rate_timeout)
				{
					if (successful_verified == required_verified && unsuccessful_verified == 0)
					{
						// LOG Switch successful
					}
					else
					{
						if (comm.configure)
							comm.configure(comm, old_baudrate);
						comm_reset(comm);
						// TODO: Initiate identification after a further timeout
					}
				} */
				else if (proto.header.tag == PACKET_CFG_SETUP)
				{ // Received setup packet
					if (proto_fetchCmd(proto) && proto.cmdSz >= CONFIG_PACKET_SIZE)
					{
						ConfigPacket packet = parseConfigPacket(&proto.rcvBuf[proto.cmdPos]);
						printf("Received configuration: %dx%d @ %d fps, exposure level %d, %dx gain, m=%.2f, n=%.2f, extTrig? %c, strobe? %c (%d)\n",
							packet.width, packet.height, packet.fps, packet.shutterSpeed,
							packet.analogGain, packet.blobProc.thresholds.absolute/255.0f, packet.blobProc.thresholds.edge/255.0f,
							packet.extTrig? 'y' : 'n', packet.strobe? 'y' : 'n', packet.strobeLength);
						state.newConfigPacket = packet;
						state.updateSetupQPU = true;
						state.updateSetupCPU = true;
					}
				}
				else if (proto.header.tag == PACKET_CFG_MODE)
				{ // Mode set
					if (proto_fetchCmd(proto) && proto.cmdSz >= 1)
					{
						uint8_t modePacket = proto.rcvBuf[proto.cmdPos];
						TrackingCameraMode newMode = {};
						newMode.streaming = (modePacket&TRCAM_FLAG_STREAMING) != 0;
						if (newMode.streaming)
						{
							uint8_t modeSwitch = modePacket&TRCAM_MODE_MASK;
							uint8_t mode;
							for (mode = 7; mode > 0; mode--)
								if (modeSwitch >> mode) break;
							uint8_t optMask = 0xFF >> (8-mode);
							newMode.mode = (TrCamMode)(modeSwitch&(~optMask));
							newMode.opt = (TrCamMode)(modeSwitch&optMask);
						}
						while (state.updateMode)
							std::this_thread::sleep_for(std::chrono::milliseconds(1));
						state.newModeRaw = (TrCamMode)modePacket;
						state.newMode = newMode;
						state.updateMode = true;
					}
				}
				else if (proto.header.tag == PACKET_CFG_WIFI)
				{
					if (proto_fetchCmd(proto))
					{
						parseWirelessConfigPacket(state, proto.rcvBuf.data()+proto.cmdPos, proto.cmdSz);
					}
				}
				else if (proto.header.tag == PACKET_CFG_IMAGE)
				{
					if (proto_fetchCmd(proto) && proto.cmdSz >= 1)
					{
						ImageStreamState stream = {};
						stream.enabled = proto.rcvBuf[proto.cmdPos+0] != 0;
						bool oneshot = proto.rcvBuf[proto.cmdPos+0] == 2;
						if (stream.enabled && proto.cmdSz >= 12)
						{
							stream.bounds = Bounds2<int>(
								*((uint16_t*)&proto.rcvBuf[proto.cmdPos+1]),
								*((uint16_t*)&proto.rcvBuf[proto.cmdPos+3]),
								*((uint16_t*)&proto.rcvBuf[proto.cmdPos+5]),
								*((uint16_t*)&proto.rcvBuf[proto.cmdPos+7])
							);

							stream.subsampling = proto.rcvBuf[proto.cmdPos+9];
							stream.jpegQuality = proto.rcvBuf[proto.cmdPos+10];
							stream.frame = proto.rcvBuf[proto.cmdPos+11];

							/* if (oneshot)
							{
								printf("Requested image of frame %d in rect (%d,%d)x(%d,%d) subsampled %d at %d quality\n",
									stream.frame,
									stream.bounds.minX, stream.bounds.minY, stream.bounds.maxX, stream.bounds.maxY,
									stream.subsampling, stream.jpegQuality);
							}
							else
							{
								printf("Requested streaming every %d frames in rect (%d,%d)x(%d,%d) subsampled %d at %d quality\n",
									stream.frame,
									stream.bounds.minX, stream.bounds.minY, stream.bounds.maxX, stream.bounds.maxY,
									stream.subsampling, stream.jpegQuality);
							} */
						}
						else
							printf("Frame streaming disabled!\n");

						if (oneshot)
							state.imageRequests.push(stream);
						else
							state.streaming = stream;
					}
				}
				else if (proto.header.tag == PACKET_CFG_VIS)
				{
					if (proto_fetchCmd(proto))
					{
						state.visualisation.enabled = proto.rcvBuf[proto.cmdPos+0];
						if (state.visualisation.enabled)
						{
							state.visualisation.displayFrame = proto.rcvBuf[proto.cmdPos+0];
							state.visualisation.width = *((uint16_t*)&proto.rcvBuf[proto.cmdPos+1]);
							state.visualisation.height = *((uint16_t*)&proto.rcvBuf[proto.cmdPos+3]);
							state.visualisation.interval = 1000000/proto.rcvBuf[proto.cmdPos+5];
							state.visualisation.displayFrame = proto.rcvBuf[proto.cmdPos+6];
							state.visualisation.displayBlobs = proto.rcvBuf[proto.cmdPos+7];
							printf("Visualisation at %dx%d every %d frames!\n", state.visualisation.width, state.visualisation.height, state.visualisation.interval);
						}
						else
							printf("Visualisation disabled!\n");
					}
				}

				// Continue parsing if the current command was handled fully
				int rem = proto.tail-proto.head;
				newToParse = rem > 0 && !proto.cmdSkip && !proto.isCmd && comm.enabled && comm.started;
				num = 0;
			}

			// Check timeout
			// TODO: Add toggle for timeout, not needed for TCP, only for UART
			if (dtMS(time_read, sclock::now()) > UART_COMM_TIMEOUT_MS)
			{ // Setup timeout, send NAK
				printf("Ping dropped out, resetting comm!\n");
				comm_abort(comm);
				break;
			}

			fflush(stdout);
		}
	}

	comm_stop(comm);
}