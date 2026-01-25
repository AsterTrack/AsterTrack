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
#include "parsing.hpp"
#include "comm/uart.h"
#include "mcu/mcu.hpp"
#include "version.hpp"

#include "util/util.hpp"

#include <thread>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <unistd.h>
#include <sys/prctl.h>

// Identification and connection
// TODO: Parametrise values and pick optimal for both UART and Wifi
#define COMM_PING_TIMEOUT_MS		500		// Controller/Server sends a ping every 100ms when not already streaming
											// 250ms is good for Controller, Server needs higher timeout
#define COMM_RESET_TIMEOUT_MS		100		// Timeout beween comm loss and next try
#define COMM_IDENT_TIMEOUT_MS		50		// Timeout between sending identification and requiring a response
 											// should be << 1ms for UART, < 50ms for wifi
#define COMM_IDENT_BACKOFF_MS		1000	// Additional backoff interval between failed identifications attempts to not spam log
#define COMM_INTERVAL_US			500

/*
 * Reading and parsing is done in CommThread
 * CommThread handles initial connection and sets comm.ready to open comm for non-protocol communication
 * - CommThread generally handles all writing for other threads via packetQueue
 * When comm.ready, writing may be claimed by main thread instead iff realTimeAff == comm.medium
 * - Then, main thread may use comm_packet/write/submit and comm_send_immediate APIs that assume full control to send realtime data
 * - It will also interleave other non-protocol packets from packetQueue (from CommThread or other threads)
 * - Protocol packets may still be sent from CommThread, using writeAccess mutex to schedule, but the need for that is exceedingly rare (usually only to stop comms anyway)
 * - In case a write error occurs while main thread has write control, it may only set the comm.error flag for CommThread to restart comms
 * Any thread intending to write without assuming full control has to use comm_send / comm_queue_send method to append to packetQueue
 */

CommList comms;
CommMedium realTimeAff = COMM_MEDIUM_UART, largeDataAff = COMM_MEDIUM_WIFI;

void comm_enable(CommState &comm, TrackingCameraState *state, CommMedium medium)
{
	assert(medium >= 0 && medium < COMM_MEDIUM_MAX);
	comm.enabled = true;
	comm.medium = medium;
	if (!comm.thread)
		comm.thread = new std::thread(CommThread, &comm, state);
	comms.medium[medium] = &comm;
}

void comm_disable(CommState &comm)
{
	auto pos = std::find(comms.medium.begin(), comms.medium.end(), &comm);
	if (pos != comms.medium.end())
		*pos = nullptr;
	{ // Don't know with what affinity packets were queued, just send them through any means necessary
		std::unique_lock lock(comm.queueMutex);
		CommState *altComm = comms.get(realTimeAff);
		if (altComm)
			altComm->queueMutex.lock();
		while (!comm.packetQueue.empty())
		{
			if (altComm)
				altComm->packetQueue.push(std::move(comm.packetQueue.front()));
			comm.packetQueue.pop();
		}
		if (altComm)
			altComm->queueMutex.unlock();
	}
	comm.enabled = false;
	if (comm.thread && comm.thread->joinable())
		comm.thread->join();
	delete comm.thread;
	comm.thread = nullptr;
}

bool comm_packet(CommState *commPtr, PacketHeader header)
{
	if (!commPtr || !commPtr->ready || commPtr->error) return false;
	CommState &comm = *commPtr;
	if (header.tag < PACKET_HOST_COMM)
	{
		printf("Attempting to send packet %d to Controller using methods intended for Host comm!\n", header.tag);
		return false;
	}
	comm.writeAccess.lock();
	if (!commPtr->ready || commPtr->error)
	{
		comm.writeAccess.unlock();
		return false;
	}
	// Write packet start
	UARTPacketRef packet;
	writeUARTPacketHeader(&packet, header);
	if (comm.write(comm.port, (uint8_t*)&packet, sizeof(packet)) < 0)
	{
		comm.error = true;
		comm.writeAccess.unlock();
		return false;
	}
	// Store ongoing checksum calculation data in handle - may be share among multiple CommStates
	comm.writing = true;
	comm.totalSize = header.length;
	comm.sentSize = 0;
	comm.crc.reset();
	return true;
}

void comm_write(CommState *commPtr, const uint8_t *data, uint32_t length)
{
	if (!commPtr || !commPtr->ready || !commPtr->writing || commPtr->error) return;
	CommState &comm = *commPtr;
	// Write packet data
	if (comm.write(comm.port, data, length) < 0)
	{
		comm.writing = false;
		comm.error = true;
		comm.writeAccess.unlock();
		return;
	}
	// Update checksum of block
	if (PACKET_CHECKSUM_SIZE == 4)
		comm.crc.add(data, length);
	comm.sentSize += length;
}

void comm_submit(CommState *commPtr)
{
	if (!commPtr || !commPtr->ready || !commPtr->writing || commPtr->error) return;
	CommState &comm = *commPtr;
	if (comm.sentSize != comm.totalSize)
		printf("Comm submit after %d of %d bytes got sent!\n", comm.sentSize, comm.totalSize);
	// Write packet end (see writeUARTPacketEnd)
	uint8_t buffer[PACKET_CHECKSUM_SIZE+UART_TRAILING_SEND];
	for (int i = 0; i < UART_TRAILING_SEND; i++)
		buffer[PACKET_CHECKSUM_SIZE+i] = UART_TRAILING_BYTE;
	if (comm.totalSize > 0)
	{ // Send packet checksum
		comm.crc.getHash(buffer);
		if (comm.write(comm.port, buffer, sizeof(buffer)) < 0)
			comm.error = true;
	}
	else
	{ // Skip packet checksum
		if (comm.write(comm.port, buffer+PACKET_CHECKSUM_SIZE, sizeof(buffer)-PACKET_CHECKSUM_SIZE) < 0)
			comm.error = true;
	}
	if (!comm.error)
		comm.submit(comm.port);
	comm.writing = false;
	comm.writeAccess.unlock();
}

bool comm_send_immediate(CommState *commPtr, PacketHeader header, const uint8_t *data)
{
	if (comm_packet(commPtr, header))
	{
		if (header.length > 0)
			comm_write(commPtr, data, header.length);
		comm_submit(commPtr);
	}
	return true;
}

int comm_send_block(CommState *commPtr, PacketHeader header, const std::vector<uint8_t> &data, std::vector<uint8_t> &largePacketHeader, int budget, int &sent)
{
	sent = 0;
	if (budget < 100) return 0;
	LargePacketHeader *largePacket = (LargePacketHeader*)largePacketHeader.data();
	int numBytes = budget - largePacketHeader.size();
	int bytesLeft = largePacket->totalSize - largePacket->blockOffset;
	if (numBytes >= bytesLeft) numBytes = bytesLeft;
	else if (bytesLeft-numBytes < 100) numBytes = bytesLeft;

	// Send as many bytes
	largePacket->blockSize = numBytes;
	header.length = largePacketHeader.size() + numBytes;
	if (comm_packet(commPtr, header))
	{
		comm_write(commPtr, largePacketHeader.data(), largePacketHeader.size());
		comm_write(commPtr, data.data()+largePacket->blockOffset, largePacket->blockSize);
		comm_submit(commPtr);
	}
	else return -1;
	largePacket->blockOffset += largePacket->blockSize;
	largePacket->blockSize = 0;
	sent = numBytes;
	return largePacket->blockOffset >= largePacket->totalSize? 2 : 1;
}

bool comm_queue_send(CommState *commPtr, PacketHeader header, std::vector<uint8_t> &&data)
{
	if (!commPtr || !commPtr->enabled || !commPtr->started || !commPtr->ready || commPtr->error) return false;
	std::unique_lock lock(commPtr->queueMutex);
	commPtr->packetQueue.emplace(header, std::move(data));
	return true;
}

bool comm_queue_send(CommState *commPtr, PacketHeader header, std::vector<uint8_t> &&data, std::vector<uint8_t> largePacketHeader)
{
	if (!commPtr || !commPtr->enabled || !commPtr->started || !commPtr->ready || commPtr->error) return false;
	std::unique_lock lock(commPtr->queueMutex);
	commPtr->packetQueue.emplace(header, std::move(data), std::move(largePacketHeader));
	return true;
}

bool comm_send(CommState *commPtr, PacketHeader header, const uint8_t *data)
{
	if (!commPtr || !commPtr->enabled || !commPtr->started || !commPtr->ready || commPtr->error) return false;
	if (commPtr->realtimeControlled || std::this_thread::get_id() != commPtr->thread->get_id())
	{ // Need to enqueue for different controlling thread to send
		std::vector<uint8_t> packet(header.length);
		memcpy(packet.data(), data, header.length);
		return comm_queue_send(commPtr, header, std::move(packet));
	}
	else
		return comm_send_immediate(commPtr, header, data);
}

bool comm_send(CommState *commPtr, PacketHeader header, std::vector<uint8_t> &&data)
{
	if (!commPtr || !commPtr->enabled || !commPtr->started || !commPtr->ready || commPtr->error) return false;
	if (data.size() != header.length) return false;
	if (commPtr->realtimeControlled || std::this_thread::get_id() != commPtr->thread->get_id())
	{ // Need to enqueue for different controlling thread to send
		return comm_queue_send(commPtr, header, std::move(data));
	}
	else
		return comm_send_immediate(commPtr, header, data.data());
}

bool comm_send(CommMedium comm, PacketHeader header, const uint8_t *data)
{
	return comm_send(comms.get(comm), header, data);
}

bool comm_send(CommMedium comm, PacketHeader header, std::vector<uint8_t> &&data)
{
	return comm_send(comms.get(comm), header, std::move(data));
}

inline void comm_reset(CommState &comm)
{
	comm.ready = false;
	// Read and discard what came immediately after abort
	comm.read(comm.port, comm.protocol.rcvBuf.data(), comm.protocol.rcvBuf.size());
	std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT_MS));
	comm.read(comm.port, comm.protocol.rcvBuf.data(), comm.protocol.rcvBuf.size());
	proto_clear(comm.protocol);
}

inline void comm_stop(CommState &comm)
{
	if (comm.started)
		comm.stop(comm.port);
	comm.started = comm.ready = comm.error = false;
	proto_clear(comm.protocol);
}

inline bool comm_write_internal(CommState &comm, const uint8_t *data, uint16_t length)
{ // All internal (protocol) writes are simple messages
	comm.writeAccess.lock();
	int ret = comm.write(comm.port, data, length);
	if (ret >= 0)
		comm.submit(comm.port);
	else
		comm.error = true;
	comm.writeAccess.unlock();
	return ret >= 0;
}

inline int comm_read_internal(CommState &comm, uint32_t timeoutUS)
{
	int num = 0;
	if (timeoutUS > 0 && (num = comm.wait(comm.port, timeoutUS)) <= 0)
	{ // wait returns 1 on read, negative on error, 0 on timeout
		if (num < 0)
		{ // TODO: what kind of errors can wait (select) return?
			if (errno != EAGAIN)
				return -1;
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
			return -1;
		return 0;
	}
	comm.protocol.tail += num;
	return num;
}

bool comm_interject(CommState *commPtr)
{
	if (!commPtr || !commPtr->ready || commPtr->error) return false;
	CommState &comm = *commPtr;
	if (!comm.writing) return true;
	comm.writing = false;
	// Interrupting, if we send more than required, that's fine, just can't rely on existing writer finishing their write
	uint8_t buffer[UART_TRAILING_SEND];
	for (int i = 0; i < UART_TRAILING_SEND; i++)
		buffer[i] = UART_TRAILING_BYTE;
	if (comm.write(comm.port, comm.protocol.rcvBuf.data(), std::min((int)comm.protocol.rcvBuf.size(), comm.totalSize-comm.sentSize+PACKET_CHECKSUM_SIZE)) < 0)
		comm.error = true;
	else if (comm.write(comm.port, buffer, UART_TRAILING_SEND) < 0)
		comm.error = true;
	comm.writeAccess.unlock();
	return true;
}

void comm_force_close(CommState *commPtr)
{
	if (!commPtr || !commPtr->ready || commPtr->error) return;
	CommState &comm = *commPtr;
	uint8_t msg_nak[UART_PACKET_OVERHEAD_SEND];
	finaliseDirectUARTPacket(msg_nak, PacketHeader(PACKET_NAK, 0));
	comm.write(comm.port, msg_nak, sizeof(msg_nak));
	comm.flush(comm.port);
	comm.error = true;
}

void CommThread(CommState *comm_ptr, TrackingCameraState *state_ptr)
{
	CommState &comm = *comm_ptr;
	ProtocolState &proto = comm.protocol;
	TrackingCameraState &state = *state_ptr;
	TimeSync timesync;

	// Init predefined messages
	uint8_t msg_ack[UART_PACKET_OVERHEAD_SEND], msg_nak[UART_PACKET_OVERHEAD_SEND], msg_ping[UART_PACKET_OVERHEAD_SEND];
	finaliseDirectUARTPacket(msg_ping, PacketHeader(PACKET_PING, 0));
	finaliseDirectUARTPacket(msg_nak, PacketHeader(PACKET_NAK, 0));
	finaliseDirectUARTPacket(msg_ack, PacketHeader(PACKET_ACK, 0));

	// Set name to use for thread and logging
	const char* commName = "Unknown";
	if (comm.medium == COMM_MEDIUM_UART)
		commName = "UART";
	else if (comm.medium == COMM_MEDIUM_WIFI)
		commName = "Wireless";

	// Set thread properties
	prctl(PR_SET_NAME, asprintf_s("Comm_%s", commName).c_str());
	nice(-20); // Highest priority

	TimePoint_t time_begin, time_read, time_ident, time_start;
	time_begin = sclock::now();
	int ident_backoff = 0;
	bool sentInfo = false;
	while (comm.enabled)
	{

		/* Starting Step */

phase_start:

		if (!comm.enabled)
			break;

		if (comm.error)
		{
			printf("%s: Handling error from external thread!\n", commName);
			comm_stop(comm);
		}

		while (!comm.started && comm.enabled)
		{
			comm.started = comm.start(comm.port);
			std::this_thread::sleep_for(std::chrono::milliseconds(comm.started? 10 : 1000));
		}
		comm.ready = false;
		proto_clear(comm.protocol);

		if (!comm.enabled)
			break;


		/* Identification Step */

phase_identification:

		if (!comm.enabled || !comm.started || comm.error)
			continue;

		if (ident_backoff++ > 5)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(COMM_IDENT_BACKOFF_MS));

			if (!comm.enabled || !comm.started || comm.error)
				continue;
		}

		{ // Send own ID
			printf("%s: Sending identification!\n", commName);
			uint8_t identBuffer[UART_PACKET_OVERHEAD_SEND+IDENT_PACKET_SIZE];
			UARTPacketRef *packet = (UARTPacketRef*)identBuffer;
			storeIdentPacket(comm.ownIdent, packet->data);
			finaliseDirectUARTPacket(packet, PacketHeader(PACKET_IDENT, IDENT_PACKET_SIZE));
			comm_write_internal(comm, (uint8_t*)packet, sizeof(identBuffer));
		}
		time_ident = sclock::now();

		while (comm.enabled && comm.started && !comm.error)
		{ // Identification loop to make sure communication works

			fflush(stdout);

			int num = comm_read_internal(comm, COMM_INTERVAL_US);
			if (num < 0)
			{ // Error
				printf("%s: Read error during identification!\n", commName);
				comm_write_internal(comm, msg_nak, sizeof(msg_nak));
				comm_stop(comm);
				goto phase_start;
			}
			bool prevInCmd = proto.isCmd;
			bool newToParse = num > 0;
			while (newToParse && proto_rcvCmd(proto) && !comm.error)
			{ // Got a new command to handle
				if (proto.header.tag == PACKET_NAK)
				{ // NAK received
					if (proto_fetchCmd(proto))
					{
						printf("%s: Identification rejected (NAK) %.2fms after sending!\n", commName, dtMS(time_ident, sclock::now()));
						comm_reset(comm);
						goto phase_identification;
					}
				}
				else if (proto.header.tag == PACKET_IDENT)
				{
					if (proto_fetchCmd(proto))
					{ // Received full identification command (implicit acknowledgement of own identification)
						bool correct = proto.cmdSz == IDENT_PACKET_SIZE && proto.validChecksum;
						if (correct)
						{
							IdentPacket rcvIdent = parseIdentPacket(proto.rcvBuf.data()+proto.cmdPos);
							correct = (rcvIdent.device&comm.expIdent.device) != 0 && rcvIdent.type == comm.ownIdent.type;
							// TODO: Handle differing versions - ideally, try to connect anyway to allow for updating
							if (rcvIdent.version.major != comm.ownIdent.version.major)
								printf("%s: Potential Version Mismatch; Own is v%d.%d and Remote is v%d.%d!\n", commName,
									comm.ownIdent.version.major, comm.ownIdent.version.minor, rcvIdent.version.major, rcvIdent.version.minor);
							if (correct)
							{ // Proper identity
								printf("%s: Valid identification response received %.2fms after sending!\n", commName, dtMS(time_ident, sclock::now()));
								if (!comm.started)
									break;
								comm.ready = true;
								comm_write_internal(comm, msg_ack, sizeof(msg_ack));
								comm.otherIdent = rcvIdent;
								ident_backoff = 0;
								goto phase_comm;
							}
							else
								printf("%s: Incorrect identification response received %.2fms after sending!\n", commName, dtMS(time_ident, sclock::now()));
						}
						else
							printf("%s: Invalid identification response received %.2fms after sending!\n", commName, dtMS(time_ident, sclock::now()));
						// Wrong identity
						comm_write_internal(comm, msg_nak, sizeof(msg_nak));
						comm_reset(comm);
						goto phase_identification;
					}
				}
				else
				{
					printf("%s: Received unexpected or unknown tag %d!\n", commName, proto.header.tag);
					comm_write_internal(comm, msg_nak, sizeof(msg_nak));
					comm_reset(comm);
					goto phase_identification;
				}

				// Continue parsing if the current command was handled fully
				newToParse = proto.tail > proto.head && !proto.cmdSkip && !proto.isCmd && comm.enabled && comm.started;
				num = 0;
			}

			if (dtMS(time_ident, sclock::now()) > COMM_IDENT_TIMEOUT_MS)
			{ // Identification timeout
				printf("%s: Identification timeout exceeded, trying again!\n", commName);
				comm_write_internal(comm, msg_nak, sizeof(msg_nak));
				comm_reset(comm);
				goto phase_identification;
			}
		}


		/* Comm phase */

phase_comm:

		if (!comm.enabled || !comm.started || comm.error)
			continue;

		ResetTimeSync(timesync);

		time_start = sclock::now();
		sentInfo = false;

		time_read = sclock::now();
		while (comm.enabled && comm.started && !comm.error)
		{
			if (mcu_active)
			{
				// UART RX on the Pi has severe problems, it relies on interrupts clearing the 16-byte FIFO within 22us at 8Mbaud
				// So frequently, there are missed interrupts and thus bytes missing, making UART RX unreliable
				// That's why frame sync and all control packets are routed through the MCU (and it needs to parse most of the packets anyway)
				// The only exception are firmware packets, which are too large and already have robust error handling built-in
				if (timesync.measurements)
					ResetTimeSync(timesync);
			}

			int num = comm_read_internal(comm, COMM_INTERVAL_US);
			if (num < 0)
			{ // Error
				printf("%s: Read error during communications!\n", commName);
				comm_write_internal(comm, msg_nak, sizeof(msg_nak));
				comm_stop(comm);
				goto phase_start;
			}
			else if (num > 0) time_read = sclock::now();
			bool prevInCmd = proto.isCmd;
			bool newToParse = num > 0;
			while (newToParse && proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.header.tag == PACKET_NAK && proto_fetchCmd(proto))
				{ // NAK received
					printf("%s: NAK Received, resetting comm!\n", commName);
					comm_reset(comm);
					goto phase_identification;
				}
				else if (proto.header.tag == PACKET_ACK && proto_fetchCmd(proto))
				{ // ACK received
					printf("%s: Redundant ACK Received!\n", commName);
				}
				else if (proto.header.tag == PACKET_PING)
				{ // Received ping, answer ping
					if (proto_fetchCmd(proto))
						comm_write_internal(comm, msg_ping, sizeof(msg_ping));
				}
				else if (proto.header.tag == PACKET_IDENT)
				{ // Received redundant identification packet
					printf("%s: Unexpected Identification received, dropping existing comms!\n", commName);
					comm_write_internal(comm, msg_nak, sizeof(msg_nak));
					comm_reset(comm);
					goto phase_identification;
				}
				else if (proto.header.tag == PACKET_SYNC && !mcu_active)
				{ // Received sync packet
					if (proto_fetchCmd(proto) && proto.validChecksum && proto.cmdSz >= SYNC_PACKET_SIZE)
					{
						struct SyncPacket sync = parseSyncPacket(&proto.rcvBuf[proto.cmdPos]);
						if (dtUS(timesync.lastTime, sclock::now()) > 1000000)
							ResetTimeSync(timesync);
						UpdateTimeSync(timesync, sync.timeUS, 1<<24, time_read);
					}
				}
				else if (proto.header.tag == PACKET_SOF && !mcu_active)
				{ // Received sof packet
					if (proto_fetchCmd(proto) && proto.validChecksum && proto.cmdSz >= SOF_PACKET_SIZE)
					{
						struct SOFPacket sync = parseSOFPacket(&proto.rcvBuf[proto.cmdPos]);
						if (dtUS(timesync.lastTime, sclock::now()) > 1000000)
							ResetTimeSync(timesync);
						TimePoint_t SOF = UpdateTimeSync(timesync, sync.timeUS, 1<<24, time_read);
						std::unique_lock lock(framesync.access);
						if (framesync.frameSOFs.size() > 5)
							framesync.frameSOFs.pop();
						framesync.frameSOFs.emplace(sync.frameID & 0xFF, SOF);
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
						comm_write_internal(comm, msg_nak, sizeof(msg_nak));
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
						comm_write_internal(comm, msg_ack, sizeof(msg_ack));
						successful_verified++;
					}
					else
					{
						comm_write_internal(comm, msg_nak, sizeof(msg_nak));
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
				else if (ReceivePacketHeader(comm, proto.header))
				{
					if (proto_fetchCmd(proto))
					{
						ReceivePacketData(state, comm, proto.header, proto.rcvBuf.data()+proto.cmdPos, proto.cmdSz, !proto.validChecksum);
					}
				}

				// Continue parsing if the current command was handled fully
				int rem = proto.tail-proto.head;
				newToParse = rem > 0 && !proto.cmdSkip && !proto.isCmd && comm.enabled && comm.started;
				num = 0;
			}

			// Check timeout
			// TODO: Add toggle for timeout, not needed for TCP, only for UART
			if (dtMS(time_read, sclock::now()) > COMM_PING_TIMEOUT_MS)
			{ // Setup timeout, send NAK
				printf("%s: Ping dropped out, resetting comm!\n", commName);
				comm_write_internal(comm, msg_nak, sizeof(msg_nak));
				comm_reset(comm);
				break;
			}

			fflush(stdout);

			if (!sentInfo && dtMS(time_start, sclock::now()) > 250)
			{
				printf("%s: Sent info packet!\n", commName);
				sendInfoPacket(comm.medium);
				sentInfo = true;
			}

			// Parsing done, now send from packet queue if it's not handled by main thread for realtime data
			std::unique_lock lock(comm.queueMutex);
			while (!comm_ptr->realtimeControlled && !comm.packetQueue.empty())
			{
				auto &packet = comm.packetQueue.front();
				if (!packet.largePacketHeader.empty())
				{ // Send large packet all at once since it's not realtime-relevant
					int sent = 0;
					int ret = comm_send_block(comm_ptr, packet.header, packet.data, packet.largePacketHeader, std::numeric_limits<int>::max(), sent);
					if (ret != 2)
					{ // Not fully sent, should not occur
						printf("%s: Failed to send large packet from comm thread!\n", commName);
						comm.packetQueue.pop();
						break;
					}
				}
				else if (!comm_send_immediate(comm_ptr, packet.header, packet.data.data()))
				{ // Failed to send queued packet
					printf("%s: Failed to send queued packet from comm thread!\n", commName);
					comm.packetQueue.pop();
					break;
				}
				comm.packetQueue.pop();
			}
		}

		if (comm.error)
			printf("%s: Comms got interrupted because of a write error in another thread!\n", commName);
		else
			printf("%s: Comms got interrupted for unknown reason!\n", commName);
	}

	comm_stop(comm);
}