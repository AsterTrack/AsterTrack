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

#include "util/util.hpp" // sclock, TimePoint_t
#include "util/log.hpp"

#include "comm/wireless_server.hpp"
#include "comm/wireless_server_client.hpp"
#include "comm/socket.hpp"
#include "comm/protocol_stream.hpp"
#include "comm/uart.h" // Use same format as UART just to share protocol_stream
#include "comm/packet.hpp"

#include <fcntl.h>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>

#if defined(__unix__)
#include <termios.h>
#endif

// ID of connected UART device and acknowledgement of own ID
static uint8_t msg_ack[UART_PACKET_OVERHEAD_SEND], msg_nak[UART_PACKET_OVERHEAD_SEND], msg_ping[UART_PACKET_OVERHEAD_SEND];

// UART Identification and connection
#define COMM_RESET_TIMEOUT_MS		50		// Timeout beween comm loss and next try
#define COMM_INTERVAL_US			10000
#define COMM_PING_INTERVAL_MS		100
#define COMM_PING_TIMEOUT_MS		1000

// Found in version.cpp
extern VersionDesc serverVersion;
extern std::string serverVersionDescriptor;

void ClientThread(std::stop_token stop_token, ClientCommState *clientState);

template<> void OpaqueDeleter<ClientCommState>::operator()(ClientCommState* ptr) const
{ delete ptr; }

static int socket_server_init(const char *port)
{
	struct addrinfo hints = {};
	hints.ai_family = AF_INET6; // Should be able to accept IPv4 as well on most systems
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// TODO: Explicit selection of server properties
	// ai_family=AF_UNSPEC returns both IPv4 and IPv6 options (as chained addrinfo in ai_next field)
	// Some platforms support dualstack, so IPv6 also accepts IPv4, which is the simplest
	// On windows, IPv6 will have IPV6_V6ONLY enabled by default, so will have to disable that for dualstack
	// But dualstack isn't supported on windows XP, BSD, etc.
	// Alternative is to bind to both IPv4 and IPv6 and handle multiple sockets (nasty)
	// getaddrinfo will ALSO return multiple options (ai_next) if multiple network interfaces exist
	// So in the future, we will have to select one by user configuration (IPvX, network interface) if multiple exist

	struct addrinfo *server_addr;
	int status = getaddrinfo(NULL, port, &hints, &server_addr);
	if (status != 0)
	{
		LOG(LServer, LError, "Failed to get addr info: %s\n", gai_strerror(status));
#if defined(__unix__)
		if (status == EAI_SYSTEM)
			LOG(LServer, LError, "System error: %s (%d)\n", strerror(errno), errno);
#endif
		return -1;
	}
	if (server_addr->ai_next != NULL)
	{
		LOG(LServer, LDebug, "Got more options than just one!\n");
	}

	SOCKET sock = socket(server_addr->ai_family, server_addr->ai_socktype, server_addr->ai_protocol);
	if (!VALID_SOCKET(sock))
	{
		LOG(LServer, LError, "Failed to create socket! Error %d\n", SOCKET_ERR_NUM);
		freeaddrinfo(server_addr);
		return -1;
	}

	int opt = 1;
	status = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
	if (status != 0)
	{ // If getaddrinfo returns ANY address (which gets resolved to be a specific network interface once something connects to it)
		LOG(LServer, LError, "Failed to set SO_REUSEADDR socket options! Error %d\n", SOCKET_ERR_NUM);
		freeaddrinfo(server_addr);
		return -1;
	}
	if (server_addr->ai_family == AF_INET6)
	{
		opt = 0;
		status = setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, (char*)&opt, sizeof(opt));
		if (status != 0)
		{
			LOG(LServer, LError, "Failed to set IPV6_V6ONLY socket option! Error %d\n", SOCKET_ERR_NUM);
			freeaddrinfo(server_addr);
			return -1;
		}
	}

	status = bind(sock, server_addr->ai_addr, server_addr->ai_addrlen);
	if (status != 0)
	{
		LOG(LServer, LError, "Failed to bind socket to port! Error %d\n", SOCKET_ERR_NUM);
		freeaddrinfo(server_addr);
		return -1;
	}

	freeaddrinfo(server_addr);

	return sock;
}

std::string WirelessServerGetHostname()
{
	return getHostnameString();
}

void WirelessServerInit()
{
	if (socket_initialise())
	{
		LOG(LServer, LDebug, "Failed to initialise socket!\n");
	}
	// Init predefined messages
	finaliseDirectUARTPacket(msg_ping, PacketHeader(PACKET_PING, 0));
	finaliseDirectUARTPacket(msg_nak, PacketHeader(PACKET_NAK, 0));
	finaliseDirectUARTPacket(msg_ack, PacketHeader(PACKET_ACK, 0));
}

void WirelessServerCleanup()
{
	socket_cleanup();
}

int WirelessServerOpen(std::string port)
{
	int socket = socket_server_init(port.c_str());
	if (socket < 0)
		return 0;
	return socket;
}

void WirelessServerThread(std::stop_token stop_token, ServerCommState *serverState)
{
	ServerCommState &server = *serverState;

	if (server.socket == 0)
		return;

	int status = listen(server.socket, 10);
	if (status < 0)
		LOG(LServer, LDebug, "Failed to listen on socket: %s (%d)\n", strerror(SOCKET_ERR_NUM), SOCKET_ERR_NUM);

#if defined(_WIN32)
	unsigned long mode = 1;
	ioctlsocket(server.socket, FIONBIO, &mode);
#elif defined(__unix__)
	int flags = fcntl(server.socket, F_GETFL, 0);
	fcntl(server.socket, F_SETFL, flags | O_NONBLOCK);
#endif

	LOG(LServer, LDebug, "Waiting for server connections...\n");
	while (!stop_token.stop_requested())
	{
		struct sockaddr_storage client_addr;
		socklen_t addr_size = sizeof(client_addr);
		int socket = accept(server.socket, (struct sockaddr *)&client_addr, &addr_size);
		if (socket < 0)
		{
#if defined(_WIN32)
	#pragma warn "Verify socket error handling!"
			if (SOCKET_ERR_NUM != WSAEWOULDBLOCK)
#elif defined(__unix__)
			// All these are acceptable: ENETDOWN, EPROTO, ENOPROTOOPT, EHOSTDOWN, ENONET, EHOSTUNREACH, EOPNOTSUPP, or ENETUNREACH
			if (!(SOCKET_ERR_NUM == EAGAIN || SOCKET_ERR_NUM == EWOULDBLOCK))
#endif
				LOG(LServer, LDebug, "Failed to accept connection: %s (%d)\n", SOCKET_ERR_STR, SOCKET_ERR_NUM);
			continue;
		}
		LOG(LServer, LDebug, "Connected to client %s!\n", getAddrString(&client_addr).c_str());
#if defined(_WIN32)
		ioctlsocket(socket, FIONBIO, &mode);
#elif defined(__unix__)
		flags = fcntl(socket, F_GETFL, 0);
		fcntl(socket, F_SETFL, flags | O_NONBLOCK);
#endif
		
		// Start client thread
		auto comm = make_opaque<ClientCommState>();
		comm->socket = socket;
		comm->callbacks = server.callbacks;
		comm->ownIdent = IdentPacket(DEVICE_SERVER, INTERFACE_SERVER, serverVersion);
		comm->expIdent.device = DEVICE_TRCAM;
		comm->thread = new std::jthread(ClientThread, comm.get());
		server.clients.push_back(std::move(comm));
	}

	LOG(LServer, LDebug, "Server threads closing...\n");

	// Stop client comm
	for (auto &client : server.clients)
		delete client->thread;
	for (auto &client : server.clients)
		socket_close(client->socket);
	server.clients.clear();
	socket_close(server.socket);

	LOG(LServer, LDebug, "All server threads closed!\n");
}

inline void comm_reset(ClientCommState &comm)
{
	comm.ready = comm.rsp_id = comm.rsp_ack = false;
	// Read and discard what came immediately after abort
	recv(comm.socket, (char*)comm.protocol.rcvBuf.data(), comm.protocol.rcvBuf.size(), 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT_MS));
	recv(comm.socket, (char*)comm.protocol.rcvBuf.data(), comm.protocol.rcvBuf.size(), 0);
	proto_clear(comm.protocol);
}

inline void comm_stop(ClientCommState &comm)
{
	comm.thread->request_stop();
	comm.ready = comm.rsp_id = comm.rsp_ack = false;
	proto_clear(comm.protocol);
}

inline bool comm_write_internal(ClientCommState &comm, const uint8_t *data, uint16_t length)
{
#if defined(_WIN32)
	int ret = send(comm.socket, (const char*)data, length, 0);
#elif defined(__unix__)
	int ret = send(comm.socket, data, length, MSG_NOSIGNAL);
#endif
	if (ret < 0)
	{
#if defined(_WIN32)
		if (SOCKET_ERR_NUM == WSAECONNRESET)
#elif defined(__unix__)
		if (SOCKET_ERR_NUM == EPIPE)
#endif
		{
			comm_stop(comm);
			return false;
		}
		LOG(LServer, LWarn, "Socket error on send: %s (%d)\n", SOCKET_ERR_STR, SOCKET_ERR_NUM);
	}
	return true;
}

inline int comm_read_internal(ClientCommState &comm, uint32_t timeoutUS)
{
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeoutUS;

	// Return values for select indicating events in camera file descriptor
	fd_set readFD;
	FD_ZERO(&readFD);
	FD_SET(comm.socket, &readFD);

	// Wait for changes from camera file descriptor
	int status = select(comm.socket + 1, &readFD, nullptr, nullptr, &timeout);
	if (status < 0)
	{ // Error
		if (SOCKET_ERR_NUM != EAGAIN)
		{
			LOG(LServer, LError, "Socket error on wait: %s (%d)\n", SOCKET_ERR_STR, SOCKET_ERR_NUM);
			return -1;
		}
		return 0;
	}
	if (FD_ISSET(comm.socket, &readFD) == 0)
		return 0;
	proto_clean(comm.protocol, true);

	int num = recv(comm.socket, (char*)comm.protocol.rcvBuf.data()+comm.protocol.tail, comm.protocol.rcvBuf.size()-comm.protocol.tail, 0);
	if (num < 0)
	{
#if defined(_WIN32)
		LOG(LServer, LError, "Socket error on read: %s (%d)\n", SOCKET_ERR_STR, SOCKET_ERR_NUM);
		if (SOCKET_ERR_NUM != WSAEWOULDBLOCK)
		{
 			LOG(LServer, LError, "Resetting comm!\n");
			return -1;
		}
#elif defined(__unix__)
		if (SOCKET_ERR_NUM == ECONNRESET)
			LOG(LServer, LError, "Connection reset by peer!\n");
		if (SOCKET_ERR_NUM == EBADF)
			LOG(LServer, LError, "Bad file descriptor (EBADF)!\n");
		else if (SOCKET_ERR_NUM != EAGAIN && SOCKET_ERR_NUM != EWOULDBLOCK)
 			LOG(LServer, LError, "Socket error on read: %s (%d) - resetting comm!\n", SOCKET_ERR_STR, SOCKET_ERR_NUM);
		// React to error
		if (SOCKET_ERR_NUM != EAGAIN && SOCKET_ERR_NUM != EWOULDBLOCK)
			return -1;
#endif
		return 0;
	}
	comm.protocol.tail += num;
	return num;
}

static inline void comm_flush(ClientCommState &comm)
{
#if defined(__unix__)
	tcdrain(comm.socket); // Wait for bytes to be sent
#else
#endif
}

static inline void comm_NAK(ClientCommState &comm)
{
	if (comm_write_internal(comm, msg_nak, sizeof(msg_nak)))
		comm_flush(comm);
}

static inline void comm_ACK(ClientCommState &comm)
{
	if (comm_write_internal(comm, msg_ack, sizeof(msg_ack)))
		comm_flush(comm);
}

static void comm_identify(ClientCommState &comm)
{
	uint8_t identBuffer[UART_PACKET_OVERHEAD_SEND+IDENT_PACKET_SIZE];
	UARTPacketRef *packet = (UARTPacketRef*)identBuffer;
	storeIdentPacket(comm.ownIdent, packet->data);
	finaliseDirectUARTPacket(packet, PacketHeader(PACKET_IDENT, IDENT_PACKET_SIZE));
	if (comm_write_internal(comm, (uint8_t*)packet, sizeof(identBuffer)))
		comm_flush(comm);
	LOG(LServer, LDebug, "Sent identification packet!\n");
}

bool comm_write(ClientCommState &comm, PacketTag tag, const uint8_t *data, uint16_t length)
{
	thread_local std::vector<uint8_t> packetBuffer;
	packetBuffer.resize(UART_PACKET_OVERHEAD_SEND + (length > 0? length : -PACKET_CHECKSUM_SIZE));
	UARTPacketRef *packet = (UARTPacketRef*)packetBuffer.data();
	writeUARTPacketHeader(packet, PacketHeader(tag, length));
	if (length > 0)
	{ // Write appropriate checksum
		memcpy(packet->data, data, length);
		if (tag >= PACKET_HOST_COMM)
			calculateForwardPacketChecksum(packet->data, length, packet->data+length);
		else // We should not be sending these packets, but do allow for it
			calculateDirectPacketChecksum(packet->data, length, packet->data+length);
		writeUARTPacketEnd(packet, length+PACKET_CHECKSUM_SIZE);
	}
	else // No checksum
		writeUARTPacketEnd(packet, length);
	bool success = comm_write_internal(comm, packetBuffer.data(), packetBuffer.size());
	if (success) comm_flush(comm);
	return success;
}

void ClientThread(std::stop_token stop_token, ClientCommState *clientState)
{
	ClientCommState &comm = *clientState;
	ProtocolState &proto = comm.protocol;

	ScopedLogContext ServerLogContext(clientState->socket);

	TimePoint_t time_begin, time_read, time_start, time_lastPing;
	time_begin = sclock::now();

	while (!stop_token.stop_requested())
	{

		/* Identification Step */

phase_identification:

		if (stop_token.stop_requested())
			break;

		while (!stop_token.stop_requested())
		{ // Identification loop to make sure communication works

			int num = comm_read_internal(comm, COMM_INTERVAL_US);
			if (num < 0)
			{ // Error
				LOG(LServer, LError, "Read error during identification!\n");
				comm_NAK(comm);
				comm_stop(comm);
				break;
			}
			bool prevInCmd = proto.isCmd;
			bool newToParse = num > 0;
			while (!stop_token.stop_requested() && newToParse && proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.header.tag == PACKET_NAK)
				{ // NAK received
					if (proto_fetchCmd(proto))
					{
						LOG(LServer, LWarn, "Identification rejected (NAK)!");
						comm_reset(comm);
						std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT_MS));
						goto phase_identification;
					}
				}
				else if (proto.header.tag == PACKET_ACK)
				{ // ACK received
					if (proto_fetchCmd(proto))
					{
						if (!comm.rsp_id)
						{ // Was not expecting an ACK
							LOG(LServer, LDarn, "Received unexpected ACK during identification!");
							comm_NAK(comm);
							comm_reset(comm);
							goto phase_identification;
						}
						LOG(LServer, LDebug, "Identification accepted!");
						comm.ready = true;
						if (comm.callbacks.onIdentify)
							if (!comm.callbacks.onIdentify(comm))
								break;
						goto phase_comm;
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
								LOG(LServer, LWarn, "Potential Version Mismatch Server is v%d.%d and Camera is v%d.%d!\n",
									comm.ownIdent.version.major, comm.ownIdent.version.minor, rcvIdent.version.major, rcvIdent.version.minor);
							if (correct)
							{ // Proper identity
								LOG(LServer, LDebug, "Valid identification response received!");
								comm.rsp_id = true;
								comm.otherIdent = rcvIdent;
								comm_identify(comm);
							}
						}
						if (!correct)
						{
							LOG(LServer, LDebug, "Invalid identification response received!");
							comm_NAK(comm);
							comm_reset(comm);
							goto phase_identification;
						}
					}
					else
					{
						LOG(LServer, LDebug, "Received unexpected or unknown tag %d!", proto.header.tag);
						comm_NAK(comm);
						comm_reset(comm);
						goto phase_identification;
					}
				}

				// Continue parsing if the current command was handled fully
				newToParse = proto.tail > proto.head && !proto.cmdSkip && !proto.isCmd;
				num = 0;
			}
		}


		/* Comm phase */

phase_comm:

		if (stop_token.stop_requested())
			break;

		//ResetTimeSync(comm.sync.time);

		time_start = sclock::now();

		time_read = sclock::now();
		while (!stop_token.stop_requested())
		{
			int num = comm_read_internal(comm, COMM_INTERVAL_US);
			TimePoint_t receiveTime = sclock::now();
			if (num < 0)
			{ // Error
				LOG(LServer, LError, "Read error during communications!\n");
				comm_NAK(comm);
				comm_stop(comm);
				break;
			}
			else if (num > 0) time_read = sclock::now();
			bool prevInCmd = proto.isCmd;
			bool newToParse = num > 0;
			while (!stop_token.stop_requested() && newToParse && proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.header.tag == PACKET_NAK && proto_fetchCmd(proto))
				{ // NAK received
					LOG(LServer, LWarn, "NAK Received, resetting comm!\n");
					if (comm.callbacks.onDisconnect)
						comm.callbacks.onDisconnect(comm);
					comm_reset(comm);
					goto phase_identification;
				}
				else if (proto.header.tag == PACKET_ACK && proto_fetchCmd(proto))
				{ // ACK received
					LOG(LServer, LDebug, "Redundant ACK Received!\n");
				}
				else if (proto.header.tag == PACKET_PING)
				{ // Received ping back
					proto_fetchCmd(proto);
				}
				else if (proto.header.tag == PACKET_IDENT)
				{ // Received redundant identification packet
					LOG(LServer, LWarn, "Received redundant identification packet!\n");
					comm_NAK(comm);
					comm_reset(comm);
					goto phase_identification;
				}
				else
				{ // Parse in blocks for lowest latency
					bool skip = false;
					if (!prevInCmd && comm.callbacks.onReceivePacketHeader)
						skip = !comm.callbacks.onReceivePacketHeader(comm, proto.header, receiveTime);
					if (skip)
					{
						proto.cmdSkip = true;
						prevInCmd = false;
						LOG(LServer, LDarn, "Skipping command %d!\n", proto.header.tag);
					}
					else
					{
						bool done = proto_fetchCmd(proto);
						LOG(LServer, LTrace, "Received packet %d, block %d bytes, total %d / %d bytes%s\n",
							proto.header.tag, proto.blockLen, proto.blockPos + proto.blockLen - proto.cmdPos, proto.cmdSz, done? (proto.validChecksum? ", valid" : ", invalid") : "");

						if (proto.blockLen != 0 && comm.callbacks.onReceivePacketBlock)
							comm.callbacks.onReceivePacketBlock(comm, proto.header, proto.rcvBuf.data()+proto.blockPos, proto.blockLen, receiveTime);
						if (done && comm.callbacks.onReceivePacket)
							comm.callbacks.onReceivePacket(comm, proto.header, proto.rcvBuf.data()+proto.cmdPos, proto.cmdSz, receiveTime, !proto.validChecksum);
						prevInCmd = !done;
					}
				}

				// Continue parsing if the current command was handled fully
				newToParse = proto.tail > proto.head && !proto.cmdSkip && !proto.isCmd;
				num = 0;
			}

			// Check timeout
			// TODO: Add toggle for timeout, not needed for TCP, only for UART
			TimePoint_t curClock = sclock::now();
			int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(curClock - time_read).count();
			if (elapsedMS > COMM_PING_TIMEOUT_MS)
			{ // Setup timeout, send NAK
				LOG(LServer, LWarn, "Ping dropped out, resetting comm!\n");
				if (comm.callbacks.onDisconnect)
					comm.callbacks.onDisconnect(comm);
				comm_NAK(comm);
				comm_reset(comm);
				break;
			}
			elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(curClock - time_lastPing).count();
			if (elapsedMS > COMM_PING_INTERVAL_MS)
			{ // Send ping
				if (comm_write_internal(comm, msg_ping, sizeof(msg_ping)))
					comm_flush(comm);
				time_lastPing = curClock;
			}

			fflush(stdout);
		}

		if (stop_token.stop_requested())
			break;
	}

	LOG(LServer, LDebug, "Stopping server comm thread!\n");
	if (comm.callbacks.onDisconnect)
		comm.callbacks.onDisconnect(comm);
}