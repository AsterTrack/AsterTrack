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

#include "comm/server.hpp"
#include "comm/socket.hpp"
#include "comm/protocol_stream.hpp"

#include <fcntl.h>
#include <thread>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>

#if defined(__unix__)
#include <termios.h>
#endif

typedef std::chrono::steady_clock sclock;

// ID of connected UART device and acknowledgement of own ID
static uint8_t msg_ack[1+PACKET_HEADER_SIZE], msg_nak[1+PACKET_HEADER_SIZE], msg_ping[1+PACKET_HEADER_SIZE];

// UART Identification and connection
#define COMM_PING_TIMEOUT 3000
#define COMM_RESET_TIMEOUT 500
#define COMM_IDENT_INTERVAL std::chrono::milliseconds(10)
#define COMM_IDENT_CYCLES 50 // Ident timeout is COMM_IDENT_CYCLES*COMM_SLEEP_INTERVAL
#define COMM_INTERVAL std::chrono::microseconds(100)
#define COMM_PING_INTERVAL 1000

void ClientThread(ClientCommState *clientState);

int socket_server_init(const char *port, struct sockaddr_storage* addr)
{
	struct addrinfo hints = {};
	hints.ai_family = AF_INET6;
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
		return -1;
	}
	*addr = *((sockaddr_storage*)server_addr->ai_addr);
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

	// Init predefined messages
	msg_ping[0] = '#';
	struct PacketHeader header(PACKET_PING, 0);
	storePacketHeader(header, msg_ping+1);
	msg_nak[0] = '#';
	header.tag = PACKET_NAK;
	storePacketHeader(header, msg_nak+1);
	msg_ack[0] = '#';
	header.tag = PACKET_ACK;
	storePacketHeader(header, msg_ack+1);
	
	return sock;
}

int ServerInit(std::string port)
{
	if (socket_initialise())
	{
		LOG(LServer, LDebug, "Failed to initialise socket!\n");
		return 0;
	}

	struct sockaddr_storage server_addr;
	int socket = socket_server_init(port.c_str(), &server_addr);
	if (socket < 0)
	{
		LOG(LServer, LDebug, "Server address was %s:%d!\n", getIPString(&server_addr), getPort(&server_addr));
		LOG(LServer, LDebug, "Server address was %s!\n", getAddrString(&server_addr).c_str());
		socket_cleanup();
		return 0;
	}
	LOG(LServer, LDebug, "Server address is %s:%d!\n", getIPString(&server_addr), getPort(&server_addr));
	LOG(LServer, LDebug, "Server address is %s!\n", getAddrString(&server_addr).c_str());
	LOG(LServer, LDebug, "Local server address is %s:%d!\n", getHostnameString(), getPort(&server_addr));
	return socket;
}

void ServerThread(ServerCommState *serverState)
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
	while (server.threadRun)
	{
		struct sockaddr_storage client_addr;
		socklen_t addr_size = sizeof(client_addr);
		int socket = accept(server.socket, (struct sockaddr *)&client_addr, &addr_size);
		if (socket < 0)
		{
#if defined(_WIN32)
			int err = WSAGetLastError();
			if (err != WSAEWOULDBLOCK)
				LOG(LServer, LDebug, "Failed to accept connection: %s (%d)\n", strerror(err), err);
#elif defined(__unix__)
			// All these are acceptable: ENETDOWN, EPROTO, ENOPROTOOPT, EHOSTDOWN, ENONET, EHOSTUNREACH, EOPNOTSUPP, or ENETUNREACH
			if (!(errno == EAGAIN || errno == EWOULDBLOCK))
				LOG(LServer, LDebug, "Failed to accept connection: %s (%d)\n", strerror(errno), errno);
#endif
			continue;
		}
		LOG(LServer, LInfo, "Connected to client %s:%d!\n", getIPString(&client_addr), getPort(&client_addr));
		LOG(LServer, LInfo, "Connected to client %s!\n", getAddrString(&client_addr).c_str());
#if defined(_WIN32)
		ioctlsocket(socket, FIONBIO, &mode);
#elif defined(__unix__)
		flags = fcntl(socket, F_GETFL, 0);
		fcntl(socket, F_SETFL, flags | O_NONBLOCK);
#endif
		
		// Start client thread
		server.clients.push_back({});
		ClientCommState &comm = server.clients.back();
		comm.socket = socket;
		comm.callbacks = server.callbacks;
		VersionDesc version(0, 0, 0); // TODO: Add proper version, pass on from ONE compilation unit (uses TIME!)
		comm.ownIdent = IdentPacket(DEVICE_SERVER, INTERFACE_SERVER, version);
		comm.expIdent.device = DEVICE_TRCAM;
		comm.enabled = true;
		comm.thread = new std::thread(ClientThread, &comm);
	}

	LOG(LServer, LDebug, "Server threads closing...\n");

	// Stop client comm
	for (ClientCommState &client : server.clients)
		client.enabled = false;
	for (ClientCommState &client : server.clients)
	{
		if (client.thread->joinable())
			client.thread->join();
	}
	for (ClientCommState &client : server.clients)
		socket_close(client.socket);
	server.clients.clear();
	socket_close(server.socket);
	socket_cleanup();
}

inline void comm_reset(ClientCommState &comm)
{
	comm.ident_timeout = 0;
	comm.ident_interval = 0;
	comm.rsp_ack = false;
	comm.rsp_id = false;
	comm.ready = false;
	proto_clear(comm.protocol);
}

inline void comm_close(ClientCommState &comm)
{
	comm.callbacks.onDisconnect(comm);
	comm.enabled = false;
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
		LOG(LServer, LWarn, "Socket error on send: %s (%d)\n", strerror(SOCKET_ERR_NUM), SOCKET_ERR_NUM);
#if defined(_WIN32)
		if (ret == EPIPE)
#elif defined(__unix__)
		if (ret == EPIPE)
#endif
		{
			comm_close(comm);
			return false;
		}
	}
	return true;
}

inline int comm_read_internal(ClientCommState &comm)
{
#if defined(_WIN32)
	int num = recv(comm.socket, (char *)comm.protocol.rcvBuf.data()+comm.protocol.tail, comm.protocol.rcvBuf.size()-comm.protocol.tail, 0);
#elif defined(__unix__)
	int num = read(comm.socket, comm.protocol.rcvBuf.data()+comm.protocol.tail, comm.protocol.rcvBuf.size()-comm.protocol.tail);
#endif
	if (num < 0)
	{
/*#if defined(_WIN32)
		int err = WSAGetLastError();
		LOG(LServer, LError, "WSA returned error %d on read\n", err);
		if (err != WSAEWOULDBLOCK)
		{
 			LOG(LServer, LError, "Received error num %d, resetting comm!\n", err);
			comm_close(comm);
			return -1;
		}
#elif defined(__unix__)*/
		if (errno == EBADF)
		{
			LOG(LServer, LError, "Bad file descriptor (EBADF)!\n");
			comm_close(comm);
			return -1;
		}
		else if (errno != EAGAIN && errno != EWOULDBLOCK)
		{
 			LOG(LServer, LError, "Socket error on read: %s (%d) - resetting comm!\n", strerror(errno), errno);
			comm_close(comm);
			return -1;
		}
//#endif
		return 0;
	}
	return num;
}

inline void comm_flush(ClientCommState &comm)
{
#if defined(__unix__)
	tcdrain(comm.socket); // Wait for bytes to be sent
#else
#endif
}

bool comm_writeHeader(ClientCommState &comm, PacketTag tag, uint16_t packetLength)
{
	if (comm.ready)
	{
		uint8_t buffer[1+PACKET_HEADER_SIZE];
		buffer[0] = '#';
		const PacketHeader header(tag, packetLength);
		storePacketHeader(header, buffer+1);
		return comm_write_internal(comm, buffer, sizeof(buffer));
	}
	return false;
}

bool comm_write(ClientCommState &comm, const uint8_t *data, uint16_t length)
{
	if (comm.ready)
	{
		if (comm_write_internal(comm, data, length))
		{
			return true;
		}
	}
	return false;
}


void comm_abort(ClientCommState &comm)
{
	if (comm_write_internal(comm, msg_nak, sizeof(msg_nak)))
	{
		comm_flush(comm);
		comm_reset(comm);
	}
}

static void comm_identify(ClientCommState &comm)
{
	const PacketHeader header(PACKET_IDENT, IDENT_PACKET_SIZE);
	uint8_t buffer[1+PACKET_HEADER_SIZE+IDENT_PACKET_SIZE];
	buffer[0] = '#';
	storePacketHeader(header, buffer+1);
	storeIdentPacket(comm.ownIdent, buffer+(1+PACKET_HEADER_SIZE));
	if (comm_write_internal(comm, buffer, sizeof(buffer)))
		comm_flush(comm);
	LOG(LServer, LDebug, "Sent identification packet!\n");
}

void ClientThread(ClientCommState *clientState)
{
	ClientCommState &comm = *clientState;
	ProtocolState &proto = comm.protocol;

	ScopedLogContext ServerLogContext(clientState->socket);

	TimePoint_t time_start, time_lastPing, time_read;
	time_start = sclock::now();

	while (comm.enabled)
	{
		if (comm.rsp_id && comm.rsp_ack)
			goto phase_idle;

		/* Identification Step */

phase_identification:

		if (!comm.enabled)
			break;

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		comm.ident_timeout = 0;
		comm.ident_interval = 0;

		if (!comm.enabled)
			break;

		// Send own ID
		comm_identify(comm);

		while (comm.enabled)
		{ // Identification loop to make sure communication works

			int num = comm_read_internal(comm);
			if (num < 0) break;
			if (proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.cmdNAK)
				{ // NAK received
					LOG(LServer, LWarn, "NAK Received, resetting comm!\n");
					comm_reset(comm);
					std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT));
					goto phase_identification;
				}
				else if (proto.cmdACK && !comm.rsp_ack)
				{ // ACK received
					LOG(LServer, LDebug, "Acknowledged!\n");
					comm.rsp_ack = true;
				}
				else if (proto.header.tag == PACKET_IDENT && proto_fetchCmd(proto))
				{ // Received full id command
					bool correct = proto.cmdSz == IDENT_PACKET_SIZE;
					if (correct)
					{
						IdentPacket rcvIdent = parseIdentPacket(proto.rcvBuf.data()+proto.cmdPos);
						correct = (rcvIdent.device&comm.expIdent.device) != 0 
							&& rcvIdent.type == comm.ownIdent.type
							&& rcvIdent.version.major == comm.ownIdent.version.major
							&& rcvIdent.version.minor >= comm.ownIdent.version.minor;
						if (correct)
						{ // Proper identity
							LOG(LServer, LDebug, "Identified communication partner!\n");
							comm.rsp_id = true;
							if (comm_write_internal(comm, msg_ack, sizeof(msg_ack)))
							{
								comm_flush(comm);
								if (!comm.rsp_ack) // Send own ID
									comm_identify(comm);
							}
							else
								break;
							comm.otherIdent = rcvIdent;
						}
					}
					if (!correct)
					{ // Wrong identity
						LOG(LServer, LDebug, "Failed to identify communication partner!\n");
						comm_abort(comm);
						std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT));
						goto phase_identification;
					}
				}
				if (comm.rsp_id && comm.rsp_ack)
				{ // Comm is ready
					LOG(LServer, LDebug, "Comm is ready!\n");
					comm.ready = true;
					proto_clear(proto);
					if (comm.callbacks.onIdentify)
						comm.callbacks.onIdentify(comm);
					goto phase_idle;
				}
			}

			// Count timeout from first interaction
			if (comm.rsp_ack || comm.rsp_id) comm.ident_timeout++;
			if (comm.ident_timeout > COMM_IDENT_CYCLES)
			{ // Identification timeout, send NAK
				LOG(LServer, LWarn, "Identification timeout exceeded, resetting comm!\n");
				comm_abort(comm);
				std::this_thread::sleep_for(std::chrono::milliseconds(COMM_RESET_TIMEOUT));
				goto phase_identification;
			}

			if (!comm.rsp_ack && !comm.rsp_id)
			{
				comm.ident_interval++;
				if (comm.ident_interval > 50)
				{ // UART ID packet send timeout
					comm_identify(comm);
					comm.ident_interval = 0;
				}
			}

			std::this_thread::sleep_for(COMM_IDENT_INTERVAL);
		}


		/* Setup phase */

phase_idle:

		if (!comm.enabled)
			break;

		time_read = sclock::now();
		while (comm.enabled)
		{ // Setup packets and start blob detection command
			int num = comm_read_internal(comm);
			if (num < 0) break;
			else if (num > 0) time_read = sclock::now();
			bool prevInCmd = proto.isCmd;
			bool newToParse = true;
			while (newToParse && proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.cmdNAK)
				{ // NAK received
					LOG(LServer, LWarn, "NAK Received, resetting comm!\n");
					comm_reset(comm);
					goto phase_identification;
				}
				else if (proto.cmdACK)
				{ // NAK received
					LOG(LServer, LDebug, "Redundant ACK Received!\n");
				}
				else if (proto.header.tag == PACKET_PING)
				{ // Received ping back
					proto_fetchCmd(proto);
				}
				else if (proto.header.tag == PACKET_IDENT)
				{ // Received redundant identification packet
					if (proto_fetchCmd(proto))
						LOG(LServer, LDebug, "Received redundant identification packet!\n");
				}
				else
				{ // Parse in blocks for lowest latency
					bool skip = false;
					if (!prevInCmd && comm.callbacks.onReceivePacketHeader)
						skip = !comm.callbacks.onReceivePacketHeader(comm, proto.header);
					if (skip)
					{ // TODO: Switch to proper skipping
						// This does NOT properly skip it with full size, but considers the header to be erroneous and starts searching for the next immediately
						proto.cmdSkip = true;
						LOG(LServer, LDebug, "Skipping command %d!\n", proto.header.tag);
					}
					else
					{
						int len = proto_handleCmdBlock(proto);
						if (len != 0 && comm.callbacks.onReceivePacketBlock)
							comm.callbacks.onReceivePacketBlock(comm, proto.header, proto.rcvBuf.data()+proto.cmdPos, len);
					}
				}

				// Continue parsing if the current command was handled fully
				int rem = proto.tail-proto.head;
				newToParse = rem > 0 && !proto.cmdSkip && !proto.isCmd && comm.enabled && comm.started;
				num = 0;
				if (newToParse)
					LOG(LServer, LDebug, "Continuing parsing of %d remaining bytes\n", rem);
			}

			// Check timeout
			// TODO: Add toggle for timeout, not needed for TCP, only for UART
			TimePoint_t curClock = sclock::now();
			int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(curClock - time_read).count();
			if (elapsedMS > COMM_PING_TIMEOUT)
			{ // Setup timeout, send NAK
				LOG(LServer, LWarn, "Ping dropped out, resetting comm!\n");
				comm_abort(comm);
				comm.callbacks.onDisconnect(comm);
				break;
			}
			elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(curClock - time_lastPing).count();
			if (elapsedMS > COMM_PING_INTERVAL)
			{ // Send ping
				if (comm_write_internal(comm, msg_ping, sizeof(msg_ping)))
					comm_flush(comm);
				time_lastPing = curClock;
			}

			std::this_thread::sleep_for(COMM_INTERVAL);

			fflush(stdout);
		}
	}
}