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

#include "server.hpp"
#include "comm/socket.hpp"

#include <fcntl.h>
#include <termios.h>

//#include <netinet/tcp.h>
//#include <limits.h>
#include <mutex>

struct ServerConnection
{
	int sock;
	std::string host;
	std::string port;
	std::mutex writeAccess;
};

void *server_init(std::string host, std::string port)
{
	ServerConnection *socket = new ServerConnection();
	socket->host = host;
	socket->port = port;
	return socket;
}

void server_deinit(void *port)
{
	if (port == NULL)
		return;
	delete (ServerConnection*)port;
}

int socket_client_init(const char *server, const char *port, struct sockaddr_storage* addr)
{
	struct addrinfo hints = {};
	hints.ai_family = AF_INET; // tinycore by default does not support IPv6, though support could be added
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	struct addrinfo *server_addr;
	int status = getaddrinfo(server, port, &hints, &server_addr);
	if (status != 0)
	{
		printf("Failed to get addr info: %s\n", gai_strerror(status));
		return -1;
	}
	*addr = *((sockaddr_storage*)server_addr->ai_addr);
	if (server_addr->ai_next != NULL)
	{
		printf("Got more options than just one!\n");
	}

	int sock = socket(server_addr->ai_family, server_addr->ai_socktype, server_addr->ai_protocol);
	if (sock < 0)
	{
		printf("Failed to create socket!\n");
		freeaddrinfo(server_addr);
		return -1;
	}

	status = connect(sock, server_addr->ai_addr, server_addr->ai_addrlen);
	if (status < 0)
	{
		freeaddrinfo(server_addr);
		return -1;
	}

	freeaddrinfo(server_addr);
	
	return sock;
}

bool server_start(void *port)
{
	ServerConnection &socket = *((ServerConnection*)port);
	struct sockaddr_storage server_addr;
	socket.sock = socket_client_init(socket.host.c_str(), socket.port.c_str(), &server_addr);
	if (socket.sock < 0)
	{
		if (socket.sock > -4)
		{
			printf("Server address was %s:%d!\n", getIPString(&server_addr), getPort(&server_addr));
			printf("Server address was %s!\n", getAddrString(&server_addr).c_str());
		}
		return false;
	}
	printf("Connected to server %s:%d!\n", getIPString(&server_addr), getPort(&server_addr));
	printf("Connected to server %s!\n", getAddrString(&server_addr).c_str());
	int flags = fcntl(socket.sock, F_GETFL, 0);
	fcntl(socket.sock, F_SETFL, flags | O_NONBLOCK);
	int state = 1;
	// TODO: Set TCP_NODELAY to make write be immediate writes (disable Nagle algorithm to coalesce write data)
	setsockopt(socket.sock, IPPROTO_TCP, TCP_NODELAY, &state, sizeof(state));
	// OR better, set Cork so it coalesces packets, and then disable+enable it on flush to force immediate write
	setsockopt(socket.sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
	// OR, even better, use writev() and TCP_NODELAY to send packets consisting of multiple buffers as one packet immediately
	return true;
}

void server_stop(void *port)
{
	if (port == NULL)
		return;
	ServerConnection &socket = *((ServerConnection*)port);
	if (socket.sock != 0)
		close(socket.sock);
}

int server_read(void *port, uint8_t *buf, uint32_t len)
{
	ServerConnection &socket = *((ServerConnection*)port);
	return read(socket.sock, buf, len);
//	return recv(socket.sock, buf, len);
}

int server_wait(void *port, uint32_t timeoutUS)
{
	ServerConnection &socket = *((ServerConnection*)port);
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeoutUS;

	// Return values for select indicating events in camera file descriptor
	fd_set readFD;
	FD_ZERO(&readFD);
	FD_SET(socket.sock, &readFD);

	// Wait for changes from camera file descriptor
	int status = select(socket.sock + 1, &readFD, nullptr, nullptr, &timeout);
	if (status < 0) return status;
	return FD_ISSET(socket.sock, &readFD);
}

int server_write(void *port, const uint8_t *buf, uint32_t len)
{
	ServerConnection &socket = *((ServerConnection*)port);
	socket.writeAccess.lock();
	int wr = send(socket.sock, buf, len, MSG_NOSIGNAL);
//	int wr = write(socket.sock, buf, len);
	socket.writeAccess.unlock();
	return wr;
}

void server_submit(void *port)
{
	ServerConnection &socket = *((ServerConnection*)port);
	int state = 0;
	setsockopt(socket.sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
	state = 1;
	setsockopt(socket.sock, IPPROTO_TCP, TCP_CORK, &state, sizeof(state));
}

void server_flush(void *port)
{
	ServerConnection &socket = *((ServerConnection*)port);
	tcdrain(socket.sock);
}