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

#ifndef SOCKET_H
#define SOCKET_H

#include <string.h>

#if defined(_WIN32)

#include <codecvt>
#include <locale>

#include <winsock2.h>
#include <ws2tcpip.h>

#ifndef HOST_NAME_MAX
#define HOST_NAME_MAX 255
#endif

#define SOCKET_ERR_NUM (WSAGetLastError())
#define SOCKET_ERR_STR (strerror(WSAGetLastError()))
#define VALID_SOCKET(sock) (sock != INVALID_SOCKET)

#elif defined(__unix__)

#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <limits.h>

#define SOCKET_ERR_NUM errno
#define SOCKET_ERR_STR (strerror(errno))
#define SOCKET int	// Because Windows uses unsigned int for socket...
#define VALID_SOCKET(sock) (sock >= 0)

#else
#error Unsupported platform for socket!
#endif


void *get_in_addr(struct sockaddr_storage *addr)
{
	if (addr->ss_family == AF_INET)
		return &(((struct sockaddr_in *)addr)->sin_addr);
	else if (addr->ss_family == AF_INET6)
		return &(((struct sockaddr_in6 *)addr)->sin6_addr);
	return NULL;
}

char *getHostnameString()
{
	static char host_str[HOST_NAME_MAX+1];
	gethostname(host_str, sizeof(host_str));
	return host_str;
}

char *getIPString(struct sockaddr_storage *addr)
{
	static char addr_str[INET6_ADDRSTRLEN+1];
	inet_ntop(addr->ss_family, get_in_addr(addr), addr_str, sizeof(addr_str));
	return addr_str;
}

uint16_t getPort(struct sockaddr_storage *addr)
{
	if (addr->ss_family == AF_INET)
		return htons(((struct sockaddr_in *)addr)->sin_port);
	else if (addr->ss_family == AF_INET6)
		return htons(((struct sockaddr_in *)addr)->sin_port);
	return 0;
}


std::string getAddrString(struct sockaddr_storage *addr)
{
	static char host_num[NI_MAXHOST+1];
	static char host_name[NI_MAXHOST+1];
	static char port[NI_MAXSERV+1];
	int status = getnameinfo((struct sockaddr*)addr, sizeof(*addr),
							host_name, sizeof(host_name),
							port, sizeof(port),
							NI_NUMERICSERV);
	int status2 = getnameinfo((struct sockaddr*)addr, sizeof(*addr),
							host_num, sizeof(host_num),
							port, sizeof(port),
							NI_NUMERICHOST | NI_NUMERICSERV);
#if defined(_WIN32) // Because windows silently breaks API and uses wide characters when unicode is enabled
	if (status != 0)
		return std::string("unknown:") + std::string(gai_strerror(status));
	else if (status2 != 0)
		return std::string("unknown2:") + std::string(gai_strerror(status2));
#elif defined(__unix__)
	if (status != 0)
		return std::string("unknown:") + gai_strerror(status);
	else if (status2 != 0)
		return std::string("unknown2:") + gai_strerror(status2);
#endif
	else
	{
		if (addr->ss_family == AF_INET)
			return std::string(host_name) + " (" + std::string(host_num) + ") :: " + std::string(port);
		else if (addr->ss_family == AF_INET6)
			return std::string(host_name) + " ([" + std::string(host_num) + "]) :: " + std::string(port);
		else
			return std::string(host_name) + " (unknown family) :: " + std::string(port);
	}
}

int socket_initialise()
{
#ifdef _WIN32
	WSADATA wsaData;
	int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (result != NO_ERROR)
		return -1;
#endif
	return 0;
}

bool socket_initialised()
{
#ifdef _WIN32
	SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock == INVALID_SOCKET && WSAGetLastError() == WSANOTINITIALISED)
		return false;
	closesocket(sock);
#endif
	return true;
}

void socket_cleanup()
{
#ifdef _WIN32
	WSACleanup();
#endif
}

void socket_close(int fd)
{
#ifdef _WIN32
	closesocket(fd);
#else
	close(fd);
#endif
}

#endif // SOCKET_H