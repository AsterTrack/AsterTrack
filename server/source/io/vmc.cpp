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

#include "vmc.hpp"

#include "comm/socket.hpp"
#include "signals.hpp" // Signals

#include "util/log.hpp"
#include "util/util.hpp"
#include "util/error.hpp"

#include "oscpack/OscOutboundPacketStream.hpp"

#include <array>
#include <cstring>

struct vmc_output
{
	std::string host, port;
	int socket = -1;
	bool connected = false;
	bool erroneous = false;
};

const std::array<std::string, (std::size_t)VMCRole::MAX> deviceRoleMap = {
	"/VMC/Ext/Cam",
	"/VMC/Ext/Hmd/Pos",
	"/VMC/Ext/Con/Pos",
	"/VMC/Ext/Tra/Pos"
};

template<> void OpaqueDeleter<vmc_output>::operator()(vmc_output* ptr) const
{
	if (ptr->socket >= 0)
		socket_close(ptr->socket);
	delete ptr;
}

static std::optional<ErrorMessage> socket_udp_init(const std::string &host, const std::string &port, int &sock, struct sockaddr_storage &addr)
{
	sock = -1;

	struct addrinfo hints = {};
	hints.ai_family = host == "localhost"? AF_INET : AF_UNSPEC; // Prefer IPv4 localhost
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_protocol = IPPROTO_UDP;
	hints.ai_flags = AI_ALL;

	struct addrinfo *server_addr;
	int status = getaddrinfo(host.c_str(), port.c_str(), &hints, &server_addr);
	if (status != 0)
	{
		auto error = asprintf_s("Failed to resolve '%s:%s': %d", host.c_str(), port.c_str(), status);
#if defined(__unix__)
		if (status == EAI_SYSTEM)
			error += asprintf_s(": %s (%d)", strerror(errno), errno);
#endif
		return error;
	}
	memcpy(&addr, server_addr->ai_addr, server_addr->ai_addrlen);

	sock = socket(server_addr->ai_family, server_addr->ai_socktype, server_addr->ai_protocol);
	if (sock < 0)
	{
		freeaddrinfo(server_addr);
		return "Failed to create UDP socket!";
	}

	status = connect(sock, server_addr->ai_addr, server_addr->ai_addrlen);
	if (status < 0)
	{
		socket_close(sock);
		sock = -1;
		freeaddrinfo(server_addr);
		return asprintf_s("Failed to connect to UDP remote '%s:%s'!", host.c_str(), port.c_str());
	}

	freeaddrinfo(server_addr);
	return std::nullopt;
}

opaque_ptr<vmc_output> vmc_init_output(const std::string &host, const std::string &port)
{
	return make_opaque<vmc_output>(host, port);
}

bool vmc_is_opened(opaque_ptr<vmc_output> &vmc)
{
	return vmc->socket >= 0;
}

bool vmc_is_connected(opaque_ptr<vmc_output> &vmc)
{
	return vmc->socket >= 0 && vmc->connected;
}

bool vmc_try_open(opaque_ptr<vmc_output> &vmc, std::string &address)
{
	if (vmc->socket >= 0) return true;

	struct sockaddr_storage addr;
	auto error = socket_udp_init(vmc->host, vmc->port, vmc->socket, addr);
	if (error)
		LOG(LIO, LError, "%s", error->c_str());
	else
	{
		address = getAddrString(&addr);
		LOG(LIO, LInfo, "Opened VMC server port %s!\n", address.c_str());
	}
	return !error.has_value();
}

static bool vmc_send(opaque_ptr<vmc_output> &vmc, osc::OutboundPacketStream &p)
{
	if (vmc->socket < 0) return false;
	int ret = send(vmc->socket, p.Data(), p.Size(), MSG_NOSIGNAL);
	if (ret < 0)
	{
#if defined(_WIN32)
		if (SOCKET_ERR_NUM == WSAECONNRESET)
#elif defined(__unix__)
		if (SOCKET_ERR_NUM == (int)std::errc::broken_pipe)
#endif
		{
			LOG(LIO, LWarn, "VMC socket broke!");
			socket_close(vmc->socket);
			vmc->socket = -1;
			return false;
		}
#if defined(_WIN32)
		if (SOCKET_ERR_NUM == WSAENETUNREACH)
#elif defined(__unix__)
		if (SOCKET_ERR_NUM == (int)std::errc::network_unreachable)
#endif
		{
			vmc->connected = false;
			return false;
		}
		LOG(LIO, LWarn, "VMC socket error on send: %s (%d)\n", SOCKET_ERR_STR, SOCKET_ERR_NUM);
		vmc->erroneous = true;
		return false;
	}
	vmc->erroneous = false;
	vmc->connected = true;
	return true;
}

static uint64_t toOSCTimeTag(std::chrono::system_clock::time_point tp)
{
	// Convert to NTP seconds since 1900 + fractions of a seconds
	auto dtNTP = tp.time_since_epoch() + std::chrono::days(70*365 + 17);
	auto dtSec = std::chrono::duration_cast<std::chrono::seconds>(dtNTP);
	auto dtSub = std::chrono::duration_cast<std::chrono::microseconds>(dtNTP - dtSec);

	// TODO: This is using 32bit seconds which will lapse in 2036 - but OSC demands it
	// A future receiver should be able to detect a 1900+ timestamp is likely lapsed
	uint32_t ntpSec = dtSec.count();
	uint32_t ntpFrac = (dtSub.count() << 32) / 1000000;

	// Assemble OSC timetag in NTP format
	uint64_t timetag;
	uint8_t *ttPtr = (uint8_t*)&timetag + sizeof(timetag);
	for (int i = 0; i < sizeof(timetag)/2; i++)
	{
        *--ttPtr = ntpSec & 0xFF;
        ntpSec >>= 8;
    }
	for (int i = sizeof(timetag)/2; i < sizeof(timetag); i++)
	{
        *--ttPtr = ntpFrac & 0xFF;
        ntpFrac >>= 8;
    }
	return timetag;
}

void vmc_send_device_packets(opaque_ptr<vmc_output> &vmc, const std::vector<vmc_device> &trackers, TimePoint_t timestamp, float deltaS)
{
	const int IP_MTU_SIZE = 1536;
	char buffer[IP_MTU_SIZE];
	osc::OutboundPacketStream s(buffer, IP_MTU_SIZE);

	// Rotate to Unity coordinate system used by VMC (also left-handed)
	Eigen::Isometry3f transform;
	transform.matrix()
	 << 1, 0, 0, 0,
		0, 0, 1, 0,
		0,-1, 0, 0,
		0, 0, 0, 1;
	Eigen::Isometry3f transformT;
	transformT.matrix() = transform.matrix().transpose();

	/* typename TimePoint_t::duration tolerance = std::chrono::nanoseconds(200);
	int limit = 5;
	auto ref_now = getAccurateClockReference<std::chrono::system_clock::time_point, TimePoint_t>(tolerance, limit);
	LOG(LIO, LInfo, "Converted timestamp from %.2fms ago with %ldns delta in %d iterations!",
		dtMS(timestamp, sclock::now()),
		std::chrono::duration_cast<std::chrono::nanoseconds>(tolerance).count(), limit);
	auto timestampSystem = convertClockWithRef(timestamp, ref_now); */

	auto timestampSystem = convertClockAccurate<std::chrono::system_clock::time_point>(timestamp);
	s << osc::BeginBundle(toOSCTimeTag(timestampSystem));
	s << osc::BeginMessage("/VMC/Ext/T") << deltaS << osc::EndMessage;
	for (auto &tracker : trackers)
	{
		assert((int)tracker.role >= 0 && tracker.role < VMCRole::MAX);

		s << osc::BeginMessage(deviceRoleMap[(int)tracker.role].c_str());
		s << tracker.serial.c_str();

		Eigen::Isometry3f pose = transform * tracker.pose * transformT;
		Eigen::Vector3f p = pose.translation();
		Eigen::Quaternionf q(pose.rotation());
		s << p.x() << p.y() << p.z();
		s << q.x() << q.y() << q.z() << q.w();
		if (tracker.role == VMCRole::Camera)
			s << tracker.fov;
		s << osc::EndMessage;
	}
	s << osc::EndBundle;

	vmc_send(vmc, s);
}