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

#include "wireless.hpp"

#include "util/util.hpp"
#include "util/system.hpp"

#include <thread>
#include <fstream>

void disconnectWifi()
{
	std::system("ifconfig wlan0 down");
	std::system("pkill wpa_supplicant");
	std::system("pkill udhcpc");
}

bool connectToWifi(std::string &SSID, std::string &IP, std::string &error)
{
	printf("Connecting to wifi...\n");

	std::string WPADRV;
	std::ifstream("/etc/sysconfig/wifi-wpadrv") >> WPADRV;
	printf("Read wifi-wpadrv as '%s'\n", WPADRV.c_str());

	bool hasDevice = std::system("iwconfig wlan0 2>&1 | grep -q 'No such device'") != 0;
	if (!hasDevice)
	{
		error = "Device wlan0 not set up!";
		printf("%s\n", error.c_str());
		return false;
	}

	std::string hostname = exec("hostname -s");
	printf("Read hostname as '%s'\n", hostname.c_str());
	fflush(stdout);

	// Disconnect first
	disconnectWifi();

	int ret = std::system("wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf -B");
	if (ret)
	{
		error = asprintf_s("Failed to parse wpa_supplicant.conf! Error %d", ret);
		printf("%s\n", error.c_str());
		return false;
	}
	printf("Triggered wpa_supplicant update!\n");
	fflush(stdout);

	bool associated = false;
	for (int i = 0; i < 40; i++)
	{
		associated = std::system("iwconfig wlan0 | grep -q Not-Associated") != 0;
		if (associated) break;
		printf(".");
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	printf("\n");
	if (!associated)
	{
		error = asprintf_s("Failed to associate with network!");
		printf("%s\n", error.c_str());
		return false;
	}
	fflush(stdout);

	SSID = exec("iwconfig wlan0 | awk '/ESSID/ {gsub(\"ESSID:\", \"\", $4); gsub(/\"/, \"\", $4); print $4} '");
	if (SSID.size() > 32) // Should not happen, not allowed
		printf("Associated with wifi '%s', too long for real SSID!\n", SSID.c_str());
	else
		printf("Associated with wifi '%s'!\n", SSID.c_str());

	printf("Requesting IP from AP via DHCP!\n");
	fflush(stdout);
	ret = std::system(asprintf_s("udhcpc -n -i wlan0 -x hostname:%s -F %s 2>&1 > /dev/null", hostname.c_str(), hostname.c_str()).c_str());
	if (ret)
	{
		error = asprintf_s("Failed to request IP from AP!");
		printf("%s\n", error.c_str());
		return false;
	}

	IP = exec("ifconfig wlan0 | awk '/inet addr/ {gsub(\"addr:\", \"\", $2); print $2}'");
	printf("Got IP address %s!\n", IP.c_str());

	std::system("/usr/local/etc/init.d/openssh start");

	return true;
}

void fillWirelessStatusPacket(const TrackingCameraState &state, std::vector<uint8_t> &packet)
{
	packet.resize(state.wireless.connected? 4 : (state.wireless.failed? 3 : 2));
	packet[0] = state.wireless.connected? 2 : (state.wireless.failed? 1 : 0);
	packet[1] = state.server.enabled? (state.server.ready? 2 : 1) : 0;
	if (state.wireless.connected)
	{
		uint8_t SSIDLen = state.wireless.SSID.size() > 32? 32 : state.wireless.SSID.size();
		uint8_t IPLen = state.wireless.IP.size() > 15? 15 : state.wireless.IP.size(); // 4*3+3 = 15
		packet[2] = SSIDLen;
		packet[3] = IPLen;
		packet.reserve(2+2+SSIDLen+IPLen); // No string null-termination
		packet.insert(packet.end(), state.wireless.SSID.begin(), state.wireless.SSID.begin()+SSIDLen);
		packet.insert(packet.end(), state.wireless.IP.begin(), state.wireless.IP.begin()+IPLen);

		printf("Sending wireless status packet of size %d, with %d bytes SSID and %d bytes IP!\n", (int)packet.size(), SSIDLen, IPLen);
	}
	else if (state.wireless.failed)
	{
		uint8_t errorLen = state.wireless.error.size() > 128? 128 : state.wireless.error.size();
		packet[2] = errorLen;
		packet.reserve(2+1+errorLen); // No string null-termination
		packet.insert(packet.end(), state.wireless.error.begin(), state.wireless.error.begin()+errorLen);

		printf("Sending wireless status packet of size %d, with %d bytes error string!\n", (int)packet.size(), errorLen);
	}
	else
	{
		printf("Sending wireless status packet of size %d!\n", (int)packet.size());
	}
}

void UpdateWirelessStateThread(TrackingCameraState *state)
{
	auto &wireless = state->wireless;
	wireless.updating = true; // Should already be set by spawner
	while (wireless.dirty)
	{
		wireless.dirty = false; // could do atomic exchange but don't expect this to change often

		if (wireless.enabled)
		{
			std::string SSID, IP, error;
			wireless.connected = connectToWifi(SSID, IP, error);
			if (wireless.connected)
			{
				wireless.SSID = std::move(SSID);
				wireless.IP = std::move(IP);
				wireless.failed = false;
			}
			else
			{
				wireless.error = std::move(error);
				wireless.failed = true;
				continue;
			}
		}
		else
		{
			disconnectWifi();
			wireless.failed = false;
			wireless.connected = false;
			continue;
		}

		assert(wireless.connected);

		// TODO: Explicitly start/stop server
		if (wireless.Server && !state->server.enabled)
		{
			/* serverThread = new std::thread(CommThread, &state.server, &state);
			comms.push_back(&state.server);
			printf("Initiating server connection...\n"); */
		}
		else if (!wireless.Server && state->server.enabled)
		{
			/* state.server.enabled = false;
			if (serverThread && serverThread->joinable())
				serverThread->join();
			delete serverThread; */
		}

		printf("Successfully connected, checking for new packet...\n");
	}

	// Tell comm threads to not rely on this thread, dirty will not be checked anymore
	// if they need to update wirelessm, they need to spawn a new thread
	wireless.updating = false;

	// Tell main thread to send a wireless status packet
	// Safe to do here, but don't want to send a packet right before streaming packets are sent
	wireless.needsStatusPacket = true;

	printf("Done, requesting wireless status packet!\n");
}