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
#include "../state.hpp"

#include "util/util.hpp"
#include "util/system.hpp"

#include <thread>
#include <fstream>
#include <filesystem>

static const std::filesystem::path configPath("/mnt/mmcblk0p2/config");

static bool hasWifiHardware()
{
	return std::system("iwconfig wlan0 2>&1 | grep -q 'No such device'") != 0;
}

static bool isWifiSetup()
{
	return std::filesystem::exists("/usr/local/sbin/wpa_supplicant");
}

static bool isWifiBlocked()
{
	return std::filesystem::exists(configPath / "disallow_wifi");
}

static bool isWifiEnabled()
{
	return std::system("pidof wpa_supplicant") == 0;
}

static bool isWifiConnected()
{
	return std::system("iwconfig wlan0 | grep -q Not-Associated") != 0;
}

static bool reassignIP()
{
	return std::system("udhcpc -n -i wlan0 -x hostname:$(hostname -s) -F $(hostname -s) 2>&1 > /dev/null") == 0;
}

static void disconnectWifi()
{
	std::system("pkill wpa_supplicant");
	std::system("pkill udhcpc");
	std::system("ifconfig wlan0 down");
}

static bool reconnectWifi(std::string &error)
{
	printf("Connecting to wifi...\n");

	std::string WPADRV;
	std::ifstream("/etc/sysconfig/wifi-wpadrv") >> WPADRV;
	printf("Read wifi-wpadrv as '%s'\n", WPADRV.c_str());

	// Disconnect first
	disconnectWifi();

	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	int ret = std::system("wpa_supplicant -i wlan0 -c /etc/wpa_supplicant.conf -B");
	if (ret)
	{
		error = asprintf_s("Failed to parse wpa_supplicant.conf! Error %d", ret);
		printf("%s\n", error.c_str());
		return false;
	}
	printf("Triggered wpa_supplicant update!\n");
	return true;
}

static void getWirelessStatus(std::string &SSID, std::string &IP)
{
	SSID = exec("iwgetid -r");
	if (SSID.size() > 32) // Should not happen, not allowed
		printf("Associated with wifi '%s', too long for real SSID!\n", SSID.c_str());
	else
		printf("Associated with wifi '%s'!\n", SSID.c_str());

	std::string IPs = exec("awk '/32 host/ { print i } {i=$2}' /proc/net/fib_trie");

	std::stringstream ss(IPs);
    while (std::getline(ss, IP, '\n'))
	{
		if (IP.compare("127.0.0.1") != 0)
			break;
	}
}

static void WirelessMonitorThread(TrackingCameraState *state);

void initWirelessMonitor(TrackingCameraState &state)
{
	if (!hasWifiHardware()) return;
	if (!isWifiSetup()) return;
	if (isWifiBlocked()) return;

	if (isWifiEnabled())
		state.wireless.config = (WirelessConfig)(WIRELESS_CONFIG_WIFI | WIRELESS_CONFIG_SSH);

	state.wireless.monitor = true;
	if (!state.wireless.monitorThread)
		state.wireless.monitorThread = new std::thread(WirelessMonitorThread, &state);
}

void stopWirelessMonitor(TrackingCameraState &state)
{
	state.wireless.monitor = false;
	if (state.wireless.monitorThread && state.wireless.monitorThread->joinable())
		state.wireless.monitorThread->join();
}

void fillWirelessStatusPacket(TrackingCameraState &state, std::vector<uint8_t> &packet)
{
	WirelessState &wireless = state.wireless;
	std::unique_lock lock(wireless.mutex);

	uint8_t errorLen = std::min<std::size_t>(255, wireless.error.size());
	uint8_t SSIDLen = 0, IPLen = 0;
	if (wireless.wifi == WIRELESS_STATUS_CONNECTED)
	{
		SSIDLen = std::min<std::size_t>(255, wireless.SSID.size());
		IPLen = std::min<std::size_t>(15, wireless.IP.size());
	}

	packet.resize(WIRELESS_PACKET_HEADER + errorLen + SSIDLen + IPLen);
	packet[0] = wireless.wifi;
	packet[1] = wireless.ssh;
	packet[2] = wireless.server;
	// 2 free byte for future use
	packet[5] = errorLen;
	packet[6] = SSIDLen;
	packet[7] = IPLen;

	uint8_t *ptr = packet.data() + WIRELESS_PACKET_HEADER;
	if (errorLen > 0)
		memcpy(ptr, wireless.error.data(), errorLen);
	ptr += errorLen;
	if (SSIDLen > 0)
		memcpy(ptr, wireless.SSID.data(), SSIDLen);
	ptr += SSIDLen;
	if (IPLen > 0)
		memcpy(ptr, wireless.IP.data(), IPLen);

	wireless.error.clear();
}

bool parseWirelessConfigPacket(TrackingCameraState &state, uint8_t *packet, uint16_t length)
{
	printf("Received wifi configuration packet of size %d\n", length);
	WirelessState &wireless = state.wireless;
	if (length < 8) return false;
	WirelessConfig config = (WirelessConfig)packet[0];
	WirelessActions actions = (WirelessActions)packet[1];
	uint16_t credSize = (packet[2] << 8) | packet[3];
	// 4 free bytes for future use
	if (length < WIRELESS_PACKET_HEADER+credSize)
	{
		printf("Received wifi configuration with %d additional bytes of wpa_supplicant, but packet size was only %d\n",
			credSize, length);
		wireless.error = "Invalid packet!";
		wireless.lastStatus = sclock::now();
		wireless.sendStatus = true;
		return false;
	}

	std::unique_lock lock(wireless.mutex);

	// Allow some actions to configure the image even when wifi is blocked for some reason

	if (credSize > 0)
	{
		printf("Received wpa_supplicant of size %d!\n", credSize);
		std::string wpa_supplicant((char*)&packet[WIRELESS_PACKET_HEADER], credSize);
		// Format check is done by wpa_supplicant itself, cannot do everything
		std::ofstream("/etc/wpa_supplicant.conf") << wpa_supplicant;
	}

	if (actions & WIRELESS_STORE_CREDS)
	{
		std::filesystem::copy("/etc/wpa_supplicant.conf", configPath / "wpa_supplicant.conf");
	}

	if (actions & WIRELESS_CLEAR_CREDS)
	{
		std::filesystem::remove(configPath / "wpa_supplicant.conf");
	}

	if (actions & WIRELESS_ENABLE_AUTOCONNECT)
	{
		std::ofstream(configPath / "wireless_autoconnect");
	}

	if (actions & WIRELESS_DISABLE_AUTOCONNECT)
	{
		std::filesystem::remove(configPath / "wireless_autoconnect");
	}

	// Check if wifi is blocked / unavailable

	if (!hasWifiHardware())
	{
		wireless.error = "No Wifi hardware!";
		wireless.wifi = wireless.ssh = wireless.server = WIRELESS_STATUS_ERROR;
		wireless.lastStatus = sclock::now();
		wireless.sendStatus = true;
		return false;
	}
	if (!isWifiSetup())
	{
		wireless.error = "Non-Wifi image!";
		wireless.wifi = wireless.ssh = wireless.server = WIRELESS_STATUS_ERROR;
		wireless.lastStatus = sclock::now();
		wireless.sendStatus = true;
		return false;
	}
	if (isWifiBlocked())
	{
		wireless.error = "Wifi is blocked!";
		wireless.wifi = wireless.ssh = wireless.server = WIRELESS_STATUS_ERROR;
		wireless.lastStatus = sclock::now();
		wireless.sendStatus = true;
		return false;
	}

	// Only set blocked status now to allow final configuration change

	if (actions & WIRELESS_DISALLOW_WIFI_PERMANENTLY)
	{
		std::ofstream(configPath / "disallow_wifi");
	}

	if (actions & WIRELESS_DISALLOW_SSH_PERMANENTLY)
	{
		std::ofstream(configPath / "disallow_ssh");
	}

	if (wireless.config != config)
	{ // Apply configuration change
		wireless.changed = (WirelessConfig)(wireless.config ^ config);
		if (credSize > 0)
			wireless.changed = (WirelessConfig)(wireless.changed | WIRELESS_CONFIG_WIFI);
		wireless.config = config;
		wireless.updateConfig = true;
		printf("Received packet of size %d that set wireless state to %s!\n",
			length, config & WIRELESS_CONFIG_WIFI? "enabled" : "disabled");
	}

	return true;
}

static void WirelessMonitorThread(TrackingCameraState *state)
{
	WirelessState &wireless = state->wireless;
	bool sendStatus = true;
	bool needIP = false;
	wireless.lastStatus = sclock::now();
	while (wireless.monitor)
	{
		if (wireless.changed)
		{
			std::unique_lock lock(wireless.mutex);

			if (wireless.changed & WIRELESS_CONFIG_WIFI)
			{
				if (wireless.config & WIRELESS_CONFIG_WIFI)
				{
					if (reconnectWifi(wireless.error))
						wireless.wifi = WIRELESS_STATUS_ENABLED;
					else
						wireless.wifi = WIRELESS_STATUS_ERROR;
					needIP = true;
				}
				else if (!(wireless.config & WIRELESS_CONFIG_WIFI))
				{
					disconnectWifi();
					wireless.wifi = WIRELESS_STATUS_DISABLED;
				}
			}

			if (wireless.config & WIRELESS_CONFIG_SSH)
			{
				if (!std::filesystem::exists(configPath / "disallow_ssh"))
				{
					std::system("/usr/local/etc/init.d/openssh start");
					wireless.ssh = WIRELESS_STATUS_ENABLED;
				}
				else
					wireless.ssh = WIRELESS_STATUS_ERROR;
			}
			else
			{
				std::system("/usr/local/etc/init.d/openssh stop");
				wireless.ssh = WIRELESS_STATUS_DISABLED;
			}

			// TODO: Explicitly start/stop server
			wireless.server = WIRELESS_STATUS_DISABLED;
			if ((wireless.config & WIRELESS_CONFIG_SERVER) && !state->server.enabled)
			{
				/* serverThread = new std::thread(CommThread, &state.server, &state);
				comms.push_back(&state.server);
				printf("Initiating server connection...\n"); */
			}
			else if (!(wireless.config & WIRELESS_CONFIG_SERVER) && state->server.enabled)
			{
				/* state.server.enabled = false;
				if (serverThread && serverThread->joinable())
					serverThread->join();
				delete serverThread; */
			}

			wireless.changed = WIRELESS_CONFIG_NONE;
			sendStatus = true;
		}

		if ((wireless.config & WIRELESS_CONFIG_WIFI) && wireless.wifi != WIRELESS_STATUS_CONNECTED &&
			dtMS(wireless.lastCheck, sclock::now()) > 500)
		{
			wireless.lastCheck = sclock::now();
			if (isWifiConnected())
			{
				wireless.wifi = WIRELESS_STATUS_CONNECTED;
				if (needIP)	reassignIP(); // May block for a second or so
				needIP = false;
				getWirelessStatus(wireless.SSID, wireless.IP);
				sendStatus = true;
			}
		}

		if (sendStatus || dtMS(wireless.lastStatus, sclock::now()) > 1000)
		{
			// Tell main thread to send a wireless status packet
			// Safe to do here, but don't want to send a packet right before streaming packets are sent
			wireless.lastStatus = sclock::now();
			wireless.sendStatus = true;
			sendStatus = false;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	printf("Done, requesting wireless status packet!\n");
}