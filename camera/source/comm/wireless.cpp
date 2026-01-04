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
#include "comm/comm.hpp"
#include "comm/server.hpp"

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

static bool hasWifiDrivers()
{
	return std::filesystem::exists("/usr/local/sbin/wpa_supplicant");
}

static bool hasWifiCredentials()
{
	return std::filesystem::exists("/etc/wpa_supplicant.conf");
}

static bool hasWifiCredentialsStored()
{
	return std::filesystem::exists(configPath / "wpa_supplicant.conf");
}

static bool hasServerConfiguration()
{
	return std::filesystem::exists(configPath / "server.conf");
}

static bool isWifiAutoConnecting()
{
	return std::filesystem::exists(configPath / "wireless_autoconnect");
}

static bool isSSHAutoEnabled()
{
	return std::filesystem::exists(configPath / "enable_ssh");
}

static bool isServerAutoEnabled()
{
	return std::filesystem::exists(configPath / "enable_server");
}

static bool isWifiBlocked()
{
	return std::filesystem::exists(configPath / "disallow_wifi");
}

static bool isSSHBlocked()
{
	return std::filesystem::exists(configPath / "disallow_ssh");
}

static bool realtimeDataUseServer()
{
	return std::filesystem::exists(configPath / "server_realtime_data");
}

static bool isWifiEnabled()
{
	return std::system("pidof wpa_supplicant") == 0;
}

static bool isSSHEnabled()
{
	return std::system("pidof sshd") == 0;
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
	std::system("pkill wpa_supplicant & pkill udhcpc & ifconfig wlan0 down");
}

static bool reconnectWifi(std::string &error)
{
	printf("Connecting to wifi...\n");

	/* std::string WPADRV;
	std::ifstream("/etc/sysconfig/wifi-wpadrv") >> WPADRV;
	printf("Read wifi-wpadrv as '%s'\n", WPADRV.c_str()); */

	// Disconnect first
	disconnectWifi();

	std::this_thread::sleep_for(std::chrono::milliseconds(200));

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

static bool getWirelessStatus(std::string &SSID, std::string &IP)
{
	SSID = exec("iwgetid -r");
	if (SSID.empty())
	{
		printf("Failed to read SSID!\n");
		return false;
	}
	else if (SSID.size() > 32) // Should not happen, not allowed
	{
		printf("Associated with wifi '%s', too long for real SSID!\n", SSID.c_str());
		return false;
	}
	else
		printf("Associated with wifi '%s'!\n", SSID.c_str());

	std::string IPs = exec("awk '/32 host/ { print i } {i=$2}' /proc/net/fib_trie");

	std::stringstream ss(IPs);
	while (std::getline(ss, IP, '\n'))
	{
		if (IP.compare("127.0.0.1") != 0)
			return true;
	}

	printf("Failed to get own IP!\n");
	return false;
}

static void WirelessMonitorThread(TrackingCameraState *state);

void startWirelessMonitor(TrackingCameraState &state)
{
	state.wireless.monitor = true;
	if (!state.wireless.monitorThread)
		state.wireless.monitorThread = new std::thread(WirelessMonitorThread, &state);
	else printf("Wanting to start wireless monitor when it's already started!\n");
	state.wireless.lastCheck = sclock::now() - std::chrono::milliseconds(1000);
}

void stopWirelessMonitor(TrackingCameraState &state)
{
	state.wireless.monitor = false;
	if (state.wireless.monitorThread && state.wireless.monitorThread->joinable())
		state.wireless.monitorThread->join();
	state.wireless.monitorThread = nullptr;
}

void initWirelessMonitor(TrackingCameraState &state)
{
	state.wireless.config = WIRELESS_CONFIG_NONE;
	state.wireless.wifi = WIRELESS_STATUS_NONE;
	state.wireless.ssh = WIRELESS_STATUS_NONE;
	state.wireless.server = WIRELESS_STATUS_NONE;
	if (!hasWifiHardware()) return;
	if (!hasWifiDrivers()) return;
	if (isWifiBlocked()) return;

	if (hasServerConfiguration())
	{
		std::ifstream serverAddrFile(configPath / "server.conf");
		std::getline(serverAddrFile, state.wireless.serverHost);
		std::getline(serverAddrFile, state.wireless.serverPort);
	}

	// NOTE: We can either accept the current state of wifi and ssh as config
		// e.g. program crashed, but server on host PC previously configured wifi & ssh temporarily
	// OR we can respect autostart config exactly, and ignore what was previously set up
		// e.g. disable wifi & ssh when it was enabled but is configured to not be
	// Currently decided to just observe the current state and accept that as the current desired config
		// This only affects wifi and ssh, server and rtdata config have no persistent system state

	if (isWifiEnabled())
	{ // Either startup scripts (autoconnect) or a previous program instance have enabled wifi already
		// Wireless monitor will check if it's connected and get IP
		state.wireless.config = (WirelessConfig)(state.wireless.config | WIRELESS_CONFIG_WIFI);
		state.wireless.wifi = WIRELESS_STATUS_ENABLED;
		printf("Wireless was enabled on startup!\n");

		if (isSSHEnabled())
		{ // Either startup scripts (autoenable) or a previous program instance have enabled ssh already
			state.wireless.config = (WirelessConfig)(state.wireless.config | WIRELESS_CONFIG_SSH);
			state.wireless.ssh = WIRELESS_STATUS_ENABLED;
			printf("SSH is enabled!\n");
		}
		else state.wireless.ssh = WIRELESS_STATUS_DISABLED;

		if (isServerAutoEnabled() && hasServerConfiguration())
		{ // Scripts don't control the server, so just follow autoenable configuration
			state.wireless.config = (WirelessConfig)(state.wireless.config | WIRELESS_CONFIG_SERVER);
			state.wireless.server = WIRELESS_STATUS_ENABLED;
			printf("Initiating server connection to %s:%s...\n", state.wireless.serverHost.c_str(), state.wireless.serverPort.c_str());
			state.server.port = server_init(state.wireless.serverHost, state.wireless.serverPort);
			comm_enable(state.server, &state, COMM_MEDIUM_WIFI);

			if (realtimeDataUseServer())
			{
				state.wireless.config = (WirelessConfig)(state.wireless.config | WIRELESS_CONFIG_RTDATA);
				realTimeAff = COMM_MEDIUM_WIFI;
			}
			else
				realTimeAff = COMM_MEDIUM_UART;
		}
		else state.wireless.server = WIRELESS_STATUS_DISABLED;

		// Start wireless monitor if wifi is already enabled, else start on-demand
		startWirelessMonitor(state);
	}
	else
	{
		printf("Wireless was disabled on startup!\n");
		state.wireless.wifi = WIRELESS_STATUS_DISABLED;
	}
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
	packet[0] = wireless.config;
	packet[1] = wireless.wifi | wireless.wifiFlags;
	packet[2] = wireless.ssh | wireless.sshFlags;
	packet[3] = wireless.server | wireless.serverFlags;
	// 1 free byte for future use
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
	ptr += IPLen;

	wireless.error.clear();
}

bool parseWirelessConfigPacket(TrackingCameraState &state, const uint8_t *packet, uint16_t length)
{
	WirelessState &wireless = state.wireless;
	if (length < 8) return false;
	WirelessConfig config = (WirelessConfig)packet[0];
	uint8_t actionLength = packet[1];
	uint16_t credSize = (packet[2] << 8) | packet[3];
	uint8_t serverAddrSize = packet[4];
	// 3 free bytes for future use
	if (length < WIRELESS_PACKET_HEADER+actionLength+credSize+serverAddrSize)
	{
		printf("Received wifi configuration with %d actions, %d bytes of wpa_supplicant, %d bytes of server address, but packet size was only %d\n",
			actionLength, credSize, serverAddrSize, length);
		wireless.error = "Invalid packet!";
		wireless.lastStatus = sclock::now();
		wireless.sendStatus = true;
		return false;
	}

	printf("Parsing wifi configuration packet of size %d\n", length);
	std::unique_lock lock(wireless.mutex);
	WirelessConfig changed = WIRELESS_CONFIG_NONE;

	// Allow some actions to configure the image even when wifi is blocked for some reason

	if (credSize > 0)
	{
		printf("Received wpa_supplicant of size %d!\n", credSize);
		std::string wpa_supplicant((char*)&packet[WIRELESS_PACKET_HEADER+actionLength], credSize);
		// Format check is done by wpa_supplicant itself, cannot do everything
		// Write to RAM disk to ensure any automatic conversion is applied just the same
		std::ofstream("/etc/wpa_supplicant.conf.new") << wpa_supplicant;
		// Then compare
		std::ifstream wpa_supplicant_old("/etc/wpa_supplicant.conf");
		std::ifstream wpa_supplicant_new("/etc/wpa_supplicant.conf.new");
		std::stringstream wpas_old, wpas_new;
		wpas_old << wpa_supplicant_old.rdbuf();
		wpas_new << wpa_supplicant_new.rdbuf();
		if (wpas_old.str() == wpas_new.str())
		{
			std::filesystem::remove("/etc/wpa_supplicant.conf.new");
		}
		else
		{ // Apply and ensure wifi is re-connected
			printf("    Updating wifi since wpa_supplicant changed!\n");
			std::filesystem::remove("/etc/wpa_supplicant.conf");
			std::filesystem::rename("/etc/wpa_supplicant.conf.new", "/etc/wpa_supplicant.conf");
			changed = (WirelessConfig)(changed | WIRELESS_CONFIG_WIFI);
		}
		sync();
	}

	if (serverAddrSize > 0)
	{
		std::string_view serverAddr((char*)&packet[WIRELESS_PACKET_HEADER+actionLength+credSize], serverAddrSize);
		std::size_t split = serverAddr.find(':');
		std::string host(serverAddr.substr(0, split));
		std::string port(split != std::string::npos? serverAddr.substr(split+1) : "48532");
		if (host != wireless.serverHost || port != wireless.serverPort)
		{
			if (config & WIRELESS_CONFIG_SERVER)
				changed = (WirelessConfig)(changed | WIRELESS_CONFIG_SERVER);
			wireless.serverHost = host;
			wireless.serverPort = port;
		}
	}

	for (int i = 0; i < actionLength; i++)
	{
		switch((WirelessAction)packet[WIRELESS_PACKET_HEADER+i])
		{
			case WIRELESS_ACTION_NONE:
				break;
			case WIRELESS_STORE_CONFIG:
				printf("Making wireless configuration persistent!\n");
				if (config & WIRELESS_CONFIG_WIFI)
				{
					std::filesystem::copy("/etc/wpa_supplicant.conf", configPath / "wpa_supplicant.conf", std::filesystem::copy_options::overwrite_existing);
					std::ofstream(configPath / "wireless_autoconnect");
				}
				else
				{ // Might still want to keep credentials, require clear creds to delete them
					std::filesystem::remove(configPath / "wireless_autoconnect");
				}
				if (config & WIRELESS_CONFIG_SSH)
					std::ofstream(configPath / "enable_ssh");
				else
					std::filesystem::remove(configPath / "enable_ssh");
				if (config & WIRELESS_CONFIG_SERVER)
				{
					std::ofstream(configPath / "server.conf") << wireless.serverHost << std::endl << wireless.serverPort;
					std::ofstream(configPath / "enable_server");
				}
				else
				{ // No need to retain creds, will be send next time
					std::filesystem::remove(configPath / "server.conf");
					std::filesystem::remove(configPath / "enable_server");
				}
				if (config & WIRELESS_CONFIG_RTDATA)
					std::ofstream(configPath / "server_realtime_data");
				else
					std::filesystem::remove(configPath / "server_realtime_data");
				sync();
				break;
			case WIRELESS_CLEAR_CREDS:
				printf("Clearing wireless credentials!\n");
				if (credSize == 0)
				{ // If not still supplying credentials, also delete the temporary ones
					std::filesystem::remove("/etc/wpa_supplicant.conf");
				}
				std::filesystem::remove(configPath / "wpa_supplicant.conf");
				sync();
				break;
			case WIRELESS_DISALLOW_WIFI_PERMANENTLY:
				printf("Set persistent configuration to disallow wifi!\n");
				config = (WirelessConfig)(config & ~WIRELESS_CONFIG_WIFI);
				std::ofstream(configPath / "disallow_wifi");
				// Also delete credentials
				std::filesystem::remove("/etc/wpa_supplicant.conf");
				std::filesystem::remove(configPath / "wpa_supplicant.conf");
				sync();
				break;
			case WIRELESS_DISALLOW_SSH_PERMANENTLY:
				printf("Set persistent configuration to disallow SSH!\n");
				config = (WirelessConfig)(config & ~WIRELESS_CONFIG_SSH);
				std::ofstream(configPath / "disallow_ssh");
				sync();
				break;
		}
	}

	// Check if wifi is blocked / unavailable

	if (!state.wireless.monitorThread)
	{ // Wifi had not been monitored yet - check if there is a reason or if we can start monitoring it now
		// This is just to return an error quickly before starting, thread still does actual checks
		wireless.error.clear();
		if (!hasWifiHardware())
			wireless.error = "No Wifi hardware!";
		else if (!hasWifiDrivers())
			wireless.error = "No Wifi drivers!";
		else if (isWifiBlocked())
			wireless.error = "Wifi is blocked!";

		if (wireless.error.empty())
		{ // No issue starting wireless
			startWirelessMonitor(state);
		}
		else
		{ // There's a reason we can not start the wifi thread
			printf("Failed to setup wifi: '%s'\n", wireless.error.c_str());
			wireless.wifi = wireless.ssh = wireless.server = WIRELESS_STATUS_ERROR;
			wireless.lastStatus = sclock::now();
			wireless.sendStatus = true;
			return false;
		}
	}

	changed = (WirelessConfig)(changed | (wireless.config ^ config));
	if (changed)
	{ // Apply configuration change (not vetted yet)
		wireless.config = config;
		wireless.changed = changed;
	}

	return true;
}

static void WirelessMonitorThread(TrackingCameraState *state)
{
	WirelessState &wireless = state->wireless;
	bool sendStatus = true;
	bool needIP = false;
	bool waitForNetwork = false;
	printf("Starting wireless monitor...\n");
	wireless.lastStatus = sclock::now();
	while (wireless.monitor)
	{
		int changed = wireless.changed;
		int config = wireless.config;
		wireless.changed = WIRELESS_CONFIG_NONE;
		if (changed)
		{
			printf("Updating wireless configuration...\n");
			std::unique_lock lock(wireless.mutex);

			if (changed & WIRELESS_CONFIG_WIFI)
			{
				if (config & WIRELESS_CONFIG_WIFI)
				{
					wireless.wifi = WIRELESS_STATUS_ERROR;
					if (!hasWifiHardware())
						wireless.error = "No Wifi hardware!";
					else if (!hasWifiDrivers())
						wireless.error = "No Wifi drivers!";
					else if (isWifiBlocked())
						wireless.error = "Wifi is blocked!";
					else if (!hasWifiCredentials())
						wireless.error = "No Wifi creds!";
					else if (reconnectWifi(wireless.error))
						wireless.wifi = WIRELESS_STATUS_ENABLED;

					if (wireless.wifi == WIRELESS_STATUS_ERROR)
					{
						printf("Failed to setup wifi: '%s'\n", wireless.error.c_str());
						disconnectWifi();
						wireless.config = (WirelessConfig)(wireless.config & ~WIRELESS_CONFIG_WIFI); 
					}
					else needIP = true;
				}
				else
				{
					printf("Disconnecting Wifi!\n");
					disconnectWifi();
					wireless.config = (WirelessConfig)(wireless.config & ~WIRELESS_CONFIG_WIFI);
					wireless.wifi = WIRELESS_STATUS_DISABLED;
				}
			}
			bool hasWifi = wireless.wifi == WIRELESS_STATUS_ENABLED || wireless.wifi == WIRELESS_STATUS_CONNECTED;

			if (hasWifi && (config & WIRELESS_CONFIG_SSH) && !isSSHBlocked())
			{
				if (!isSSHEnabled())
				{
					printf("Enabling SSH...\n");
					std::system("/usr/local/etc/init.d/openssh start");
				}
				wireless.ssh = WIRELESS_STATUS_ENABLED;
			}
			else
			{
				if (isSSHEnabled())
				{
					printf("Disabling SSH...\n");
					std::system("/usr/local/etc/init.d/openssh stop");
				}
				wireless.config = (WirelessConfig)(wireless.config & ~WIRELESS_CONFIG_SSH);
				if (!(config & WIRELESS_CONFIG_SSH))
					wireless.ssh = WIRELESS_STATUS_DISABLED;
				else if (!hasWifi)
					wireless.ssh = WIRELESS_STATUS_UNABLE;
				else // Blocked
					wireless.ssh = WIRELESS_STATUS_ERROR;
			}

			if (hasWifi && (config & WIRELESS_CONFIG_SERVER))
			{
				if (state->server.enabled && (changed & WIRELESS_CONFIG_SERVER))
				{
					printf("Restarting server connection...\n");
					comm_disable(state->server);
					server_deinit(state->server.port);
				}
				if (!state->server.enabled)
				{
					printf("Initiating server connection...\n");
					state->server.port = server_init(wireless.serverHost, wireless.serverPort);
					comm_enable(state->server, state, COMM_MEDIUM_WIFI);
				}
				wireless.server = WIRELESS_STATUS_ENABLED;
			}
			else
			{
				if (state->server.enabled)
				{
					printf("Stopping server connection...\n");
					comm_disable(state->server);
					server_deinit(state->server.port);
				}
				wireless.config = (WirelessConfig)(wireless.config & ~WIRELESS_CONFIG_SERVER);
				if (!(config & WIRELESS_CONFIG_SERVER))
					wireless.server = WIRELESS_STATUS_DISABLED;
				else if (!hasWifi)
					wireless.server = WIRELESS_STATUS_UNABLE;
				else // Should not happen
					wireless.server = WIRELESS_STATUS_ERROR;
			}

			if (hasWifi && state->server.enabled && (config & WIRELESS_CONFIG_RTDATA))
				realTimeAff = COMM_MEDIUM_WIFI;
			else
			{
				wireless.config = (WirelessConfig)(wireless.config & ~WIRELESS_CONFIG_RTDATA);
				realTimeAff = COMM_MEDIUM_UART;
			}

			sendStatus = true;
		}

		if (state->wireless.wifi == WIRELESS_STATUS_ENABLED
			&& dtMS(wireless.lastCheck, sclock::now()) > 500)
		{
			wireless.lastCheck = sclock::now();
			if (isWifiConnected())
			{
				printf("Wifi has connected!\n");
				wireless.wifi = WIRELESS_STATUS_CONNECTED;
				if (needIP)	reassignIP(); // May block for a second or so
				needIP = false;
				waitForNetwork = true;
			}
			else
				printf("Wifi waiting to be connected...\n");
		}

		if ((state->wireless.wifi == WIRELESS_STATUS_CONNECTED && waitForNetwork)
			&& dtMS(wireless.lastCheck, sclock::now()) > 500)
		{
			wireless.lastCheck = sclock::now();
			if (getWirelessStatus(wireless.SSID, wireless.IP))
			{
				waitForNetwork = false;
				sendStatus = true;
			}
		}

		if (state->wireless.wifi == WIRELESS_STATUS_CONNECTED && !waitForNetwork
			&& dtMS(wireless.lastCheck, sclock::now()) > 1000)
		{
			wireless.lastCheck = sclock::now();
			if (!isWifiConnected())
			{
				printf("Lost wifi connection!\n");
				state->wireless.wifi = WIRELESS_STATUS_ENABLED;
				sendStatus = true;
			}
		}

		if (sendStatus || dtMS(wireless.lastStatus, sclock::now()) > 1000)
		{
			wireless.wifiFlags = wireless.sshFlags = wireless.serverFlags = WIRELESS_STATUS_NONE;
			if (isWifiAutoConnecting()) wireless.wifiFlags = (WirelessStatus)(wireless.wifiFlags | WIRELESS_STATUS_AUTOENABLE);
			if (hasWifiCredentialsStored()) wireless.wifiFlags = (WirelessStatus)(wireless.wifiFlags | WIRELESS_STATUS_CONFIGURATED);
			if (isSSHAutoEnabled()) wireless.sshFlags = (WirelessStatus)(wireless.sshFlags | WIRELESS_STATUS_AUTOENABLE);
			if (isServerAutoEnabled()) wireless.serverFlags = (WirelessStatus)(wireless.serverFlags | WIRELESS_STATUS_AUTOENABLE);
			if (hasServerConfiguration()) wireless.serverFlags = (WirelessStatus)(wireless.serverFlags | WIRELESS_STATUS_CONFIGURATED);
			// Tell main thread to send a wireless status packet
			// Safe to do here, but don't want to send a packet right before streaming packets are sent
			wireless.lastStatus = sclock::now();
			wireless.sendStatus = true;
			sendStatus = false;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	printf("Stopping wireless monitor!\n");
	wireless.monitor = false;
}