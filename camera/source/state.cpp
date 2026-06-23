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

#include <getopt.h>
#include <unistd.h>
#include <string>
#include <cstdio>
#include <cstring>

#include "state.hpp"

bool options_read(TrackingCameraState &state, int argc, char **argv)
{
	const struct option long_options[] = {
		{"h",					no_argument,		0,	1 },
		{"help",				no_argument,		0,	1 },
		{"r",					required_argument,	0,	2 },
		{"res",					required_argument,	0,	2 },
		{"f",					required_argument,	0,	3 },
		{"fps",					required_argument,	0,	3 },
		{"ss",					required_argument,	0,	4 },
		{"shutter-speed",		required_argument,	0,	4 },
		{"g",					required_argument,	0,	5 },
		{"gain",				required_argument,	0,	5 },
		{"dg",					required_argument,	0,	6 },
		{"digital-gain",		required_argument,	0,	6 },
		{"nostrobe",			no_argument,		0,	7 },
		{"s",					no_argument,		0,	8 },
		{"sync",				no_argument,		0,	8 },
		{"ct",					required_argument,	0,	10 },
		{"center-threshold",	required_argument,	0,	10 },
		{"et",					required_argument,	0,	11 },
		{"edge-threshold",		required_argument,	0,	11 },
		{"program",				required_argument,	0,	12 },
		{"qpu",					required_argument,	0,	13 },
		{"db",					no_argument,		0,	16 },
		{"display-blobs",		no_argument,		0,	16 },
		{"df",					no_argument,		0,	17 },
		{"display-frame",		no_argument,		0,	17 },
		{"display",				required_argument,	0,	18 },
		{"nostatlog",			no_argument,		0,	19 },
		{"id",					required_argument,	0,	20 },
		{"u",					optional_argument,	0,	21 },
		{"uart",				optional_argument,	0,	21 },
		{"server-host",			required_argument,	0,	22 },
		{"server-port",			required_argument,	0,	23 },
		{"nocomms",			no_argument,		0,	24 },	
		{"nomcu",				no_argument,		0,	25 },
		{"probe",				no_argument,		0,	26 },
		{0,						0,					0,	0 }
	};

	int c, i;
	auto print_help = [](const char *progname)
	{
		printf(
"Usage: %s [options]\n"
"  options:\n"
"    -h, --help                   Display this help message\n"
"    -r,  --res WxH               Sets camera resolution (1280x800)\n"
"    -f,  --fps fps               Sets target camera framerate in Hz (144)\n"
"    -ss, --shutter-speed ss      Sets the camera shutter speed in ns, [10-904] (100)\n"
"    -g,  --gain ag               Sets the analogue gain, [1-16] (1)\n"
"    -ct, --center-threshold ct   Sets the absolute threshold for blob centers [0.0-1.0] (0.16)\n"
"    -et, --edge-threshold et     Sets the relative threshold for blob edges [0.0-1.0] (1.0)\n"
"    -db, --display-blobs         Set to display blobs on screen\n"
"    -df, --display-frame         Set to display frame on screen\n"
"    --display WxHxF              Set display resolution and interval (in frames) (640x480x10)\n"
"    -u,  --uart [=port]          Set to communicate over UART to wait for commands - optionally set uart port (/dev/ttyAMA0)\n"
"    --nocomms                    Explicitly don't wait for any comms\n"
"    -s,  --sync                  Set camera to frame sync mode\n"
"    --nostrobe                   Set camera to NOT output frame exposure on the strobe pin\n"
"    --nostatlog                  Do NOT log frame statistics. Improves performance significantly when printed to screen.\n"
"  Some Dev Options are hidden. See Code for more.\n"
"\n"
"Examples:\n"
"  sudo %s -r 1280x800 -f 144 -ss 15 -g 16 -ct 0.99 -et 0.3 -db\n"
"  sudo %s -r 1280x800 -f 144 -ss 904 -g 4 -ct 1.0 -et 1.0 -df -db\n", progname, progname, progname);
	};

	while ((c = getopt_long_only(argc, argv, "", long_options, &i)) != -1)
	{
		switch (c)
		{
			case 2:
			{
				int width, height;
				sscanf(optarg, "%dx%d", &width, &height);
				if (width <= 2000 && width >= 64 && height <= 2000 && height >= 64)
				{
					state.camera.width = width;
					state.camera.height = height;
				}
				else
				{
					printf("Invalid format '%s' for resolution! Expecting WxH", optarg);
				}
				break;
			}
			case 3:
				state.camera.fps = std::stoi(optarg);
				break;
			case 4:
				state.camera.shutterSpeed = std::stoi(optarg);
				break;
			case 5:
				state.camera.analogGain = std::stoi(optarg);
				break;
			case 6:
				state.camera.digitalGain = std::stoi(optarg);
				break;
			case 7:
				state.camera.strobe = false;
				break;
			case 8:
				state.camera.extTrig = 1;
				break;
			case 10:
				state.thresholds.absolute = 255*std::stof(optarg);
				break;
			case 11:
				state.thresholds.edge = 255*std::stof(optarg);
				break;
			case 12:
				if (optarg && *optarg)
					state.codeFile = std::string(optarg);
				break;
			case 13:
				for (int i = 0; i < 12 && i < strlen(optarg); i++)
					state.qpuCores.enabled[i] = optarg[i] == '1';
				break;
			case 16:
				state.visualisation.enabled = true;
				state.visualisation.displayBlobs = true;
				break;
			case 17:
				state.visualisation.enabled = true;
				state.visualisation.displayFrame = true;
				break;
			case 18:
				if (optarg && *optarg)
					sscanf(optarg, "%dx%dx%d", &state.visualisation.width, &state.visualisation.height, &state.visualisation.interval);
				break;
			case 19:
				state.writeStatLogs = false;
				break;
			case 20:
				state.id = std::stoi(optarg);
				break;
			case 21:
				state.enableUART = true;
				if (optarg && *optarg)
					state.serialName = std::string(optarg);
				break;
			case 22:
				state.server_host = std::string(optarg);
				if (!state.server_host.empty())
				{ // If there's a wifi connection setup, this will start the server
					state.wireless.config = (WirelessConfig)(state.wireless.config | WIRELESS_CONFIG_SERVER);
					state.wireless.changed = (WirelessConfig)(state.wireless.changed | WIRELESS_CONFIG_SERVER);
				}
				break;
			case 23:
				state.server_port = std::string(optarg);
				break;
			case 24:
				state.noComms = true;
				break;
			case 25:
				state.noMCU = true;
				break;
			case 26:
				state.probeMode = true;
				break;
			case 1:
			default:
				printf("Invalid option %d!", c);
				print_help(argv[0]);
				return false;
		}
	}
	if (optind < argc - 1)
	{
		print_help(argv[0]);
		return false;
	}

	// ---- Checks ----

	if (FILE *file = fopen(state.codeFile.c_str(), "r"))
		fclose(file);
	else
	{
		printf("Main code file %s does not exist!\n", state.codeFile.c_str());
		return false;
	}
	return true;
}

void acceptCPUConfig(TrackingCameraState &state)
{
	ConfigPacket &setupPacket = state.newConfigPacket;
	state.blobParams = setupPacket.blobProc;
}
void acceptQPUConfig(TrackingCameraState &state)
{
	ConfigPacket &setupPacket = state.newConfigPacket;
	if (!state.curMode.streaming)
	{ // If not streaming
		state.camera.width = setupPacket.width;
		state.camera.height = setupPacket.height;
	}
	state.camera.fps = setupPacket.fps;
	state.camera.shutterSpeed = setupPacket.shutterSpeed;
	state.camera.analogGain = setupPacket.analogGain;
	state.thresholds = setupPacket.blobProc.thresholds;
	state.camera.extTrig = setupPacket.extTrig;
	state.camera.strobe = setupPacket.strobe;
	state.camera.strobeOffset = setupPacket.strobeOffset;
	state.camera.strobeLength = setupPacket.strobeLength;
}