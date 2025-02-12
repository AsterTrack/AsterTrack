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

#ifndef OPTIONS_H
#define OPTIONS_H

#include <getopt.h>
#include <unistd.h>
#include <string>
#include <cstdio>
#include <cstring>

#include "state.hpp"

static bool options_read(TrackingCameraState &state, int argc, char **argv)
{
	const struct option long_options[] = {
		{"h",					no_argument,		0,	'h' },
		{"help",				no_argument,		0,	'h' },
		{"r",					required_argument,	0,	'r' },
		{"res",					required_argument,	0,	'r' },
		{"f",					required_argument,	0,	'f' },
		{"fps",					required_argument,	0,	'f' },
		{"ss",					required_argument,	0,	's' },
		{"shutter-speed",		required_argument,	0,	's' },
		{"g",					required_argument,	0,	'g' },
		{"gain",				required_argument,	0,	'g' },
		{"dg",					required_argument,	0,	'm' },
		{"digital-gain",		required_argument,	0,	'm' },
		{"ct",					required_argument,	0,	'c' },
		{"center-threshold",	required_argument,	0,	'c' },
		{"et",					required_argument,	0,	'e' },
		{"edge-threshold",		required_argument,	0,	'e' },
		{"db",					no_argument,		0,	'b' },
		{"display-blobs",		no_argument,		0,	'b' },
		{"df",					no_argument,		0,	'd' },
		{"display-frame",		no_argument,		0,	'd' },
		{"display",				required_argument,	0,	'v' },
		{"id",					required_argument,	0,	'i' },
		{"u",					optional_argument,	0,	'u' },
		{"uart",				optional_argument,	0,	'u' },
		{"server-host",			required_argument,	0,	'y' }, // Dev only
		{"server-port",			required_argument,	0,	'z' }, // Dev only
		{"s",					no_argument,		0,	'x' },
		{"sync",				no_argument,		0,	'x' },
		{"nostrobe",			no_argument,		0,	'o' },
		{"nostatlog",			no_argument,		0,	'l' },
		{"qpu",					required_argument,	0,	'q' }, // Dev only
		{"program",				required_argument,	0,	'p' }, // Dev only
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
"    -ss, --shutter-speed ss      Sets the camera shutter speed in ns, [7-904] (100)\n"
"    -g,  --gain ag               Sets the analogue gain, [1-16] (1)\n"
"    -ct, --center-threshold ct   Sets the absolute threshold for blob centers [0.0-1.0] (0.9)\n"
"    -et, --edge-threshold et     Sets the relative threshold for blob edges [0.0-1.0] (1.0)\n"
"    -db, --display-blobs         Set to display blobs on screen\n"
"    -df, --display-frame         Set to display frame on screen\n"
"    --display WxHxF              Set display resolution and interval (in frames) (640x480x10)\n"
"    -u,  --uart [=port]          Set to communicate over UART to wait for commands - optionally set uart port (/dev/ttyAMA0)\n"
"    -s,  --sync                  Set camera to frame sync mode\n"
"    --nostrobe                   Set camera to NOT output frame exposure on the strobe pin\n"
"    --nostatlog                  Do NOT log frame statistics. Improves performance significantly when printed to screen. \n"
"\n"
"Examples:\n"
"  sudo %s -r 1280x800 -f 144 -ss 15 -g 16 -ct 0.99 -et 0.3 -db\n"
"  sudo %s -r 1280x800 -f 144 -ss 904 -g 4 -ct 1.0 -et 1.0 -df -db\n", progname, progname, progname);
	};

	while ((c = getopt_long_only(argc, argv, "", long_options, &i)) != -1)
	{
		switch (c)
		{
			case 'p':
				if (optarg && *optarg)
					state.codeFile = std::string(optarg);
				break;
			case 'r':
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
			case 'f':
				state.camera.fps = std::stoi(optarg);
				break;
			case 's':
				state.camera.shutterSpeed = std::stoi(optarg);
				break;
			case 'g':
				state.camera.analogGain = std::stoi(optarg);
				break;
			case 'm':
				state.camera.digitalGain = std::stoi(optarg);
				break;
			case 'c':
				state.thresholds.absolute = 255*std::stof(optarg);
				break;
			case 'e':
				state.thresholds.edge = 255*std::stof(optarg);
				break;
			case 'q':
				for (int i = 0; i < 12 && i < strlen(optarg); i++)
					state.enableQPU[i] = optarg[i] == '1';
				break;
			case 'b':
				state.visualisation.enabled = true;
				state.visualisation.displayBlobs = true;
				break;
			case 'd':
				state.visualisation.enabled = true;
				state.visualisation.displayFrame = true;
				break;
			case 'v':
				if (optarg && *optarg)
					sscanf(optarg, "%dx%dx%d", &state.visualisation.width, &state.visualisation.height, &state.visualisation.interval);
				break;
			case 'i':
				state.id = std::stoi(optarg);
				break;
			case 'u':
				state.uart.enabled = true;
				if (optarg && *optarg)
					state.serialName = std::string(optarg);
				break;
			case 'x':
				state.camera.extTrig = true;
				break;
			case 'o':
				state.camera.strobe = false;
				break;
			case 'l':
				state.writeStatLogs = false;
				break;
			case 'y':
				state.server_host = std::string(optarg);
				state.server.enabled = !state.server_host.empty();
				break;
			case 'z':
				state.server_port = std::string(optarg);
				break;
			case 'h':
			default:
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

static void acceptCPUConfig(TrackingCameraState &state)
{
	ConfigPacket &setupPacket = state.newConfigPacket;
	state.blobParams = setupPacket.blobProc;
}
static void acceptQPUConfig(TrackingCameraState &state)
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

#endif // OPTIONS_H