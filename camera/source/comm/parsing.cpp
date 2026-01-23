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

#include "parsing.hpp"

#include "wireless.hpp"
#include "firmware.hpp"
#include "version.hpp"

#include <thread>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <unistd.h>

bool ReceivePacketHeader(CommState &comm, const PacketHeader header)
{
	if (header.tag < PACKET_HOST_COMM)
		return false;
	switch (header.tag)
	{
		case PACKET_SYNC:
			return header.length >= SYNC_PACKET_SIZE;
		case PACKET_SOF:
			return header.length >= SOF_PACKET_SIZE;
		case PACKET_CFG_SETUP:
			return header.length >= CONFIG_PACKET_SIZE;
		case PACKET_CFG_MODE:
			return header.length >= 1;
		case PACKET_CFG_WIFI:
			return header.length >= 8;
		case PACKET_CFG_IMAGE:
			return header.length >= 1;
		case PACKET_CFG_VIS:
			return header.length >= 1;
		case PACKET_FW_PREPARE:
		case PACKET_FW_BLOCK:
		case PACKET_FW_APPLY:
		case PACKET_FW_STATUS:
			return header.length >= FIRMWARE_PACKET_HEADER;
		case PACKET_CAMERA_INFO:
			return header.length == 0;
		default:
			printf("Received packet header with invalid tag %d!\n", header.tag);
			return false;
	}
}

bool ReceivePacketData(TrackingCameraState &state, CommState &comm, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (header.tag < PACKET_HOST_COMM)
		return false;
	if (erroneous)
		return false;
	switch (header.tag)
	{
		case PACKET_CFG_SETUP:
		{ // Received setup packet
			ConfigPacket packet = parseConfigPacket(data);
			printf("Received configuration: %dx%d @ %d fps, exposure level %d, %dx gain, m=%.2f, n=%.2f, extTrig? %d, strobe? %c (%d)\n",
				packet.width, packet.height, packet.fps, packet.shutterSpeed,
				packet.analogGain, packet.blobProc.thresholds.absolute/255.0f, packet.blobProc.thresholds.edge/255.0f,
				packet.extTrig, packet.strobe? 'y' : 'n', packet.strobeLength);
			state.newConfigPacket = packet;
			state.updateSetupQPU = true;
			state.updateSetupCPU = true;
			return true;
		}
		case PACKET_CFG_MODE:
		{ // Mode set
			uint8_t modePacket = data[0];
			TrackingCameraMode newMode = {};
			newMode.streaming = (modePacket&TRCAM_FLAG_STREAMING) != 0;
			if (newMode.streaming)
			{
				uint8_t modeSwitch = modePacket&TRCAM_MODE_MASK;
				uint8_t mode;
				for (mode = 7; mode > 0; mode--)
					if (modeSwitch >> mode) break;
				uint8_t optMask = 0xFF >> (8-mode);
				newMode.mode = (TrCamMode)(modeSwitch&(~optMask));
				newMode.opt = (TrCamMode)(modeSwitch&optMask);
			}
			while (state.updateMode)
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			state.newModeRaw = (TrCamMode)modePacket;
			state.newMode = newMode;
			state.updateMode = true;
			return true;
		}
		case PACKET_CFG_WIFI:
		{
			return parseWirelessConfigPacket(state, data, length);
		}
		case PACKET_CFG_IMAGE:
		{
			ImageStreamState stream = {};
			stream.enabled = data[0] != 0;
			bool oneshot = data[0] == 2;
			if (stream.enabled && length >= 12)
			{
				stream.bounds = Bounds2<int>(
					*((uint16_t*)&data[1]),
					*((uint16_t*)&data[3]),
					*((uint16_t*)&data[5]),
					*((uint16_t*)&data[7])
				);

				stream.subsampling = data[9];
				stream.jpegQuality = data[10];
				stream.frame = data[11];

				/* if (oneshot)
				{
					printf("Requested image of frame %d in rect (%d,%d)x(%d,%d) subsampled %d at %d quality\n",
						stream.frame,
						stream.bounds.minX, stream.bounds.minY, stream.bounds.maxX, stream.bounds.maxY,
						stream.subsampling, stream.jpegQuality);
				}
				else
				{
					printf("Requested streaming every %d frames in rect (%d,%d)x(%d,%d) subsampled %d at %d quality\n",
						stream.frame,
						stream.bounds.minX, stream.bounds.minY, stream.bounds.maxX, stream.bounds.maxY,
						stream.subsampling, stream.jpegQuality);
				} */
			}
			else
				printf("Frame streaming disabled!\n");

			if (oneshot)
				state.imageRequests.push(stream);
			else
				state.streaming = stream;
			return true;
		}
		case PACKET_CFG_VIS:
		{
			state.visualisation.enabled = data[0];
			if (state.visualisation.enabled)
			{
				state.visualisation.displayFrame = data[0];
				state.visualisation.width = *((uint16_t*)&data[1]);
				state.visualisation.height = *((uint16_t*)&data[3]);
				state.visualisation.interval = 1000000/data[5];
				state.visualisation.displayFrame = data[6];
				state.visualisation.displayBlobs = data[7];
				printf("Visualisation at %dx%d every %d frames!\n", state.visualisation.width, state.visualisation.height, state.visualisation.interval);
			}
			else
				printf("Visualisation disabled!\n");
			return true;
		}
		case PACKET_FW_PREPARE:
		{ // Prepare to receive a firmware file
			return SetupFirmwareUpdate(state, comm, data, length);
		}
		case PACKET_FW_BLOCK:
		{ // Prepare to receive a firmware file
			return ReceiveFirmwareBlock(state, comm, data, length);
		}
		case PACKET_FW_APPLY:
		{ // Prepare to receive a firmware file
			return ReceiveFirmwareApplyRequest(state, comm, data, length);
		}
		case PACKET_FW_STATUS:
		{ // Requesting a status that might have gone lost in transfer, or update cancel request
			return ReceiveFirmwareStatus(state, comm, data, length);
		}
		case PACKET_CAMERA_INFO:
		{
			sendInfoPacket(comm.medium);
			return true;
		}
		default:
			printf("Received packet data with unsupported header tag!\n");
			return false;
	}
}