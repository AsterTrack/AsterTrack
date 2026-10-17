/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include "parsing.hpp"

#include "state.hpp"
#include "wireless.hpp"
#include "firmware.hpp"
#include "mcu/mcu.hpp"
#include "info.hpp"

#include <thread>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <unistd.h>

bool ReceivePacketHeader(CommState &comm, const PacketHeader header)
{
	if (header.tag < PACKET_HOST_SBC)
		return false;
	switch (header.tag)
	{
		case PACKET_SYNC:
			return header.length >= SYNC_PACKET_SIZE;
		case PACKET_SOF:
			return header.length >= SOF_PACKET_SIZE;
		case PACKET_CONFIG:
			return header.length >= CONFIG_PACKET_MIN_SIZE;
		case PACKET_MODE:
			return header.length >= 1;
		case PACKET_WIRELESS:
			return header.length >= 8;
		case PACKET_IMAGE:
			return header.length >= 1;
		case PACKET_FW_PREPARE:
		case PACKET_FW_BLOCK:
		case PACKET_FW_APPLY:
		case PACKET_FW_STATUS:
			return header.length >= FIRMWARE_PACKET_HEADER;
		case PACKET_CAMERA_INFO:
			return header.length == 0;
		case PACKET_CAMERA_STATE:
			return header.length == EMBEDDED_STATE_SIZE;
		default:
			printf("Received packet header with invalid tag %d!\n", header.tag);
			return false;
	}
}

bool ReceivePacketData(TrackingCameraState &state, CommState &comm, const PacketHeader header, const uint8_t *data, int length, bool erroneous)
{
	if (header.tag < PACKET_HOST_SBC)
		return false;
	if (erroneous && header.tag != PACKET_FW_BLOCK)
		return false;
	switch (header.tag)
	{
		case PACKET_CONFIG:
		{ // Received setup packet
			ConfigPacket packet = parseConfigPacket(data, length);
			if (packet.version == 0 || packet.version > CONFIG_PACKET_VERSION)
			{
				printf("Could not parse configuration version %d, supported %d\n", packet.version, CONFIG_PACKET_VERSION);
				std::vector<uint8_t> response = { CONFIG_REPORT_INVALID_VERSION, CONFIG_PACKET_VERSION };
				comm_send(&comm, PacketHeader(PACKET_CONFIG, response.size()), std::move(response));
				return false;
			}
			printf("Received configuration: %dx%d @ %.1f fps, exposure level %d, %dx gain, m=%.2f, n=%.2f, sync? %d, strobe? %c (%d)\n",
				packet.width, packet.height, 1000000000.0/packet.frameTimeNS, packet.shutterSpeed,
				packet.analogGain, packet.blobProc.thresholds.absolute/255.0f, packet.blobProc.thresholds.edge/255.0f,
				packet.sync, packet.strobe? 'y' : 'n', packet.strobeLength);
			state.newConfigPacket = packet;
			state.updateSetupQPU = true;
			state.updateSetupCPU = true;
			return true;
		}
		case PACKET_MODE:
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
		case PACKET_WIRELESS:
		{
			return parseWirelessConfigPacket(state, data, length);
		}
		case PACKET_IMAGE:
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

				if (oneshot)
				{
					printf("Requested image of frame %d in rect (%d,%d)x(%d,%d) subsampled %d at %d quality\n",
						stream.frame,
						stream.bounds.min.x(), stream.bounds.min.y(), stream.bounds.max.x(), stream.bounds.max.y(),
						stream.subsampling, stream.jpegQuality);
				}
				else if (!state.streaming.enabled)
				{
					printf("Requested streaming every %d frames in rect (%d,%d)x(%d,%d) subsampled %d at %d quality\n",
						stream.frame,
						stream.bounds.min.x(), stream.bounds.min.y(), stream.bounds.max.x(), stream.bounds.max.y(),
						stream.subsampling, stream.jpegQuality);
				}
			}
			else
				printf("Frame streaming disabled!\n");

			if (oneshot)
				state.imageRequests.push(stream);
			else
				state.streaming = stream;
			return true;
		}
		case PACKET_FW_PREPARE:
		{ // Prepare to receive a firmware file
			return SetupFirmwareUpdate(state, comm, data, length);
		}
		case PACKET_FW_BLOCK:
		{ // Prepare to receive a firmware file
			return ReceiveFirmwareBlock(state, comm, data, length, erroneous);
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
		{ // Request for camera info, but will send other state along as well
			sendInitialState(state, comm.medium);
			return true;
		}
		case PACKET_CAMERA_STATE:
		{
			mcu_update_embedded_state(data);
			return true;
		}
		default:
			printf("Received packet data with unsupported header tag!\n");
			return false;
	}
}