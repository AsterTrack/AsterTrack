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

#include "ui/ui.hpp"

#include "device/tracking_camera.hpp"

#include "comm/protocol_stream.hpp"

#include "comm/uart.h"

void InterfaceState::UpdateTestingTool(InterfaceWindow &window)
{
	if (!window.open)
		return;
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}

	{
		static TimePoint_t lastAction = sclock::now();
		static int cycle = 0;
		static bool running = false;
		if (!running && ImGui::Button("Test Startup Reliability", SizeWidthFull()))
		{
			StopDeviceMode(GetState());
			cycle = 0;
			running = true;
		}
		else if (running && ImGui::Button("Abort Test##Startup", SizeWidthFull()))
		{
			StopDeviceMode(GetState());
			running = false;
		}
		if (running)
		{
			auto &state = GetState();
			if (state.mode == MODE_None && !state.isStreaming && dtMS(lastAction, sclock::now()) > 2000)
			{
				if (cycle == 0)
					StartDeviceMode(state);
				else
				{
					LOG(LGUI, LOutput, "Unexpected cycle %d when disconnected!", cycle);
					running = false;
				}
				cycle = 1;
				lastAction = sclock::now();
			}
			else if (state.mode == MODE_Device && !state.isStreaming && dtMS(lastAction, sclock::now()) > 2000)
			{
				if (state.cameras.empty())
				{
					LOG(LGUI, LOutput, "Did not connect to camera as expected!");
					running = false;
				}
				else if (cycle == 1)
					StartStreaming(state);
				else
				{
					LOG(LGUI, LOutput, "Unexpected cycle %d when connected!", cycle);
					running = false;
				}
				cycle = 2;
				lastAction = sclock::now();
			}
			else if (state.mode == MODE_Device && state.isStreaming && dtMS(lastAction, sclock::now()) > 5000)
			{
				if (state.pipeline.frameNum.load() < 256)
				{
					LOG(LGUI, LOutput, "Only got %ld frames!", state.pipeline.frameNum.load());
					running = false;
				}
				if (cycle == 2)
					StopDeviceMode(state);
				else
				{
					LOG(LGUI, LOutput, "Unexpected cycle %d when streaming!", cycle);
					running = false;
				}
				cycle = 0;
				lastAction = sclock::now();
			}
		}
	}

	{
		static bool running = false;
		if (!running && ImGui::Button("Test Camera Firmware Update Reliability", SizeWidthFull()))
		{
			running = true;
		}
		else if (running && ImGui::Button("Abort Test##FWCam", SizeWidthFull()))
		{
			running = false;
		}
		bool next = false;
		if (running && GetState().cameraFirmwareUpdate)
		{
			auto status = GetState().cameraFirmwareUpdate->contextualLock();
			if (status->concluded)
			{
				if (status->code == FW_STATUS_UPDATED && cameraFWSetup.valid)
					next = true;
				else
					running = false;
			}
		}
		else if (running)
			next = true;
		if (next)
		{ // Autostart next
			std::vector<std::shared_ptr<TrackingCameraState>> firmwareUpdateCameras;
			bool doNext = true;
			for (auto &camera : GetState().cameras)
			{
				camera->firmware = nullptr;
				if (camera->selectedForFirmware)
					firmwareUpdateCameras.push_back(camera); // new shared_ptr
				if (!camera->hasComms())
					doNext = false; // Still waiting for comms from last one
			}
			if (doNext)
				GetState().cameraFirmwareUpdate = CamerasFlashFirmwareFile(firmwareUpdateCameras, cameraFWSetup.file);
		}
	}

	{
		static bool running = false;
		static ProtocolState proto;
		static std::vector<uint8_t> testData;
		static int progress;
		static int parseProgress;
		static std::vector<std::pair<PacketHeader, uint32_t>> packets;
		auto regenPackets = [&]()
		{
			testData.resize(2000);
			packets.clear();
			progress = 0;
			LOG(LGUI, LInfo, "Generating test packets totalling %d bytes!", (int)testData.size());
			while (progress < testData.size())
			{
				int length = std::min((int)testData.size()-progress-UART_PACKET_OVERHEAD_SEND, rand() % 300);
				if (length < 0)
					break;
				if (length > 200)
					length = 0;
				UARTPacketRef *packet = (UARTPacketRef *)(testData.data()+progress);
				PacketTag tag = (PacketTag)(rand()%PACKET_MAX_ID_POSSIBLE);
				writeUARTPacketHeader(packet, PacketHeader(tag, length));
				for (int i = 0; i < length; i++)
					packet->data[i] = rand()%256;
				int checksum = length > 0? PACKET_CHECKSUM_SIZE : 0;
				if (checksum)
				{
					if (tag >= PACKET_HOST_COMM)
						calculateForwardPacketChecksum(packet->data, length, packet->data+length);
					else // We should not be sending these packets, but do allow for it
						calculateDirectPacketChecksum(packet->data, length, packet->data+length);
				}
				writeUARTPacketEnd(packet, length + checksum);
				if (tag % 13 != 0)
				{ // Set some packets up for discarding
					LOG(LGUI, LInfo, "    Setting up test packet %d of size %d with checksum %.8x!", tag, length, *(uint32_t*)(packet->data+length));
					packets.emplace_back(PacketHeader(tag, length), *(uint32_t*)(packet->data+length));
				}
				else
				{
					LOG(LGUI, LInfo, "    Setting up test packet %d of size %d to skip!", tag, length);
				}
				progress += UART_PACKET_OVERHEAD_SEND - PACKET_CHECKSUM_SIZE + length + checksum;
			}
			LOG(LGUI, LInfo, "Used a total of %d bytes!", progress);
			testData.resize(progress);
			progress = 0;
			parseProgress = 0;
		};
		if (!running && ImGui::Button("Test Stream Protocol Reliability", SizeWidthFull()))
		{
			regenPackets();
			proto = {};
			proto.rcvBuf.resize(testData.size());
			running = true;
		}
		else if (running && ImGui::Button("Abort Test##ProtoStream", SizeWidthFull()))
		{
			running = false;
		}

		if (running)
		{
			int len = std::min<int>(testData.size()-progress, rand() % 100);
			proto_clean(proto, true);
			memcpy(proto.rcvBuf.data()+proto.tail, testData.data()+progress, len);
			proto.tail += len;
			progress += len;

			while (len > 0 && proto_rcvCmd(proto))
			{ // Got a new command to handle
				if (proto.header.tag % 13 != 0)
				{
					if (proto_fetchCmd(proto))
					{
						uint32_t checksum = *(uint32_t*)(proto.rcvBuf.data()+proto.cmdPos+proto.cmdSz);
						if (proto.cmdSz > 0)
							LOG(LGUI, LInfo, "    Received test packet %d of size %d with checksum %.8x!", proto.header.tag, proto.cmdSz, checksum);
						else
						 	LOG(LGUI, LInfo, "    Received test packet %d - zero-length-packet!", proto.header.tag);
						if (packets[parseProgress].first.tag != proto.header.tag || 
							packets[parseProgress].first.length != proto.cmdSz || 
							(proto.cmdSz > 0 && packets[parseProgress].second != checksum))
						{
							LOG(LGUI, LError, "Differs!");
							running = false;
							break;
						}
						parseProgress++;
					}
				}
				else
				{
					LOG(LGUI, LInfo, "    Skipping test packet %d of size %d!", proto.header.tag, proto.cmdSz);
				}
			}

			if (progress >= testData.size() && running)
			{
				if (proto.head < proto.tail)
				{
					LOG(LGUI, LError, "Haven't parsed all datam %d bytes left!", proto.tail-proto.head);
					running = false;
				}
				if (parseProgress != packets.size())
				{
					LOG(LGUI, LError, "Missing packets!");
					running = false;
				}
				else regenPackets();
			}

			RequestUpdates();
		}
	}

	ImGui::End();
}