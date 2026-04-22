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

	ImGui::End();
}