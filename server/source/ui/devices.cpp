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

#define IMGUI_DEFINE_MATH_OPERATORS
#include "ui.hpp"

#include "device/tracking_camera.hpp"
#include "device/tracking_controller.hpp"

#include "imgui/imgui_onDemand.hpp"
#include "gl/visualisation.hpp"

// For firmware flashing

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include "nativefiledialog-extended/nfd.h"
#include "nativefiledialog-extended/nfd_glfw3.h"

#include <filesystem>


void InterfaceState::UpdateDevices(InterfaceWindow &window)
{
	static FirmwareUpdateRef firmwareUpdate;
	bool firmwareUpdateSetup = false;

	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		// TODO: Could abort firmware update here, but window might be accidentally hidden
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	int pad = 16; // Padding for controllers
	ImVec2 controllerSize = ImVec2(128, 128);
	ImVec2 cameraSize = ImVec2(32, 64);

	auto drawCamera = [&](std::shared_ptr<TrackingCameraState> &cameraPtr, float width)
	{
		TrackingCameraState &camera = *cameraPtr;
		ImGui::PushID(camera.id);
		ImVec2 pos = ImGui::GetCursorPos();
		ImGui::Image(icons().camera, cameraSize);
		ImRect bb(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
		ImRect bbIcon(bb.GetCenter(), bb.GetCenter());
		bbIcon.Expand(ImGui::GetFontSize()/2);
		OnDemandItem *icon = AddOnDemandRender(bbIcon, [](const ImDrawList* dl, const ImDrawCmd* dc)
		{
			OnDemandItem &render = *static_cast<OnDemandItem*>(dc->UserCallbackData);
			CameraID id = (CameraID)(intptr_t)render.userData;

			// Get camera from id (to make sure it's still valid)
			std::shared_lock dev_lock(GetState().deviceAccessMutex); // cameras
			auto viewIt = GetUI().cameraViews.find(id);
			if (viewIt == GetUI().cameraViews.end())
				return; // Just removed, but UI hasn't been updated yet

			ImVec2 size = SetOnDemandRenderArea(render, dc->ClipRect);
			Color color = getStatusColor(*viewIt->second.camera);
			visSetupProjection(Eigen::Isometry3f::Identity());
			visualiseCircle<true>(Eigen::Vector2f::Zero(), 0.8f, color);
		}, (void*)(intptr_t)camera.id);

		// Detail text
		//pos.x = bb.Max.x + ImGui::GetStyle().ItemInnerSpacing.x;
		ImGui::SetCursorPos(pos);
		ImGui::Indent(bb.GetWidth() + ImGui::GetStyle().ItemInnerSpacing.x);

		auto &wireless = camera.config.wireless;
		if (wireless.wifiStatus == WIRELESS_STATUS_CONNECTED)
		{ // Have to manually add frame padding
			ImGui::SetCursorPosY(ImGui::GetCursorPosY()+ImGui::GetStyle().FramePadding.y);
			ImGui::Image(icons().wireless, ImVec2(ImGui::GetFontSize()*6/5, ImGui::GetFontSize()));
			if (ImGui::BeginItemTooltip())
			{
				ImGui::Text("Camera is connected to wireless network '%s' with IP '%s'", wireless.SSID.c_str(), wireless.IP.c_str());
				if (wireless.serverStatus == WIRELESS_STATUS_ENABLED || wireless.serverStatus == WIRELESS_STATUS_CONNECTED)
				{
					if (camera.client && camera.client->ready)
						ImGui::Text("The server is running and connected.");
					else
						ImGui::Text("The server is running but not connected.");
				}
				ImGui::EndTooltip();
			}
			ImGui::SameLine(0.0f, ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x);
		}

		// ID Label
		if (camera.pipeline)
			ImGui::Text("#%d (%d)", camera.id, camera.pipeline->index);
		else
			ImGui::Text("#%d", camera.id);

		{ // Sync
			// TODO: Allow detaching from SyncGroup controller is part of via toggle to switch to free-running
			// This is currently part of camera settings but it should be part of the SyncGroup configuration UI rework
			if (camera.sync)
			{
				SyncSource source;
				float intervalMS;
				{
					auto sync = camera.sync->contextualRLock();
					source = sync->source;
					intervalMS = sync->frameIntervalMS;
				}
				if (source == SYNC_NONE)
					ImGui::Text("%dHz", (int)(1000.0f/intervalMS));
				else
					ImGui::Text("%dHz", state.controllerConfig.framerate);
			}
			else
			{
				ImGui::Text("%dHz", state.controllerConfig.framerate);
			}
			ImGui::SameLine();
		}

		{ // Config
			auto getConfigLabel = [&](int index)
			{
				if (state.cameraConfig.configurations[index].label.empty())
					return asprintf_s("Config %d", index);
				return asprintf_s("%s", state.cameraConfig.configurations[index].label.c_str());
			};
			int index = state.cameraConfig.getCameraConfigIndex(camera.id);
			const ImGuiID popupID = ImGui::GetCurrentWindowRead()->GetID("##ConfigPopup");
			/* if (ImGui::Selectable(asprintf_s("%s##config", getConfigLabel(index).c_str()).c_str()))
			{
				ImGui::OpenPopup(popupID);
			}
			if (ImGui::BeginComboPopup(popupID, ImRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()), ImGuiComboFlags_None))
			{ */
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
			ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0,0,0,0));
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0,0,0,0));
			std::string configPreview = asprintf_s("%s##config", getConfigLabel(index).c_str());
			if (ImGui::BeginCombo("##Config", configPreview.c_str(), ImGuiComboFlags_WidthFitPreview))
			{
				ImGui::PopStyleColor(2);
				ImGui::PopStyleVar();
				for (int i = 0; i < state.cameraConfig.configurations.size(); i++)
				{
					if (ImGui::Selectable(getConfigLabel(i).c_str(), i == index))
					{
						state.cameraConfig.cameraConfigs[camera.id] = i;
						camera.pipeline->mode = getCameraMode(state, camera.id);
						CameraUpdateSetup(state, camera);
					}
				}
				ImGui::EndCombo();
			}
			else
			{
				ImGui::PopStyleColor(2);
				ImGui::PopStyleVar();
			}
		}

		// Firmware
		ImGui::Text("FW");
		ImGui::SameLine();
		//ImGui::Text("v1.2.4");
		//ImGui::SameLine();
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
		ImGui::BeginDisabled(state.isStreaming || firmwareUpdate);
		if (ImGui::Button("v1.2.4"))
			camera.selectedForFirmware = !camera.selectedForFirmware;
		if (camera.selectedForFirmware) firmwareUpdateSetup = true;
		ImGui::EndDisabled();
		ImGui::PopStyleVar();

		ImGui::Unindent(bb.GetWidth() + ImGui::GetStyle().ItemInnerSpacing.x);

		ImGui::PopID();
	};

	auto drawController = [&](TrackingControllerState &controller)
	{
		ImVec2 pos = ImGui::GetCursorPos();
		ImGui::Image(icons().controller, controllerSize);
		ImRect bb(ImGui::GetItemRectMin() + ImVec2(pad, pad), ImGui::GetItemRectMax() - ImVec2(pad, pad));

		ImGui::SetCursorPos(pos + ImVec2(0, pad));
		ImGui::Indent(pad);
		float colX = ImGui::GetCursorPosX() + ImGui::GetTextLineHeight()*2.0f;

		{ // Status
			ImVec2 statusSize(ImGui::GetTextLineHeight(), ImGui::GetTextLineHeight());
			OnDemandItem *icon = AddOnDemandIcon("icon", statusSize, statusSize, [](const ImDrawList* dl, const ImDrawCmd* dc)
			{
				OnDemandItem &render = *static_cast<OnDemandItem*>(dc->UserCallbackData);
				int id = (int)(intptr_t)render.userData;

				// Get camera from id (to make sure it's still valid)
				std::shared_lock dev_lock(GetState().deviceAccessMutex); // cameras
				auto contIt = std::find_if(GetState().controllers.begin(), GetState().controllers.end(), [&](const auto &c) { return c->id == id; });
				if (contIt == GetState().controllers.end())
					return; // Just removed, but UI hasn't been updated yet
				const TrackingControllerState &controller = *contIt->get();

				ImVec2 size = SetOnDemandRenderArea(render, dc->ClipRect);
				Color color = getStatusColor(controller);
				visSetupProjection(Eigen::Isometry3f::Identity());
				visualiseCircle<true>(Eigen::Vector2f::Zero(), 0.8f, color);
			}, (void*)(intptr_t)controller.id);
			ImGui::SameLine(colX);
			ImGui::Text("Status");
		}

		{ // Synchronisation
			if (controller.syncGen)
			{
				ImGui::Text("Gen");
				ImGui::SameLine(colX);
				ImGui::Text("%dHz", (int)(1000.0f/controller.syncGen->contextualRLock()->frameIntervalMS));
			}
			else if (controller.sync)
			{
				auto sync_lock = controller.sync->contextualRLock();
				if (sync_lock->source == SYNC_NONE)
					ImGui::Text("Free");
				else if (sync_lock->source == SYNC_INTERNAL)
					ImGui::Text(controller.syncGen? "Gen" : "Lock");
				else if (sync_lock->source == SYNC_EXTERNAL)
					ImGui::Text(controller.syncGen? "Ext" : "Lock");
				else // Virtual should never be associated with a controller
					ImGui::Text("????");
				ImGui::SameLine(colX);
				ImGui::Text("%dHz", (int)(1000.0f/sync_lock->frameIntervalMS));
			}
			else
			{
				ImGui::Text("No Sync");
			}
		}

		{ // Power Passthrough
			if (true)
			{ // Has power
				if (true)
					ImGui::Text("PD");
				else // External
					ImGui::Text("Ext");
				ImGui::SameLine(colX);
				ImGui::Text("%.1fV", 15.0f);
			}
			else
			{
				ImGui::Text("No Power");
			}
		}

		{ // Firmware
			ImGui::Text("FW");
			ImGui::SameLine(colX);
			ImGui::Text("v1.2.4");
		}

		ImGui::Unindent(pad);
	};

	// Check for free cameras not associated with a controller
	int numFreeCameras = 0;
	for (auto &camera : state.cameras)
	{
		if (!camera->controller)
			numFreeCameras++;
	}
	// Determine number of rows and cols for ports and free cameras respectively
	int numRows = 0;
	for (auto &controller : state.controllers)
		numRows = std::max<int>(numRows, controller->cameras.size());
	if (numRows == 0 && numFreeCameras > 0)
		numRows = std::min(numFreeCameras, 8);
	int freeCols = numFreeCameras == 0? 0 : (int)std::ceil((float)numFreeCameras/numRows);
	int numCols = state.controllers.size() + freeCols;

	if (numCols > 0 && ImGui::BeginTable("Devices", numCols,
		ImGuiTableFlags_PadOuterX | ImGuiTableFlags_BordersInner))
	{
		// Register columns, but without headers
		for (auto &controller : state.controllers)
			ImGui::TableSetupColumn(asprintf_s("Con %d", controller->id).c_str());
		for (int i = 0; i < freeCols; i++)
			ImGui::TableSetupColumn(asprintf_s("Free %d", i).c_str());

		// Draw header ourselves
		{
			ImGui::TableNextRow();
			for (auto &controller : state.controllers)
			{
				ImGui::TableNextColumn();
				drawController(*controller);
			}
			for (int i = 0; i < freeCols; i++)
			{
				ImGui::TableNextColumn();
				ImVec2 pos = ImGui::GetCursorPos();
				ImGui::Image(icons().controller, controllerSize);
				ImGui::SetCursorPos(pos + ImVec2(pad, pad));
				ImGui::Text("Free");
			}
		}

		int free = 0;
		for (int row = 0; row < numRows; row++)
		{
			ImGui::TableNextRow();
			for (auto &controller : state.controllers)
			{
				ImGui::TableNextColumn();
				if (row >= controller->cameras.size()) continue;
				if (controller->cameras[row])
					drawCamera(controller->cameras[row], ImGui::GetColumnWidth());
				else
					ImGui::Image(icons().camera, cameraSize);
			}
			for (int i = 0; i < freeCols; i++)
			{
				ImGui::TableNextColumn();
				if (free >= numFreeCameras) continue;
				int cnt = 0;
				for (auto &camera : state.cameras)
				{
					if (camera->controller) continue;
					if (cnt++ == free)
					{ // Draw next free camera
						drawCamera(camera, ImGui::GetColumnWidth());
						free++;
						break;
					}
				}
			}
		}

		ImGui::EndTable();
	}

	float minButtonWidth = MinSharedLabelWidth("Close", "Abort", "Select", "Flash");
	ImVec2 button = SizeWidthDiv4(minButtonWidth);
	if (firmwareUpdate)
	{
		ImGui::SeparatorText("Camera Firmware Update");

		auto status = firmwareUpdate->contextualRLock();
		ImGui::AlignTextToFramePadding();
		ImGui::Text("%s", status->text.c_str());
		SameLineTrailing(button.x);
		bool ended = status->code == FW_STATUS_UPDATED || status->code == FW_STATUS_ERROR || status->code == FW_STATUS_ABORT;
		if (ended && ImGui::Button("Close", button))
		{ // Close or abort firmware update
			for (auto &camera : state.cameras)
			{
				camera->selectedForFirmware = false;
				camera->firmware = nullptr;
			}
			firmwareUpdate = nullptr;
		}
		if (!ended && ImGui::Button("Abort", button))
		{ // Close or abort firmware update
			status->abort.request_stop();
		}
		for (auto &camera : state.cameras)
		{
			if (camera->firmware)
			{
				auto camStatus = camera->firmware->contextualLock();
				ImGui::Text("Camera #%d: %s", camera->id, camStatus->text.c_str());
				/* if (camStatus->code == FW_STATUS_UPDATED)
				{
					ImGui::SetCursorPosX(button.x);
					if (ImGui::Button("Restart", button))
					{ // Restart somehow? Update auto-restarts for most update types
					}
				} */
			}
			else if (camera->selectedForFirmware)
				ImGui::Text("Camera #%d is not part of the firmware update.", camera->id);
		}
	}
	else if (firmwareUpdateSetup)
	{
		ImGui::SeparatorText("Camera Firmware Update");

		// Static, shared between firmware flashing popups
		static std::string firmwareFile = "Firmware File";
		static std::string selectError;
		ImGui::AlignTextToFramePadding();
		if (!selectError.empty())
			ImGui::Text("%s", selectError.c_str());
		else
		{
			ImGui::Text("%s", std::filesystem::path(firmwareFile).filename().c_str());
			ImGui::SetItemTooltip("%s", firmwareFile.c_str());
		}
		SameLineTrailing(button.x);
		if (ImGui::Button("Select", button))
		{
			threadPool.push([](int id)
			{
				if (!NFD_Init())
				{ // Thread-specific init
					LOG(LGUI, LError, "Failed to initialise File Picker: %s", NFD_GetError());
					selectError = NFD_GetError();
					return;
				}

				const int filterLen = 2;
				nfdfilteritem_t filterList[filterLen] = {
					{"CameraPi Data Tarball", "tgz,tar.gz"},
					{"CameraMCU Program Binary", "bin"},
				};
				nfdchar_t *outPath;
			#ifdef _WIN32
				std::string defPath = "Downloads";
			#else
				std::string defPath = std::filesystem::current_path();
			#endif
				nfdopendialogu8args_t args;
				args.filterList = filterList;
				args.filterCount = filterLen;
				args.defaultPath = defPath.c_str();
				NFD_GetNativeWindowFromGLFWWindow(GetUI().glfwWindow, &args.parentWindow);
				nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
				if (result == NFD_OKAY)
				{
					firmwareFile = outPath;
					selectError.clear();
					NFD_FreePath(outPath);
				}
				else if (result == NFD_ERROR)
				{
					LOG(LGUI, LError, "Failed to use File Picker: %s", NFD_GetError());
					selectError = NFD_GetError();
				}

				NFD_Quit();

				GetUI().RequestUpdates();
			});
		}

		for (auto &camera : state.cameras)
		{
			if (camera->selectedForFirmware)
				ImGui::Text("Camera #%d set to update", camera->id);
		}

		if (ImGui::Button("Flash", SizeWidthFull()))
		{
			std::vector<std::shared_ptr<TrackingCameraState>> firmwareUpdateCameras;
			for (auto &camera : state.cameras)
			{
				if (camera->selectedForFirmware)
					firmwareUpdateCameras.push_back(camera); // new shared_ptr
			}
			firmwareUpdate = CamerasFlashFirmwareFile(firmwareUpdateCameras, firmwareFile);
		}
	}

	// TODO: List IMU providers and their IMU Devices
	// Including indication of connection stability, rssi, battery, etc.

	ImGui::End();
}