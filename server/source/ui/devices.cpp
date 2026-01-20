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
#include "comm/wireless_server_client.hpp"

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
	static FirmwareUpdateRef cameraFWUpdateState;
	bool cameraFWUpdateSetup = false;
	static ControllerFirmwareUpdateRef controllerFWUpdateState;
	bool controllerFWUpdateSetup = false;

	if (!window.open)
	{ // TODO: Could abort firmware update here
		return;
	}
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	auto &style = ImGui::GetStyle();
	float cameraHeight = ImGui::GetTextLineHeight()*3 + style.ItemSpacing.y*2;
	float controllerHeight = ImGui::GetTextLineHeight()*4 + style.ItemSpacing.y*3;
	int pad = (int)(controllerHeight/6); // Padding for controllers
	controllerHeight += pad*2;
	ImVec2 cameraSize = ImVec2(cameraHeight/2, cameraHeight);
	ImVec2 controllerSize = ImVec2(controllerHeight, controllerHeight);

	auto drawCamera = [&](std::shared_ptr<TrackingCameraState> &cameraPtr, float width)
	{
		TrackingCameraState &camera = *cameraPtr;
		ImGui::PushID(camera.id);

		// Draw Camera Symbol
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
			auto viewIt = GetUI().cameraViews.find(id);
			if (viewIt == GetUI().cameraViews.end())
				return; // Just removed, but UI hasn't been updated yet

			ImVec2 size = SetOnDemandRenderArea(render, dc->ClipRect);
			Color color = getStatusColor(*viewIt->second.camera);
			visSetupProjection(Eigen::Isometry3f::Identity());
			visualiseCircle<true>(Eigen::Vector2f::Zero(), 0.8f, color);
		}, (void*)(intptr_t)camera.id);
		if (ImGui::BeginItemTooltip())
		{
			ImGui::TextUnformatted(getStatusText(camera).c_str());
			ImGui::EndTooltip();
		}

		// Detail text
		ImGui::SetCursorPos(pos);
		ImGui::Indent(bb.GetWidth() + style.ItemInnerSpacing.x);
		auto &info = camera.storage.info;
		auto &wireless = camera.config.wireless;

		// ID Label
		if (camera.pipeline)
			ImGui::Text("#%u (%d)", camera.id, camera.pipeline->index);
		else
			ImGui::Text("#%u", camera.id);
		ImGui::SameLine();

		// Information Popup
		const ImGuiID infoPopupID = ImGui::GetCurrentWindowRead()->GetID("##InfoPopup");
		if (InlineIconButton(ICON_LA_INFO_CIRCLE))
			ImGui::OpenPopup(infoPopupID);
		if (ImGui::BeginComboPopup(infoPopupID, ImRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()),
			ImGuiComboFlags_PopupAlignLeft | ImGuiComboFlags_HeightLarge))
		{
			auto desc = CameraDescribeInfo(info);
			if (desc.empty())
				ImGui::TextUnformatted("Camera has not sent info yet!");
			else
			{
				ImGui::AlignTextToFramePadding();
				ImGui::Text("FW & HW Info:");
				SameLineTrailing(SizeWidthDiv3().x);
				if (ImGui::Button("Copy", SizeWidthDiv3()))
				{
					int totalSize = 0;
					for (const auto &str : desc)
						totalSize += str.length() + 1;
					std::string infoCopy;
					infoCopy.resize(totalSize);
					int index = 0;
					for (const auto &str : desc)
					{
						memcpy(infoCopy.data()+index, str.data(), str.length());
						index += str.length();
						infoCopy.data()[index++] = '\n';
					}
					infoCopy.resize(index);
					ImGui::SetClipboardText(infoCopy.c_str());
				}
				for (auto &str : desc)
					ImGui::TextUnformatted(str.c_str());
			}
			ImGui::EndCombo();
		}

		if (wireless.wifiStatus == WIRELESS_STATUS_CONNECTED)
		{ // Have to manually add frame padding
			ImGui::SameLine();
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
		}

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
		ImGui::BeginGroup();
		ImGui::Text("FW");
		if (info.sbcFWVersion.num)
		{
			ImGui::SameLine();
			ImGui::Text("v%d.%d.%d", info.sbcFWVersion.major, info.sbcFWVersion.minor, info.sbcFWVersion.patch);
		}
		if (info.mcuFWVersion.num)
		{
			ImGui::SameLine();
			ImGui::Text("(v%d.%d.%d)", info.mcuFWVersion.major, info.mcuFWVersion.minor, info.mcuFWVersion.patch);
		}
		if (!info.sbcFWVersion.num && !info.mcuFWVersion.num)
		{
			ImGui::SameLine();
			ImGui::TextUnformatted("Unknown");
		}
		ImGui::EndGroup();
		if (ImGui::BeginItemTooltip())
		{
			ImGui::Text("SBC Firmware v%d.%d.%d (Build %.2x - %s)",
				info.sbcFWVersion.major, info.sbcFWVersion.minor, info.sbcFWVersion.patch, info.sbcFWVersion.build, info.sbcFWDescriptor.c_str());
			ImGui::Text("MCU Firmware v%d.%d.%d (Build %.2x - %s)",
				info.mcuFWVersion.major, info.mcuFWVersion.minor, info.mcuFWVersion.patch, info.mcuFWVersion.build, info.mcuFWDescriptor.c_str());
			ImGui::EndTooltip();
		}
		ImGui::SameLine();
		ImGui::BeginDisabled(state.isStreaming || cameraFWUpdateState);
		if (InlineIconButton(camera.selectedForFirmware? ICON_LA_BAN"###Update" : ICON_LA_DOWNLOAD"###Update"))
			camera.selectedForFirmware = !camera.selectedForFirmware;
		if (camera.selectedForFirmware) cameraFWUpdateSetup = true;
		ImGui::EndDisabled();

		ImGui::Unindent(bb.GetWidth() + style.ItemInnerSpacing.x);

		ImGui::PopID();
	};

	auto drawController = [&](TrackingControllerState &controller)
	{
		ImVec2 pos = ImGui::GetCursorPos();
		ImGui::Image(icons().controller, controllerSize);
		ImRect bb(ImGui::GetItemRectMin() + ImVec2(pad, pad), ImGui::GetItemRectMax() - ImVec2(pad, pad));

		ImGui::SetCursorPos(pos + ImVec2(0, pad));
		ImGui::Indent(pad);
		float colX = (controllerSize.x - pad*2 - style.ItemInnerSpacing.x) / 2;

		{ // Status
			ImGui::BeginGroup();
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
			ImGui::EndGroup();
		}

		{ // Synchronisation
			ImGui::BeginGroup();
			if (controller.syncGen)
			{
				ImGui::TextUnformatted(controller.status.syncCfg == SYNC_CFG_GEN_RATE? "Gen" : "Gen...");
				ImGui::SameLine(colX);
				ImGui::Text("%dHz", (int)(1000.0f/controller.syncGen->contextualRLock()->frameIntervalMS));
			}
			else if (controller.sync)
			{
				auto sync_lock = controller.sync->contextualRLock();
				switch (sync_lock->source)
				{
					case SYNC_NONE:
						ImGui::TextUnformatted(controller.status.syncCfg == SYNC_CFG_NONE? "Free" : "Free~");
						break;
					case SYNC_INTERNAL: // Locked to a sync generated by another controller
						ImGui::TextUnformatted(controller.status.syncCfg == SYNC_CFG_EXT_TRIG? "Lock" : "Lock~");
						break;
					case SYNC_EXTERNAL: // Locked to an external sync source
						ImGui::TextUnformatted(controller.status.syncCfg == SYNC_CFG_EXT_TRIG? "Ext" : "Ext~");
						break;
					case SYNC_VIRTUAL:// Virtual should never be associated with a controller
						ImGui::TextUnformatted("????");
						break;
				}
				ImGui::SameLine(colX);
				ImGui::Text("%dHz", (int)(1000.0f/sync_lock->frameIntervalMS));
			}
			else
			{
				ImGui::TextUnformatted("No Sync");
			}
			ImGui::EndGroup();
		}

		// Power Control
		ImGui::BeginGroup();
		switch (controller.status.powerState)
		{
			case POWER_WAITING:
				ImGui::TextUnformatted("No Power");
				break;
			case POWER_PD_IN:
			case POWER_PD_IN_EXT_OUT:
				ImGui::TextUnformatted(controller.status.powerState == POWER_PD_IN_EXT_OUT? "PD out" : "PD");
				ImGui::SameLine(colX);
				ImGui::Text("%.1fV", controller.status.voltagePD.rounded);
				break;
			case POWER_EXT_IN:
				ImGui::TextUnformatted("Ext");
				ImGui::SameLine(colX);
				ImGui::Text("%.1fV", controller.status.voltageExt.rounded);
				break;
		}
		ImGui::EndGroup();
		if (ImGui::BeginItemTooltip())
		{
			ImGui::Text("Voltages:\nPD: %.2fV\nExt: %.2fV", controller.status.voltagePD.raw, controller.status.voltageExt.raw);
			if (controller.status.powerState == POWER_PD_IN_EXT_OUT)
				ImGui::TextUnformatted("The PD Power is exposed on the External Power Port!");
			ImGui::EndTooltip();
		}

		{ // Firmware
			ImGui::BeginGroup();
			ImGui::TextUnformatted("FW");
			ImGui::SameLine(colX);
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
			ImGui::BeginDisabled(state.isStreaming || controller.firmware);
			if (ImGui::Button("v1.2.4"))
				controller.selectedForFirmware = !controller.selectedForFirmware;
			if (controller.selectedForFirmware || controller.firmware)
				controllerFWUpdateSetup = true;
			ImGui::EndDisabled();
			ImGui::PopStyleVar();
			ImGui::EndGroup();
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

	if (numCols == 0)
	{
		ImGui::TextWrapped("There are no controllers or cameras connected.\n"
			"Plug in a controller, then Disconnect and Connect again for it to appear.");
	}
	else if (ImGui::BeginTable("Devices", numCols,
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
	if (cameraFWUpdateState)
	{
		ImGui::SeparatorText("Camera Firmware Update");

		auto status = cameraFWUpdateState->contextualLock();
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
			cameraFWUpdateState = nullptr;
		}
		if (!ended && ImGui::Button("Abort", button))
		{ // Close or abort firmware update
			status->abort.request_stop();
		}
		for (auto &camera : state.cameras)
		{
			bool currentlyDisconnected = !camera->client && !camera->controller;
			if (camera->firmware)
			{
				auto camStatus = camera->firmware->contextualLock();
				ImGui::Text("Camera #%u: %s", camera->id, camStatus->text.c_str());
				/* if (camStatus->code == FW_STATUS_UPDATED)
				{
					ImGui::SetCursorPosX(button.x);
					if (ImGui::Button("Restart", button))
					{ // Restart somehow? Update auto-restarts for most update types
					}
				} */
			}
			else if (camera->selectedForFirmware)
				ImGui::Text("Camera #%u is not part of the firmware update.", camera->id);
		}
	}
	else if (cameraFWUpdateSetup)
	{
		BeginSection("Camera Firmware Update");

		// Static, shared between firmware flashing popups
		static std::string firmwareFile = "Firmware File";
		ImGui::AlignTextToFramePadding();
		ImGui::Text("%s", std::filesystem::path(firmwareFile).filename().c_str());
		ImGui::SetItemTooltip("%s", firmwareFile.c_str());
		SameLineTrailing(button.x);
		if (ImGui::Button("Select", button))
		{
			threadPool.push([](int id)
			{
				if (!NFD_Init())
				{ // Thread-specific init
					SignalErrorToUser(asprintf_s("Failed to initialise File Picker: %s", NFD_GetError()));
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
					NFD_FreePath(outPath);
				}
				else if (result == NFD_ERROR)
				{
					SignalErrorToUser(asprintf_s("Failed to use File Picker: %s", NFD_GetError()));
				}

				NFD_Quit();

				GetUI().RequestUpdates();
			});
		}

		for (auto &camera : state.cameras)
		{
			if (camera->selectedForFirmware)
				ImGui::Text("Camera #%u set to update", camera->id);
		}

		if (ImGui::Button("Flash", SizeWidthFull()))
		{
			std::vector<std::shared_ptr<TrackingCameraState>> firmwareUpdateCameras;
			for (auto &camera : state.cameras)
			{
				if (camera->selectedForFirmware)
					firmwareUpdateCameras.push_back(camera); // new shared_ptr
			}
			cameraFWUpdateState = CamerasFlashFirmwareFile(firmwareUpdateCameras, firmwareFile);
		}

		EndSection();
	}

	// TODO: Ask controller via USB to automatically switch to Bootloader 2/3
	// So for now, just always show FW update for controllers
	//if (controllerFWUpdate)
	{
		BeginSection("Controller Firmware Update");

		// Static, shared between firmware flashing popups
		static std::string firmwareFile = "Firmware File";
		ImGui::AlignTextToFramePadding();
		ImGui::Text("%s", std::filesystem::path(firmwareFile).filename().c_str());
		ImGui::SetItemTooltip("%s", firmwareFile.c_str());
		SameLineTrailing(button.x);
		if (ImGui::Button("Select", button))
		{
			threadPool.push([](int id)
			{
				if (!NFD_Init())
				{ // Thread-specific init
					SignalErrorToUser(asprintf_s("Failed to initialise File Picker: %s", NFD_GetError()));
					return;
				}

				const int filterLen = 1;
				nfdfilteritem_t filterList[filterLen] = {
					{"Controller Program Binary", "bin"},
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
					NFD_FreePath(outPath);
				}
				else if (result == NFD_ERROR)
				{
					SignalErrorToUser(asprintf_s("Failed to use File Picker: %s", NFD_GetError()));
				}

				NFD_Quit();

				GetUI().RequestUpdates();
			});
		}

		// TODO: Ask controller via USB to automatically switch to Bootloader 3/3
		// Only offer option to flash if it doesn't remove the controller itself...
		/* for (auto &controller : state.controllers)
		{
			if (controller->firmware)
			{
				auto status = controller->firmware->contextualLock();
				ImGui::Text("Controller #%d: %s", controller->id, status->text.c_str());
				SameLineTrailing(button.x);
				if (!status->complete && ImGui::Button("Abort", button))
				{
					status->abort.request_stop();
				}
				if (status->complete && ImGui::Button("Close", button))
				{
					controller->firmware = nullptr;
				}
			}
			else if (controller->selectedForFirmware)
			{
				ImGui::Text("Controller #%d set to update", controller->id);
				SameLineTrailing(button.x);
				if (ImGui::Button("Flash", button))
				{
					controller->firmware = ControllerFlashFirmwareFile(controller, firmwareFile);
					controller->selectedForFirmware = false;
				}
			}
		} */
		
		if (controllerFWUpdateState)
		{
			auto status = controllerFWUpdateState->contextualLock();
			ImGui::Text("%s", status->text.c_str());
			SameLineTrailing(button.x);
			if (!status->complete && ImGui::Button("Abort", button))
			{
				status->abort.request_stop();
			}
			if (status->complete && ImGui::Button("Close", button))
			{
				controllerFWUpdateState = nullptr;
			}
		}
		else
		{
			if (ImGui::Button("Flash Controller in Bootloader", SizeWidthFull()))
			{
				controllerFWUpdateState = ControllerFlashFirmwareFile(nullptr, firmwareFile);
			}
			ImGui::SetItemTooltip("The controller already has to be running the bootloader, appearing as a WinChipHead USB device.\n"
				"To get a controller to reboot into the bootloader, hold the 'Flash' button (middle) for one second.");
		}

		EndSection();
	}

	ImGui::SeparatorText("IMU Devices");
	// TODO: IMU Display: Include indication of connection stability, rssi, battery, etc.

	if (state.mode != MODE_Device)
		ImGui::Text("Not in Device Mode! Connect to Hardware to see IMU Devices");
	
	auto imuProviders = state.imuProviders.contextualRLock();
	bool hasIMUDevices = false;
	for (auto &provider : *imuProviders)
	{
		std::string desc = provider->getDescriptor();
		if (desc.empty() && provider->devices.empty()) continue;
		hasIMUDevices = true;
		ImGui::Text("%s", desc.c_str());
		ImGui::Indent();
		for (auto &imu : provider->devices)
		{
			if (!imu)
				ImGui::Text("Unassigned IMU!");
			else if (imu->lastConnected == TimePoint_t::min())
				ImGui::Text("%s (Disconnected)", imu->getDescriptor().c_str());
			else
			{
				if (imu->isFused)
					ImGui::Text("%s - %d fused samples", imu->getDescriptor().c_str(), (int)imu->samplesFused.getView().size());
				else
					ImGui::Text("%s - %d raw samples", imu->getDescriptor().c_str(), (int)imu->samplesRaw.getView().size());
				if (ImGui::BeginItemTooltip())
				{
					if (imu->hasBattery)
					{
						if (imu->isPlugged)
							ImGui::Text("Charging %d%%, %.2fV", (int)(imu->batteryLevel*100), imu->batteryVolts);
						else
							ImGui::Text("Battery %d%%, %.2fV", (int)(imu->batteryLevel*100), imu->batteryVolts);
					}
					else if (imu->isPlugged)
						ImGui::TextUnformatted("Wired");
					else
						ImGui::TextUnformatted("Power Source Unknown");
					ImGui::Text("Signal Strength %d%%", (int)(imu->signalStrength*100));
					ImGui::EndTooltip();
				}
			}
		}
		ImGui::Unindent();
	}
	if (state.mode == MODE_Device && !hasIMUDevices)
		ImGui::Text("No IMU Devices or Receivers connected!");

	ImGui::End();
}