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

void InterfaceState::UpdateDevices(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	if (state.cameras.empty())
	{
		ImGui::End();
		return;
	}

	ImGui::TextUnformatted("Largely still placeholder!");

	int pad = 16; // Padding for controllers
	ImVec2 controllerSize = ImVec2(128, 128);
	ImVec2 cameraSize = ImVec2(32, 64);

	auto drawCamera = [&](TrackingCameraState &camera, float width)
	{
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
			Color color = GetUI().getStatusColor(*viewIt->second.camera);
			showSolidEllipse(Eigen::Vector2f::Zero(), Eigen::Vector2f::Constant(0.8f), color);
		}, (void*)(intptr_t)camera.id);

		// Detail text
		//pos.x = bb.Max.x + ImGui::GetStyle().ItemInnerSpacing.x;
		ImGui::SetCursorPos(pos);
		ImGui::Indent(bb.GetWidth() + ImGui::GetStyle().ItemInnerSpacing.x);

		auto &wireless = camera.config.wireless;
		if (wireless.connected)
		{
			ImGui::SetCursorPosY(ImGui::GetCursorPosY()+ImGui::GetStyle().FramePadding.y);
			ImGui::Image(icons().wireless, ImVec2(ImGui::GetFontSize()*6/5, ImGui::GetFontSize()));
			if (ImGui::BeginItemTooltip())
			{
				ImGui::Text("On wireless network '%s' with IP %s", wireless.SSID.c_str(), wireless.IP.c_str());
				if (wireless.ServerStatus && camera.client && camera.client->ready)
				{
					ImGui::Text("The server is running and connected.");
				}
				else if (wireless.ServerStatus)
				{
					ImGui::Text("The server is running but not connected.");
				}
				ImGui::EndTooltip();
			}
			ImGui::SameLine();
		}

		{ // Sync
			// TODO: Allow detaching from SyncGroup controller is part of via toggle to switch to free-running
			// This is currently part of camera settings but it should be part of the SyncGroup configuration UI rework
			if (camera.sync)
			{
				ImGui::Text("%dHz", (int)(1000.0f/camera.sync->contextualRLock()->frameIntervalMS));
			}
			else
			{
				ImGui::Text("%dHz", state.controllerConfig.framerate);
			}
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
			if (ImGui::BeginCombo("##Config", asprintf_s("%s##config", getConfigLabel(index).c_str()).c_str()))
			{
				ImGui::PopStyleColor(2);
				ImGui::PopStyleVar();
				for (int i = 0; i < state.cameraConfig.configurations.size(); i++)
				{
					if (ImGui::Selectable(getConfigLabel(i).c_str(), i == index))
					{
						state.cameraConfig.cameraConfigs[camera.id] = i;
						camera.pipeline->mode = getCameraMode(state, camera.id);
						DeviceUpdateCameraSetup(state, camera);
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
			ImVec2 iconSize(ImGui::GetTextLineHeight(), ImGui::GetTextLineHeight());
			OnDemandItem *icon = AddOnDemandIcon("icon", iconSize, iconSize, [](const ImDrawList* dl, const ImDrawCmd* dc)
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
				Color color = GetUI().getStatusColor(controller);
				showSolidEllipse(Eigen::Vector2f::Zero(), Eigen::Vector2f::Constant(0.8f), color);
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
				{
					ImGui::Text("Free Running");
				}
				else if (sync_lock->source == SYNC_INTERNAL)
				{
					ImGui::Text("Lock");
					ImGui::SameLine(colX);
					ImGui::Text("%dHz", (int)(1000.0f/sync_lock->frameIntervalMS));
				}
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
	int freeCols = (int)std::ceil(numFreeCameras/8.0f);

	if (ImGui::BeginTable("Devices", state.controllers.size()+freeCols,
		ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY | ImGuiTableFlags_PadOuterX | ImGuiTableFlags_BordersInner))
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
		for (int r = 0; r < 8; r++)
		{
			ImGui::TableNextRow();
			for (auto &controller : state.controllers)
			{
				ImGui::TableNextColumn();
				auto &camera = controller->cameras[r];
				if (camera)
				{
					drawCamera(*camera, ImGui::GetColumnWidth());
				}
				else
				{
					ImGui::Image((isDarkMode? lightModeIcons : darkModeIcons).camera, cameraSize);
				}
			}
			for (int i = 0; i < freeCols; i++)
			{
				ImGui::TableNextColumn();
				if (free >= numFreeCameras)
				{ // No more to draw
					ImGui::Image((isDarkMode? lightModeIcons : darkModeIcons).camera, cameraSize);
					continue;
				}
				int cnt = 0;
				for (auto &camera : state.cameras)
				{
					if (camera->controller) continue;
					if (cnt++ == free)
					{ // Draw next free camera
						drawCamera(*camera, ImGui::GetColumnWidth());
						free++;
						break;
					}
				}
			}
		}

		ImGui::EndTable();
	}

	ImGui::End();
}