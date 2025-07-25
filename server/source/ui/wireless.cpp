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

#include "ui.hpp"

#include "device/tracking_camera.hpp"

void InterfaceState::UpdateWirelessSetup(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	BeginSection("Wifi Credentials");
	if (ImGui::TreeNode("wpa_supplicant.conf"))
	{
		ImGui::InputTextMultiline("##wpa_supplicant", &state.wpa_supplicant_conf, ImVec2(ImGui::GetContentRegionAvail().x, 0));
		if (ImGui::Button("Reset") || state.wpa_supplicant_conf.empty())
		{
			state.wpa_supplicant_conf = "network={\n\tssid=\"SSID\"\n\t#psk=PSK\n\tpsk=\"PASSWORD\"\n}";
		}
		ImGui::SameLine();
		ImGui::TextUnformatted("See 'wpa_supplicant.conf' for expected format");
		if (state.wpa_supplicant_conf.size() > 2000)
		{
			ImGui::Text("Length of %ld is over limit of 2000!", state.wpa_supplicant_conf.size());
		}
		ImGui::TreePop();
	}
	EndSection();

	BeginSection("Wireless Cameras");

	if (ImGui::BeginTable("WirelessCameras", 5,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
	{
		ImGui::TableSetupColumn("##Wifi", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("SSID", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("IP", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("##Server", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());

		// Draw headers automatically
		//ImGui::TableHeadersRow();
		{ // Draw headers manually with icons and interaction to toggle columns as a whole
			ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
			auto iconHeader = [](int index, ImTextureID icon, ImVec2 size)
			{
				ImGui::TableSetColumnIndex(index);
				// Center icon
				ImGui::SetCursorPosX(ImGui::GetCursorPosX()+(ImGui::GetColumnWidth(index)-size.x)/2);
				ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
				ImGui::Image(icon, size);
				ImGui::PopStyleVar();
				// Draw Label if it exists, and provide ID
				ImGui::SameLine();
				ImGui::TableHeader(ImGui::TableGetColumnName(index));
			};
			auto toggleColumnProperties = [&](bool &(*getProperty)(TrackingCameraState::Wireless &wireless), bool def = true)
			{
				bool toggle = def;
				for (auto &camera : state.cameras)
					if (getProperty(camera->config.wireless) == def)
						toggle = !def;
				for (auto &camera : state.cameras)
				{
					if (getProperty(camera->config.wireless) != toggle)
					{
						getProperty(camera->config.wireless) = toggle;
						CameraUpdateWireless(state, *camera);
					}
				}
			};
			// Wireless toggle column header
			ImVec2 iconSize(ImGui::GetFontSize()*6/5,ImGui::GetFontSize());
			iconHeader(0, icons().wireless, iconSize);
			if (ImGui::IsItemClicked())
				toggleColumnProperties([](TrackingCameraState::Wireless &wireless) -> bool& { return wireless.enabled; }, true);
			for (int i = 1; i < 4; i++)
			{ // Three normal headers
				ImGui::TableSetColumnIndex(i);
				ImGui::TableHeader(ImGui::TableGetColumnName(i));
			}
			// Server toggle column header
			iconHeader(4, icons().server, iconSize);
			if (ImGui::IsItemClicked())
				toggleColumnProperties([](TrackingCameraState::Wireless &wireless) -> bool& { return wireless.Server; }, true);
		}

		for (auto &camera : state.cameras)
		{
			auto &config = camera->config.wireless;
			bool changed = false;
			ImGui::PushID(camera->id); // Required to differentiate controls in multiple rows
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			{
				ImGui::BeginDisabled(config.updating);
				changed |= ImGui::Checkbox("##Wireless", &config.enabled);
				ImGui::EndDisabled();
				//if (config.updating)
					// TODO: Display loading icon while camera is connecting to wireless network?
			}
			ImGui::TableNextColumn();
			{
				ImGui::AlignTextToFramePadding();
				ImGui::Text("%d", camera->id);
			}
			ImGui::TableNextColumn();
			{
				ImGui::AlignTextToFramePadding();
				if (config.updating && config.enabled && !config.connected)
					ImGui::TextUnformatted("Connecting...");
				else if (config.updating)
					ImGui::TextUnformatted("Updating...");
				else if (config.failed)
					ImGui::Text("Failed: '%s", config.error.c_str());
				else if (config.connected)
					ImGui::Text("%s", config.SSID.c_str());
				else
					ImGui::TextUnformatted("Not connected.");
			}
			ImGui::TableNextColumn();
			if (config.connected)
			{
				ImGui::AlignTextToFramePadding();
				ImGui::Text("%s", config.IP.c_str());
			}
			ImGui::TableNextColumn();
			{
				ImGui::BeginDisabled(!config.connected);
				changed |= ImGui::Checkbox("##Server", &config.Server);
				ImGui::EndDisabled();
			}
			/* bool select = selectedCamera == camera;
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
				selectedCamera = camera; */
			ImGui::PopID();

			if (changed)
			{
				CameraUpdateWireless(state, *camera);
			}
		}
		ImGui::EndTable();
	}

	EndSection();

	ImGui::End();
}