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

	if (ImGui::BeginTable("WirelessCameras", 4,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
	{
		ImVec2 iconSize(ImGui::GetFontSize()*6/5,ImGui::GetFontSize());
		ImGui::TableSetupColumn("##Wifi", ImGuiTableColumnFlags_WidthFixed, iconSize.x);
		ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 2);
		ImGui::TableSetupColumn("##Server", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());

		auto toggleAllConfigFlags = [&](WirelessConfig flag)
		{
			WirelessConfig toggle = flag;
			for (auto &camera : state.cameras)
				if (camera->config.wireless.config & flag)
					toggle = WIRELESS_CONFIG_NONE;
			for (auto &camera : state.cameras)
			{
				if ((camera->config.wireless.config & flag) != toggle)
				{
					camera->config.wireless.config = (WirelessConfig)((camera->config.wireless.config & ~flag) | toggle);
					CameraUpdateWireless(state, *camera);
				}
			}
		};
		auto toggleOneConfigFlags = [&](const char* str_id, ImTextureID active, ImTextureID inactive,
			TrackingCameraState::Wireless &wireless, WirelessConfig flag, WirelessConfigStatus status)
		{
			ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ChildBg));
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
			bool set = wireless.config & flag;
			bool stat = status == WIRELESS_STATUS_ENABLED || status == WIRELESS_STATUS_CONNECTED;
			ImGui::BeginDisabled(set != stat && dtMS(wireless.sendTime, sclock::now()) < 1000);
			bool changed = ImGui::ImageButton(str_id, stat? active : inactive, iconSize);
			if (changed)
			{
				if (stat)
					wireless.config = (WirelessConfig)(wireless.config & ~flag);
				else
					wireless.config = (WirelessConfig)(wireless.config | flag);
			}
			ImGui::EndDisabled();
			ImGui::PopStyleVar();
			ImGui::PopStyleColor();
			return changed;
		};
	
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
			// Wireless toggle column header
			ImVec2 iconSize(ImGui::GetFontSize()*6/5,ImGui::GetFontSize());
			iconHeader(0, icons().wireless, iconSize);
			if (ImGui::IsItemClicked())
				toggleAllConfigFlags(WIRELESS_CONFIG_WIFI);
			for (int i = 1; i < 3; i++)
			{ // Three normal headers
				ImGui::TableSetColumnIndex(i);
				ImGui::TableHeader(ImGui::TableGetColumnName(i));
			}
			// Server toggle column header
			iconHeader(3, icons().server, iconSize);
			if (ImGui::IsItemClicked())
				toggleAllConfigFlags(WIRELESS_CONFIG_SERVER);
		}

		for (auto &camera : state.cameras)
		{
			auto &config = camera->config.wireless;
			bool changed = false;
			ImGui::PushID(camera->id); // Required to differentiate controls in multiple rows
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			changed |= toggleOneConfigFlags("##Wireless", icons().wireless, icons().controller, config, WIRELESS_CONFIG_WIFI, config.wifiStatus);
			ImGui::TableNextColumn();
			{
				ImGui::Text("%d", camera->id);
			}
			ImGui::TableNextColumn();
			{
				bool hasError = !config.error.empty() && 
					(config.wifiStatus & (WIRELESS_STATUS_DISABLED | WIRELESS_STATUS_ERROR) || dtMS(config.errorTime, sclock::now()) > 4000);
				if (hasError)
					ImGui::Text("%s", config.error.c_str());
				else if (config.wifiStatus == WIRELESS_STATUS_ENABLED)
					ImGui::TextUnformatted("Connecting...");
				else if (config.wifiStatus == WIRELESS_STATUS_CONNECTED)
					ImGui::Text("%s", config.IP.c_str());
				else
					ImGui::TextUnformatted("Not connected.");
	
				if (config.wifiStatus == WIRELESS_STATUS_ENABLED)
					ImGui::SetItemTooltip("Wifi is enabled, but not connected yet.");
				else if (config.wifiStatus == WIRELESS_STATUS_CONNECTED)
					ImGui::SetItemTooltip("Camera is connected to network '%s' with IP '%s'", config.SSID.c_str(), config.IP.c_str());
				else if (config.wifiStatus == WIRELESS_STATUS_ERROR)
					ImGui::SetItemTooltip("Camera either does not have packages requried for wireless, or it has been configured to block wireless connections!");
				else if (config.wifiStatus == WIRELESS_STATUS_DISABLED)
					ImGui::SetItemTooltip("Wifi is disabled.");
				
			}
			ImGui::TableNextColumn();
			changed |= toggleOneConfigFlags("##Server", icons().server, icons().controller, config, WIRELESS_CONFIG_SERVER, config.serverStatus);
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