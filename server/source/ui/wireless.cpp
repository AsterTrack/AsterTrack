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

void InterfaceState::
UpdateWirelessSetup(InterfaceWindow &window)
{
	if (!window.open)
		return;
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	static std::string wpa_supplicant_edit;
	if (ImGui::CollapsingHeader("Wifi Credentials"))
	{
		ImGui::AlignTextToFramePadding();
		if (wpa_supplicant_edit.empty())
		{
			if (state.wpa_supplicant_conf.empty())
				ImGui::Text("wpa_supplicant.conf not configured");
			else
				ImGui::Text("wpa_supplicant.conf configured");

			SameLineTrailing(SizeWidthDiv3().x); 
			if (ImGui::Button("Edit", SizeWidthDiv3()))
			{
				if (state.wpa_supplicant_conf.empty())
					wpa_supplicant_edit = "network={\n\tssid=\"SSID\"\n\t#psk=PSK\n\tpsk=\"PASSWORD\"\n}";
				else
					wpa_supplicant_edit = state.wpa_supplicant_conf;
			}
		}
		else
		{
			ImGui::TextUnformatted("Editing wpa_supplicant.conf:");

			ImGui::InputTextMultiline("##wpa_supplicant", &wpa_supplicant_edit, ImVec2(ImGui::GetContentRegionAvail().x, 0));

			if (wpa_supplicant_edit.size() > 2000)
			{ // TODO: wpa_supplicant limited by CTRL_TRANSFER_SIZE and USBD_CTRL_MAX_PACKET_SIZE (1/2)
				ImGui::Text("Length of %ld is over limit of 2000!", wpa_supplicant_edit.size());
			}

			if (ImGui::Button("Cancel", SizeWidthDiv3()))
			{
				wpa_supplicant_edit.clear();
			}
			ImGui::SameLine();
			if (ImGui::Button("Delete", SizeWidthDiv3()))
			{
				state.wpa_supplicant_conf.clear();
				wpa_supplicant_edit.clear();
			}
			ImGui::SameLine();
			if (ImGui::Button("Accept", SizeWidthDiv3()))
			{
				state.wpa_supplicant_conf = wpa_supplicant_edit;
				wpa_supplicant_edit.clear();
			}
		}
	}

	if (ImGui::CollapsingHeader("Server Config"))
	{
		ImGui::InputText("Server Host", &state.server.host);
		ImGui::InputText("Server Port", &state.server.portSet);
		if (!state.server.portUsed.empty())
		{
			ImGui::AlignTextToFramePadding();
			ImGui::Text("Server is running on port %s", state.server.portUsed.c_str());
			if (state.server.portSet != state.server.portUsed)
			{
				SameLineTrailing(ImGui::GetFrameHeight());
				if (RetryButton("RestartServer"))
				{
					// Cheating a bit. Updating portUsed before restarting the server so we can notify cameras of the new port using the old server
					state.server.portUsed = state.server.portSet;
					for (auto &camera : state.cameras)
					{
						if (camera->config.wireless.lastConfig & WIRELESS_CONFIG_SERVER)
						{
							camera->config.wireless.setConfig = camera->config.wireless.lastConfig;
							CameraUpdateWireless(state, *camera);
						}
					}
					StopWirelessServer(state);
					StartWirelessServer(state);
				}
				ImGui::SetItemTooltip("Restart the server on the new port.\nTries to communicate new port to currently connected cameras.");
			}
		}
	}

	ImGui::Separator();

	if (ImGui::BeginTable("WirelessCameras", 7,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
	{
		ImGui::TableSetupColumn("##Wifi", ImGuiTableColumnFlags_WidthFixed, iconSize().x);
		ImGui::TableSetupColumn("##SSH", ImGuiTableColumnFlags_WidthFixed, iconSize().x);
		ImGui::TableSetupColumn("##Server", ImGuiTableColumnFlags_WidthFixed, iconSize().x);
		ImGui::TableSetupColumn("##RTData", ImGuiTableColumnFlags_WidthFixed, iconSize().x);
		ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("##Actions", ImGuiTableColumnFlags_WidthFixed, iconSize().x);

		auto toggleAllConfigFlags = [&](WirelessConfig flag)
		{
			WirelessConfig toggle = flag;
			for (auto &camera : state.cameras)
				if (camera->config.wireless.lastConfig & flag)
					toggle = WIRELESS_CONFIG_NONE;
			for (auto &camera : state.cameras)
			{
				if ((camera->config.wireless.lastConfig & flag) != toggle)
				{
					camera->config.wireless.setConfig = (WirelessConfig)((camera->config.wireless.lastConfig & ~flag) | toggle);
					CameraUpdateWireless(state, *camera);
				}
			}
		};
		auto toggleOneConfigFlags = [&](const char* str_id, ImTextureID active, ImTextureID inactive,
			TrackingCameraState::Wireless &wireless, WirelessConfig flag)
		{
			ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ChildBg));
			ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
			ImGui::BeginDisabled(wireless.setConfig != wireless.lastConfig && dtMS(wireless.sendTime, sclock::now()) < 2000);
			bool changed = ImGui::ImageButton(str_id, wireless.lastConfig & flag? active : inactive, iconSize());
			if (changed)
			{
				if (wireless.lastConfig & flag)
					wireless.setConfig = (WirelessConfig)(wireless.lastConfig & ~flag);
				else
					wireless.setConfig = (WirelessConfig)(wireless.lastConfig | flag);
			}
			ImGui::EndDisabled();
			ImGui::PopStyleVar();
			ImGui::PopStyleColor();
			return changed;
		};
		auto wifiStatusTooltip = [](const TrackingCameraState::Wireless &config){
			if (ImGui::BeginItemTooltip())
			{
				ImGui::Text("Enable wifi connection of camera.");
				ImGui::Text("Status:");

				if (config.wifiStatus == WIRELESS_STATUS_ENABLED)
					ImGui::Text("Wifi is enabled, but not connected yet.");
				else if (config.wifiStatus == WIRELESS_STATUS_CONNECTED)
					ImGui::Text("Camera is connected to network '%s' with IP '%s'", config.SSID.c_str(), config.IP.c_str());
				else if (config.wifiStatus == WIRELESS_STATUS_ERROR)
					ImGui::Text("Camera could not setup a wireless connection!");
				else if (config.wifiStatus == WIRELESS_STATUS_DISABLED)
					ImGui::Text("Wifi is disabled.");
				else // No status (yet) -> no wifi available
					ImGui::Text("Wireless system is not set up.");
				
				if (config.wifiFlags == WIRELESS_STATUS_CONFIGURATED)
					ImGui::Text("Wifi credentials have been stored.");
				else if (config.wifiFlags == (WIRELESS_STATUS_CONFIGURATED | WIRELESS_STATUS_AUTOENABLE))
					ImGui::Text("Wifi is configured to autoconnect.");
				else if (config.wifiFlags == WIRELESS_STATUS_AUTOENABLE)
					ImGui::Text("Wifi is set to autoconnect but credentials are missing.");

				ImGui::EndTooltip();
			}
		};
		auto sshStatusTooltip = [](const TrackingCameraState::Wireless &config){
			if (ImGui::BeginItemTooltip())
			{
				ImGui::Text("Enable SSH access of camera.");
				ImGui::Text("Status:");

				if (config.sshStatus == WIRELESS_STATUS_ENABLED)
					ImGui::Text("SSH is enabled.");
				else if (config.sshStatus == WIRELESS_STATUS_CONNECTED)
					ImGui::Text("SSH is enabled.");
				else if (config.sshStatus == WIRELESS_STATUS_ERROR)
					ImGui::Text("Failed to set up SSH!");
				else if (config.sshStatus == WIRELESS_STATUS_DISABLED)
					ImGui::Text("SSH is disabled.");
				else if (config.sshStatus == WIRELESS_STATUS_UNABLE)
					ImGui::Text("Unable to set up SSH.");
				else // No status (yet) -> no wifi available
					ImGui::Text("Wireless system is not set up.");
				
				if (config.sshFlags & WIRELESS_STATUS_AUTOENABLE)
					ImGui::Text("SSH is set to auto-enable on startup.");

				ImGui::EndTooltip();
			}
		};
		auto serverStatusTooltip = [](const TrackingCameraState::Wireless &config){
			if (ImGui::BeginItemTooltip())
			{
				ImGui::Text("Enable wireless server connection of camera.");
				ImGui::Text("Status:");

				if (config.serverStatus == WIRELESS_STATUS_ENABLED)
					ImGui::Text("Server is enabled, but not connected yet.");
				else if (config.serverStatus == WIRELESS_STATUS_CONNECTED)
					ImGui::Text("Server connection is established.");
				else if (config.serverStatus == WIRELESS_STATUS_ERROR)
					ImGui::Text("Failed to set up server!");
				else if (config.serverStatus == WIRELESS_STATUS_DISABLED)
					ImGui::Text("Server is disabled.");
				else if (config.serverStatus == WIRELESS_STATUS_UNABLE)
					ImGui::Text("Unable to set up server.");
				else // No status (yet) -> no wifi available
					ImGui::Text("Wireless system is not set up.");
				
				if (config.serverFlags == WIRELESS_STATUS_CONFIGURATED)
					ImGui::Text("Server host address has been configured.");
				else if (config.serverFlags == (WIRELESS_STATUS_CONFIGURATED | WIRELESS_STATUS_AUTOENABLE))
					ImGui::Text("Server connection is configured to auto-enable.");
				else if (config.serverFlags == WIRELESS_STATUS_AUTOENABLE)
					ImGui::Text("Server is set to auto-enable but host address has not been configured.");

				ImGui::EndTooltip();
			}
		};
		auto actionDropdown = [&state](int cameraID)
		{
			WirelessAction action = WIRELESS_ACTION_NONE;
			if (ImGui::MenuItem("Apply Config"))
				action = WIRELESS_STORE_CONFIG;
			ImGui::SetItemTooltip("Applies the current config to storage. \nWARNING: This will not delete any wifi credentials stored on the camera, even if you disable wifi.\n");
			if (ImGui::MenuItem("Clear Credentials"))
				action = WIRELESS_CLEAR_CREDS;
			ImGui::SetItemTooltip("Removes any wifi credentials stored on the camera.");
			if (ImGui::MenuItem("Disallow Wifi Permanently"))
				action = WIRELESS_DISALLOW_WIFI_PERMANENTLY;
			ImGui::SetItemTooltip("Disallows Wifi from being enabled on this camera.\nTo undo, you need physical access to the cameras storage (or upload firmware that bypasses this).");
			if (ImGui::MenuItem("Disallow SSH Permanently"))
				action = WIRELESS_DISALLOW_SSH_PERMANENTLY;
			ImGui::SetItemTooltip("Disallows SSH from being enabled on this camera.\nTo undo, you need physical access to the cameras storage (or upload firmware that bypasses this).");
			if (action == WIRELESS_ACTION_NONE)
				return;
			for (auto &camera : state.cameras)
			{
				if (cameraID != 0 && camera->id != cameraID) continue;
				auto &wireless = camera->config.wireless;
				wireless.setConfig = wireless.lastConfig;
				CameraUpdateWireless(state, *camera, action);
			}
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
			iconHeader(0, icons().wireless, iconSize());
			if (ImGui::IsItemClicked() || ImGui::IsItemActive())
				toggleAllConfigFlags(WIRELESS_CONFIG_WIFI);
			ImGui::SetItemTooltip("Enable wifi connection of camera.");
			// SSH toggle column header
			iconHeader(1, icons().ssh, iconSize());
			if (ImGui::IsItemClicked() || ImGui::IsItemActive())
				toggleAllConfigFlags(WIRELESS_CONFIG_SSH);
			ImGui::SetItemTooltip("Enable SSH access of camera.");
			// Server toggle column header
			iconHeader(2, icons().server, iconSize());
			if (ImGui::IsItemClicked() || ImGui::IsItemActive())
				toggleAllConfigFlags(WIRELESS_CONFIG_SERVER);
			ImGui::SetItemTooltip("Enable wireless server connection of camera.");
			// Server toggle column header
			iconHeader(3, icons().rtdata, iconSize());
			if (ImGui::IsItemClicked() || ImGui::IsItemActive())
				toggleAllConfigFlags(WIRELESS_CONFIG_RTDATA);
			ImGui::SetItemTooltip("Enable use of wireless server for realtime data stream of camera.");
			// ID
			ImGui::TableSetColumnIndex(4);
			ImGui::TableHeader(ImGui::TableGetColumnName(4));
			// Status
			ImGui::TableSetColumnIndex(5);
			ImGui::TableHeader(ImGui::TableGetColumnName(5));
			// Actions
			iconHeader(6, icons().vdots, iconSize());
			ImGui::SetItemTooltip("Actions for configuring the cameras.");
			const ImGuiID popupID = ImGui::GetCurrentWindowRead()->GetID("ActionsPopup");
			if (ImGui::IsItemClicked() || ImGui::IsItemActive())
				ImGui::OpenPopup(popupID);
			if (ImGui::BeginComboPopup(popupID, ImRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()), ImGuiComboFlags_PopupAlignLeft))
			{
				actionDropdown(0);
				ImGui::EndCombo();
			}
		}

		for (auto &camera : state.cameras)
		{
			auto &wireless = camera->config.wireless;
			bool changed = false;
			ImGui::PushID(camera->id); // Required to differentiate controls in multiple rows
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			changed |= toggleOneConfigFlags("##Wireless", icons().wireless, icons().config_disabled, wireless, WIRELESS_CONFIG_WIFI);
			wifiStatusTooltip(wireless);
			ImGui::TableNextColumn();
			changed |= toggleOneConfigFlags("##SSH", icons().ssh, icons().config_disabled, wireless, WIRELESS_CONFIG_SSH);
			sshStatusTooltip(wireless);
			ImGui::TableNextColumn();
			changed |= toggleOneConfigFlags("##Server", icons().server, icons().config_disabled, wireless, WIRELESS_CONFIG_SERVER);
			serverStatusTooltip(wireless);
			ImGui::TableNextColumn();
			changed |= toggleOneConfigFlags("##RTData", icons().rtdata, icons().config_disabled, wireless, WIRELESS_CONFIG_RTDATA);
			ImGui::SetItemTooltip("Enable use of wireless server for realtime data stream of camera.");
			ImGui::TableNextColumn();
			{
				ImGui::Text("%d (%d)", camera->id, camera->pipeline->index);
			}
			ImGui::TableNextColumn();
			{
				bool hasError = !wireless.error.empty() && 
					(wireless.wifiStatus & (WIRELESS_STATUS_DISABLED | WIRELESS_STATUS_ERROR) || dtMS(wireless.errorTime, sclock::now()) > 4000);
				if (hasError)
					ImGui::Text("%s", wireless.error.c_str());
				else if (wireless.wifiStatus == WIRELESS_STATUS_ENABLED)
					ImGui::TextUnformatted("Connecting...");
				else if (wireless.wifiStatus == WIRELESS_STATUS_CONNECTED)
					ImGui::Text("%s", wireless.IP.c_str());
				else
					ImGui::TextUnformatted("Not connected.");
	
				wifiStatusTooltip(wireless);
			}
			ImGui::TableNextColumn();
			{
				ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ChildBg));
				ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
				ImGui::BeginDisabled(wireless.setConfig != wireless.lastConfig && dtMS(wireless.sendTime, sclock::now()) < 2000);
				if (BeginIconDropdown("##Actions", icons().vdots, iconSize(), ImGuiComboFlags_PopupAlignLeft))
				{
					actionDropdown(camera->id);
					ImGui::EndCombo();
				}
				ImGui::EndDisabled();
				ImGui::SetItemTooltip("Actions for configuring the camera.");
				ImGui::PopStyleVar();
				ImGui::PopStyleColor();
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

	ImGui::End();
}