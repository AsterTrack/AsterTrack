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

#include "imgui/imgui_custom.hpp"
#include "imgui/imgui_custom.hpp"

#include "io/vrpn.hpp"


void InterfaceState::UpdateIntegrations(InterfaceWindow &window)
{
	if (!window.open)
		return;
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	if (ImGui::CollapsingHeader("Exposed Trackers", ImGuiTreeNodeFlags_DefaultOpen))
	{
		for (auto &tracker : state.trackerConfigs)
		{
			ImGui::PushID(tracker.id);
			ImGui::AlignTextToFramePadding();
			ImGui::Text("Tracker %d: %s", tracker.id, tracker.label.c_str());
			SameLineTrailing(ImGui::GetFrameHeight());
			if (!tracker.exposed && ImGui::ArrowButton("Expose", ImGuiDir_Right))
				tracker.exposed = true;
			else if (tracker.exposed && CrossButton("Expose"))
				tracker.exposed = false;
			ImGui::PopID();
		}
	}

	if (ImGui::CollapsingHeader(state.io.useVRPN? "VRPN (enabled)###vrpnHdr" : "VRPN (disabled)###vrpnHdr", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::Button(state.io.useVRPN? "Stop VRPN Server###vrpnUse" : "Start VRPN Server###vrpnUse", SizeWidthFull()))
		{
			if (state.io.useVRPN)
				ResetIO(state);
			else
				SetupIO(state);
		}
		if (state.io.useVRPN)
		{
			auto io_lock = std::unique_lock(state.io.mutex);
			if (state.io.connectionHost.empty() || state.io.connectionHost[0] != ':')
				ImGui::Text("VRPN Server '%s'", state.io.connectionHost.c_str());
			else
			 	ImGui::Text("VRPN Server 'localhost%s'", state.io.connectionHost.c_str());

			if (!state.io.vrpn_server->connected())
			{
				ImGui::TextUnformatted("No clients connected.");
			}
			else if (!state.io.vrpn_server->doing_okay())
			{
				ImGui::TextUnformatted("A client lost connection!");
			}
			else
			{
				ImGui::TextUnformatted("At least one client connected!");
			}

			if (ImGui::TreeNodeEx("Exposed Connections", ImGuiTreeNodeFlags_DefaultOpen))
			{ // These may not all be real trackers, but also e.g. cameras
				for (auto &tracker : state.io.vrpn_trackers)
				{
					const char *connectionStatus = "not connected";
					if (tracker.second->isConnected())
					{ // This is based on last ping
						if (!tracker.second->connectionPtr()->doing_okay())
							connectionStatus = "some broke"; // At least one connection broke
						else if (tracker.second->connectionPtr()->connected())
							connectionStatus = "connected"; // At least one connected
						else // Have recent ping, but nothing connected, not sure if possible
							connectionStatus = "was connected";
					}
					ImGui::Text("%s - %s", tracker.second->path.c_str(), connectionStatus);
				}
				ImGui::TreePop();
			}
		}
	}

	ImGui::End();
}