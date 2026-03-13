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

	bool vrpnEnabled = state.io.vrpn.enabled;
	if (ImGui::CollapsingHeader(vrpnEnabled? "VRPN (enabled)###vrpnHdr" : "VRPN (disabled)###vrpnHdr", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::BeginDisabled(vrpnEnabled);

		std::string defaultHost = "";
		TextProperty("VRPN Host", &state.config.integrations.vrpn_host, &defaultHost);
		int defaultPort = vrpn_DEFAULT_LISTEN_PORT_NO;
		ScalarProperty<int>("VRPN Port", nullptr, &state.config.integrations.vrpn_port, &defaultPort, 1024, 65535, 0);

		ImGui::EndDisabled();

		BooleanProperty("Auto-Enable VRPN", &state.config.integrations.vrpn_auto_enable, nullptr);

		if (BooleanProperty("Enable VRPN Server", &vrpnEnabled, nullptr))
		{
			std::unique_lock io_lock(state.io.mutex);
			state.io.vrpn.enabled = vrpnEnabled;
			IntegrationsReconfigureVRPN(state.io, state.config);
		}
		if (vrpnEnabled)
		{
			auto io_lock = std::unique_lock(state.io.mutex);

			if (!state.io.vrpn.server->connected())
				ImGui::Text("'%s': No clients connected.", state.io.vrpn.host.c_str());
			else if (!state.io.vrpn.server->doing_okay())
				ImGui::Text("'%s': A client connection broke.", state.io.vrpn.host.c_str());
			else
				ImGui::Text("'%s': Has connected clients.", state.io.vrpn.host.c_str());

			if (ImGui::TreeNodeEx("Exposed Connections"))
			{ // These may not all be real trackers, but also e.g. cameras
				for (auto &tracker : state.io.vrpn.trackers)
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