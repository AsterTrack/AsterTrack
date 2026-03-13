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

#include "io/integrations.hpp"
#include "io/vrpn.hpp"
#include "server.hpp"

#include "util/log.hpp"

void IntegrationsInit(IntegrationsState &state, const GeneralConfig &config)
{
	auto io_lock = std::unique_lock(state.mutex);
	state.vrpn.enabled = config.integrations.vrpn_auto_enable;

	IntegrationsReconfigureVRPN(state, config);
}

void IntegrationsCleanup(IntegrationsState &state, const GeneralConfig &config)
{
	auto io_lock = std::unique_lock(state.mutex);
	state.vrpn.enabled = false;

	IntegrationsReconfigureVRPN(state, config);
}

void IntegrationsReconfigureVRPN(IntegrationsState &state, const GeneralConfig &config)
{
	state.vrpn.trackers.clear();
	state.vrpn.server = nullptr;
	if (!state.vrpn.enabled) return;
	// Setup VRPN Connection
	auto &cfg = config.integrations;
	std::string host = cfg.vrpn_host.empty()? "localhost" : cfg.vrpn_host;
	int port = cfg.vrpn_port? cfg.vrpn_port : vrpn_DEFAULT_LISTEN_PORT_NO;
	state.vrpn.host = host + ":" + std::to_string(port);
	std::string cname = cfg.vrpn_host + ":" + std::to_string(port);
	state.vrpn.server = opaque_ptr<vrpn_Connection>(vrpn_create_server_connection(cname.c_str()));
	state.vrpn.server->setAutoDeleteStatus(true);
}

void IntegrationsUpdate(IntegrationsState &state, ServerState &server)
{ // Gets called regularly while in a mode, even when not streaming
	auto io_lock = std::unique_lock(state.mutex);

	// Timestamp for camera position
	TimePoint_t timestamp = sclock::now();
	OptFrameNum frame = server.pipeline.frameNum.load();
	if (frame > 0)
	{ // Replay may not be realtime, so take last frame time
		auto view = server.pipeline.record.frames.getView();
		if (frame < view.endIndex())
			timestamp = view[frame]->time;
	}

	if (state.vrpn.enabled)
	{
		for (auto &tracker : server.trackerConfigs)
		{
			auto io_tracker = state.vrpn.trackers.find(tracker.id);
			if (!tracker.exposed)
			{
				if (io_tracker != state.vrpn.trackers.end())
				{ // Disconnect after expose was disabled
					LOG(LIO, LInfo, "VRPN Tracker '%s' is no longer exposed!", tracker.label.c_str());
					if (io_tracker->second->markedConnected)
						tracker.connected--;
					state.vrpn.trackers.erase(io_tracker);
					ServerUpdateTrackerConditions(server, tracker);
				}
				continue;
			}
			if (io_tracker == state.vrpn.trackers.end())
			{
				LOG(LIO, LInfo, "Exposing VRPN Tracker '%s'", tracker.label.c_str());
				auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(tracker.id, tracker.label.c_str(), state.vrpn.server.get());
				io_tracker = state.vrpn.trackers.insert({ tracker.id, std::move(vrpn_tracker) }).first;
			}
			if (!io_tracker->second->markedConnected && io_tracker->second->isConnected())
			{
				LOG(LIO, LInfo, "VRPN Tracker '%s' has been connected!", tracker.label.c_str());
				io_tracker->second->markedConnected = true;
				tracker.connected++;
				ServerUpdateTrackerConditions(server, tracker);
			}
			else if (io_tracker->second->markedConnected && !io_tracker->second->isConnected())
			{
				LOG(LIO, LInfo, "VRPN Tracker '%s' has been disconnected!", tracker.label.c_str());
				io_tracker->second->markedConnected = false;
				tracker.connected--;
				ServerUpdateTrackerConditions(server, tracker);
			}
		}

		// Add cameras as trackers to give clients an opportunity to display them as references
		// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
		for (const auto &camera : server.pipeline.cameras)
		{
			auto io_tracker = state.vrpn.trackers.find(camera->id);
			if (io_tracker == state.vrpn.trackers.end())
			{
				std::string path = asprintf_s("AsterCamera_%d", camera->index);
				auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(camera->id, path.c_str(), state.vrpn.server.get());
				io_tracker = state.vrpn.trackers.insert({ camera->id, std::move(vrpn_tracker) }).first;
			}
			// Send static position of cameras as reference for clients
			// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
			io_tracker->second->updatePose(0, timestamp, camera->calib.transform.cast<float>());
		}

		LOG(LIO, LTrace, "Updating individual trackers!\n");
		for (auto &trackerIO : state.vrpn.trackers)
		{ // Handle mainloop (mostly ping-pong) here as well
			trackerIO.second->mainloop();
		}
	}
}

void IntegrationsReceive(IntegrationsState &state, const ServerState &server)
{
	auto io_lock = std::unique_lock(state.mutex);

	if (state.vrpn.enabled)
	{ // Fetch incoming packets
		LOG(LIO, LTrace, "Updating VRPN server connection to fetch IMU packets!\n");
		state.vrpn.server->mainloop();
		if (!state.vrpn.server->doing_okay())
			LOG(LIO, LWarn, "VRPN Connection Error!\n");
		else
			LOG(LIO, LTrace, "     Server connection is doing ok!\n");
	}
}

void IntegrationsSendFrame(IntegrationsState &state, const ServerState &server, std::shared_ptr<FrameRecord> &frame)
{
	auto io_lock = std::unique_lock(state.mutex);

	if (state.vrpn.enabled)
	{
		for (auto &trackRecord : frame->trackers)
		{
			if (!trackRecord.result.isDetected() && !trackRecord.result.isTracked()) continue;
			auto io_tracker = state.vrpn.trackers.find(trackRecord.id);
			if (io_tracker == state.vrpn.trackers.end()) continue;
			// TODO: Send both poseObserved and poseFiltered?
			io_tracker->second->updatePose(0, frame->time, trackRecord.poseFiltered);
		}

		LOG(LIO, LTrace, "Updating server connection to push tracker packets!\n");
		state.vrpn.server->send_pending_reports();
		if (!state.vrpn.server->doing_okay())
		{
			LOG(LIO, LWarn, "VRPN Connection Error!\n");
		}
		else
			LOG(LIO, LTrace, "     Server connection is doing ok!\n");
	}
}