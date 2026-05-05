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
#include "io/vmc.hpp"
#include "server.hpp"

#include "util/log.hpp"
#include "util/eigenutil.hpp"


void IntegrationsInit(IntegrationsState &state, const GeneralConfig &config)
{
	auto io_lock = std::unique_lock(state.mutex);
	state.vrpn.enabled = config.integrations.vrpn_auto_enable;
	state.vmc.enabled = config.integrations.vmc_auto_enable;

	IntegrationsReconfigureVRPN(state, config);
	IntegrationsReconfigureVMC(state, config);
}

void IntegrationsCleanup(IntegrationsState &state, const GeneralConfig &config)
{
	auto io_lock = std::unique_lock(state.mutex);
	state.vrpn.enabled = false;
	state.vmc.enabled = false;

	IntegrationsReconfigureVRPN(state, config);
	IntegrationsReconfigureVMC(state, config);
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

void IntegrationsReconfigureVMC(IntegrationsState &state, const GeneralConfig &config)
{
	state.vmc.output = nullptr;
	if (!state.vmc.enabled) return;
	// Setup VRPN Connection
	auto &cfg = config.integrations;
	std::string host = cfg.vmc_host.empty()? "localhost" : cfg.vmc_host;
	int port = cfg.vmc_port? cfg.vmc_port : vmc_DEFAULT_PERFORMER_PORT_NO;
	state.vmc.host = host + ":" + std::to_string(port);
	state.vmc.output = vmc_init_output(host, std::to_string(port));
	vmc_try_open(state.vmc.output, state.vmc.host);
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

	if (state.vmc.enabled && vmc_try_open(state.vmc.output, state.vmc.host))
	{ // Try opening UDP port if not already done
		std::vector<vmc_device> cameras;
		cameras.reserve(server.pipeline.cameras.size());
		for (const auto &camera : server.pipeline.cameras)
		{
			// TODO: Camera role is intended for tracked cameras for virtual production, not as debug for optical tracking systems...
			cameras.emplace_back(VMCRole::Camera,
				asprintf_s("Camera #%u", camera->id),
				camera->calib.transform.cast<float>(),
				(float)getEffectiveFoVD(camera->calib, camera->mode)
			);
		}
		vmc_send_device_packets(state.vmc.output, cameras, timestamp, dtS(server.lastStreamingStart, timestamp));
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

	if (state.vmc.enabled && vmc_is_opened(state.vmc.output))
	{
		std::vector<vmc_device> trackers;
		trackers.reserve(frame->trackers.size());
		for (const auto &trackRecord : frame->trackers)
		{
			if (!trackRecord.result.isDetected() && !trackRecord.result.isTracked()) continue;
			auto cfg_tracker = std::find_if(server.trackerConfigs.begin(), server.trackerConfigs.end(),
					[&](auto &t){ return t.id == trackRecord.id; });
			if (cfg_tracker == server.trackerConfigs.end()) continue;
			VMCRole role = VMCRole::Tracker;
			if (cfg_tracker->role == TrackerConfig::ROLE_HMD) role = VMCRole::HMD;
			if (cfg_tracker->role == TrackerConfig::ROLE_CONTROLLER) role = VMCRole::Controller;
			trackers.emplace_back(role,
				cfg_tracker->label,
				trackRecord.poseFiltered
			);
		}
		vmc_send_device_packets(state.vmc.output, trackers, frame->time, dtS(server.lastStreamingStart, frame->time));
	}
}