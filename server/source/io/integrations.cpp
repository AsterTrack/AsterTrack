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
#include "server/server.hpp"

#include "util/log.hpp"
#include "util/eigenutil.hpp"


void IntegrationsInit(IntegrationsState &state , const GeneralConfig &config)
{
	auto io_lock = std::unique_lock(state.mutex);
	state.vrpn.enabled = config.integrations.vrpn_auto_enable;
	state.vmc.enabled = config.integrations.vmc_auto_enable;
	state.lastUpdatedCameras = sclock::now() - std::chrono::seconds(2);

	IntegrationsUpdateNonessentialConfig(state, config);
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

void IntegrationsUpdateNonessentialConfig(IntegrationsState &state, const GeneralConfig &config)
{
	auto &cfg = config.integrations;
	state.vrpn.lowLatencySend = cfg.vrpn_low_latency;
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

	bool updateCameras = dtS(state.lastUpdatedCameras, sclock::now()) > 1;
	if (updateCameras)
		state.lastUpdatedCameras = sclock::now();

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
					if (server.trackerOutput.contains(tracker.id))
						server.trackerOutput[tracker.id].vrpn = nullptr;
					ServerUpdateTrackerConditions(server, tracker);
				}
				continue;
			}
			if (io_tracker == state.vrpn.trackers.end())
			{
				LOG(LIO, LInfo, "Exposing VRPN Tracker '%s'", tracker.label.c_str());
				auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(tracker.id, tracker.label.c_str(), state.vrpn.server.get());
				if (server.trackerOutput.contains(tracker.id))
					server.trackerOutput[tracker.id].vrpn = vrpn_tracker;
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

		if (updateCameras)
		{ // Add cameras as trackers to give clients an opportunity to display them as references
			// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
			for (const auto &camera : server.pipeline.cameras)
			{
				auto io_tracker = state.vrpn.trackers.find(camera->id);
				if (io_tracker == state.vrpn.trackers.end())
				{
					std::string path = asprintf_s("AsterTrack_Camera_%d", camera->index);
					auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(camera->id, path.c_str(), state.vrpn.server.get());
					io_tracker = state.vrpn.trackers.insert({ camera->id, std::move(vrpn_tracker) }).first;
				}
				// Send static position of cameras as reference for clients
				// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
				io_tracker->second->updatePose(0, timestamp, camera->calib.transform.cast<float>());
			}
		}

		LOG(LIO, LTrace, "Updating individual trackers!\n");
		for (auto &trackerIO : state.vrpn.trackers)
		{ // Handle mainloop (mostly ping-pong) here as well
			trackerIO.second->mainloop();
		}
	}

	if (state.vmc.enabled && vmc_try_open(state.vmc.output, state.vmc.host))
	{ // Try opening UDP port if not already done
		if (updateCameras)
		{ // Send cameras as trackers to give clients an opportunity to display them as references
			std::vector<vmc_device> cameras;
			cameras.reserve(server.pipeline.cameras.size());
			for (const auto &camera : server.pipeline.cameras)
			{
				cameras.emplace_back(VMCRole::Tracker,
					asprintf_s("AsterTrack_Camera_%d", camera->index),
					camera->calib.transform.cast<float>(),
					(float)getEffectiveFoVD(camera->calib, camera->mode)
				);
			}
			vmc_send_device_packets(state.vmc.output, cameras, timestamp, dtS(server.lastStreamingStart, timestamp));
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

void IntegrationsSendTracker(IntegrationsState &state, TrackerOutput &tracker, const TrackerOutputData &data, TimePoint_t time)
{
	if (!state.vrpn.enabled && !state.vmc.enabled) return;

	auto io_lock = std::unique_lock(state.mutex);

	if (tracker.vrpn && state.vrpn.lowLatencySend)
	{ // Fast-track output to VRPN
		// TODO: Not thread-safe?
		tracker.vrpn->updatePose(0, data.processedTime, data.processedPose);
		state.vrpn.server->send_pending_reports();
	}

	// Pose may have been post-processed for realtime output, e.g. with extra filtering
	// So store pose for any enabled I/O that can't update individual trackers

	// VMC is generally expected to send all trackers in one bundle, and use cases generally aren't as latency sensitive
	//if (!io.vmc.enabled) return;

	// IntegrationsSendFrame may not always be guaranteed to be called before next frames IntegrationsSendTracker
	// So need to store pose in a thread-safe manner and associated with the respective frame number

	tracker.processed.push(data);
}

void IntegrationsSendFrame(IntegrationsState &state, ServerState &server, std::shared_ptr<FrameRecord> &frame)
{
	auto io_lock = std::unique_lock(state.mutex);

	bool vmc_connected = state.vmc.enabled && vmc_is_opened(state.vmc.output);
	std::vector<vmc_device> vmc_output;
	if (vmc_connected)
		vmc_output.reserve(frame->trackers.size());

	/* for (auto &trackRecord : frame->trackers)
	{
		if (!trackRecord.result.isDetected() && !trackRecord.result.isTracked()) continue;
		auto trackConfig = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
			[&](auto &t){ return t.id == trackRecord.id; }); */
	for (auto &outputIt : server.trackerOutput)
	{
		TrackerOutput &output = outputIt.second;
		if (output.processed.empty()) continue;
		while (output.processed.front().frame < frame->num)
			output.processed.pop();
		if (output.processed.front().frame > frame->num)
			continue;
		auto &frameOutput = output.processed.front();

		// Already sent via vrpn in IntegrationsSendTracker
		if (state.vrpn.enabled && !state.vrpn.lowLatencySend)
		{
			auto io_tracker = state.vrpn.trackers.find(output.id);
			if (io_tracker != state.vrpn.trackers.end())
				io_tracker->second->updatePose(0, frameOutput.processedTime, frameOutput.processedPose);
		}

		if (vmc_connected)
		{
			VMCRole role = VMCRole::Tracker;
			if (output.role == TrackerConfig::ROLE_HMD) role = VMCRole::HMD;
			if (output.role == TrackerConfig::ROLE_CONTROLLER) role = VMCRole::Controller;
			if (output.role == TrackerConfig::ROLE_CAMERA) role = VMCRole::Camera;
			// TODO: Giving up on processed time, won't predict ahead
			vmc_output.emplace_back(role, output.label, frameOutput.processedPose);
		}
		output.processed.pop();
	}

	if (state.vrpn.enabled)
	{
		LOG(LIO, LTrace, "Updating VRPN server connection to push tracker packets!\n");
		state.vrpn.server->send_pending_reports();
		if (!state.vrpn.server->doing_okay())
			LOG(LIO, LDarn, "    A VRPN connection is facing issues!\n");
	}

	if (vmc_connected)
	{
		LOG(LIO, LTrace, "Pushing tracker packets to VMC output port!\n");
		vmc_send_device_packets(state.vmc.output, vmc_output, frame->time, dtS(server.lastStreamingStart, frame->time));
	}
}