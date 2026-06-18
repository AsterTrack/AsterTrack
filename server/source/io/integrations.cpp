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


void IntegrationsInit(IntegrationsState &io, const GeneralConfig &config)
{
	auto io_lock = std::unique_lock(io.mutex);
	io.vrpn.enabled = config.integrations.vrpn_auto_enable;
	io.vmc.enabled = config.integrations.vmc_auto_enable;
	io.lastUpdatedCameras = sclock::now() - std::chrono::seconds(2);

	IntegrationsUpdateNonessentialConfig(io, config);
	IntegrationsReconfigureVRPN(io, config);
	IntegrationsReconfigureVMC(io, config);
}

void IntegrationsCleanup(IntegrationsState &io, const GeneralConfig &config)
{
	auto io_lock = std::unique_lock(io.mutex);
	io.vrpn.enabled = false;
	io.vmc.enabled = false;

	IntegrationsReconfigureVRPN(io, config);
	IntegrationsReconfigureVMC(io, config);
}

void IntegrationsUpdateNonessentialConfig(IntegrationsState &io, const GeneralConfig &config)
{
	auto &cfg = config.integrations;
	io.vrpn.lowLatencySend = cfg.vrpn_low_latency;
}

void IntegrationsReconfigureVRPN(IntegrationsState &io, const GeneralConfig &config)
{
	io.vrpn.trackers.clear();
	io.vrpn.server = nullptr;
	if (!io.vrpn.enabled) return;
	// Setup VRPN Connection
	auto &cfg = config.integrations;
	std::string host = cfg.vrpn_host.empty()? "localhost" : cfg.vrpn_host;
	int port = cfg.vrpn_port? cfg.vrpn_port : vrpn_DEFAULT_LISTEN_PORT_NO;
	io.vrpn.host = host + ":" + std::to_string(port);
	std::string cname = cfg.vrpn_host + ":" + std::to_string(port);
	io.vrpn.server = opaque_ptr<vrpn_Connection>(vrpn_create_server_connection(cname.c_str()));
	io.vrpn.server->setAutoDeleteStatus(true);
}

void IntegrationsReconfigureVMC(IntegrationsState &io, const GeneralConfig &config)
{
	io.vmc.output = nullptr;
	if (!io.vmc.enabled) return;
	// Setup VRPN Connection
	auto &cfg = config.integrations;
	std::string host = cfg.vmc_host.empty()? "localhost" : cfg.vmc_host;
	int port = cfg.vmc_port? cfg.vmc_port : vmc_DEFAULT_PERFORMER_PORT_NO;
	io.vmc.host = host + ":" + std::to_string(port);
	io.vmc.output = vmc_init_output(host, std::to_string(port));
	vmc_try_open(io.vmc.output, io.vmc.host);
}

void IntegrationsUpdate(IntegrationsState &io, ServerState &state)
{ // Gets called regularly while in a mode, even when not streaming
	auto io_lock = std::unique_lock(io.mutex);

	// Timestamp for camera position
	TimePoint_t timestamp = sclock::now();
	OptFrameNum frame = state.pipeline.frameNum.load();
	if (frame > 0)
	{ // Replay may not be realtime, so take last frame time
		auto view = state.pipeline.record.frames.getView();
		if (frame < view.endIndex())
			timestamp = view[frame]->time;
	}

	bool updateCameras = dtS(io.lastUpdatedCameras, sclock::now()) > 1;
	if (updateCameras)
		io.lastUpdatedCameras = sclock::now();

	if (io.vrpn.enabled)
	{
		for (auto &tracker : state.trackerConfigs)
		{
			auto io_tracker = io.vrpn.trackers.find(tracker.id);
			if (!tracker.exposed)
			{
				if (io_tracker != io.vrpn.trackers.end())
				{ // Disconnect after expose was disabled
					LOG(LIO, LInfo, "VRPN Tracker '%s' is no longer exposed!", tracker.label.c_str());
					if (io_tracker->second->markedConnected)
						tracker.connected--;
					io.vrpn.trackers.erase(io_tracker);
					if (state.trackerOutput.contains(tracker.id))
						state.trackerOutput[tracker.id].vrpn = nullptr;
					ServerUpdateTrackerConditions(state, tracker);
				}
				continue;
			}
			if (io_tracker == io.vrpn.trackers.end())
			{
				LOG(LIO, LInfo, "Exposing VRPN Tracker '%s'", tracker.label.c_str());
				auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(tracker.id, tracker.label.c_str(), io.vrpn.server.get());
				if (state.trackerOutput.contains(tracker.id))
					state.trackerOutput[tracker.id].vrpn = vrpn_tracker;
				io_tracker = io.vrpn.trackers.insert({ tracker.id, std::move(vrpn_tracker) }).first;
			}
			if (!io_tracker->second->markedConnected && io_tracker->second->isConnected())
			{
				LOG(LIO, LInfo, "VRPN Tracker '%s' has been connected!", tracker.label.c_str());
				io_tracker->second->markedConnected = true;
				tracker.connected++;
				ServerUpdateTrackerConditions(state, tracker);
			}
			else if (io_tracker->second->markedConnected && !io_tracker->second->isConnected())
			{
				LOG(LIO, LInfo, "VRPN Tracker '%s' has been disconnected!", tracker.label.c_str());
				io_tracker->second->markedConnected = false;
				tracker.connected--;
				ServerUpdateTrackerConditions(state, tracker);
			}
		}

		if (updateCameras)
		{ // Add cameras as trackers to give clients an opportunity to display them as references
			// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
			for (const auto &camera : state.pipeline.cameras)
			{
				auto io_tracker = io.vrpn.trackers.find(camera->id);
				if (io_tracker == io.vrpn.trackers.end())
				{
					std::string path = asprintf_s("AsterTrack_Camera_%d", camera->index);
					auto vrpn_tracker = std::make_shared<vrpn_Tracker_AsterTrack>(camera->id, path.c_str(), io.vrpn.server.get());
					io_tracker = io.vrpn.trackers.insert({ camera->id, std::move(vrpn_tracker) }).first;
				}
				// Send static position of cameras as reference for clients
				// TODO: Implement proper custom protocol for meta-information like cameras, single 3D markers, etc.
				io_tracker->second->updatePose(0, timestamp, camera->calib.transform.cast<float>());
			}
		}

		LOG(LIO, LTrace, "Updating individual trackers!\n");
		for (auto &trackerIO : io.vrpn.trackers)
		{ // Handle mainloop (mostly ping-pong) here as well
			trackerIO.second->mainloop();
		}
	}

	if (io.vmc.enabled && vmc_try_open(io.vmc.output, io.vmc.host))
	{ // Try opening UDP port if not already done
		if (updateCameras)
		{ // Send cameras as trackers to give clients an opportunity to display them as references
			std::vector<vmc_device> cameras;
			cameras.reserve(state.pipeline.cameras.size());
			for (const auto &camera : state.pipeline.cameras)
			{
				cameras.emplace_back(VMCRole::Tracker,
					asprintf_s("AsterTrack_Camera_%d", camera->index),
					camera->calib.transform.cast<float>(),
					(float)getEffectiveFoVD(camera->calib, camera->mode)
				);
			}
			vmc_send_device_packets(io.vmc.output, cameras, timestamp, dtS(state.lastStreamingStart, timestamp));
		}
	}
}

void IntegrationsReceive(IntegrationsState &io, const ServerState &state)
{
	auto io_lock = std::unique_lock(io.mutex);

	if (io.vrpn.enabled)
	{ // Fetch incoming packets
		LOG(LIO, LTrace, "Updating VRPN server connection to fetch IMU packets!\n");
		io.vrpn.server->mainloop();
		if (!io.vrpn.server->doing_okay())
			LOG(LIO, LWarn, "VRPN Connection Error!\n");
		else
			LOG(LIO, LTrace, "     Server connection is doing ok!\n");
	}
}

void IntegrationsSendTracker(IntegrationsState &io, TrackerOutput &tracker, const TrackerOutputData &data, TimePoint_t time)
{
	if (!io.vrpn.enabled && !io.vmc.enabled) return;

	auto io_lock = std::unique_lock(io.mutex);

	if (tracker.vrpn && io.vrpn.lowLatencySend)
	{ // Fast-track output to VRPN
		// TODO: Not thread-safe?
		tracker.vrpn->updatePose(0, data.processedTime, data.processedPose);
		io.vrpn.server->send_pending_reports();
	}

	// Pose may have been post-processed for realtime output, e.g. with extra filtering
	// So store pose for any enabled I/O that can't update individual trackers

	// VMC is generally expected to send all trackers in one bundle, and use cases generally aren't as latency sensitive
	//if (!io.vmc.enabled) return;

	// IntegrationsSendFrame may not always be guaranteed to be called before next frames IntegrationsSendTracker
	// So need to store pose in a thread-safe manner and associated with the respective frame number

	tracker.processed.push(data);
}

void IntegrationsSendFrame(IntegrationsState &io, ServerState &state, std::shared_ptr<FrameRecord> &frame)
{
	auto io_lock = std::unique_lock(io.mutex);

	bool vmc_connected = io.vmc.enabled && vmc_is_opened(io.vmc.output);
	std::vector<vmc_device> vmc_output;
	if (vmc_connected)
		vmc_output.reserve(frame->trackers.size());

	/* for (auto &trackRecord : frame->trackers)
	{
		if (!trackRecord.result.isDetected() && !trackRecord.result.isTracked()) continue;
		auto trackConfig = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
			[&](auto &t){ return t.id == trackRecord.id; }); */
	for (auto &outputIt : state.trackerOutput)
	{
		TrackerOutput &output = outputIt.second;
		if (output.processed.empty()) continue;
		while (output.processed.front().frame < frame->num)
			output.processed.pop();
		if (output.processed.front().frame > frame->num)
			continue;
		auto &frameOutput = output.processed.front();

		// Already sent via vrpn in IntegrationsSendTracker
		if (io.vrpn.enabled && !io.vrpn.lowLatencySend)
		{
			auto io_tracker = io.vrpn.trackers.find(output.id);
			if (io_tracker != io.vrpn.trackers.end())
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

	if (io.vrpn.enabled)
	{
		LOG(LIO, LTrace, "Updating VRPN server connection to push tracker packets!\n");
		io.vrpn.server->send_pending_reports();
		if (!io.vrpn.server->doing_okay())
			LOG(LIO, LDarn, "    A VRPN connection is facing issues!\n");
	}

	if (vmc_connected)
	{
		LOG(LIO, LTrace, "Pushing tracker packets to VMC output port!\n");
		vmc_send_device_packets(io.vmc.output, vmc_output, frame->time, dtS(state.lastStreamingStart, frame->time));
	}
}