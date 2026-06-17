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

#ifndef INTEGRATIONS_H
#define INTEGRATIONS_H

#include "config.hpp"
#include "tracking/detail/filters.hpp"

#include "util/memory.hpp"

#include <map>

/* Structures */

// Forward-declared opaque structs
class vrpn_Connection; // io/vrpn.hpp
class vrpn_Tracker_AsterTrack; // io/vrpn.hpp
struct vmc_output; // io/vmc.hpp
struct ServerState; // server.hpp

struct IntegrationsState
{
	std::mutex mutex;

	TimePoint_t lastUpdatedCameras;

	struct
	{ // Virtual Reality Peripheral Network
		bool enabled = false;
		bool lowLatencySend;
		std::string host;
		opaque_ptr<vrpn_Connection> server;
		std::map<int, std::shared_ptr<vrpn_Tracker_AsterTrack>> trackers;
	} vrpn;

	struct
	{ // Virtual Motion Capture over OSC
		bool enabled = false;
		std::string host;
		opaque_ptr<vmc_output> output;
	} vmc;
};

struct TrackerOutputData
{
	FrameNum frame;
	Eigen::Isometry3f pose;
	// May add velocity, covariance, etc.

	// May have post-processed the pose already
	// May even be extrapolated past original frame time
	TimePoint_t processedTime;
	Eigen::Isometry3f processedPose;
};

struct TrackerOutput
{
	int id;
	std::string label;
	TrackerConfig::TrackerRole role;
	TrackerOutputConfig config;
	std::queue<TrackerOutputData> processed;

	// Post-processing states
	OneEuroFilter<Eigen::Vector3f, float, Eigen::Vector3f> filterPos;
	OneEuroFilter<Eigen::Quaternionf, float, Eigen::Vector3f> filterRot;

	// I/O-specific shortcuts
	std::shared_ptr<vrpn_Tracker_AsterTrack> vrpn;

	inline void adoptConfig()
	{
		if (config.applyFiltering == TrackerOutputConfig::ONE_EURO_FILTER)
		{ // Update OneEuroFilter
			auto &params = config.oneEuroFilter;
			filterPos.setParams(params.posCutoffBase, params.posCutoffBeta, params.posCutoffDelta);
			filterRot.setParams(params.rotCutoffBase, params.rotCutoffBeta, params.rotCutoffDelta);
		}
	}
};


/* Functions */

void IntegrationsInit(IntegrationsState &io, const GeneralConfig &config);
void IntegrationsCleanup(IntegrationsState &io, const GeneralConfig &config);
void IntegrationsUpdateNonessentialConfig(IntegrationsState &io, const GeneralConfig &config);
void IntegrationsReconfigureVRPN(IntegrationsState &io, const GeneralConfig &config);
void IntegrationsReconfigureVMC(IntegrationsState &io, const GeneralConfig &config);

void IntegrationsUpdate(IntegrationsState &io, ServerState &state);
void IntegrationsReceive(IntegrationsState &io, const ServerState &state);
void IntegrationsSendTracker(IntegrationsState &io, TrackerOutput &tracker, const TrackerOutputData &data, TimePoint_t time);
void IntegrationsSendFrame(IntegrationsState &io, ServerState &state, std::shared_ptr<FrameRecord> &frame);

#endif // INTEGRATIONS_H