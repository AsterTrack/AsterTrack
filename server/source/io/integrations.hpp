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

	struct
	{ // Virtual Reality Peripheral Network
		bool enabled = false;
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


/* Functions */

void IntegrationsInit(IntegrationsState &state, const GeneralConfig &config);
void IntegrationsCleanup(IntegrationsState &state, const GeneralConfig &config);
void IntegrationsReconfigureVRPN(IntegrationsState &state, const GeneralConfig &config);
void IntegrationsReconfigureVMC(IntegrationsState &state, const GeneralConfig &config);

void IntegrationsUpdate(IntegrationsState &state, ServerState &server);
void IntegrationsReceive(IntegrationsState &state, const ServerState &server);
void IntegrationsSendFrame(IntegrationsState &state, const ServerState &server, std::shared_ptr<FrameRecord> &frame);

#endif // INTEGRATIONS_H