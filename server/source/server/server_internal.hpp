/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef SERVER_INTERNAL_H
#define SERVER_INTERNAL_H

#include "server.hpp"

void SetupVirtualSyncGroup(ServerState &state);
void DeleteVirtualSyncGroup(ServerState &state);

void DevicesStartStreaming(ServerState &state);
void DevicesStopStreaming(ServerState &state);

void StopCoprocessingThread(ServerState &state);

void RealtimeProcessingThread(std::stop_token stop_token, ServerState *statePtr);

#endif // SERVER_INTERNAL_H