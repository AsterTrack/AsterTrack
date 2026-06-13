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

#ifndef WIRELESS_CAMERAS_H
#define WIRELESS_CAMERAS_H

#include "comm/wireless_server.hpp"

struct ServerState;

void StartWirelessServer(ServerCommState &server, ServerState &state);
void StopWirelessServer(ServerCommState &server);

#endif // WIRELESS_CAMERAS_H
