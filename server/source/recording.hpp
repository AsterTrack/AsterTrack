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

#ifndef RECORDING_H
#define RECORDING_H

#include "util/error.hpp"

#include <string>
#include <vector>
#include <map>

struct ServerState;

/*
 * Handles loading of recordings for replay
 */

/* Structures */

struct Recording
{
    int number;
    std::string label;
    bool corrupt = false;
    std::vector<std::string> captures;
    std::vector<std::string> tracking;
    std::string calib;
};

struct RecordingSegment
{
    std::size_t frameStart, frameCount, frameOffset;
};


/* Functions */

/**
 * Parses recording folder for all record entries that can be loaded
 */
void parseRecordEntries(std::map<int,Recording> &recordEntries);

/**
 * Loads the recording into state for replay, optionally appending to an existing recording
 */
HANDLE_ERROR loadRecording(ServerState &state, Recording &&recordEntries, bool append);

#endif // RECORDING_H
