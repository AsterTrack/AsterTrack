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
	std::string calib, images;
};


/* Functions */

/**
 * Sanitise label for recording and return reason if it was modified.
 */
std::optional<ErrorMessage> sanitiseRecordingLabel(std::string &label);

/**
 * Parses recording folder for all record entries that can be loaded
 */
void parseRecordEntries(std::map<int,Recording> &recordEntries);

/**
 * Parses recording folder to find the given recording
 */
std::optional<Recording> findRecording(int recording);

/**
 * Loads the recording into state for replay, optionally appending to an existing recording
 */
HANDLE_ERROR loadRecording(ServerState &state, Recording &&recordEntries, bool append, bool separate);

/**
 * Load a specified set of recordings by ID from the recordings folder
 */
std::optional<ErrorMessage> loadRecordingSet(ServerState &state, const std::vector<int> &recordings);

/**
 * Rename the files of the given recording to assign the given label 
 */
std::optional<ErrorMessage> renameRecording(Recording &recording, std::string label);

/**
 * Remove all files of the given recording
 */
std::optional<ErrorMessage> deleteRecording(Recording &recording);

#endif // RECORDING_H
