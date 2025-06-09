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

#include "recording.hpp"

#include "server.hpp"
#include "config.hpp"

#include "util/log.hpp"

#include <filesystem>

void parseRecordEntries(std::map<int, Recording> &recordEntries)
{
	if (!std::filesystem::exists(std::filesystem::path("dump"))) return;

	for (const auto &file : std::filesystem::directory_iterator("dump"))
	{
		if (file.path().extension().compare(".json") != 0) continue;
		const std::string &str = file.path().stem().string();

		// Parse number and type
		int num = -1, pos;
		if (std::sscanf(str.data(), "%d_%n", &num, &pos) != 1) continue;
		int type = 0;
		static const std::array<std::string,3> types = { "capture", "tracking", "calib" };
		for (; type < types.size(); type++)
			if (std::strncmp(str.data()+pos, types[type].data(), types[type].length()) == 0) break;
		if (type == types.size()) continue;
		pos += types[type].length();

		// Parse part (optional) and label (optional)
		int part = 0;
		char label[100] = "";
		if (str.length() == pos) {}
		else if (std::sscanf(str.data()+pos, "_%d_%99s", &part, label) == 2) {}
		else if (std::sscanf(str.data()+pos, "_%d", &part) == 1) {}
		else if (std::sscanf(str.data()+pos, "_%99s", label) == 1) { part = 0; }
		else continue; // Something other than _ after type

		// Update record entry
		auto &entry = recordEntries[num];
		entry.number = num;
		if (part > 10000)
		{
			entry.corrupt = true;
			continue;
		}
		if (entry.label.empty()) entry.label = label;
		if (entry.captures.size() <= part) entry.captures.resize(part+1);
		if (entry.tracking.size() <= part) entry.tracking.resize(part+1);
		if (type == 2) entry.calib = file.path().string();
		else if (type == 0) entry.captures[part] = file.path().string();
		else if (type == 1) entry.tracking[part] = file.path().string();
	}
}

void loadRecording(ServerState &state, Recording &&recordEntries, bool append)
{
	std::vector<CameraConfigRecord> cameras;
	if (append)
	{ // Add existing cameras for verification
		cameras.resize(state.pipeline.cameras.size());
		for (auto &camera : state.pipeline.cameras)
			cameras[camera->index] = { camera->id, camera->mode.widthPx, camera->mode.heightPx };
	}
	else
	{ // Clear previous recording
		state.stored.frames.cull_clear();
		state.stored.imus.clear();
		state.recording = {};
		state.stored.frames.delete_culled();
	}
	// Will load recording in numbered segments
	std::size_t segmentOffset = state.recording.segments.size(), framesCount = 0;
	state.recording.segments.reserve(segmentOffset + recordEntries.captures.size());
	// Load all capture segments containing recorded data
	for (int i = 0; i < recordEntries.captures.size(); i++)
	{
		std::size_t start = state.stored.frames.getView().size();
		std::size_t offset = parseRecording(recordEntries.captures[i], cameras, state.stored);
		// May fail, in which case frame count will be 0 for this segment
		std::size_t count = state.stored.frames.getView().size() - start;
		state.recording.segments.emplace_back(start, count, offset);
		framesCount += count;
	}
	// Load all tracking segments containing recorded tracking results
	for (int i = 0; i < recordEntries.tracking.size(); i++)
	{
		auto &segment = state.recording.segments[segmentOffset + i];
		if (segment.frameCount == 0) continue; // Invalid or missing segment
		if (recordEntries.tracking[i].empty())
			recordEntries.tracking[i] = recordEntries.captures[i];
		parseTrackingResults(recordEntries.tracking[i], state.stored, segment.frameOffset);
	}
	// Store paths of each numbered segment as well
	std::move(std::begin(recordEntries.captures), std::end(recordEntries.captures), std::back_inserter(state.recording.captures));
	std::move(std::begin(recordEntries.tracking), std::end(recordEntries.tracking), std::back_inserter(state.recording.tracking));
	state.recording.frames += framesCount;

	LOG(LGUI, LInfo, "Loaded %ld frames for replay!\n", framesCount);
	if (state.mode != MODE_None)
	{
		LOG(LGUI, LWarn, "Already entered a mode, will not start replay!\n");
		return;
	}

	// Setup replay mode with relevant cameras
	StartReplay(state, cameras);

	if (!recordEntries.calib.empty())
	{ // Overwrite any calibrations
		std::vector<CameraCalib> cameraCalibs;
		parseCameraCalibrations(recordEntries.calib, cameraCalibs);

		for (auto &cam : state.pipeline.cameras)
		{
			for (int i = 0; i < cameraCalibs.size(); i++)
			{
				if (cameraCalibs[i].id == cam->id)
				{
					cam->calib = cameraCalibs[i];
					cam->calib.index = cam->index;
					break;
				}
			}
		}
	}
}