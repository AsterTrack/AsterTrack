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
#include "ui/shared.hpp" // Signals to UI

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

std::optional<ErrorMessage> loadRecording(ServerState &state, Recording &&recordEntries, bool append, bool separate)
{
	int prevAdvance = state.simAdvance;
	int prevIMUs = state.stored.imus.size(), prevCams = state.pipeline.cameras.size();
	std::vector<CameraConfigRecord> cameras;
	if (append)
	{
		// Pause replay
		state.simAdvance = 0;
		// Add existing cameras for verification
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
		std::size_t offset;
		auto error = parseRecording(recordEntries.captures[i], cameras, state.stored, offset, separate);
		if (error) return error;
		std::size_t count = state.stored.frames.getView().size() - start;
		state.recording.segments.emplace_back(start, count, offset);
		framesCount += count;
	}
	// Load all tracking segments containing recorded tracking results
	for (int i = 0; i < recordEntries.tracking.size(); i++)
	{
		auto &segment = state.recording.segments[segmentOffset + i];
		if (segment.frameCount == 0) continue; // Invalid or missing segment
		auto error = parseTrackingResults(recordEntries.tracking[i], state.stored, segment.frameOffset);
		if (error && error->code != ENOENT) return error;
	}
	// Store paths of each numbered segment as well
	std::move(std::begin(recordEntries.captures), std::end(recordEntries.captures), std::back_inserter(state.recording.captures));
	std::move(std::begin(recordEntries.tracking), std::end(recordEntries.tracking), std::back_inserter(state.recording.tracking));
	state.recording.frames += framesCount;

	std::vector<CameraCalib> cameraCalibs;
	if (!recordEntries.calib.empty())
	{ // Parse calibrations
		auto error = parseCameraCalibrations(recordEntries.calib, cameraCalibs);
		for (int c = 0; c < cameraCalibs.size(); c++)
			cameraCalibs[c].index += prevCams;
		if (error && error->code != ENOENT) return error;
	}

	LOG(LGUI, LInfo, "Loaded %ld frames for replay!\n", framesCount);
	if (append)
	{
		// Add new IMUs
		state.pipeline.record.imus.reserve(state.stored.imus.size());
		for (int i = prevIMUs; i < state.stored.imus.size(); i++)
		{
			auto imu = std::make_shared<IMURecord>(*state.stored.imus[i]);
			imu->index = state.pipeline.record.imus.size();
			state.pipeline.record.imus.push_back(std::move(imu));
		}
		if (separate)
		{
			// Ensure newly added cameras have a unique ID
			for (int c = prevCams; c < cameras.size(); c++)
			{
				for (int cc = 0; cc < cameras.size(); cc++)
				{
					if (cameras[c].ID != cameras[cc].ID) continue;
					int newID = rand();
					bool found = false;
					for (auto &calib : cameraCalibs)
					{ // If we loaded it's calib, change it's ID
						if (calib.id != cameras[c].ID) continue;
						calib.id = newID;
						found = true;
						break;
					}
					if (!found)
					{ // If we have not loaded it's calib, find stored calib and ensure it's adopted for the new ID
						for (auto &calib : state.cameraCalibrations)
						{
							if (calib.id != cameras[c].ID) continue;
							cameraCalibs.push_back(calib);
							cameraCalibs.back().id = newID;
						}

					}
					cameras[c].ID = newID;
				}
			}
			{ // Add new cameras
				std::unique_lock dev_lock(state.deviceAccessMutex); // cameras 
				for (auto cam : cameras)
					EnsureCamera(state, cam.ID);
			}
			// Adopt calibrations for new cameras
			AdoptNewCalibrations(state.pipeline, cameraCalibs, true);
			{ // Calculate fundamental matrices from calibration
				auto lock = folly::detail::lock(folly::detail::wlock(state.pipeline.calibration), folly::detail::rlock(state.pipeline.seqDatabase));
				UpdateCalibrationRelations(state.pipeline, *std::get<0>(lock), *std::get<1>(lock));
			}
			SignalServerEvent(EVT_UPDATE_CAMERAS);
		}
		// Continue replay
		state.simAdvance = prevAdvance;
		state.simAdvance.notify_all();
		return std::nullopt;
	}
	if (state.mode != MODE_None)
	{
		LOG(LGUI, LWarn, "Already entered a mode, will not start replay!\n");
		return "Entered a mode while loading replay!";
	}

	// Setup replay mode with relevant cameras
	StartReplay(state, cameras);

	// Adopt calibrations stored alongside (replacing existing calibrations)
	AdoptNewCalibrations(state.pipeline, cameraCalibs, true);

	return std::nullopt;
}