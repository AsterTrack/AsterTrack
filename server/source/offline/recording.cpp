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

#include "recording.hpp"

#include "server/server.hpp"
#include "config.hpp"
#include "ui/shared.hpp" // Signals to UI

#include "util/log.hpp"

#include <filesystem>


#define LABEL_ILLEGAL "/\\\n\t\r"
#define LABEL_MATCH "%200[^" LABEL_ILLEGAL "]"
const int RECORDING_NAME_MAX_SIZE = 200;

std::optional<ErrorMessage> sanitiseRecordingLabel(std::string &label)
{
	std::optional<ErrorMessage> sanitiseReason;
	int num, pos;
	bool startNum = std::sscanf(label.c_str(), "%d%n", &num, &pos) == 1;
	if (startNum && pos == label.length())
	{ // Number without _ would be fine
		sanitiseReason = asprintf_s("May not be just a number.");
		label += ";";
	}
	if (startNum && label[pos] == '_')
	{ // Number without _ would be fine
		sanitiseReason = asprintf_s("May not start with a number followed by underscore!");
		label[pos] = ';';
	}
	if (label.size() > RECORDING_NAME_MAX_SIZE)
	{
		sanitiseReason = asprintf_s("Exceeding size %d by %d characters", RECORDING_NAME_MAX_SIZE, (int)label.size() - RECORDING_NAME_MAX_SIZE);
		label.resize(RECORDING_NAME_MAX_SIZE);
	}
	auto illegalChar = label.find_first_of(LABEL_ILLEGAL);
	if (illegalChar != std::string::npos)
	{
		sanitiseReason = asprintf_s("Illegal character '%c'", label[illegalChar]);
		label.resize(illegalChar);
	}
	return sanitiseReason;
}

enum RecordFileType { File_Capture, File_Tracking, File_Calib, File_Unknown };
static const std::array<std::string,3> FileTypes = { "capture", "tracking", "calib" };

static bool parseRecordFileName(const std::string &file, int &num, int &type, int &part, std::string &label)
{
	// Parse number and type
	int head, tail;
	if (std::sscanf(file.data(), "%d_%n", &num, &head) != 1)
		return false;
	for (type = 0; type < FileTypes.size(); type++)
	{
		if (file.size() < head + FileTypes[type].length())
			continue;
		if (std::strncmp(file.data()+head, FileTypes[type].data(), FileTypes[type].length()) == 0)
			break;
	}
	if (type == FileTypes.size())
		return false;
	head += FileTypes[type].length();

	// Parse part (optional) and label (optional)
	char labelBuf[RECORDING_NAME_MAX_SIZE+1] = "";
	part = 0;
	if (file.length() == head) {}
	else if (std::sscanf(file.data()+head, "_%d_" LABEL_MATCH "%n", &part, labelBuf, &tail) == 2) {}
	else if (std::sscanf(file.data()+head, "_%d%n", &part, &tail) == 1 && file.length() == head+tail) {}
	else if (std::sscanf(file.data()+head, "_" LABEL_MATCH "%n", labelBuf, &tail) == 1) { part = 0; }
	else return false; // Something other than _ after type
	label = labelBuf;
	if (label.starts_with("to"))
		LOG(LGUI, LInfo, "Derp");
	// TODO: What if file.length() != pos+end? e.g. longer label or number X NOT followed by _?
	return true;
}

void parseRecordEntries(std::map<int, Recording> &recordEntries)
{
	std::filesystem::path recordings(recordingsFolder);
	if (!std::filesystem::is_directory(recordings)) return;

	for (const auto &file : std::filesystem::directory_iterator(recordings))
	{
		if (!file.is_regular_file()) continue;
		if (file.path().extension().compare(".json") != 0) continue;
		const std::string &str = file.path().stem().string();

		int num, type, part;
		std::string label;
		if (!parseRecordFileName(str, num, type, part, label))
			continue;

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
		if (type == File_Calib) entry.calib = file.path().string();
		else if (type == File_Capture) entry.captures[part] = file.path().string();
		else if (type == File_Tracking) entry.tracking[part] = file.path().string();
	}

	for (auto &recordIt : recordEntries)
	{
		auto &recording = recordIt.second;
		std::filesystem::path imageFolder = recordings / asprintf_s("%d_capture", recording.number);
		if (std::filesystem::is_directory(imageFolder))
			recording.images = imageFolder.string();
	}
}

std::optional<Recording> findRecording(int recording)
{
	std::map<int, Recording> recordEntries;
	parseRecordEntries(recordEntries);
	auto recIt = recordEntries.find(recording);
	if (recIt != recordEntries.end())
		return recIt->second;
	return std::nullopt;
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
		state.recording = {};
		state.stored.frames.cull_clear();
		state.stored.imus.clear();
		state.stored.frames.delete_culled();
	}
	// Prepare full recording data
	std::size_t recStart = state.stored.frames.getView().size();
	std::vector<int> cameraIndices;
	// Will load recording in numbered segments
	std::size_t segmentOffset = state.recording.segments.size();
	state.recording.segments.reserve(segmentOffset + recordEntries.captures.size());
	// Load all capture segments containing recorded data
	for (int i = 0; i < recordEntries.captures.size(); i++)
	{
		std::size_t start = state.stored.frames.getView().size();
		std::size_t offset;
		auto error = parseRecording(recordEntries.captures[i], cameras, cameraIndices, state.stored, offset, separate);
		if (error) return error;
		std::size_t count = state.stored.frames.getView().size() - start;
		state.recording.segments.emplace_back(start, count, offset);
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
	std::size_t recCount = state.stored.frames.getView().size() - recStart;
	state.recording.recordings.emplace_back(recordEntries.number, recordEntries.label, recStart, recCount, std::move(cameraIndices));
	state.recording.frames += recCount;

	std::vector<CameraCalib> cameraCalibs;
	if (!recordEntries.calib.empty())
	{ // Parse calibrations
		auto error = parseCameraCalibrations(recordEntries.calib, cameraCalibs);
		if (error && error->code != ENOENT) return error;
	}

	LOG(LGUI, LInfo, "Loaded %ld frames for replay!\n", recCount);
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
		if (prevCams != cameras.size())
		{ // Ensure newly added cameras have a unique ID
			for (int c = prevCams; c < cameras.size(); c++)
			{
				for (int cc = 0; cc < cameras.size(); cc++)
				{
					if (cameras[c].ID != cameras[cc].ID) continue;
					// Found this camera in the existing replay
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

std::optional<ErrorMessage> loadRecordingSet(ServerState &state, const std::vector<int> &recordings)
{
	std::map<int, Recording> recordEntries;
	parseRecordEntries(recordEntries);

	std::vector<Recording> selectedRecordings;
	selectedRecordings.reserve(recordings.size());
	for (int selected : recordings)
		if (recordEntries.contains(selected))
			selectedRecordings.push_back(recordEntries[selected]);

	if (selectedRecordings.empty())
		return "Found none of the specified recordings!";

	state.isLoading = true;

	if (state.mode != MODE_None)
		StopReplay(state);

	// Load all recordings marked for testing with all their captures
	bool first = true;
	for (auto &recording : selectedRecordings)
	{
		auto error = loadRecording(state, std::move(recording), !first, !first);
		if (error) SignalErrorToUser(error.value());
		else first = false;
	}

	state.isLoading = false;

	if (first)
		return "Failed to load any specified recordings!";
	return std::nullopt;
}

std::optional<ErrorMessage> renameRecording(Recording &recording, std::string label)
{
	auto sanitiseReason = sanitiseRecordingLabel(label);
	if (sanitiseReason) return sanitiseReason;

	auto renameFile = [&](std::string &fileName, RecordFileType fileType) -> std::optional<ErrorMessage>
	{
		std::filesystem::path path(fileName);
		if (!std::filesystem::exists(path))
			return asprintf_s("File '%s' does not exist!", path.filename().c_str());

		int num, type, part;
		std::string l;
		if (!parseRecordFileName(path.stem().string(), num, type, part, l))
			return asprintf_s("File '%s' is not an expected capture file!", path.filename().c_str());
		else if (num != recording.number || type != fileType)
			return asprintf_s("File '%s' is not an expected capture file!", path.filename().c_str());

		std::filesystem::path newPath;
		if (recording.captures.size() > 1)
			newPath = asprintf_s("%d_%s_%d_%s.json", recording.number, FileTypes[fileType].c_str(), part, label.c_str());
		else
			newPath = asprintf_s("%d_%s_%s.json", recording.number, FileTypes[fileType].c_str(), label.c_str());
		newPath = path.parent_path() / newPath;
		if (path == newPath)
			return std::nullopt;
		if (std::filesystem::exists(newPath))
			return asprintf_s("File '%s' already exists, renaming failed!", newPath.filename().c_str());

		std::filesystem::rename(path, newPath);
		fileName = newPath.string();
		return std::nullopt;
	};


	std::optional<ErrorMessage> error;

	for (auto &capture : recording.captures)
		if ((error = renameFile(capture, File_Capture)))
			return error;

	for (auto &tracking : recording.tracking)
		if ((error = renameFile(tracking, File_Tracking)))
			return error;

	// Currently, calib is not named at all
	//error = renameFile(recording.calib, File_Calib);

	// Image folder is never renamed, paths in capture rely on it

	recording.label = label;
	return error;
}

std::optional<ErrorMessage> deleteRecording(Recording &recording)
{
	int missing = 0;

	for (auto &capture : recording.captures)
		if (!std::filesystem::remove(capture))
			missing++;

	for (auto &tracking : recording.tracking)
		if (!std::filesystem::remove(tracking))
			missing++;

	if (!std::filesystem::remove(recording.calib))
		missing++;

	if (!recording.images.empty())
	{
		if (std::filesystem::is_directory(recording.images))
			std::filesystem::remove_all(recording.images);
		else missing++;
	}

	if (missing > 0)
		return asprintf_s("Failed to delete %d files of the recording.", missing);
	return std::nullopt;
}
