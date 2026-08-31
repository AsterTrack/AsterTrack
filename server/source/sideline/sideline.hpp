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

#ifndef SIDELINE_H
#define SIDELINE_H

#include "recording.hpp"

#include "pipeline/record.hpp"

#include <vector>
#include <atomic>


/* Structures */

struct SidelineState
{
	// Settings for record-keeping
	bool keepFrameImages = true, keepTrackingResults = true;

	// Sections marked for recording
	std::vector<RecordedSections> recordSections;
	OptFrameNum recordSectionStart = -1;

	// Simulation/Replay control
	enum AdvanceTiming { ADV_NORMAL, ADV_REALTIME, ADV_QUICKLY, ADV_MAX };
	struct 
	{
		std::atomic<int> mode = { -1 };
		std::atomic<bool> waiting = { false };
		AdvanceTiming timing;
	} advance;

	// Dropout Simulation
	std::atomic<int> dropoutIndex = { -1 };
	std::vector<float> dropoutSeverity = { 1 };

	// Copy detections of trackers from stored records (to replace slow detection attempts)
	bool copyDetectionsFromStored = false, copyAlsoFromTracked = true, copyLimitedReinstatement = true;

	// Manual selection of tracking data to compare
	std::vector<TrackerCompareRecord> compareTrackers;

	struct
	{ // Automatic testing of configured captures (potentially headless)
		bool isTesting;
		std::string condition;
		std::vector<int> recordings;
	} testing;

	// Loaded records for replay
	struct Segment { FrameNum frameStart, frameCount, frameOffset; };
	struct Recording { int number; std::string label; FrameNum frameStart, frameCount; std::vector<int> cameras; };
	struct {
		// This may contain info about multiple appended recordings
		// And each recording may be split into multiple segments (capture+tracking)
		// The following lists have an entry for each segment
		std::vector<std::string> captures;
		std::vector<std::string> tracking;
		std::vector<Segment> segments;
		// This list has an entry for each appended recording
		std::vector<Recording> recordings;
		FrameNum frames = 0;
		TimePoint_t replayTime;
	} recording = {};
	TrackingRecord record;
};

#endif // SIDELINE_H