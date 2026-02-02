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

#ifndef CALIBRATION_TARGET_AQUISITION_H
#define CALIBRATION_TARGET_AQUISITION_H

#include "parameters.hpp"

#include "calib/obs_data.hpp"
#include "calib/optimisation.hpp"
#include "point/sequence_data.hpp"

#include "util/eigendef.hpp"
#include "circular_buffer/circular_buffer.hpp"
#include "util/stats.hpp"

#include <vector>
#include <map>


/**
 * Calibration of target structure
 */

struct BlockStats
{
	int sampleCount, frameCount, markerCount, minMarkerObs, minFrameObs, minSharedMarkers, paramCount;
};

const static int TargetViewAquisitionWindowSize = 1000;

struct TargetViewAquisition
{
	// Stats during aquisition phase
	CircularBuffer<int> markerCount;
	CircularBuffer<int> triangulationsCount;
	CircularBuffer<int> sharedMarkerCount;
	CircularBuffer<float> avgMarkerCount;
	CircularBuffer<float> valueGraph;

	// Current value used for user feedback
	float currentValue;

	// Sliding window average
	FrameNum nextUpdate;

	// Current local maxima
	bool localMaxSearching;
	FrameNum localMaxFrame;
	float localMaxValue;
	StatDistf localMaximaStats;
	struct LocalMaxima
	{
		FrameNum maxima;
		float value;
		FrameNum begin, end;
	};
	std::vector<LocalMaxima> localMaximas;

	TargetViewAquisition() :
		markerCount(0), triangulationsCount(0), sharedMarkerCount(0), avgMarkerCount(0), valueGraph(0)
	{ // Above initialisation list is just to satisfy compiler
		reset();
	}

	void reset()
	{
		markerCount = CircularBuffer<int>(TargetViewAquisitionWindowSize, TargetViewAquisitionWindowSize);
		triangulationsCount = CircularBuffer<int>(TargetViewAquisitionWindowSize, TargetViewAquisitionWindowSize);
		sharedMarkerCount = CircularBuffer<int>(TargetViewAquisitionWindowSize, TargetViewAquisitionWindowSize);
		avgMarkerCount = CircularBuffer<float>(TargetViewAquisitionWindowSize, TargetViewAquisitionWindowSize);
		valueGraph = CircularBuffer<float>(TargetViewAquisitionWindowSize, TargetViewAquisitionWindowSize);
		localMaximaStats.reset();
		localMaximas.clear();
		localMaxSearching = false;
		localMaxValue = 0.0f;
		localMaxFrame = -1;
	}
};

struct TargetViewRange
{
	FrameNum beginFrame, endFrame;
	BlockStats stats = {};
	ObsTarget target = {};
};

// Extract detailed information about the markes in a range of frames (potential Target View)
void prepareBlock(const SequenceData &sequences, FrameNum blockBegin, FrameNum blockEnd, const TargetAquisitionParameters &params, std::vector<int> &frames, std::map<int, int> &markers, std::vector<int> &sharedMarkers);

// Get simplified block statistics used to determine quality of potential Target View
BlockStats getBlockStats(std::vector<int> &frames, std::map<int, int> &markers, std::vector<int> &sharedMarkers);

// Update statistics for the given frame (used mostly for visualisation)
void updateTargetViewAquisitionStats(const SequenceData &sequences, TargetViewAquisition &aquisition, int frameNum, int offset);

// Update the aquisition state, searching for frame ranges to consider adopting as Target Views
bool updateTargetViewAquisition(TargetViewAquisition &aquisition, FrameNum frame, int offset, const TargetAquisitionParameters &params);

// Finalise the current aquisition range and produce a new Target View if it is good
bool finaliseTargetViewAquisitionRange(const SequenceData &sequences, TargetViewAquisition &aquisition, const TargetAquisitionParameters &params, TargetViewRange &newRange);

// Calculate the transform to the GT target, used to align target to GT for debugging
Eigen::Isometry3f getTransformToGT(const ObsTarget &target, const ObsTarget &targetGT);

// Debug results compared to GT of given target calibration - targetGT may be the same as target when not simuulating
void debugTargetResults(const ObsTarget &target, const ObsTarget &targetGT);

// Calculate proper reprojection error distribution of the target
OptErrorRes getTargetErrorDist(const std::vector<CameraCalib> &calibs, const ObsTarget &target);

#endif // CALIBRATION_TARGET_AQUISITION_H