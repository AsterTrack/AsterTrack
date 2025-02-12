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

#include "aquisition.hpp"

#include "calib/obs_data.inl"
#include "calib/opt/utilities.hpp" // getErrorStats
#include "point/sequence_data.inl"

#include "util/eigenalg.hpp"
#include "util/log.hpp"

#include <cassert>


/**
 * Calibration of target structure
 */

// Stat params
const int sharedFrames = 10;

// Search params
// const int TargetViewAquisitionWindowSize;
const int slidingWindowSize = 100;




// Extract detailed information about the markes in a range of frames (potential Target View)
void prepareBlock(const SequenceData &sequences, int blockBegin, int blockEnd, const TargetAquisitionParameters &params, std::vector<int> &frames, std::map<int, int> &markers, std::vector<int> &sharedMarkers)
{
	frames.clear();
	frames.resize(blockEnd-blockBegin);
	sharedMarkers.clear();
	sharedMarkers.resize(blockEnd-blockBegin);
	markers.clear();
	for (int m = 0; m < sequences.markers.size(); m++)
	{
		const MarkerSequences &marker = sequences.markers[m];

		auto range = marker.getFrameRange();
		if (range.first > blockEnd-10 || range.second < blockBegin+10) continue;

		int first = std::numeric_limits<int>::max(), last = std::numeric_limits<int>::lowest();
		int markers2D = 0;
		for (const CameraSequences &cam : marker.cameras)
		{
			auto begin = cam.upper(blockBegin), end = cam.upper(blockEnd);
			if (begin < end)
			{
				first = std::min(first, begin.frame());
				last = std::max(last, std::prev(end).frame());
				assert(last < blockEnd);
			}
			for (auto cur = begin; cur < end; cur++)
			{
				frames[cur.frame()-blockBegin]++;
				markers2D++;
			}
		}
		assert(markers2D > 0);
		if (markers2D > params.minMarkerObsCount*1.5)
		{ // Filter markers first
			markers[m] = markers2D;
			for (long f = first-blockBegin; f < last+1-blockBegin; f++)
				sharedMarkers[f]++;
		}
	}
	// Then filter frames based on optimistic, pre-marker-filter stats
	for (int f = 0; f < frames.size(); f++)
	{
		frames[f] = frames[f] < params.minFrameObsCount || sharedMarkers[f] < params.minSharedMarkerCount? -1 : 0;
		sharedMarkers[f] = frames[f];
	}
	for (auto m = markers.begin(); m != markers.end();)
	{
		if (m->second < params.minMarkerObsCount*1.5) m = markers.erase(m);
		else m++;
	}

	// Then re-determine frame and marker stats
	for (auto m = markers.begin(); m != markers.end(); m++)
	{
		const MarkerSequences &marker = sequences.markers[m->first];
		m->second = 0;
		int first = std::numeric_limits<int>::max(), last = std::numeric_limits<int>::lowest();
		for (const CameraSequences &cam : marker.cameras)
		{
			auto begin = cam.upper(blockBegin), end = cam.upper(blockEnd);
			if (begin < end)
			{
				first = std::min(first, begin.frame());
				last = std::max(last, std::prev(end).frame());
				assert(last < blockEnd);
			}
			for (auto cur = begin; cur < end; cur++)
			{
				auto &frame = frames[cur.frame()-blockBegin];
				if (frame >= 0)
				{
					frame++;
					m->second++;
				}
			}
		}
		// Record as shared markers even if a specific frame lacks a marker
		for (int f = first-blockBegin; f < last+1-blockBegin; f++)
		{
			if (sharedMarkers[f] >= 0)
				sharedMarkers[f]++;
		}
	}
	for (auto m = markers.begin(); m != markers.end();)
	{
		if (m->second < params.minMarkerObsCount) m = markers.erase(m);
		else m++;
	}
}

// Get simplified block statistics used to determine quality of potential Target View
BlockStats getBlockStats(std::vector<int> &frames, std::map<int, int> &markers, std::vector<int> &sharedMarkers)
{
	BlockStats stats;
	stats.minMarkerObs = std::numeric_limits<int>::max();
	stats.sampleCount = 0;
	for (auto &m : markers)
	{
		stats.minMarkerObs = std::min(stats.minMarkerObs, m.second);
		stats.sampleCount += m.second;
	}
	stats.markerCount = markers.size();
	stats.frameCount = 0;
	stats.minFrameObs = std::numeric_limits<int>::max();
	stats.minSharedMarkers = std::numeric_limits<int>::max();
	for (int f = 0; f < frames.size(); f++)
	{
		if (frames[f] >= 0)
		{
			stats.frameCount++;
			stats.minFrameObs = std::min(stats.minFrameObs, frames[f]);
			stats.minSharedMarkers = std::min(stats.minSharedMarkers, sharedMarkers[f]);
		}
	}

	stats.paramCount = stats.markerCount*3 + stats.frameCount*6;
	return stats;
}

// Update statistics for the given frame (used mostly for visualisation)
void updateTargetViewAquisitionStats(const SequenceData &sequences, TargetViewAquisition &aquisition, int frameNum, int offset)
{
	int markers2D = 0;
	float avgMarkers2D = 0;
	int triangulations = 0;
	int sharedMarkers = 0;

	for (int m = 0; m < sequences.markers.size(); m++)
	{
		const MarkerSequences &marker = sequences.markers[m];

		int numMarkers = 0;
		float avgMarkers = 0;
		bool isShared = false;
		for (int c = 0; c < marker.cameras.size(); c++)
		{
			const CameraSequences &cam = marker.cameras[c];

			auto frame = cam.upper(frameNum-sharedFrames+1);
			int sharedCount = 0;
			while (frame != cam.end() && frame.frame() <= frameNum)
			{
				sharedCount++;
				if (frame.frame() == frameNum)
				{
					numMarkers++;
					break;
				}
				frame++;
			}
			avgMarkers += (float)sharedCount/sharedFrames;
			if (sharedCount == sharedFrames)
				isShared = true;
		}
		if (numMarkers > 1)
			triangulations++;
		markers2D += numMarkers;
		avgMarkers2D += avgMarkers;
		if (isShared)
			sharedMarkers++;
	}

	if (offset == 0)
	{ // Push new values
		aquisition.markerCount.push_back(markers2D);
		aquisition.avgMarkerCount.push_back(avgMarkers2D);
		aquisition.triangulationsCount.push_back(triangulations);
		aquisition.sharedMarkerCount.push_back(sharedMarkers);
	}
	else
	{ // Re-evaluate past values
		*std::prev(aquisition.markerCount.end(), offset) = markers2D;
		*std::prev(aquisition.avgMarkerCount.end(), offset) = avgMarkers2D;
		*std::prev(aquisition.triangulationsCount.end(), offset) = triangulations;
		*std::prev(aquisition.sharedMarkerCount.end(), offset) = sharedMarkers;
	}
}

// Update the aquisition state, searching for frame ranges to consider adopting as Target Views
bool updateTargetViewAquisition(TargetViewAquisition &aquisition, int frame, int offset, const TargetAquisitionParameters &params)
{
	// Remove ranges out of window bounds
	while (!aquisition.localMaximas.empty() && aquisition.localMaximas.front().begin <= frame+offset-TargetViewAquisitionWindowSize)
		aquisition.localMaximas.erase(aquisition.localMaximas.begin());

	// Move value graph along
	aquisition.valueGraph.push_back(0);
	aquisition.currentValue = 0.0f;

	if (*std::prev(aquisition.sharedMarkerCount.end(), offset) < params.minSharedMarkerCount)
	{ // Useless frame, block aquisition until it's outside sliding window size
		aquisition.nextUpdate = frame+slidingWindowSize+1;
		aquisition.localMaxSearching = false;
		aquisition.localMaxFrame = -1;
		aquisition.localMaxValue = 0.0f;
		return false;
	}

	if (aquisition.nextUpdate < frame)
	{ // Init (used only if frist frame is not invalid)
		aquisition.nextUpdate = frame+slidingWindowSize;
		return false;
	}
	else if (aquisition.nextUpdate > frame)
		return false; // Wait
	else // Handle frame
		aquisition.nextUpdate++;

	float newValue = 0.0f;
	{ // Update sliding window average
		auto it = std::prev(aquisition.sharedMarkerCount.end(), offset);
		for (int i = 0; i < slidingWindowSize; i++, it--)
			newValue += *it;
		newValue /= slidingWindowSize;
		*std::prev(aquisition.valueGraph.end(), offset+slidingWindowSize/2) = newValue;
		aquisition.currentValue = newValue;
	}

	const float minRelMaxDiff = 1.05f;

	if (aquisition.localMaxSearching)
	{ // Graph is rising, search for local maximum
		if (newValue > aquisition.localMaxValue)
		{ // Still rising
			aquisition.localMaxValue = newValue;
			aquisition.localMaxFrame = frame-slidingWindowSize/2;
			return false;
		}
		else if (frame-aquisition.localMaxFrame < slidingWindowSize/2+10 || newValue*minRelMaxDiff > aquisition.localMaxValue)
		{ // Give a few frames of buffer
			return false;
		}
		// Else: dropped off, record as local maximum
		aquisition.localMaximaStats.update(aquisition.localMaxValue);
		if (aquisition.localMaxValue < aquisition.localMaximaStats.avg)
			return false;
		TargetViewAquisition::LocalMaxima range;
		range.maxima = aquisition.localMaxFrame;
		range.value = aquisition.localMaxValue;
		// Extend range to the left
		int maxOffset = frame-aquisition.localMaxFrame;
		auto it = std::prev(aquisition.sharedMarkerCount.end(), offset+maxOffset+slidingWindowSize/2);
		int i = 0;
		for (; i < slidingWindowSize; i++, it--)
			if (*it < aquisition.localMaxValue*0.9f)
				break;
		range.begin = aquisition.localMaxFrame-slidingWindowSize/2-i;
		it = std::prev(aquisition.sharedMarkerCount.end(), offset+maxOffset-slidingWindowSize/2);
		i = 0;
		for (; i < slidingWindowSize; i++, it++)
			if (*it < aquisition.localMaxValue*0.9f)
				break;
		range.end = aquisition.localMaxFrame+slidingWindowSize/2+i;
		LOG(LTargetCalib, LInfo, "Got new maxima at %d, range %d-%d (%d length)",
			aquisition.localMaxFrame, range.begin, range.end, (range.end-range.begin));
		aquisition.localMaximas.push_back(range);
		aquisition.localMaxSearching = false;
		aquisition.localMaxFrame = -1;
		aquisition.localMaxValue = newValue;
		return true;
	}
	else
	{ // Graph is falling, wait for it to rise again
		if (newValue > aquisition.localMaxValue*minRelMaxDiff)
		{ // Consider graph as rising again
			aquisition.localMaxSearching = true;
			aquisition.localMaxValue = newValue;
			aquisition.localMaxFrame = frame-slidingWindowSize/2;
		}
		else if (newValue < aquisition.localMaxValue)
			aquisition.localMaxValue = newValue;
	}
	return false;
}

// Finalise the current aquisition range and produce a new Target View if it is good
bool finaliseTargetViewAquisitionRange(const SequenceData &sequences, TargetViewAquisition &aquisition, const TargetAquisitionParameters &params, TargetViewRange &newRange)
{
	TargetViewAquisition::LocalMaxima max = aquisition.localMaximas.back();
	int begin = max.begin, end = max.end;

	LOG(LTargetCalib, LDebug, "Finalising Range %d-%d, max %f!", begin, end, max.value);

	LOG(LTargetCalib, LDebug, "Stats are %f avg, %f first sigma!", aquisition.localMaximaStats.avg, aquisition.localMaximaStats.avg+aquisition.localMaximaStats.stdDev());
	if (max.value < aquisition.localMaximaStats.avg*0.9999f-aquisition.localMaximaStats.stdDev())
	{
		LOG(LTargetCalib, LDebug, "Too small maxima!");
		return false;
	}

	if (end-begin < params.minLength)
		return false;

	std::vector<int> framesObsCnt;
	std::map<int, int> markersObsCnt;
	std::vector<int> sharedMarkerCnt;
	prepareBlock(sequences, begin, end, params, framesObsCnt, markersObsCnt, sharedMarkerCnt);
	BlockStats stats = getBlockStats(framesObsCnt, markersObsCnt, sharedMarkerCnt);

	if (stats.frameCount < params.minLength)
		return false;

	float ratio = (float)stats.sampleCount/stats.paramCount;
	if (ratio < params.ratioHardLimit)
	{
		LOG(LTargetCalib, LDebug, "Not recording frame range from %d (%d frames), ratio %.2f, with %d samples for %d params over %d frames (min %d values) and %d markers (min %d values)\n",
			begin, end-begin, ratio,
			stats.sampleCount, stats.paramCount, stats.frameCount, stats.minFrameObs, stats.markerCount, stats.minMarkerObs);
		return false;
	}

	// Setup frames
	std::vector<int> frames;
	frames.reserve(framesObsCnt.size());
	for (int f = 0; f < framesObsCnt.size(); f++)
		if (framesObsCnt[f] > params.minFrameObsCount && sharedMarkerCnt[f] > params.minSharedMarkerCount)
			frames.push_back(begin+f);
	if (frames.size() < params.minLength)
	{
		LOG(LTargetCalib, LDebug, "Not recording frame range from %d (%d frames), have only %d frames left!",
			begin, end-begin, (int)frames.size());
		return false;
	}

	LOG(LTargetCalib, LDebug, "Recording frame range from %d (%d frames), ratio %.2f, with %d samples for %d params over %d frames (min %d values) and %d markers (min %d values)\n",
		begin, end-begin, ratio,
		stats.sampleCount, stats.paramCount, stats.frameCount, stats.minFrameObs, stats.markerCount, stats.minMarkerObs);

	newRange = {};
	newRange.beginFrame = begin;
	newRange.endFrame = end;
	newRange.stats = stats;

	// Setup markers
	int i = 0;
	for (auto m : markersObsCnt)
		newRange.target.markerMap[m.first] = i++;
	newRange.target.markers.resize(i);

	// Gather observations for target
	addTargetObservations(newRange.target, sequences.markers, frames);

	newRange.stats.frameCount = newRange.target.frames.size();
	newRange.stats.markerCount = newRange.target.markers.size();
	newRange.stats.sampleCount = newRange.target.totalSamples;

	return true;
}

// Calculate the transform to the GT target, used to align target to GT for debugging
Eigen::Isometry3f getTransformToGT(const ObsTarget &target, const ObsTarget &targetGT)
{
	std::vector<int> gtMap(target.markers.size(), -1);
	for (auto map : target.markerMap)
	{
		int gt = targetGT.markerMap.at(map.first);
		if (gtMap[map.second] != -1 && gtMap[map.second] != gt)
		{ // targetGT currently contains double values, so might be fine
			LOG(LTargetCalib, LDebug, "!!! Potential conflict of marker assignment! Target maps a marker to both gt markers %d and %d\n", gtMap[map.second], gt);
		}
		gtMap[map.second] = gt;
	}
	// Collect markers as corresponding point clouds
	Eigen::MatrixXf trMat(target.markers.size(), 3);
	Eigen::MatrixXf mkMat(3, target.markers.size());
	for (int i = 0; i < target.markers.size(); i++)
	{
		trMat.row(i) = targetGT.markers[gtMap[i]];
		mkMat.col(i) = target.markers[i];
	}

	// Determine affine transformation between them
	return kabsch<float,Eigen::Isometry>(trMat, mkMat);
}

// Debug results compared to GT of given target calibration - targetGT may be the same as target when not simuulating
void debugTargetResults(const ObsTarget &target, const ObsTarget &targetGT)
{
	LOG(LTargetCalib, LDebug, "Comparing targets - frames original %d, gt %d!\n", (int)target.frames.size(), (int)targetGT.frames.size());
	Eigen::Isometry3f GTTransform = getTransformToGT(target, targetGT);

	float frameErrorPos = 0, frameErrorRot = 0, frameErrorMarker = 0, markerError = 0;
	auto it = target.frames.begin(), itGT = targetGT.frames.begin();
	while (it != target.frames.end())
	{
		while (itGT != targetGT.frames.end() && itGT->frame < it->frame)
			itGT++; // Account for deleted frames in target (not deleted in GT target)
		assert(itGT != targetGT.frames.end());
		assert(it->frame == itGT->frame);

		Eigen::Isometry3f poseDiff = it->pose * (itGT->pose * GTTransform).inverse();
		frameErrorPos += poseDiff.translation().norm();
		frameErrorRot += Eigen::AngleAxisf(poseDiff.rotation()).angle();

		{ // Calculate 3D error for each 2D observation
			float error3D = 0;
			for (auto &sample : it->samples)
			{
				Eigen::Vector3f pt3D = it->pose * target.markers[target.markerMap.at(sample.marker)];
				Eigen::Vector3f gt3D = itGT->pose * targetGT.markers[targetGT.markerMap.at(sample.marker)];
				error3D += (pt3D-gt3D).norm();
			}
			error3D = error3D/it->samples.size();
			frameErrorMarker += error3D;
		}

		it++; itGT++;
	}
	frameErrorPos = frameErrorPos/target.frames.size()*1000;
	frameErrorRot = frameErrorRot/target.frames.size()/(float)PI*180;
	frameErrorMarker = frameErrorMarker/target.frames.size()*1000;
	LOG(LTargetCalib, LDebug, "Frame pose error avg is %fmm and %fdg over %d frames\n", frameErrorPos, frameErrorRot, (int)target.frames.size());

	std::vector<int> gtMap(target.markers.size(), -1);
	for (auto map : target.markerMap)
		gtMap[map.second] = targetGT.markerMap.at(map.first);
	for (int m = 0; m < target.markers.size(); m++)
	{
		Eigen::Vector3f markerDiff = (GTTransform * target.markers[m]) - targetGT.markers[gtMap[m]];
		markerError += markerDiff.norm();
	}
	markerError = markerError/target.markers.size() * 1000	;
	LOG(LTargetCalib, LDebug, "Average 3D error per frame is %fmm, resulting marker error is %fmm for %d markers\n", frameErrorMarker, markerError, (int)target.markers.size());
}

// Calculate proper reprojection error distribution of the target
OptErrorRes getTargetErrorDist(const std::vector<CameraCalib> &calibs, const ObsTarget &target)
{
	if (target.frames.empty()) return {};

	Eigen::VectorXf errorVec(target.totalSamples);
	int i = 0;
	for (auto &frame : target.frames)
	{
		// 2D reprojection error
		for (auto &samples : frame.samples)
		{
			int m = target.markerMap.at(samples.marker);
			Eigen::Vector3f pt3D = frame.pose * target.markers[m];
			Eigen::Vector2f proj = (calibs[samples.camera].camera.cast<float>() * pt3D.homogeneous()).hnormalized().head<2>();
			Eigen::Vector2f pt2D = undistortPoint(calibs[samples.camera], samples.point);
			errorVec(i++) = (proj-pt2D).norm();
		}
	}
	assert(i == errorVec.size());

	return getErrorStats(errorVec);
}