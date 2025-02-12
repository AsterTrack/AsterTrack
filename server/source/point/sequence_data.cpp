/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include "point/sequence_data.inl"
#include "util/stats.hpp"


/**
 * Filter out frames without any observations in between. FrameMap contains mappings from frame index to continuous index
 */
int getObservationFrameMap(const MarkerSequences &marker, std::map<int, int> &frameMap, int frameStart, int frameEnd)
{
	// Find first frame and prepare
	int frameCount = 0;
	auto range = marker.getFrameRange();
	int curFrame = std::max(range.first, frameStart);
	if (curFrame >= frameEnd) return 0;

	// Setup first frame
	frameMap.clear();
	frameMap.insert({ curFrame, frameCount });
	std::vector<int> curSeq(marker.cameras.size());
	for (int c = 0; c < marker.cameras.size(); c++)
	{ // Advance sequence index to first frame or beyond
		curSeq[c] = marker.cameras[c].upper(curFrame).seq;
	}

	// Now get frames with at least one observation
	// Take the longest sequence of the active frame and go to its end, if none is there go to the start of the next sequence
	// Along the way, record the active frame periods in frameMap so that they can be mapped to a continuous frame timeline
	int lastPeriodFrame = curFrame;
	while (curFrame < frameEnd)
	{
		// Find extends of next period
		int maxIncrement = std::numeric_limits<int>::min(), minIncrement = std::numeric_limits<int>::max();
		for (int c = 0; c < marker.cameras.size(); c++)
		{
			const CameraSequences &camObs = marker.cameras[c];
			if (curSeq[c] >= camObs.sequences.size()) continue;
			auto &seq = camObs.sequences[curSeq[c]];
			if (seq.startFrame <= curFrame)
				maxIncrement = std::max(maxIncrement, seq.startFrame + (int)seq.points.size());
			else
				minIncrement = std::min(minIncrement, seq.startFrame);
		}
		if (maxIncrement <= curFrame)
		{ // No measurements in this period
			if (minIncrement <= curFrame)
				break; // No next period
			if (minIncrement == std::numeric_limits<int>::max())
				break; // No more sequences
			// Skip until next period with certain measurements, and register start of it
			curFrame = minIncrement;
			frameMap.insert({ curFrame, frameCount });
		}
		else
		{ // Measurements until at least maxIncrement, continue currently registered period
			int advanceTo = std::min(maxIncrement, frameEnd);
			frameCount += advanceTo-curFrame;
			curFrame = advanceTo;
			lastPeriodFrame = curFrame;
		}
		// Advance curSeq to sequences active in curFrame or later
		for (int c = 0; c < marker.cameras.size(); c++)
		{
			const CameraSequences &camObs = marker.cameras[c];
			int &i = curSeq[c];
			while (i < camObs.sequences.size() && camObs.sequences[i].startFrame + camObs.sequences[i].points.size() <= curFrame)
				i++;
		}
	}
	frameMap.insert({ lastPeriodFrame, frameCount });
	return frameCount;
}

/**
 * Filter out frames without at least 2 observations in between. FrameMap contains mappings from frame index to continuous index
 */
int getTriangulationFrameMap(const MarkerSequences &marker, std::map<int, int> &frameMap, int frameStart, int frameEnd)
{
	// Find first frame and prepare
	DualExtremum<int> minFrame(std::numeric_limits<int>::max()), maxFrame(std::numeric_limits<int>::lowest());
	for (int c = 0; c < marker.cameras.size(); c++)
	{
		if (!marker.cameras[c].sequences.empty())
		{
			minFrame.min(marker.cameras[c].sequences.front().startFrame);
			maxFrame.max(marker.cameras[c].sequences.back().startFrame + marker.cameras[c].sequences.back().length());
		}
	}
	if (!minFrame.hasAll())
		return 0; // No overlap of at least two frames

	// This case assumes if there is observations by at least two cameras, that they overlap (needed for correspondence matching anyway)
	int frameCount = 0;
	int curFrame = minFrame.rank[1];
	if (frameStart > curFrame)
		curFrame = frameStart;
	if (frameStart >= maxFrame.rank[1])
		return 0; // This marker has no timely overlap

	// Setup first frame
	frameMap.clear();
	frameMap.insert({ curFrame, frameCount });

	std::vector<int> curSeq(marker.cameras.size());
	for (int c = 0; c < marker.cameras.size(); c++)
	{ // Advance sequence index to first frame or beyond
		curSeq[c] = marker.cameras[c].upper(curFrame).seq;
	}

	// Now get frames with at least one observation
	// Take the longest sequence of the active frame and go to its end, if none is there go to the start of the next sequence
	// Along the way, record the active frame periods in frameMap so that they can be mapped to a continuous frame timeline
	bool inPeriod = true;
	int lastPeriodFrame = 0;
	while (curFrame < frameEnd)
	{
		// Find extends of next period
		DualExtremum<int> minIncrement(std::numeric_limits<int>::max()), maxIncrement(std::numeric_limits<int>::lowest());
		int seqNum = 0;
		for (int c = 0; c < marker.cameras.size(); c++)
		{
			const CameraSequences &camObs = marker.cameras[c];
			if (curSeq[c] >= camObs.sequences.size()) continue;
			auto &seq = camObs.sequences[curSeq[c]];
			if (seq.startFrame <= curFrame)
			{
				maxIncrement.max(seq.startFrame + (int)seq.points.size());
				seqNum++;
			}
			else
				minIncrement.min(seq.startFrame);
		}
		if (seqNum == 0)
		{
			if (!minIncrement.hasAll()) break;
			curFrame = minIncrement.rank[1];
			inPeriod = false;
		}
		else if (seqNum == 1)
		{ // No two observations in this frame, skip until the next frame with possibly two observations
			if (!minIncrement.hasAny()) break; // No next second observation
			curFrame = minIncrement.rank[0];
			// Cannot assume this if there are periods of NO observations (if markers are consolidated)
			//frameMap.insert({ curFrame, frameCount });
			inPeriod = false;
		}
		else
		{ // Two concurrent observations until at least the second maximum end frame, continue currently registered period
			if (!inPeriod)
			{
				frameMap.insert({ curFrame, frameCount });
				inPeriod = true;
			}
			int advanceTo = std::min(maxIncrement.rank[1], frameEnd);
			frameCount += advanceTo-curFrame;
			curFrame = advanceTo;
			lastPeriodFrame = curFrame;
		}
		// Advance curSeq to sequences active in curFrame or later
		for (int c = 0; c < marker.cameras.size(); c++)
		{
			const CameraSequences &camObs = marker.cameras[c];
			int &i = curSeq[c];
			while (i < camObs.sequences.size() && camObs.sequences[i].startFrame + camObs.sequences[i].points.size() <= curFrame)
				i++;
		}
	}
	frameMap.insert({ lastPeriodFrame, frameCount });
	return frameCount;
}

/**
 * Iterate over all sequences observing marker m that are mapped using frameMap
 */
void handleMappedSequences(const MarkerSequences &marker, const std::map<int, int> &frameMap, 
	const std::function<void(const PointSequence&, int, int, int, int, int)> &handleSequence)
{
	if (frameMap.size() < 2)
		return;
	for (int c = 0; c < marker.cameras.size(); c++)
	{
		const CameraSequences &camObs = marker.cameras[c];
		auto mapA = frameMap.begin();
		auto mapB = std::next(mapA);
		auto seq = camObs.sequences.begin();
		int s = 0;
		// Get mapped part of sequence (the part that has overlap with at least one other sequence)
		while (mapB != frameMap.end() && seq != camObs.sequences.end())
		{
			int seqOffset = std::max(0, mapA->first - seq->startFrame);
			if (seqOffset >= seq->length())
			{ // No overlap, next sequence
				seq++;
				s++;
				continue;
			}
			int mapLength = mapB->second - mapA->second;
			int mapOffset = std::max(0, seq->startFrame - mapA->first);
			if (mapOffset >= mapLength)
			{ // No overlap, next mapped period
				mapA = mapB;
				mapB = std::next(mapB);
				continue;
			}
			int start = mapA->second + mapOffset;
			int length = std::min(mapLength - mapOffset, (int)seq->length() - seqOffset);
			handleSequence(*seq, c, s, seqOffset, start, length);
			if (length == seq->length() - seqOffset)
			{
				seq++;
				s++;
			}
			if (length == mapLength - mapOffset)
			{
				mapA = mapB;
				mapB = std::next(mapB);
			}
		}
	}
}

/**
 * Returns the observations of all points shared between the two cameras camA and camB in the given frame range
 */
int getSharedObservations(const std::vector<MarkerSequences> &markers, int startFrame, int endFrame, int camA, int camB,
	std::vector<Eigen::Vector2f>  &pointsA, std::vector<Eigen::Vector2f> &pointsB)
{
	int pointCount = 0;
	for (int m = 0; m < markers.size(); m++)
	{ // Add one unique 3D point after another
		const CameraSequences &cameraA = markers[m].cameras[camA], &cameraB = markers[m].cameras[camB];
		auto itA = cameraA.upper(startFrame), itB = cameraB.upper(startFrame);
		auto endA = cameraA.upper(endFrame), endB = cameraB.upper(endFrame);

		while (itA < endA && itB < endB)
		{
			// Advance to next overlap
			while (itA < endA && itA.frame() < itB.frame())
				itA += std::min(itB.frame()-itA.frame(), itA.rangeLength()+1);
			if (itA >= endA) break;
			while (itB < endB && itB.frame() < itA.frame())
				itB += std::min(itA.frame()-itB.frame(), itB.rangeLength()+1);
			if (itB >= endB) break;
			 // Check if there was an overlap in the last few sequences checked
			if (itA.frame() != itB.frame()) continue;
			// Calculate length of overlap
			int overlapLength = std::min(std::min(itA.rangeLength(), itB.rangeLength()), endFrame-itA.frame());
			// Add overlapping observations of the same point to the point correspondence
			pointsA.insert(pointsA.end(), itA.rangeStart(), itA.rangeStart()+overlapLength);
			pointsB.insert(pointsB.end(), itB.rangeStart(), itB.rangeStart()+overlapLength);
			itA += overlapLength;
			itB += overlapLength;
			pointCount += overlapLength;
		}
	}
	return pointCount;
}

/**
 * Calls the handleShared function for any shared range of observations between the two cameras camA and camB in the given frame range
 */
void handleSharedObservations(const std::vector<MarkerSequences> &markers, int startFrame, int endFrame, int camA, int camB,
	const std::function<void(int, CameraSequences::range_iterator, CameraSequences::range_iterator, int, int)> &handleShared)
{
	int minSize = std::max(camA, camB)+1;
	for (int m = 0; m < markers.size(); m++)
	{ // Add one unique 3D point after another
		if (markers[m].cameras.size() < minSize) continue;
		const CameraSequences &cameraA = markers[m].cameras[camA], &cameraB = markers[m].cameras[camB];
		auto itA = cameraA.upper(startFrame), itB = cameraB.upper(startFrame);
		auto endA = cameraA.upper(endFrame), endB = cameraB.upper(endFrame);

		while (itA < endA && itB < endB)
		{
			// Advance to next overlap
			while (itA < endA && itA.frame() < itB.frame())
				itA += std::min(itB.frame()-itA.frame(), itA.rangeLength()+1);
			if (itA >= endA) break;
			while (itB < endB && itB.frame() < itA.frame())
				itB += std::min(itA.frame()-itB.frame(), itB.rangeLength()+1);
			if (itB >= endB) break;
			 // Check if there was an overlap in the last few sequences checked
			if (itA.frame() != itB.frame()) continue;
			// Calculate length of overlap
			int overlapLength = std::min(std::min(itA.rangeLength(), itB.rangeLength()), endFrame-itA.frame());
			handleShared(m, itA.rangeStart(), itB.rangeStart(), itA.frame(), overlapLength);
			itA += overlapLength;
			itB += overlapLength;
		}
	}
}

/**
 * Checks for any overlap in the [camera, time] domain.
 * If there is, one camera observe each marker as a distinct sequence at some point, and the markers are distinct.
 * Returns the first frame that overlaps, or -1 if they don't overlap
 */
int canProveMarkerDistinct(const MarkerSequences &markerA, const MarkerSequences &markerB)
{
	int cams = std::min(markerA.cameras.size(), markerB.cameras.size());
	for (int c = 0; c < cams; c++)
	{
		auto &camA = markerA.cameras[c];
		auto &camB = markerB.cameras[c];
		for (int seqItA = 0, seqItB = 0; seqItA < camA.sequences.size() && seqItB < camB.sequences.size();)
		{
			auto &seqA = camA.sequences[seqItA];
			auto &seqB = camB.sequences[seqItB];
			if (seqA.startFrame < seqB.endFrame())
			{
				if (seqA.endFrame() > seqB.startFrame)
					return std::max(seqA.startFrame, seqB.startFrame);
				seqItA++;
			}
			else //if (seqB.startFrame < seqA.endFrame())
			{
				if (seqB.endFrame() > seqA.startFrame)
					return std::max(seqA.startFrame, seqB.startFrame);
				seqItB++;
			}
		}
	}
	return -1; // found no overlap
}