/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef SEQUENCE_DATA_INL
#define SEQUENCE_DATA_INL

#include "point/sequence_data.hpp"

#include <cassert>
#include <map>


/* Structures */


struct OverlapRange
{
	int startA;
	int startB;
	int length;
};


/* Function Declarations */


/**
 * Filter out frames without any observations in between. FrameMap contains mappings from frame index to continuous index
 */
int getObservationFrameMap(const MarkerSequences &marker, std::map<int, int> &frameMap, int frameStart = 0, int frameEnd = std::numeric_limits<int>::max());

/**
 * Filter out frames without at least 2 observations in between. FrameMap contains mappings from frame index to continuous index
 */
int getTriangulationFrameMap(const MarkerSequences &marker, std::map<int, int> &frameMap, int frameStart = 0, int frameEnd = std::numeric_limits<int>::max());

/**
 * Iterate over all sequences observing the marker that are mapped using frameMap
 */
void handleMappedSequences(const MarkerSequences &marker, const std::map<int, int> &frameMap,
	const std::function<void(const PointSequence&, int, int, int, int, int)> &handleSequence);

/**
 * Returns the observations of all points shared between the two cameras camA and camB in the given frame range
 */
int getSharedObservations(const std::vector<MarkerSequences> &markers, int startFrame, int endFrame, int camA, int camB,
	std::vector<Eigen::Vector2f>  &pointsA, std::vector<Eigen::Vector2f> &pointsB);

/**
 * Calls the handleShared function for any shared range of observations between the two cameras camA and camB in the given frame range
 */
void handleSharedObservations(const std::vector<MarkerSequences> &markers, int startFrame, int endFrame, int camA, int camB,
	const std::function<void(int, CameraSequences::range_iterator, CameraSequences::range_iterator, int, int)> &handleShared);

/**
 * Checks for any overlap in the [camera, time] domain.
 * If there is, one camera observe each marker as a distinct sequence at some point, and the markers are distinct.
 * Returns the first frame that overlaps, or -1 if they don't overlap
 */
int canProveMarkerDistinct(const MarkerSequences &markerA, const MarkerSequences &markerB);

/**
 * Returns two ranges in each sequence that overlap temporarily
 */
static OverlapRange determineOverlap(const PointSequence &sequenceA, const PointSequence &sequenceB)
{
	OverlapRange overlap = {};
	int overlapStart = std::max(sequenceA.startFrame, sequenceB.startFrame);
	int overlapEnd = std::min(sequenceA.startFrame + sequenceA.length(), sequenceB.startFrame + sequenceB.length());
	if (overlapStart < overlapEnd)
	{ // Overlap
		overlap.startA = overlapStart - sequenceA.startFrame;
		overlap.startB = overlapStart - sequenceB.startFrame;
		overlap.length = overlapEnd - overlapStart;
	}
	return overlap;
}


/* Inline Function Definitions */


/**
 * Get the total number of recorded observations for the marker by this camera
 */
inline unsigned int CameraSequences::getSampleCount()
{
	unsigned int count = 0;
	for (auto &seq : sequences)
		count += seq.length(); // NOTE: This includes 
	return count;
}

/**
 * Get the total number of recorded observations for this marker
 */
inline unsigned int MarkerSequences::getSampleCount()
{
	unsigned int count = 0;
	for (auto &camera : cameras)
		count += camera.getSampleCount();
	return count;
}

/**
 * Get the total number of recorded markers
 */
inline unsigned int SequenceData::getSampleCount()
{
	unsigned int count = 0;
	for (auto &marker : markers)
		count += marker.getSampleCount();
	return count;
}

inline std::pair<int, float> MarkerSequences::resolveGTMarker() const
{ // Resolve most likely GT marker
	std::map<int, int> gtCandidates;
	for (auto &camera : cameras)
		for (auto &sequence : camera.sequences)
			gtCandidates[sequence.lastGTMarker] += sequence.length();
	int totalWeight = 0;
	std::pair<int, int> best;
	for (auto &cand : gtCandidates)
	{
		totalWeight += cand.second;
		if (best.second < cand.second)
			best = cand;
	}
	return { best.first, (float)best.second/totalWeight };
}

inline std::pair<int,int> MarkerSequences::getFrameRange() const
{
	std::pair<int,int> range = { std::numeric_limits<int>::max(), std::numeric_limits<int>::lowest() };
	for (int c = 0; c < cameras.size(); c++)
	{
		if (cameras[c].sequences.empty()) continue;
		range.first = std::min(range.first, cameras[c].sequences.front().startFrame);
		range.second = std::max(range.second, cameras[c].sequences.back().endFrame());
	}
	return range;
}


/* Implementation Details */


class CameraSequences::const_iterator {
private:
	CameraSequences *base;
public:
	int seq, index;

	const_iterator(const CameraSequences &obs, int Seq, int Index)
	{
		base = (CameraSequences*)&obs;
		seq = Seq;
		index = Index;
	}
	const_iterator& operator++() { return operator+=(1); }
	const_iterator operator++(int) { const_iterator retval = *this; operator++(); return retval; }
	const_iterator& operator--() { return operator-=(1); }
	const_iterator operator--(int) { const_iterator retval = *this; operator--(); return retval; }
	const_iterator& operator+=(int n)
	{
		index += n;
		while (seq < base->sequences.size() && base->sequences[seq].length() <= index)
		{ // Handle sequence break
			index -= base->sequences[seq].length();
			seq++;
		}
		if (seq >= base->sequences.size())
			index = 0; // Handle end
		return *this;
	}
	const_iterator operator+(int n) { const_iterator retval = *this; retval += n; return retval; }
	const_iterator& operator-=(int n)
	{
		index -= n;
		while (seq > 0 && index < 0)
		{ // Handle sequence break
			seq--;
			index += base->sequences[seq].length();
		}
		if (index < 0)
		{ // Handle rend
			seq = -1;
			index = 0;
		}
		return *this;
	}
	const_iterator operator-(int n) { const_iterator retval = *this; retval -= n; return retval; }
	bool operator<(const_iterator other) const { return seq < other.seq || (seq == other.seq && index < other.index); }
	bool operator<=(const_iterator other) const { return seq < other.seq || (seq == other.seq && index <= other.index); }
	bool operator>(const_iterator other) const { return seq > other.seq || (seq == other.seq && index > other.index); }
	bool operator>=(const_iterator other) const { return seq > other.seq || (seq == other.seq && index >= other.index); }
	bool operator==(const_iterator other) const { return seq == other.seq && index == other.index; }
	bool operator!=(const_iterator other) const { return !(*this == other); }
	bool operator()() const { return seq >= 0 && seq < base->sequences.size(); }

	// Accessors current pos
	using value_type = const Eigen::Vector2f;
	const value_type operator*() const { return base->sequences[seq].getPoint(index); }
	const value_type raw() const { return base->sequences[seq].getRaw(index); }

	int frame() const { return base->sequences.size() <= seq? (base->sequences.empty()? -1 : base->sequences.back().endFrame()) : (base->sequences[seq].startFrame + index); }
	// Access continuous ranges
	CameraSequences::range_iterator rangeStart() const { return base->sequences[seq].points.begin()+index; } 
	CameraSequences::range_iterator rangeRawStart() const { return base->sequences[seq].rawPoints.begin()+index; } 
	int rangeLength() const { return base->sequences[seq].length()-index; } 
	// iterator traits
	using pointer = const value_type*;
	using reference = const value_type&;
	using difference_type = int;
	using iterator_category = std::bidirectional_iterator_tag;
};

inline CameraSequences::const_iterator CameraSequences::begin() const { return const_iterator(*this, 0, 0); }
inline CameraSequences::const_iterator CameraSequences::end() const { return const_iterator(*this, sequences.size(), 0); }
inline CameraSequences::const_iterator CameraSequences::rbegin() const { return const_iterator(*this, sequences.size()-1, sequences.back().length()-1); }
inline CameraSequences::const_iterator CameraSequences::rend() const { return const_iterator(*this, -1, 0); }

/**
 * Return iterator of frame - if not possible, end
 */
inline CameraSequences::const_iterator CameraSequences::frame(int frame) const
{
	for (int s = (int)sequences.size()-1; s >= 0; s--)
	{
		if (sequences[s].endFrame() <= frame)
			return end();
		if (sequences[s].startFrame <= frame)
			return const_iterator(*this, s, frame-sequences[s].startFrame);
	}
	return end();
}

/**
 * Return iterator of frame or higher - if not available, end
 */
inline CameraSequences::const_iterator CameraSequences::upper(int frame) const
{
	for (int s = (int)sequences.size()-1; s >= 0; s--)
	{
		if (sequences[s].endFrame() <= frame)
			return const_iterator(*this, s+1, 0); // may be end
		if (sequences[s].startFrame <= frame)
			return const_iterator(*this, s, frame-sequences[s].startFrame);
	}
	return begin(); // may be end
}

/**
 * Return iterator of frame or lower - if not available, begin
 */
inline CameraSequences::const_iterator CameraSequences::lower(int frame) const
{
	for (int s = (int)sequences.size()-1; s >= 0; s--)
	{
		if (sequences[s].endFrame() <= frame)
			return const_iterator(*this, s, sequences[s].length()-1);
		if (sequences[s].startFrame <= frame)
			return const_iterator(*this, s, frame-sequences[s].startFrame);
	}
	return begin();
}

#endif // SEQUENCE_DATA_INL