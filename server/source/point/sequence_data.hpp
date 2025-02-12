/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef SEQUENCE_DATA_H
#define SEQUENCE_DATA_H

#include "util/eigendef.hpp"


/* Structures */

struct PointSequence;
struct CameraSequences;
struct MarkerSequences;
struct SequenceData;


/**
 * A sequence of 2D points, likely from the same 3D marker, observed by one camera over time
 * Can be inactive, approximated by a single point, or active
 * Records outliers separately from the recorded point list
 */
struct PointSequence 
{
	int startFrame;
	int marker = -1;
	float value; // Flowing average of value grading of blob sequence
	Eigen::Vector2f avgPos = Eigen::Vector2f::Zero(); // Flowing average, used as inactive center
	// Observations
	std::vector<Eigen::Vector2f> points;
	std::vector<Eigen::Vector2f> rawPoints;
	// Inactive Sequence Observations
	int inactiveLength = 0;
	// TODO: Add standard deviation? See StatValue for running M2/stdDev calculations

	int lastGTMarker = -1; // For debugging only

	PointSequence() {};
	PointSequence(int StartFrame, Eigen::Vector2f point2D, Eigen::Vector2f raw2D, float val)
		: startFrame(StartFrame), avgPos(point2D), points({ point2D }), rawPoints({ raw2D }), marker(-1), value(val) {};

	inline bool isInactive() const { return inactiveLength > 0; }
	inline int length() const { return inactiveLength > 0? inactiveLength : (int)points.size(); }
	inline int lastFrame() const { return startFrame + length() - 1; }
	inline int endFrame() const { return startFrame + length(); }
	inline Eigen::Vector2f getPoint(int p) const { return inactiveLength > 0? avgPos : points[p]; }
	inline Eigen::Vector2f getRaw(int p) const { return inactiveLength > 0? avgPos : rawPoints[p]; }
};

/**
 * Observations of a single marker by a camera as multiple interrupted sequences
 */
struct CameraSequences
{
	std::vector<PointSequence> sequences;
	// TODO: Add current 2D size

	inline unsigned int getSampleCount();

	using range_iterator = std::vector<Eigen::Vector2f>::const_iterator;
	class const_iterator;

	inline const_iterator begin() const;
	inline const_iterator end() const;
	inline const_iterator rbegin() const;
	inline const_iterator rend() const;

	inline const_iterator frame(int frame) const;
	inline const_iterator upper(int frame) const;
	inline const_iterator lower(int frame) const;
};

/**
 * Observations of a single marker by multiple cameras, correlated with epipolar geometry
 */
struct MarkerSequences
{
	int lastFrame = 0;
	std::vector<CameraSequences> cameras; // Follows PipelineState::cameras mapping
	// TODO: Add 3D size?

	inline unsigned int getSampleCount();

	inline std::pair<int, float> resolveGTMarker() const;

	inline std::pair<int,int> getFrameRange() const;
};

/**
 * Observations of multiple markers by multiple cameras, and the corresponding epipolar constraints
 */
struct SequenceData
{
	int lastRecordedFrame;
	std::vector<MarkerSequences> markers;
	std::vector<std::vector<PointSequence>> temporary; // Follows PipelineState::cameras mapping

	inline unsigned int getSampleCount();

	void verifyCameraCount(int camCnt)
	{
		if (temporary.size() < camCnt)
			temporary.resize(camCnt);
	}
	
	void clear()
	{
		lastRecordedFrame = 0;
		markers.clear();
		for (int i = 0; i < temporary.size(); i++)
			temporary[i].clear();
	}

	void init(int camCnt)
	{
		clear();
		verifyCameraCount(camCnt);
	}
};


#endif // SEQUENCE_DATA_H