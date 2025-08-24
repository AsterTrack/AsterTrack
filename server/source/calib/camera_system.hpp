/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef CAMERA_SYSTEM_H
#define CAMERA_SYSTEM_H

#include "util/eigendef.hpp"

#include "util/stats.hpp"


/* Structures */

/**
 * Fundamental Matrix between to cameras and its accompanying statistics and ratings
 */
struct FundamentalMatrix
{
	// Range of frames used for calculation where the FM is deemed to be valid
	int firstFrame, lastFrame, lastCalculation; // Before and after, FMs may have changed

	// Affects whether sequence2D should itself recalculate FM
	// Even if true, errors will still be accumulated
	bool precalculated;

	// Ongoing stats of verified correspondences using this FM
	// Initialised by the last full recalculation
	// Updated when sequences end (?)
	StatDistf stats;

	// A trust value that decays over time if not updated
	// Update means a correspondence uses and thus confirms this FM is still valid
	int lastTrustUpdate; // Frame of last update of trust value
	float floatingTrust;

	inline float updateTrust(int frame, float decayRate);

	Eigen::Matrix3f matrix;
};

/**
 * A store for epipolar relations between pairs of camera in the form of rated fundamental matrix candidates
 */
struct EpipolarCameras
{
	struct FMEntry
	{
		std::vector<FundamentalMatrix> candidates;

		inline int getBestCandidateIndex() const;
		inline const FundamentalMatrix &getBestCandidate() const;
		inline FundamentalMatrix &getBestCandidate();
	};

	std::vector<std::vector<FMEntry>> FMStore;

	inline const EpipolarCameras::FMEntry &getFMEntry(int camA, int camB) const;
	inline EpipolarCameras::FMEntry &getFMEntry(int camA, int camB);
	inline const bool getFundamentalMatrix(int camA, int camB, FundamentalMatrix &fundamentalMatrix) const;
	inline void setFundamentalMatrix(int camA, int camB, FundamentalMatrix fundamentalMatrix);
};

/**
 * Management of calibrations for a camera system
 */
struct CameraSystemCalibration
{
	EpipolarCameras relations;

	// TODO: Store CameraCalibrations here, too, in line with pipeline camera indices
	// Then use this to work on, until finally AdoptNewCalibrations will copy it into cameras

	// TODO: Handle calibrations better
	// -> Keep running error
	// -> Handle trust (and loosing trust) in Fundamental Matrices of Sequence2D
	// -> Notify calibration (and potentially user) when most of cameras' FMs loose trust

	// Currently, relations have a trust metric, useful for establishing new Fundamental Matrices between cameras
	// But it's not very useful for detecting changes in calibration that would require a (partial) recalibration
	// Should also tie in with continuous calibration

	void verifyCameraCount(int camCnt)
	{
		while (relations.FMStore.size() <= camCnt)
		{ // Make sure camera exists
			relations.FMStore.push_back({});
			relations.FMStore.back().resize(relations.FMStore.size()-1);
		}
	}

	void init(int camCnt)
	{
		relations = {};
		verifyCameraCount(camCnt);
	}
};


#endif // CAMERA_SYSTEM_H