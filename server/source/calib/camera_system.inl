/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef CAMERA_SYSTEM_INL
#define CAMERA_SYSTEM_INL

#include "calib/camera_system.hpp"

#include <cassert>


/* Function Declarations */

inline float FundamentalMatrix::updateTrust(int frame, float decayRate)
{
	// TODO: Account for number of rejected correspondences in FM trust
	// Also tune decayRate better once simulation gives more control over changing FMs, missing observations, etc.
	int updateFrames = frame-std::max(lastFrame, lastTrustUpdate);
	int framesPassed = frame-lastFrame;
	if (framesPassed < 10000)
		floatingTrust -= (long)framesPassed * updateFrames * decayRate;
	//floatingTrust *= 1.0f-decayRateAlt;
	floatingTrust = std::max(0.0f, floatingTrust);
	lastTrustUpdate = frame;
	return floatingTrust;
}

inline const EpipolarCameras::FMEntry &EpipolarCameras::getFMEntry(int camA, int camB) const
{
	assert(camA > camB);
	assert(FMStore.size() > camA);
	assert(FMStore[camA].size() > camB);
	return FMStore[camA][camB];
}

inline EpipolarCameras::FMEntry &EpipolarCameras::getFMEntry(int camA, int camB)
{
	assert(camA > camB);
	assert(FMStore.size() > camA);
	assert(FMStore[camA].size() > camB);
	return FMStore[camA][camB];
}

inline const bool EpipolarCameras::getFundamentalMatrix(int camA, int camB, FundamentalMatrix &fundamentalMatrix) const
{
	bool flipped = camA < camB;
	if (flipped) std::swap(camA, camB);
	if (FMStore.size() <= camA || FMStore[camA].size() <= camB) return false;
	const auto &entry = getFMEntry(camA, camB);
	if (entry.candidates.empty()) return false;
	fundamentalMatrix = entry.getBestCandidate();
	if (flipped) fundamentalMatrix.matrix.transposeInPlace();
	return true;
}

inline void EpipolarCameras::setFundamentalMatrix(int camA, int camB, FundamentalMatrix fundamentalMatrix)
{
	if (camA < camB)
	{
		fundamentalMatrix.matrix.transposeInPlace();
		std::swap(camA, camB);
	}
	auto &entry = getFMEntry(camA, camB);
	entry.candidates = { fundamentalMatrix };
}

inline int EpipolarCameras::FMEntry::getBestCandidateIndex() const
{
	assert(!candidates.empty());
	int best = 0;
	for (int i = 1; i < candidates.size(); i++)
	{
		if (candidates[i].floatingTrust > candidates[best].floatingTrust)
			best = i;
	}
	return best;
}

inline const FundamentalMatrix &EpipolarCameras::FMEntry::getBestCandidate() const
{
	return candidates[getBestCandidateIndex()];
}

inline FundamentalMatrix &EpipolarCameras::FMEntry::getBestCandidate()
{
	return candidates[getBestCandidateIndex()];
}

#endif // CAMERA_SYSTEM_INL