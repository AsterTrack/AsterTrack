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

inline EpipolarCameras::FMEntry &EpipolarCameras::getFMEntry(int camA, int camB)
{
	assert(camA != camB);
	if (camA < camB) std::swap(camA, camB);
	while (FMStore.size() <= camA)
	{ // Make sure camera exists
		FMStore.push_back({});
		FMStore.back().resize(FMStore.size()-1);
	}
	return FMStore[camA][camB];
}

inline const EpipolarCameras::FMEntry *EpipolarCameras::getFMEntry(int camA, int camB) const
{
	assert(camA != camB);
	if (camA < camB) std::swap(camA, camB);
	if (FMStore.size() > camA)
		return &FMStore[camA][camB];
	return nullptr;
}

inline bool EpipolarCameras::getFundamentalMatrix(int camA, int camB, FundamentalMatrix &fundamentalMatrix)
{
	const auto &entry = getFMEntry(camA, camB);
	if (entry.candidates.empty())
		return false;
	fundamentalMatrix = entry.getBestCandidate();
	if (camA < camB) fundamentalMatrix.matrix.transposeInPlace();
	return true;
}

inline void EpipolarCameras::setFundamentalMatrix(int camA, int camB, FundamentalMatrix fundamentalMatrix)
{
	if (camA < camB) fundamentalMatrix.matrix.transposeInPlace();
	auto &entry = getFMEntry(camA, camB);
	entry.candidates = { fundamentalMatrix };
}

inline const FundamentalMatrix &EpipolarCameras::FMEntry::getBestCandidate() const
{
	assert(!candidates.empty());
	int best = 0;
	float trust = candidates.front().floatingTrust;
	for (int i = 1; i < candidates.size(); i++)
	{
		if (candidates[i].floatingTrust > trust)
		{
			best = i;
			trust = candidates[i].floatingTrust;
		}
	}
	return candidates[best];
}

inline FundamentalMatrix &EpipolarCameras::FMEntry::getBestCandidate()
{
	assert(!candidates.empty());
	int best = 0;
	float trust = candidates.front().floatingTrust;
	for (int i = 1; i < candidates.size(); i++)
	{
		if (candidates[i].floatingTrust > trust)
		{
			best = i;
			trust = candidates[i].floatingTrust;
		}
	}
	return candidates[best];
}

#endif // CAMERA_SYSTEM_INL