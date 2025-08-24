/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef SEQUENCE_PARAMETERS_H
#define SEQUENCE_PARAMETERS_H

#define INCLUDE_MATCHING_PARAMS_ONLY
#include "util/matching.hpp"

#include "util/eigendef.hpp"

#include <array>

/**
 * Parameters for sequence 2D
 */

struct SequenceAquisitionParameters
{
	std::string label;

	// Used for matching points to sequences each frame:
	float maxAcceleration; // Max change from predicted position in one frame. Set to 1cm at 1 meter distance for 90°hFoV camera.
	float maxValueDiff;
	float valueDiffFactor;
	int valueFlowingAvgerage;
	MatchingParameters match;

	// Limits for inactive sequences
	int minInactivityLength;
	float maxInactivityMovement; // Max change from inactive state. Set to 1cm at 1 meter distance for 90°hFoV camera.
	float maxSigmaInactivity; // Max deviation from standard deviation during inactivity

	// Limits for drops in obvervation
	int allowedDrops; // Can be due to missing frames / erroneous checksum / temporary conflicts
	int allowedDropsTemp;
	int allowedDropsTempInactive; // Don't drop inactive temporaries, easier to keep them locked as inactive

	// Limits based on sequence lengths
	int minSequenceLength;
	int sequenceCorrespondenceLength;

	struct {
		int ConfidentMinSamples; // Determines when a correspondence (FM) is considered confident
		float UpperError;
		float ConfidentMinTrust;
		float TrustDecayRate; // Applied per frame, per frame since last update
		float FitPointSigma;
		float FitSeqSigmaSelect;
		float FitSeqSigmaMax;
		float FitNewSigma;
	} FM;

	struct {
		float TrustBaseSupporting;
		float TrustBaseDiscrediting;
		// Limits for correspondence verification
		float minWeight;
		float maxDiscrediting;
		// Comparision between competing correspondences
		float minPrimAdvantage; // Discard correspondences that are not better than the secondary correspondence by a significant factor
		float errorUncertainty; // Uncertainty of each correspondence calculation
		// for confident FMs
		// see FM.FitSeq
		// for uncertain FMs
		// see FM.FitNew
		int minOverlap; // Overlap (samples) for one correspondence check between cameras
		float maxError; // Has to deal with uncorrected distorted sequences
		float minConfidence;
	} correspondences;
};

// Delay for functions requiring a stable observation sequence state
// Past this many frames in the past sequence2D will not change marker associations
const static int stableSequenceDelay = 100;

struct SequenceParameters
{
	const SequenceAquisitionParameters sequenceUncalibrated = {
		"Uncalibrated",
		20*PixelSize,
		200,
		0.1f/(1000*PixelFactor),
		3,
		{
			1.0f*PixelSize,
			2,
			3,
			1,
			10
		},
		100,
		3*PixelSize,
		2.0f,
		5,
		5,
		5,
		stableSequenceDelay,
		stableSequenceDelay,
		{
			1000,
			0.6f,
			10000.0f,
			0.00001f,
			3,
			0,
			3,
			4,
		},
		{
			100,
			1000,
			100,
			0.3f,
			2,
			0.00005f,
			stableSequenceDelay,
			0.05f,
			5,
		},
	};

	const SequenceAquisitionParameters sequenceCalibrated = {
		"Calibrated",
		10*PixelSize,
		200,
		0.1f/(1000*PixelFactor),
		3,	
		{
			0.5f*PixelSize,
			2,
			2,
			1,
			10
		},
		100,
		3*PixelSize,
		2.0f,
		2,
		2,
		5,
		50,
		50,
		{
			1000,
			0.2f,
			5000.0f,
			0.0f,
			3,
			1,
			1,
			4,
		},
		{
			5000,
			5000,
			100,
			0.1f,
			2,
			0.00005f,
			stableSequenceDelay,
			0.01f,
			10,
		},
	};

	const std::array<SequenceAquisitionParameters, 2> standard = { sequenceUncalibrated, sequenceCalibrated };
	std::vector<SequenceAquisitionParameters> sequence = { sequenceUncalibrated, sequenceCalibrated };
	int selected = -1;

	SequenceAquisitionParameters &get(int pref = 0)
	{
		return sequence[selected >= 0? selected : pref];
	}

	const SequenceAquisitionParameters &get(int pref = 0) const
	{
		return sequence[selected >= 0? selected : pref];
	}
};

#endif // SEQUENCE_PARAMETERS_H