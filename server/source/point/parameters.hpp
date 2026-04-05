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
	// For editing purposes only
	std::string label;
	int ref;

	// Used for matching points to sequences each frame:
	float maxAcceleration; // Max change from predicted position in one frame. Set to 1cm at 1 meter distance for 90°hFoV camera.
	float maxValueDiff;
	float valueDiffFactor;
	int valueFlowingAverage;
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
		// Merge markers if correspondence allows for both
		// While it allows for error correction for initial calibration
		// it may be less comservative than proper correspondence testing and thus bad for target calibration
		bool allowMarkerMerging;
	} correspondences;
};

// Delay for functions requiring a stable observation sequence state
// Past this many frames in the past sequence2D will not change marker associations
const static int stableSequenceDelay = 100;

extern const SequenceAquisitionParameters sequenceUncalibrated;
extern const SequenceAquisitionParameters sequenceCalibrated;

struct SequenceParameters
{
	const std::array<const SequenceAquisitionParameters*, 2> standard = { &sequenceUncalibrated, &sequenceCalibrated };
	std::vector<SequenceAquisitionParameters> sequence;
	int selected = -1;

	const SequenceAquisitionParameters &get(int pref = 0) const
	{
		int index = selected >= 0? selected : pref;
		return pref < sequence.size()? sequence[index] : *standard[index];
	}
};

#endif // SEQUENCE_PARAMETERS_H