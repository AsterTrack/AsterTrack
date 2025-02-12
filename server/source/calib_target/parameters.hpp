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

#ifndef CALIBRATION_TARGET_PARAMETERS_H
#define CALIBRATION_TARGET_PARAMETERS_H

#define INCLUDE_MATCHING_PARAMS_ONLY
#include "util/matching.hpp"

#include "target/parameters.hpp"

/**
 * Parameters for target calibration
 */


struct TargetAquisitionParameters
{
	int minLength = 80;
	int minFrameObsCount = 8;
	int minMarkerObsCount = 15;
	int minSharedMarkerCount = 4;
	float ratioHardLimit = 1.2f;
};

struct TargetOutlierErrors
{
	float trigger = 4;
	float sample = 4;
	float frame = 2;
	float marker = 2;
};

struct TargetViewParameters
{
	float initialOptTolerance = 1.0f;
	int initialOptLimit = 40;
	float manualOptTolerance = 0.01f;
	int manualOptLimitIncrease = 40;
	TargetOutlierErrors outlierSigmas = {};
};

struct ReevaluateSequenceParameters
{
	float uncertainty = 0.5f;
	float maxSeqRMSE = 10.0f;
	float sigmaAddSeq = 3;
	float sigmaMerge = 3;
	int minOverlap = 10;
	MatchingParameters match = { 1, 3, 3, 2, 1 };
};

struct SubsampleTargetParameters
{
	int targetSampleCount = 5000;
	int maxSelectBatch = 10;
	float percentile = 0.75f;
	float sampleFactorPower = 2.0f;
	float randomStdDev = 0.1f;
	int resampleInterval = 0; // 5 is fine, but its disabled rn
	// Resampling is only half implemented, e.g. it discards outliers
};

struct TrackFrameParameters
{
	float uncertainty = 0.5f;
	int minTrackPts = 15;
	float sigmaFill = 3;
	float sigmaAdd = 1.5f;
	TargetTrackingParameters track = {};
};

struct TargetAssemblyParameters
{
	float alignPointError = 1;	// In mm
	float alignPointSigma = 3;
	float alignPoseSigma = 0.2;
	float alignMaxRMSE = 10;	// In mm
	int alignMinPoints = 8;

	float mergePointLimit = 10; // In mm
	int mergeMinPoints = 5;
	int mergeMinFrameObs = 15;
	float viewThresh = 1.5f;
	float viewMax = 5.0f;
	MatchingParameters match = { 1.0f/1000, 3, 3, 2, 1 };

	float optThresh = 1.2f;
	float optTolerance = 2.0f;
	int optMaxIt = 15;
	TargetOutlierErrors outlierSigmas = {};

	ReevaluateSequenceParameters reevaluation = {};
	SubsampleTargetParameters subsampling = {};
	TrackFrameParameters trackFrame = {};
};

struct TargetPostProcessingParameters
{
	float assumedSize = 0.005f;
	float assumedAngle = NAN;
	struct
	{
		float extremityCenteringPower = 5;
		// Mixing limits:
		std::array<float,3> ownSampleExtremes = { 0.1f, 0.2f, 0.7f };
			// Own limit of marker from 3 most extreme samples
		std::array<float,3> topMarkerExtremes = { 0.2f, 0.3f, 0.5f };
			// Top limit for all markers from top 3 markers with widest FoV
		std::array<float,3> markerLimitMix = { 0.0f, 0.5f, 0.5f };
			// Resulting limit of marker from own, top, and conservative limit accounting for the difference
	} viewCone;
};

struct TargetCalibParameters
{
	TargetAquisitionParameters aquisition;
	TargetViewParameters view;
	TargetAssemblyParameters assembly;
	TargetPostProcessingParameters post;
};

#endif // CALIBRATION_TARGET_PARAMETERS_H