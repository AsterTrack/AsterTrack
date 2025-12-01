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

#ifndef CALIBRATION_TARGET_ASSEMBLY_H
#define CALIBRATION_TARGET_ASSEMBLY_H

#include "parameters.hpp"
#include "aquisition.hpp" // BlockStats

#include "calib/optimisation.hpp" // OptErrorRes

#include "pipeline/record.hpp"

#include "target/target.hpp" // TargetCalibration3D
#include "target/detection3D.hpp" // TargetCandidate3D, only for debug

#include "util/synchronised.hpp"
#include "util/threading.hpp"

#include <vector>


/**
 * Calibration of target structure
 */

struct TargetView
{
	int id;
	// Range
	uint32_t beginFrame = std::numeric_limits<uint32_t>::max(), endFrame = 0;
	bool selected; // For final calibration
	bool deleted; // E.g. due to insufficient data
	BlockStats stats = {};
	// Data
	SynchronisedS<ObsTarget> target = {};
	TargetCalibration3D targetCalib = {};
	// TODO: Synchronise together with target for use in VisTargetLock. Currently unused
	// Calibration
	bool planned;
	ThreadControl control;
	enum CalibrationStep { NONE,
		OPTIMISE_FINE,
		OPTIMISE_COARSE,
		RECONSTRUCT,
		TEST_REEVALUATE_MARKERS,
		REEVALUATE_MARKERS,
		EXPAND_FRAMES
	};
	std::vector<CalibrationStep> plan;
	struct
	{
		bool calibrated = false;
		CalibrationStep step;
		int numSteps = 0, maxSteps = 0;
		OptErrorRes errors = {};
		int lastStopCode;
		bool complete = false;
	} state;
	// Stepping through calibration
	bool debugIterations = false;
	struct Iteration
	{
		OptErrorRes error;
		ObsTarget target;
		Iteration(OptErrorRes e, ObsTarget &t) : error(e), target(t) {}
	};
	std::vector<Iteration> iterationStates;

	struct
	{
		const TargetCalibration3D *targetGT = nullptr;
	} simulation;
};

struct TargetAssemblyBase
{
	int initialViewID;

	// Views merged into base
	std::shared_ptr<TargetView> merging;
	std::vector<int> merged;
	std::map<int, std::pair<int, float>> alignmentStats;

	// Resulting assembled target
	ObsTarget target; // Holds observation samples for further optimisation
	OptErrorRes errors;
	TargetCalibration3D targetCalib;

	struct
	{
		const TargetCalibration3D *targetGT = nullptr;
	} simulation;
};

struct TargetMergeCandidate
{
	int id;
	bool merged;
	std::map<int,int> pointMap;
	std::vector<Eigen::Vector3f> markers;
	OptErrorRes error;
};

struct TargetAlignResults
{
	int id;
	bool success;
	int bestCandidate = -1;
	std::vector<TargetCandidate3D> candidates;
	std::vector<Eigen::Vector3f> markers;
	float RMSE;
};

enum TargetAssemblyStageID
{
	STAGE_LOADED,
	STAGE_EDITED,
	STAGE_OPTIMISATION,
	STAGE_REEVALUATE_MARKERS,
	STAGE_EXPAND_FRAMES,
	STAGE_INT_ALIGN,
	STAGE_INT_VIEW,
	STAGE_INT_FRAMES,
	STAGE_INT_MARKERS
};

struct TargetAssemblyStage
{
	std::string label;
	TargetAssemblyStageID stage;
	int step;
	TargetAssemblyBase base;
	std::vector<TargetAlignResults> alignResults;
	std::vector<TargetMergeCandidate> mergeTests;

	TargetAssemblyStage(TargetAssemblyBase &Base, TargetAssemblyStageID Stage, int Step, std::string &&Label)
		: base(Base), stage(Stage), step(Step), label(std::move(Label)) {}
	TargetAssemblyStage(TargetAssemblyBase &&Base, TargetAssemblyStageID Stage, int Step, std::string &&Label)
		: base(std::move(Base)), stage(Stage), step(Step), label(std::move(Label)) {}
};

static TargetOutlierErrors SigmaToErrors(const TargetOutlierErrors sigmas, const OptErrorRes errors)
{
	TargetOutlierErrors maxErrors;
	maxErrors.trigger = errors.mean + sigmas.trigger * errors.stdDev;
	maxErrors.sample = errors.mean + sigmas.sample * errors.stdDev;
	maxErrors.frame = errors.mean + sigmas.frame * errors.stdDev;
	maxErrors.marker = errors.mean + sigmas.marker * errors.stdDev;
	return maxErrors;
}

// Merge pairwise close markers
int mergeCloseMarkers(std::vector<Eigen::Vector3f> &markers, std::map<int,int> &markerMap, std::vector<int> *markerSamples = nullptr, float mergeMM = 2.0f, int maxMerges = 0);

// Check per-frame and per-marker errors of current Target View Calibration
bool determineTargetOutliers(const std::vector<CameraCalib> &calibs, ObsTarget &target, TargetOutlierErrors maxErrors, const TargetAquisitionParameters &params);

template<bool APPLY = true>
int reevaluateMarkerSequences(const std::vector<CameraCalib> &calibs, const std::vector<MarkerSequences> &observations, ObsTarget &target,
	ReevaluateSequenceParameters params);

ObsTarget subsampleTargetObservations(const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, const ObsTarget &target, SubsampleTargetParameters params);

void expandFrameObservations(const std::vector<CameraCalib> &calibs, const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords,
	ObsTarget &target, const TargetCalibration3D &trkTarget, TrackFrameParameters params);

template<bool APPLY = true>
OptErrorRes reevaluateFrameObservations(const std::vector<CameraCalib> &calibs, const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, ObsTarget &target, const TargetCalibration3D &trkTarget, TrackFrameParameters params);

void verifyTargetObservations(const std::vector<CameraCalib> &calibs, const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, const ObsTarget &target, const TargetCalibration3D &trkTarget);

Eigen::Vector3f analyzeMarkerOrientation(std::vector<Eigen::Vector3f> &markerRays, const TargetPostProcessingParameters &params);

void updateMarkerOrientations(std::vector<TargetMarker> &markers, const std::vector<CameraCalib> &calibs, const ObsTarget &target, const TargetPostProcessingParameters &params);

std::vector<TargetMarker> finaliseTargetMarkers(const std::vector<CameraCalib> &calibs, const ObsTarget &target, const TargetPostProcessingParameters &params);

#endif // CALIBRATION_TARGET_ASSEMBLY_H