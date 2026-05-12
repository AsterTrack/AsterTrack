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

#ifndef SIMULATION_H
#define SIMULATION_H

#include "target/target.hpp"
#include "pipeline/record.hpp"

#include "util/trackdef.hpp"
#include "util/eigendef.hpp"
#include "util/blocked_vector.hpp"

#include <queue>
#include <map>
#include <memory>

/*
 * Generate simulated data
 */

// Forward-declared opaque structs
struct PipelineState; // pipeline/pipeline.hpp
struct FrameRecord; // pipeline/pipeline.hpp

extern TargetCalibration3D PointCalibMarker;


/* Structures */

struct MotionParameters
{
	std::string label;
	float accT = 0.2f, accR = 0.005f;
	float centerForce = 0.01f, centerAttenuation = 0.01f;
	float dampT = 0.05f, dampR = 0.1f;
	float slowT = 0.01f, slowR = 0.05f;
	float minAcc = 0.05f;
};

extern std::vector<MotionParameters> motionPresets;

struct SimulatedObject
{
	bool enabled, logPose;
	int id;
	std::string label;
	TargetCalibration3D target;
	int motionPreset;
	struct {
		Eigen::Vector3f TGT = Eigen::Vector3f(0,0,1);
		Eigen::Matrix3f RGT = Eigen::Matrix3f::Identity();
		Eigen::Vector3f TD = Eigen::Vector3f::Zero();
		Eigen::Vector3f RD = Eigen::Vector3f::Zero();
		Eigen::Vector3f TA = Eigen::Vector3f::Zero();
		Eigen::Vector3f RA = Eigen::Vector3f::Zero();
	} internalMotionState = {};
	Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
};

struct ReplacedObject
{
	int srcID;
	TargetCalibration3D srcCalib;
	int tgtID;
	TargetCalibration3D tgtCalib;
	std::string tgtLabel;
};

struct ReplaceParameters
{
	bool suspendReplacing = false;
	float matchDist = 1.0f * PixelSize;
	float expandMarkerViewAngle = 0.1f;
	float occludeAngleTolerance = 0.1f;
	float occlusionDiscredit = 0.1f;
};

struct SimProjectionParameters
{
	float blobNoiseStdDev = 0.3f * PixelSize;
	float blobNoiseMaxSigma = 4.0f; 
	float expandMarkerViewAngle = -0.04f;
	float minSourceBlobSize = 0.0f * PixelSize;
	float blobVisualSizeFlare = 0.5f * PixelSize;
	float blobVisualSizeFactor = 2.0f;
	bool grazingAngleDiminishSize = true;
	float grazingAngleLower = -expandMarkerViewAngle-0.1f;
	float grazingAngleUpper = 0.8f;
	float mergeFactor = 0.5f;
};

struct SimulationState
{
	BlockedVector<Eigen::Isometry3f> framePoses;
	struct FrameInfo
	{
		FrameNum frame;
		Eigen::Isometry3f pose;
		std::vector<std::pair<int, Eigen::Vector3f>> triangulation;
	};
	// Generating custom pose
	FrameInfo triangulatedPoints3D;

	std::vector<SimulatedObject> objects;
	int primaryObject;

	std::map<int, ReplacedObject> replace;
	ReplaceParameters replaceParams;

	SimProjectionParameters projectionParams;

	inline const SimulatedObject &getPrimary() const
	{
		if (primaryObject < 0 || primaryObject >= objects.size() || !objects[primaryObject].enabled)
		{
			for (const auto &object : objects)
			{
				if (object.enabled)
					return object;
			}
			return objects[0];
		}
		return objects[primaryObject];
	}

	inline const SimulatedObject *getGT(int trackerID) const
	{
		for (const auto &object : objects)
		{
			if (object.id == trackerID)
				return &object;
		}
		return nullptr;
	}

	inline Eigen::Isometry3f getGTPose(int trackerID) const
	{ // Try to find ground truth pose of what it believes it is tracking (it may have detected wrong)
		Eigen::Isometry3f gtPose;
		auto gt = getGT(trackerID);
		if (gt) gtPose = gt->pose;
		return gtPose;
	}

	inline void resetState()
	{
		for (auto &object : objects)
		{
			object.enabled = false;
			object.internalMotionState = {};
		}
		if (objects.empty())
		{
			objects = {
			SimulatedObject { .id = 0, .label = "Single Marker", .target = PointCalibMarker, .motionPreset = 0 }
		 	};
		}
		primaryObject = 0;
		framePoses.clear();
	}
};


/* Functions */

/**
 * Generates simulated data according to config and current phase
 */
void GenerateSimulationData(PipelineState &pipeline, FrameRecord &frame);

/**
 * Replace point data in frameState belonging to the given tracker records with simulated data if configured
 */
void ReplaceTargetObservations(const PipelineState &pipeline, FrameRecord &frame, const std::vector<TrackerRecord> &trackers, bool keepUnmatchedObservations);

#endif // SIMULATION_H