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
	float accT = 0.2f, accR = 0.005f;
	float centerForce = 0.01f, centerAttenuation = 0.01f;
	float dampT = 0.05f, dampR = 0.1f;
	float slowT = 0.01f, slowR = 0.05f;
	float minAcc = 0.05f;
};

const int MotionCustom = 3;
const extern std::array<MotionParameters, MotionCustom> motionPresets;

struct SimulatedObject
{
	bool enabled, logPose;
	int id;
	std::string label;
	TargetCalibration3D target;
	int motionPreset;
	MotionParameters motion;
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

struct SimulationState
{
	BlockedVector<Eigen::Isometry3f> framePoses;
	struct FrameInfo
	{
		unsigned int frame;
		Eigen::Isometry3f pose;
		std::vector<std::pair<int, Eigen::Vector3f>> triangulation;
	};
	// Generating custom pose
	FrameInfo triangulatedPoints3D;
	float blobPxStdDev = 0;

	std::vector<SimulatedObject> objects;
	int primaryObject;

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
			SimulatedObject { .id = 0, .label = "Single Marker", .target = PointCalibMarker, .motionPreset = 0, .motion = motionPresets[0] }
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

#endif // SIMULATION_H