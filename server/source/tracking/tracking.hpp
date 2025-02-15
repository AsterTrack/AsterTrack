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

#ifndef TRACKING_3D_H
#define TRACKING_3D_H

#include "target/target.hpp"
#include "target/tracking2D.hpp"
#include "imu/imu.hpp"

#include "util/util.hpp" // TimePoint_t
#include "util/eigendef.hpp"

#include "flexkalman/process/PoseSeparatelyDampedConstantVelocity.h"
#include "flexkalman/state/PoseState.h"

/**
 * Tracking a target (set of markers) in 3D space
 */

/* Tracker Components */

struct TrackerState
{
	using State = flexkalman::pose_externalized_rotation::State;
	using Model = flexkalman::PoseSeparatelyDampedConstantVelocityProcessModel<State>;

	// Current state
	State state;
	TimePoint_t time;

	// Information about state
	long lastIMUSample = -1;
	long lastObsFrame = -1;
	TimePoint_t lastObservation;

	TrackerState() {}
	TrackerState(Eigen::Isometry3f pose, TimePoint_t time, const TargetTrackingParameters &params)
		: time(time)
	{
		state = {};
		state.position() = pose.translation().cast<double>();
		state.setQuaternion(Eigen::Quaterniond(pose.rotation().cast<double>()));
		auto &errorCov = state.errorCovariance();
		errorCov.diagonal().segment<3>(0).setConstant(params.filter.stdDevPos*params.filter.sigmaInitState);
		errorCov.diagonal().segment<3>(3).setConstant(params.filter.stdDevEXP*params.filter.sigmaInitState);
		errorCov.diagonal().segment<3>(6).setConstant(params.filter.stdDevPos*params.filter.sigmaInitChange);
		errorCov.diagonal().segment<3>(9).setConstant(params.filter.stdDevEXP*params.filter.sigmaInitChange);
	}
};

struct TrackerTarget
{
	// Calibrated target template
	TargetCalibration3D const * calib;

	// Store internal data for visualisation purposes (low overhead)
	TargetTracking2DData data;

	// 2D Tracking result
	TargetMatch2D match2D;

	TrackerTarget() {}
	TrackerTarget(TargetCalibration3D const * target) : calib(target) {}
};


struct TrackerMarker
{
	// For matching of markers, either single marker size or target radius
	float size;

	TrackerMarker() {}
	TrackerMarker(float size) : size(size) {}
};

typedef std::shared_ptr<IMU> TrackerIMU;

struct TrackerObservation
{
	TimePoint_t time;
	Eigen::Isometry3f predicted;
	Eigen::Isometry3f observed; // Pose as observed by the cameras
	Eigen::Isometry3f filtered; // Pose filtered from all observations
	Eigen::Matrix<float,6,6> covPredicted, covFiltered, covObserved;

	TrackerObservation() {}
	TrackerObservation(Eigen::Isometry3f pose, TimePoint_t time, const TargetTrackingParameters &params)
		: predicted(pose), observed(pose), filtered(pose), time(time)
	{
		covObserved.diagonal().segment<3>(0).setConstant(params.filter.stdDevPos*params.filter.sigmaInitState);
		covObserved.diagonal().segment<3>(3).setConstant(params.filter.stdDevEXP*params.filter.sigmaInitState);
		covPredicted = covFiltered = covObserved;
	}
};

/* Tracked Object Representations */

struct TrackedTarget
{
	TrackerState state;

	// Optional IMU tracking source
	TrackerIMU imu;

	// Target tracking source
	TrackerTarget target;

	// Latest observation
	TrackerObservation pose;

	TrackedTarget() {}
	TrackedTarget(TargetCalibration3D const *target, Eigen::Isometry3f pose, TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params)
		: target(target), state(pose, time, params), pose(pose, time, params)
	{
		state.lastObsFrame = frame;
	}
};

struct TrackedMarker
{
	TrackerState state;

	// Optional IMU tracking source
	TrackerIMU imu;

	// Single Marker tracking source
	TrackerMarker marker;

	// Latest observation
	TrackerObservation pose;

	TrackedMarker(Eigen::Vector3f pos, float size, TimePoint_t time, const TargetTrackingParameters &params)
		: state(Eigen::Isometry3f(Eigen::Translation3f(pos)), time, params),
		  pose(Eigen::Isometry3f(Eigen::Translation3f(pos)), time, params),
		  marker(size) {}
};

struct DormantTarget
{
	// Optional IMU tracking source
	TrackerIMU imu;

	// Target tracking source
	TrackerTarget target;

	DormantTarget() {}
	DormantTarget(TargetCalibration3D const *target)
		: target(target) {}
};

const static Eigen::Isometry3f orphanedIMUPose = Eigen::Isometry3f(Eigen::Translation3f(0, 0, 1));

struct OrphanedIMU
{
	TrackerState state;

	// IMU tracking source
	TrackerIMU imu;

	// Latest observation
	TrackerObservation pose;

	OrphanedIMU(TrackerIMU &imu, const TargetTrackingParameters &params)
		: state(orphanedIMUPose, TimePoint_t::min(), params),
		  pose(orphanedIMUPose, TimePoint_t::min(), params),
		  imu(imu) {}
	
	OrphanedIMU(TrackerIMU &&imu, const TargetTrackingParameters &params)
		: state(orphanedIMUPose, TimePoint_t::min(), params),
		pose(orphanedIMUPose, TimePoint_t::min(), params),
		imu(std::move(imu)) {}
};


/* Functions */

bool trackTarget(TrackerState &state, TrackerTarget &target, TrackerObservation &observation,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, unsigned int frame, int cameraCount, const TargetTrackingParameters &params);

int trackMarker(TrackerState &state, TrackerMarker &marker, TrackerObservation &observation,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices,
	TimePoint_t time, float sigma);

bool integrateIMU(TrackerState &state, const TrackerIMU &imu, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params);

#endif // TRACKING_3D_H