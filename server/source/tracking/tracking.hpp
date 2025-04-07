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
	long lastIMUSample;
	TimePoint_t lastIMUTime;
	long lastObsFrame;
	TimePoint_t lastObservation;

	TrackerState(Eigen::Isometry3f pose, TimePoint_t time, const TargetTrackingParameters &params) :
		state{}, time(time), lastIMUSample(-1), lastIMUTime(time), lastObsFrame(-1), lastObservation(time) 
	{
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
	const TargetCalibration3D * calib;

	// Store internal data for visualisation purposes (low overhead)
	TargetTracking2DData data;

	// 2D Tracking result
	TargetMatch2D match2D;

	TrackerTarget(const TargetCalibration3D * target) :
		calib(target), data{}, match2D{} {}
};


struct TrackerMarker
{
	// For matching of markers, either single marker size or target radius
	float size;

	TrackerMarker(float size) : size(size) {}
};

struct TrackerInertial
{
	std::shared_ptr<IMU> imu;
	struct {
		Eigen::Quaterniond quat;
		Eigen::Vector3d accel;
	} fusion;

	TrackerInertial() : imu() {}
	TrackerInertial(std::shared_ptr<IMU> &imu) : imu(imu) {}
	TrackerInertial(std::shared_ptr<IMU> &&imu) : imu(std::move(imu)) {}

	operator bool() const { return imu != nullptr; }
};

struct TrackerObservation
{
	TimePoint_t time;
	Eigen::Isometry3f predicted; // Predicted using extrapolation and/or IMU integration
	Eigen::Isometry3f extrapolated; // Extrapolated using just the model from last observation
	Eigen::Isometry3f inertial; // Pose as integrated from new IMU samples
	Eigen::Isometry3f observed; // Pose as observed by the cameras
	Eigen::Isometry3f filtered; // Pose filtered from all observations
	Eigen::Matrix<float,6,6> covPredicted, covFiltered, covObserved;

	TrackerObservation(Eigen::Isometry3f pose, TimePoint_t time, const TargetTrackingParameters &params) :
		predicted(pose), extrapolated(pose), inertial(pose), observed(pose), filtered(pose), time(time)
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

	// Optional inertial tracking source
	TrackerInertial inertial;

	// Target tracking source
	TrackerTarget target;

	// Latest observation
	TrackerObservation pose;

	TrackedTarget(const TargetCalibration3D *target, Eigen::Isometry3f pose,
		TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params) :
		target(target), state(pose, time, params), pose(pose, time, params)
	{
		state.lastObservation = time;
		state.lastObsFrame = frame;
	}
};

struct TrackedMarker
{
	TrackerState state;

	// Optional inertial tracking source
	TrackerInertial inertial;

	// Single Marker tracking source
	TrackerMarker marker;

	// Latest observation
	TrackerObservation pose;

	TrackedMarker(Eigen::Vector3f pos, float size, TimePoint_t time, const TargetTrackingParameters &params) :
		state(Eigen::Isometry3f(Eigen::Translation3f(pos)), time, params),
		inertial(), marker(size),
		pose(Eigen::Isometry3f(Eigen::Translation3f(pos)), time, params) {}
};

struct DormantTarget
{
	// Optional inertial tracking source
	TrackerInertial inertial;

	// Target tracking source
	TrackerTarget target;

	DormantTarget(const TargetCalibration3D *target) :
		inertial(), target(target) {}

	DormantTarget(TrackedTarget &&tracker) :
		inertial(std::move(tracker.inertial)), target(std::move(tracker.target)) {}
};

const static Eigen::Isometry3f orphanedIMUPose = Eigen::Isometry3f(Eigen::Translation3f(0, 0, 1));

struct OrphanedIMU
{
	TrackerState state;

	// Inertial tracking source
	TrackerInertial inertial;

	// Latest observation
	TrackerObservation pose;

	OrphanedIMU(TrackerInertial &imu, const TargetTrackingParameters &params) :
		state(orphanedIMUPose, sclock::now(), params),
		inertial(imu),
		pose(orphanedIMUPose, sclock::now(), params) {}
	
	OrphanedIMU(TrackerInertial &&imu, const TargetTrackingParameters &params) :
		state(orphanedIMUPose, sclock::now(), params),
		inertial(std::move(imu)),
		pose(orphanedIMUPose, sclock::now(), params) {}
};


/* Functions */

bool simulateTrackTarget(TrackerState &state, TrackerObservation &observation,
	Eigen::Isometry3f simulatedPose, CovarianceMatrix covariance, bool success,
	TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params);

bool trackTarget(TrackerState &state, TrackerTarget &target, TrackerObservation &observation,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, unsigned int frame, int cameraCount, const TargetTrackingParameters &params);

int trackMarker(TrackerState &state, TrackerMarker &marker, TrackerObservation &observation,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices,
	TimePoint_t time, float sigma);

bool integrateIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params);

#endif // TRACKING_3D_H