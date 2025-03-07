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

#include "util/util.hpp" // TimePoint_t
#include "util/eigendef.hpp"

#include "flexkalman/process/PoseSeparatelyDampedConstantVelocity.h"
#include "flexkalman/state/PoseState.h"

#include <vector>

/**
 * Tracking a target (set of markers) in 3D space
 */


/* Structures */

// UKF-based Filter from monado based on flexkalman
struct FlexUKFFilter
{
	using Scalar = float;
	using State = flexkalman::pose_externalized_rotation::State;
	using Model = flexkalman::PoseSeparatelyDampedConstantVelocityProcessModel<State>;

	// State
	int measurements = 0; // Optical measurements

	// Filter
	TimePoint_t time;
	State state;
	Model model;

	// Prediction
	Isometry3<Scalar> posePredicted;
	Eigen::Matrix<Scalar,3,1> stdDev;

	// Results
	Isometry3<Scalar> poseObserved; // Pose as observed by the cameras
	Isometry3<Scalar> poseFiltered; // Pose filtered from all observations


	void init(Isometry3<Scalar> pose, const TargetTrackingParameters &params);
};

template<typename FilterImpl>
struct TrackedTarget
{
	using Filter = FilterImpl;
	using Scalar = typename FilterImpl::Scalar;

	// Filter providing filtering and smoothing
	FilterImpl filter;

	// Calibrated target template
	TargetTemplate3D const * target;

	// Store internal data for visualisation purposes (low overhead)
	TargetTracking2DData tracking2DData;
	// TODO: Keep internal data for debug when target looses tracking

	// Tracking results for visualisation purposes
	TargetMatch2D match2D;

	int lastTrackedFrame;

	TrackedTarget() {}
	TrackedTarget(TargetTemplate3D const *target, Eigen::Isometry3f pose, TimePoint_t time, const TargetTrackingParameters &params) : target(target)
	{
		filter.init(pose.cast<Scalar>(), params);
		filter.time = time;
	}

	Isometry3<Scalar> getPoseObserved() const { return filter.poseObserved; }
	Isometry3<Scalar> getPoseFiltered() const { return filter.poseFiltered; }
	Isometry3<Scalar> getPosePredicted() const { return filter.posePredicted; }
	Vector3<Scalar> getPredictionStdDev() const { return filter.stdDev; }
	int getMeasurements() const { return filter.measurements; }
};

typedef TrackedTarget<FlexUKFFilter> TrackedTargetFiltered;

/* Functions */

template<typename TARGET = TrackedTargetFiltered>
bool trackTarget(TARGET &target, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, int cameraCount, const TargetTrackingParameters &params);

#endif // TRACKING_3D_H