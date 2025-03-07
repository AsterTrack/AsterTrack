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

#ifndef POINT_TRACKING_3D_H
#define POINT_TRACKING_3D_H

#include "pipeline/parameters.hpp"
#include "point/triangulation.hpp"

#include "util/util.hpp" // TimePoint_t
#include "util/eigendef.hpp"

#include "flexkalman/process/PoseSeparatelyDampedConstantVelocity.h"
#include "flexkalman/state/PoseState.h"

#include <vector>

/**
 * Tracking a single point (marker) in 3D space
 * Treated as having a 6-DOF pose to allow for connecting with IMU later on for rotation
 */


/* Structures */


template<typename Scalar>
struct TrackedMarker
{
	using ScalarType = Scalar;
	using State = flexkalman::pose_externalized_rotation::State;
	using Model = flexkalman::PoseSeparatelyDampedConstantVelocityProcessModel<State>;

	// State
	int measurements = 0;
	Scalar size; // For matching of markers, either single marker size or target radius

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
};


/* Functions */

template<typename TrackedMarker = TrackedMarker<float>>
int trackMarker(TrackedMarker &marker,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices,
	TimePoint_t time, float sigma);

#endif // POINT_TRACKING_3D_H