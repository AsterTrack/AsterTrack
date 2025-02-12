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

#include "util/eigendef.hpp"
#include "util/kalman.hpp"
#include "point/triangulation.hpp"

#include <vector>

/**
 * Tracking a single point (marker) in 3D space
 */


/* Structures */


template<typename Scalar>
struct TrackedMarker
{
	typedef Scalar ScalarType;

	// 3-DOF Kalman filter
	typedef State3DOF<Scalar> State;
	typedef SystemModel3DOF<Scalar> MovementModel;
	typedef MeasurementModel3DOF<Scalar> MeasurementModel;

	// Filter
	Kalman::ExtendedKalmanFilter<State> filter;
	MovementModel movementModel;
	MeasurementModel measurementModel;
	float errorScaleT;
	// State
	int measurements;
	State3DOF<Scalar> state;
	// Identification
	Scalar size; // For matching of markers, either single marker size or target radius
};


/* Functions */

int trackMarker(TrackedMarker<float> &marker,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &indices,
	float timestep, float sigma);

#endif // POINT_TRACKING_3D_H