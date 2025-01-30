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

#include "util/eigendef.hpp"
#include "util/kalman.hpp"
#include "target/target.hpp"
#include "target/tracking2D.hpp"

#include "flexkalman/process/PoseSeparatelyDampedConstantVelocity.h"
#include "flexkalman/state/PoseState.h"


#include <vector>

/**
 * Tracking a target (set of markers) in 3D space
 */


/* Structures */

struct None { };

// Own EKF-based filter variants based on kalman library
template<typename FilterImpl>
struct TrackingFilter
{
	using Scalar = typename FilterImpl::Scalar;
	using TimeStep = typename FilterImpl::TimeStep;
	using StatePred = typename FilterImpl::StatePred;
	using StateFilter = typename FilterImpl::StateFilter;

	// State
	int measurements;
	StatePred curStatePred;
	StateFilter curStateFilter;
	Isometry3<Scalar> poseObserved; // Pose as observed by the cameras
	Isometry3<Scalar> poseFiltered; // Pose filtered from all observations

	// Filter
	typename FilterImpl::Filter filter;
	typename FilterImpl::MovementModel movementModel;
	typename FilterImpl::MeasurementModel measurementModel;
	float errorScaleT = 0.001f, errorScaleR = 1.0f/180*PI; // Initially expecting 1mm, 1dg accuracy 

	// Prediction
	StateFilter predStateFilter; // Only stored for debug
	StatePred predStatePred; // Only stored for debug
	Isometry3<Scalar> posePredicted;
	Eigen::Matrix<Scalar,3,1> stdDev;
	Scalar errorProbability;

	void init(Isometry3<Scalar> initialPose);
	inline Isometry3<Scalar> predict(TimeStep timeStep);
	inline void update(TimeStep timeStep, Isometry3<Scalar> newPose);
	inline void updateError(TimeStep timeStep, Scalar sigma);
	inline void debugState(StateFilter &stateFilter, StatePred &statePred, std::string label, TimeStep timeStep);
};

// UKF-based Filter from monado based on flexkalman
struct FlexUKFFilter
{
	using Scalar = float;
	using StateFilter = flexkalman::pose_externalized_rotation::State;
	using ProcessModel = flexkalman::PoseSeparatelyDampedConstantVelocityProcessModel<StateFilter>;

	// State
	int measurements;
	TimePoint_t curTime;
	StateFilter curStateFilter;
	Isometry3<Scalar> poseObserved; // Pose as observed by the cameras
	Isometry3<Scalar> poseFiltered; // Pose filtered from all observations

	// Filter
	ProcessModel movementModel;

	// Prediction
	Isometry3<Scalar> posePredicted;
	Eigen::Matrix<Scalar,3,1> stdDev;

	void init(Isometry3<Scalar> initialPose)
	{
		curStateFilter.position() = initialPose.translation().cast<double>();
		curStateFilter.setQuaternion(Eigen::Quaterniond(initialPose.rotation().cast<double>()));
	}
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
	TrackedTarget(TargetTemplate3D const *target, Eigen::Isometry3f pose) : target(target)
	{
		filter.init(pose.cast<Scalar>());
	}

	Isometry3<Scalar> getPoseObserved() const { return filter.poseObserved; }
	Isometry3<Scalar> getPoseFiltered() const { return filter.poseFiltered; }
	Isometry3<Scalar> getPosePredicted() const { return filter.posePredicted; }
	Vector3<Scalar> getPredictionStdDev() const { return filter.stdDev; }
	int getMeasurements() const { return filter.measurements; }
};

template<typename ScalarType>
struct Filter_Man_PV_Man_RV
{
	using Scalar = ScalarType;
	using TimeStep = Scalar;
	using StatePred = State6DOF_Quat<Scalar>;
	
	using StateFilter = None;
	using MovementModel = None;
	using MeasurementModel = None;
	using Filter = None;
};

template<typename ScalarType>
struct Filter_EKF_PV_Man_RV
{
	using Scalar = ScalarType;
	using TimeStep = TimeControl<Scalar>;
	using StatePred = State3DOF_Quat<Scalar>;
	
	using StateFilter = State3DOFV<Scalar>;
	using MovementModel = SystemModel3DOFV<Scalar>;
	using MeasurementModel = MeasurementModel3DOFV<Scalar>;
	using Filter = Kalman::ExtendedKalmanFilter<StateFilter>;
};

template<typename ScalarType>
struct Filter_EKF_PA_Man_RV
{
	using Scalar = ScalarType;
	using TimeStep = TimeControl<Scalar>;
	using StatePred = State3DOF_Quat<Scalar>;
	
	using StateFilter = State3DOF<Scalar>;
	using MovementModel = SystemModel3DOF<Scalar>;
	using MeasurementModel = MeasurementModel3DOF<Scalar>;
	using Filter = Kalman::ExtendedKalmanFilter<StateFilter>;
};

template<typename ScalarType>
struct Filter_EKF_PA_EKF_RV
{
	using Scalar = ScalarType;
	using TimeStep = TimeControl<Scalar>;
	using StatePred = State3DOF_Quat<Scalar>; // None

	using StateFilter = State6DOF_MEKF<Scalar>;
	using MovementModel = SystemModel6DOF_MEKF<Scalar>;
	using MeasurementModel = MeasurementModel6DOF_MEKF<Scalar>;
	using Filter = Kalman::ExtendedKalmanFilter<StateFilter>;
};

// Works fine
//typedef TrackedTarget<TrackingFilter<Filter_Man_PV_Man_RV<float>>> TrackedTargetFiltered;

// Works, not sure if this is any better than fully manual
//typedef TrackedTarget<TrackingFilter<Filter_EKF_PV_Man_RV<float>>> TrackedTargetFiltered;

// Very jittery, might need parameter tuning, won't use EKF with acceleration model for now
//typedef TrackedTarget<TrackingFilter<Filter_EKF_PA_Man_RV<float>>> TrackedTargetFiltered;

// Barely implemented, doesn't work well at all, but it is fully MEKF
//typedef TrackedTarget<TrackingFilter<Filter_EKF_PA_EKF_RV<float>>> TrackedTargetFiltered;

typedef TrackedTarget<FlexUKFFilter> TrackedTargetFiltered;

/* Functions */

template<typename TARGET = TrackedTargetFiltered>
bool trackTarget(TARGET &target, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, float timestep, int cameraCount, const TargetTrackingParameters &params);

#endif // TRACKING_3D_H