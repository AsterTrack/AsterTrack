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
#include "util/stats.hpp"

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
	long firstObsFrame;
	TimePoint_t lastObservation;
	TimePoint_t firstObservation;

	TrackerState(Eigen::Isometry3f pose, long frame, TimePoint_t time, const TargetTrackingParameters &params) :
		state{}, time(time), lastIMUSample(-1), lastIMUTime(time), lastObsFrame(-1), lastObservation(time), firstObsFrame(frame), firstObservation(time)
	{
		state.position() = pose.translation().cast<double>();
		state.setQuaternion(Eigen::Quaterniond(pose.rotation().cast<double>()));
		Eigen::Matrix<double,6,6> covariance = params.filter.getSyntheticCovariance<double>();
		state.errorCovariance().topLeftCorner<6,6>() = covariance * params.filter.sigmaInitState;
		state.errorCovariance().bottomRightCorner<6,6>() = covariance * params.filter.sigmaInitChange;
	}
};

struct TrackerTarget
{
	// Calibrated target template
	TargetCalibration3D calib;

	// Config of detection methods used for this target 
	TargetDetectionConfig detectionConfig;
	int detectionCycle;

	// Store internal data for visualisation purposes (low overhead)
	TargetTracking2DData data;

	// 2D Tracking result
	TargetMatch2D match2D;

	TrackerTarget(TargetCalibration3D &&calib, TargetDetectionConfig config) :
		calib(std::move(calib)), detectionConfig(config), detectionCycle(0), data{}, match2D{} {}
};


struct TrackerMarker
{
	// For matching of markers, either single marker size or target radius
	float size;

	TrackerMarker(float size) : size(size) {}
};

enum IMUCalibrationPhase
{
	IMU_CALIB_UNKNOWN = 0,
	// Internal calibration
	// TODO: Proper bias calibration for raw samples
	// External calibration
	IMU_CALIB_EXT_GRAVITY = 1,
	IMU_CALIB_EXT_ALIGNMENT = 2,
	IMU_CALIB_EXT_ORIENTATION = IMU_CALIB_EXT_GRAVITY | IMU_CALIB_EXT_ALIGNMENT,
	IMU_CALIB_EXT_OFFSET = 4,
	IMU_CALIB_DONE = 8
};

struct TrackerInertial
{
	std::shared_ptr<IMU> imu;
	IMUCalib calib;

	using State = flexkalman::pose_externalized_rotation::State;
	struct AccelAlignSample
	{
		Eigen::Vector3d optical = Eigen::Vector3d::Zero();
		Eigen::Vector3d inertial = Eigen::Vector3d::Zero();
	};
	struct GyroAlignSample
	{
		Eigen::Quaterniond quatStart, quatEnd;
		std::vector<std::pair<TimePoint_t,Eigen::Vector3d>> gyroSamples;
	};
	struct FusedAlignSample
	{
		Eigen::Quaterniond reference;
		Eigen::Quaterniond observed;
		Eigen::Quaterniond quatAlt1;
		Eigen::Quaterniond quatAlt2;
		Eigen::Quaterniond dQuat;
	};
	struct {
		IMUCalibrationPhase phase = IMU_CALIB_UNKNOWN;
		// Data
		State lastState;
		TimePoint_t lastTime;
		struct
		{
			std::vector<AccelAlignSample> samples;
			AccelAlignSample current;
			int numCurrent = 0;
			bool sampling = false;
		} accel = {};
		struct
		{
			std::vector<GyroAlignSample> samples;
			GyroAlignSample current;
			bool sampling = false;
			IMUSampleRaw opticalInterpolated;
			bool aborted = false;
		} gyro = {};
		struct
		{
			std::vector<FusedAlignSample> samples;
			FusedAlignSample current;
			bool sampling = true;
		} fused = {};
		Eigen::Matrix3d mat; // quat * conversion
	} calibration = {};
	struct {
		TimePoint_t time;
		Eigen::Quaterniond integrated = Eigen::Quaterniond::Identity();
		Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
		int corrections = 0;
		TimePoint_t lastIntegration;
		StatFloatingf sampleInterval;

		Eigen::Vector3d imuVelocity = Eigen::Vector3d::Zero(); // Positional velocity of IMU (due to tracker positional velocity AND rotation)
		Eigen::Vector3d angularVelocity; // Angular velocity of tracker rotation
		Eigen::Vector3d tangentialVelocity; // Positional velocity of IMU relative to tracker due to rotation
		Eigen::Vector3d positionalVelocity; // Positional velocity of tracker
		Eigen::Vector3d accel, accelLocal, accelRaw;
		Eigen::Vector3d gravityDir;
	} fusion = {};

	/**
	 * Interpolated samples at the current time in case they're needed for prediction/calibration
	 * E.g. if all IMU samples before an optical measurement are integrated, but a newer one does exist
	 * this is interpolated between them at the time of optical measurement to be used for better prediction
	 */
	struct {
		IMUSampleFused fused;
		IMUSampleRaw raw;
	} interpolatedSample;

	TrackerInertial() : imu(), calibration{}, fusion{} {}
	TrackerInertial(std::shared_ptr<IMU> &imu, IMUCalib calib) : imu(imu), calib(calib), calibration{}, fusion{} {}
	TrackerInertial(std::shared_ptr<IMU> &&imu, IMUCalib calib) : imu(std::move(imu)), calib(calib), calibration{}, fusion{} {}

	operator bool() const { return imu != nullptr; }
};

struct TrackerObservation
{
	TimePoint_t time;
	Eigen::Isometry3f predicted; // Predicted using extrapolation and/or IMU integration
	Eigen::Isometry3f extrapolated; // Extrapolated using just the model from last observation
	Eigen::Isometry3f inertialIntegrated; // Pose as integrated from new IMU samples
	Eigen::Isometry3f inertialFused; // Pose as integrated and fused with optical samples
	Eigen::Isometry3f inertialFiltered; // Pose as integrated and filtered with kalman filter
	Eigen::Isometry3f observed; // Pose as observed by the cameras
	Eigen::Isometry3f filtered; // Pose filtered from all observations
	Eigen::Matrix<float,6,6> covPredicted, covFiltered, covObserved;

	TrackerObservation(Eigen::Isometry3f pose, TimePoint_t time, const TargetTrackingParameters &params) :
		predicted(pose), observed(pose), filtered(pose), time(time)
	{
		covObserved = params.filter.getSyntheticCovariance<float>() * params.filter.sigmaInitState;
		covPredicted = covFiltered = covObserved;
	}
};

/* Tracked Object Representations */

struct TrackedTarget;
struct DormantTarget;
struct TrackedMarker;
struct DormantMarker;
struct OrphanedIMU;

struct TrackedBase
{
	int id;
	std::string label;
	TrackingResult result;
	float procTimeMS;
	float mistrust = 0.0f;
};

struct TrackedTarget : public virtual TrackedBase
{
	// Target tracking source
	TrackerTarget target;

	// Optional inertial tracking source
	TrackerInertial inertial;

	// Current filtered state
	TrackerState state;

	// Latest observation
	TrackerObservation pose;

	inline TrackedTarget(DormantTarget &&dormant, Eigen::Isometry3f pose,
		TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params);
};

struct TrackedMarker : public virtual TrackedBase
{
	// Single Marker tracking source
	TrackerMarker marker;

	// Optional inertial tracking source
	TrackerInertial inertial;

	// Current filtered state
	TrackerState state;

	// Latest observation
	TrackerObservation pose;

	inline TrackedMarker(DormantMarker &&dormant, Eigen::Vector3f pos,
		TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params);
};

struct DormantTarget : public virtual TrackedBase
{
	// Target tracking source
	TrackerTarget target;

	// Optional inertial tracking source
	TrackerInertial inertial;

	inline DormantTarget(TrackedTarget &&tracker);
	DormantTarget(int id, std::string label, TrackerTarget &&target) : TrackedBase(id, label), target(std::move(target)), inertial() {}
};

struct DormantMarker : public virtual TrackedBase
{
	// Single Marker tracking source
	TrackerMarker marker;

	// Optional inertial tracking source
	TrackerInertial inertial;

	inline DormantMarker(TrackedMarker &&tracker);
	DormantMarker(int id, std::string label, TrackerMarker &&marker) : TrackedBase(id, label), marker(std::move(marker)), inertial() {}
};

const static Eigen::Isometry3f orphanedIMUPose = Eigen::Isometry3f(Eigen::Translation3f(0, 0, 1));

struct OrphanedIMU
{
	TrackerState state;

	// Inertial tracking source
	TrackerInertial inertial;

	// Latest observation
	TrackerObservation pose;

	OrphanedIMU(std::shared_ptr<IMU> &imu, const TargetTrackingParameters &params) :
		state(orphanedIMUPose, -1, sclock::now(), params),
		inertial(imu, IMUCalib()),
		pose(orphanedIMUPose, sclock::now(), params) {}
	
	OrphanedIMU(std::shared_ptr<IMU> &&imu, const TargetTrackingParameters &params) :
		state(orphanedIMUPose, -1, sclock::now(), params),
		inertial(std::move(imu), IMUCalib()),
		pose(orphanedIMUPose, sclock::now(), params) {}
};


/* Functions */

TrackingResult simulateTrackTarget(TrackerState &state, TrackerTarget &target, TrackerObservation &observation,
	const std::vector<CameraCalib> &calibs, const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const TrackerRecord &record, TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params);

TrackingResult trackTarget(TrackerState &state, TrackerTarget &target, TrackerObservation &observation,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, unsigned int frame, int cameraCount, const TargetTrackingParameters &params);

TrackingResult trackMarker(TrackerState &state, TrackerMarker &marker, TrackerObservation &observation,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices, int *bestPoint,
	TimePoint_t time, float sigma);

bool integrateIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params);
void postCorrectIMU(TrackedBase &tracker, TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params);
void interruptIMU(TrackerInertial &inertial);


inline TrackedTarget::TrackedTarget(DormantTarget &&dormant, Eigen::Isometry3f pose,
	TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params) :
	TrackedBase(dormant.id, dormant.label),
	target(std::move(dormant.target)), inertial(std::move(dormant.inertial)),
	state(pose, frame, time, params), pose(pose, time, params)
{
	state.lastObservation = time;
	state.lastObsFrame = frame;
	if (inertial)
		postCorrectIMU(*this, state, inertial, this->pose, time, params);
}

inline DormantTarget::DormantTarget(TrackedTarget &&tracker) :
	TrackedBase(tracker.id, tracker.label),
	inertial(std::move(tracker.inertial)), target(std::move(tracker.target)) 
{
	interruptIMU(inertial);
}

inline TrackedMarker::TrackedMarker(DormantMarker &&dormant, Eigen::Vector3f pos,
	TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params) :
	TrackedBase(dormant.id, dormant.label),
	marker(std::move(dormant.marker)), inertial(std::move(dormant.inertial)),
	state(Eigen::Isometry3f(Eigen::Translation3f(pos)), frame, time, params), 
	pose(Eigen::Isometry3f(Eigen::Translation3f(pos)), time, params)
{
	state.lastObservation = time;
	state.lastObsFrame = frame;
	if (inertial)
		postCorrectIMU(*this, state, inertial, this->pose, time, params);
}

inline DormantMarker::DormantMarker(TrackedMarker &&tracker) :
	TrackedBase(tracker.id, tracker.label),
	inertial(std::move(tracker.inertial)), marker(std::move(tracker.marker)) 
{
	interruptIMU(inertial);
}

#endif // TRACKING_3D_H