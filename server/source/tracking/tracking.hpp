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
#include "tracking/virtual.hpp"
#include "pipeline/record.hpp"

#include "util/util.hpp" // TimePoint_t
#include "util/eigendef.hpp"
#include "util/stats.hpp"

#include "flexkalman/process/PoseSeparatelyDampedConstantVelocity.h"
#include "flexkalman/state/PoseState.h"

/**
 * Tracking a target (set of markers) in 3D space
 */

/* Tracker Components */

struct TrackerFilter
{
	using State = flexkalman::pose_externalized_rotation::State;
	using Model = flexkalman::PoseSeparatelyDampedConstantVelocityProcessModel<State>;

	// Current state
	State state;
	TimePoint_t time;

	// Information about state
	OptFrameNum firstObsFrame;
	TimePoint_t firstObservation;
	OptFrameNum lastObsFrame;
	TimePoint_t lastObservation;
	OptFrameNum lastIMUSample;
	TimePoint_t lastIMUTime;

	TrackerFilter() : state{}, firstObsFrame(-1), lastObsFrame(-1), lastIMUSample(-1) {}

	inline void initialise(Eigen::Isometry3f pose, TimePoint_t time, OptFrameNum frame, const TargetTrackingParameters &params)
	{
		firstObsFrame = lastObsFrame = frame;
		firstObservation = lastObservation = time;
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

	// Store internal data for visualisation purposes (low overhead)
	TargetTracking2DData data;

	TrackerTarget(TargetCalibration3D &&calib, TargetDetectionConfig config) :
		calib(std::move(calib)), detectionConfig(config), data{} {}
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
	TrackerPose pose;
	TrackerPoseExtended ext;

	TrackerObservation() {}

	TrackerObservation(Eigen::Isometry3f pose_, TimePoint_t time, const TargetTrackingParameters &params) :
		time(time), pose(pose_, pose_), ext(pose_, pose_)
	{
		pose.observedCov = pose.filteredCov = ext.predictedCov = params.filter.getSyntheticCovariance<float>() * params.filter.sigmaInitState;
	}
};

struct TrackerVirtual
{
	TrackerVirtualConfig config;

	// Current internal state
	OptFrameNum lastValidFrame;
	OptFrameNum mistrustFrames;
	bool alignmentDirty;
	TrackerVirtualError error;

	// Calibrated up vector of each subtracker, manually calibrated
	int collectingUpVectors;
	std::vector<Eigen::Matrix4f> trackerUpAccum;
	std::vector<Eigen::Quaternionf> trackerUpVector;

	// Stored in TrackerRecord, for visualisation
	std::vector<Eigen::Vector3f> relations; // Calibrated offsets of a virtual tracker to subtrackers in world space
	std::vector<VirtualSubtrackerDebug> subtrackers; // Debug information for each subtracker that is currently tracked
};

/* Tracked Object Representations */

struct TrackedTarget;
struct DormantTarget;
struct IMUMarker;
struct OrphanedIMU;

struct TrackedBase
{
	int id;
	std::string label;
	TrackingResult result;	// Read-Write by main pipeline only
	float mistrust = 0.0f;	// Read-Write by main pipeline only

	inline TrackedBase(int id, std::string label) : id(id), label(label), mistrust(0.0f) {} // For MSVC...
};

/**
 * A target with some information to feed the filter
 * Might be currently or recently optically tracked
 * Or might have an IMU associated that updates filter
 */
struct TrackedTarget : public virtual TrackedBase
{
	// Target tracking source
	TrackerTarget target;

	// Optional inertial tracking source
	TrackerInertial inertial;

	// Current filtered state
	TrackerFilter filter;

	// Latest observation
	TrackerObservation obs;

	inline TrackedTarget(DormantTarget &&dormant);

	inline TrackedTarget(DormantTarget &&dormant, Eigen::Isometry3f obsPose,
		TimePoint_t time, FrameNum frame, const TargetTrackingParameters &params);

	inline void PickUpTracking(Eigen::Isometry3f obsPose,
		TimePoint_t time, FrameNum frame, const TargetTrackingParameters &params);

	inline void InterruptTracking();
};

/**
 * A target with no information where it is
 * May have been observed before or not
 * Does not have an (active) IMU
 */
struct DormantTarget : public virtual TrackedBase
{
	// Target tracking source
	TrackerTarget target;

	// TODO: May add information about when it was last seen
	int detectionCycle;

	// Optional inertial tracking source
	// Generally inactive if associated at all, otherwise it should be kept as TrackedTarget with filter
	TrackerInertial inertial;

	inline DormantTarget(TrackedTarget &&tracker);

	DormantTarget(int id, std::string label, TrackerTarget &&target)
		: TrackedBase(id, label), target(std::move(target)), detectionCycle(0), inertial{} {}
};

/**
 * A marker of known size with IMU associated, allowing for consistent identification.
 * 6-DOF by combining 3-DOF absolute optical position and 3-DOF inertial rotation (may drift).
 */
struct IMUMarker : public virtual TrackedBase
{
	// Single Marker tracking source
	TrackerMarker marker;

	// Inertial tracking source
	// Should always be assigned, otherwise hard to reliably identify just based on size
	TrackerInertial inertial;

	// Current filtered state
	TrackerFilter filter;

	// Latest observation
	TrackerObservation obs;

	IMUMarker(int id, std::string label, TrackerMarker marker) :
		TrackedBase(id, label), marker(std::move(marker)), inertial{} {}

	inline void PickUpTracking(Eigen::Vector3f pos,
		TimePoint_t time, FrameNum frame, const TargetTrackingParameters &params);

	inline void InterruptTracking();
};

struct VirtualTracker : public virtual TrackedBase
{
	// Virtual tracking source
	TrackerVirtual virt;

	// Current filtered state
	TrackerFilter filter;

	// Latest virtual observation
	TrackerObservation obs;

	inline VirtualTracker(int id, std::string label, TrackerVirtualConfig config)
		: TrackedBase(id, label), virt{std::move(config)} {};
};

const static Eigen::Isometry3f orphanedIMUPose = Eigen::Isometry3f(Eigen::Translation3f(0, 0, 1));

struct OrphanedIMU
{
	TrackerFilter filter;

	// Inertial tracking source
	TrackerInertial inertial;

	// Latest observation
	TrackerObservation obs;

	OrphanedIMU(std::shared_ptr<IMU> &imu, const TargetTrackingParameters &params) :
		filter{},
		inertial(imu, IMUCalib()),
		obs(orphanedIMUPose, sclock::now(), params)
	{
		filter.initialise(orphanedIMUPose, sclock::now(), -1, params);
	}

	OrphanedIMU(std::shared_ptr<IMU> &&imu, const TargetTrackingParameters &params) :
		filter{},
		inertial(std::move(imu), IMUCalib()),
		obs(orphanedIMUPose, sclock::now(), params)
	{
		filter.initialise(orphanedIMUPose, sclock::now(), -1, params);
	}
};


/* Functions */

TrackingResult simulateTrackTarget(TrackerFilter &filter, TrackerTarget &target, TrackerObservation &obs,
	const std::vector<CameraCalib> &calibs, const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const TrackerRecord &record, TimePoint_t time, FrameNum frame, const TargetTrackingParameters &params);

TrackingResult trackTarget(TrackerFilter &filter, TrackerTarget &target, TrackerObservation &obs, TargetMatch2D &match2D,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, FrameNum frame, int cameraCount, const TargetTrackingParameters &params);

TrackingResult trackMarker(TrackerFilter &filter, TrackerMarker &marker, TrackerObservation &obs,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices, int *bestPoint,
	TimePoint_t time, float sigma);

TrackingResult processVirtualTracker(TrackerFilter &filter, TrackerVirtual &virt, TrackerObservation &obs,
	std::vector<TrackerFilter*> &subtrackers, TimePoint_t time, FrameNum frame, const VirtualTrackingParameters &params);

bool integrateIMU(TrackerFilter &filter, TrackerInertial &inertial, TrackerObservation &obs,
	TimePoint_t time, const TargetTrackingParameters &params);
void postCorrectIMU(TrackedBase &tracker, TrackerFilter &filter, TrackerInertial &inertial, TrackerObservation &obs,
	TimePoint_t time, const TargetTrackingParameters &params);
void interruptIMU(TrackerInertial &inertial);
void resetIMU(TrackerInertial &inertial);


/* Methods */

inline TrackedTarget::TrackedTarget(DormantTarget &&dormant) :
	TrackedBase(dormant.id, dormant.label), target(std::move(dormant.target)), inertial(std::move(dormant.inertial)) {}

inline TrackedTarget::TrackedTarget(DormantTarget &&dormant, Eigen::Isometry3f obsPose,
	TimePoint_t time, FrameNum frame, const TargetTrackingParameters &params) :
	TrackedBase(dormant.id, dormant.label), target(std::move(dormant.target)), inertial(std::move(dormant.inertial))
{
	PickUpTracking(obsPose, time, frame, params);
}

inline void TrackedTarget::PickUpTracking(Eigen::Isometry3f obsPose,
	TimePoint_t time, FrameNum frame, const TargetTrackingParameters &params)
{
	filter.initialise(obsPose, time, frame, params);
	obs = TrackerObservation(obsPose, time, params);
	if (inertial)
		postCorrectIMU(*this, filter, inertial, obs, time, params);
}

inline void TrackedTarget::InterruptTracking()
{
	// TODO: Add seeded re-detection for targets with inertial units (1/3)
	// This will be called with the intention to interrupt optical tracking
	// with subsequent re-detection efforts based on inertial data 
	interruptIMU(inertial);
}

inline DormantTarget::DormantTarget(TrackedTarget &&tracker) :
	TrackedBase(tracker.id, tracker.label), target(std::move(tracker.target)), detectionCycle(0), inertial(std::move(tracker.inertial))
{
	resetIMU(inertial);
}

inline void IMUMarker::PickUpTracking(Eigen::Vector3f pos,
	TimePoint_t time, FrameNum frame, const TargetTrackingParameters &params)
{
	Eigen::Isometry3f obsPose;
	obsPose.translation() = pos;
	obsPose.linear() = inertial.fusion.quat.toRotationMatrix().cast<float>();
	filter.initialise(obsPose, time, frame, params);
	obs = TrackerObservation(obsPose, time, params);
}

inline void IMUMarker::InterruptTracking()
{
	// TODO: Now in search for marker of given size following same pattern as IMU accelerometer
}

#endif // TRACKING_3D_H