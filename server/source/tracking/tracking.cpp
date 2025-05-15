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

#include "tracking/tracking.hpp"
#include "tracking/kalman.hpp"

#include "util/log.hpp"

#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"

#include "unsupported/Eigen/NonLinearOptimization"

bool simulateTrackTarget(TrackerState &state, TrackerObservation &observation,
	Eigen::Isometry3f simulatedPose, CovarianceMatrix covariance, bool success,
	TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params)
{
	TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	if (dtMS(state.time, time) > 0.01f)
	{ // Predict new state if not done already using IMU samples
		flexkalman::predict(state.state, model, dtS(state.time, time));
		state.time = time;
		observation.extrapolated = state.state.getIsometry().cast<float>();
	}
	observation.predicted = state.state.getIsometry().cast<float>();
	observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	if (!success) return false;
	observation.observed = simulatedPose;
	//observation.covObserved = covariance;
	observation.covObserved = params.filter.getCovariance<float>();

	// Update state
	auto measurement = AbsolutePoseMeasurement(
		observation.observed.translation().cast<double>(),
		Eigen::Quaterniond(observation.observed.rotation().cast<double>()),
		observation.covObserved.cast<double>()
	);
	flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
	if (!flexkalman::correctUnscented(state.state, measurement, true, sigmaParams))
	{
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
	}
	state.lastObservation = time;
	state.lastObsFrame = frame;

	observation.filtered = state.state.getIsometry().cast<float>();
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	return true;
}

bool trackTarget(TrackerState &state, TrackerTarget &target, TrackerObservation &observation,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, unsigned int frame, int cameraCount, const TargetTrackingParameters &params)
{
	TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	if (dtMS(state.time, time) > 0.01f)
	{ // Predict new state if not done already using IMU samples
		flexkalman::predict(state.state, model, dtS(state.time, time));
		state.time = time;
		observation.extrapolated = state.state.getIsometry().cast<float>();
	}
	observation.predicted = state.state.getIsometry().cast<float>();
	observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	// Match target with points and optimise pose
	target.match2D = trackTarget2D(*target.calib, observation.predicted, observation.covPredicted,
		calibs, cameraCount, points2D, properties, relevantPoints2D, params, target.data);
	if (target.match2D.error.samples < params.minTotalObs) return false;
	LOG(LTracking, LDebug, "    Pixel Error after 2D target track: %fpx mean over %d points\n",
		target.match2D.error.mean*PixelFactor, target.match2D.error.samples);
	observation.observed = target.match2D.pose;

	// Option 1: Initial values
	observation.covObserved = params.filter.getCovariance<float>() * params.filter.trackSigma;
	// Option 2: Full covariance numerically estimated
	//observation.covObserved = target.match2D.covariance;
	// But yields a bit odd results, distorts the positional covariance when I don't see why it should
	// Option 3: Numerical covariance for position only
	//observation.covObserved.topLeftCorner<3,3>() = fitCovarianceToSamples<3,float>(target.match2D.deviations);

	// Update state
	auto measurement = AbsolutePoseMeasurement(
		observation.observed.translation().cast<double>(),
		Eigen::Quaterniond(observation.observed.rotation().cast<double>()),
		observation.covObserved.cast<double>()
	);
	flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
	if (!flexkalman::correctUnscented(state.state, measurement, true, sigmaParams))
	{
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in optical filter! Reset!");
	}
	state.lastObservation = time;
	state.lastObsFrame = frame;

	observation.filtered = state.state.getIsometry().cast<float>();
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
	observation.time = time;

	return target.match2D.error.samples >= params.minTotalObs && target.match2D.error.mean < params.maxTotalError;
}

int trackMarker(TrackerState &state, TrackerMarker &marker, TrackerObservation &observation,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices,
	TimePoint_t time, float sigma)
{
	// TODO: Track individual large markers - expose parameters
	TargetTrackingParameters params = {};

	TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	if (dtMS(state.time, time) > 0.01f)
	{ // Predict new state if not done already using IMU samples
		flexkalman::predict(state.state, model, dtS(state.time, time));
		state.time = time;
		observation.extrapolated = state.state.getIsometry().cast<float>();
	}
	observation.predicted = state.state.getIsometry().cast<float>();
	observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	Eigen::Vector3f predPos = observation.predicted.translation();
	Eigen::Matrix3f predCov = observation.covPredicted.topLeftCorner<3,3>();

	// Find best candidate
	int matchedPoint = -1;
	float matchedErrorProbability = std::numeric_limits<float>::max();
	for (int p : triIndices)
	{
		Eigen::Vector3f diff = points3D[p]-predPos;
		float err = diff.norm();
		Eigen::Vector3f dir = diff/err;
		float var = dir.transpose() * predCov * dir;
		float limit = std::sqrt(var) * 3;
		if (err < limit)
		{ // Within sigma interval
			matchedPoint = p;
			matchedErrorProbability = err/limit;
		}
	}

	if (matchedPoint < 0 || matchedErrorProbability > 1000)
	{
		LOG(LTracking, LWarn, "Best point %d of %d points had %f error probability\n", matchedPoint, (int)points3D.size(), matchedErrorProbability);
		return -1;
	}
	observation.observed.translation() = points3D[matchedPoint];
	observation.covObserved.setIdentity();
	observation.covObserved.diagonal().head<3>().setConstant(params.filter.stdDevPos);

	// Update state
	auto measurement = AbsolutePositionMeasurement(
		points3D[matchedPoint].cast<double>(),
		observation.covObserved.diagonal().head<3>().cast<double>().eval());
	flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
	if (!flexkalman::correctUnscented(state.state, measurement, true, sigmaParams))
	{
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in marker filter! Reset!");
	}

	state.lastObservation = time;
	state.lastObsFrame++;
	// TODO: Track individual large markers - switch to frame number

	observation.filtered = state.state.getIsometry().cast<float>() ;
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
	observation.time = time;

	return matchedPoint;
}

static Eigen::Quaterniond GyroToQuat(Eigen::Vector3d gyro)
{
	double angle = gyro.norm(); // In rad/s
	if (angle < 0.0001f) return Eigen::Quaterniond::Identity();
	return Eigen::Quaterniond(Eigen::AngleAxisd(angle, gyro / angle));
}

template<typename IMUSample>
static bool integrateIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params);

template<typename IMUSample>
static void integrateIMUSample(TrackerInertial &inertial, const IMUSample &sample, const IMUSample &lastSample, const TargetTrackingParameters &params, TrackerInertial::State &filterState, bool updateFilter, bool isOptical, bool isInterpolated);

template<typename IMUSample>
static IMUSample interpolateIMUSample(const IMUSample &sampleA, const IMUSample &sampleB, TimePoint_t time);

bool integrateIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	if (inertial.imu->isFused)
		return integrateIMU<IMUSampleFused>(state, inertial, observation, time, params);
	else
		return integrateIMU<IMUSampleRaw>(state, inertial, observation, time, params);
	return true;
}

template<> IMUSampleFused interpolateIMUSample(const IMUSampleFused &sampleA, const IMUSampleFused &sampleB, TimePoint_t time)
{
	double t = dtMS(sampleA.timestamp, time) / dtMS(sampleA.timestamp, sampleB.timestamp);
	IMUSampleFused sample;
	sample.timestamp = time;
	sample.accel = sampleA.accel * (1-t) + sampleB.accel * t;
	sample.quat = sampleA.quat.slerp(t, sampleB.quat);
	return sample;
}

template<> IMUSampleRaw interpolateIMUSample(const IMUSampleRaw &sampleA, const IMUSampleRaw &sampleB, TimePoint_t time)
{
	double t = dtMS(sampleA.timestamp, time) / dtMS(sampleA.timestamp, sampleB.timestamp);
	IMUSampleRaw sample;
	sample.timestamp = time;
	sample.accel = sampleA.accel * (1-t) + sampleB.accel * t;
	sample.gyro = sampleA.gyro * (1-t) + sampleB.gyro * t;
	sample.mag = sampleA.mag * (1-t) + sampleB.mag * t;
	return sample;
}

template<typename IMUSample>
bool integrateIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	typename TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	if (inertial.calibration.phase == IMU_CALIB_UNKNOWN)
	{ // Initialise calibration
		if (inertial.imu->tracker.orientation.coeffs().hasNaN())
			inertial.calibration.phase = IMU_CALIB_EXT_ORIENTATION;
		else
		{
			inertial.calibration.conversion = inertial.imu->tracker.conversion;
			inertial.calibration.quat = inertial.imu->tracker.orientation.cast<double>();
			inertial.calibration.mat = inertial.calibration.quat.toRotationMatrix() * inertial.calibration.conversion.cast<double>();
			if (inertial.imu->tracker.offset.hasNaN())
				inertial.calibration.phase = IMU_CALIB_EXT_OFFSET;
			else
			{
				inertial.calibration.offset = inertial.imu->tracker.offset.cast<double>();
				inertial.calibration.phase = IMU_CALIB_DONE;
			}
		}
	}

	{ // Extrapolate without IMU samples for debug purposes
		auto filterState = state.state;
		flexkalman::predict(filterState, model, dtS(state.time, time));
		observation.extrapolated = filterState.getIsometry().cast<float>();
	}

	auto filterState = state.state;
	auto filterTime = state.time;
	bool updateFilter = inertial.calibration.phase > IMU_CALIB_EXT_ORIENTATION;

	// Find first IMU sample after last state time (e.g. last frame)
	typename BlockedQueue<IMUSample, 16384>::template View<true> samples;
	IMUSample *interpolatedSample;
	if constexpr (std::is_same_v<IMUSample, IMUSampleFused>)
	{
		samples = inertial.imu->samplesFused.getView<true>();
		interpolatedSample = &inertial.interpolatedSample.fused;
	}
	else
	{
		samples = inertial.imu->samplesRaw.getView<true>();
		interpolatedSample = &inertial.interpolatedSample.raw;
	}
	auto itBegin = samples.begin();
	if (state.lastIMUSample >= samples.beginIndex() && state.lastIMUSample < samples.endIndex())
	{ // Start search from last sample used
		itBegin = samples.pos(state.lastIMUSample+1);
		while (itBegin != samples.end() && itBegin->timestamp < filterTime) itBegin++;
	}
	else if (!samples.empty())
	{ // Cold start
		itBegin = std::lower_bound(samples.begin(), samples.end(), filterTime);
		LOG(LTrackingIMU, LDebug, "    Cold start of IMU %d with sample %lu!", inertial.imu->index, itBegin.index());
	}
	if (itBegin == samples.end())
	{
		state.lastIMUSample = samples.endIndex()-1;
		return false;
	}

	// Pick reference sample (last or first)
	IMUSample lastSample;
	if (itBegin == samples.begin() || dtMS(std::prev(itBegin)->timestamp, itBegin->timestamp) > 20)
	{ // No prior reference, use first IMU sample as reference
		if ((state.lastObsFrame >= 0 || state.lastIMUSample >= 0) && filterTime < itBegin->timestamp)
		{ // Have prior filter state to update
			flexkalman::predict(filterState, model, dtS(filterTime, itBegin->timestamp));
		}
		filterTime = itBegin->timestamp;
		lastSample = *itBegin;
		itBegin = std::next(itBegin);
	}
	else
	{ // Use last IMU sample as reference
		lastSample = *std::prev(itBegin);
	}

	// Find last IMU sample before specified maximum time (e.g. current frame)
	auto itEnd = itBegin;
	while (itEnd != samples.end() && itEnd->timestamp < time) itEnd++;

	// Integrate range of IMU samples
	for (auto sample = itBegin; sample < itEnd; sample++)
	{
		LOG(LTrackingFilter, LTrace, "    Integrating sample %fms ahead of last IMU sample, %fms ahead of state, %fms ahead of fusion%s",
			dtMS(lastSample.timestamp, sample->timestamp), dtMS(filterTime, sample->timestamp), dtMS(inertial.fusion.time, sample->timestamp),
			state.lastObsFrame == 0? "!" : asprintf_s(", %fms ahead of last observation!", dtMS(state.lastObservation, sample->timestamp)).c_str());

		// Predict up until new IMU filter time
		flexkalman::predict(filterState, model, dtS(filterTime, sample->timestamp));
		filterTime = sample->timestamp;

		// Integrate on current IMU filter state
		integrateIMUSample(inertial, *sample, lastSample, params, filterState, updateFilter, state.lastObsFrame >= 0, false);
		lastSample = *sample;
		state.lastIMUSample = sample.index();
		state.lastIMUTime = sample->timestamp;
	}

	// Predict new state
	flexkalman::predict(filterState, model, dtS(filterTime, time));
	filterTime = time;

	// Get best estimate for sample at current time (extrapolated if need be) for prediction
	if (itEnd != samples.end())
		*interpolatedSample = interpolateIMUSample(lastSample, *itEnd, time);
	else
	{ // Extrapolate if no more recent sample exists
		*interpolatedSample = lastSample;
		interpolatedSample->timestamp = time;
	}

	// Integrate interpolated IMU sample to bring IMU fusion time up to date
	integrateIMUSample(inertial, *interpolatedSample, lastSample, params, filterState, updateFilter, state.lastObsFrame >= 0, true);
	state.lastIMUTime = time;

	observation.inertialIntegrated.translation() = filterState.position().cast<float>();
	observation.inertialIntegrated.linear() = inertial.fusion.integrated.toRotationMatrix().cast<float>();
	observation.inertialFused.translation() = filterState.position().cast<float>();
	observation.inertialFused.linear() = inertial.fusion.quat.toRotationMatrix().cast<float>();
	observation.inertialFiltered.translation() = filterState.position().cast<float>();
	observation.inertialFiltered.linear() = filterState.getCombinedQuaternion().toRotationMatrix().cast<float>();

	//observation.covIMU = ;

	if (updateFilter)
	{ // Actually apply IMU integrations to state
		state.state = filterState;
		state.time = filterTime;
	}

	return true;
}

template<> void integrateIMUSample(TrackerInertial &inertial, const IMUSampleFused &sample, const IMUSampleFused &lastSample, const TargetTrackingParameters &params, TrackerInertial::State &filterState, bool updateFilter, bool isOptical, bool isInterpolated)
{
	bool prevInterpolated = inertial.interpolatedSample.raw.timestamp > lastSample.timestamp;
	float factor = 0;
	if (lastSample.timestamp < inertial.fusion.time)
	{ // Account for time difference between last IMU sample and last filter time (e.g. optical measurement)
		factor = dtMS(lastSample.timestamp, inertial.fusion.time) / dtMS(lastSample.timestamp, sample.timestamp);
		LOG(LTrackingFilter, LTrace, "        Lerping last sample to current with a factor of %f", factor);
	}
	Eigen::Quaterniond lastQuat = lastSample.quat.slerp(factor, sample.quat).cast<double>();
	Eigen::Quaterniond quat = sample.quat.cast<double>();

	float dt = dtS(inertial.fusion.time, sample.timestamp);
	if (dt > 0.1) dt = 0;
	inertial.fusion.time = sample.timestamp;

	Eigen::Quaterniond dQuat = quat * lastQuat.conjugate();

	if (!isOptical)
	{ // Take as absolute
		inertial.fusion.quat = quat;
	}
	/* else if (inertial.calibration.phase > IMU_CALIB_EXT_ORIENTATION && !inertial.imu->hasMag)
	{ // Make Z-axis change relative if there is no mag to compensate for drift
		auto getZ = [](Eigen::Quaterniond q)
		{ // Both are the same
			//return q.toRotationMatrix().canonicalEulerAngles(2, 1, 0).x();
			return std::atan2(2*q.x()*q.y() + 2*q.w()*q.z(), 1 - (2*q.y()*q.y() + 2*q.z()*q.z()));
		};
		double eZ = getZ(inertial.fusion.quat) - getZ(lastQuat);
		inertial.fusion.quat = Eigen::AngleAxisd(eZ, Eigen::Vector3d::UnitZ()) * quat;
	}
	else if (inertial.calibration.phase > IMU_CALIB_EXT_ORIENTATION && inertial.imu->hasMag)
	{ // Take as absolute
		inertial.fusion.quat = quat;
	} */
	else // TODO: Force relative only if last optical sample is recent, and ensure proper transition to absolute
	{ // Make all axis change relative to compensate for drift
		inertial.fusion.quat = inertial.fusion.quat * dQuat;
	}

	// Fused Alignment
	// TODO: Implement automatic calibration for fused IMU
	inertial.calibration.fused.current.quatAlt1 = inertial.calibration.fused.current.quatAlt1 * dQuat;
	inertial.calibration.fused.current.quatAlt2 = dQuat * inertial.calibration.fused.current.quatAlt2;
	inertial.calibration.fused.current.dQuat = inertial.calibration.fused.current.dQuat * dQuat;

	inertial.fusion.accelRaw = sample.accel.cast<double>();
	// Add gravity back in
	inertial.fusion.accelRaw.z() -= 9.81f;
	// Reorient back from world to IMU-local space
	inertial.fusion.accelRaw = sample.quat.cast<double>().conjugate() * inertial.fusion.accelRaw;
	// Reorient from IMU-local to tracker-local space
	inertial.fusion.accelLocal = inertial.calibration.mat * inertial.fusion.accelRaw;
	// Reorient to corrected quat
	inertial.fusion.accel = inertial.fusion.quat * inertial.fusion.accelLocal;
	// Remove gravity again
	inertial.fusion.accel.z() += 9.81f;
	// Integrate velocity
	inertial.fusion.imuVelocity *= 0.99f;
	inertial.fusion.imuVelocity += inertial.fusion.accel * dt;


	if (updateFilter && inertial.calibration.phase > IMU_CALIB_EXT_ORIENTATION)
	{ // Correct with updated IMU filter quat
		auto measurement = FusedIMUMeasurement{
			inertial.fusion.quat,
			Eigen::Vector3d::Constant(params.filter.stdDevIMU*params.filter.stdDevIMU)
		};
		flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
		if (!flexkalman::correctUnscented(filterState, measurement, true, sigmaParams))
		{
			LOG(LTrackingFilter, LWarn, "Failed to correct pose in IMU filter! Reset!");
		}
	}
	if (updateFilter && inertial.calibration.phase > IMU_CALIB_EXT_OFFSET)
	{ // TODO: Correct with updated IMU filter accel
	}
}

template<> void integrateIMUSample(TrackerInertial &inertial, const IMUSampleRaw &sample, const IMUSampleRaw &lastSample, const TargetTrackingParameters &params, TrackerInertial::State &filterState, bool updateFilter, bool isOptical, bool isInterpolated)
{
	bool prevInterpolated = inertial.interpolatedSample.raw.timestamp > lastSample.timestamp;
	bool ignoreInterpolated = false;
	if (isInterpolated && ignoreInterpolated)
		return;

	auto &calib = inertial.calibration.gyro;
	if (calib.sampling && !calib.aborted && !isInterpolated)
	{ // Gyro Alignment
		assert(!calib.current.gyroSamples.empty());
		float dt = dtS(calib.current.gyroSamples.back().first, sample.timestamp);
		float angDiff = sample.gyro.norm() * dt * 180/PI;
		if (angDiff > 5.0f || dt > 0.02)
		{ // Too fast, retract gyro samples to time of last optical measurement
			calib.aborted = true;
			if (calib.current.gyroSamples.size() > 50)
				LOG(LTrackingIMU, LDebug, "    Collection of %d gyro samples was aborted due to sample of %fdg over %fms!",
					(int)calib.current.gyroSamples.size(), angDiff, dt*1000);
		}
		else
		{ // Add new sample
			calib.current.gyroSamples.emplace_back(sample.timestamp, sample.gyro.cast<double>() ) ;	
		}
	}

	// Update fusion time delta
	float dt = dtS(inertial.fusion.time, sample.timestamp);
	if (dt > 0.1) dt = 0;

	// Determine integration steps
	Eigen::Vector3d gyroInt, accelInt;
	if (prevInterpolated && !ignoreInterpolated)
	{ // This assumes postCorrectIMU already somewhat corrects for errors introduced with the interpolatedSample
		// As inertial.interpolatedSample might have been extrapolated only
		IMUSampleRaw interpolated = interpolateIMUSample(lastSample, sample, inertial.fusion.time);
		gyroInt = inertial.calibration.mat * (interpolated.gyro.cast<double>() + sample.gyro.cast<double>()) / 2 * dt;
	}
	else
	{ // Integrate normally as current fusion state matches lastSample
		gyroInt = inertial.calibration.mat * (lastSample.gyro.cast<double>() + sample.gyro.cast<double>()) / 2 * dt;
	}
	accelInt = inertial.fusion.accel * dt;

	// Update gyrometer fusion state
	Eigen::Quaterniond dQuat = GyroToQuat(gyroInt);
	inertial.fusion.integrated = inertial.fusion.integrated * dQuat;
	inertial.fusion.quat = inertial.fusion.quat * dQuat;
	inertial.fusion.angularVelocity = inertial.calibration.mat * sample.gyro.cast<double>();

	// Update accelerometer fusion state
	inertial.fusion.accelRaw = sample.accel.cast<double>();
	// Reorient from IMU-local to tracker-local space
	inertial.fusion.accelLocal = inertial.calibration.mat * inertial.fusion.accelRaw;
	// Reorient from tracker-local to world space
	inertial.fusion.accel = inertial.fusion.quat * inertial.fusion.accelLocal;
	// Remove gravity
	inertial.fusion.accel.z() += 9.81f;
	// Integrate velocity
	inertial.fusion.imuVelocity *= 0.99f;
	inertial.fusion.imuVelocity += accelInt;

	// Update fusion time
	inertial.fusion.time = sample.timestamp;

	if (updateFilter && inertial.calibration.phase > IMU_CALIB_EXT_ORIENTATION)
	{ // Correct with updated IMU filter quat
		auto measurement = FusedIMUMeasurement{
			inertial.fusion.quat,
			Eigen::Vector3d::Constant(params.filter.stdDevIMU*params.filter.stdDevIMU)
		};
		flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
		if (!flexkalman::correctUnscented(filterState, measurement, true, sigmaParams))
		{
			LOG(LTrackingFilter, LWarn, "Failed to correct pose in IMU filter! Reset!");
		}
	}
	if (updateFilter && inertial.calibration.phase > IMU_CALIB_EXT_OFFSET)
	{ // TODO: Correct with updated IMU filter accel
	}
}

static bool collectGravitySamples(TrackerInertial &inertial, const TrackerInertial::State &state);
static bool collectGyroSamples(TrackerInertial &inertial, const TrackerState &state);
static bool collectFusedSamples(TrackerInertial &inertial, const TrackerInertial::State &state);
static void alignIMUOrientation(TrackerInertial &inertial);

static void calibrateIMUOffset(TrackerInertial &inertial, const TrackerInertial::State &lastState, const TrackerInertial::State &newState);

void postCorrectIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	auto &calib = inertial.calibration;

	if (calib.phase < IMU_CALIB_DONE)
	{ // In calibration phase
		bool newSamples = false;

		if (calib.phase == IMU_CALIB_EXT_GRAVITY)
			newSamples |= collectGravitySamples(inertial, state.state);

		if (calib.phase == IMU_CALIB_EXT_ALIGNMENT && !inertial.imu->isFused)
			newSamples |= collectGyroSamples(inertial, state);

		float dtLast = dtMS(calib.lastTime, time);
		if (dtLast > 400)
		{
			if (calib.phase == IMU_CALIB_EXT_ALIGNMENT && inertial.imu->isFused)
					newSamples |= collectFusedSamples(inertial, state.state);

			// Update last state based on observation for calibration
			calib.lastState = state.state;
			calib.lastTime = state.time;
		}

		int countSamples = calib.accel.samples.size() + calib.gyro.samples.size() + calib.fused.samples.size();
		if (newSamples && calib.accel.samples.size() >= 3 && (countSamples % 5) == 0)
			alignIMUOrientation(inertial);
	}
	else
	{
		calib.accel.sampling = false;
		calib.gyro.sampling = false;
		calib.fused.sampling = false;
	}

	// Update IMU positions
	observation.inertialIntegrated.translation() = state.state.position().cast<float>();
	observation.inertialFused.translation() = state.state.position().cast<float>();

	//LOG(LTrackingIMU, LDebug, "Re-basing IMU Integration!");
	if (inertial.fusion.corrections == 0)
	{
		inertial.fusion.integrated = state.state.getCombinedQuaternion();
		inertial.fusion.quat = state.state.getCombinedQuaternion();
		inertial.fusion.time = time;
	}
	else
	{
		inertial.fusion.quat = inertial.fusion.quat.slerp(0.1f, state.state.getCombinedQuaternion());
		inertial.fusion.time = time;
	}
	inertial.fusion.corrections++;

	/*
	inertial.fusion.positionalVelocity = state.state.velocity();
	inertial.fusion.angularVelocity = state.state.angularVelocity();
	{
		Eigen::Quaterniond rot = flexkalman::util::quat_exp(state.state.angularVelocity());
		Eigen::AngleAxisd eul(rot);
		inertial.fusion.angularVelocity = eul.angle() * eul.axis();
	}
	inertial.fusion.tangentialVelocity = inertial.fusion.angularVelocity.cross(calib.offset);
	inertial.fusion.imuVelocity = state.state.velocity() + inertial.fusion.tangentialVelocity;
	*/
}

void interruptIMU(TrackerInertial &inertial)
{
	inertial.calibration.accel.sampling = false;
	inertial.calibration.gyro.sampling = false;
	inertial.calibration.gyro.aborted = false;
	inertial.calibration.fused.sampling = false;
	inertial.fusion = {};
}

static bool collectGravitySamples(TrackerInertial &inertial, const TrackerInertial::State &state)
{
	double vel = state.velocity().norm();
	double ang = state.angularVelocity().norm();
	double grv = std::abs(inertial.fusion.accelRaw.norm()-9.81f);
	bool abortSampling =
		vel > 0.2f ||
		ang > 0.4f ||
		grv > 0.3f;
	const int MIN = 100, MAX = 200;

	auto &calib = inertial.calibration.accel;

	float diff = Eigen::AngleAxisd(Eigen::Quaterniond::FromTwoVectors(calib.current.inertial, calib.current.optical)).angle()*180/PI;

	if (abortSampling)
	{ // Abort, may add anyway
		calib.sampling = false;
	}
	else if (!calib.sampling)
	{ // Start new
		calib.current = {};
		calib.numCurrent = 0;
		calib.sampling = true;
	}

	if (calib.sampling)
	{ // Record gravity vectors
		calib.current.optical += state.getCombinedQuaternion().conjugate() * -Eigen::Vector3d::UnitZ();
		calib.current.inertial += inertial.fusion.accelRaw.normalized();
		calib.numCurrent++;

		if (calib.numCurrent >= MAX)
		{ // Stop after reaching limits
			calib.sampling = false;
		}
	}

	if (!calib.sampling && calib.numCurrent > 0)
	{ // Check if samples should be accepted
		if (calib.numCurrent >= MIN)
		{ // Accept
			calib.current.optical /= calib.numCurrent;
			calib.current.inertial /= calib.numCurrent;
			calib.samples.push_back(calib.current);
			LOG(LTrackingIMU, LInfo,
				"    Collected gravity vectors across %d samples, diff of %fdg, motion (vel %f, ang vel %f, grav diff %f)",
				calib.numCurrent, diff, vel, ang, grv);

			calib.numCurrent = 0;
			return true;
		}
		else if (calib.numCurrent >= 50)
		{
			LOG(LTrackingIMU, LDebug,
				"    Collection of gravity vectors across %d samples, diff of %fdg, was interrupted by motion (vel %f, ang vel %f, grav diff %f)",
				calib.numCurrent, diff, vel, ang, grv);
		}
		calib.numCurrent = 0;
	}

	return false;
}

static bool collectGyroSamples(TrackerInertial &inertial, const TrackerState &state)
{
	const int MIN = 50, MAX = 300;
	const double MinAng = 20, MaxAng = 60;

	assert(std::abs(dtMS(state.time, inertial.interpolatedSample.raw.timestamp)) < 0.001);

	auto &calib = inertial.calibration.gyro;
	float diff = Eigen::AngleAxisd(calib.current.quatStart.conjugate() * state.state.getCombinedQuaternion()).angle()*180/PI;

	double angVel = state.state.angularVelocity().norm();
	bool abortSampling = angVel < 0.1f || angVel > 5.0f;
	if (abortSampling && calib.current.gyroSamples.size() > 50)
		LOG(LTrackingIMU, LDebug, "    Collection of %d gyro samples was aborted due to filtered angular velocity of %fdg/s",
			(int)calib.current.gyroSamples.size(), angVel*180/PI);

	if (calib.aborted || abortSampling)
	{ // Abort collection, return to last "checkpoint"
		calib.sampling = false;
		calib.aborted = false;
		if (!calib.current.gyroSamples.empty())
		{ // Cut off samples after last optical measurement ("checkpoint")
			int i;
			for (i = calib.current.gyroSamples.size()-1; i >= 0; i--)
				if (calib.current.gyroSamples[i].first < calib.opticalInterpolated.timestamp)
					break;
			calib.current.gyroSamples.resize(i+1);
		}
		if (!calib.current.gyroSamples.empty())
		{ // Add final sample interpolated at the time of final optical measurement
			calib.current.gyroSamples.emplace_back(calib.opticalInterpolated.timestamp, calib.opticalInterpolated.gyro.cast<double>());
		}
	}
	else if (calib.current.gyroSamples.size() >= MAX || diff >= MaxAng)
	{ // Stop collection after reaching limits
		calib.sampling = false;
		// Add final sample interpolated at the time of final optical measurement
		calib.current.gyroSamples.emplace_back(inertial.interpolatedSample.raw.timestamp, inertial.interpolatedSample.raw.gyro.cast<double>());
	}
	else if (!calib.sampling)
	{ // Start new collection
		calib.sampling = true;
		calib.current = {};
		calib.current.quatStart = state.state.getCombinedQuaternion();
		calib.current.quatEnd = state.state.getCombinedQuaternion();
		// Add first sample interpolated at the time of reference optical measurement
		calib.current.gyroSamples.emplace_back(inertial.interpolatedSample.raw.timestamp, inertial.interpolatedSample.raw.gyro.cast<double>());
	}

	if (calib.sampling)
	{ // Record new optical measurement as "checkpoint"
		calib.opticalInterpolated = inertial.interpolatedSample.raw;
		calib.current.quatEnd = state.state.getCombinedQuaternion();
	}
	else if (calib.current.gyroSamples.size() >= MIN && diff >= MinAng)
	{ // Accept collected samples for calibration
		LOG(LTrackingIMU, LInfo, "    Collected %d gyro samples over rotation of %fdg!",
			(int)calib.current.gyroSamples.size(), diff);
		calib.samples.push_back(std::move(calib.current));
		calib.current = {};
		return true;
	}
	else if (!calib.current.gyroSamples.empty())
		calib.current = {};

	return false;
}

static bool collectFusedSamples(TrackerInertial &inertial, const TrackerInertial::State &state)
{
	auto &calib = inertial.calibration.fused;
	calib.sampling = true;
	calib.current.observed = state.getCombinedQuaternion();

	const double MinAng = 20, MaxAng = 60;
	float diff = Eigen::AngleAxisd(calib.current.reference.conjugate() * calib.current.observed).angle()*180/PI;

	// TODO: Implement automatic calibration for fused IMU

	bool accept = false;
	if (!calib.current.reference.isApprox(Eigen::Quaterniond::Identity())
		&& diff > MinAng && diff < MaxAng)
	{ // Accept
		calib.samples.push_back(calib.current);
		accept = true;
	}

	// Reset
	calib.current = {};
	calib.current.reference = state.getCombinedQuaternion();
	calib.current.observed = state.getCombinedQuaternion();

	calib.current.quatAlt1 = state.getCombinedQuaternion();
	calib.current.quatAlt2 = state.getCombinedQuaternion();
	calib.current.dQuat.setIdentity();

	return accept;
}

template<typename M> 
constexpr auto Mat = [](auto m, auto... ms) {
return ((M()<<m),...,ms).finished();
};

static const std::vector<Eigen::Matrix3i> convAxisSwizzle = {
	Mat<Eigen::Matrix3i>( 1,0,0, 0,1,0, 0,0,1 ),
	Mat<Eigen::Matrix3i>( 0,1,0, 1,0,0, 0,0,1 ),
	Mat<Eigen::Matrix3i>( 0,0,1, 0,1,0, 1,0,0 ),
	Mat<Eigen::Matrix3i>( 1,0,0, 0,0,1, 0,1,0 ),
};
static const std::vector<Eigen::Vector3i> convAxisFlip = {
	Eigen::Vector3i(+1,+1,+1),
	Eigen::Vector3i(-1,+1,+1),
	Eigen::Vector3i(+1,-1,+1),
	Eigen::Vector3i(+1,+1,-1),
	Eigen::Vector3i(-1,-1,+1),
	Eigen::Vector3i(+1,-1,-1),
	Eigen::Vector3i(-1,+1,-1),
	Eigen::Vector3i(-1,-1,-1)
};

struct AlignementErrorTerm
{
	enum
	{
		InputsAtCompileTime = Eigen::Dynamic,
		ValuesAtCompileTime = Eigen::Dynamic
	};
	typedef double Scalar;
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	std::vector<TrackerInertial::AccelAlignSample> accelSamples;
	std::vector<TrackerInertial::GyroAlignSample> gyroSamples;
	std::vector<TrackerInertial::FusedAlignSample> fusedSamples;

	static int conversions() { return convAxisSwizzle.size()*convAxisFlip.size(); }
	static int fusedOpts() { return 12; }

	int optFused = 0;

	int conv = 0;

	Eigen::Matrix3i getConversion() const
	{
		Eigen::Matrix3i conversion = convAxisSwizzle[conv/convAxisFlip.size()];
		Eigen::Vector3i flip = convAxisFlip[conv%convAxisFlip.size()];
		conversion.col(0) *= flip(0);
		conversion.col(1) *= flip(1);
		conversion.col(2) *= flip(2);
		return conversion;
	}

	Eigen::Quaterniond convert(Eigen::Matrix3d conversion, Eigen::Quaterniond quat) const
	{
		quat.vec() = conversion * quat.vec();
		return quat;
	}

	Eigen::Quaterniond getQuatDiff(TrackerInertial::AccelAlignSample sample, Eigen::Matrix3d conversion, Eigen::Quaterniond orientation) const
	{
		return Eigen::Quaterniond::FromTwoVectors(
			orientation * conversion * sample.inertial,
			sample.optical);
	}

	Eigen::Quaterniond getQuatDiff(TrackerInertial::GyroAlignSample sample, Eigen::Matrix3d conversion, Eigen::Quaterniond orientation) const
	{
		Eigen::Quaterniond dQuat = sample.quatStart.conjugate() * sample.quatEnd;
		Eigen::Quaterniond gyroIntegration = Eigen::Quaterniond::Identity();
		for (int i = 0; i < sample.gyroSamples.size()-1; i++)
		{
			auto &gyroLast = sample.gyroSamples[i];
			auto &gyroNext = sample.gyroSamples[i+1];
			float dt = dtS(gyroLast.first, gyroNext.first);
			Eigen::Vector3d gyro = (gyroLast.second + gyroNext.second) / 2;
			gyroIntegration = gyroIntegration * (orientation * GyroToQuat(conversion * gyro * dt) * orientation.conjugate());
		}
		return dQuat.conjugate() * gyroIntegration;
	}

	Eigen::Quaterniond getQuatDiff(TrackerInertial::FusedAlignSample sample, Eigen::Matrix3d conversion, Eigen::Quaterniond orientation) const
	{
		// TODO: Implement automatic calibration for fused IMU - current attempts don't work
		switch (optFused)
		{
			case 0: return sample.observed.conjugate() * sample.reference * orientation * convert(conversion, sample.quatAlt1);
			case 1: return sample.observed.conjugate() * orientation * convert(conversion, sample.quatAlt1) * sample.reference;
			case 2: return orientation * convert(conversion, sample.quatAlt1) * sample.reference * sample.observed.conjugate();
			case 3: return convert(conversion, sample.observed.conjugate() * sample.reference * sample.quatAlt1) * orientation;
			case 4: return convert(conversion, sample.quatAlt1 * sample.reference * sample.observed.conjugate()) * orientation;
			case 5: return sample.observed.conjugate() * sample.reference * orientation * convert(conversion, sample.quatAlt2);
			case 6: return sample.observed.conjugate() * orientation * convert(conversion, sample.quatAlt2) * sample.reference;
			case 7: return orientation * convert(conversion, sample.quatAlt2) * sample.reference * sample.observed.conjugate();
			case 8: return convert(conversion, sample.observed.conjugate() * sample.reference * sample.quatAlt2) * orientation;
			case 9: return convert(conversion, sample.quatAlt2 * sample.reference * sample.observed.conjugate()) * orientation;
			case 10: return sample.observed.conjugate() * sample.reference * orientation * convert(conversion, sample.quatAlt1) * orientation.conjugate();
			case 11: return sample.observed.conjugate() * sample.reference * orientation * convert(conversion, sample.quatAlt2) * orientation.conjugate();
			default: assert(false); return Eigen::Quaterniond(NAN, NAN, NAN, NAN);
		}
	}

	int operator()(const Eigen::VectorXd &coeffs, Eigen::VectorXd &errors) const
	{
		Eigen::Quaterniond orientation = flexkalman::util::quat_exp(coeffs.head<3>());
		Eigen::Matrix3d conversion = getConversion().cast<double>();
		int index = 0;
		for (int i = 0; i < accelSamples.size(); i++, index++)
			errors(index) = Eigen::AngleAxisd(getQuatDiff(accelSamples[i], conversion, orientation)).angle();
		for (int i = 0; i < gyroSamples.size(); i++, index++)
			errors(index) = Eigen::AngleAxisd(getQuatDiff(gyroSamples[i], conversion, orientation)).angle();
		for (int i = 0; i < fusedSamples.size(); i++, index++)
			errors(index) = Eigen::AngleAxisd(getQuatDiff(fusedSamples[i], conversion, orientation)).angle();
		return 0;
	}

	int inputs() const { return 3; }
	int values() const { return accelSamples.size() + gyroSamples.size() + fusedSamples.size(); }
};

static void alignIMUOrientation(TrackerInertial &inertial)
{
	// Precalculation for gyro alignment
	float gyroDiffAvg = 0;
	for (auto &sample : inertial.calibration.gyro.samples)
		gyroDiffAvg += Eigen::AngleAxisd(sample.quatEnd * sample.quatStart.conjugate()).angle();
	gyroDiffAvg = gyroDiffAvg/inertial.calibration.gyro.samples.size();

	// Precalculation for fused alignment
	float fusedDiffAvg = 0;
	for (auto &sample : inertial.calibration.fused.samples)
		fusedDiffAvg += Eigen::AngleAxisd(sample.observed * sample.reference.conjugate()).angle();
	fusedDiffAvg = fusedDiffAvg/inertial.calibration.fused.samples.size();

	// Start off with gravity alignment, sufficient but requires user action

	float bestError = std::numeric_limits<float>::max();
	Eigen::Matrix<int8_t,3,3> conversion;
	Eigen::Quaterniond orientation; 
	int bestConv = -1;

	LOG(LTrackingIMU, LInfo, "Attempting to calibrate IMU orientation alignment to target!");

	AlignementErrorTerm errorTerm = {};
	errorTerm.accelSamples = inertial.calibration.accel.samples;

	for (int c = 0; c < AlignementErrorTerm::conversions(); c++)
	{
		errorTerm.gyroSamples.clear();
		errorTerm.fusedSamples.clear();
		errorTerm.conv = c;
		Eigen::VectorXd quatCoeff = Eigen::VectorXd::Random(errorTerm.inputs());

		if (errorTerm.accelSamples.size() > 3)
		{ // If user placed down tracker in different orientation (no matter what IMU type)
			LOG(LTrackingIMU, LInfo, "    Aligning with just %d gravity samples assuming conversion %d:",
				(int)inertial.calibration.accel.samples.size(), c);

			Eigen::NumericalDiff<AlignementErrorTerm> errorGradient(errorTerm);
			Eigen::LevenbergMarquardt<Eigen::NumericalDiff<AlignementErrorTerm>, double> lm(errorGradient);
			auto status = lm.minimize(quatCoeff);
			auto &errorsOpt = lm.fvec;

			LOG(LTrackingIMU, LInfo,
				"        Optimised error to %fdg with max %fdg",
				errorsOpt.mean()*180/PI, errorsOpt.maxCoeff()*180/PI);

			if (lm.fvec.mean()*180/PI > 5.0)
				continue;
		}

		if (!inertial.calibration.gyro.samples.empty())
		{ // IF IMU is providing IMUSampleRaw
			LOG(LTrackingIMU, LInfo, "    Aligning with %d gravity samples and %d gyro samples assuming conversion %d:",
				(int)inertial.calibration.accel.samples.size(), (int)inertial.calibration.gyro.samples.size(), c);

			errorTerm.gyroSamples = inertial.calibration.gyro.samples;

			Eigen::VectorXd errorsPre(errorTerm.values());
			errorTerm(quatCoeff, errorsPre);

			Eigen::NumericalDiff<AlignementErrorTerm> errorGradient(errorTerm);
			Eigen::LevenbergMarquardt<Eigen::NumericalDiff<AlignementErrorTerm>, double> lm(errorGradient);
			auto status = lm.minimize(quatCoeff);
			auto &errorsOpt = lm.fvec;

			// LOGGING ONLY:

			auto errorsGyro = errorsOpt.tail(inertial.calibration.gyro.samples.size());
			LOG(LTrackingIMU, LInfo,
				"        Optimised error of %fdg to %fdg with max %fdg - gyro relative error %f%%",
				errorsPre.mean()*180/PI, errorsOpt.mean()*180/PI, errorsOpt.maxCoeff()*180/PI, errorsGyro.mean()/gyroDiffAvg*100);

			LOG(LTrackingIMU, LDebug, "            Errors Pre: %s", printMatrix((errorsPre*180/PI).transpose()).c_str());
			LOG(LTrackingIMU, LDebug, "            Errors Opt: %s", printMatrix((errorsOpt*180/PI).transpose()).c_str());

			Eigen::VectorXd errorsRel = Eigen::VectorXd::Zero(errorsGyro.size());
			Eigen::VectorXd maxGyro = Eigen::VectorXd::Zero(errorsGyro.size());
			Eigen::VectorXd maxDGyro = Eigen::VectorXd::Zero(errorsGyro.size());
			Eigen::VectorXd maxDT = Eigen::VectorXd::Zero(errorsGyro.size());
			for (int i = 0; i < inertial.calibration.gyro.samples.size(); i++)
			{
				auto &sample = inertial.calibration.gyro.samples[i];
				errorsRel(i) = errorsOpt(inertial.calibration.accel.samples.size() + i)
					/ Eigen::AngleAxisd(sample.quatEnd * sample.quatStart.conjugate()).angle();
				for (int j = 0; j < sample.gyroSamples.size()-1; j++)
				{
					auto &gyroLast = sample.gyroSamples[j];
					auto &gyroNext = sample.gyroSamples[j+1];
					float dt = dtS(gyroLast.first, gyroNext.first);
					Eigen::Vector3d gyro = (gyroLast.second + gyroNext.second) / 2;
					maxGyro(i) = std::max(gyroNext.second.norm(), maxGyro(i));
					maxDGyro(i) = std::max(gyro.norm()*dt*180/PI, maxDGyro(i));
					maxDT(i) = std::max((double)dt*1000, maxDT(i));
				}
			}

			LOG(LTrackingIMU, LDebug, "            Relative Errors (x):  %s", printMatrix(errorsRel.transpose()).c_str());
			LOG(LTrackingIMU, LDebug, "            Max Gyro (rad/s):  %s", printMatrix(maxGyro.transpose()).c_str());
			LOG(LTrackingIMU, LDebug, "            Max dGyro (dg):  %s", printMatrix(maxDGyro.transpose()).c_str());
			LOG(LTrackingIMU, LDebug, "            Max dT (ms):  %s", printMatrix(maxDT.transpose()).c_str());
		}

		if (!inertial.calibration.fused.samples.empty())
		{ // IF IMU is providing IMUSampleFused

			// TODO: Implement automatic calibration for fused IMU

			LOG(LTrackingIMU, LInfo, "    Aligning with %d gravity samples and %d fused quat samples assuming conversion %d:",
				(int)inertial.calibration.accel.samples.size(), (int)inertial.calibration.fused.samples.size(), c);

			errorTerm.fusedSamples = inertial.calibration.fused.samples;

			for (int f = 0; f < AlignementErrorTerm::fusedOpts(); f++)
			{
				errorTerm.optFused = f;
				if (errorTerm.fusedSamples.size() == 0) continue;

				Eigen::VectorXd errorsPre(errorTerm.values());
				errorTerm(quatCoeff, errorsPre);

				Eigen::NumericalDiff<AlignementErrorTerm> errorGradient(errorTerm);
				Eigen::LevenbergMarquardt<Eigen::NumericalDiff<AlignementErrorTerm>, double> lm(errorGradient);
				auto status = lm.minimize(quatCoeff);
				auto &errorsOpt = lm.fvec;

				LOG(LTrackingIMU, LInfo,
					"        Formula %d optimised error of %fdg to %fdg with max %fdg - fused quat relative error %f%%",
					f, errorsPre.mean()*180/PI, errorsOpt.mean()*180/PI, errorsOpt.maxCoeff()*180/PI,
					errorsOpt.tail(inertial.calibration.fused.samples.size()).mean()/fusedDiffAvg*100);
			}
		}

		Eigen::VectorXd errors(errorTerm.values());
		errorTerm(quatCoeff, errors);
		float angleErr = errors.mean()*180/PI;
		if (angleErr*1.001f < bestError)
		{
			bestError = angleErr;
			conversion = errorTerm.getConversion().cast<int8_t>();
			orientation = flexkalman::util::quat_exp(quatCoeff.head<3>()); 
			bestConv = c;
			if (angleErr < 2.0f)
				break;
		}
	}

	if (bestConv >= 0)
	{
		LOG(LTrackingIMU, LInfo, "Selected best conversion %d with error %fdg", bestConv, bestError);
		LOG(LTrackingIMU, LDebug, "Orientation Quat: (%f, %f, %f), %f", orientation.x(), orientation.y(), orientation.z(), orientation.w());
		LOG(LTrackingIMU, LDebug, "Conversion matrix: \n%s", printMatrix(conversion).c_str());

		inertial.calibration.conversion = conversion;
		inertial.calibration.quat = orientation;
		inertial.calibration.mat = orientation.toRotationMatrix() * conversion.cast<double>();
		inertial.imu->tracker.conversion = conversion;
		inertial.imu->tracker.orientation = orientation.cast<float>();
	}
}

static void calibrateIMUOffset(TrackerInertial &inertial, const TrackerInertial::State &lastState, const TrackerInertial::State &newState)
{
	inertial.calibration.offset.setIdentity();
	//inertial.calibration.offset = Eigen::Vector3d(0.2f, 0.05f, -0.15f);

	// Global velocity change observed at IMUs offset from the tracker
	Eigen::Vector3d dVelIntegrated = inertial.fusion.imuVelocity - (lastState.velocity() + lastState.angularVelocity().cross(inertial.calibration.offset));

	// Determine observed global velocity change at a given offset from the tracker

}
