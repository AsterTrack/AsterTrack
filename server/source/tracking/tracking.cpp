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

bool simulateTrackTarget(TrackerState &state, TrackerObservation &observation,
	Eigen::Isometry3f simulatedPose, CovarianceMatrix covariance, bool success,
	TimePoint_t time, unsigned int frame, const TargetTrackingParameters &params)
{
	TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	// Predict new state
	flexkalman::predict(state.state, model, dtS(state.time, time));
	state.time = time;
	observation.predicted = observation.extrapolated = state.state.getIsometry().cast<float>();
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

	if (state.time != time)
	{ // Predict new state if not done already using IMU samples
		flexkalman::predict(state.state, model, dtS(state.time, time));
		state.time = time;

		observation.predicted = observation.extrapolated = state.state.getIsometry().cast<float>();
		observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
	}
	Eigen::Vector3f stdDev = observation.covPredicted.diagonal().head<3>().cwiseSqrt();

	// Match target with points and optimise pose
	target.match2D = trackTarget2D(*target.calib, observation.predicted, stdDev, calibs, cameraCount, 
		points2D, properties, relevantPoints2D, params, target.data);
	if (target.match2D.error.samples < params.minTotalObs) return false;
	LOG(LTracking, LDebug, "    Pixel Error after 2D target track: %fpx mean over %d points\n",
		target.match2D.error.mean*PixelFactor, target.match2D.error.samples);
	observation.observed = target.match2D.pose;
	observation.covObserved = params.filter.getCovariance<float>();

	// TODO: Get variance/covariance from optimisation via jacobian
	// Essentially, any parameter that barely influences output error has a high variance
	// Neatly encapsulates the degrees of freedom (and their strength) left by varying distribution of blobs

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
	// TODO: Move into parameters once this is used
	TargetTrackingParameters params = {};

	TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	// Predict new state
	flexkalman::predict(state.state, model, dtS(state.time, time));
	state.time = time;

	observation.predicted = observation.extrapolated = state.state.getIsometry().cast<float>();
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
	state.lastObsFrame++; // TODO: Switch to frame once used

	observation.filtered = state.state.getIsometry().cast<float>() ;
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
	observation.time = time;

	return matchedPoint;
}

static void integrateIMUSample(TrackerInertial &inertial, const IMUSampleFused &sample, const IMUSampleFused &lastSample, const TargetTrackingParameters &params, bool isOptical);
static void integrateIMUSample(TrackerInertial &inertial, const IMUSampleRaw &sample, const IMUSampleRaw &lastSample, const TargetTrackingParameters &params, bool isOptical);

template<typename Sample>
static bool integrateIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	typename TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	{ // Extrapolate without IMU samples for debug purposes
		auto filterState = state.state;
		flexkalman::predict(filterState, model, dtS(state.time, time));
		observation.extrapolated = filterState.getIsometry().cast<float>();
	}

	auto filterState = state.state;
	auto filterTime = state.time;
	bool updateFilter = inertial.calibration.phase > IMU_CALIB_EXT_ORIENTATION;

	// Find first IMU sample after last state time (e.g. last frame)
	typename BlockedQueue<Sample, 16384>::template View<true> samples;
	if constexpr (std::is_same_v<Sample, IMUSampleFused>)
		samples = inertial.imu->samplesFused.getView<true>();
	else
		samples = inertial.imu->samplesRaw.getView<true>();
	auto itBegin = samples.begin();
	if (state.lastIMUSample >= samples.beginIndex() && state.lastIMUSample < samples.endIndex())
	{ // Start search from last sample used
		itBegin = samples.pos(state.lastIMUSample+1);
		while (itBegin != samples.end() && itBegin->timestamp < filterTime) itBegin++;
	}
	else if (!samples.empty())
	{ // Cold start
		itBegin = std::lower_bound(samples.begin(), samples.end(), filterTime);
		LOG(LTrackingIMU, LDebug, "Cold start of IMU %d with sample %lu!", inertial.imu->index, itBegin.index());
	}
	if (itBegin == samples.end())
	{
		state.lastIMUSample = itBegin.index()-1;
		return false;
	}

	// Pick reference sample (last or first)
	Sample lastSample;
	if (itBegin == samples.begin() || dtMS(std::prev(itBegin)->timestamp, itBegin->timestamp) > 20)
	{ // No prior reference, use first IMU sample as reference
		if ((state.lastObsFrame >= 0 || state.lastIMUSample >= 0) && filterTime < itBegin->timestamp)
		{ // Have prior filter state to update
			flexkalman::predict(filterState, model, dtS(filterTime, itBegin->timestamp));
		}
		filterTime = itBegin->timestamp;
		lastSample = *itBegin;
		itBegin = std::next(itBegin);
		LOG(LTrackingFilter, LTrace, "Initialising IMU %d integration!", inertial.imu->index);
	}
	else
	{ // Use last IMU sample as reference
		lastSample = *std::prev(itBegin);
		LOG(LTrackingFilter, LTrace, "Continuing IMU %d integration!", inertial.imu->index);
	}

	// Find last IMU sample before specified maximum time (e.g. current frame)
	auto itEnd = itBegin;
	while (itEnd != samples.end() && itEnd->timestamp < time) itEnd++;

	// Integrate range of IMU samples
	for (auto sample = itBegin; sample < itEnd; sample++)
	{
		if (state.lastObsFrame >= 0)
		{
			LOG(LTrackingFilter, LTrace, "    Integrating sample %fms ahead of last observation, %fms ahead of last IMU sample, %fms ahead of state, %fms ahead of fusion!",
				dtMS(state.lastObservation, sample->timestamp),
				dtMS(lastSample.timestamp, sample->timestamp),
				dtMS(filterTime, sample->timestamp),
				dtMS(inertial.fusion.time, sample->timestamp));
		}
		else
		{
			LOG(LTrackingFilter, LTrace, "    Integrating sample %fms ahead of last IMU sample, %fms ahead of state, %fms ahead of fusion!",
				dtMS(lastSample.timestamp, sample->timestamp),
				dtMS(filterTime, sample->timestamp),
				dtMS(inertial.fusion.time, sample->timestamp));
		}
		// Integrate on current IMU filter state
		integrateIMUSample(inertial, *sample, lastSample, params, state.lastObsFrame >= 0);
		lastSample = *sample;
		// Update IMU sample state
		state.lastIMUSample = sample.index();
		state.lastIMUTime = sample->timestamp;

		if (!updateFilter) continue;

		// Predict up until new IMU filter time
		flexkalman::predict(filterState, model, dtS(filterTime, sample->timestamp));
		filterTime = sample->timestamp;
		if (inertial.calibration.phase > IMU_CALIB_EXT_ORIENTATION)
		{ // Correct with updated IMU filter quat
			auto measurement = FusedIMUMeasurement{
				inertial.calibration.quat * inertial.fusion.quat,
				Eigen::Vector3d::Constant(params.filter.stdDevIMU*params.filter.stdDevIMU)
			};
			flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
			if (!flexkalman::correctUnscented(filterState, measurement, true, sigmaParams))
			{
				LOG(LTrackingFilter, LWarn, "Failed to correct pose in IMU filter! Reset!");
			}
		}
		if (inertial.calibration.phase > IMU_CALIB_EXT_OFFSET)
		{ // TODO: Correct with updated IMU filter accel
		}
	}

	// Predict new state
	if (updateFilter)
	{
		Eigen::Quaterniond preQuat = filterState.getCombinedQuaternion();
		flexkalman::predict(filterState, model, dtS(filterTime, time));
		filterTime = time;
		Eigen::Quaterniond dQuat = filterState.getCombinedQuaternion() * preQuat.conjugate();
	
		observation.inertial.translation() = filterState.position().cast<float>();
		observation.inertial.linear() = (dQuat * inertial.fusion.quat).toRotationMatrix().cast<float>();
	}
	else
	{
		observation.inertial.translation() = filterState.position().cast<float>();
		observation.inertial.linear() = inertial.fusion.quat.toRotationMatrix().cast<float>();
	}
	//observation.covIMU = ;

	{ // Actually apply IMU integrations to prediction
		state.state = filterState;
		state.time = filterTime;

		observation.predicted = state.state.getIsometry().cast<float>();
		observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

		observation.filtered = state.state.getIsometry().cast<float>();
		observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
		observation.time = time;
	}

	return true;
}

bool integrateIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	if (inertial.imu->isFused)
		return integrateIMU<IMUSampleFused>(state, inertial, observation, time, params);
	else
		return integrateIMU<IMUSampleRaw>(state, inertial, observation, time, params);
	return true;
}

static void integrateIMUSample(TrackerInertial &inertial, const IMUSampleFused &sample, const IMUSampleFused &lastSample, TimePoint_t filterTime, bool isOptical)
{
	float factor = 0;
	if (lastSample.timestamp < inertial.fusion.time)
	{ // Account for time difference between last IMU sample and last filter time (e.g. optical measurement)
		factor = dtMS(lastSample.timestamp, inertial.fusion.time) / dtMS(lastSample.timestamp, sample.timestamp);
		LOG(LTrackingFilter, LTrace, "        Lerping last sample to current with a factor of %f", factor);
	}
	Eigen::Quaterniond lastQuat = lastSample.quat.slerp(factor, sample.quat).cast<double>();
	Eigen::Quaterniond quat = sample.quat.cast<double>();

	float dt = dtS(inertial.fusion.time, sample.timestamp);
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

	inertial.fusion.accelRaw = sample.accel.cast<double>();
	// Add gravity back in
	inertial.fusion.accelRaw.z() -= 9.81f;
	// Reorient to corrected quat
	inertial.fusion.accel = (inertial.fusion.quat * quat.conjugate()) * inertial.fusion.accelRaw;
	// Remove gravity again
	inertial.fusion.accel.z() += 9.81f;
	// Integrate velocity
	inertial.fusion.imuVelocity += inertial.fusion.accel * dt;
}

static void integrateIMUSample(TrackerInertial &inertial, const IMUSampleRaw &sample, const IMUSampleRaw &lastSample, const TargetTrackingParameters &params, bool isOptical)
{
	float dt = dtS(inertial.fusion.time, sample.timestamp);
	inertial.fusion.time = sample.timestamp;

	Eigen::Vector3d gyro = sample.gyro.cast<double>();
	double angle = gyro.norm(); // In rad/s
	Eigen::Vector3d axis = gyro/angle;

	// TODO: Apply calibration / coordinate system change

	Eigen::Quaterniond dQuat(Eigen::AngleAxisd(angle*dt, axis));

	// Apply local change to global fused quat
	inertial.fusion.quat = inertial.fusion.quat * dQuat;
	//inertial.fusion.dQuat = inertial.fusion.dQuat * dQuat;
	//inertial.fusion.angularVelocity = gyro;

	inertial.fusion.accelRaw = sample.accel.cast<double>();
	// Reorient from local to world space
	inertial.fusion.accel = inertial.fusion.quat * sample.accel.cast<double>();
	// Remove gravity
	inertial.fusion.accel.z() += 9.81f;
	// Integrate velocity
	inertial.fusion.imuVelocity += inertial.fusion.accel * dt;
}

void postCorrectIMU(TrackerState &state, TrackerInertial &inertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	if (inertial.calibration.phase < IMU_CALIB_EXT_DONE)
	{ // In calibration phase

		// Update last state based on observation for calibration
		inertial.calibration.lastState = state.state;
		inertial.calibration.lastTime = state.time;
	}

	// Re-base/correct integration/fusion of fused/raw samples
	LOG(LTrackingIMU, LDebug, "Re-basing IMU Integration!");
	inertial.fusion.time = time;
	inertial.fusion.quat = state.state.getCombinedQuaternion();
	inertial.fusion.positionalVelocity = state.state.velocity();
	inertial.fusion.angularVelocity = state.state.angularVelocity();
	inertial.fusion.tangentialVelocity = inertial.fusion.angularVelocity.cross(inertial.calibration.offset);
	inertial.fusion.imuVelocity = state.state.velocity() + inertial.fusion.tangentialVelocity;
}
