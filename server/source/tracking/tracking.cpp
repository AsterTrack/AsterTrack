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

bool integrateIMU(TrackerState &state, const TrackerInertial &intertial, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	typename TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	// Find first IMU sample after last state time (e.g. last frame)
	auto samples = intertial.imu->samples.getView<true>();
	auto itBegin = samples.begin();
	if (state.lastIMUSample >= samples.beginIndex() && state.lastIMUSample < samples.endIndex())
	{ // Start search from last sample used
		itBegin = samples.pos(state.lastIMUSample+1);
		while (itBegin != samples.end() && itBegin->timestamp < state.time) itBegin++;
	}
	else // Cold start
		itBegin = std::lower_bound(samples.begin(), samples.end(), state.time);
	if (itBegin == samples.end()) return false;

	{ // Extrapolate without IMU samples for debug purposes
		auto filterState = state.state;
		flexkalman::predict(filterState, model, dtS(state.time, time));
		observation.extrapolated = filterState.getIsometry().cast<float>();
	}

	// Pick reference sample (last or first)
	Eigen::Quaternionf lastQuat;
	if (itBegin == samples.begin() || dtMS(std::prev(itBegin)->timestamp, itBegin->timestamp) > 50)
	{ // No prior reference, use first IMU sample as reference
		if (state.lastObsFrame < 0 && state.lastIMUSample < 0)
		{ // Initialise with first IMU orientation
			state.state.setQuaternion(itBegin->quat.cast<double>());
		}
		else
		{ // Skip to first IMU sample as reference to correct from
			flexkalman::predict(state.state, model, dtS(state.time, itBegin->timestamp));
		}
		lastQuat = itBegin->quat;
		state.time = itBegin->timestamp;
		itBegin = std::next(itBegin);
		LOG(LTrackingFilter, LDebug, "Starting IMU integration with raw initialisation!");
	}
	else
	{ // Use last IMU sample as reference
		auto itLast = std::prev(itBegin);
		// Account for time difference between last IMU record and last optical measurement (which is state.time)
		float factor = dtMS(itLast->timestamp, state.time) / dtMS(itLast->timestamp, itBegin->timestamp);
		lastQuat = itLast->quat.slerp(factor, itBegin->quat);
		LOG(LTrackingFilter, LDebug, "Starting IMU integration with lerp from last sample of %f", factor);
	}

	Eigen::Quaterniond baseQuat = state.state.getCombinedQuaternion();

	// Find last IMU sample before specified maximum time (e.g. current frame)
	auto itEnd = itBegin;
	while (itEnd != samples.end() && itEnd->timestamp < time) itEnd++;

	// Integrate range of IMU samples
	for (auto sample = itBegin; sample < itEnd; sample++)
	{
		Eigen::Quaterniond quat = sample->quat.cast<double>();
		/* if (state.lastObsFrame >= 0 && !imu->hasMag)
		{ // Make Z-axis change relative if there is no mag to compensate for drift
			auto getZ = [](Eigen::Quaterniond q)
			{ // Both are the same
				//return q.toRotationMatrix().canonicalEulerAngles(2, 1, 0).x();
				return std::atan2(2*q.x()*q.y() + 2*q.w()*q.z(), 1 - (2*q.y()*q.y() + 2*q.z()*q.z()));
			};
			double eZ = getZ(baseQuat) - getZ(lastQuat.cast<double>());
			quat = Eigen::AngleAxisd(eZ, Eigen::Vector3d::UnitZ()) * quat;
			LOG(LTrackingFilter, LDebug, "    Adjusted Z axis to be relative!");
		} */
		// TODO: Absolute rotation requires calibration of IMU in relation to target
		// else
		if (state.lastObsFrame >= 0)
		{ // Make all axis change relative to compensate for drift
			quat = baseQuat * quat * lastQuat.cast<double>().conjugate();
			LOG(LTrackingFilter, LDebug, "    Adjusted all axis to be relative!");
		}
		lastQuat = sample->quat;
		baseQuat = quat;
		if (state.lastObsFrame >= 0)
		{
			LOG(LTrackingFilter, LDebug, "    Integrating sample %fms ahead of last observation, dT to last state update %fms, last IMU sample %fms!",
				dtMS(state.lastObservation, sample->timestamp),
				dtMS(state.time, sample->timestamp), dtMS(state.lastIMUTime, sample->timestamp));
		}
		else
		{
			LOG(LTrackingFilter, LDebug, "    Integrating sample with dT to last state update %fms, last IMU sample %fms!",
				dtMS(state.time, sample->timestamp), dtMS(state.lastIMUTime, sample->timestamp));
		}
		// Predict up until IMU sample timestamp
		flexkalman::predict(state.state, model, dtS(state.time, sample->timestamp));
		state.time = sample->timestamp;
		// Correct with IMU sample
		state.lastIMUSample = sample.index();
		state.lastIMUTime = sample->timestamp;
		auto measurement = FusedIMUMeasurement{
			quat,
			Eigen::Vector3d::Constant(params.filter.stdDevIMU*params.filter.stdDevIMU)
		};
		flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
		if (!flexkalman::correctUnscented(state.state, measurement, true, sigmaParams))
		{
			LOG(LTrackingFilter, LWarn, "Failed to correct pose in IMU filter! Reset!");
		}
	}

	// Predict new state
	Eigen::Quaterniond preQuat = state.state.getCombinedQuaternion();
	flexkalman::predict(state.state, model, dtS(state.time, time));
	state.time = time;
	Eigen::Quaterniond dQuat = state.state.getCombinedQuaternion() * preQuat.conjugate();

	observation.predicted = state.state.getIsometry().cast<float>();
	observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	observation.imu.translation() = state.state.position().cast<float>();
	observation.imu.linear() = (dQuat * baseQuat).toRotationMatrix().cast<float>();
	//observation.covIMU = ;

	observation.filtered = state.state.getIsometry().cast<float>();
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
	observation.time = time;

	return true;
}