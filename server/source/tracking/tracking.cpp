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

	// Predict new state
	flexkalman::predict(state.state, model, dtS(state.time, time));
	state.time = time;
	observation.predicted = state.state.getIsometry().cast<float>();
	observation.covPredicted = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();
	Eigen::Vector3f stdDev = observation.covPredicted.diagonal().head<3>().cwiseSqrt();

	// Make sure to allocate memory for any newly added cameras and reset internal data
	target.data.init(cameraCount);

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
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
	}
	state.lastObservation = time;
	state.lastObsFrame = frame;

	observation.filtered = state.state.getIsometry().cast<float>();
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

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
	observation.covObserved = params.filter.getCovariance<float>();

	// Update state
	auto measurement = AbsolutePositionMeasurement(
		points3D[matchedPoint].cast<double>(),
		observation.covObserved.diagonal().head<3>().cast<double>().eval());
	flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
	if (!flexkalman::correctUnscented(state.state, measurement, true, sigmaParams))
	{
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
	}

	state.lastObservation = time;
	state.lastObsFrame++; // TODO: Switch to frame once used

	observation.filtered = state.state.getIsometry().cast<float>() ;
	observation.covFiltered = state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	return matchedPoint;
}

bool integrateIMU(TrackerState &state, const TrackerIMU &imu, TrackerObservation &observation,
	TimePoint_t time, const TargetTrackingParameters &params)
{
	typename TrackerState::Model model(params.filter.dampeningPos, params.filter.dampeningRot);

	// Find first IMU sample after last state time (e.g. last frame)
	auto samples = imu->samples.getView<true>();
	auto itBegin = samples.begin();
	if (state.lastIMUSample >= samples.beginIndex() && state.lastIMUSample < samples.endIndex())
	{ // Start search from last sample used
		itBegin = samples.pos(state.lastIMUSample);
		while (itBegin != samples.end() && itBegin->timestamp < state.time) itBegin++;
	}
	else // Cold start
		itBegin = std::lower_bound(samples.begin(), samples.end(), state.time);
	if (itBegin == samples.end()) return false;

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
	}
	else
	{ // Use last IMU sample as reference
		auto itLast = std::prev(itBegin);
		// Account for time difference between last IMU record and last optical measurement (which is state.time)
		float factor = dtMS(itLast->timestamp, state.time) / dtMS(itLast->timestamp, itBegin->timestamp);
		lastQuat = itLast->quat.slerp(factor, itBegin->quat);
	}

	// Find last IMU sample before specified maximum time (e.g. current frame)
	auto itEnd = itBegin;
	while (itEnd != samples.end() && itEnd->timestamp < time) itEnd++;

	// Integrate range of IMU samples
	for (auto sample = itBegin; sample < itEnd; sample++)
	{
		Eigen::Quaterniond quat = sample->quat.cast<double>();
		if (state.lastObsFrame >= 0)
		{ // Determine new quat based on last reference (here basic difference, all absolute information is disregarded)
			Eigen::Quaternionf dQuat = sample->quat * lastQuat.conjugate();
			quat = dQuat.cast<double>() * state.state.getCombinedQuaternion();
			lastQuat = sample->quat;
			// TODO: Use IMU quat around last optical measurement as measurement to smoothly switch between absolute and relative tracking
		}
		// Predict up until IMU sample timestamp
		flexkalman::predict(state.state, model, dtS(state.time, sample->timestamp));
		state.time = sample->timestamp;
		// Correct with IMU sample
		state.lastIMUSample = sample.index();
		auto measurement = FusedIMUMeasurement{
			quat,
			Eigen::Vector3d::Constant(params.filter.stdDevEXP*params.filter.stdDevEXP)
		};
		flexkalman::SigmaPointParameters sigmaParams(params.filter.sigmaAlpha, params.filter.sigmaBeta, params.filter.sigmaKappa);
		if (!flexkalman::correctUnscented(state.state, measurement, true, sigmaParams))
		{
			LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
		}
	}

	// Predict new state
	flexkalman::predict(state.state, model, dtS(state.time, time));
	state.time = time;

	observation.predicted = observation.observed = observation.filtered = state.state.getIsometry().cast<float>();
	observation.covPredicted = observation.covObserved = observation.covFiltered = 
		state.state.errorCovariance().topLeftCorner<6,6>().cast<float>();

	return true;
}