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

#include "target/tracking3D.hpp"
#include "target/kalman.inl"

#include "util/log.hpp"

#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"

/**
 * Tracking a target (set of markers) in 3D space
 */

void FlexUKFFilter::init(Isometry3<Scalar> pose, const TargetTrackingParameters &params)
{
	state.position() = pose.translation().cast<double>();
	state.setQuaternion(Eigen::Quaterniond(pose.rotation().cast<double>()));
	auto &errorCov = state.errorCovariance();
	errorCov.diagonal().segment<3>(0).setConstant(params.uncertaintyPos*params.initialUncertaintyState);
	errorCov.diagonal().segment<3>(3).setConstant(params.uncertaintyRot*params.initialUncertaintyState);
	errorCov.diagonal().segment<3>(6).setConstant(params.uncertaintyPos*params.initialUncertaintyChange);
	errorCov.diagonal().segment<3>(9).setConstant(params.uncertaintyRot*params.initialUncertaintyChange);
}

// -------
// Tracking function
// -------

template<>
bool trackTarget<TrackedTarget<FlexUKFFilter>>(TrackedTarget<FlexUKFFilter> &target, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, int cameraCount, const TargetTrackingParameters &params)
{
	auto &filter = target.filter;
	Eigen::Isometry3f oldObservedPose = filter.poseObserved;
	filter.model.setDamping(params.dampeningPos, params.dampeningRot);

	// Predict new state
	flexkalman::predict(filter.state, filter.model, dtS(filter.time, time));
	filter.posePredicted = filter.state.getIsometry().cast<float>();
	filter.stdDev = filter.state.errorCovariance().template block<3,3>(0,0).diagonal().cwiseSqrt().cast<float>();
	filter.time = time;

	// Calculate uncertainty
	Eigen::Vector3f uncertainty = Eigen::Vector3f::Constant(params.minUncertainty3D).cwiseMax(filter.stdDev*params.uncertaintySigma);
	Eigen::Vector3f factor = Eigen::Vector3f::Constant(1).cwiseQuotient(uncertainty);
	LOG(LTrackingFilter, LDebug, "    Predicted pose with uncertainty %.2fmm, matching target in 2D...\n", uncertainty.mean()*1000);

	// Make sure to allocate memory for any newly added cameras and reset internal data
	target.tracking2DData.init(cameraCount);

	// Match target with points and optimise pose
	target.match2D = trackTarget2D(*target.target, filter.posePredicted, filter.stdDev, calibs, cameraCount, 
		points2D, properties, relevantPoints2D, params, target.tracking2DData);
	if (target.match2D.error.samples < params.minTotalObs) return false;
	LOG(LTracking, LDebug, "    Pixel Error after 2D target track: %fpx mean over %d points\n",
		target.match2D.error.mean*PixelFactor, target.match2D.error.samples);

	// TODO: Get variance/covariance from optimisation via jacobian
	// Essentially, any parameter that barely influences output error has a high variance
	// Neatly encapsulates the degrees of freedom (and their strength) left by varying distribution of blobs

	Eigen::Matrix<double,6,6> covariance;
	covariance.setIdentity();
	covariance.diagonal().head<3>().setConstant(params.uncertaintyPos*params.uncertaintyPos);
	covariance.diagonal().tail<3>().setConstant(params.uncertaintyRot*params.uncertaintyRot);

	// Update filter
	filter.opticalMeasurements++;
	filter.poseObserved = target.match2D.pose;
	auto measurement = AbsolutePoseMeasurement(
		target.match2D.pose.translation().cast<double>(),
		Eigen::Quaterniond(target.match2D.pose.rotation().cast<double>()),
		covariance.cast<double>()
	);
	flexkalman::SigmaPointParameters sigmaParams(params.sigmaAlpha, params.sigmaBeta, params.sigmaKappa);
	if (!flexkalman::correctUnscented(filter.state, measurement, true, sigmaParams))
	{
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
	}
	filter.poseFiltered = filter.state.getIsometry().cast<float>();

	// Calculate error probability
	Eigen::Vector3f compErrorProbability = (filter.poseObserved.translation()-filter.posePredicted.translation()).cwiseAbs().cwiseProduct(factor);
	float errorProbability = 1-(Eigen::Vector3f::Constant(1)-compErrorProbability).prod();	

	LOG(LTrackingFilter, LDebug, "    Filter error %.2f%%! Obs change %.3fmm, %.3fdg; Pred diff %.3fmm, %.3fdg, Filter diff %.3fmm, %.3fdg!\n", 
		100*errorProbability, 
		(filter.poseObserved.translation() - oldObservedPose.translation()).norm()*1000, 
		Eigen::AngleAxisf(filter.poseObserved.rotation() * oldObservedPose.rotation().transpose()).angle()/PI*180, 
		(filter.poseObserved.translation() - filter.posePredicted.translation()).norm()*1000, 
		Eigen::AngleAxisf(filter.poseObserved.rotation() * filter.posePredicted.rotation().transpose()).angle()/PI*180,
		(filter.poseObserved.translation() - filter.poseFiltered.translation()).norm()*1000, 
		Eigen::AngleAxisf(filter.poseObserved.rotation() * filter.poseFiltered.rotation().transpose()).angle()/PI*180);

	return target.match2D.error.samples >= params.minTotalObs && target.match2D.error.mean < params.maxTotalError;
}

template bool trackTarget<TrackedTargetFiltered>(TrackedTargetFiltered &target, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, int cameraCount, const TargetTrackingParameters &params);

template<typename Target>
bool integrateIMU(Target &target, TimePoint_t time,
	const TargetTrackingParameters &params)
{
	using Scalar = typename Target::Scalar;

	if (!target.imu) return false;

	auto &filter = target.filter;
	filter.model.setDamping(params.dampeningPos, params.dampeningRot);

	TimePoint_t curTime = filter.time;

	// Find first IMU sample after last filter time (e.g. last frame)
	auto samples = target.imu->samples.template getView<true>();
	auto itBegin = samples.begin();
	if (target.lastIMUSample >= samples.beginIndex() && target.lastIMUSample < samples.endIndex())
	{ // Start search from last sample used
		itBegin = samples.pos(target.lastIMUSample);
		while (itBegin != samples.end() && itBegin->timestamp < curTime) itBegin++;
	}
	else // Cold start
		itBegin = std::lower_bound(samples.begin(), samples.end(), curTime);
	if (itBegin == samples.end()) return false;

	// Pick reference sample (last or first)
	Eigen::Quaternionf lastQuat;
	if (itBegin == samples.begin() || dtMS(std::prev(itBegin)->timestamp, itBegin->timestamp) > 50)
	{ // No prior reference, use first IMU sample as reference
		if (filter.opticalMeasurements == 0 && filter.inertialMeasurements == 0)
		{ // Initialise with first IMU orientation
			filter.state.setQuaternion(itBegin->quat.template cast<double>());
		}
		else
		{ // Skip to first IMU sample as reference to correct from
			flexkalman::predict(filter.state, filter.model, dtS(curTime, itBegin->timestamp));
		}
		lastQuat = itBegin->quat;
		curTime = itBegin->timestamp;
		itBegin = std::next(itBegin);
	}
	else
	{ // Use last IMU sample as reference
		auto itLast = std::prev(itBegin);
		// Account for time difference between last IMU record and last optical measurement (which is curTime / filter time)
		float factor = dtMS(itLast->timestamp, curTime) / dtMS(itLast->timestamp, itBegin->timestamp);
		lastQuat = itLast->quat.slerp(factor, itBegin->quat);
	}

	// Find last IMU sample before specified maximum time (e.g. current frame)
	auto itEnd = itBegin;
	while (itEnd != samples.end() && itEnd->timestamp < time) itEnd++;

	// Integrate range of IMU samples
	for (auto sample = itBegin; sample < itEnd; sample++)
	{
		Eigen::Quaterniond quat = sample->quat.template cast<double>();
		if (filter.opticalMeasurements > 0)
		{ // Determine new quat based on last reference (here basic difference, all absolute information is disregarded)
			Eigen::Quaternionf dQuat = sample->quat * lastQuat.conjugate();
			quat = dQuat.cast<double>() * filter.state.getCombinedQuaternion();
			lastQuat = sample->quat;
			// TODO: Use IMU quat around last optical measurement as measurement to smoothly switch between absolute and relative tracking
		}
		// Predict up until IMU sample timestamp
		flexkalman::predict(filter.state, filter.model, dtS(curTime, sample->timestamp));
		curTime = sample->timestamp;
		// Correct with IMU sample
		filter.inertialMeasurements++;
		auto measurement = FusedIMUMeasurement{
			quat,
			Eigen::Vector3d::Constant(params.uncertaintyRot*params.uncertaintyRot)
		};
		flexkalman::SigmaPointParameters sigmaParams(params.sigmaAlpha, params.sigmaBeta, params.sigmaKappa);
		if (!flexkalman::correctUnscented(filter.state, measurement, true, sigmaParams))
		{
			LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
		}
	}

	// Predict new state
	flexkalman::predict(filter.state, filter.model, dtS(curTime, time));
	filter.posePredicted = filter.poseObserved = filter.poseFiltered = filter.state.getIsometry().template cast<float>();
	filter.stdDev = filter.state.errorCovariance().template block<3,3>(0,0).diagonal().cwiseSqrt().template cast<float>();
	filter.time = time;

	return true;
}

template bool integrateIMU<TrackedTargetFiltered>(TrackedTargetFiltered &target,
	TimePoint_t time, const TargetTrackingParameters &params);
template bool integrateIMU<TrackedIMUFiltered>(TrackedIMUFiltered &target,
	TimePoint_t time, const TargetTrackingParameters &params);