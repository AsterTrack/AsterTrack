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
	filter.measurements++;
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
