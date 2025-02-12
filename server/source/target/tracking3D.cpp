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

#include "util/flexUKF.hpp"
#include "util/log.hpp"

#include "flexkalman/measurement/AbsoluteOrientationMeasurement.h"
#include "flexkalman/measurement/AbsolutePositionMeasurement.h"
#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"

/**
 * Tracking a target (set of markers) in 3D space
 */

// -------
// Position predicted manually (velocity model), rotation predicted manually (velocity model)
// -------

template<> void TrackingFilter<Filter_Man_PV_Man_RV<float>>::init(Isometry3<Scalar> pose, const TargetTrackingParameters &params)
{
	measurements = 1;
	poseObserved = pose;
	curStatePred.setZero();
	curStatePred.pos() = pose.translation();
	curStatePred.setQuatR(Eigen::Quaternionf(pose.rotation()));
	curStatePred.setQuatV(Eigen::Quaternionf::Identity());
}
template<> Isometry3<float> TrackingFilter<Filter_Man_PV_Man_RV<float>>::predict(TimeStep timeStep)
{
	if (measurements == 1)
	{
		predStatePred = curStatePred;
		stdDev = Eigen::Vector3f::Constant(0.1f);
	}
	else
	{
		predStatePred.vel() = curStatePred.vel();
		predStatePred.pos() = curStatePred.pos() + curStatePred.vel() * timeStep;
		predStatePred.quatV() = curStatePred.quatV();
		predStatePred.setQuatR(curStatePred.getQuatR() * Eigen::Quaternionf::Identity().slerp(timeStep, curStatePred.getQuatV()));
		stdDev = predStatePred.vel()*2 + Eigen::Vector3f::Constant(0.05f);
	}
	posePredicted.translation() = predStatePred.pos();
	posePredicted.linear() = predStatePred.getQuatR().toRotationMatrix();
	return posePredicted;
}
template<> void TrackingFilter<Filter_Man_PV_Man_RV<float>>::update(TimeStep timeStep, Isometry3<Scalar> newPose)
{
	// Update manual position
	curStatePred.vel() = (newPose.translation() - curStatePred.pos()) / timeStep;
	curStatePred.pos() = newPose.translation();
	// Update manual rotation
	Eigen::Quaternionf newQuat(newPose.rotation());
	curStatePred.setQuatV(Eigen::Quaternionf::Identity().slerp(1.0f/timeStep, newQuat * curStatePred.getQuatR().inverse()));
	curStatePred.setQuatR(newQuat);
	// Update poses
	poseFiltered.translation() = curStatePred.pos();
	poseFiltered.linear() = newPose.rotation();
	poseObserved = newPose;
}
template<> void TrackingFilter<Filter_Man_PV_Man_RV<float>>::updateError(TimeStep timeStep, Scalar sigma)
{}
template<> void TrackingFilter<Filter_Man_PV_Man_RV<float>>::debugState(StateFilter &stateFilter, StatePred &statePred, std::string label, TimeStep timeStep)
{
	LOG(LTrackingFilter, LDebug, "%s: %.3fmm/f -- %.3fdg/f\n", label.c_str(), statePred.vel().norm()*1000*timeStep, Eigen::AngleAxisf(statePred.getQuatV()).angle()*timeStep/PI*180);
}


// -------
// Position filtered with EKF (velocity model), rotation predicted manually (velocity model)
// -------

template<> void TrackingFilter<Filter_EKF_PV_Man_RV<float>>::init(Isometry3<Scalar> pose, const TargetTrackingParameters &params)
{
	measurements = 1;
	poseObserved = pose;
	curStateFilter.setZero();
	curStateFilter.pos() = pose.translation();
	curStatePred.setZero();
	curStatePred.setQuatR(Eigen::Quaternionf(pose.rotation()));
	curStatePred.setQuatV(Eigen::Quaternionf::Identity());
	measurementModel.setExpectedError(errorScaleT);
}
template<> Isometry3<float> TrackingFilter<Filter_EKF_PV_Man_RV<float>>::predict(TimeStep timeStep)
{
	if (measurements == 1)
	{
		predStateFilter = curStateFilter;
		predStatePred = curStatePred;
		stdDev = Eigen::Vector3f::Constant(0.1f);
	}
	else if (measurements == 2)
	{
		predStateFilter = movementModel.f(curStateFilter, timeStep);
		stdDev = curStateFilter.vel()*2 + Eigen::Vector3f::Constant(0.05f);
		predStatePred.quatV() = curStatePred.quatV();
		predStatePred.setQuatR(curStatePred.getQuatR() * Eigen::Quaternionf::Identity().slerp(timeStep.dt(), curStatePred.getQuatV()));
	}
	else
	{
		predStateFilter = filter.predict(movementModel, timeStep);
		stdDev = filter.getCovariance().template block<3,3>(0,0).diagonal().cwiseSqrt();
		predStatePred.quatV() = curStatePred.quatV();
		predStatePred.setQuatR(curStatePred.getQuatR() * Eigen::Quaternionf::Identity().slerp(timeStep.dt(), curStatePred.getQuatV()));
	}
	posePredicted.translation() = predStateFilter.pos();
	posePredicted.linear() = predStatePred.getQuatR().toRotationMatrix();
	return posePredicted;
}
template<> void TrackingFilter<Filter_EKF_PV_Man_RV<float>>::update(TimeStep timeStep, Isometry3<Scalar> newPose)
{
	// Update filtered position
	if (measurements == 2)
	{
		curStateFilter.vel() = (newPose.translation() - poseObserved.translation()) / timeStep.dt();
		curStateFilter.pos() = newPose.translation();
		filter.init(curStateFilter);
	}
	else if (measurements > 2)
	{ // Create measurement and update filter
		curStateFilter.pos() = newPose.translation();
		auto measurement = measurementModel.h(curStateFilter);
		curStateFilter = filter.update(measurementModel, measurement);
	}
	// Update manual rotation
	Eigen::Quaternionf newQuat(newPose.rotation());
	curStatePred.setQuatV(Eigen::Quaternionf::Identity().slerp(1.0f/timeStep.dt(), newQuat * curStatePred.getQuatR().inverse()));
	curStatePred.setQuatR(newQuat);
	// Update poses
	poseFiltered.translation() = curStateFilter.pos();
	poseFiltered.linear() = newPose.rotation();
	poseObserved = newPose;
}
template<> void TrackingFilter<Filter_EKF_PV_Man_RV<float>>::updateError(TimeStep timeStep, Scalar sigma)
{
	float prevErrorScaleT = errorScaleT;
	// Adjust error scale so target confidence is reached
	const float errorScaleLerp = 0.1f;
	const float errorProbTarget = 0.1f; // Expecting 10% error probabilty
	float changeFac = std::min(1.0f, errorProbability)/errorProbTarget;
	errorScaleT = errorScaleLerp*errorScaleT + (1-errorScaleLerp)*errorScaleT * changeFac;
	movementModel.setExpectedError(errorScaleT);

	LOG(LTrackingFilter, LDebug, "    Error Scale: Previous: %.6f, New: %.6f, Desired Factor: %.6f due to error probability %.2f%%\n",
		prevErrorScaleT, errorScaleT, changeFac, errorProbability*100);

}
template<> void TrackingFilter<Filter_EKF_PV_Man_RV<float>>::debugState(StateFilter &stateFilter, StatePred &statePred, std::string label, TimeStep timeStep)
{
	LOG(LTrackingFilter, LDebug, "%s: %.3fmm/f -- %.3fdg/f\n", label.c_str(),
		stateFilter.vel().norm()*1000*timeStep.dt(),
		Eigen::AngleAxisf(statePred.getQuatV()).angle()*timeStep.dt()/PI*180);
}

// -------
// Position filtered with EKF (acceleration model), rotation predicted manually (velocity model)
// -------

template<> void TrackingFilter<Filter_EKF_PA_Man_RV<float>>::init(Isometry3<Scalar> pose, const TargetTrackingParameters &params)
{
	measurements = 1;
	poseObserved = pose;
	curStateFilter.setZero();
	curStateFilter.pos() = pose.translation();
	curStatePred.setZero();
	curStatePred.setQuatR(Eigen::Quaternionf(pose.rotation()));
	curStatePred.setQuatV(Eigen::Quaternionf::Identity());
	measurementModel.setExpectedError(errorScaleT);
}
template<> Isometry3<float> TrackingFilter<Filter_EKF_PA_Man_RV<float>>::predict(TimeStep timeStep)
{
	if (measurements == 1)
	{
		predStateFilter = curStateFilter;
		predStatePred = curStatePred;
		stdDev = Eigen::Vector3f::Constant(0.1f);
	}
	else if (measurements == 2)
	{
		predStateFilter = movementModel.f(curStateFilter, timeStep);
		stdDev = curStateFilter.vel()*2 + Eigen::Vector3f::Constant(0.05f);
		predStatePred.quatV() = curStatePred.quatV();
		predStatePred.setQuatR(curStatePred.getQuatR() * Eigen::Quaternionf::Identity().slerp(timeStep.dt(), curStatePred.getQuatV()));
	}
	else
	{
		predStateFilter = filter.predict(movementModel, timeStep);
		stdDev = filter.getCovariance().template block<3,3>(0,0).diagonal().cwiseSqrt();
		predStatePred.quatV() = curStatePred.quatV();
		predStatePred.setQuatR(curStatePred.getQuatR() * Eigen::Quaternionf::Identity().slerp(timeStep.dt(), curStatePred.getQuatV()));
	}
	posePredicted.translation() = predStateFilter.pos();
	posePredicted.linear() = predStatePred.getQuatR().toRotationMatrix();
	return posePredicted;
}
template<> void TrackingFilter<Filter_EKF_PA_Man_RV<float>>::update(TimeStep timeStep, Isometry3<Scalar> newPose)
{
	// Update filtered position
	if (measurements == 2)
	{
		curStateFilter.vel() = (newPose.translation() - poseObserved.translation()) / timeStep.dt();
		curStateFilter.pos() = newPose.translation();
	}
	else if (measurements == 3)
	{
		Eigen::Matrix<Scalar,3,1> vel = (newPose.translation() - poseObserved.translation()) / timeStep.dt();
		curStateFilter.acc() = (vel - curStateFilter.vel()) / timeStep.dt();
		curStateFilter.vel() = vel;
		curStateFilter.pos() = newPose.translation();
		filter.init(curStateFilter);
	}
	else if (measurements > 3)
	{ // Create measurement and update filter
		curStateFilter.pos() = newPose.translation();
		auto measurement = measurementModel.h(curStateFilter);
		curStateFilter = filter.update(measurementModel, measurement);
	}
	// Update manual rotation
	Eigen::Quaternionf newQuat(newPose.rotation());
	curStatePred.setQuatV(Eigen::Quaternionf::Identity().slerp(1.0f/timeStep.dt(), newQuat * curStatePred.getQuatR().inverse()));
	curStatePred.setQuatR(newQuat);
	// Update poses
	poseFiltered.translation() = curStateFilter.pos();
	poseFiltered.linear() = newPose.rotation();
	poseObserved = newPose;
}
template<> void TrackingFilter<Filter_EKF_PA_Man_RV<float>>::updateError(TimeStep timeStep, Scalar sigma)
{
	float prevErrorScaleT = errorScaleT;
	// Adjust error scale so target confidence is reached
	const float errorScaleLerp = 0.1f;
	const float errorProbTarget = 0.1f; // Expecting 10% error probabilty
	float changeFac = std::min(1.0f, errorProbability)/errorProbTarget;
	errorScaleT = errorScaleLerp*errorScaleT + (1-errorScaleLerp)*errorScaleT * changeFac;
	movementModel.setExpectedError(errorScaleT);

	LOG(LTrackingFilter, LDebug, "    Error Scale: Previous: %.6f, New: %.6f, Desired Factor: %.6f due to error probability %.2f%%\n",
		prevErrorScaleT, errorScaleT, changeFac, errorProbability*100);

}
template<> void TrackingFilter<Filter_EKF_PA_Man_RV<float>>::debugState(StateFilter &stateFilter, StatePred &statePred, std::string label, TimeStep timeStep)
{
	LOG(LTrackingFilter, LDebug, "%s: %.3fmm/f, %.3fmm/f^2 -- %.3fdg/f\n", label.c_str(),
		stateFilter.vel().norm()*1000*timeStep.dt(), stateFilter.acc().norm()*1000*timeStep.dt()*timeStep.dt(),
		Eigen::AngleAxisf(statePred.getQuatV()).angle()*timeStep.dt()/PI*180);
}


// -------
// Position filtered with EKF (acceleration model), rotation filtered with EKF (velocity model)
// -------

// TODO: Fix the EKF filter for tracker position and rotation
// Been a while since this code was tested, but it didn't work well before
template<> void TrackingFilter<Filter_EKF_PA_EKF_RV<float>>::init(Isometry3<Scalar> pose, const TargetTrackingParameters &params)
{
	measurements = 1;
	poseObserved = pose;
	curStateFilter.setZero();
	curStateFilter.pos() = pose.translation();
	curStateFilter.referenceQuat = Eigen::Quaternionf(pose.rotation());
	movementModel.setExpectedError(errorScaleT, errorScaleR);
	// Remove
	curStatePred.setZero();
	curStatePred.setQuatR(Eigen::Quaternionf(pose.rotation()));
	curStatePred.setQuatV(Eigen::Quaternionf::Identity());
}
template<> Isometry3<float> TrackingFilter<Filter_EKF_PA_EKF_RV<float>>::predict(TimeStep timeStep)
{
	if (measurements == 1)
	{
		predStateFilter = curStateFilter;
		stdDev = Eigen::Vector3f::Constant(0.1f);
		// Remove
		predStatePred = curStatePred;

	}
	else if (measurements == 2)
	{
		predStateFilter = movementModel.f(curStateFilter, timeStep);
		stdDev = curStateFilter.vel()*2 + Eigen::Vector3f::Constant(0.05f);
		// Remove
		predStatePred.quatV() = curStatePred.quatV();
		predStatePred.setQuatR(curStatePred.getQuatR() * Eigen::Quaternionf::Identity().slerp(timeStep.dt(), curStatePred.getQuatV()));
	}
	else
	{
		predStateFilter = filter.predict(movementModel, timeStep);
		stdDev = filter.getCovariance().template block<3,3>(0,0).diagonal().cwiseSqrt();
		// Remove
		predStatePred.quatV() = curStatePred.quatV();
		predStatePred.setQuatR(curStatePred.getQuatR() * Eigen::Quaternionf::Identity().slerp(timeStep.dt(), curStatePred.getQuatV()));
	}
	posePredicted.translation() = predStateFilter.pos();
	posePredicted.linear() = predStateFilter.quat().toRotationMatrix();
	return posePredicted;
}
template<> void TrackingFilter<Filter_EKF_PA_EKF_RV<float>>::update(TimeStep timeStep, Isometry3<Scalar> newPose)
{
	// Update filtered position
	if (measurements == 2)
	{
		// Position
		curStateFilter.vel() = (newPose.translation() - poseObserved.translation()) / timeStep.dt();
		curStateFilter.pos() = newPose.translation();
		// Rotation
		Eigen::Quaternion<Scalar> newQuat(newPose.rotation()), oldQuat(poseObserved.rotation());
		Eigen::Quaternion<Scalar> changeQuat = newQuat * oldQuat.inverse();
		//Eigen::Quaternion<Scalar> changeQuat(newPose.rotation() * pose.rotation().transpose());
		curStateFilter.setRVelQuat(Eigen::Quaternionf::Identity().slerp(1.0f/timeStep.dt(), changeQuat));
		curStateFilter.referenceQuat = newQuat;
	}
	else if (measurements == 3)
	{
		// Position
		Eigen::Matrix<Scalar,3,1> vel = (newPose.translation() - poseObserved.translation()) / timeStep.dt();
		curStateFilter.acc() = (vel - curStateFilter.vel()) / timeStep.dt();
		curStateFilter.vel() = vel;
		curStateFilter.pos() = newPose.translation();
		// Rotation
		Eigen::Quaternion<Scalar> newQuat(newPose.rotation()), oldQuat(poseObserved.rotation());
		Eigen::Quaternion<Scalar> changeQuat = newQuat * oldQuat.inverse();
		//Eigen::Quaternion<Scalar> changeQuat(newPose.rotation() * pose.rotation().transpose());
		// Rotational acceleration? Dubious if necessary, just a manual predictor anyway
		//Eigen::Quaternion<Scalar> accelQuat = changeQuat * curStateFilter.getRVelQuat().inverse();
		//curStateFilter.setRAccQuat(Eigen::Quaternionf::Identity().slerp(1.0f/timeStep.dt(), accelQuat));
		curStateFilter.setRVelQuat(Eigen::Quaternionf::Identity().slerp(1.0f/timeStep.dt(), changeQuat));
		curStateFilter.referenceQuat = newQuat;
		filter.init(curStateFilter);
	}
	else if (measurements > 3)
	{ // Update filter
		// Create measurement
		curStateFilter = filter.getState(); // Make sure they use the same referenceQuat
		curStateFilter.pos() = newPose.translation();
		Eigen::Quaternion<Scalar> newQuat(newPose.rotation());
		curStateFilter.setRDelQuat(newQuat * curStateFilter.referenceQuat.inverse()); // Set rotational measurement (as delta from referenceQuat)
		auto measurement = measurementModel.h(curStateFilter);
		// Update filter
		filter.update(measurementModel, measurement);
		// Filter State: Reset operation that updates referenceQuat to current filtered state
		filter.getState().reset();
		curStateFilter = filter.getState();
	}

	// Remove: Update manual rotation
	Eigen::Quaternionf newQuat(newPose.rotation());
	curStatePred.setQuatV(Eigen::Quaternionf::Identity().slerp(1.0f/timeStep.dt(), newQuat * curStatePred.getQuatR().inverse()));
	curStatePred.setQuatR(newQuat);
	// Update poses
	poseFiltered.translation() = curStateFilter.pos();
	poseFiltered.linear() = curStateFilter.quat().toRotationMatrix();
	poseObserved = newPose;
}
template<> void TrackingFilter<Filter_EKF_PA_EKF_RV<float>>::updateError(TimeStep timeStep, Scalar sigma)
{
	float prevErrorScaleT = errorScaleT;
	// Adjust error scale so target confidence is reached
	const float errorScaleLerp = 0.1f;
	const float errorProbTarget = 0.1f; // Expecting 10% error probabilty
	float changeFac = std::min(1.0f, errorProbability)/errorProbTarget;
	errorScaleT = errorScaleLerp*errorScaleT + (1-errorScaleLerp)*errorScaleT * changeFac;

	LOG(LTrackingFilter, LDebug, "    Error Scale: Previous: %.6f, New: %.6f, Desired Factor: %.6f due to error probability %.2f%%\n",
		prevErrorScaleT, errorScaleT, changeFac, errorProbability*100);

	// Predicted Delta
	Eigen::Quaternionf predictedDelta = predStateFilter.getRDelQuat();
	float predictedAngle = Eigen::AngleAxisf(predictedDelta).angle()/(float)PI*180.0f;
	
	// Observed Delta
	//Eigen::Quaternionf obsDelta = curStateFilter.quat() * prevState.quat().inverse();
	//float obsAngle = Eigen::AngleAxisf(obsDelta).angle()/(float)PI*180.0f;

	// Prediction Error
	Eigen::Quaternionf predError = predStateFilter.quat() * curStateFilter.quat().inverse();
	float predErrorAngle = Eigen::AngleAxisf(predError).angle()/(float)PI*180.0f;

	// Filter Error
	Eigen::Quaternionf filterError = filter.getState().quat() * curStateFilter.quat().inverse();
	float filterErrorAngle = Eigen::AngleAxisf(filterError).angle()/(float)PI*180.0f;


	/*Eigen::Vector3f rotStdDevPrev = prevCov.template block<3,3>(9,9).diagonal().cwiseSqrt();
	//rotStdDevPrev = Eigen::Vector3f::Constant(0.0000001f).cwiseMax(rotStdDevPrev);
	float angleStdDevPrev = Eigen::AngleAxisf(MRP2Quat(rotStdDevPrev)).angle()/(float)PI*180.0f;
	float rotUncertaintyPrev = std::max(0.0000001f, angleStdDevPrev*sigma);

	Eigen::Vector3f rotStdDevPred = predCov.template block<3,3>(9,9).diagonal().cwiseSqrt();
	//rotStdDevPred = Eigen::Vector3f::Constant(0.0000001f).cwiseMax(rotStdDevPred);
	float angleStdDevPred = Eigen::AngleAxisf(MRP2Quat(rotStdDevPred)).angle()/(float)PI*180.0f;
	float rotUncertaintyPred = std::max(0.0000001f, angleStdDevPred*sigma);*/

	Eigen::Vector3f rotStdDev = filter.getCovariance().template block<3,3>(9,9).diagonal().cwiseSqrt();
	//rotStdDev = Eigen::Vector3f::Constant(0.0000001f).cwiseMax(rotStdDev);
	float angleStdDev = Eigen::AngleAxisf(MRP2Quat(rotStdDev)).angle()/(float)PI*180.0f;
	float rotUncertainty = std::max(0.0000001f, angleStdDev*sigma);

	float prevErrorScaleR = errorScaleR;

	const float errorScaleRLerp = 0.1f;
	float rotErrorFactor = predErrorAngle/rotUncertainty * 10;
	errorScaleR = errorScaleLerp*errorScaleRLerp + (1-errorScaleLerp)*errorScaleRLerp * rotErrorFactor;
	// TODO: Rotational errorScale has no change at all for EKF filter

	// Debug updated state
	//LOG(LTrackingFilter, LDebug, "Predicted Delta %.6f, Observed Delta %.6f, Prediction Error %.6f, Filter Error %.6f\n", predictedAngle, obsAngle, predErrorAngle, filterErrorAngle);
	//LOG(LTrackingFilter, LDebug, "Rotation Uncertainty: Previous: %.6f, Predicted: %.6f, Corrected: %.6f\n", rotUncertaintyPrev, rotUncertaintyPred, rotUncertainty);
	LOG(LTrackingFilter, LDebug, "Predicted Delta %.6f, Prediction Error %.6f, Filter Error %.6f\n", predictedAngle, predErrorAngle, filterErrorAngle);
	LOG(LTrackingFilter, LDebug, "Rotation Uncertainty: %.6f\n", rotUncertainty);
	LOG(LTrackingFilter, LDebug, "Error Scale: Previous: %.6f, New: %.6f, Desired Factor: %.6f\n", prevErrorScaleR, errorScaleR, rotErrorFactor);

	Eigen::Matrix3f cov = filter.getCovariance().template block<3,3>(9,9);
	float fac = 1000.0f;
	LOG(LTrackingFilter, LDebug, "Rotation Delta Covariance (*%f):\n", fac);
	LOG(LTrackingFilter, LDebug, "%.3f %.3f %.3f\n", cov(0,0)*fac, cov(0,1)*fac, cov(0,2)*fac);
	LOG(LTrackingFilter, LDebug, "%.3f %.3f %.3f\n", cov(1,0)*fac, cov(1,1)*fac, cov(1,2)*fac);
	LOG(LTrackingFilter, LDebug, "%.3f %.3f %.3f\n", cov(2,0)*fac, cov(2,1)*fac, cov(2,2)*fac);

	movementModel.setExpectedError(errorScaleT, errorScaleR);
}
template<> void TrackingFilter<Filter_EKF_PA_EKF_RV<float>>::debugState(StateFilter &stateFilter, StatePred &statePred, std::string label, TimeStep timeStep)
{
	LOG(LTrackingFilter, LDebug, "%s: %.3fmm/f, %.3fmm/f^2 -- %.3fdg/f (Change: %.3fdg/f)\n", label.c_str(),
		stateFilter.vel().norm()*1000*timeStep.dt(), stateFilter.acc().norm()*1000*timeStep.dt()*timeStep.dt(),
		Eigen::AngleAxisf(stateFilter.getRVelQuat()).angle()*timeStep.dt()/PI*180,
		Eigen::AngleAxisf(statePred.getQuatV()).angle()*timeStep.dt()/PI*180);
}


// -------
// Tracking function
// -------

template<typename TARGET>
bool trackTarget(TARGET &target, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, float timestep, int cameraCount, const TargetTrackingParameters &params)
{
	using Target = TARGET;
	using Scalar = typename Target::Scalar;
	using StateFilter = typename Target::Filter::StateFilter;
	using StatePred = typename Target::Filter::StatePred;

	typename Target::Filter::TimeStep timeStep(timestep);

	auto &filter = target.filter;
	Eigen::Isometry3f oldObservedPose = target.filter.poseObserved;

	// Predict new state
	filter.predict(timeStep);

	// Calculate uncertainty
	Eigen::Vector3f uncertainty = Eigen::Vector3f::Constant(params.minUncertainty3D).cwiseMax(filter.stdDev*params.uncertaintySigma);
	Eigen::Vector3f factor = Eigen::Vector3f::Constant(1).cwiseQuotient(uncertainty);
	LOG(LTracking, LDebug, "    Predicted pose with uncertainty %.2fmm, matching target in 2D...\n", uncertainty.mean()*1000);

	// Make sure to allocate memory for any newly added cameras and reset internal data
	target.tracking2DData.init(cameraCount);

	// Match target with points and optimise pose
	target.match2D = trackTarget2D(*target.target, filter.posePredicted, filter.stdDev, calibs, cameraCount, 
		points2D, properties, relevantPoints2D, params, target.tracking2DData);
	if (target.match2D.error.samples < params.minTotalObs) return false;
	LOG(LTracking, LDebug, "    Pixel Error after 2D target track: %fpx mean over %d points\n",
		target.match2D.error.mean*PixelFactor, target.match2D.error.samples);

	// Update filter
	filter.measurements++;
	filter.update(timeStep, target.match2D.pose);

	// Calculate error probability
	Eigen::Vector3f compErrorProbability = (filter.poseObserved.translation()-filter.posePredicted.translation()).cwiseAbs().cwiseProduct(factor);
	filter.errorProbability = 1-(Eigen::Vector3f::Constant(1)-compErrorProbability).prod();	

	// Update error used during prediction
	if (filter.measurements > 3)
		filter.updateError(timeStep, params.uncertaintySigma);

	LOG(LTrackingFilter, LDebug, "    Matched target with error probability %.2f%%! Change %.3fmm and %.3fdg; Error %.3fmm and %.3fdg!\n", 
		100*filter.errorProbability, 
		(filter.poseObserved.translation() - oldObservedPose.translation()).norm()*1000, 
		Eigen::AngleAxisf(filter.poseObserved.rotation() * oldObservedPose.rotation().transpose()).angle()/PI*180, 
		(filter.poseObserved.translation() - filter.posePredicted.translation()).norm()*1000, 
		Eigen::AngleAxisf(filter.poseObserved.rotation() * filter.posePredicted.rotation().transpose()).angle()/PI*180);

	return target.match2D.error.samples >= params.minTotalObs && target.match2D.error.mean < params.maxTotalError;
}

void FlexUKFFilter::init(Isometry3<Scalar> pose, const TargetTrackingParameters &params)
{
	curStateFilter.position() = pose.translation().cast<double>();
	curStateFilter.setQuaternion(Eigen::Quaterniond(pose.rotation().cast<double>()));
	auto &errorCov = curStateFilter.errorCovariance();
	errorCov.diagonal().segment<3>(0).setConstant(params.uncertaintyPos*params.initialUncertaintyState);
	errorCov.diagonal().segment<3>(3).setConstant(params.uncertaintyRot*params.initialUncertaintyState);
	errorCov.diagonal().segment<3>(6).setConstant(params.uncertaintyPos*params.initialUncertaintyChange);
	errorCov.diagonal().segment<3>(9).setConstant(params.uncertaintyRot*params.initialUncertaintyChange);
}

template<>
bool trackTarget<TrackedTarget<FlexUKFFilter>>(TrackedTarget<FlexUKFFilter> &target, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	TimePoint_t time, float timestep, int cameraCount, const TargetTrackingParameters &params)
{
	using Target = TrackedTarget<FlexUKFFilter>;
	using Scalar = typename Target::Scalar;

	auto &filter = target.filter;
	Eigen::Isometry3f oldObservedPose = target.filter.poseObserved;

	// Predict new state
	filter.movementModel.setDamping(params.dampeningPos, params.dampeningRot);
	flexkalman::predict(filter.curStateFilter, filter.movementModel, timestep);
	filter.posePredicted = filter.curStateFilter.getIsometry().cast<float>();
	filter.stdDev = filter.curStateFilter.errorCovariance().template block<3,3>(0,0).diagonal().cwiseSqrt().cast<float>();

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

	// Update filter
	filter.measurements++;
	filter.curTime = time;
	filter.poseObserved = target.match2D.pose;
	auto measurement = AbsolutePoseMeasurement{
		target.match2D.pose.translation().cast<double>(),
		Eigen::Quaterniond(target.match2D.pose.rotation().cast<double>()),
		Eigen::Vector3d::Constant(params.uncertaintyPos*params.uncertaintyPos),
		Eigen::Vector3d::Constant(params.uncertaintyRot*params.uncertaintyRot)
	};
	flexkalman::SigmaPointParameters sigmaParams(params.sigmaAlpha, params.sigmaBeta, params.sigmaKappa);
	if (!flexkalman::correctUnscented(filter.curStateFilter, measurement, true, sigmaParams))
	{
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
	}
	filter.poseFiltered = filter.curStateFilter.getIsometry().cast<float>();

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
	TimePoint_t time, float timestep, int cameraCount, const TargetTrackingParameters &params);