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

#include "point/tracking3D.hpp"
#include "util/log.hpp"

/**
 * Tracking a single point (marker) in 3D space
 */


/* Functions */

int trackMarker(TrackedMarker<float> &marker,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices,
	float timestep, float sigma)
{
	TimeControl<float> timeStep;
	timeStep.dt() = timestep;
	
	// Predict new state and uncertainty
	State3DOF<float> predState;
	Eigen::Vector3f predStdDev;
	if (marker.measurements == 1)
	{
		predState = marker.state;
		predStdDev = Eigen::Vector3f::Constant(10);
	}
	else if (marker.measurements == 2)
	{
		predState = marker.movementModel.f(marker.state, timeStep);
		predStdDev = marker.state.vel()*2 + Eigen::Vector3f::Constant(1);
	}
	else
	{
		marker.movementModel.setExpectedError(marker.errorScaleT);
		predState = marker.filter.predict(marker.movementModel, timeStep);
		predStdDev = marker.filter.getCovariance().block<3,3>(0,0).diagonal().cwiseSqrt();
	}

	// Calculate uncertainty
	Eigen::Vector3f uncertainty = predStdDev*sigma;
	Eigen::Vector3f factor = Eigen::Vector3f::Constant(1).cwiseQuotient(uncertainty);

	// Find best candidate
	int matchedPoint = -1;
	float matchedErrorProbability = std::numeric_limits<float>::max();
	for (int p : triIndices)
	{
		// Probability: >1, then outside confidence interval and likely not matching
		// TODO: Proper gaussian probability calculation?
		auto diff = points3D[p]-predState.pos();
		auto compErrorProbability = diff.cwiseAbs().cwiseProduct(factor);
		float errorProbability = 1-(Eigen::Vector3f::Constant(1)-compErrorProbability).prod();
		if (errorProbability < matchedErrorProbability)
		{
			matchedPoint = p;
			matchedErrorProbability = errorProbability;
		}
	}

	if (matchedPoint < 0 || matchedErrorProbability > 1000)
	{
		LOG(LTracking, LWarn, "Best point %d of %d points had %f error probability\n", matchedPoint, (int)points3D.size(), matchedErrorProbability);
		return -1;
	}

	marker.measurements++;
	Eigen::Vector3f position = points3D[matchedPoint];
	float posError = (position - predState.pos()).norm();
	float posDiff = (position - marker.state.pos()).norm();
	
	auto debugState = [](State3DOF<float> &state, std::string label = "State")
	{
		LOG(LTracking, LDebug, "%s: %.3fcm/f, %.3fcm/f^2\n", label.c_str(), state.vel().norm(), state.acc().norm());
	};

	// Correct state and uncertainty
	if (marker.measurements == 2)
	{ // Calculate velocity
		marker.state.vel() = (position - marker.state.pos())/timeStep.dt();
		marker.state.pos() = position;

		debugState(marker.state, "First State");
		LOG(LTracking, LDebug, "Matched point %d with error probability %f%%, uncertainty %fcm! Moved %.3fcm, error %.3fcm! Preliminary State: %fcm/s\n", matchedPoint, 100*matchedErrorProbability, uncertainty.mean(), posDiff, posError, marker.state.vel().norm());
	}
	else if (marker.measurements == 3)
	{ // Calculate acceleration and update velocity
		Eigen::Vector3f vel = (position-marker.state.pos())/timeStep.dt();
		marker.state.acc() = (vel-marker.state.vel())/timeStep.dt();
		marker.state.vel() = vel;
		marker.state.pos() = position;
		marker.filter.init(marker.state);

		debugState(marker.filter.getState(), "Second State");
		LOG(LTracking, LDebug, "Matched point %d with error probability %f%%, uncertainty %fcm! Moved %.3fcm, error %.3fcm! Preliminary State: %fcm/s, %fcm/s^2\n", matchedPoint, 100*matchedErrorProbability, uncertainty.mean(), posDiff, posError, marker.filter.getState().vel().norm(), marker.filter.getState().acc().norm());
	}
	else
	{ // Use initialised filter

		// Debug predicted state
		debugState(marker.filter.getState(), "Predicted State");

		// Create measurement
		marker.state.pos() = position;
		auto measurement = marker.measurementModel.h(marker.state); 

		// Update filter
		marker.measurementModel.setExpectedError(0.1f); // Expecting 1mm accuracy.
		// TODO: Adapt expected error like in trackMarker3D
		marker.state = marker.filter.update(marker.measurementModel, measurement);
		
		// Adjust error scale so target confidence is reached
		marker.errorScaleT = 0.5f*marker.errorScaleT + 0.5f*std::min(1.0f, matchedErrorProbability);

		// Debug updated state
		debugState(marker.filter.getState(), "Updated State");
		LOG(LTracking, LDebug, "Matched point %d with error probability %.4f%%, uncertainty %.3fcm! Moved %.3fcm, error %.3fcm! State: %.3fcm/s, %.3fcm/s^2; Error scale adjusted to %.2f\n", matchedPoint, 100*matchedErrorProbability, uncertainty.mean(), posDiff, posError, marker.filter.getState().vel().norm(), marker.filter.getState().acc().norm(), marker.errorScaleT);
	}

	return matchedPoint;
}