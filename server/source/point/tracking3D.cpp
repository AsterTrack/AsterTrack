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
#include "point/kalman.inl"

#include "util/log.hpp"

#include "flexkalman/FlexibleKalmanFilter.h"
#include "flexkalman/FlexibleUnscentedCorrect.h"

/**
 * Tracking a single point (marker) in 3D space
 */


/* Functions */

template<typename TrackedMarker>
int trackMarker(TrackedMarker &marker,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices,
	float timestep, float sigma)
{
	using Scalar = typename TrackedMarker::ScalarType;

	auto &filter = marker;

	// TODO: Move into parameters once this is used
	TargetTrackingParameters params = {};

	// Predict new state
	filter.model.setDamping(params.dampeningPos, params.dampeningRot);
	flexkalman::predict(filter.state, filter.model, timestep);
	filter.posePredicted = filter.state.getIsometry().template cast<float>();
	filter.stdDev = filter.state.errorCovariance().template block<3,3>(0,0).diagonal().cwiseSqrt().template cast<float>();

	Eigen::Vector3f predPos = filter.posePredicted.translation();
	Eigen::Matrix3f predCov = filter.state.errorCovariance().template topLeftCorner<3,3>().template cast<float>();

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

	// Update filter
	filter.measurements++;
	filter.poseObserved.translation() = points3D[matchedPoint];
	auto measurement = AbsolutePositionMeasurement(
		points3D[matchedPoint].template cast<double>(),
		Eigen::Vector3d::Constant(params.uncertaintyPos).eval());
	flexkalman::SigmaPointParameters sigmaParams(params.sigmaAlpha, params.sigmaBeta, params.sigmaKappa);
	if (!flexkalman::correctUnscented(filter.state, measurement, true, sigmaParams))
	{
		LOG(LTrackingFilter, LWarn, "Failed to correct pose in filter! Reset!");
	}
	filter.poseFiltered = filter.state.getIsometry().template cast<float>() ;

	return matchedPoint;
}

template int trackMarker(TrackedMarker<float> &marker,
	const std::vector<Eigen::Vector3f> &points3D, const std::vector<int> &triIndices,
	float timestep, float sigma);