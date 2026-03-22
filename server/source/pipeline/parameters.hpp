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

#ifndef TRACKING_PARAMETERS_H
#define TRACKING_PARAMETERS_H

#include "target/parameters.hpp"
#include "calib_target/parameters.hpp"

#include "util/eigendef.hpp"

/**
 * Parameters for tracking algorithms
 */

struct TriangulationParameters
{
	float minIntersectError = 0.2f/1000;
	float maxIntersectError = 1.0f/1000;
	float minIntersectionConfidence = 4.0f;
	int refineIterations = 2;
};

struct ClusteringParameters
{
	// 2D clusters of blobs
	struct
	{
		int minPoints = 2;
		float maxDistance = 40.0f*PixelSize;
	} blob2DCluster;
	// Triangulate 2D clusters to 3D clusters
	struct
	{
		int minFocusClusterPoints = 6;
		float min2DClusterOverlap = 0.7f;
		float min3DClusterScore = 8.0f;
		bool allowCompeting = false;
	} clusterTri;
	// 3D clusters of triangulated points
	struct
	{
		int minPoints = 3;
		float maxDistance = 0.1f;
	} tri3DCluster;
};

struct ContinuousOptimisationParameters
{
	struct
	{
		int minStrongCameraSamples = 4;
		int minStrongCameras = 2;
		int minCameraSamples = 2;
		int minTotalSamples = 12;
	} targetObs;

	SubsampleTargetParameters targetOptSubsampling = {};
	SubsampleTargetParameters cameraOptSubsampling = {
		0, 10000,
		1.0f, 2.0f,
		0.0f, 0.0f
	};
};

struct VirtualTrackingParameters
{
    struct {
		// General behaviour for all filters
		float sigmaInitState = 50000, sigmaInitChange = 10000000;
		float detectSigma = 1000000, trackSigma = 1;
		float dampeningPos = 0.95f, dampeningRot = 0.9f;

		// UKR settings for all filters
		float sigmaAlpha = 0.001f, sigmaBeta = 2.0f, sigmaKappa = 0.0f;

		// Full Target Pose Filter Update
		float stdDevPos = 0.002f, stdDevEXP = 0.005f;

		template<typename Scalar>
		Eigen::Matrix<Scalar,6,6> getSyntheticCovariance() const
		{
			Eigen::Matrix<Scalar,6,6> covariance = Eigen::Matrix<Scalar,6,6>::Identity();
			covariance.diagonal().template head<3>().setConstant(stdDevPos*stdDevPos);
			covariance.diagonal().template tail<3>().setConstant(stdDevEXP*stdDevEXP);
			return covariance;
		}
	} filter = {};

	struct {
		float lerpFactorPos = 0.0f;
		float lerpFactorRot = 0.0f;
	} calibration = {};
};

struct TrackingParameters
{
	TriangulationParameters tri = {};
	ClusteringParameters cluster = {};
	TargetDetectionParameters detect = {};
	TargetTrackingParameters track = {};
	ContinuousOptimisationParameters cont = {};
	VirtualTrackingParameters virt = {};
};

#endif // TRACKING_PARAMETERS_H