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
		int minStrongCameraSamples = 8;
		int minStrongCameras = 2;
		int minCameraSamples = 6;
		int minTotalSamples = 25;
	} targetObs;
};

struct TrackingParameters
{
	TriangulationParameters tri = {};
	ClusteringParameters cluster = {};
	TargetDetectionParameters detect = {};
	TargetTrackingParameters track = {};
	ContinuousOptimisationParameters cont = {};
};

#endif // TRACKING_PARAMETERS_H