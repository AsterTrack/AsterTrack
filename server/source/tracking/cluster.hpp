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

#ifndef TRACKING_CLUSTER_H
#define TRACKING_CLUSTER_H

/**
 * Keeping track of likely, loosely-defined clusters in 3D
 */

#include "pipeline/parameters.hpp"

#include "util/eigendef.hpp"

typedef std::vector<int> TriCluster3D; // Indices into triangulations3D

typedef std::vector<int> Cluster2D; // Indices into a cameras points2D

// Stat representation of a Cluster2D
struct Cluster2DStats
{
	Eigen::Vector2f center;
	Eigen::Matrix2f covariance;
	int points;
};

// Association of multiple Cluster2D by way of triangulation into a form of 3D cluster
struct Cluster2DTri3D
{
	std::vector<int> camClusters; // Index into cluster list for each involved camera
	float score;
	Eigen::Vector3f center;

	Cluster2DTri3D(int cams) : camClusters(cams, -1) {}
};

// Association of multiple Cluster2D by way of triangulation into a form of 3D cluster
struct Cluster3D
{
	float score;
	Eigen::Vector3f center;
	Eigen::Matrix3f covariance;
};

static Cluster2DStats calculateClusterStats2D(const std::vector<int> &cluster, const std::vector<Eigen::Vector2f> &points2D)
{
	Eigen::MatrixXf points(cluster.size(), 2);
	for (int p = 0; p < cluster.size(); p++)
		points.row(p) = points2D[cluster[p]];
	Cluster2DStats stats;
	stats.center = points.colwise().mean();
	Eigen::MatrixXf centered = points.rowwise() - stats.center.transpose();
	stats.covariance = (centered.transpose() * centered) / (points.rows() - 1);
	stats.points = cluster.size();
	return stats;
}

float calculateCluster2DOverlap(const Cluster2DStats clusterA, const CameraCalib &camA, const Cluster2DStats clusterB, const CameraCalib &camB);

std::vector<Cluster2DTri3D> triangulateClusters2D(const std::vector<std::vector<Cluster2DStats>> clusterStats,
	const std::vector<CameraCalib> &calibs, const ClusteringParameters &params);


#endif // TRACKING_CLUSTER_H