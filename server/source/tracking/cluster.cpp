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


#include "tracking/cluster.hpp"
#include "point/triangulation.hpp"

#include "util/eigenutil.hpp"
#include "util/log.hpp"

float calculateCluster2DOverlap(const Cluster2DStats clusterA, const CameraCalib &camA, const Cluster2DStats clusterB, const CameraCalib &camB)
{
	auto bhattacharyyaCoeff = [](const Cluster2DStats clusterA, const CameraCalib &camA, const Cluster2DStats clusterB, const CameraCalib &camB)
	{
		// Cluster in A is projected into camera B as a ray denoting it's depth axis
		Ray3f ray = castRay<float>(clusterA.center, camA);
		Eigen::Vector2f rayStart = projectPoint2D(camB.camera, ray.pos);
		Eigen::Vector2f rayDir = (projectPoint2D(camB.camera, ray.pos+ray.dir) - rayStart).normalized();
		Eigen::Vector2f rayPerp(rayDir.y(), -rayDir.x());

		// To get variance of cluster A orthogonal to the projected ray in camera B, we need to
		// project the ray perpendicular back into camera A to probe the covariance matrix of cluster A
		Eigen::Matrix2f rotA = camA.camera.rotation().topLeftCorner<2,2>().cast<float>();
		Eigen::Matrix2f rotB = camB.transform.rotation().topLeftCorner<2,2>().cast<float>();
		Eigen::Vector2f projPerp = (rotA * rotB * rayPerp).normalized();

		// Calculate the center/mean difference
		float section = rayDir.dot(clusterB.center - rayStart) / rayDir.dot(rayDir);
		Eigen::Vector2f proj = rayStart + rayDir * section;
		float diffSq = (clusterB.center - proj).squaredNorm();

		// Now we get the variances of both clusters in view of camera B perpendicular to the ray of cluster A
		float varianceB = (clusterB.covariance * rayPerp).norm();
		// Since varianceA is calculated in view of camera A, transfer to camera B via second ray along perpendicular axis
		float varianceA_CamA = (clusterA.covariance * projPerp).norm();
		Ray3f rayCov = castRay<float>((clusterA.center + projPerp*varianceA_CamA).eval(), camA);
		Eigen::Vector2f rayCovStart = projectPoint2D(camB.camera, rayCov.pos);
		Eigen::Vector2f rayCovDir = (projectPoint2D(camB.camera, rayCov.pos+rayCov.dir) - rayStart).normalized();
		Eigen::Vector2f projCov = rayCovStart + rayCovDir * section;
		float varianceA = (projCov - proj).norm();

		// TODO: Double-check above calculations

		// Using these, the bhattacharyya "distance" [0-1] can be used to denote overlap between both clusters
		float bhattacharyya_dist = diffSq / (varianceA + varianceB) / 4 + std::log((varianceA + varianceB) / (2*std::sqrt(varianceA * varianceB))) / 2;
		float bhattacharyya_coeff = std::exp(-bhattacharyya_dist);
		return bhattacharyya_coeff;
	};

	return bhattacharyyaCoeff(clusterA, camA, clusterB, camB) * bhattacharyyaCoeff(clusterB, camB, clusterA, camA);
}


std::vector<Cluster2DTri3D> triangulateClusters2D(const std::vector<std::vector<Cluster2DStats>> clusterStats,
	const std::vector<CameraCalib> &calibs, const ClusteringParameters &params)
{
	LOG(LCluster, LDebug, "Checking cluster overlaps:");
	std::vector<Cluster2DTri3D> clusterTri;

	std::vector<std::vector<Eigen::Vector2f>> blobContainer(calibs.size());
	std::vector<std::vector<Eigen::Vector2f> const *> points2D(calibs.size());
	for (int c = 0; c < calibs.size(); c++)
	{
		blobContainer[c].resize(1);		 // Used to store a single blob for each camera involved with a point
		points2D[c] = &blobContainer[c]; // Just interfacing
	}
	TriangulatedPoint triPoint(Eigen::Vector3f::Zero(), 0, 10, calibs.size());

	for (int c = 0; c < clusterStats.size(); c++)
	{
		for (int i = 0; i < clusterStats[c].size(); i++)
		{
			if (clusterStats[c][i].points < params.clusterTri.minFocusClusterPoints)
				break; // Since they are sorted, can break here

			Cluster2DTri3D cluster(clusterStats.size());
			cluster.camClusters[c] = i;
			cluster.score = clusterStats[c][i].points;

			for (int cc = 0; cc < clusterStats.size(); cc++)
			{
				if (c == cc) continue;

				int best = -1;
				float bestOverlap = 0;
				int bestPoints = 0;
				for (int j = 0; j < clusterStats[cc].size(); j++)
				{
					float overlap = calculateCluster2DOverlap(clusterStats[c].front(), calibs[c], clusterStats[cc][j], calibs[cc]);
					LOG(LCluster, LTrace, "    Cluster %d of camera %d with %d 2D points has overlap of %f with cluster %d of camera %d with %d 2D points!",
						i, c, (int)clusterStats[c][i].points, overlap, j, cc, (int)clusterStats[cc][j].points);
					if (overlap < params.clusterTri.min2DClusterOverlap) continue;
					if (overlap > bestOverlap)
					{
						best = j;
						bestOverlap = overlap;
						bestPoints = clusterStats[cc][j].points;
					}
				}
				if (best >= 0)
				{
					cluster.camClusters[cc] = best;
					cluster.score += bestPoints * bestOverlap;
				}
			}

			if (cluster.score < params.clusterTri.min3DClusterScore)
				continue;

			// TODO: Not only check for exact match, but also partial, and keep only cluster with higher score

			bool addCluster = true;
			{ // Only check if exact cluster is already recorded, if so don't add but take highest score
				for (auto &comp : clusterTri)
				{
					bool isExactMatch = true;
					for (int cc = 0; cc < comp.camClusters.size(); cc++)
					{
						if (comp.camClusters[cc] != cluster.camClusters[cc])
						{
							isExactMatch = false;
							break;
						}
					}
					if (isExactMatch)
					{ // Found an exact match
						comp.score = std::max(comp.score, cluster.score);
						addCluster = false;
						LOG(LCluster, LTrace, "    Matched 3D cluster of score %f around 2D cluster %d of camera %d with %d 2D points, with new score of %f",
							cluster.score, i, c, (int)clusterStats[c][i].points, comp.score);
						break;
					}
				}
			}
			if (addCluster && !params.clusterTri.allowCompeting)
			{ // Remove any competing cluster with lesser score than this one
				bool foundCompeting = false;
				for (auto &comp : clusterTri)
				{
					for (int cc = 0; cc < comp.camClusters.size(); cc++)
					{
						if (comp.camClusters[cc] == cluster.camClusters[cc])
						{ // Is competing
							foundCompeting = true;
							if (comp.score > cluster.score)
							{
								LOG(LCluster, LDebug, "    Found 3D cluster of score %f around 2D cluster %d of camera %d with %d 2D points, to be competing with existing cluster of score %f",
									cluster.score, i, c, (int)clusterStats[c][i].points, comp.score);
								addCluster = false;
							}
							break;
						}
					}
					if (!addCluster) break;
				}
				if (foundCompeting && addCluster)
				{ // Remove other competing clusters
					LOG(LCluster, LDebug, "    Adopting 3D cluster of score %f around 2D cluster %d of camera %d with %d 2D points, despite competition with other clusters!",
						cluster.score, i, c, (int)clusterStats[c][i].points);
					for (auto compIt = clusterTri.begin(); compIt != clusterTri.end();)
					{
						bool isCompeting = false;
						for (int cc = 0; cc < compIt->camClusters.size() && !isCompeting; cc++)
							isCompeting = compIt->camClusters[cc] == cluster.camClusters[cc];
						if (!isCompeting) compIt++;
						else compIt = clusterTri.erase(compIt);
					}
				}
				else if (addCluster)
				{
					LOG(LCluster, LDebug, "    Adopting 3D cluster of score %f around 2D cluster %d of camera %d with %d 2D points",
						cluster.score, i, c, (int)clusterStats[c][i].points);
				}
			}
			else if (addCluster)
			{
				LOG(LCluster, LDebug, "    Adopting 3D cluster of score %f around 2D cluster %d of camera %d with %d 2D points",
					cluster.score, i, c, (int)clusterStats[c][i].points);
			}

			if (addCluster)
			{
				for (int c = 0; c < cluster.camClusters.size(); c++)
				{
					bool gotCluster = cluster.camClusters[c] >= 0;
					triPoint.blobs[c] = gotCluster? 0 : InvalidBlob;
					if (gotCluster)
						blobContainer[c][0] = clusterStats[c][cluster.camClusters[c]].center;
				}
				// Calculate optimal triangulation
				//cluster.center = refineTriangulation<float, float, double>(points2D, calibs, triPoint);
				cluster.center = refineTriangulationIterative<float, float, double>(points2D, calibs, triPoint, 5);
				clusterTri.push_back(std::move(cluster));
			}
		}
	}
	return clusterTri;
}