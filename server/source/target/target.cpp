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

#include "target.hpp"

#include "target/parameters.hpp"

#include "util/eigenutil.hpp"

int triTargetMinRelations = 4;
float triTargetMinDistance = 0.1f;

/**
 * Updates properties reliant on marker positions
 */
void TargetCalibration3D::updateMarkers()
{
	// Update bounds
	bounds = Bounds3f();
	for (const auto &marker : markers)
		bounds.include(marker.pos);

	// Setup lookup tables for quick marker identification
	relationDist.clear();
	pointRelation.clear();
	pointRelation.resize(markers.size());

	// Step 1: Create all relations
	std::vector<PointRelation> relations;
	for (int i = 0; i < markers.size(); i++)
	{
		for (int j = i+1; j < markers.size(); j++)
		{
			Eigen::Vector3f dir = markers[j].pos - markers[i].pos;
			relations.push_back({ i, j, dir, dir.squaredNorm() });
		}
	}

	// Step 2: Sort based on distance
	std::sort(relations.begin(), relations.end());

	// Step 3: Enter relevant relations
	for (PointRelation &rel : relations)
	{
		if (pointRelation[rel.pt1].size() < triTargetMinRelations || 
			pointRelation[rel.pt2].size() < triTargetMinRelations || 
			rel.distance < triTargetMinDistance*triTargetMinDistance)
		{ // Each point should have at least triTargetMinRelations stored relations
			int relIndex = relationDist.size();
			rel.distance = std::sqrt(rel.distance);
			rel.dir = rel.dir / rel.distance;
			relationDist.push_back(rel);
			pointRelation[rel.pt1].push_back(relIndex);
			pointRelation[rel.pt2].push_back(relIndex);
		}
	}

	// markerDistLookup is now a list of relations sorted by distance with at least triTargetMinRelations of the closest relations per point
}

static inline bool projectMarker(Eigen::Vector2f &projected, const TargetMarker &marker,
	const CameraCalib &calib, const Eigen::Isometry3f mv)
{
	Eigen::Vector3f camPoint = mv * marker.pos;
	projected = applyProjection2D(calib, camPoint);
	return camPoint.z() > 0; // Cull back
}

static inline bool projectMarker(Eigen::Vector2f &projected, const TargetMarker &marker,
	const CameraCalib &calib, const Eigen::Isometry3f mv, float expandFoV)
{
	Eigen::Vector3f camPoint = mv * marker.pos;
	// Cull back
	if (camPoint.z() < 0)
		return false;
	// Project
	projected = applyProjection2D(calib, camPoint);
	// Calculate and clip marker points not facing the camera in regards to their field of view
	Eigen::Vector3f ptNrm = mv.linear() * marker.nrm;
	float facing = -ptNrm.dot(camPoint.normalized());
	return facing+expandFoV >= marker.angleLimit;
}

/**
 * Projects target markers into camera view, clipping out-of-view points
 * Writes those points into their position in points2D corresponding to the marker point index
 * Writes indices of points projected into projected
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D, std::vector<int> &projected, 
	const TargetCalibration3D &target, const CameraCalib &calib,
	const Eigen::Isometry3f &pose, float expandFoV)
{
	Eigen::Isometry3f mv = calib.view.cast<float>() * pose;

	projected.clear();
	points2D.resize(target.markers.size());
	projected.reserve(target.markers.size());
	for (int i = 0; i < target.markers.size(); i++)
		if (projectMarker(points2D[i], target.markers[i], calib, mv, expandFoV))
			projected.push_back(i);
}

/**
 * Projects relevant target markers into camera view, clipping out-of-view points
 * Writes those points into their position in points2D corresponding to the marker point index
 * Writes indices of points projected into projected
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D, std::vector<int> &projected,
	const TargetCalibration3D &target, const CameraCalib &calib,
	const std::vector<int> &relevant,
	const Eigen::Isometry3f &pose, float expandFoV)
{
	Eigen::Isometry3f mv = calib.view.cast<float>() * pose;

	projected.clear();
	points2D.resize(target.markers.size());
	projected.reserve(target.markers.size());
	for (int i : relevant)
		if (projectMarker(points2D[i], target.markers[i], calib, mv, expandFoV))
			projected.push_back(i);
}

/**
 * Projects target markers into camera view, clipping out-of-view points
 * Writes only those points to points2D
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D, 
	const TargetCalibration3D &target, const CameraCalib &calib,
	const Eigen::Isometry3f &pose, float expandFoV)
{
	Eigen::Isometry3f mv = calib.view.cast<float>() * pose;

	points2D.clear();
	points2D.reserve(target.markers.size());
	Eigen::Vector2f projPt;
	for (int i = 0; i < target.markers.size(); i++)
		if (projectMarker(projPt, target.markers[i], calib, mv, expandFoV))
			points2D.push_back(projPt);
}


/**
 * Projects relevant target markers into camera view, clipping out-of-view points
 * Writes only those points to points2D
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D, 
	const TargetCalibration3D &target, const CameraCalib &calib,
	const std::vector<int> &relevant,
	const Eigen::Isometry3f &pose)
{
	Eigen::Isometry3f mv = calib.view.cast<float>() * pose;

	points2D.clear();
	points2D.reserve(target.markers.size());
	Eigen::Vector2f projPt;
	for (int i : relevant)
		if (projectMarker(projPt, target.markers[i], calib, mv))
			points2D.push_back(projPt);
}