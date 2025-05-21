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

#ifndef TARGET_H
#define TARGET_H

#include "util/eigendef.hpp"

struct TargetMarker
{
	Eigen::Vector3f pos;
	Eigen::Vector3f nrm;
	float angleLimit;
	float size;
};

struct PointRelation
{
	int pt1;
	int pt2;
	Eigen::Vector3f dir;
	float distance;

	PointRelation(int p1, int p2, Eigen::Vector3f d, float dst) : pt1(p1), pt2(p2), dir(d), distance(dst) {}
};
static bool operator<(const struct PointRelation& a, const struct PointRelation& b) { return a.distance < b.distance; }

struct TargetCalibration3D
{
	int id;
	std::string label;
	std::vector<TargetMarker> markers;

	Bounds3f bounds;
	std::vector<PointRelation> relationDist; // Shortest neighbouring relations of all points, sorted by distance
	std::vector<std::vector<int>> pointRelation; // Index of relations for each point

	TargetCalibration3D() {}

	TargetCalibration3D(int ID, std::string Label, const std::vector<TargetMarker> &Markers)
		: id(ID), label(Label), markers(Markers)
	{
		updateMarkers();
	}

	TargetCalibration3D(const std::vector<TargetMarker> &Markers)
		: markers(Markers)
	{
		updateMarkers();
	}

	TargetCalibration3D(const std::vector<Eigen::Vector3f> &Points)
	{
		initialise(Points);
	}

	/**
	* Updates properties reliant on marker positions
	*/
	void updateMarkers();

	Eigen::Vector3f getCOM() const
	{
		Eigen::Vector3f COM = Eigen::Vector3f::Zero();
		for (const TargetMarker &marker : markers)
			COM += marker.pos;
		return COM / markers.size();
	}

	void estimateNormals()
	{
		Eigen::Vector3f COM = getCOM();
		for (TargetMarker &marker : markers)
			marker.nrm = (marker.pos-COM).normalized();
	}

	void resetMarkerEstimations()
	{
		for (TargetMarker &marker : markers)
		{
			marker.angleLimit = std::cos(360/360*PI);
			marker.size = 0.005f;
		}
	}

	void initialise(const std::vector<Eigen::Vector3f> &points)
	{
		markers.clear();
		markers.reserve(points.size());
		for (auto &pt : points)
			markers.push_back({ pt });
		estimateNormals();
		resetMarkerEstimations();
		updateMarkers();
	}
};

/**
 * Projects target markers into camera view, clipping out-of-view points
 * Writes those points into their position in points2D corresponding to the marker point index
 * Writes indices of points projected into projected
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D, std::vector<int> &projected,
	const TargetCalibration3D &target, const CameraCalib &calib,
	const Eigen::Isometry3f &pose, float expandFoV = 2.0f);

/**
 * Projects relevant target markers into camera view, clipping out-of-view points
 * Writes those points into their position in points2D corresponding to the marker point index
 * Writes indices of points projected into projected
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D, std::vector<int> &projected,
	const TargetCalibration3D &target, const CameraCalib &calib,
	const std::vector<int> &relevant,
	const Eigen::Isometry3f &pose, float expandFoV = 2.0f);

/**
 * Projects target markers into camera view, clipping out-of-view points
 * Writes only those points to points2D
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D,
	const TargetCalibration3D &target, const CameraCalib &calib,
	const Eigen::Isometry3f &pose, float expandFoV = 2.0f);

/**
 * Projects relevant target markers into camera view, clipping out-of-view points
 * Writes only those points to points2D
 */
void projectTarget(std::vector<Eigen::Vector2f> &points2D,
	const TargetCalibration3D &target, const CameraCalib &calib,
	const std::vector<int> &relevant,
	const Eigen::Isometry3f &pose);

#endif // TARGET_H