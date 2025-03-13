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

#ifndef TARGET_DETECTION_3D_H
#define TARGET_DETECTION_3D_H

#include "target/target.hpp"
#include "point/triangulation.hpp"

#include "util/eigendef.hpp"

/**
 * Detecting a target (set of markers) in a triangulated 3D point cloud
 */


/* Structures */

struct ErrorRangeComp
{
	float error;
	ErrorRangeComp(float error) { this->error = error; }
	bool operator() (const PointRelation& rel, float value) { return rel.distance < value-error; }
	bool operator() (float value, const PointRelation& rel) { return value+error < rel.distance; }
};

struct TargetCandidate3D
{
	std::vector<int> points; // Triangulated Points that were matched
	std::vector<int> pointMap; // Map of triangulated points to marker points
	// To get triangulated points: points3D[points[i]]
	// To get actual points: target3D.points[pointMap[points[i]]]
	Eigen::Isometry3f pose;
	float MSE = NAN;
};


/* Functions */

/**
 * Calculate MSE of candidate marker in given point cloud in mm^2
 */
float calculateCandidateMSE(const TargetCalibration3D &target3D,
	const std::vector<TriangulatedPoint> &points3D,
	const TargetCandidate3D &candidate);

/**
 * Picks the best candidate by point count and internal MSE.
 * Returns the index, MSE in mm^2 and count of other candidates with the same maximum point count
 */
std::tuple<int,int> getBestTargetCandidate(const TargetCalibration3D &target3D, 
	const std::vector<TriangulatedPoint> &points3D,
	std::vector<TargetCandidate3D> &candidates);

/**
 * Detect a marker in the triangulated 3D Point cloud
 * Returns all candidates
 */
void detectTarget3D(const TargetCalibration3D &target3D,
	const std::vector<TriangulatedPoint> &points3D, const std::vector<int> &indices,
	std::vector<TargetCandidate3D> &candidates,
	float sigmaError = 3, float poseSigmaError = 6, bool quickAssign = true);

/**
 * Detect a marker in the triangulated 3D Point cloud
 * Returns the best candidate
 */
TargetCandidate3D detectTarget3D(const TargetCalibration3D &target3D,
	const std::vector<TriangulatedPoint> &points3D, const std::vector<int> &indices,
	float sigmaError = 3, float poseSigmaError = 6, bool quickAssign = true);

/**
 * Detect a marker in the triangulated 3D Point cloud
 * Returns all candidate poses with respective MSE in mm^2 and point count
 */
void detectTarget3D(const TargetCalibration3D &target3D,
	const std::vector<TriangulatedPoint> &points3D, const std::vector<int> &indices,
	std::vector<Eigen::Isometry3f> &poses3D, std::vector<std::pair<float,int>> &posesMSE,
	float sigmaError = 3, float poseSigmaError = 6, bool quickAssign = true);

#endif // TARGET_DETECTION_3D_H