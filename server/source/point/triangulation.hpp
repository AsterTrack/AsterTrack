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

#ifndef POINT_TRIANGULATION_H
#define POINT_TRIANGULATION_H

#include "util/eigendef.hpp"

#include <vector>

/**
 * Triangulation of 3D points from 2D blobs
 */


/* Structures */

typedef uint16_t CamIndex;
typedef uint16_t BlobIndex;

/**
 * A triangulated point with reference to the blobs it was triangulated from
 */
template<typename Scalar>
struct TriangulatedPoint_t
{
	Vector3<Scalar> pos;
	Scalar error; // Mean distance to involved rays 
	Scalar confidence; // Validity score
	Scalar size; // Estimated size in 3D
	struct TriSample
	{
		CamIndex camera;
		BlobIndex blob;
	};
	std::vector<TriSample> samples;

	TriangulatedPoint_t () {}
	TriangulatedPoint_t (Vector3<Scalar> pos, Scalar error, Scalar confidence) : pos(pos), error(error), confidence(confidence) {}
};
typedef TriangulatedPoint_t<float> TriangulatedPoint;


/* Functions */


float getTriConfidence(int obsClean, int obsConflicted);

/**
 * Calculate triangulatedPoints as the intersection points between rays of each camera 
 * Calculates mean error of triangulated points to rays and confidence based on rays involved
 * With unconflicted (NC) and conflicted (C) involved rays, point confidence is 2*nc^2 + c
 * Stores intersection data internally for later use in conflict resolving
 */
void triangulateRayIntersections(const std::vector<CameraCalib> &cameras, 
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const std::vector<std::vector<int> const *> &relevantPoints2D,
	std::vector<TriangulatedPoint> &points3D, float maxError, float minError);

/**
 * Pick best points for each blob conflict and reevaluate point confidences
 * Drop such blobs and any point that is below the given confidence threshold
 * Leaves points3D in a semi-sorted order, highest confidence (sec. error) first
 * Requires internally stored intersection data from previous triangulateRayIntersections call
 */
void resolveTriangulationConflicts(const std::vector<CameraCalib> &cameras, std::vector<TriangulatedPoint> &points3D, float maxError, float confidenceThreshold);

/**
 * Refine triangulation accuracy of point by minimising the reprojection error (not projection invariant)
 * NOTE: Relies on TriangulatedPoint::TriSample::camera indexing into given subset of cameras
 */
template<typename Scalar, typename PointScalar, typename CalibScalar = CVScalar>
Eigen::Matrix<Scalar,3,1> refineTriangulation(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &points2D,
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D);

/**
 * Refine triangulation accuracy of point by minimising the reprojection error iteratively (nearly projection invariant)
 * NOTE: Relies on TriangulatedPoint::TriSample::camera indexing into given subset of cameras
 */
template<typename Scalar, typename PointScalar, typename CalibScalar = CVScalar, typename TriScalar = float>
Eigen::Matrix<Scalar,3,1> refineTriangulationIterative(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &points2D,
	const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint_t<TriScalar> &point3D, int maxIterations = 20, float threshold3D = 0.000001f);

#endif // POINT_TRIANGULATION_H