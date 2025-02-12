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

typedef uint16_t BlobIndex;
const BlobIndex InvalidBlob = (BlobIndex)-1;

/**
 * A triangulated point with reference to the blobs it was triangulated from
 */
template<typename Scalar>
struct TriangulatedPoint_t
{
	typedef Eigen::Matrix<Scalar,3,1> Position;
	Position pos;
	Scalar error; // Mean distance to involved rays 
	Scalar confidence; // Validity score, 2*NC^2+C
	Scalar size; // Estimated size in 3D
	// with C being rays with other intersections, and NC rays that only had this point (very likely to be valid point then)
	std::vector<BlobIndex> blobs; // blobs[cameraIndex] = blobIndex, -1 = not seen from camera

	TriangulatedPoint_t (Position Pos, Scalar Error, Scalar Confidence) : pos(Pos), error(Error), confidence(Confidence) {}
	TriangulatedPoint_t (Position Pos, Scalar Error, Scalar Confidence, int CamCount) : pos(Pos), error(Error), confidence(Confidence)
	{
		blobs.resize(CamCount, InvalidBlob);
	}
};
typedef TriangulatedPoint_t<float> TriangulatedPoint;


/* Functions */


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
 * Pick best points for each ray conflict and reevaluate point confidences based on it
 * Leaves points3D in a semi-sorted order, highest confidence (and secondarily lowest error) first
 * Requires internally stored intersection data from previous triangulateRayIntersections call
 */
void resolveTriangulationConflicts(std::vector<TriangulatedPoint> &points3D, float maxError);

/**
 * Filter out points below the confidence threshold into the discarded3D list
 */
void filterTriangulatedPoints(std::vector<TriangulatedPoint> &points3D, std::vector<TriangulatedPoint> &discarded3D, float confidenceThreshold);

/**
 * Basic triangulation of point through ray intersection. The same as performed in triangulateRayIntersections
 */
template<typename Scalar, typename PointScalar, typename CalibScalar = CVScalar>
Eigen::Matrix<Scalar,3,1> triangulatePoint(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &pointGroups, const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D);

/**
 * Refine triangulation accuracy of point by minimising the reprojection error (not projection invariant)
 */
template<typename Scalar, typename PointScalar, typename CalibScalar = CVScalar>
Eigen::Matrix<Scalar,3,1> refineTriangulation(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &pointGroups, const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D);

/**
 * Refine triangulation accuracy of point by minimising the reprojection error iteratively (nearly projection invariant)
 */
template<typename Scalar, typename PointScalar, typename CalibScalar = CVScalar>
Eigen::Matrix<Scalar,3,1> refineTriangulationIterative(const std::vector<std::vector<Eigen::Matrix<PointScalar,2,1>> const *> &pointGroups, const std::vector<CameraCalib_t<CalibScalar>> &cameras, TriangulatedPoint &point3D, int maxIterations = 20, float threshold3D = 0.000001f);

#endif // POINT_TRIANGULATION_H