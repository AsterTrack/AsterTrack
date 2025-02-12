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

#ifndef BLOB_REFINEMENT_H
#define BLOB_REFINEMENT_H

#include "../util/eigendef.hpp"

#include <vector>

struct HoughParameters
{
	Vector2<int> boundsPX;
	Vector2<float> boundsMid;

	float radiusMin, radiusStep;
	int radiusRange;

	float circleWidth;
	float positionStep;
};


/**
 * Refines the blob edge using the grayscale gradient
 */
void refineBlobEdge(const std::vector<Vector2<uint16_t>> &edge, const uint8_t *frame, uint32_t stride, std::vector<Vector2<float>> &refinedEdge, int target, float maxOffsetPX);

/**
 * Finds the best estimate for a range of radii and positions for a blob given the edge points
 * Accumulates votes for all parameter combinations from edge points (Hough Transform)
 * Supply with desired parameters tuned for accuracy or performance
 */
bool getRefinementEstimate(const std::vector<Vector2<float>> &edge, const HoughParameters &params, int &radiusStep, Vector2<float> &center);

/**
 * Iteratively refines the center given a set of refined edge points and a good estimate of the center and radius
 */
int iterativeRefinement(Vector2<float> &center, float &radius, const std::vector<Vector2<float>> &edge, std::vector<bool> &outliers, int iterations);

#endif // BLOB_REFINEMENT_H