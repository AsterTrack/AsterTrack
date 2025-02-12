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

#ifndef BLOB_H
#define BLOB_H

#include "parameters.hpp"
#include "refinement.hpp"

#include "../util/eigendef.hpp"

// Cluster of pixel dots with bounds that define a point (centroid)
struct Cluster
{
	uint32_t ptCnt = 0;
	Bounds2<uint16_t> bounds = Bounds2<uint16_t>();
	Vector2<float> centroid = Vector2<float>::Zero();
	std::vector<Vector2<uint16_t>> dots;
	std::vector<Vector2<uint16_t>> edge;
	float size = 0;			// Estimate of the radius in pixels, doesn't scale well with small blobs
	float contrast = 0;		// Contrast between base and peak, derived value
	float certainty = 0;	// Rough measure for scale*peak value, from maxima hint stage
	float reliability = 0;	// Measure of how much data supports the accuracy of the centroid
	float circularity = 0;	// Measure of how circular the blob is
	float weight = 0;		// Weight, as in the sum of the pixel value 
	float value = 0;		// Measure of how pronounced the blob is, derived from size and contrast

	Cluster() {}
	Cluster(float x, float y, Bounds2<uint16_t> b, uint32_t cnt) : ptCnt(cnt), bounds(b), centroid(x,y) {}
};

extern std::vector<Vector2<float>> edgeRefined;
extern std::vector<bool> edgeOutliers;

struct PrecomputedKernels
{
    std::vector<float> base;
    std::vector<std::vector<float>> ssr;
};

PrecomputedKernels precomputeKernels(const BlobProcessingParameters &params);

std::vector<Cluster> handleCluster(Cluster &&cluster, const uint8_t *frame, uint32_t stride, uint32_t width, uint32_t height,
	const BlobProcessingParameters &params, const PrecomputedKernels &kernels,
	int &blurTimeUS, int &maximaTimeUS, int &localMaxTimeUS, int &maxIterTimeUS,
	int &finalCheckTimeUS, int &resegTimeUS, int &refineTimeUS);

bool handleClusterSingle(Cluster &blob, const uint8_t *frame, uint32_t stride, const BlobProcessingParameters &params);

HoughParameters estimateHoughParameters1(Cluster &blob);
HoughParameters estimateHoughParameters2(Cluster &blob);

bool refineCluster(Cluster &blob, const uint8_t *frame, uint32_t stride, const RefinementParameters params, const HoughParameters &hough, int refineIt);

#endif // BLOB_H