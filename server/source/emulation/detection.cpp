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

#include "detection.hpp"

#include <algorithm>
#include <cassert>


/* Last blob data */
std::bitset<1280*800> mask;
std::bitset<1280*800> edgeMask;
std::bitset<1280*800> centerMask;


/* Functions  */

/**
 * Basic blob detection routine
 */
void performBlobDetection(const CameraConfig &config, Bounds2i bounds,
	const uint8_t *image, int width, int height, int stride,
	std::vector<Cluster> &blobs)
{
	assert(mask.size() >= width*height);
	mask.reset();
	edgeMask.reset();
	centerMask.reset();

	auto TestPixel = [&](int x, int y, bool set)
	{
		bool masked = false;
		uint8_t val = image[y*stride+x];
		if (val >= config.blobProcessing.thresholds.absolute)
		{
			if (set)
			{
				mask.set(y*width+x);
				centerMask.set(y*width+x);
			}
			masked = true;
			//return true; // Do edge test anyway for edgeMask
		}
		if (val >= config.blobProcessing.thresholds.edge)
		{ // Check 5x5 minimum
			uint8_t min55 = val;
			for (int yy = std::max(0, y-2); yy < std::min(height, y+3); yy++)
			for (int xx = std::max(0, x-2); xx < std::min(width, x+3); xx++)
			{
				uint8_t neighbor = image[yy*stride+xx];
				if (min55 > neighbor)
					min55 = neighbor;
			}
			if ((val-min55) >= config.blobProcessing.thresholds.edge)
			{
				if (set)
				{
					mask.set(y*width+x);
					edgeMask.set(y*width+x);
				}
				masked = true;
			}
		}
		return masked;
	};

	auto TestJoinBlob = [&](Cluster &blob, int x, int y)
	{
		auto test_impl = [&](Cluster &blob, int x, int y, auto &test)
		{
			if (blob.ptCnt > 10000) return false; // Blob will be split up
			if (x < bounds.min.x() || y < bounds.min.y() || x >= bounds.max.x() || y >= bounds.max.y()) return false;
			if (mask.test(y*width+x)) return true; // Already part of blob
			if (!TestPixel(x, y, true)) return false;
			// Add dot to blob
			Vector2<uint16_t> dot(x, y);
			blob.centroid += dot.cast<float>();
			blob.bounds.include(Bounds2<uint16_t>(x, y, x+1, y+1));
			blob.dots.push_back(dot);
			blob.ptCnt++;
			// Test surrounding members (combined blob-detection and connected-component labelling step)
			int outsideCount = 0;
			outsideCount += test(blob, x-1, y-1, test);
			outsideCount += test(blob, x+0, y-1, test);
			outsideCount += test(blob, x+1, y-1, test);
			outsideCount += test(blob, x-1, y+0, test);
			//test(blob, x+0, y+0, test);
			outsideCount += test(blob, x+1, y+0, test);
			outsideCount += test(blob, x-1, y+1, test);
			outsideCount += test(blob, x+0, y+1, test);
			outsideCount += test(blob, x+1, y+1, test);
			// Add as edge point if one surrounding member is not part of the blob 
			if (outsideCount < 8)
				blob.edge.push_back(dot);
			return true;
		};
		return test_impl(blob, x, y, test_impl);
	};

	for (int y = bounds.min.y(); y < bounds.max.y(); y++)
	for (int x = bounds.min.x(); x < bounds.max.x(); x++)
	{
		if (mask.test(y*width+x)) continue; // Already handled
		if (!TestPixel(x, y, false)) continue;

		// Recursively build blob
		Cluster blob = {};
		TestJoinBlob(blob, x, y);
		if (blob.ptCnt == 0) continue;

		// Finalise centroid
		blob.centroid /= blob.ptCnt;
		// No better estimation of size
		blob.size = std::sqrt((float)blob.ptCnt / PI);

		blobs.push_back(std::move(blob));
	}
}