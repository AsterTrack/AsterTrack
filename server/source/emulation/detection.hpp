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

#ifndef BLOB_DETECTION_H
#define BLOB_DETECTION_H

#include "config.hpp"
#include "blob/blob.hpp"

#include "util/eigendef.hpp"

#include <vector>
#include <bitset>

/*
 * Blob Detection
 * Responsible for extracting blobs (as Clusters) from the input camera frames
 * Clusters are connected sets of pixels in screen space
 */

/* Last blob data */

extern std::bitset<1280*800> mask;
extern std::bitset<1280*800> edgeMask;
extern std::bitset<1280*800> centerMask;


/* Functions */

/**
 * Basic blob detection routine
 */
void performBlobDetection(const CameraConfig &config, Bounds2i bounds,
	const uint8_t *image, int width, int height, int stride,
	std::vector<Cluster> &blobs);


#endif // BLOB_DETECTION_H
