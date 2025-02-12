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

#include "blob/blob.hpp"

#include "util/eigendef.hpp"

#include <vector>

/*
 * Blob Detection
 * Responsible for extracting blobs (as Clusters) from the input camera frames
 * Clusters are connected sets of pixels in screen space
 */

/* Functions */

/*
 * Initialize resources required for blob detection
 */
bool initBlobDetection(Vector2<int> maskSize, Vector2<int> maskOffset, int cpuCores);

/*
 * Reads back blobMap from the GPU into the specified buffer, ready for analysation on the CPU
 */
void performBlobDetectionRegionsFetch(uint32_t *mask);

/*
 * Analyses the regions detected in the last GPU step and outputs detected blobs into target array
 */
void performBlobDetectionCPU(std::vector<Cluster> &blobs);

/*
 * Destroys resources used for blob detection
 */
void cleanBlobDetection();

/* Background Calibration */
void initBackgroundCalibration();
void acceptBackgroundCalibration();
void resetBackgroundCalibration();
std::vector<uint8_t> updateBackgroundCalibration();
std::vector<uint8_t> getTempBGTiles();

#endif // BLOB_DETECTION_H
