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
#include <bitset>

/*
 * Blob Detection
 * Responsible for extracting blobs (as Clusters) from the input camera frames
 * Clusters are connected sets of pixels in screen space
 */

struct VisualisationLock; // visualisation.hpp

/* Functions */

typedef uint8_t CompID;
struct Comp;
struct Tile;
typedef std::bitset<16384> BGBitset; // 1280/8 x 800/8 = 16000, rounded up for higher performance

struct BlobDetection
{
	bool initialised;

	// Sizes
	uint32_t maskIndexSize;
	Vector2<int> bitmaskSize;
	int tileW, tileH, blkW, blkH, blkTileW;
	// Blob offset to compensate for processing margins
	Vector2<int> blobOffset;
	// Dynamic buffer for all 8x4 tiles that are part of a blob
	std::vector<Tile> blobTiles;
	// Shared buffer serving as a map from initial component ID to merged component ID
	CompID *blobCompMerge;
	// Shared buffer serving as a data accumulator while components are processed, used in conjunction with merge map
	Comp *blobCompData;
	// Map from initial blob to merged blob index
	CompID *blobMerge;
	// Background mask
	BGBitset storedBackgroundMask;
	BGBitset tempBackgroundMask;
	BGBitset *backgroundMask;
	std::vector<uint32_t> backupBGBitmask;

	BlobDetection(Vector2<int> maskSize, Vector2<int> maskOffset, int cpuCores);
	~BlobDetection();

	inline operator bool() { return initialised; }

	/* Read map and extract mask tiles with dots (1s) as blob tiles and enter them in a list */
	template<bool BG = true>
	void registerBlobTile(const uint32_t *bitmask, uint32_t tileID);
	/* Reads back blobMap from the given QPU bitmask into the internal buffer, ready for analysis on the CPU */
	void fetchMaskRegionsCPU(const uint32_t *bitmask);
	/* Estimate maximum size needed for VPU output buffer. */
	uint32_t getRegionBufferSize();
	/* Reads back blobMap from the given QPU bitmask into the internal buffer, ready for analysis on the CPU */
	void fetchMaskRegionsVPU(const uint32_t *bitmask, const void *vpuMaskIndex);
	/* Fetches entirely on CPU and with VPU index side-by-side to verify correctness */
	void verifyMaskRegionsFetchVPU(const uint32_t *bitmask, const void *vpuMaskIndex);

	/* Manually mask image on CPU, used for verification only */
	void manualMasking(const uint8_t *image, uint32_t stride, uint32_t width, uint32_t height,
		uint32_t *bitmask, uint8_t thresholdCO, uint8_t diffCO, VisualisationLock vis);
	/* Compare mask from QPU and CPU and mark differences as blobTiles, for verification only */
	void compareMasks(const uint32_t *bitmaskQPU, const uint32_t *bitmaskCPU, bool diffAsTiles);

	/* Analyses the regions detected in the last GPU step and outputs detected blobs into target array */
	void performCCL(std::vector<Cluster> &blobs);

	/* Background Calibration */
	void initBackgroundCalibration(uint32_t *bgBitmask);
	void resetBackgroundCalibration(uint32_t *bgBitmask);
	void retryBackgroundCalibration(uint32_t *bgBitmask);
	void acceptBackgroundCalibration(uint32_t *bgBitmask);
	void discardBackgroundCalibration(uint32_t *bgBitmask);
	void updateBackgroundCalibration(uint32_t *bgBitmask, std::vector<uint8_t> &bgTiles);
	std::vector<uint8_t> getTempBGTiles();
};

#endif // BLOB_DETECTION_H
