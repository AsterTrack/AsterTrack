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
#include "visualisation.hpp"
#include "blob/qpu_blob_tiled.hpp"
#include "blob/vpu_find_active_tiles.hpp"

#include <cstring>
#include <cassert>
#include <algorithm>
#include <unistd.h>

//#define BLOB_DEBUG
//#define BLOB_TRACE

#define CALCULATE_BOUNDS_EARLY
#define BLOB_MERGE_SATELLITES
#define BLOB_POST_PASS

#if !defined(BLOB_POST_PASS) && !defined(CALCULATE_BOUNDS_EARLY)
#error "If bounds aren't calculated as blobs are built, and there's no post-pass on all dots, no bounds will be generated!"
#endif

// Component data config
typedef uint8_t CompID; // Lower -> significantly better performance (cache size?), but limits MaxCompID
const CompID MaxCompID = 255; // Max intermediate component ID - higher than final merged component count, 0 is reserved for none
typedef uint32_t TileIndex; // Index into all tiles of an image, 16bit may not be enough for high resolutions
typedef uint16_t TileID; // ID for stored tiles, limits MaxTileCount. Lower->better performance (cache size?)
TileID InvalidTile = (TileID)-1;
TileID MaxTileCount = 1024; // Max regarded tile count, reserved for none - set later to model-dependant number

const int minPtCnt = 2;

/* Structures  */

// Tile in blob mask
typedef uint32_t MaskTile;
// Tile as used for processing
struct Tile
{ // This should be packed by default
	uint16_t x, y;
	TileID t, l, r, b;
	MaskTile bytes;
	CompID compMap[8][4];
};
// Additional data for components
struct Comp
{
	uint32_t x, y;
	uint16_t ptCnt;
#ifdef CALCULATE_BOUNDS_EARLY
	Bounds2<uint16_t> bounds = {};
#endif
};

BlobDetection::BlobDetection(Vector2<int> maskSize, Vector2<int> maskOffset, int cpuCores)
{
	initialised = (storedBackgroundMask.size() >= (maskSize.x()/8*maskSize.y()/8));
	if (!initialised) return;
	maskIndexSize = getVPUMaskIndexBufferSize(maskSize.prod());

	bitmaskSize = maskSize;
	// Tile is 4x8 pixels
	tileW = maskSize.x() / 8;
	tileH = maskSize.y() / 4;
	// Block is 5x1 tiles, or 20x8 pixels (for blockTiles=5)
	blkW = tileW;
	blkH = tileH / mskBlockTiles;
	// Number of tiles in a row of blocks (blockTiles rows of tiles interlaced)
	blkTileW = blkW*mskBlockTiles;

	// Mask doesn't span the full image due to processing margins, offset blob output into image space
	blobOffset = maskOffset;

	// Since CPU has to search for tiles with blobs, we are limited heavily by CPU resources
	if (cpuCores > 1) // Zero 2
		MaxTileCount = 1024; // TODO: Adjust with multi-core blob detection
	else // Zero 1
		MaxTileCount = 384;

	// Setup resources used during blob detection
	blobTiles.reserve(MaxTileCount);
	blobCompMerge = new CompID[MaxCompID+1];
	blobCompData = new Comp[MaxCompID+1];
	blobMerge = new CompID[MaxCompID+1];
	//storedBackgroundMask.reset();
	backgroundMask = &storedBackgroundMask;
	// Already allocate backup bitmask for background calibration
	backupBGBitmask.resize(bitmaskSize.prod());
}

BlobDetection::~BlobDetection()
{
	if (!initialised) return;
	delete[](blobCompMerge);
	delete[](blobCompData);
	delete[](blobMerge);
	printf("-- Blob Detection Cleaned --\n");
}


/* Blob Detection */

static inline CompID resolveComponentMerge(CompID *blobCompMerge, CompID compID);

#define TILE_PARAMS(tileIndex) \
	uint32_t blkID = tileIndex / mskBlockTiles; \
	uint_fast8_t blkRow = tileIndex % mskBlockTiles; \
	uint_fast16_t blkX = blkID % blkW; \
	uint_fast16_t blkY = blkID / blkW; \
	uint16_t tX = blkX; \
	uint16_t tY = blkY*mskBlockTiles + blkRow;

template<bool BG>
void BlobDetection::registerBlobTile(const uint32_t *bitmask, TileIndex tile)
{
	MaskTile *maskTiles = (MaskTile*)bitmask;
	TILE_PARAMS(tile)

	auto test_bg = [&](std::size_t index)
	{
		if constexpr (!BG) return true;
		return !backgroundMask->test(index);
	};
	uint32_t bgIndex = (tY>>1)*tileW + tX;
	if (!test_bg(bgIndex))
		return;

	TileID t = InvalidTile;
	TileID l = InvalidTile;
	// Now find any tiles to the top or left quickly
	if (blkRow > 0)
	{ // Quick way of finding top tile (bounded by 1)
		if (maskTiles[tile-1] && test_bg(bgIndex-(tY&1? 0 : tileW)))
			t = blobTiles.size()-1;
	}
	else if (blkY > 0)
	{ // Very slow way, have to go to top row of blocks (bounded by blkTileW, 400 for 480p)
		TileIndex topTile = tile-blkTileW+mskBlockTiles-1;
		if (maskTiles[topTile] && test_bg(bgIndex-(tY&1? 0 : tileW)))
		{ // Have a top tile
			TileID max = blobTiles.size()-1;
			for (int i = 0; i < blkTileW; i++)
			{
				if (blobTiles[max-i].x == tX && blobTiles[max-i].y == tY-1)
				{
					t = max-i;
					break;
				}
			}
		}
	}
	if (blkX > 0)
	{ // Relatively fast way of finding left region (bounded by blockTiles)
		TileIndex leftTile = tile-mskBlockTiles;
		if (maskTiles[leftTile] && test_bg(bgIndex-1))
		{ // Have a left tile
			TileID max = blobTiles.size()-1;
			for (int i = 0; i < mskBlockTiles; i++)
			{
				if (blobTiles[max-i].y == tY && blobTiles[max-i].x == tX-1)
				{
					l = max-i;
					break;
				}
			}
		}
	}
#ifdef BLOB_TRACE
	printf("  Added region %d (%d, %d) with top %d and left %d\n", bgIndex, x, y, t, l);
#endif
	blobTiles.emplace_back(tX, tY, t, l, InvalidTile, InvalidTile, maskTiles[tile]);
}

void BlobDetection::fetchMaskRegionsCPU(const uint32_t *bitmask)
{
	MaskTile *maskTiles = (MaskTile*)bitmask;
	blobTiles.clear();

	// This loop is the most critical part of the performance.
	// It is expected to discard on the *ptr check the majority of the time
	// And yet, even with nothing on the image (empty loop), it takes 2ms on a Zero 2 (single core)

	uint64_t *max = (uint64_t*)(maskTiles+(tileW*tileH));
	uint64_t *ptr = (uint64_t*)__builtin_assume_aligned(maskTiles, 128); // Not sure if this helps much
	for (; ptr < max; ptr++)
	{
		if (*ptr)
		{ // Found dots in 64-Bit region, check each 32-Bit tile individually
			MaskTile *tile = (MaskTile*)ptr;
			if (*(tile+0))
				registerBlobTile(maskTiles, (tile+0)-maskTiles);
			if (*(tile+1))
				registerBlobTile(maskTiles, (tile+1)-maskTiles);
			if (blobTiles.size()+2 > MaxTileCount)
				break;
		}
	}

#ifdef BLOB_DEBUG
	printf("CPU fetched %d regions with blobs!\n", blobTiles.size());
#endif
}

void BlobDetection::fetchMaskRegionsVPU(const uint32_t *bitmask, const void *vpuMaskIndex)
{
	MaskTile *maskTiles = (MaskTile*)__builtin_assume_aligned(bitmask, 128); // Not sure if this helps much
	blobTiles.clear();

	int indexEntries = 0;
	for (VPU_MASK_INDEX *ptr = (VPU_MASK_INDEX*)vpuMaskIndex; *ptr != VPU_INDEX_TERM; ptr++)
	{
		indexEntries++;
		assert(indexEntries < maskIndexSize);
		uint32_t entry = *ptr;
		assert((entry+1) * VPU_INDEX_TILES <= tileW*tileH);
		for (int i = 0; i < VPU_INDEX_TILES; i++)
		{
			TileIndex tileIndex = entry * VPU_INDEX_TILES + i;
			if (maskTiles[tileIndex])
				registerBlobTile<false>(maskTiles, tileIndex);
		}
		if (blobTiles.size()+1 > MaxTileCount)
			break;
	}
#ifdef BLOB_DEBUG
	printf("VPU fetched %d regions from %d index entries!\n", (int)blobTiles.size(), indexEntries);
#endif
}

void BlobDetection::verifyMaskRegionsFetchVPU(const uint32_t *bitmask, const void *vpuMaskIndex)
{
	MaskTile *maskTiles = (MaskTile*)__builtin_assume_aligned(bitmask, 128); // Not sure if this helps much

	uint32_t maxIndexEntry = (tileW*tileH) / VPU_INDEX_TILES;
	VPU_MASK_INDEX *ptr = (VPU_MASK_INDEX*)vpuMaskIndex;
	for (uint32_t entry = 0; entry < maxIndexEntry; entry++)
	{
		bool vpuMarkedLine = *ptr == entry;
		int found = 0;
		for (int i = 0; i < VPU_INDEX_TILES; i++)
		{
			TileIndex tileIndex = entry * VPU_INDEX_TILES + i;
			if (maskTiles[tileIndex])
				found++;
		}
		if (found != 0)
		{
			if (!vpuMarkedLine)
				printf("VPU did NOT mark line %d but found %d tiles!\n", entry, found);
		}
		else if (vpuMarkedLine)
			printf("VPU marked line %d but found nothing!\n", entry);
		if (vpuMarkedLine && *ptr != VPU_INDEX_TERM)
			ptr++;
	}
}

void BlobDetection::manualMasking(const uint8_t *image, uint32_t stride, uint32_t width, uint32_t height,
	uint32_t *bitmask, uint8_t thresholdCO, uint8_t diffCO, VisualisationLock vis)
{
	auto TestPixel = [&](int x, int y)
	{
		if (y < 0) return false; // May happen on first row
		uint8_t val = image[y*stride+x];
		if (val >= thresholdCO)
			return true; // Do edge test anyway for edgeMask
		if (val < diffCO)
			return false;
		// Check 5x5 minimum
		uint8_t min55 = val;
		for (int yy = std::max(0, y-2); yy < std::min<int>(height, y+3); yy++)
			for (int xx = std::max(0, x-2); xx < std::min<int>(width, x+3); xx++)
				min55 = std::min(min55, image[yy*stride+xx]);
		return ((val-min55) >= diffCO);
	};

	for (TileIndex tile = 0; tile < tileW*tileH; tile++)
	{
		TILE_PARAMS(tile)
		int pX = tX*8 + blobOffset.x();
		int pY = tY*4 + blobOffset.y();

		bitmask[tile] = 0;
		for (int x = 0; x < 8; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				if (TestPixel(pX+x, pY+y))
				{
					bitmask[tile] |= 1 << DOTID(x,y);
					vis.write(pX+x, pY+y, 0xFF0000FF);
				}
			}
		}
	}
}

void BlobDetection::compareMasks(const uint32_t *bitmaskQPU, const uint32_t *bitmaskCPU, bool diffAsTiles)
{
	MaskTile *maskTilesQPU = (MaskTile*)__builtin_assume_aligned(bitmaskQPU, 128); // Not sure if this helps much
	MaskTile *maskTilesCPU = (MaskTile*)__builtin_assume_aligned(bitmaskCPU, 128); // Not sure if this helps much

	int differCnt = 0;
	int diffExpected = 0;
	int diffUnexpected = 0;
	for (TileIndex tile = 0; tile < tileW*tileH; tile++)
	{
		if (maskTilesCPU[tile] == maskTilesQPU[tile])
			continue;
		TILE_PARAMS(tile)

		bool expected = tY == 0 && maskTilesQPU[tile] == 0x0F0F0F0F;
		differCnt++;
		if (expected) diffExpected++;
		else diffUnexpected++;
		if (!expected && diffUnexpected < 20)
			printf("    Tile %d x %d differs! %u on cpu vs %u on qpu\n", tX, tY, maskTilesCPU[tile], maskTilesQPU[tile]);

		// No need for top or left tiles
		blobTiles.emplace_back(tX, tY, InvalidTile, InvalidTile, InvalidTile, InvalidTile, maskTilesCPU[tile] ^ maskTilesQPU[tile]);
	}
	printf("In total, %d tiles differed, of those %d expected and %d unexpected!\n", differCnt, diffExpected, diffUnexpected);
}

void BlobDetection::performCCL(std::vector<Cluster> &blobs)
{
	blobCompMerge[0] = 0; // Setup 'No Component' ID

	CompID compNum = 0; // Number of final components
	CompID compIndex = 0; // Number of intermediary components

	// Iterate over blob regions do connected component labeling
	for (int i = 0; i < blobTiles.size(); i++)
	{
		Tile *tile = &blobTiles[i];
		Tile *topTile = NULL, *leftTile = NULL;
		if (tile->t != InvalidTile)
		{
			topTile = &blobTiles[tile->t];
			topTile->b = i;
		}
		if (tile->l != InvalidTile)
		{
			leftTile = &blobTiles[tile->l];
			leftTile->r = i;
		}

#ifdef BLOB_TRACE
		printf("Tile %d / %d: T?%c, L?%c! -- State: %d(%d) components!\n", tile->x, tile->y, topTile? 'y' : 'n', leftTile? 'y' : 'n', compNum, compIndex);
#endif

		if (tile->bytes == 0xFFFFFFFF)
		{ // Optimization on large blobs
			auto updateComp = [this, &compNum](CompID compID, CompID comp)
			{ // Update compID with comp and merge to smallest if needed
				if (comp == 0) return compID;
				comp = resolveComponentMerge(blobCompMerge, comp);
				if (compID == 0) compID = comp;
				else if (comp < compID)
				{ // Merge compID with comp
					blobCompMerge[compID] = comp;
					compNum--;
					compID = comp;
				}
				else if (comp > compID)
				{ // Merge comp with CompID
					blobCompMerge[comp] = compID;
					compNum--;
				}
				return compID;
			};
			// Merge components among shared border with left and top
			CompID compID = 0;
			if (leftTile)
			{
				for (int y = 0; y < 4; y++)
					compID = updateComp(compID, leftTile->compMap[7][y]);
			}
			if (topTile)
			{
				for (int x = 0; x < 8; x++)
					compID = updateComp(compID, topTile->compMap[x][3]);
			}
			// If none shared, create new component
			if (compID == 0 && compIndex < MaxCompID)
			{
				compID = ++compIndex;
				blobCompMerge[compID] = compID;
				blobCompData[compID] = {};
				compNum++;
#ifdef BLOB_TRACE
				printf("Assigning new component ID %d!\n", (int)compID);
#endif
			}
			// Apply merged component to all cells
			memset(tile->compMap, compID, 4*8);
			// Update component data with new dots
			Comp *comp = &blobCompData[compID];
			comp->x += (tile->x*8*8 + 28)*4;
			comp->y += (tile->y*4*4 + 6)*8;
			comp->ptCnt += 8*4;
#ifdef CALCULATE_BOUNDS_EARLY
			comp->bounds.include(Bounds2<uint16_t>(tile->x*8, tile->y*4, tile->x*8+7+1, tile->y*4+3+1));
#endif
			continue;
		}

		//#pragma GCC unroll n
		// Do connected component labelling within 8x4 tile using connected components from top and left tiles
		for (int x = 0; x < 8; x++)
		{
			CompID top = topTile? topTile->compMap[x][3] : 0;

			for (int y = 0; y < 4; y++)
			{
				CompID compID = 0;
				if (DOT(tile->bytes, x, y))
				{ // Dot at current pixel
					CompID left = 0;
					if (x > 0) left = tile->compMap[x-1][y];
					else if (leftTile) left = leftTile->compMap[7][y];

					if (top != 0 && left != 0)
					{ // Two connected dots, assign and make sure both are merged
						CompID topComp = resolveComponentMerge(blobCompMerge, top);
						CompID leftComp = resolveComponentMerge(blobCompMerge, left);
						compID = topComp;

						if (leftComp != topComp)
						{ // Separated components connected through this dot, merge components
							if (topComp > leftComp)
								blobCompMerge[topComp] = compID = leftComp;
							else
								blobCompMerge[leftComp] = compID = topComp;
							compNum--;
#ifdef BLOB_TRACE
							printf("Merging %d (%d) into %d (%d) at pos %d/%d!\n", left, leftComp, top, topComp, x, y);
#endif
						}
					}
					else if (top == 0 && left == 0)
					{ // No connected components from top or left, assign new component
						if (compIndex < MaxCompID)
						{
							compID = ++compIndex;
							blobCompMerge[compID] = compID;
							blobCompData[compID] = {};
							compNum++;
#ifdef BLOB_TRACE
							printf("Assigning new component ID %d!\n", (int)compID);
#endif
						}
					}
					else
					{ // One connected component to assign to
						compID = resolveComponentMerge(blobCompMerge, top | left);
					}
					// Update component data with new dot
					uint16_t dotX = tile->x * 8 + x, dotY = tile->y * 4 + y;
					Comp *comp = &blobCompData[compID];
					comp->x += dotX;
					comp->y += dotY;
					comp->ptCnt++;
#ifdef CALCULATE_BOUNDS_EARLY
					comp->bounds.include(Bounds2<uint16_t>(dotX, dotY, dotX+1, dotY+1));
#endif
				}

				// Assign component ID
				top = compID;
				tile->compMap[x][y] = compID;
			}
		}
	}

	// Compile clusters for merged components
	// Assumption: blobCompMerge[i] <= i -- e.g. the true ID comes before the merged IDs
	int clusterID = 0;
	// Temporary cluster list
	static std::vector<Cluster> tempBlobs;
	tempBlobs.clear();
	tempBlobs.reserve(compNum);
	for (int i = 1; i < compIndex+1; i++)
	{
		assert(blobCompMerge[i] <= i);
		if (blobCompMerge[i] == i)
		{ // Claim cluster index
			blobCompMerge[i] = clusterID++;
			blobMerge[blobCompMerge[i]] = blobCompMerge[i];
			tempBlobs.emplace_back(
				(float)blobCompData[i].x, (float)blobCompData[i].y,
#ifdef CALCULATE_BOUNDS_EARLY
				blobCompData[i].bounds,
#else
				Bounds2<uint16_t>(),
#endif
				blobCompData[i].ptCnt
			);
		}
		else
		{ // Copy cluster index from previous merged components
			int b = blobCompMerge[i] = blobCompMerge[blobCompMerge[i]];
			tempBlobs[b].centroid.x() += blobCompData[i].x;
			tempBlobs[b].centroid.y() += blobCompData[i].y;
			tempBlobs[b].ptCnt += blobCompData[i].ptCnt;
#ifdef CALCULATE_BOUNDS_EARLY
			tempBlobs[b].bounds.include(blobCompData[i].bounds);
#endif
		}
	}

	for (int i = 0; i < tempBlobs.size(); i++)
	{
		tempBlobs[i].centroid.x() /= tempBlobs[i].ptCnt;
		tempBlobs[i].centroid.y() /= tempBlobs[i].ptCnt;
	}

#ifdef BLOB_DEBUG
	printf("Found %d initial merged components, with max %d, list %d!\n", compNum, compIndex, (int)blobs.size());
#endif

#ifdef BLOB_MERGE_SATELLITES
	// Create sorted mapping
	std::vector<int> sortOrder;
	sortOrder.reserve(tempBlobs.size());
	for (int i = 0; i < tempBlobs.size(); i++)
		sortOrder.push_back(i);
	std::partial_sort(sortOrder.begin(), sortOrder.begin() + std::min(100, (int)sortOrder.size()), sortOrder.end(), [](const int &a, const int &b) -> bool {
		return tempBlobs[a].ptCnt > tempBlobs[b].ptCnt;
	});

	const int mergePasses = 1;
	const int satelliteMaxPtCnt = 5, mainMinPtCnt = 5;
	const int minSatelliteAdvantage = 5;
	const float extendFac = 1.2f, extendBase = 0.0f;
	
	// Finalize clusters
	blobs.clear();
	blobs.reserve(tempBlobs.size());
	for (int i = 0; i < sortOrder.size(); i++)
	{
		int index = sortOrder[i];
		if (index < 0) continue; // Already claimed
		if (tempBlobs[index].ptCnt < minPtCnt) 
		{ // Too small to be a reasonable blob
			blobMerge[index] = 0;
			continue;
		}

		// Setup as actual blob
		blobs.push_back(tempBlobs[index]);
		blobMerge[index] = blobs.size();
		Cluster &cluster = blobs[blobMerge[index]-1];

		if (tempBlobs[index].ptCnt < mainMinPtCnt) continue; // Too small to have satellites

#ifdef CALCULATE_BOUNDS_EARLY
		Bounds2<float> bounds;
#else
		Vector3<float> circle;
#endif
		auto updateArea = [&]()
		{
			// Extend range factor
			float s = extendFac * (1.0f + extendBase/cluster.ptCnt);

#ifdef CALCULATE_BOUNDS_EARLY
			bounds = cluster.bounds.cast<float>();
			bounds.extendBy(bounds.extends()/2.0f * (s - 1.0f));
#else // Use circle approximation
			circle = cluster.centroid.homogeneous();
			circle.z() = std::sqrt((float)cluster.ptCnt / M_PI) * s;
#endif
		};
		updateArea();

//		printf("Cluster %d at pos %f / %f had %d dots!\n", i, cluster.centroid.x(), cluster.centroid.y(), (int)cluster.ptCnt);
//		printf("Cluster %d had bounds %d/%d/%d/%d!\n", i, cluster.bounds.minX, cluster.bounds.maxX, cluster.bounds.minY, cluster.bounds.maxY);
//		printf("Cluster %d searches around %f / %f with circle of size %f!\n", i, circle.x(), circle.y(), circle.z());

		// Attempt to merge with smaller blobs (later in sort order) that are closeby (satellite artifacts)
		int mergeCnt = 0;
		for (int n = 0; n < mergePasses; n++)
		for (int j = i+1; j < sortOrder.size(); j++)
		{
			int candIndex = sortOrder[j];
			if (candIndex < 0) continue; // Already claimed

			const Cluster &cand = tempBlobs[candIndex];
			if (cand.ptCnt > satelliteMaxPtCnt || cand.ptCnt*minSatelliteAdvantage > cluster.ptCnt)
				continue; // Not small enough to be a satellite

#ifdef CALCULATE_BOUNDS_EARLY
			if (!bounds.includes(cand.centroid.head<2>()))
				continue;
#else
			if ((cand.centroid.head<2>() - circle.head<2>()).squaredNorm() > circle.z()*circle.z())
				continue;
#endif

			// Peform merge
			blobMerge[candIndex] = blobMerge[index];
			sortOrder[j] = -1;
			mergeCnt++;
			// Update centroid
			int totalCnt = cluster.ptCnt + cand.ptCnt;
			cluster.centroid = (cluster.centroid * cluster.ptCnt + cand.centroid * cand.ptCnt) * (1.0f/totalCnt);
			cluster.ptCnt = totalCnt;
#ifdef CALCULATE_BOUNDS_EARLY
			cluster.bounds.include(cand.bounds);
#endif
			// Update search area
			updateArea();
		}

#ifdef BLOB_TRACE
		printf("Cluster %d has size %f around %f / %f  with %d dots, merged with %d satellites!\n",
			i, cluster.size, cluster.centroid.x(), cluster.centroid.y(), (int)cluster.ptCnt, mergeCnt);
#endif
	}

#else // BLOB_MERGE_SATELLITES

	blobs.clear();
	blobs.reserve(tempBlobs.size());
	for (int i = 0; i < tempBlobs.size(); i++)
	{
		if (tempBlobs[i].ptCnt < minPtCnt) continue; // Too small to be a reasonable blob

		// Setup as actual blob
		blobs.push_back(tempBlobs[i]);
		blobMerge[i] = blobs.size();

#ifdef BLOB_TRACE
		printf("Cluster %d has size %f around %f / %f  with %d dots!\n",
			i, cluster.size, cluster.centroid.x(), cluster.centroid.y(), (int)cluster.ptCnt);
#endif
	}

#endif

	for (Cluster &cluster : blobs)
	{
		// Offset blob coordinates by mask offset
		cluster.centroid.head<2>() += blobOffset.cast<float>();
#ifdef CALCULATE_BOUNDS_EARLY
		cluster.bounds.min += blobOffset.cast<uint16_t>();
		cluster.bounds.max += blobOffset.cast<uint16_t>();
#endif

		// Approximate blob size if it was a circle
#ifdef CALCULATE_BOUNDS_EARLY
		cluster.size = cluster.bounds.extends().maxCoeff()/2.0f;
#else
		cluster.size = std::sqrt((float)cluster.ptCnt / M_PI);
#endif
	}

#ifdef BLOB_POST_PASS
	// Fill clusters dots and edge arrays (and bounds if not CALCULATE_BOUNDS_EARLY)
	for (Cluster &cluster : blobs)
	{
		cluster.dots.reserve(cluster.ptCnt);
		// Approximation for roughly circular objects, slightly overestimated
		cluster.edge.reserve(std::min(cluster.ptCnt, cluster.ptCnt/2 + 15));
	}
	for (int i = 0; i < blobTiles.size(); i++)
	{
		Tile *tile = &blobTiles[i];
		for (int x = 0; x < 8; x++)
		{
			for (int y = 0; y < 4; y++)
			{
				CompID compID = tile->compMap[x][y];
				if (compID == 0) continue; // Not in mask
				CompID clusterID = blobMerge[blobCompMerge[compID]];
				if (clusterID == 0) continue; // Blob was ignored
				Cluster *cluster = &blobs[clusterID-1];

				// Add dot to cluster
				Vector2<uint16_t> dot(tile->x * 8 + x + blobOffset.x(), tile->y * 4 + y + blobOffset.y());
				cluster->dots.push_back(dot);

				// Test for edge case
				bool isEdge = false;
				if (!isEdge)
				{
					if (x == 0) isEdge = (tile->l == InvalidTile) || (!blobTiles[tile->l].compMap[7][y]);
					else isEdge = !tile->compMap[x-1][y];
				}
				if (!isEdge)
				{
					if (x == 7) isEdge = (tile->r == InvalidTile) || (!blobTiles[tile->r].compMap[0][y]);
					else isEdge = !tile->compMap[x+1][y];
				}
				if (!isEdge)
				{
					if (y == 0) isEdge = (tile->t == InvalidTile) || (!blobTiles[tile->t].compMap[x][3]);
					else isEdge = !tile->compMap[x][y-1];
				}
				if (!isEdge)
				{
					if (y == 3) isEdge = (tile->b == InvalidTile) || (!blobTiles[tile->b].compMap[x][0]);
					else isEdge = !tile->compMap[x][y+1];
				}
				if (isEdge)
				{ // Add to edges array
					cluster->edge.push_back(dot);
				}

#ifndef CALCULATE_BOUNDS_EARLY
				cluster->bounds.include(Bounds2<uint16_t>(dot.x(), dot.y(), dot.x()+1, dot.y()+1));
#endif
			}
		}
	}
#endif

#ifdef BLOB_DEBUG
	printf("Found %d blobs from %d initial blobs!\n", blobs.size(), tempBlobs.size());
#endif
}

static inline CompID resolveComponentMerge(CompID *blobCompMerge, CompID compID)
{
	CompID id = blobCompMerge[compID];
	if (id != blobCompMerge[id])
	{ // Resolve recursive merge hierarchy
		do id = blobCompMerge[id];
		while (id != blobCompMerge[id]);
		// Update all components on the way
		CompID idIt = compID, idNxt;
		do
		{
			idNxt = blobCompMerge[idIt];
			blobCompMerge[idIt] = id;
			idIt = idNxt;
		}
		while (blobCompMerge[idIt] != id);
	}
	return id;
}

/* Background Calibration */

void BlobDetection::initBackgroundCalibration(uint32_t *bgBitmask)
{
	tempBackgroundMask.reset();
	backgroundMask = &tempBackgroundMask;

	if (bgBitmask)
	{ // Backup existing VPU-side background bitmask, then reset it
		memcpy(backupBGBitmask.data(), bgBitmask, bitmaskSize.prod());
		memset(bgBitmask, 0xFF, bitmaskSize.prod());
	}
}
void BlobDetection::resetBackgroundCalibration(uint32_t *bgBitmask)
{
	tempBackgroundMask.reset();
	storedBackgroundMask.reset();
	backgroundMask = &storedBackgroundMask;

	if (bgBitmask)
	{ // Reset both VPU-side background bitmasks
		memset(backupBGBitmask.data(), 0xFF, bitmaskSize.prod());
		memset(bgBitmask, 0xFF, bitmaskSize.prod());
	}
}
void BlobDetection::retryBackgroundCalibration(uint32_t *bgBitmask)
{
	tempBackgroundMask.reset();

	if (bgBitmask)
	{ // Reset used VPU-side background bitmask, but not backup
		memset(bgBitmask, 0xFF, bitmaskSize.prod());
	}
}
void BlobDetection::acceptBackgroundCalibration(uint32_t *bgBitmask)
{
	storedBackgroundMask = tempBackgroundMask;
	tempBackgroundMask.reset();
	backgroundMask = &storedBackgroundMask;

	if (bgBitmask)
	{ // Copy as backup of VPU-side background bitmask
		memcpy(backupBGBitmask.data(), bgBitmask, bitmaskSize.prod());
	}
}
void BlobDetection::discardBackgroundCalibration(uint32_t *bgBitmask)
{
	backgroundMask = &storedBackgroundMask;

	if (bgBitmask)
	{ // Restore backed up VPU-side background bitmask
		memcpy(bgBitmask, backupBGBitmask.data(), bitmaskSize.prod());
	}
}
void BlobDetection::updateBackgroundCalibration(uint32_t *bgBitmask, std::vector<uint8_t> &bgTiles)
{ // Add all tiles that had a blob on them to the background mask
	auto getTileIndex = [this](uint16_t x, uint16_t y) -> TileIndex
	{
		uint16_t blkX = x;
		uint16_t blkY = y / mskBlockTiles;
		uint16_t blkRow = y % mskBlockTiles;
		uint32_t blkID = blkY * blkW + blkX;
		return blkID * mskBlockTiles + blkRow;
	};
	for (int i = 0; i < blobTiles.size(); i++)
	{
		int bgIndex = (blobTiles[i].y/2)*tileW + blobTiles[i].x;
		if (!tempBackgroundMask.test(bgIndex))
		{
			tempBackgroundMask.set(bgIndex);
			bgTiles.push_back(blobTiles[i].x);
			bgTiles.push_back(blobTiles[i].y/2);
		}
		if (bgBitmask)
		{ // NOTE: Emulating current behaviour
			TileIndex tileIndexLo = getTileIndex(blobTiles[i].x, (blobTiles[i].y/2)+0);
			TileIndex tileIndexHi = getTileIndex(blobTiles[i].x, (blobTiles[i].y/2)+1);
			bgBitmask[tileIndexLo] = 0; // Remove entire block from bitmask
			bgBitmask[tileIndexHi] = 0; // Remove entire block from bitmask
		}
		/* if (bgBitmask)
		{ // NOTE: Potential future behaviour
			TileIndex tileIndex = getTileIndex(blobTiles[i].x, blobTiles[i].y);
			bgBitmask[tileIndex] &= ~blobTiles[i].bytes; // Remove bits from bitmask
		} */
	}
}
std::vector<uint8_t> BlobDetection::getTempBGTiles()
{
	std::vector<uint8_t> bgList;
	std::size_t pos = tempBackgroundMask._Find_first();
	while (pos != tempBackgroundMask.size())
	{
		bgList.push_back((uint8_t)(pos%tileW)); // x
		bgList.push_back((uint8_t)(pos/tileW)); // y
		pos = tempBackgroundMask._Find_next(pos);
	}
	return bgList;
}