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

#ifndef QPU_BLOB_TILED_H
#define QPU_BLOB_TILED_H

#include "util/eigendef.hpp"

#include <stdint.h>
#include <vector>
#include <cassert>


#define ROUND_DOWN(VAL, ROUND) (VAL)/(ROUND)*(ROUND)
#define ROUND_UP(VAL, ROUND) ((VAL)+(ROUND-1))/(ROUND)*(ROUND)


struct ProgramLineSpans
{
	// Lines handled by the program
	// lineCount is a multiple of blockLines
	uint_fast16_t lineStart, lineCount;
	// All lines are mapped to mask and image as follows:
	// - Map to mask with line + maskOffset.y()
	// - Map to image with line + srcOffset.y()
};

struct ProgramLayout
{
	Vector2<int> maskSize;
	Vector2<int> maskOffset;
	Vector2<int> srcOffset;
	Bounds2i validMaskRect;

	int instances;
	int columns;
	std::vector<ProgramLineSpans> rows;
};

// Constant properties of the QPU blob detection program
const int numUnif = 8;

// QPU program does a 5x5min calculation for 8pixels on a 12pixels wide input line
// It cannot deal with out-of-bounds pixels on the sides, so horizontal padding is required
// Since image address needs to be 4-byte aligned, this constrains where that padding ends up being
constexpr uint32_t mskPaddingX = 2;

// Each QPU thread operates on 8 central pixel columns (12 columns total for the 5x5min calculation)
constexpr uint32_t mskBlockWidth = 8;

// Each QPU Core is 16-way, so 16 such threads operate in lockstep
constexpr uint32_t qpuThreadCount = 16;

// Currently, the QPU program can not mask a thread to prevent it from writing
// So the total width in pixels that we process is a multiple of this
constexpr uint32_t mskColumnWidth = mskBlockWidth*qpuThreadCount;
// TODO: Allow for masking of qpu threads so we don't need to throw remaining horizontal pixels away
// However, currently the main limit is that only 9 QPUs can run this program, so a 10th wouldn't help much anyway
// But if we DID have 10 available, masking would allow for 1272 width instead of current 1152 (1276 max since we need 2 padding on each side)
// But arguably, the extra width doesn't add FoV where we really need more anyway, so not super useful

// The program is an unrolled loop that processes 20 lines at once, related to the looping dynamics of the registers
// Internally, each such block is made up of 5 32-bit tiles, each in turn made up of 4 8-bit lines
constexpr uint32_t mskBlockTiles = 5;
// Since it can not mask any lines, the total height that is process is a multiple of this
constexpr uint32_t mskBlockLines = mskBlockTiles * 4;

// The mask is written block-by-block, with each pixel of the block being one bit
constexpr uint32_t mskBlockBytes = mskBlockWidth*mskBlockLines/8;

// Accessors for 8x4 tile encoded in 32bit integer
#define DOTID(X, Y) ((((X)%4)*8)+(((X)>>2)&1)+((Y)*2))
#define DOT(BYTES, X, Y) (uint8_t)(((BYTES) >> DOTID(X,Y)) & 1)


// Additionally, the processing may be split vertically
// So program may start and end on lines within the image
// But due to the 5x5 mask, when the program processes a line, it actually writes the line two lines above

// These are the relevant considerations for correctness:
// For the first two rows of the image, the program outputs a valid mask with incomplete 5x5min (2 rows outside are assumed to be 0xFF)
// For programs started in the middle of the image, it preloads 2 rows above the source address (triggered by lookBehind uniform)
// This allows it to output a valid mask from the very first line in any scenario
// For the last two rows of each program, it implicitly reads beyond the intended source range
// This implies the image needs to be padded with two trailing rows of 0xFF for a valid mask in the last two rows of an image
// For programs ending within in the image, this implicitly results in a valid mask if those pixels are already available

static inline ProgramLayout SetupProgramLayout(uint32_t width, uint32_t height, uint32_t cores, bool debug = true)
{
	ProgramLayout layout = {};

	/**
	* Processing dimensions 
	*/

	// Split image in columns that a single QPU core can process
	layout.columns = (width - mskPaddingX*2) / mskColumnWidth;
	// TODO: Allow processing of full width and just mark padding in mask as invalid

	// Calculate processed pixels in the output mask
	layout.maskSize = Vector2<int>(layout.columns*mskColumnWidth, ROUND_DOWN(height, mskBlockLines));

	/**
	* Vertical distribution
	*/

	// Split columns vertically until most or all QPUs are used
	int splitCols = 1;
	while (layout.columns * (splitCols+1) <= cores)
		splitCols++;
	// TODO: Allow deliberate splitting to minimize latencies
	layout.instances = layout.columns * splitCols;

	// Calculate line spans fed into of each row of QPU programs so that the lines in the output masks are all valid
	// This is purposefully verbose to allow verification of mapping
	layout.rows.resize(splitCols);
	uint32_t lineCount = 0;
	for (int i = 0; i < splitCols; i++)
	{ // Accumulatively split up columns vertically
		ProgramLineSpans &prog = layout.rows[i];
		// Determine lines input to program (valid will be inStart to inStart+inCount-2)
		prog.lineStart = lineCount;
		uint32_t progRemaining = splitCols - i;
		uint32_t linesRemaining = height - prog.lineStart;
		prog.lineCount = ROUND_DOWN(linesRemaining/progRemaining, mskBlockLines);
		lineCount += prog.lineCount;
	}
	assert(layout.maskSize.y() == lineCount);

	// Denote pixels actually processed and available later
	uint32_t validWidth = layout.maskSize.x(); // TODO: May not hold when padding is allowed outside of image bounds
	uint32_t validLines = layout.maskSize.y();

	/**
	* Aligning & Centering
	*/

	// Due to restrictions, some pixels may have been dropped on the edges
	uint32_t droppedColumns = width - validWidth;
	uint32_t droppedLines = height - validLines;
	assert(droppedColumns >= mskPaddingX*2);
	assert(droppedLines == 0); // Should not be necessary anymore
	
	// The QPU address (without the implicit paddingX) needs to be 4-byte aligned
	// With droppedColumns == 0, paddingX == 2 the alignment of 4 forces us to drop 4 columns anyway
	// We can choose whether these happen on the left (srcOffset.x == -4) or right (srcOffset.x == 0)
	// Either way, after dropping them, the valid mask rect IS centered with two columns on each side missing
	layout.srcOffset.x() = ROUND_DOWN(std::max<int>(0, droppedColumns/2 - mskPaddingX), 4);
	layout.maskOffset.x() = layout.srcOffset.x() + mskPaddingX;

	// Center valid lines vertically
	layout.srcOffset.y() = std::min(droppedLines/2, height - layout.maskSize.y());
	layout.maskOffset.y() = layout.srcOffset.y();

	/**
	* Debug output
	*/

	// Determine the image space rect for which a valid mask exists
	// As of now, this should be equal to maskOffset, but validWidth may be lower in the future
	layout.validMaskRect.min.x() = layout.srcOffset.x() + mskPaddingX;
	layout.validMaskRect.min.y() = layout.srcOffset.y();
	layout.validMaskRect.max.x() = layout.validMaskRect.min.x() + validWidth;
	layout.validMaskRect.max.y() = layout.validMaskRect.min.y() + validLines;

#ifdef LOG
	if (debug)
	{
		LOG(LCameraBlob, LInfo,
			"SETUP: %d instances processing 1/%d columns each, covering %dx%d pixels total (%d x %d dropped, offset %d x %d)",
			layout.instances, splitCols, validWidth, validLines,
			droppedColumns, droppedLines, layout.validMaskRect.min.x(), layout.validMaskRect.min.y());
	}
#endif
	// With this, a 1280x800 image becomes 1152x800 of effectively processed pixels. Not optimal but fastest way

	return layout;
}

#endif // QPU_BLOB_TILED_H