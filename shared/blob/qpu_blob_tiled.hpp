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

#include "../util/eigendef.hpp"
#include "util/log.hpp"

#include <stdint.h>
#include <algorithm>
#include <vector>
#include <cassert>


#define ROUND_DOWN(VAL, ROUND) (VAL)/(ROUND)*(ROUND)
//#define ROUND_DOWN(VAL, ROUND) (VAL) - (VAL)%(ROUND)

#define ROUND_UP(VAL, ROUND) ((VAL)+(ROUND-1))/(ROUND)*(ROUND)
//#define ROUND_UP(VAL, ROUND) (VAL) + (ROUND) - (VAL)%(ROUND)


struct ProgramLineSpans
{
	// Lines input into the program
	// Since the same number are witten to mask, inCount has to be a multiple of blockLines
	uint_fast16_t inStart, inCount;
	// Lines this program is intended to handle
	// Might write more valid lines before this as an overlap
	uint_fast16_t outStart, outCount;
	// All lines are mapped to mask and image as follows:
	// - Map to mask with line + maskTop
	// - Map to image with line + layout.srcOffset.y()
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
const int numUnif = 7;

// QPU program does a 5x5min calculation for 8pixels on a 12pixels wide input line
// It cannot deal with out-of-bounds pixels there, so we need to add horizontal padding
// In total, 2 pixels on each horizontal side are implicitly consumed
constexpr uint32_t paddingX = 2;

// Similarly, the first two rows written are invalid in order to initialise the 5x5 minimum calculation
// The next two rows have a valid center pixels, but the 5x5min calculation is not fully done
// Sadly, it's going to be near impossible to prevent writing of those first 2 or 4 rows, so we have to deal with that
// In more detail, the first rows processed by a program are as follows:
	// Row 0: Build 5x5min (1), write invalid row -2	(center value 0, incomplete 5x5min)
	// Row 1: Build 5x5min (2), write invalid row -1	(center value 0, incomplete 5x5min)
	// Row 2: Build 5x5min (3), write incomplete row 0	(incomplete 5x5min)
	// Row 3: Build 5x5min (4), write incomplete row 1	(incomplete 5x5min)
	// Row 4: Build 5x5min (5), write valid row 2
// The program handles out-of-bound pixels at the top of the image as follows:
// Row 0&1 are invalid (usually 1 in bitmask) and need special handling to ignore them and offset the mask
// If row 2&3 are the top edge of the image, then the incomplete 5x5 min doesn't matter and they can be safely used
constexpr uint32_t maskInvalidTop = 2;
constexpr uint32_t maskIncompleteTop = 4;
// The program cannot deal with out of bounds pixels at the bottom edge though
// The last two lines are only used for the 5x5min calculation up to the third to last line:
	// Row n-3: Build 5x5min, write row n-5
	// Row n-2: Build 5x5min, write row n-4
	// Row n-1: Build 5x5min, write row n-3
// So the last two image rows are never written to the mask, the last row written is the third to last image row and it is valid
constexpr uint32_t maskMissingBottom = 2;
// Finally, since columns are split vertically, programs may start or end within the image, complicating things
// When Line a to b are processed:
// - Lines a-2 to b-2 are written
// - Lines a+2 to b-2 are valid (a to b-2 only when at the very top)
// As such, a certain internal overlap is required (maskIncompleteTop and maskMissingBottom)

// Each QPU thread operates on 8 central pixel columns (12 columns total for the 5x5min calculation)
constexpr uint32_t blockWidth = 8;

// Each QPU Core is 16-way, so 16 such threads operate in lockstep
constexpr uint32_t qpuThreadCount = 16;

// Currently, the QPU program can not mask a thread to prevent it from writing
// So the total width in pixels that we process needs to be multiples of this
constexpr uint32_t qpuCoreWidth = blockWidth*qpuThreadCount;
// TODO: Allow for masking of qpu threads so we don't need to throw remaining horizontal pixels away
// However, currently the main limit is that only 9 QPUs can run this program, so a 10th wouldn't help much anyway
// But if we DID have 10 available, masking would allow for 1272 width instead of current 1152 (1276 max since we need 2 padding on each side)
// But arguably, the extra width doesn't add FoV where we really need more anyway, so not super useful

// Internally, each program is an unrolled loop that processes 20 lines at once
// Currently, the QPU program can not mask any of the lines
// So the total height in pixels that we process needs to be multiples of this
constexpr uint32_t blockLines = 20; // 4*BLK_TILES

// The mask is written block-by-block, with each pixel of the block being one bit
constexpr uint32_t blockMaskBytes = blockWidth*blockLines/8;

static inline ProgramLayout SetupProgramLayout(uint32_t width, uint32_t height, uint32_t cores, bool debug = true)
{
	ProgramLayout layout = {};

	/**
	* Processing dimensions 
	*/

	// Split image in columns that a single QPU core can process
	layout.columns = (width - paddingX*2) / qpuCoreWidth;

	// Calculate processed pixels in the output mask
	layout.maskSize = Vector2<int>(layout.columns*qpuCoreWidth, ROUND_DOWN(height, blockLines));

	/**
	* Vertical distribution
	*/

	// Split columns vertically until most or all QPUs are used
	int splitCols = 1;
	while (layout.columns * (splitCols+1) <= cores)
		splitCols++;
	layout.instances = layout.columns * splitCols;

	// Calculate line spans fed into of each row of QPU programs so that the lines in the output masks are all valid
	// This is purposefully verbose to allow verification of mapping
	layout.rows.resize(splitCols);
	uint32_t startLine = 0;
	for (int i = 0; i < splitCols; i++)
	{ // Accumulatively split up columns vertically
		ProgramLineSpans &prog = layout.rows[i];
		// Determine lines input to program (valid will be inStart to inStart+inCount-2)
		uint32_t clipTop = i > 0? maskIncompleteTop : 0;
		prog.inStart = ROUND_DOWN(startLine - clipTop, blockLines);
		uint32_t progRemaining = splitCols - i;
		uint32_t linesRemaining = height - prog.inStart;
		prog.inCount = ROUND_DOWN(linesRemaining/progRemaining, blockLines);
		// Determine the lines handled by the program (without overlap that was already handled) 
		prog.outStart = startLine;
		// Alternate form 1:
		uint32_t topOverlap = startLine - prog.inStart;
		prog.outCount = prog.inCount - topOverlap - maskMissingBottom;
		// Alternate form 2 (equally valid):
		//uint32_t validEnd = prog.inStart+prog.inCount - maskMissingBottom;
		//prog.outCount = validEnd - prog.outStart;
		startLine += prog.outCount;
	}

	// The strategy to ensure a continuous mask relies on the column to be calculated bottom-to-top
	// Since the first 2-4 rows written by a program are garbage, but the last 2 rows written are good
	// So in order to get correct values in the vertical overlap, the bottom program has to execute first
	// This is achieved by iterating over the split columns like this:
	// for (int r = splitCols-1; r >= 0; r--)

	// Calculate lines input to the program (with some invalid in the output)
	uint32_t maskLines = layout.rows.back().inStart+layout.rows.back().inCount;
	assert(layout.maskSize.y() == maskLines);
	// Calculate lines actually processed and available later
	uint32_t validLines = layout.rows.back().outStart+layout.rows.back().outCount;
	assert(layout.maskSize.y() == validLines+maskMissingBottom);
	uint32_t validWidth = layout.maskSize.x();

	/**
	* Aligning & Centering
	*/

	// Due to restrictions, some pixels will have been dropped on the edges
	uint32_t droppedColumns = width - validWidth;
	uint32_t droppedLines = height - validLines;
	assert(droppedColumns >= paddingX*2);
	assert(droppedLines >= maskMissingBottom);
	
	// The QPU address (without the implicit paddingX) needs to be 4-byte aligned
	layout.srcOffset.x() = ROUND_DOWN(droppedColumns/2 - paddingX, 4);
	layout.maskOffset.x() = layout.srcOffset.x() + paddingX;

	// Center valid lines vertically
	layout.srcOffset.y() = std::min(droppedLines/2, height - layout.maskSize.y());
	layout.maskOffset.y() = layout.srcOffset.y() - maskInvalidTop;

	/**
	* Debug output
	*/

	// Determine the image space rect for which a valid mask exists
	layout.validMaskRect.minX = layout.srcOffset.x() + paddingX;
	layout.validMaskRect.minY = layout.srcOffset.y();
	layout.validMaskRect.maxX = layout.validMaskRect.minX + validWidth;
	layout.validMaskRect.maxY = layout.validMaskRect.minY + validLines;

	if (debug)
	{
		LOG(LCameraEmulation, LInfo,
			"SETUP: %d instances processing 1/%d columns each, covering %dx%d pixels total (%d x %d dropped, offset %d x %d)\n",
			layout.instances, splitCols, validWidth, validLines,
			droppedColumns, droppedLines, layout.validMaskRect.minX, layout.validMaskRect.minY);
	}
	// With this, a 1280x800 image becomes 1152x798 of effectively processed pixels. Not optimal but fastest way

	return layout;
}

static inline void SetupProgramUniforms(ProgramLayout &layout, uint32_t *uniforms, uint32_t srcStride)
{
	uint32_t tgtLineStride = layout.maskSize.x()/8;
	uint32_t tgtBlockStride = tgtLineStride * blockLines;
	for (int c = 0; c < layout.columns; c++)
	{
		for (int r = layout.rows.size()-1; r >= 0; r--)
		{ // Set up each program instance with their column
			ProgramLineSpans &lines = layout.rows[r];
			uniforms[(r*layout.columns+c)*numUnif + 0] = 0; // Enter source pointer each frame
			uniforms[(r*layout.columns+c)*numUnif + 1] = 0; // Enter target pointer each frame
			uniforms[(r*layout.columns+c)*numUnif + 2] = srcStride;
			uniforms[(r*layout.columns+c)*numUnif + 3] = tgtBlockStride;
			uniforms[(r*layout.columns+c)*numUnif + 4] = lines.inCount/blockLines; // Block count to process
			uniforms[(r*layout.columns+c)*numUnif + 5] = 255; // absolute threshold
			uniforms[(r*layout.columns+c)*numUnif + 6] = 255; // difference threshold
		}
	}
}

static void SetupProgramSettings(ProgramLayout &layout, uint32_t *uniforms, uint8_t thresholdCO, uint8_t diffCO)
{
	for (int c = 0; c < layout.columns; c++)
	{
		for (int r = layout.rows.size()-1; r >= 0; r--)
		{ // Set up each program instance with their column
			// TODO: Allow for thresholds to be calibrated per-tile and pass as chain of uniforms
			// Would differ based on LED coverage and lens vignetting / relative illumination
			// Would likely be very useful to have, but would need some kind of automatic calibration as well
			uniforms[(r*layout.columns+c)*numUnif + 5] = thresholdCO;
			uniforms[(r*layout.columns+c)*numUnif + 6] = diffCO;
		}
	}
}

static inline void SetupProgramBuffers(ProgramLayout &layout, uint32_t *uniforms, uint32_t srcStride, uint32_t sourcePtrVC, uint32_t targetPtrVC)
{
	uint32_t tgtLineStride = layout.maskSize.x()/8;
	
	// Set up individual source pointer for each program instance
	for (int c = 0; c < layout.columns; c++)
	{
		for (int r = layout.rows.size()-1; r >= 0; r--)
		{ // Set up each program instance with their column
			ProgramLineSpans &lines = layout.rows[r];
			uint32_t srcX = layout.srcOffset.x() + c*qpuCoreWidth;
			uint32_t srcY = layout.srcOffset.y() + lines.inStart;
			uniforms[(r*layout.columns+c)*numUnif + 0] = sourcePtrVC + srcX + srcY*srcStride;
			uint32_t tgtX = c*(qpuThreadCount*blockMaskBytes);
			uint32_t tgtY = lines.inStart; // Is aligned to blockLines
			uniforms[(r*layout.columns+c)*numUnif + 1] = targetPtrVC + tgtX + tgtY*tgtLineStride;
		}
	}
}

#endif // QPU_BLOB_TILED_H