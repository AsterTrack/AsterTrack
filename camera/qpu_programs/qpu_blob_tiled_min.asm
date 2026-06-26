# AsterTrack Optical Tracking System
# Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

.include "vc4.qinc"

# See qpu_blob_tiled.hpp to see explanation of layout across image

# Uniforms
.set srcPtr, ra0
.set tgtPtr, ra1
.set srcStride, rb0
.set tgtStride, rb1
.set blockIter, ra2			# Iterator over blocks (20 lines)
.set lookBehind, ra3
.set thresholdCO8, rb4
.set diffCO8, ra4
mov srcPtr, unif;
mov tgtPtr, unif;
mov srcStride, unif;
mov tgtStride, unif;
mov blockIter, unif;			# block count
mov ra5.8888i, unif;
mov diffCO8.8888i, unif;
mov thresholdCO8, ra5;
mov lookBehind, unif;			# whether to include two lines before srcPtr in minimums

# Variables
.set mskAccum, r3

# Memory Setup Constants
.set vpmSetup, rb2
.set vdwSetup, rb3

# Mask constants
.set sh1, ra5
.set sh2, ra6
.set sh3, ra7
.set sh2B, rb5 # Same as sh2, but in register file B for optimisation
.set num4, rb6
.set bitMax, rb7

# Define variables storing the current headers
# valReg stores the values of all 12 columns up until 2 rows ago
# So that the 5x5 minimum finished in any given row can be used with the values in its center 
# valReg(0,x) is the current row, valReg(2,x) is two rows ago (center of min area)
# Uses 9(12) registers from ra20 to ra31
.func valReg(y, x)
	.assert b < 5 && b >= 0
	.assert l < 4 && l >= 0
	.assert y <= 2 && y >= 0
	.assert x <= 2 && x >= 0
	ra20 + (((b*4 + (l-y)) * 3 + x + 12) % 12)  # +12 since input to % might be negative
.endf
# minReg stores the column-wise minimum of 12 columns
# Such that minReg(2,x) is the minimum of this and the last row
# And minReg(5,x) is the minimum of all last 5 rows
# For convenience minReg(1,x) is mapped to valReg(1,x), but it is in another register file!
# Uses 12 registers from rb20 to rb32
.func minReg(n, x)
	.assert n <= 5  && n >= 1
	.assert x <= 2 && x >= 0
	.if n == 1
		valReg(n, x)
	.else
		rb20 + ((n-2) * 3 + x)
	.endif
.endf

# For each row, the mask is calculated for the central 8 columns of the 12 columns
# minReg(5,x) are shifted and combined with each other to produce 8 different 5x5 min values
# Each 5x5 min value is centered around the pixels two rows ago
# The central value is thus stored in valReg(2,x) for that row of 12 pixels

# TODO: Generate vector mask to allow for any multiple of 8-wide columns (not just 16x8)

# Calculate base source of each tile column
mul24 r0, elem_num, 8;
add srcPtr, srcPtr, r0;

# Look behind 2 lines if desired (to preload 4, not 2, for proper 5x5min on first line)
									; mov r0, srcStride;
and.setf -, lookBehind, lookBehind	; mul24 r0, r0, 2;
sub.ifzc srcPtr, srcPtr, r0;

# Set mask constants
ldi sh2, 16;
ldi sh3, 24;
mov sh1, 8; mov sh2B, sh2;
mov num4, 4;
ldi bitMax, 0x01010101;

# Start loading 8/12 bytes of very first line
mov t0s, srcPtr; add srcPtr, srcPtr, 4;
nop;
mov t0s, srcPtr;

# Create VPM Setup
; mov r1, 5;
mul24 r1, qpu_num, r1;
ldi r0, vpm_setup(0, 1, h32(0));
add vpmSetup, r0, r1;

# Create VPM DMA Basic setup
shl r1, r1, 7; # Same VPM position, different place in register
ldi r0, vdw_setup_0(16, 5, dma_v32(0, 0));
add vdwSetup, r0, r1;

# Adjust values
; mov r0, 8;
sub srcStride, srcStride, r0; # Remove 2 x 4bytes read in each iteration before jumping to the next line

# Init defaults - ONLY needed if lookBehind is not used (so 2 preloaded lines, not all 4)
.lset b, 4 # Set block used in preload loop (5 ensures first real loop with b=0 accesses the right registers)
.lset l, 2 # Set first line used in non-lookBehind preload loop which is the only one this is needed for
mov r0, -1; # Max for min AND val to ensure 5x5min stays max and thus inert
mov valReg(1,0), r0;
mov valReg(1,1), r0;
mov valReg(1,2), r0;

# Jump if only preloading two lines, not looking back for 2 more
; and.setf -, lookBehind, lookBehind;
brr.anyz -, :skipLinePreload

# Finish initialisation during the 3 branching delay slots
mov valReg(0,0), r0;	mov minReg(2,0), r0;
mov valReg(0,0), r0;	mov minReg(2,1), r0;
mov valReg(0,0), r0;	mov minReg(2,2), r0;

.macro processHalf, b, l, cl, cr, ic, in0
	# b: block index, l: line index
	# cl: column left, cr: column right
	.assert cl <= 2 && cl >= 0
	.assert cr <= 2 && cr >= 0
	# ic: interleaving column (with n fixed as 2 & 3)
	#     relies on r4 still being populated from last load
	# in0: n of interleaving column 0, either 2 or 3
	#     will read the value from register valReg(0,0)

	# Calculate 5x5 min for given four pixels
										shl r1, minReg(5,cr), sh3;
										shr r0, minReg(5,cl), sh1;
	v8adds r0, r0, r1; 					shl r1, minReg(5,cr), sh2;
	v8min r2, r0, minReg(5,cl);			shr r0, minReg(5,cl), sh2;
	v8adds r0, r0, r1; 					shl r1, minReg(5,cr), sh1;
	v8min r2, r0, r2;					shr r0, minReg(5,cl), sh3;
	v8adds r0, r0, r1; 					v8min r2, r2, minReg(5,cr);
	v8min r2, r0, r2;

	# Compute mask for 4 pixels at once
	# 0 if value < min+diff and value < threshold
										;shr r1, valReg(2,cl), sh2B;
										shl r0, valReg(2,cr), sh2B;
	v8adds r2, r2, diffCO8;
		; v8min minReg(3,ic), minReg(2,ic), r4;
	v8min r2, r2, thresholdCO8; 		v8adds r0, r0, r1;
	v8subs r2, r0, r2;
		; v8min minReg(2,ic), minReg(1,ic), r4;

	# Write bit 0 to mask iff r2 accum byte is 0
	v8min r2, r2, bitMax;
		; mov r0, valReg(0,0);
	shl r2, r2, l*2+cl; # Mask Pos from 0-7
		# This reads from either ra or rb and thus MAY conflict with above small immediate and not merge
		; v8min minReg(in0,0), minReg(in0-1,0), r0;

.endm

.macro loadQueueMinBlock, pos, stride
	# Wait for load of current column (pos) values and start next queued load (stride)
	# Update column-wise minimum values for the relevant 4 columns
	# Two of those done here, the other two interleaved in processHalf
	; add srcPtr, srcPtr, stride; ldtmu0
	mov valReg(0,pos), r4;				v8min minReg(5,pos), minReg(4,pos), r4;
	mov t0s, srcPtr;					v8min minReg(4,pos), minReg(3,pos), r4;
.endm

# Emulate END of full loop to use the right registers backing minReg and valReg (they loop)
.lset b, 4
.rep l, 4 # First 4 Lines of 8Bits each
	# Handled separately to ensure first line written is actually valid
	# Since it needs the full 5x5 min, program writes mask of pixels loaded two lines ago
	# So at the top of the image, just pre-load the first two lines and calculate minimum values
	# IF we are in the middle of the image, we may load two previous lines, too
	# This is controlled by a uniform, and it may jump here to only load two:
	.if l == 2
		:skipLinePreload
	.endif

	# Load for column 0, queue current line column 2
	loadQueueMinBlock 0, num4
	v8min minReg(3,0), minReg(2,0), r4;
	v8min minReg(2,0), minReg(1,0), r4;

	# Load for column 1, queue next line column 0
	loadQueueMinBlock 1, srcStride
	v8min minReg(3,1), minReg(2,1), r4;
	v8min minReg(2,1), minReg(1,1), r4;

	# Load for column 2, queue next line column 1
	loadQueueMinBlock 2, num4
	v8min minReg(3,2), minReg(2,2), r4;
	v8min minReg(2,2), minReg(1,2), r4;

.endr

# Initiate mask accumulator for first iteration
mov mskAccum, 0;

:blockIter # Loop over blocks

	# Initiate VPM write and make sure last VDW finished
	read vw_wait; mov vw_setup, vpmSetup;

	.rep b, 5 # 5 Blocks of 32Bits each

		.rep l, 4 # 4 Lines of 8Bits each

			# Each loadQueueMinBlock updates minReg 4 & 5 of its column, but not 2 & 3 (see above)
			# So 3 columns with 2 minRegs need to be interleaved in remaining code
			# Column 2 & 3 are interleaved in processHalf right after their load, with r4 still populated
			# Column 0 is split up into both and needs to load the value from valReg(0,0) again
			# Last two parameters of processHalf concerns that interleaving (ic, in0)

			# Load for column 0, queue current line column 2
			loadQueueMinBlock 0, num4

			# Load for column 1, queue next line column 0
			loadQueueMinBlock 1, srcStride

			# Processes left 4 pixels covering column 0 and 1
			# Interleaves minReg update 3 for col 0 and both 3&2 for col 1 (last load)
			processHalf b, l,   0, 1,   1, 3

			# Accumulate mask
			v8adds mskAccum, mskAccum, r2;

			# Load for column 2, queue next line column 1
			loadQueueMinBlock 2, num4

			# Processes right 4 pixels covering column 1 and 2
			# Interleaves minReg update 2 for col 0 and both 3&2 for col 2 (last load)
			processHalf b, l,   1, 2,   2, 2

			.if l == 3 # Write to VPM
				v8adds vpm, mskAccum, r2; mov mskAccum, 0;
			.else # Continue accumulating
				v8adds mskAccum, mskAccum, r2;
			.endif

		.endr

	.endr

	# Initiate VDW from VPM to memory
	mov vw_setup, vdwSetup;
	ldi vw_setup, vdw_setup_1(0);
	mov vw_addr, tgtPtr;

	# Increase adress to next line
	;add tgtPtr, tgtPtr, tgtStride;

	# Line loop :blockIter
	sub.setf blockIter, blockIter, 1;
	brr.allnz -, :blockIter
	nop
	nop
	nop

# Read last two unused lines (outside of bounds)
ldtmu0
ldtmu0

mov.setf irq, nop;

nop; thrend
nop
nop