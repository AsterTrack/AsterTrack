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

#include "masking.hpp"

#include "videocore/mailbox.h"
#include "videocore/qpu_registers.h"
#include "blob/vpu_find_active_tiles.hpp"

MaskingProgram::MaskingProgram(VC_BASE &base, ProgramLayout layout, const std::string &codeFile)
	: layout(layout)
{
	initialised = false;

	// Set up bit target, one bit per pixel
	for (int i = 0; i < BitmaskCount; i++)
	{
		if (vc_allocBuffer(&bitmskBuffer[i], &base, layout.maskSize.prod()/8, 4096))
		{
			for (int j = i-1; j >= 0; j--)
				vc_releaseBuffer(&bitmskBuffer[j]);
			return;
		}
	}

	// Setup main blob detection program with specified progmem sizes
	if (qpu_initProgram(&blobProgram, &base, (QPU_PROGMEM){
		.codeSize = qpu_getCodeSize(codeFile.c_str()),
		.uniformsSize = (uint32_t)layout.instances*numUnif,
		.messageSize = 2
	}))
	{
		for (int i = 0; i < BitmaskCount; i++)
			vc_releaseBuffer(&bitmskBuffer[i]);
		return;
	}

	if (qpu_loadProgramCode(&blobProgram, codeFile.c_str()))
	{
		qpu_destroyProgram(&blobProgram);
		for (int i = 0; i < BitmaskCount; i++)
			vc_releaseBuffer(&bitmskBuffer[i]);
		return;
	}

	initialised = true;

	// Set up uniforms of the blob QPU program
	vc_lockBuffer(&blobProgram.progmem_buffer);
	uint32_t *uniforms = blobProgram.progmem.uniforms.arm.uptr;
	uint32_t tgtLineStride = layout.maskSize.x()/8;
	uint32_t tgtBlockStride = tgtLineStride * blockLines;
	for (int c = 0; c < layout.columns; c++)
	{
		for (int r = layout.rows.size()-1; r >= 0; r--)
		{ // Set up each program instance with their column
			int baseUnif = (r*layout.columns+c)*numUnif;
			ProgramLineSpans &lines = layout.rows[r];
			uniforms[baseUnif + 0] = 0; // Enter source pointer each frame
			uniforms[baseUnif + 1] = 0; // Enter target pointer each frame
			uniforms[baseUnif + 2] = 0; // Enter source stride each frame
			uniforms[baseUnif + 3] = tgtBlockStride;
			uniforms[baseUnif + 4] = lines.inCount/blockLines; // Block count to process
			uniforms[baseUnif + 5] = 255; // absolute threshold
			uniforms[baseUnif + 6] = 255; // difference threshold
		}
	}
	vc_unlockBuffer(&blobProgram.progmem_buffer);
}

MaskingProgram::~MaskingProgram()
{
	if (!initialised) return;
	qpu_destroyProgram(&blobProgram);
	for (int i = 0; i < BitmaskCount; i++)
		vc_releaseBuffer(&bitmskBuffer[i]);
	printf("-- QPU Cleaned --\n");
}

void MaskingProgram::SetParameters(uint8_t thresholdCO, uint8_t diffCO)
{
	vc_lockBuffer(&blobProgram.progmem_buffer);
	uint32_t *uniforms = blobProgram.progmem.uniforms.arm.uptr;
	for (int c = 0; c < layout.columns; c++)
	{
		for (int r = layout.rows.size()-1; r >= 0; r--)
		{ // Set up each program instance with their column
			// TODO: Allow for thresholds to be calibrated per-tile and pass as chain of uniforms
			// Would differ based on LED coverage and lens vignetting / relative illumination
			// Would likely be very useful to have, but would need some kind of automatic calibration as well
			int baseUnif = (r*layout.columns+c)*numUnif;
			uniforms[baseUnif + 5] = thresholdCO;
			uniforms[baseUnif + 6] = diffCO;
		}
	}
	vc_unlockBuffer(&blobProgram.progmem_buffer);
}

unsigned int MaskingProgram::Execute(VC_BASE &base, QPU_PerformanceState &perfState, uint32_t srcStride, uint32_t sourcePtrVC)
{
	VC_BUFFER &bitmskBuf = bitmskBuffer[bitmskSwitch];

	// ---- Uniform preparation ----

	vc_lockBuffer(&blobProgram.progmem_buffer);
	uint32_t *uniforms = blobProgram.progmem.uniforms.arm.uptr;
	uint32_t tgtLineStride = layout.maskSize.x()/8;
	// Set up individual source pointer for each program instance
	for (int c = 0; c < layout.columns; c++)
	{
		for (int r = layout.rows.size()-1; r >= 0; r--)
		{ // Set up each program instance with their column
			int baseUnif = (r*layout.columns+c)*numUnif;
			ProgramLineSpans &lines = layout.rows[r];
			uint32_t srcX = layout.srcOffset.x() + c*qpuCoreWidth;
			uint32_t srcY = layout.srcOffset.y() + lines.inStart;
			uniforms[baseUnif + 0] = sourcePtrVC + srcX + srcY*srcStride;
			uint32_t tgtX = c*(qpuThreadCount*blockMaskBytes);
			uint32_t tgtY = lines.inStart; // Is aligned to blockLines
			uniforms[baseUnif + 1] = bitmskBuf.ptr.vc + tgtX + tgtY*tgtLineStride;
			uniforms[baseUnif + 2] = srcStride;
		}
	}
	vc_unlockBuffer(&blobProgram.progmem_buffer);

	// ---- Program execution ----

	// Lock bitmask buffer
	//vc_lockBuffer(&bitmskBuf);

	unsigned int code = qpu_executeProgramDirect(&blobProgram, &base, layout.instances, numUnif, numUnif, &perfState);

	// Unlock bitmask buffer
	//vc_unlockBuffer(&bitmskBuf);

	return code;
}

ExclusiveQPU::ExclusiveQPU(VC_BASE &base, const QPUCoreMasking &cores, int numThreads, bool log)
	: base(base)
{
	// Enable QPU - this locks the QPU for our use and prevents other use (e.g. camera AWB, OpenGL ES)
	// We could do this setup on every frame and then disable/release the QPU after we are done
	// See https://github.com/raspberrypi/firmware/issues/793
	enabled = !qpu_enable(base.mb, 1);
	if (!enabled) return;
	printf("-- QPU Enabled --\n");

	// QPU scheduler reservation
	qpuCoresUsed = cores.getUsed();
//	for (int i = 0; i < 12; i++) // Enable only QPUs selected as parameter, disable others completely
//		qpu_setReservationSetting(&base, i, cores.enabled[i]? 0b1110 : 0b1111);
	for (int i = 0; i < 12; i++) // Enable only QPUs selected as parameter, allow GL shaders on others
		qpu_setReservationSetting(&base, i, cores.enabled[i]? 0b1110 : 0b0001);

	if (log)
	{
		qpu_logReservationSettings(&base);

		// Debug QPU Hardware
		qpu_debugHW(&base);

		// VPM memory reservation
		base.peripherals[V3D_VPMBASE] = 16; // times 4 to get number of vectors; Default: 8 (32/4), Max: 16 (64/4)
		QPU_HWConfiguration hwConfig;
		qpu_getHWConfiguration(&hwConfig, &base);
		QPU_UserProgramInfo upInfo;
		qpu_getUserProgramInfo(&upInfo, &base);
		printf("Reserved %d / %d vectors of VPM memory for user programs!\n", upInfo.VPMURSV_V, hwConfig.VPMSZ_V);
	}

	// Setup performance monitoring
	qpu_setupPerformanceCounters(&base, &perf);
	perf.qpusUsed = std::min(qpuCoresUsed, numThreads);
}

ExclusiveQPU::~ExclusiveQPU()
{
	for (int i = 0; i < 12; i++) // Reset all QPUs to be freely sheduled
		qpu_setReservationSetting(&base, i, 0b0000);

	// Disable QPU
	if (qpu_enable(base.mb, 0))
		printf("-- QPU Disable Failed --\n");
	else
		printf("-- QPU Disabled --\n");
}

FetchingProgram::FetchingProgram(VC_BASE &base, Vector2<int> maskSize, const std::string &codeFile)
	: maskSize(maskSize)
{
	initialised = false;

	tileCount = maskSize.prod()/32;
	if (ROUND_DOWN(tileCount, 5*VPU_INDEX_TILES) != tileCount)
	{ // VPU expects it to be divisible by 16*5. Layout should ensure that
		printf("Specific blocking expected by VPU program is not met!\n");
		return;
	}
	unsigned int ret;

	// Setup main blob detection program with specified progmem sizes
	ret = vpu_initProgram(&vpuProgram, &base, vpu_getCodeSize(codeFile.c_str()));
	if (ret)
	{
		printf("Failed to init program: %d\n", ret);
		return;
	}
	ret = vpu_loadProgramCode(&vpuProgram, codeFile.c_str());
	if (ret)
	{
		printf("Failed to load program code: %d\n", ret);
		vpu_destroyProgram(&vpuProgram);
		return;
	}

	// Indices of active regions into blob bitmask, size according to worst case of full bitmask
	maskIndexSize = getVPUMaskIndexBufferSize(maskSize.prod());
	ret = vc_allocBuffer(&maskIndexBuffer, &base, maskIndexSize, 4096);
	if (ret)
	{
		printf("Failed to allocate maskIndexBuffer: %d\n", ret);
		vpu_destroyProgram(&vpuProgram);
		return;
	}
	maskIndexBuffer.ptr.arm.cptr[maskIndexSize-1] = 0xFF;
	maskIndexBuffer.ptr.arm.cptr[maskIndexSize-2] = 0xFF;
	maskIndexBuffer.ptr.arm.cptr[maskIndexSize-3] = 0xFF;
	maskIndexBuffer.ptr.arm.cptr[maskIndexSize-4] = 0xFF;

	// Bitmask for background masking corresponding to exact size of blob bitmask 
	ret = vc_allocBuffer(&bgBitmaskBuffer, &base, maskSize.prod()/8, 4096);
	if (ret)
	{
		printf("Failed to allocate bgBitmaskBuffer: %d\n", ret);
		vc_releaseBuffer(&maskIndexBuffer);
		vpu_destroyProgram(&vpuProgram);
		return;
	}
	memset(bgBitmaskBuffer.ptr.arm.cptr, 0xFF, maskSize.prod()/8);

	initialised = true;
	printf("-- VPU Program Setup --\n");
}

FetchingProgram::~FetchingProgram()
{
	if (!initialised) return;
	vc_releaseBuffer(&bgBitmaskBuffer);
	vc_releaseBuffer(&maskIndexBuffer);
	vpu_destroyProgram(&vpuProgram);
	printf("-- VPU Cleaned --\n");
}

unsigned int FetchingProgram::Execute(VC_BASE &base, uint32_t bitmaskPtrVC)
{
	int fetchMaskProgram = 1;
	return vpu_executeProgramMailbox(&vpuProgram, &base, fetchMaskProgram,
		bitmaskPtrVC, bgBitmaskBuffer.ptr.vc, maskIndexBuffer.ptr.vc, tileCount, 0);
}