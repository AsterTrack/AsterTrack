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

#ifndef MASKING_H
#define MASKING_H

#include "qpu_cores_masking.hpp"

#include "blob/qpu_blob_tiled.hpp"

#include "videocore/vc_base.h"
#include "videocore/qpu_info.h"
#include "videocore/qpu_program.h"
#include "videocore/vpu_program.h"


struct ExclusiveQPU
{
	bool enabled;

	VC_BASE base;
	QPU_PerformanceState perf;
	int qpuCoresUsed;

	ExclusiveQPU(VC_BASE &base, const QPUCoreMasking &cores, int numThreads, bool log);
	~ExclusiveQPU();

	inline operator bool() { return enabled; }
};

struct MaskingProgram
{
	bool initialised;

	ProgramLayout layout;

	QPU_PROGRAM blobProgram;

	static const int BitmaskCount = 3;
	VC_BUFFER bitmskBuffer[BitmaskCount];
	int bitmskSwitch = 0;

	MaskingProgram() : initialised(false) {};
	MaskingProgram(VC_BASE &base, ProgramLayout layout, const std::string &codeFile);
	~MaskingProgram();

	MaskingProgram(const MaskingProgram&) = delete;
	MaskingProgram& operator= (const MaskingProgram&) = delete;

	void SetParameters(uint8_t thresholdCO, uint8_t diffCO);

	unsigned int Execute(VC_BASE &base, QPU_PerformanceState &perfState, uint32_t srcStride, uint32_t sourcePtrVC);

	inline operator bool() { return initialised; }
};

struct FetchingProgram
{
	bool initialised;
	Vector2<int> maskSize;
	uint32_t tileCount;
	uint32_t maskIndexSize;

	VPU_PROGRAM vpuProgram;
	VC_BUFFER maskIndexBuffer, bgBitmaskBuffer;

	FetchingProgram() : initialised(false) {}
	FetchingProgram(VC_BASE &base, Vector2<int> maskSize, const std::string &codeFile);
	~FetchingProgram();

	FetchingProgram(const FetchingProgram&) = delete;
	FetchingProgram& operator= (const FetchingProgram&) = delete;

	unsigned int Execute(VC_BASE &base, uint32_t bitmaskPtrVC);

	inline operator bool() { return initialised; }
};

#endif // MASKING_H