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

#include "qpu/qpu_base.h"
#include "qpu/qpu_info.h"
#include "qpu/qpu_program.h"


struct ExclusiveQPU
{
	bool enabled;

	QPU_BASE base;
	QPU_PerformanceState perf;
	int qpuCoresUsed;

	ExclusiveQPU(QPU_BASE &base, const QPUCoreMasking &cores, int numThreads, bool log);
	~ExclusiveQPU();

	inline operator bool() { return enabled; }
};

struct MaskingProgram
{
	bool initialised;

	ProgramLayout layout;

	QPU_PROGRAM blobProgram;

	static const int BitmaskCount = 3;
	QPU_BUFFER bitmskBuffer[BitmaskCount];
	int bitmskSwitch = 0;

	MaskingProgram() : initialised(false) {};
	MaskingProgram(QPU_BASE &base, ProgramLayout layout, const std::string &codeFile);
	~MaskingProgram();

	MaskingProgram(const MaskingProgram&) = delete;
	MaskingProgram& operator= (const MaskingProgram&) = delete;

	void SetParameters(uint8_t thresholdCO, uint8_t diffCO);

	unsigned int Execute(QPU_BASE &base, QPU_PerformanceState &perfState, uint32_t srcStride, uint32_t sourcePtrVC);

	inline operator bool() { return initialised; }
};

#endif // MASKING_H