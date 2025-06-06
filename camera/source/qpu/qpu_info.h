/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef QPU_DEBUG_H
#define QPU_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

#include "qpu_base.h"

/* QPU Info and Debugging
	Functions to read out QPU state and information and handle interrupts
*/


typedef struct QPU_UserProgramInfo {
	uint8_t QPURQCC, QPURQCM, QPURQL;
	uint16_t VPMURSV, VPMURSV_V;
	int QPURQERR;
} QPU_UserProgramInfo;

typedef struct QPU_3DPipelineInfo {
	bool BMOOM, RMBUSY, RMACTIVE, BMBUSY, BMACTIVE;
	uint8_t BMFCT, RMFCT;
} QPU_3DPipelineInfo;

typedef struct QPU_3DInterrupts {
	bool INT_SPILLUSE, INT_OUTOMEM, INT_FLDONE, INT_FRDONE;
} QPU_3DInterrupts;

typedef struct QPU_HWIdent {
	uint8_t TVER;
	char IDSTR[4];
} QPU_HWIdent;
typedef struct QPU_HWConfiguration {
	uint16_t VPMSZ, VPMSZ_V;
	bool HDRT, TLBDB;
	uint8_t NSEM, TUPS, QUPS, NSLC, QPU_NUM, REV;
	uint8_t TLBSZ, TLB_X, TLB_Y, VRISZ;
} QPU_HWConfiguration;
typedef struct QPU_PerformanceState {
	uint32_t rawLast[11];
	uint64_t rawAccum[11];
	uint64_t diffAccum[11];
	uint32_t tempSOC, clockRateMin, clockRateMax;
	uint8_t qpusUsed;
} QPU_PerformanceState;
typedef struct QPU_Performance {
	uint64_t clkIdle, clkVert, clkFrag, clkInst;
	uint64_t clkTMUStall, numTMUProc, numTMUMiss;
	uint64_t clkVDWStall, clkVCDStall;
	uint64_t numL2CHits, numL2CMiss;
} QPU_Performance;

static inline uint8_t qpu_getUserProgramsCompleted(QPU_BASE *base)
{ // QPURQCC
	return (base->peripherals[V3D_SRQCS] >> 16) & 0xFF;
}
static inline uint8_t qpu_getUserProgramsRequested(QPU_BASE *base)
{ // QPURQCM
	return (base->peripherals[V3D_SRQCS] >> 8) & 0xFF;
}
static inline uint8_t qpu_getUserProgramsQueued(QPU_BASE *base)
{ // QPURQL
	return (base->peripherals[V3D_SRQCS] >> 0) & 0x3F;
}
static inline bool qpu_getUserProgramsQueueError(QPU_BASE *base)
{ // QPURQERR
	return (base->peripherals[V3D_SRQCS] >> 7) & 0b1;
}

void qpu_getUserProgramInfo(QPU_UserProgramInfo *info, QPU_BASE *base);
void qpu_get3DPipelineInfo(QPU_3DPipelineInfo *info, QPU_BASE *base);
bool qpu_getL2CacheState(QPU_BASE *base);

int qpu_getQPUInterruptFlags(QPU_BASE *base);
void qpu_resetQPUInterruptFlags(QPU_BASE *base);
void qpu_get3DInterrupts(QPU_3DInterrupts *info, QPU_BASE *base);

void qpu_getHWIdent(QPU_HWIdent *debug, QPU_BASE *base);
void qpu_getHWConfiguration(QPU_HWConfiguration *info, QPU_BASE *base);
void qpu_debugHW(QPU_BASE *base);

int qpu_getReservationSetting(QPU_BASE *base, int qpu);
void qpu_setReservationSetting(QPU_BASE *base, int qpu, int set);
void qpu_logReservationSettings(QPU_BASE *base);

void qpu_setupPerformanceCounters(QPU_BASE *base, QPU_PerformanceState *state);
void qpu_updatePerformance(QPU_BASE *base, QPU_PerformanceState *state);
void qpu_readPerformanceState(uint64_t *perfData, QPU_Performance *perf);
void qpu_logPerformance (QPU_PerformanceState *state);

int qpu_logErrors(QPU_BASE *base);
int qpu_logStalls(QPU_BASE *base);

#ifdef __cplusplus
}
#endif

#endif
