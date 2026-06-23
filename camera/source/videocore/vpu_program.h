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

#ifndef VPU_PROGRAM_H
#define VPU_PROGRAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vc_base.h"

#include <stdint.h>

/* Program memory representation as stored in GPU memory (using VC_BUFFER).
 * Usually mapped to ARM space for access.
 * Stores all stuff the QPU needs to know in order to execute our QPU program. */
typedef struct VPU_PROGMEM {
    VC_PTR start; // Start of progmem
    VC_PTR code;
	uint32_t codeSize;
} VPU_PROGMEM;

/* VPU interface containing information, our buffers and program memory */
typedef struct VPU_PROGRAM {
	VC_BUFFER progmem_buffer; // progmem buffer in GPU memory
	VPU_PROGMEM progmem; // ARM side progmem mapped from GPU memory; need to lock buffer before access
} VPU_PROGRAM;

/* Initializes VPU program using base and given program code buffer. */
int vpu_initProgram(VPU_PROGRAM *program, VC_BASE *base, uint32_t codeSize);

/* Destroy VPU program and clean up resources */
void vpu_destroyProgram(VPU_PROGRAM *program);

/* VPU execute program using mailbox. */
unsigned int vpu_executeProgramMailbox (VPU_PROGRAM *program, VC_BASE *base, uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3, uint32_t r4, uint32_t r5);

/* Loads code from the file directly into program memory. */
int vpu_loadProgramCode(VPU_PROGRAM *program, const char *filename);

/* Returns file size of specified code file. Can be used to size program memory perfectly. */
unsigned int vpu_getCodeSize(const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* VPU_PROGRAM_H */
