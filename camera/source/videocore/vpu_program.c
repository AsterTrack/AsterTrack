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

#include "vpu_program.h"
#include "mailbox.h"

#include <stdio.h>
#include <elf.h>

/* Initializes VPU program using base and given program code buffer. */
int vpu_initProgram(VPU_PROGRAM *program, VC_BASE *base, uint32_t codeSize)
{
	// Allocate GPU memory for progmem (code)
	uint32_t progmemSize = codeSize;
	int status = vc_allocBuffer(&program->progmem_buffer, base, progmemSize, 4096);
	if (status != 0) return status;
	// Although not currently locked, addresses are already valid and will stay the same

	// Store ARM and VC side addresses
	VC_PTR pmstart = program->progmem.start = program->progmem_buffer.ptr;
	program->progmem.code = pmstart;
	program->progmem.codeSize = codeSize;

	return 0;
}

/* Destroy VPU program and clean up resources */
void vpu_destroyProgram(VPU_PROGRAM *program)
{
	// Destroy progmem in GPU memory
	vc_releaseBuffer(&program->progmem_buffer);
}

/* VPU execute program using mailbox. */
unsigned int vpu_executeProgramMailbox(VPU_PROGRAM *program, VC_BASE *base, uint32_t r0, uint32_t r1, uint32_t r2, uint32_t r3, uint32_t r4, uint32_t r5)
{
    return execute_code(base->mb, program->progmem.code.vc, r0, r1, r2, r3, r4, r5);
}

/* Loads code from the file directly into program memory. */
int vpu_loadProgramCode(VPU_PROGRAM *program, const char *filename)
{
	FILE *file = fopen(filename, "rb");
	if (!file)
	{
		printf("Could not find source file '%s'!", filename);
		return -2;
	}
	fseek(file, 0, SEEK_END);
	unsigned int fileLength = ftell(file);
	if (fileLength > program->progmem.codeSize)
	{
		printf("Unable to fit %d bytes of code into the %d bytes of code buffer!", fileLength, program->progmem.codeSize);
		fclose(file);
		return -1;
	}
	rewind(file);

	vc_lockBuffer(&program->progmem_buffer);
    fread(program->progmem.code.arm.cptr, fileLength, 1, file);
	vc_unlockBuffer(&program->progmem_buffer);
	fclose(file);
	return 0;
}

/* Returns file size of specified code file. Can be used to size program memory perfectly. */
unsigned int vpu_getCodeSize(const char *filename)
{
	FILE *file = fopen(filename, "rb");
	if (!file)
		return 0;
	fseek(file, 0, SEEK_END);
	unsigned int fileLength = ftell(file);
	fclose(file);
	return fileLength;
}
