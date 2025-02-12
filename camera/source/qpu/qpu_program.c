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

#include <stdio.h>
#include <unistd.h> // usleep
#include <string.h> // memset
#include <sched.h> // sched_yield (unused)

#include "qpu_program.h"
#include "qpu_info.h"

/* QPU Program
	Simple interface for a General purpose program

	After creation, you need to lock the progmem buffer and fill in the progmem code and uniforms
	Currently, exeuting on multiple GPUs will execute them all with the same uniforms
*/

/* Initializes QPU program using base. Provide progmem size requirements through memsize.
 * If program is a general purpose program, messageSize should be > 2 to accommodate for code and uniforms.
 * Else messageSize should be 0 as QPU V3D registers are used for execution instead of a mailbox message. */
int qpu_initProgram(QPU_PROGRAM *program, QPU_BASE *base, QPU_PROGMEM progmem)
{
	// Allocate GPU memory for progmem (code, uniforms, control lists, etc)
	uint32_t progmemSize = progmem.codeSize + progmem.uniformsSize * 4 + progmem.messageSize * 4;
	int status = qpu_allocBuffer(&program->progmem_buffer, base, progmemSize, 4096);
	if (status != 0) return status;
	// Although not currently locked, addresses are already valid and will stay the same

	// Store ARM and VC side addresses
	QPU_PTR pmstart = program->progmem.start = program->progmem_buffer.ptr;
	program->progmem.code = pmstart;
	program->progmem.codeSize = progmem.codeSize;
	program->progmem.uniforms.vc = pmstart.vc + progmem.codeSize;
	program->progmem.uniforms.arm.vptr = pmstart.arm.vptr + progmem.codeSize;
	program->progmem.uniformsSize = progmem.uniformsSize;
	program->progmem.message.vc = pmstart.vc + progmem.codeSize + progmem.uniformsSize*4;
	program->progmem.message.arm.vptr = pmstart.arm.vptr + progmem.codeSize + progmem.uniformsSize*4;
	program->progmem.messageSize = progmem.messageSize;

	// Lock progmem buffer to make base->progmem accessible
	qpu_lockBuffer(&program->progmem_buffer);

	// Reset progmem to all zeros
	memset(program->progmem.start.arm.vptr, 0x0, program->progmem_buffer.size);

	if (progmem.messageSize > 1)
	{ // Setup messages
		program->progmem.message.arm.uptr[0] = program->progmem.uniforms.vc;
		program->progmem.message.arm.uptr[1] = program->progmem.code.vc;
	}

	// Unlock progmem - base->progmem can't be accessed anymore
	qpu_unlockBuffer(&program->progmem_buffer);

	return 1;
}

/* Destroy QPU program and clean up resources */
void qpu_destroyProgram (QPU_PROGRAM *program)
{
	// Destroy progmem in GPU memory
	qpu_releaseBuffer(&program->progmem_buffer);
}

/* QPU execute program using direct access to QPU V3D registers. */
int qpu_executeProgramDirect (QPU_PROGRAM *program, QPU_BASE *base, int numInst, int unifLength, int unifStride, QPU_PerformanceState *perfState)
{
	base->peripherals[V3D_DBCFG] = 0; // Disallow IRQ
	base->peripherals[V3D_DBQITE] = 0; // Disable IRQ
	base->peripherals[V3D_DBQITC] = -1; // Resets IRQ flags

	// Enable and clear L2 cache
	base->peripherals[V3D_L2CACTL] = (1<<2);// | (1<<0);
	// Clear TMU-, uniform- and instructions cache
	base->peripherals[V3D_SLCACTL] = 0b1111<<24 | 0b1111<<16 | 0b1111<<8 | 0b1111<<0;

	// Note QPU user program numbers to determine when all our instances finished
	//int qpuQueued = (base->peripherals[V3D_SRQCS] & 0b111111);
	//int qpuFinished = (base->peripherals[V3D_SRQCS] >> 16) & 0xFF;
	//int qpuWaitCount = (qpuQueued + qpuFinished + numInst) % 256;
	//base->peripheral[V3D_SRQCS] = (1<<7) | (1<<8) | (1<<16); // Reset error bit and counts

//	if (qpuWaitCount < qpuFinished)
//		printf("QPU executing %d programs; waiting for %d with %d queued and %d already finished! \n", numInst, qpuWaitCount, qpuQueued, qpuFinished);

	int scheduler = base->peripherals[V3D_SRQCS];
	int queued = (scheduler & 0b111111), 
		requested = (scheduler >> 8) & 0b11111111,
		completed = (scheduler >> 16) & 0b11111111;
	if (queued > 0)
	{
		printf("QPU already has %d programs queued! \n", queued);
	}
	if (requested != completed)
	{
		printf("QPU already has %d programs requested, of which %d are completed! \n", requested, completed);
	}

	bool stalled = false;
	int retryCount = 0;
	long long cnt = 0;
	int executed = 0;
retry:
	// Clear Scheduler
	base->peripherals[V3D_SRQCS] = (1 << 16) | (1 << 8) | (1 << 7) | (1 << 0);

	for (int q = 0; q < numInst; q++)
	{
		cnt = 0;
		while (qpu_getUserProgramsQueued(base) >= 15)
		{ // Maximum number of queued requests reached - wait until queue has space - should only happen if numInst >= 15
			usleep(50);
			cnt++;
			if (cnt % 100 == 0)
			{
				if (q <= qpu_getUserProgramsQueued(base))
					printf("====== QPU User Program Queue has been modified by another program!\n");
				qpu_logErrors(base);
				if (perfState != NULL)
					qpu_updatePerformance(base, perfState);
	//			qpu_logStalls(base);
				if (cnt >= 1000)
				{
					printf("====== QPU stalled - queued %d / %d! \n", q, numInst);
					if (perfState != NULL) qpu_logPerformance(perfState);
					return -1;
				}
			}
		}
		// Queue new program instance
		base->peripherals[V3D_SRQUA] = program->progmem.uniforms.vc + q * unifStride * 4;
		base->peripherals[V3D_SRQUL] = unifLength;
		base->peripherals[V3D_SRQPC] = program->progmem.code.vc;
	}

	if (qpu_getUserProgramsQueueError(base))
	{
		printf("====== Failed to queue all %d programs, %d requested, only %d queued and %d already processed!\n",
			numInst, qpu_getUserProgramsRequested(base), qpu_getUserProgramsQueued(base), qpu_getUserProgramsCompleted(base));
		return -1;
	}

	if (qpu_getUserProgramsRequested(base) != numInst)
	{
		printf("====== Failed to request all %d programs, only %d requested, with %d still queued and %d already processed!\n",
			numInst, qpu_getUserProgramsRequested(base), qpu_getUserProgramsQueued(base), qpu_getUserProgramsCompleted(base));
		return -1;
	}

	// Wait for all instances to complete
	cnt = 0;
	while ((executed = qpu_getUserProgramsCompleted(base)) < numInst)
	{
		usleep(50);
		cnt++;
		if (cnt % 10 == 0)
		{ // ever 500us
			if (perfState != NULL)
				qpu_updatePerformance(base, perfState);
		}
		if (cnt >= 50 && cnt % 4 == 0)
		{ // every 0.2ms after 2.5ms
			stalled = true;
			printf("====== QPU stalled (try %d, %d) - submitted %d(%d) programs, %d still queued, %d processed!\n", retryCount, (int)cnt/10,
				numInst, qpu_getUserProgramsRequested(base), qpu_getUserProgramsQueued(base), qpu_getUserProgramsCompleted(base));
			qpu_logErrors(base);
			if (perfState != NULL) qpu_logPerformance(perfState);
			//qpu_logStalls(base);
		}
		if (cnt >= 60)
		{ // Retry completely after 3.0ms
			retryCount++;
			if (retryCount <= 3)
			{ // Try 3 times normally - seems to work
				printf("====== Clearing QPU queue to try to recover from stall and retrying!\n");
				goto retry;
			}
			if (retryCount <= 4)
			{ // Try once more with QPU reset, nuclear, not directly observed to be necessary or even to work
				qpu_enable(base->mb, 0);
				printf("====== Disabling and reenabling QPU to try to recover from stall and retrying!\n");
				qpu_enable(base->mb, 1);
				goto retry;
				
			}
			printf("====== Giving up QPU program after retrying 5 times!\n");
			return -1;
		}
	}

	if (stalled)
	{
		printf("== Managed to escape stall!\n");
	}

	// Clear Scheduler
	base->peripherals[V3D_SRQCS] = (1 << 16) | (1 << 8) | (1 << 7) | (1 << 0);

	return 0;
}

/* QPU execute general purpose program using mailbox. */
int qpu_executeProgramMailbox (QPU_PROGRAM *program, QPU_BASE *base, int numQPUs)
{
	return execute_qpu(base->mb, numQPUs, program->progmem.message.vc, QPU_NO_FLUSH, QPU_TIMEOUT);
}

int qpu_executeProgram (QPU_PROGRAM *program, QPU_BASE *base, int numQPUs)
{
	int retCode;
	if (program->progmem.messageSize < 2)
		retCode = qpu_executeProgramDirect(program, base, numQPUs, program->progmem.uniformsSize, 0, NULL);
	else
		retCode = qpu_executeProgramMailbox(program, base, numQPUs);

	return retCode;
}

/* Copies code from supplied buffer into program memory. */
void qpu_setProgramCode(QPU_PROGRAM *program, const char *code, unsigned int length)
{
	qpu_lockBuffer(&program->progmem_buffer);
	memcpy(&program->progmem.code.arm.cptr, code, length);
	qpu_unlockBuffer(&program->progmem_buffer);
}

/* Loads code from the file directly into program memory. */
int qpu_loadProgramCode(QPU_PROGRAM *program, const char *filename)
{
	FILE *file = fopen(filename, "rb");
	fseek(file, 0, SEEK_END);
	unsigned int fileLength = ftell(file);
	if (fileLength > program->progmem.codeSize)
	{
		printf("Unable to fit %d bytes of code into the %d bytes of code buffer!", fileLength, program->progmem.codeSize);
		fclose(file);
		return -1;
	}
	rewind(file);

	qpu_lockBuffer(&program->progmem_buffer);
	fread(program->progmem.code.arm.cptr, fileLength, 1, file);
	qpu_unlockBuffer(&program->progmem_buffer);
	fclose(file);
	return 0;
}

/* Returns file size of specified code file. Can be used to fit progmem code size perfectly. */
unsigned int qpu_getCodeSize(const char *filename)
{
	FILE *file = fopen(filename, "rb");
	fseek(file, 0, SEEK_END);
	unsigned int fileLength = ftell(file);
	fclose(file);
	return fileLength;
}
