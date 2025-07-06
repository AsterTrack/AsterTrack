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

#include "qpu_base.h"

/* QPU Access Base
	Provides easy access to all functionality of the QPU.

	Create a QPU access base using QPU_initBase and QPU_destroyBase.
	It provides direct access to the QPU V3D registers using the peripheral map.
	It also contains required information about the host.

	Contains a GPU buffer implementation for passing information buffers to the QPU.

	Requires sudo privileges for memmap (mailbox.h)
*/

/* Allocate the buffer of desired size in GPU memory */
int qpu_allocBuffer(QPU_BUFFER *buffer, QPU_BASE *base, uint32_t size, uint32_t align)
{
	buffer->mb = base->mb;
	if (size > 50000000 || size == 0)
	{ // Likely error
		buffer->size = 0;
		return -3;
	}
	buffer->size = size;

	// Allocate GPU memory
	buffer->handle = mem_alloc(buffer->mb, buffer->size, align, base->host.mem_flg);
	if (!buffer->handle) return -1;

	// Get VC adress - stays constant over lifetime
	buffer->ptr.vc = mem_lock(buffer->mb, buffer->handle);
	mem_unlock(buffer->mb, buffer->handle);

	// Map into ARM space and get ARM adress - stays constant over lifetime
	buffer->ptr.arm.vptr = mapmem(BUS_TO_PHYS(buffer->ptr.vc + base->host.mem_map), buffer->size);
	if (!buffer->ptr.arm.vptr)
	{ // Release GPU buffer
		mem_free(buffer->mb, buffer->handle);
		return -2;
	}

	return 0;
}

/* Lock buffer to make buffer->ptr->arm.*ptr accessible */
void qpu_lockBuffer(QPU_BUFFER *buffer)
{
	//buffer->ptr.vc == mem_lock(buffer->mb, buffer->handle));
	mem_lock(buffer->mb, buffer->handle);
}

/* Unlock buffer to make buffer->ptr->arm.*ptr inaccessible */
void qpu_unlockBuffer(QPU_BUFFER *buffer)
{
	mem_unlock(buffer->mb, buffer->handle);
}

/* Unmap buffer from ARM side and release buffer in GPU memory */
void qpu_releaseBuffer(QPU_BUFFER *buffer)
{
	// Unmap ARM memory
	unmapmem(buffer->ptr.arm.vptr, buffer->size);
	// Free GPU memory
	mem_free(buffer->mb, buffer->handle);
}


/* Initializes QPU access base */
int qpu_initBase(QPU_BASE *base)
{
	// Get host information (peripheral adresses, etc)
	if (qpu_getHostInformation(&base->host)) return -1;

	// Map peripheral registers for direct QPU access
	base->peripherals = (volatile uint32_t *)mapmem(base->host.peri_addr, base->host.peri_size);
	if (!base->peripherals) return -2;

	// Open mailbox for GPU memory allocation
	base->mb = mbox_open();
	if (base->mb <= 0)
	{
		unmapmem((void *)base->peripherals, base->host.peri_size);
		return -3;
	}
	return 0;
}

/* Destroy QPU access base and clean up resources */
void qpu_destroyBase (QPU_BASE *base)
{
	// Unmap peripheral registers
    unmapmem((void*)base->peripherals, base->host.peri_size);
}

static unsigned get_dt_ranges(const char *filename, unsigned offset)
{
	unsigned address = ~0;
	FILE *fp = fopen(filename, "rb");
	if (fp)
	{
		unsigned char buf[4];
		fseek(fp, offset, SEEK_SET);
		if (fread(buf, 1, sizeof buf, fp) == sizeof buf)
			address = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0;
		fclose(fp);
	}
	return address;
}

/* Get board-specific information about the QPU host integration */
int qpu_getHostInformation(QPU_HOST *host)
{
	/*
	Copyright (c) 2015, Andrew Holme.
	Copyright (c) 2012-2014, Broadcom Europe Ltd
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
		* Redistributions of source code must retain the above copyright
		notice, this list of conditions and the following disclaimer.
		* Redistributions in binary form must reproduce the above copyright
		notice, this list of conditions and the following disclaimer in the
		documentation and/or other materials provided with the distribution.
		* Neither the name of the copyright holder nor the
		names of its contributors may be used to endorse or promote products
		derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	*/

	// Pi 1 defaults
	host->peri_addr = 0x20000000;
	host->peri_size = 0x01000000;
	host->mem_flg = QPU_USE_VC4_L2_CACHE? 0xC : 0x4;
	host->mem_map = QPU_USE_VC4_L2_CACHE? 0x0 : 0x20000000; // Pi 1 only

	unsigned sdram_addr = get_dt_ranges("/proc/device-tree/axi/vc_mem/reg", 8);
	if (sdram_addr == ~0)
		sdram_addr = 0x40000000;
	if (sdram_addr != 0x40000000)
	{ // Pi 2?
		host->mem_flg = 0x4; // ARM cannot see VC4 L2 on Pi 2
		host->mem_map = 0x0;
	}

	host->peri_addr = get_dt_ranges("/proc/device-tree/soc/ranges", 4);
	host->peri_size = get_dt_ranges("/proc/device-tree/soc/ranges", host->peri_addr == 0? 12 : 8);
	if (host->peri_addr == 0)
		host->peri_addr = get_dt_ranges("/proc/device-tree/soc/ranges", 8);
	if (host->peri_addr == ~0)
		host->peri_addr = 0x20000000;
	if (host->peri_size == ~0)
		host->peri_size = 0x01000000;
	return 0;
}
