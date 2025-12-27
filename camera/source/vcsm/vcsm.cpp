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

#include "vcsm.hpp"

// Metadata for IOCTLs used
#include "dma-buf.h"
#include "vc_sm_cma_ioctl.h"

#include <cstdio>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>

static unsigned int vcsm_fd = -1;
static unsigned int pagesize;

bool vcsm_init()
{
	vcsm_fd = open("/dev/vcsm-cma", O_RDWR, 0);
	pagesize = getpagesize();

	return vcsm_fd >= 0;
}

void vcsm_exit()
{
	close(vcsm_fd);
	vcsm_fd = -1;
}

VCSM_BUFFER vcsm_malloc(uint32_t size)
{
	VCSM_BUFFER alloc = {};
	if (size == 0)
	{
		printf("vcsm_malloc: size is 0!\n");
		return alloc;
	}
	if (vcsm_fd < 0)
	{
		printf("vcsm_malloc: VCSM uninitialised!\n");
		return alloc;
	}

	uint32_t size_aligned = (size + pagesize - 1) & ~(pagesize - 1);

	struct vc_sm_cma_ioctl_alloc cma_alloc;
	cma_alloc.size = size_aligned;
	cma_alloc.num = 1;
	cma_alloc.cached = VC_SM_CMA_CACHE_NONE;
	cma_alloc.handle = -1;
	if (ioctl(vcsm_fd, VC_SM_CMA_IOCTL_MEM_ALLOC, &cma_alloc) < 0)
	{
		printf("vcsm_malloc: ioctl failed!\n");
		return alloc;
	}
	if (cma_alloc.handle < 0)
	{
		printf("vcsm_malloc: no allocation!\n");
		return alloc;
	}

	// Map the allocated VCSM Buffer into userspace memory
	alloc.mem = mmap(0, cma_alloc.size, PROT_READ | PROT_WRITE, MAP_SHARED, cma_alloc.handle, 0);
	if (!alloc.mem)
	{
		printf("vcsm_malloc: failed to map to userspace memory!\n");
	  	close(cma_alloc.handle);
		return alloc;
	}

	alloc.fd = cma_alloc.handle;
	alloc.size = size_aligned;
	alloc.VCHandle = cma_alloc.vc_handle;
	alloc.VCMem = cma_alloc.dma_addr;
	if (cma_alloc.dma_addr & 0xFFFFFFFF00000000ULL)
	{
		printf("vcsm_malloc: VC memory address invalid!\n");
		alloc.VCMem = 0;
	}
	return alloc;
}

void vcsm_free(VCSM_BUFFER &alloc)
{
	if (munmap(alloc.mem, alloc.size) < 0)
	{
		printf("vcsm_free: failed to unmap userspace memory!\n");
	}
	close(alloc.fd);

	alloc.fd = -1;
	alloc.size = 0;
}

bool vcsm_lock(VCSM_BUFFER &alloc)
{
	struct dma_buf_sync sync;
	sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
	if (ioctl(alloc.fd, DMA_BUF_IOCTL_SYNC, &sync) < 0)
	{
		printf("vcsm_lock: ioctl failed!\n");
		return false;
	}
	return true;
}

bool vcsm_unlock(VCSM_BUFFER &alloc)
{
	struct dma_buf_sync sync;
	sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
	if (ioctl(alloc.fd, DMA_BUF_IOCTL_SYNC, &sync) < 0)
	{
		printf("vcsm_unlock: ioctl failed!\n");
		return false;
	}
	return true;
}