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

#include "visualisation.hpp"

#include "util/fbUtil.h"

#include <unistd.h>

struct VisualisationImpl
{
	int fbfd = -1;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
};

VisualisationState::VisualisationState()
{
	impl = new VisualisationImpl();
	impl->fbfd = setupFrameBuffer(&impl->vinfo, &impl->finfo, false);
	initialised = impl->fbfd >= 0;
	if (!initialised)
		printf("Failed to initialise visualisation with framebuffer!\n");
}

VisualisationState::~VisualisationState()
{
	if (impl && impl->fbfd >= 0)
		close(impl->fbfd);
}

void VisualisationState::visualise(const std::vector<Cluster> &blobs, const std::vector<Cluster> &pastBlobs, uint8_t *srcBuf, int srcWidth, int srcHeight, int srcStride)
{
	if (impl->fbfd < 0)
		return;
	void *fbp = lock_fb(impl->fbfd, impl->finfo.smem_len);
	if ((intptr_t)fbp == -1)
	{
		printf("Failed to mmap framebuffer!\n");
		return;
	}

	// Determine visualization resolution and mapping (fit into maximum window)
	int vizH = std::min((uint32_t)width, impl->vinfo.xres), vizV = std::min((uint32_t)height, impl->vinfo.yres);
	int dH = std::min((int)srcWidth, vizH), dV = std::min((int)srcHeight, vizV);
	dH = std::min(dV*srcWidth/srcHeight, vizH);
	dV = std::min(dH*srcHeight/srcWidth, vizV);

	// Set display position with offset away from display edge
	uint8_t strideX = impl->vinfo.bits_per_pixel/8;
	uint8_t *dsp = (uint8_t*)fbp + 50*impl->finfo.line_length + 50*strideX;

	if (displayFrame)
	{ // Write grayscale image, subsampled to target res
		for (int y = 0; y < dV; y++)
		{
			int pxY = y * srcHeight/dV;
			for (int x = 0; x < dH; x++)
			{
				int pxX = x * srcWidth/dH;
				uint32_t val = srcBuf[pxY*srcStride+pxX];
				uint8_t *px = dsp + y*impl->finfo.line_length + x*strideX;
				*(uint32_t*)px = (val << 16) | (val << 8) | val | 0xFF000000;
			}
		}
	}

	if (displayBlobs)
	{
		if (!displayFrame)
		{ // Erase old blobs
			for (int c = 0; c < pastBlobs.size(); c++)
			{
				const Cluster *cluster = &pastBlobs[c];
				for (int p = 0; p < cluster->dots.size(); p++)
				{
					int x = cluster->dots[p].x(), y = cluster->dots[p].y();
					int tgtX = x * dH / srcWidth, tgtY = y * dV / srcHeight;
					uint8_t *px = dsp + tgtY*impl->finfo.line_length + tgtX*strideX;
					*(uint32_t*)px = 0xFF000000;
				}
			}
		}

		// Write new blobs
		for (int c = 0; c < blobs.size(); c++)
		{
			const Cluster *cluster = &blobs[c];
			for (int p = 0; p < cluster->dots.size(); p++)
			{
				int x = cluster->dots[p].x(), y = cluster->dots[p].y();
				int tgtX = x * dH / srcWidth, tgtY = y * dV / srcHeight;
				uint8_t *px = dsp + tgtY*impl->finfo.line_length + tgtX*strideX;
				*(uint32_t*)px = 0xFF000000 + (c&1? 0x0000FF00 : 0x00FF0000);
			}
		}
	}

	unlock_fb(fbp, impl->finfo.smem_len);
}

VisualisationLock VisualisationState::lockWrite()
{
	VisualisationLock lock = {};
	if (impl->fbfd < 0)
		return lock;
	lock.lock = lock_fb(impl->fbfd, impl->finfo.smem_len);
	if ((intptr_t)lock.lock == -1)
	{
		printf("Failed to mmap framebuffer!\n");
		return lock;
	}
	// Set display position with offset away from display edge
	lock.strideX = impl->vinfo.bits_per_pixel/8;
	lock.strideY = impl->finfo.line_length;
	lock.length = impl->finfo.smem_len;
	lock.display = (uint8_t*)lock.lock + 50*lock.strideY + 50*lock.strideX;
	return lock;
}

VisualisationLock::~VisualisationLock()
{
	if (!display) return;
	unlock_fb(lock, length);
}