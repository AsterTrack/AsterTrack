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

#ifndef VCSM_H
#define VCSM_H

#include <cstdint>

struct VCSM_BUFFER
{
	int32_t fd = -1;
	uint32_t size = 0;
	void *mem = nullptr;
	uint32_t VCHandle = 0;
	uint32_t VCMem = 0;
};

bool vcsm_init();

void vcsm_exit();

VCSM_BUFFER vcsm_malloc(uint32_t size);

void vcsm_free(VCSM_BUFFER &alloc);

bool vcsm_lock(VCSM_BUFFER &alloc);

bool vcsm_unlock(VCSM_BUFFER &alloc);

#endif // VCSM_H
