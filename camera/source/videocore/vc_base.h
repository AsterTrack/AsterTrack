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

#ifndef VC_BASE_H
#define VC_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

/* VideoCore Access Base
	Provides easy access to all functionality of the VideoCore IV.
	
	Create a VideoCore access base using vc_initBase and vc_destroyBase.
	It provides direct access to the QPU V3D registers using the peripheral map.
	It also contains required information about the host.
	
	Contains a buffer implementation for passing information buffers to the VideoCore hardware.
	
	Requires sudo privileges for memmap (mailbox.h)
*/

#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

#define VC_USE_VC4_L2_CACHE	1				// Use L2 Cache or not

/* Dual pointer representation in VideoCore and ARM space */
typedef struct VC_PTR {
	uint32_t vc;
	union {
		void *vptr;
		char *cptr;
		float *fptr;
		uint32_t *uptr;
	} arm;
} VC_PTR;

/* Host information about the VideoCore host which can vary between RaspberryPi models */
typedef struct VC_HOST {
	uint32_t mem_flg;
	uint32_t mem_map;
	uint32_t peri_addr;
	uint32_t peri_size;
} VC_HOST;

/* Generic buffer in GPU memory, allocated through mailbox */
typedef struct VC_BUFFER {
	uint32_t mb; // mailbox handle
	uint32_t handle;
	uint32_t size;
	VC_PTR ptr;
} VC_BUFFER;

/* VideoCore interface containing host information and direct VideoCore access through peripheral registers */
typedef struct VC_BASE {
	VC_HOST host;
	volatile uint32_t *peripherals; // Registers of peripherals for direct VideoCore register access
	uint32_t mb;
} VC_BASE;

/* Get board-specific information about the VideoCore host integration */
int vc_getHostInformation(VC_HOST *host);

/* Allocate the buffer of desired size in GPU memory */
int vc_allocBuffer(VC_BUFFER *buffer, VC_BASE *base, uint32_t size, uint32_t align);
/* Lock buffer to make buffer->ptr->arm.*ptr accessible */
void vc_lockBuffer(VC_BUFFER *buffer);
/* Unlock buffer to make buffer->ptr->arm.*ptr inaccessible */
void vc_unlockBuffer(VC_BUFFER *buffer);
/* Unmap buffer from ARM side and release buffer in GPU memory */
void vc_releaseBuffer(VC_BUFFER *buffer);

/* Initializes VideoCore access base */
int vc_initBase(VC_BASE *base);
/* Destroy VideoCore access base and clean up resources */
void vc_destroyBase (VC_BASE *base);


#ifdef __cplusplus
}
#endif

#endif
