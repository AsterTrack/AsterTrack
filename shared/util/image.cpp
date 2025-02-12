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

#include "image.hpp"

#include "util/log.hpp"

#include <memory> // unique_ptr
#include <cstdio>
#include <turbojpeg.h>


struct tjDeleter
{
	void operator()(tjhandle handle) const
	{
		tjDestroy(handle);
	}
};

using tjInstance = std::unique_ptr<void, tjDeleter>; // for tjHandle which is a void*

thread_local tjInstance tjCompressInstance = nullptr;
thread_local tjInstance tjDecompressInstance = nullptr;

bool compress(std::vector<uint8_t> &jpegBuf, uint8_t *data, int width, int height, size_t stride, int quality)
{
	// Init TJ
	if (!tjCompressInstance && (tjCompressInstance = tjInstance(tjInitCompress())) == NULL)
	{
		LOGC(LError, "Failed initialise TJ!\n");
		return false;
	}
	// Pre-allocate buffer
	long unsigned int jpegSize = tjBufSize(width, height, TJSAMP_GRAY);
	if (size_t(-1) == jpegSize)
	{
		LOGC(LError, "Failed to calculate maximum jpeg size!\n");
		return false;
	}
	// Compress
	jpegBuf.resize(jpegSize);
	uint8_t *dataPtr = jpegBuf.data();
	if (stride < width) stride = width;
	int result = tjCompress2(tjCompressInstance.get(),
		data, width, stride, height, TJPF_GRAY,
		&dataPtr, &jpegSize, TJSAMP_GRAY,
		quality, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
	if (result < 0)
	{
		LOGC(LError, "Failed to encode image: %s\n", tjGetErrorStr());
		return false;
	}
	LOGC(LDebug, "Encoded image %dx%d, %dKB, to %ldKB with quality %d%%!\n", width, height, (int)(width*height/1024), jpegSize/1024, quality);
	jpegBuf.resize(jpegSize);
	return true;
}


bool decompress(std::vector<uint8_t> &imgBuf, uint8_t *data, size_t len, int &width, int &height)
{
	// Init TJ
	if (!tjDecompressInstance && (tjDecompressInstance = tjInstance(tjInitDecompress())) == NULL)
	{
		LOGC(LError, "Failed initialise TJ!\n");
		return false;
	}
	// Read Header
	int subsample, colorspace;
	if (tjDecompressHeader3(tjDecompressInstance.get(), data, len, &width, &height, &subsample, &colorspace) < 0)
	{
		LOGC(LError, "Failed to read JPEG header!\n");
		return false;
	}
	//LOGC(LTrace, "Header read: %dx%d, ss: %d, cs: %d\n", width, height, subsample, colorspace);
	// Decompress
	imgBuf.resize(width*height);
	int result = tjDecompress2(tjDecompressInstance.get(), data, len, imgBuf.data(), width, width, height, TJPF_GRAY, TJFLAG_ACCURATEDCT);
	if (result < 0)
	{
		LOGC(LError, "Failed to decode image: %s\n", tjGetErrorStr());
		return false;
	}
	return true;
}

uint8_t* sampleImageBounds(uint8_t *srcImage, int srcX, int srcY, int srcStride, Bounds2<int> srcBounds,
    int subsample, int &tgtX, int &tgtY, int &tgtStride, Bounds2<int> &tgtBounds)
{	
	if (subsample == 0)
	{
		printf("Failed to sample with subsample of 0\n");
		return nullptr;
	}
	if (srcBounds.maxX > srcX || srcBounds.maxY > srcY)
	{
		printf("Failed to sample with bounds max (%d, %d) over source size (%d, %d)\n",
			srcBounds.maxX, srcBounds.maxY, srcX, srcY);
		return nullptr;
	}
	tgtX = (srcBounds.maxX-srcBounds.minX)/subsample;
	tgtY = (srcBounds.maxY-srcBounds.minY)/subsample;
	// TODO: Enforce cropped image size to be multiples of 8?
	tgtBounds = srcBounds;
	if (tgtX < 10 || tgtY < 10)
	{
		printf("Failed to sample with target image after subsampling of size (%d, %d)\n", tgtX, tgtY);
		return nullptr;
	}

	if (subsample <= 1)
	{ // Send a (sub)block in original resolution
		tgtStride = srcStride;
		return srcImage + (srcBounds.minY*srcStride+srcBounds.minX);
	}

	// Subsample a block of image
	thread_local std::vector<uint8_t> encDataBuffer;
	encDataBuffer.resize(tgtX*tgtY);
	tgtStride = tgtX;
	for (uint_fast16_t y = 0; y < tgtY; y++)
	{
		uint_fast16_t pxY = y * subsample + srcBounds.minY;
		uint_fast16_t srcBase = pxY * srcStride;
		uint_fast16_t tgtBase = y * tgtStride;
		for (uint_fast16_t x = 0; x < tgtX; x++)
		{
			uint_fast16_t pxX = x * subsample + srcBounds.minX;
			encDataBuffer[tgtBase+x] = srcImage[srcBase+pxX];
		}
	}
	return encDataBuffer.data();
}