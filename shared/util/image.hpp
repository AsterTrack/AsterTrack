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

#ifndef IMAGE_H
#define IMAGE_H

#include <vector>
#include <cstdint>
#include <cstdio>

#include "eigendef.hpp"


bool compress(std::vector<uint8_t> &jpegBuf, uint8_t *data, int width, int height, size_t stride, int quality);
bool decompress(std::vector<uint8_t> &imgBuf, uint8_t *data, size_t len, int &width, int &height);

// Warning: Return value only valid on this thread until next call to sampleImageBounds (thead_local variable)
uint8_t* sampleImageBounds(uint8_t *srcImage, int srcX, int srcY, int srcStride, Bounds2<int> srcBounds,
    int subsample, int &tgtX, int &tgtY, int &tgtStride, Bounds2<int> &tgtBounds);


#endif // IMAGE_H