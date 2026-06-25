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

#ifndef VISUALISATION_H
#define VISUALISATION_H

#include "blob/blob.hpp"

// Forward-declared opaque structs
struct VisualisationImpl;

struct VisualisationLock
{
	void *lock;
	uint8_t *display;
	uint32_t length;
	uint32_t strideX, strideY;

	void write(uint16_t x, uint16_t y, uint32_t val)
	{
		if (!display) return;
		uint8_t *px = display + y*strideY + x*strideX;
		*(uint32_t*)px = val;
	}

	~VisualisationLock();
};

struct VisualisationState
{
	bool initialised = false;
	bool enabled = false;
	VisualisationImpl *impl;
	bool displayBlobs = false;
	bool displayFrame = false;
	int width = 640, height = 480;
	int interval = 10;

	VisualisationState();
	~VisualisationState();

	void visualise(const std::vector<Cluster> &blobs, const std::vector<Cluster> &pastBlobs, uint8_t *srcBuf, int srcWidth, int srcHeight, int srcStride);

	VisualisationLock lockWrite();
};

#endif // VISUALISATION_H