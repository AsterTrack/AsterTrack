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

#ifndef PACKET_HPP
#define PACKET_HPP

/**
 * This file describes the common packet interface and is shared between host and camera only
 * Thus it needs to be kept in sync and working with only the C++ compiler!
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memcpy
#include <limits.h>
#include <assert.h>

#include "packet.h"
#include "blob/parameters.hpp"

#define STREAM_PACKET_HEADER_SIZE		0
#define STREAM_PACKET_BLOB_SIZE			6


/**
 * Configuration of camera parameters
 */
struct ConfigPacket
{
	uint16_t width, height;
	uint16_t fps, shutterSpeed;
	uint8_t analogGain;
	bool extTrig;
	bool strobe;
	int16_t strobeOffset;
	uint16_t strobeLength;
	
	BlobProcessingParameters blobProc;
};
#define CONFIG_PACKET_SIZE			66

static inline struct ConfigPacket parseConfigPacket(uint8_t data[CONFIG_PACKET_SIZE])
{
	struct ConfigPacket config;
	config.width = *(uint16_t*)&data[0];
	config.height = *(uint16_t*)&data[2];
	config.fps = *(uint16_t*)&data[4];
	config.shutterSpeed = *(uint16_t*)&data[6];
	config.analogGain = *(uint8_t*)&data[8];
	config.extTrig = *(uint8_t*)&data[9];
	config.strobe = *(uint8_t*)&data[10];
	config.strobeOffset = *(int16_t*)&data[11];
	config.strobeLength = *(uint16_t*)&data[13];

	auto &proc = config.blobProc;
	int base = 15;

	{
		proc.thresholds.absolute = *(uint8_t*)&data[base+0];
		proc.thresholds.edge = *(uint8_t*)&data[base+1];
	}
	base += 2;

	{
		proc.base.blur = *(bool*)&data[base+0];
		proc.base.radius = *(uint8_t*)&data[base+1];
		memcpy(&proc.base.sigma, &data[base+2], 4);
	}
	base += 6;

	{
		proc.classification.resegmentSingleClusters = *(uint8_t*)&data[base+0];
		proc.classification.blobTinyThreshold = *(uint16_t*)&data[base+1];
		proc.classification.resegmentationThreshold = *(uint16_t*)&data[base+3];
		proc.classification.blobRefinementThreshold = *(uint16_t*)&data[base+5];
	}
	base += 7;

	{
		auto &ssr = proc.ssr;
		ssr.sigmaSteps = *(uint8_t*)&data[base+0];
		ssr.sigmaMin = *(uint8_t*)&data[base+1];
		ssr.sigmaMax = *(uint8_t*)&data[base+2];
		ssr.sigmaCurve = *(uint8_t*)&data[base+3];
		ssr.sigmaTrunc = *(uint8_t*)&data[base+4];
	}
	base += 5;

	{
		proc.maximaHints.plateauThreshold = *(uint8_t*)&data[base+0];
		proc.maximaHints.plateauFillOffset = *(uint8_t*)&data[base+1];
		proc.maximaHints.minStable = *(uint8_t*)&data[base+2];
		proc.maximaHints.minScale = *(uint8_t*)&data[base+3];
	}
	base += 4;

	{
		proc.floodfilling.threshold.min = *(uint8_t*)&data[base+0];
		proc.floodfilling.threshold.step = *(uint8_t*)&data[base+1];
		proc.floodfilling.threshold.minSubStep = *(uint8_t*)&data[base+2];
		proc.floodfilling.threshold.acceptableLoss = *(uint8_t*)&data[base+3];
	}
	base += 4;

	{
		memcpy(&proc.floodfilling.blob.peakMinimumRatio, &data[base+0], 4);
		memcpy(&proc.floodfilling.blob.limitExpansionFactor, &data[base+4], 4);
		memcpy(&proc.floodfilling.blob.limitExpansionBase, &data[base+8], 4);
		proc.floodfilling.blob.allowBoundsExpansion = *(uint8_t*)&data[base+12];
	}
	base += 13;

	{
		memcpy(&proc.filtering.minContrastValue, &data[base+0], 4);
		*(uint8_t*)&data[base+4] = proc.filtering.minContributingPixels;
	}
	base += 5;

	{
		proc.refinement.targetEdgeVal = *(uint8_t*)&data[base+0];
		memcpy(&proc.refinement.maxEdgeOffsetPX, &data[base+1], 4);
	}
	base += 5;
	assert(base == CONFIG_PACKET_SIZE);

	return config;
};

static inline void storeConfigPacket(struct ConfigPacket config, uint8_t data[CONFIG_PACKET_SIZE])
{
	*(uint16_t*)&data[0] = config.width;
	*(uint16_t*)&data[2] = config.height;
	*(uint16_t*)&data[4] = config.fps;
	*(uint16_t*)&data[6] = config.shutterSpeed;
	*(uint8_t*)&data[8] = config.analogGain;
	*(uint8_t*)&data[9] = config.extTrig;
	*(uint8_t*)&data[10] = config.strobe;
	*(int16_t*)&data[11] = config.strobeOffset;
	*(uint16_t*)&data[13] = config.strobeLength;

	auto &proc = config.blobProc;
	int base = 15;

	{
		*(uint8_t*)&data[base+0] = proc.thresholds.absolute;
		*(uint8_t*)&data[base+1] = proc.thresholds.edge;
	}
	base += 2;

	{
		*(bool*)&data[base+0] = proc.base.blur;
		*(uint8_t*)&data[base+1] = proc.base.radius;
		memcpy(&data[base+2], &proc.base.sigma, 4);
	}
	base += 6;

	{
		*(uint8_t*)&data[base+0] = proc.classification.resegmentSingleClusters;
		*(uint16_t*)&data[base+1] = proc.classification.blobTinyThreshold;
		*(uint16_t*)&data[base+3] = proc.classification.resegmentationThreshold;
		*(uint16_t*)&data[base+5] = proc.classification.blobRefinementThreshold;
	}
	base += 7;

	{
		auto &ssr = proc.ssr;
		*(uint8_t*)&data[base+0] = ssr.sigmaSteps;
		*(uint8_t*)&data[base+1] = ssr.sigmaMin;
		*(uint8_t*)&data[base+2] = ssr.sigmaMax;
		*(uint8_t*)&data[base+3] = ssr.sigmaCurve;
		*(uint8_t*)&data[base+4] = ssr.sigmaTrunc;
	}
	base += 5;

	{
		*(uint8_t*)&data[base+0] = proc.maximaHints.plateauThreshold;
		*(uint8_t*)&data[base+1] = proc.maximaHints.plateauFillOffset;
		*(uint8_t*)&data[base+2] = proc.maximaHints.minStable;
		*(uint8_t*)&data[base+3] = proc.maximaHints.minScale;
	}
	base += 4;

	{
		*(uint8_t*)&data[base+0] = proc.floodfilling.threshold.min;
		*(uint8_t*)&data[base+1] = proc.floodfilling.threshold.step;
		*(uint8_t*)&data[base+2] = proc.floodfilling.threshold.minSubStep;
		*(uint8_t*)&data[base+3] = proc.floodfilling.threshold.acceptableLoss;
	}
	base += 4;

	{
		memcpy(&data[base+0], &proc.floodfilling.blob.peakMinimumRatio, 4);
		memcpy(&data[base+4], &proc.floodfilling.blob.limitExpansionFactor, 4);
		memcpy(&data[base+8], &proc.floodfilling.blob.limitExpansionBase, 4);
		*(uint8_t*)&data[base+12] = proc.floodfilling.blob.allowBoundsExpansion;
	}
	base += 13;

	{
		memcpy(&data[base+0], &proc.filtering.minContrastValue, 4);
		*(uint8_t*)&data[base+4] = proc.filtering.minContributingPixels;
	}
	base += 5;

	{
		*(uint8_t*)&data[base+0] = proc.refinement.targetEdgeVal;
		memcpy(&data[base+1], &proc.refinement.maxEdgeOffsetPX, 4);
	}
	base += 5;
	assert(base == CONFIG_PACKET_SIZE);
};


/**
 * Camera statistics packet
 */
struct StatPacket
{
	struct
	{
		uint32_t frame;
		uint32_t deltaUS;
		uint16_t tempSOC;
		uint8_t skipTrigger, skipQPU, skipCPU;
	} header;
	union Times {
		uint16_t data[15];
		struct {
			uint16_t processing, latency;
			uint16_t qpu, cpu, fetch, ccl, post, blur, maxima, local, iter, check, reseg, refine, send;
		};
	} times;
	union Incidents 
	{
		struct Stat
		{
			uint16_t occurences, avg, max;
		} data[9];
		struct {
			struct Stat vis, stream, lag, await, skip, access, proc, handle, cpuwait;
		};
	} incidents;
};
#define STAT_PACKET_SIZE			97

static inline struct StatPacket parseStatPacket(uint8_t data[STAT_PACKET_SIZE])
{
	struct StatPacket stat;
	{ // Header has padding, so need to manually copy
		stat.header.frame = *(uint32_t*)&data[0];
		stat.header.deltaUS = *(uint32_t*)&data[4];
		stat.header.tempSOC = *(uint16_t*)&data[8];
		stat.header.skipTrigger = *(uint8_t*)&data[10];
		stat.header.skipQPU = *(uint8_t*)&data[11];
		stat.header.skipCPU = *(uint8_t*)&data[12];
	}
	const int base = 13;
	static_assert(sizeof(stat.times) == sizeof(stat.times.data));
	memcpy(&stat.times, &data[base], sizeof(stat.times));
	static_assert(sizeof(stat.incidents) == sizeof(stat.incidents.data));
	memcpy(&stat.incidents, &data[base+sizeof(stat.times)], sizeof(stat.incidents));
	static_assert(STAT_PACKET_SIZE == base+sizeof(stat.times)+sizeof(stat.incidents));
	return stat;
};

static inline void storeStatPacket(const struct StatPacket stat, uint8_t data[STAT_PACKET_SIZE])
{
	{ // Header has padding, so need to manually copy
		*(uint32_t*)&data[0] = stat.header.frame;
		*(uint32_t*)&data[4] = stat.header.deltaUS;
		*(uint16_t*)&data[8] = stat.header.tempSOC;
		*(uint8_t*)&data[10] = stat.header.skipTrigger;
		*(uint8_t*)&data[11] = stat.header.skipQPU;
		*(uint8_t*)&data[12] = stat.header.skipCPU;
	}
	const int base = 13;
	static_assert(sizeof(stat.times) == sizeof(stat.times.data));
	memcpy(&data[base], &stat.times, sizeof(stat.times));
	static_assert(sizeof(stat.incidents) == sizeof(stat.incidents.data));
	memcpy(&data[base+sizeof(stat.times)], &stat.incidents, sizeof(stat.incidents));
	static_assert(STAT_PACKET_SIZE == base+sizeof(stat.times)+sizeof(stat.incidents));
};

#endif // PACKET_HPP