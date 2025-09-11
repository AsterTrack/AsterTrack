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

#ifndef TIMING_RECORD_H
#define TIMING_RECORD_H

#include "util/util.hpp"
#include "util/blocked_vector.hpp"

struct TimeSyncMeasurement
{
	uint64_t timestamp;
	TimePoint_t estimation;
	TimePoint_t measurement;
};

const int LatencyStackSize = 8;
struct LatencyMeasurement
{
	TimePoint_t sample;
	uint16_t latency[LatencyStackSize];
};

struct TimingRecord
{
	bool supportsTimeSync = false, recordTimeSync = false;
	BlockedQueue<TimeSyncMeasurement, 4096> timeSync;
	bool supportsLatency = false, recordLatency = false;
	std::vector<std::string> latencyDescriptions;
	BlockedQueue<LatencyMeasurement, 4096> latency;

	TimingRecord() = default;
	TimingRecord(bool timeSync, bool latency) : supportsTimeSync(timeSync), supportsLatency(latency) {}

	/* Non-Blocking - only fully deletes blocks not referenced anymore */
	void clear()
	{
		recordTimeSync = false;
		timeSync.cull_clear();
		timeSync.delete_culled();
		recordLatency = false;
		latency.cull_clear();
		latency.delete_culled();
	}
};

#endif // TIMING_RECORD_H
