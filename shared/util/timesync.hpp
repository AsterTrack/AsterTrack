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

#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include "util/util.hpp" // TimePoint_t
#include "util/stats.hpp"

/*
 * Time Sync
 * Determines a best-estimate time sync between two systems (here over USB)
 * Assumes no inherent transfer time, but variable latency
 * Estimates from lowest-latency transfer
 */

struct TimeSyncParams
{
	float driftLerp = 0.0000004f;
	float driftBias = 0.0000000f;
	int driftInitRange = 1000;
	float driftInitAdapt = 10.0f;
	float driftDownwardCorrect = 200.0f;
	float driftDownwardJump = 0.20f;
	int timeOffsetUS = -20;
};

struct TimeSync
{
	TimeSyncParams params = {};
	uint32_t measurements;
	uint64_t lastTimestamp;
	float drift; // actualUS = (long)(controllerUS * (1+drift))
	float driftAccum;
	TimePoint_t lastTime;
	StatDistf diff;
	StatDistf syncSwitch;
};

uint64_t RebaseSourceTimestamp(const TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t reference);
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp);
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp, uint64_t overflow);
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t reference);
TimePoint_t UpdateTimeSync(TimeSync &time, uint64_t timestamp, TimePoint_t measurement);
void ResetTimeSync(TimeSync &time);

#endif // TIME_SYNC_H
