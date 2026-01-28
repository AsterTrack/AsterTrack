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

#include "timesync.hpp"

//#define LOG_MAX_LEVEL LDebug
#include "util/log.hpp"

#include <cinttypes>
#include <cmath>

// Corrects source timestamp for overflow (within past ~overflow us)
uint64_t RebaseSourceTimestamp(const TimeSync &time, uint64_t timestamp, uint64_t overflow)
{
	if (time.measurements == 0)
		return timestamp;
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	int32_t usPassed = shortDiff<uint64_t, int32_t>(time.lastTimestamp&(overflow-1), timestamp, overflow/10, overflow-1);
	return time.lastTimestamp + usPassed;
}
// Corrects source timestamp for overflow using real-time reference as help
uint64_t RebaseSourceTimestamp(const TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t reference)
{
	if (time.measurements == 0)
		return timestamp;
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	int32_t usPassed = shortDiff<uint64_t, int32_t>(time.lastTimestamp&(overflow-1), timestamp, overflow/2, overflow-1);
	int32_t usPassedRT = dtUS(time.lastTime, reference);
	int32_t overflowCorrection = (int32_t)std::round((float)(usPassedRT-usPassed)/overflow);
	if (overflowCorrection != 0)
		usPassed = usPassed+overflowCorrection*overflow;
	return time.lastTimestamp + usPassed;
}

// Get real-time of full timestamp
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp)
{
	if (time.measurements < 2)
	{
		LOG(LTimesync, LWarn, "Cannot get synced time, time sync not yet established with %d samples!", 
			time.measurements);
		return sclock::now();
	}
	int32_t passed = timestamp < time.lastTimestamp? -(int32_t)(time.lastTimestamp-timestamp) : (int32_t)(timestamp-time.lastTimestamp);
	passed = (int32_t)((double)passed * (1.0+time.drift) + time.driftAccum);
	TimePoint_t synced = time.lastTime + std::chrono::microseconds(passed);
	int32_t dT = dtUS(synced, sclock::now());
	if (dT < -1000)
	{
		LOG(LTimesync, LWarn, "Synced time is in the future, with last timestamp %" PRIu64 "us %.2fms in the past, timestamp %" PRIu64 "us is %.2fms in the future", 
			time.lastTimestamp, dtMS(time.lastTime, sclock::now()), timestamp, -dT/1000.0f);
	}
	return synced;
}

// Get real-time of timestamp of specified bit depth within past ~overflow us
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp, uint64_t overflow)
{
	if (time.measurements < 2)
	{
		LOG(LTimesync, LWarn, "Cannot get synced time, time sync not yet established with %d samples!", 
			time.measurements);
		return sclock::now();
	}
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	int32_t usPassed = shortDiff<uint64_t, int32_t>(time.lastTimestamp&(overflow-1), timestamp, overflow/2, overflow-1);
	TimePoint_t synced = time.lastTime + std::chrono::microseconds((int32_t)(usPassed * (1.0+time.drift) + time.driftAccum));
	int32_t dT = dtUS(synced, sclock::now());
	if (dT < -1000)
	{
		LOG(LTimesync, LWarn, "Synced time is in the future, with last timestamp %" PRIu64 "us %.2fms in the past, timestamp %" PRIu64 "us is %.2fms in the future", 
			time.lastTimestamp, dtMS(time.lastTime, sclock::now()), timestamp, -dT/1000.0f);
	}
	return synced;
}

// Get real-time of timestamp of specified bit depth
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t reference)
{
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	if (time.measurements < 20) return reference;
	return GetTimeSynced(time, RebaseSourceTimestamp(time, timestamp, overflow, reference));
}

TimePoint_t UpdateTimeSync(TimeSync &time, uint64_t timestamp, TimePoint_t measurement)
{ // Received packet timestamp with best real-time estimation being measurement

	const auto &params = time.params;

	if (time.measurements++ < 10)
	{ // Only get minimum offset in first few measurements
		time.lastTime = measurement;
		time.lastTimestamp = timestamp;
		time.syncSwitch.reset();
		time.diff.reset();
		return measurement;
	}

	// Predict real-time of timestamp based on time-local estimation of time synchronisation
	int32_t usPassed = timestamp < time.lastTimestamp? -(int32_t)(time.lastTimestamp-timestamp) : (int32_t)(timestamp-time.lastTimestamp);
	float driftUS = usPassed * (1.0+time.drift) + time.driftAccum;
	time.driftAccum = driftUS - ((int32_t)driftUS); // Store what we can't apply now in accumulator for next update
	TimePoint_t timePred = time.lastTime + std::chrono::microseconds((int32_t)driftUS);
	int32_t diffUS = dtUS(timePred, measurement);

	if (usPassed <= 0)
	{ // Already had a newer packet, this one is delayed and thus measurement is inaccurate
		if (time.measurements == 0)
			return measurement;
		LOG(LTimesync, LDarn, "Got delayed timestamped packet, reference timestamp %" PRIu64 "us %.2fms in the past, timestamp %" PRIu64 "us %.2fms in the past, usPassed %d",
			time.lastTimestamp, dtMS(time.lastTime, sclock::now()), timestamp, dtMS(timePred, sclock::now()), usPassed);
		return timePred;
	}

	// TODO: Adapt values or full strategy based on protocol (USB vs TCP)
	// Update time-local estimation of time synchronisation with new measurement
	bool valid = true;
	float adapt = std::max(0.0f, 1.0f - (float)time.measurements/params.driftInitRange);
	auto getDriftCorrection = [&params, &adapt](double diffUS, int32_t usPassed)
	{
		double term = params.driftBias + params.driftLerp * diffUS;
		double increase = std::log(1.0f + term / usPassed);
		increase *= 1.0f + adapt * params.driftInitAdapt;
		return increase;
	};
	if (diffUS <= 0 && diffUS > -2000)
	{ // New minimum, upper bound for the actual time sync
		LOG(LTimesync, diffUS > 100? LDebug : LTrace, "New time sync minimum received, timestamp was predicted to be %ldus ago (%dus passed, including %.2fus drift) but was received %ldus ago",
			dtUS(timePred, sclock::now()), usPassed, driftUS-usPassed, dtUS(measurement, sclock::now()));
		float jumpCorrect = diffUS*std::lerp(params.driftDownwardJump, 1.0f, adapt);
		timePred += std::chrono::microseconds((int64_t)jumpCorrect);
		time.drift += getDriftCorrection(diffUS-jumpCorrect, usPassed) * params.driftDownwardCorrect;
		time.diff.update(diffUS / 1000.0f);
	}
	else if (diffUS > 0 && diffUS < 2000)
	{ // Higher than predicted, delayed by transfer stack and OS, but might also be genuine drift, so adapt slowly
		float correction = getDriftCorrection(diffUS, usPassed);
		time.drift += correction;
		timePred += std::chrono::microseconds((int64_t)correction*usPassed);
		time.diff.update(diffUS / 1000.0f);
	}
	else
	{ // Off by quite a bit - shouldn't really happen
		if (time.syncSwitch.num > 500 && time.syncSwitch.variance() < 500*500)
		{
			LOG(LTimesync, LError, "! New time sync received, timestamp was predicted to be %.2fms ago (%dus passed, including %.2fus drift), "
				"but repeated differences of %.2fms +- %.2fms will be accepted as new time sync!", 
				dtMS(timePred, sclock::now()), usPassed, driftUS-usPassed, time.syncSwitch.avg/1000.0f, time.syncSwitch.stdDev()/1000.0f);
			timePred += std::chrono::microseconds((int64_t)time.syncSwitch.avg);
			time.diff.reset();
			// valid being true will reset syncSwitch at end of function
		}
		else if (time.syncSwitch.num > 1000)
		{
			LOG(LTimesync, LError, "! Hopeless time sync loss, %d samples differed significantly by %.2fms +- %.2fms, restarting time sync!", 
				time.syncSwitch.num, time.syncSwitch.avg/1000.0f, time.syncSwitch.stdDev()/1000.0f);
			time.measurements = 1; // Reset to start adapting to time sync more quickly again
			time.diff.reset();
		}
		else
		{ // Very likely delay in scheduling of host application, but record nonetheless to detect a consistent jump
			valid = false;
			time.syncSwitch.update(diffUS);
			//time.diff.update(diffUS / 1000.0f);
			if (time.measurements > 2)
			{
				LOG(LTimesync, LDarn, "! Received timestamp %" PRIu64 "us at wrong time, was predicted to be %.2fms ago (%dus passed, including %.2fus drift), "
					"but was received %.2fms ago, diff usually %.2fms +- %.2fms! Maybe Host lagged.", 
					timestamp, dtMS(timePred, sclock::now()), usPassed, driftUS-usPassed, dtMS(measurement, sclock::now()), time.diff.avg, time.diff.stdDev());
			}
		}
	}

	if (valid)
	{
		time.lastTime = timePred;
		time.lastTimestamp = timestamp;
		time.syncSwitch.reset();
	}

	return timePred + std::chrono::microseconds(params.timeOffsetUS);
}

void ResetTimeSync(TimeSync &time)
{
	time = {};
}