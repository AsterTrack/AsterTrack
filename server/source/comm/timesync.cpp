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

// TODO: Merge with timesync in camera and put in shared code
// Currently, servers timesync has gotten more complex due to proper tools, but cameras also just works
// So without proper tools verifying camera timesync, it is safer to just not change whats not broken on the camera

#include "util/log.hpp"

#include <cmath>

// Corrects source timestamp for overflow (within past ~overflow us)
uint64_t RebaseSourceTimestamp(const TimeSync &time, uint64_t timestamp, uint64_t overflow)
{
	if (time.measurements == 0)
		return timestamp;
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	long long usPassed = shortDiff<uint64_t, long long>(time.lastTimestamp&(overflow-1), timestamp, overflow/10, overflow);
	return time.lastTimestamp + usPassed;
}
// Corrects source timestamp for overflow using real-time reference as help
uint64_t RebaseSourceTimestamp(const TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t reference)
{
	if (time.measurements == 0)
		return timestamp;
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	long long usPassed = shortDiff<uint64_t, long long>(time.lastTimestamp&(overflow-1), timestamp, overflow/2, overflow);
	long usPassedRT = dtUS(time.lastTime, reference);
	int overflowCorrection = (int)std::round((float)(usPassedRT-usPassed)/overflow);
	if (overflowCorrection != 0)
		usPassed = usPassed+overflowCorrection*overflow;
	return time.lastTimestamp + usPassed;
}

// Get real-time of full timestamp
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp)
{
	long passed = ((long long)timestamp) - time.lastTimestamp;
	passed = (long)((double)passed * (1.0+time.drift) + time.driftAccum);
	TimePoint_t synced = time.lastTime + std::chrono::microseconds(passed);
	int dT = dtUS(synced, sclock::now());
	if (dT < -1000)
	{
		LOG(LTimesync, LWarn, "Synced time is in the future, with last timestamp %luus %.2fms in the past, timestamp %luus is %.2fms in the future\n", 
			time.lastTimestamp, dtMS(time.lastTime, sclock::now()), timestamp, -dT/1000.0f);
	}
	return synced;
}

// Get real-time of timestamp of specified bit depth within past ~overflow us
TimePoint_t GetTimeSynced(const TimeSync &time, uint64_t timestamp, uint64_t overflow)
{
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	long long usPassed = shortDiff<uint64_t, long long>(time.lastTimestamp&(overflow-1), timestamp, overflow/2, overflow);
	TimePoint_t synced = time.lastTime + std::chrono::microseconds((long long)(usPassed * (1.0+time.drift) + time.driftAccum));
	int dT = dtUS(synced, sclock::now());
	if (dT < -1000)
	{
		LOG(LTimesync, LWarn, "Synced time is in the future, with last timestamp %luus %.2fms in the past, timestamp %luus is %.2fms in the future\n", 
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

std::pair<uint64_t, TimePoint_t> UpdateTimeSync(TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t measurement)
{ // Received packet timestamp with best real-time estimation being measurement

	// Correct for timestamp overflow based on existing timestamps
	uint64_t fullTimestamp = RebaseSourceTimestamp(time, timestamp, overflow, measurement);

	// Predict real-time of timestamp based on time-local estimation of time synchronisation
	long usPassed = fullTimestamp-time.lastTimestamp;
	float driftUS = usPassed * (1.0+time.drift) + time.driftAccum;
	time.driftAccum = driftUS - ((int)driftUS); // Store what we can't apply now in accumulator for next update
	TimePoint_t timePred = time.lastTime + std::chrono::microseconds((long)driftUS);
	long diffUS = dtUS(timePred, measurement);

	if (usPassed <= 0)
	{ // Already had a newer packet, this one is delayed and thus measurement is inaccurate
		if (time.measurements == 0)
			return { 0, measurement };
		LOG(LTimesync, LDarn, "Got delayed timestamped packet, reference timestamp %luus %.2fms in the past, timestamp %luus %.2fms in the past, usPassed %ld\n",
			time.lastTimestamp, dtMS(time.lastTime, sclock::now()), timestamp, dtMS(timePred, sclock::now()), usPassed);
		return { fullTimestamp, timePred };
	}

	if (time.measurements++ < 10)
	{ // Only get minimum offset in first few measurements
		time.lastTime = measurement;
		time.lastTimestamp = timestamp;
		time.syncSwitch.reset();
		time.diff.reset();
		if (diffUS < 0)
			timePred = measurement;
		return { fullTimestamp, measurement };
	}

	// TODO: Adapt values or full strategy based on protocol (USB vs TCP)
	// Update time-local estimation of time synchronisation with new measurement
	bool valid = true;
	float adapt = std::max(0.0f, 1.0f - (float)time.measurements/time.driftInitRange);
	auto getDriftCorrection = [&time, &adapt](double diffUS, long usPassed)
	{
		double term = time.driftBias + time.driftLerp * diffUS;
		double increase = std::log(1.0f + term / usPassed);
		increase *= 1.0f + adapt * time.driftInitAdapt;
		return increase;
	};
	if (diffUS < 0 && diffUS > -2000)
	{ // New minimum, upper bound for the actual time sync
		LOG(LTimesync, LTrace, "New time sync minimum received, timestamp was predicted to be %.2fms ago (including %.2fus drift) but was received %.2fms ago\n",
			dtMS(timePred, sclock::now()), driftUS, dtMS(measurement, sclock::now()));
		float jumpCorrect = diffUS*std::lerp(time.driftDownwardJump, 1.0f, adapt);
		timePred += std::chrono::microseconds((long)jumpCorrect);
		time.drift += getDriftCorrection(diffUS-jumpCorrect, usPassed) * time.driftDownwardCorrect;
		time.diff.update(diffUS / 1000.0f);
	}
	else if (diffUS > 0 && diffUS < 2000)
	{ // Higher than predicted, delayed by transfer stack and OS, but might also be genuine drift, so adapt slowly
		float correction = getDriftCorrection(diffUS, usPassed);
		time.drift += correction;
		timePred += std::chrono::microseconds((long)correction*usPassed);
		time.diff.update(diffUS / 1000.0f);
	}
	else
	{ // Off by quite a bit - shouldn't really happen
		if (time.syncSwitch.num > 500 && time.syncSwitch.variance() < 500*500)
		{
			LOG(LTimesync, LError, "! New time sync received, timestamp was predicted to be %.2fms ago (including %.2fus drift), "
				"but repeated differences of %.2fms +- %.2fms will be accepted as new time sync!\n", 
				dtMS(timePred, sclock::now()), driftUS, time.syncSwitch.avg/1000.0f, time.syncSwitch.stdDev()/1000.0f);
			timePred += std::chrono::microseconds((int)time.syncSwitch.avg);
			time.diff.reset();
			// valid being true will reset syncSwitch at end of function
		}
		else if (time.syncSwitch.num > 1000)
		{
			LOG(LTimesync, LError, "! Hopeless time sync loss, %d samples differed significantly by %.2fms +- %.2fms, restarting time sync!\n", 
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
				LOG(LTimesync, LDebug, "! Received timestamp %luus at wrong time, was predicted to be %.2fms ago (including %.2fus drift), "
					"but was received %.2fms ago, diff usually %.2fms +- %.2fms! Maybe Host lagged.\n", 
					fullTimestamp, dtMS(timePred, sclock::now()), driftUS, dtMS(measurement, sclock::now()), time.diff.avg, time.diff.stdDev());
			}
		}
	}

	if (valid)
	{
		time.lastTime = timePred;
		time.lastTimestamp = fullTimestamp;
		time.syncSwitch.reset();
	}

	return { fullTimestamp, timePred + std::chrono::microseconds(time.timeOffsetUS) };
}

void ResetTimeSync(TimeSync &time)
{
	time = {};
}