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

#include <cmath>
#include <cassert>

#define LOG(...) {}

// Corrects source timestamp for overflow (within past ~overflow us)
uint64_t RebaseSourceTimestamp(TimeSync &time, uint64_t timestamp, uint64_t overflow)
{
	if (time.measurements == 0)
		return timestamp;
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	long long usPassed = shortDiff<uint64_t, long long>(time.lastTimestamp&(overflow-1), timestamp, overflow/10, overflow);
	return time.lastTimestamp + usPassed;
}
// Corrects source timestamp for overflow using real-time reference as help
uint64_t RebaseSourceTimestamp(TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t reference)
{
	if (time.measurements == 0)
		return timestamp;
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	long long usPassed = shortDiff<uint64_t, long long>(time.lastTimestamp&(overflow-1), timestamp, overflow/10, overflow);
	long usPassedRT = dtUS(time.lastTime, reference);
	int overflowCorrection = (int)std::round((float)(usPassedRT-usPassed)/overflow);
	if (overflowCorrection != 0)
		usPassed = usPassed+overflowCorrection*overflow;
	return time.lastTimestamp + usPassed;
}

// Get real-time of timestamp of specified bit depth within past ~overflow us
TimePoint_t GetTimeSynced(TimeSync &time, uint64_t timestamp, uint64_t overflow)
{
	assert((overflow & (overflow-1)) == 0); // Else use %overflow instead of &(overflow-1)
	long long usPassed = shortDiff<uint64_t, long long>(time.lastTimestamp&(overflow-1), timestamp, overflow/10, overflow);
	return time.lastTime + std::chrono::microseconds((long long)(usPassed * (1.0+time.drift)));
}

// Get real-time of timestamp of specified bit depth
TimePoint_t GetTimeSynced(TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t reference)
{
	if (time.measurements < 20)
		return reference;
	timestamp = RebaseSourceTimestamp(time, timestamp, overflow, reference);
	TimePoint_t synced = time.lastTime + std::chrono::microseconds((long)((double)(timestamp-time.lastTimestamp) * (1.0+time.drift)));
	int dT = dtUS(synced, sclock::now());
	if (dT < -1000)
	{
		LOG(LTimesync, LWarn, "Synced time is in the future, with reference timestamp %luus %.2fms in the past, timestamp %luus %.2fms in the future\n", 
			time.lastTimestamp, dt(time.lastTime, sclock::now()), timestamp, -dT/1000.0f);
	}
	return synced;
}

TimePoint_t UpdateTimeSync(TimeSync &time, uint64_t timestamp, uint64_t overflow, TimePoint_t measurement)
{ // Received packet timestamp with best real-time estimation being measurement

	// Correct for timestamp overflow based on existing timestamps
	uint64_t fullTimestamp = RebaseSourceTimestamp(time, timestamp, overflow, measurement);

	// Predict real-time of timestamp based on time-local estimation of time synchronisation
	long usPassed = fullTimestamp-time.lastTimestamp;
	TimePoint_t timePred = time.lastTime + std::chrono::microseconds((long)(usPassed * (1.0+time.drift)));

	if (usPassed < 0)
	{ // Already had a newer packet, this one is delayed and thus measurement is inaccurate
		if (time.measurements == 0)
			return measurement;
		LOG(LTimesync, LDarn, "Got delayed timestamped packet, reference timestamp %luus %.2fms in the past, timestamp %luus %.2fms in the past, usPassed %ld\n",
			time.lastTimestamp, dt(time.lastTime, sclock::now()), timestamp, dt(timePred, sclock::now()), usPassed);
		return timePred;
	}

	// TODO: Adapt values or full strategy based on protocol (USB vs TCP)

	// Update time-local estimation of time synchronisation with new measurement
	int diffUS = dtUS(timePred, measurement);
	bool valid = true;
	if (diffUS <= 0 && diffUS > -2000)
	{ // New minimum, upper bound for the actual time sync
		LOG(LTimesync, LTrace, "New time sync minimum received, timestamp was predicted to be %.2fms ago (including %.2fms drift) but was received %.2fms ago\n",
			dt(timePred, sclock::now()), time.increase.count()/1000.0f, dt(measurement, sclock::now()));
		timePred = measurement;
		time.increase = std::chrono::microseconds(0);
		time.diff.update(diffUS / 1000.0f);
	}
	else if (diffUS > 0 && diffUS < 2000)
	{ // Higher than predicted, delayed by transfer stack and OS, but might also be genuine drift, so adapt slowly
		static float predictionLerp = 0.005f;
		auto increase = std::chrono::microseconds((int)(predictionLerp * diffUS));
		timePred += increase;
		time.increase += increase;
		time.diff.update(diffUS / 1000.0f);
	}
	else
	{ // Off by quite a bit - shouldn't really happen
		if (time.syncSwitch.num > 10 && time.syncSwitch.variance() < 500*500)
		{
			LOG(LTimesync, LError, "! New time sync received, timestamp was predicted to be %.2fms ago (including %.2fms drift), "
				"but repeated differences of %.2fms +- %.2fms will be accepted as new time sync!\n", 
				dt(timePred, sclock::now()), time.increase.count()/1000.0f, time.syncSwitch.avg/1000.0f, time.syncSwitch.stdDev()/1000.0f);
			timePred += std::chrono::microseconds((int)time.syncSwitch.avg);
			time.increase = std::chrono::microseconds(0);
			time.diff.reset();
			// valid being true will reset syncSwitch at end of function
		}
		else if (time.syncSwitch.num > 100)
		{
			LOG(LTimesync, LError, "! Hopeless time sync loss, %d samples differed significantly by %.2fms +- %.2fms, restarting time sync!\n", 
				time.syncSwitch.num, time.syncSwitch.avg/1000.0f, time.syncSwitch.stdDev()/1000.0f);
			valid = false;
			time.measurements = 0; // WIll start new time sync at end of function
			time.syncSwitch.reset();
			time.diff.reset();
		}
		else
		{ // Very likely delay in scheduling of host application, but record nonetheless to detect a consistent jump
			valid = false;
			time.syncSwitch.update(diffUS);
			//time.diff.update(diffUS / 1000.0f);
			if (time.measurements > 2)
			{
				LOG(LTimesync, LDebug, "! Received timestamp %luus at wrong time, was predicted to be %.2fms ago (including %lldus drift), "
					"but was received %.2fms ago, diff usually %.2fms +- %.2fms! Maybe Host lagged.\n", 
					timestamp, dt(timePred, sclock::now()), time.increase.count(), dt(measurement, sclock::now()), time.diff.avg, time.diff.stdDev());
			}
		}
	}

	if (time.measurements++ < 10)
	{
		time.lastTime = measurement;
		time.lastTimestamp = timestamp;
		time.syncSwitch.reset();
		time.diff.reset();
		return measurement;
	}

	if (valid)
	{
		time.lastTime = timePred;
		time.lastTimestamp = fullTimestamp;
		time.syncSwitch.reset();
	}

	return timePred;
}

void ResetTimeSync(TimeSync &time)
{
	time.lastTimestamp = 0;
	time.diff.reset();
	time.syncSwitch.reset();
	time.drift = 0;
	time.increase = std::chrono::microseconds(0);
	time.measurements = 0;
}