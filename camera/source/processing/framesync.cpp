/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include "framesync.hpp"

// Very useful, but should still disable as logging can make bad situations even worse
//#define LOG_TIMING

FrameSync frameSync;

bool getEarliestNextArrival(TimePoint_t lastArrival, TimePoint_t &nextArrival)
{
	std::unique_lock lock(frameSync.access);
	if (frameSync.estArrival > lastArrival)
	{ // Already had next arrival
		nextArrival = frameSync.estArrival;
		return false;
	}
	if (frameSync.config.synced)
	{
		if (frameSync.frameSOFs.empty())
		{ // Nothing announced, can only set lower limit based on known delay with some margin to receive SOF
			nextArrival = sclock::now() + std::chrono::microseconds((int32_t)frameSync.sensorLatency.avg - 500);
			return false;
		}
		else
		{ // Know the next frames SOF, estimate arrival time
			nextArrival = frameSync.frameSOFs.front().second + std::chrono::microseconds((int32_t)frameSync.sensorLatency.avg);
			return true;
		}
	}
	else
	{
		if (frameSync.config.continuous)
		{ // Estimate from pace
			nextArrival = frameSync.estArrival + std::chrono::microseconds((int32_t)frameSync.estInterval.floating);
			return true;
		}
		else
		{ // Estimate from minimum
			nextArrival = frameSync.estArrival + std::chrono::microseconds((int32_t)frameSync.estInterval.min);
			return false;
		}
	}
}

static void correctFromFrameSync(TimePoint_t frameReceived, bool frameDelayed, uint32_t obsAdvance)
{
#ifdef LOG_TIMING
	const char *space = "";
	#define DEBUG_TIME(str, ...) {\
		printf("%s" str, space, __VA_ARGS__);\
		space = "    ";\
	}
	#define HAS_LOGGED() (space[0] != '\0')
#else
	#define DEBUG_TIME(...) {}
	#define HAS_LOGGED() false
#endif

	float newInterval = dtMS(frameSync.estArrival, frameReceived);

	// Need to combine three possible sources of frame IDs:
	// Frame SOFs received - possible failure: MCU I2C is not polled in time due to scheduling or packet was erroneous
	// Frames received from V4L2 - possible failure: Camera may have skipped a trigger, or frame got lost some other way
	// Time passed when continuous - may indicate loss of SOF even earlier, e.g. from controller->MCU

	int expAdvance = obsAdvance;
	if (frameSync.config.continuous)
	{ // Estimate how many frames SHOULD have arrived
		float expFrameIntervals = newInterval / frameSync.config.intervalUS;
		expAdvance = std::max(1, (int)(expFrameIntervals+0.2f));
		bool likelyDropped = std::abs(expFrameIntervals - obsAdvance) > 1;
		if (likelyDropped)
			DEBUG_TIME("May have dropped frames! %.1f frame times passed, but only got %d new frames!\n", expFrameIntervals, obsAdvance);
	}

	uint32_t frameIDPre = frameSync.frameID;
	uint32_t frameIDObs = frameSync.frameID + obsAdvance;
	uint32_t frameIDExp = frameSync.frameID + expAdvance;

	frameSync.frameID = frameIDObs;
	long frameTimeUSNext = 0;

	std::unique_lock lock(frameSync.access);

	unsigned int sensorLatencyUS = frameSync.sensorLatency.avg;

	if (frameSync.frameSOFs.empty())
	{
		//if (frameDelayed && frameSync.config.continuous) // Could estimate
		frameSync.estArrival = frameReceived;
	}

	while (!frameSync.frameSOFs.empty())
	{
		auto SOF = frameSync.frameSOFs.front();
		uint32_t sofFrameIDObs = frameIDObs + shortDiff<uint16_t, int>(frameIDObs&0xFF, SOF.first, 5, 0xFF);
		uint32_t sofFrameIDExp = frameIDExp + shortDiff<uint16_t, int>(frameIDExp&0xFF, SOF.first, 5, 0xFF);
		uint32_t SOFID = frameSync.config.continuous? sofFrameIDExp : sofFrameIDObs;
		long frameTimeUS = dtUS(SOF.second, frameReceived);
		if (sofFrameIDExp != sofFrameIDObs)
		{ // Drop of frame sync longer than 255-bias frames
			DEBUG_TIME("Encountered large drop of frame sync - cannot recover frameID from last %u, with next either obs %u or exp %u, SOF either obs %u or exp %u\n",
				frameIDPre, frameIDObs, frameIDExp, sofFrameIDObs, sofFrameIDExp);
		}
		else if (HAS_LOGGED())
		{
			DEBUG_TIME("Handling SOF %u with frame time %ldus, with current adjusted frame ID of %d!\n",
				SOFID, frameTimeUS, frameSync.frameID);
		}

		if (frameTimeUS <= sensorLatencyUS && SOFID > frameIDObs)
		{ // Very likely a future SOF, take current frame ID
			DEBUG_TIME("Skipping future frame SOF %d (current observed %d), with frame time %ldus\n", SOFID, frameIDObs, frameTimeUS);
			break;
		}

		if (SOFID < frameIDObs)
		{ // Likely didn't receive SOF in time, so it wasn't accounted for before
			if (frameTimeUS > sensorLatencyUS+sensorLatencyUS)
			{ // This really should not be the current frame
				DEBUG_TIME("Only handling past SOF %d from %ldus ago now when expecting frame %d, with latest frame interval %.2fms!\n", SOFID, frameTimeUS, frameIDObs, newInterval);
				frameSync.frameSOFs.pop();
				if (frameSync.frameSOFs.empty())
					DEBUG_TIME("Expected obs frame ID %d has no SOF yet!\n", frameIDObs);
				continue;
			}
			DEBUG_TIME("Older SOF %d might actually be for current frame %d (frametime %ldus < %d) - adopting and then trying newer SOF!\n",
				SOFID, frameIDObs, frameTimeUS, sensorLatencyUS+sensorLatencyUS);

			frameTimeUSNext = frameTimeUS;
			frameSync.frameID = SOFID;
			frameSync.frameSOFs.pop();
			continue;
		}

		// Consume this SOF, may take it or a future one
		frameTimeUSNext = frameTimeUS;
		frameSync.estArrival = SOF.second + std::chrono::microseconds(sensorLatencyUS);
		frameSync.frameID = SOFID;
		frameSync.frameSOFs.pop();

		// Check delay model of trigger to receiving the frame through V4L2
		if (!frameSync.frameSOFs.empty() && frameTimeUS > frameSync.sensorLatency.avg + frameSync.sensorLatency.stdDev()*2)
		{ // Check next frame if it could be our SOF instead
			DEBUG_TIME("SOF %d (obs %d) is a bit old, frametime %ldus - adopting and then trying newer SOF!\n",
				SOFID, frameIDObs, frameTimeUS);
			continue;
		}

		// Take this SOF as reference
		break;
	}

	if (HAS_LOGGED())
		DEBUG_TIME("-> Ended up with SOF %d, with %d frames passed and %d without trigger!\n", frameSync.frameID, frameSync.frameID - frameIDPre, frameSync.frameID - frameIDObs);

	if (frameTimeUSNext > 0)
		frameSync.sensorLatency.update(frameTimeUSNext);
}

static void estimateFramePacing(TimePoint_t frameReceived, bool frameDelayed, uint32_t obsAdvance)
{
	float newInterval = dtUS(frameSync.estArrival, frameReceived);
	if (!frameSync.config.continuous)
	{
		frameSync.estInterval.update(newInterval / obsAdvance);
		frameSync.estArrival = frameReceived;
		frameSync.frameID += obsAdvance;
		return;
	}

	// Determine frames passed and update frameID
	int framesPassed = obsAdvance;
	float expInterval = frameSync.estInterval.num < 10? frameSync.config.intervalUS : frameSync.estInterval.floating;
	float expFrameIntervals = newInterval / expInterval;
	int32_t expAdvance = std::max(1, (int)(expFrameIntervals+0.2f));
	if (std::abs(expFrameIntervals - obsAdvance) > 1)
	{
		printf("May have dropped frames! %.1f frame times passed, but only got %d new frames!\n", expFrameIntervals, obsAdvance);
		if (obsAdvance == 1)
		{
			printf("    Determined to likely have skipped %d frames with no trigger!\n", expAdvance-obsAdvance);
			framesPassed = expAdvance;
		}
	}
	frameSync.frameID += framesPassed;

	if (frameSync.estInterval.num == 0)
	{ // Accept arrival time as it is in the beginning
		frameSync.estArrival = frameReceived;
		return;
	}

	if (!frameDelayed)
	{ // Update arrival time estimate with probably accurate reading (may have waited AND be delayed)
		if (frameSync.estInterval.floating - frameSync.estInterval.avg > frameSync.estInterval.stdDev())
		{ // May have changed configured interval, reset to adapt stats
			printf("Reset frame interval estimate after drifting from %.1fHz to %.1fHz - configured %.2fHz!\n",
				1000000.0f/frameSync.estInterval.avg, 1000000.0f/frameSync.estInterval.floating, 1000000.0f/frameSync.config.intervalUS);
			frameSync.estInterval.reset();
		}
		frameSync.estInterval.update(newInterval / framesPassed);
	}

	// Advance SOF prediction
	frameSync.estArrival += std::chrono::microseconds((int)(frameSync.estInterval.floating*framesPassed));
}

uint32_t getNextFrameID(TimePoint_t frameReceived, bool frameDelayed, uint32_t obsAdvance, TimePoint_t &estArrival)
{
	if (frameSync.config.synced)
		correctFromFrameSync(frameReceived, frameDelayed, obsAdvance);
	else
		estimateFramePacing(frameReceived, frameDelayed, obsAdvance);
	estArrival = frameSync.estArrival;
	return frameSync.frameID;
}
