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

#include "framesync.hpp"

FrameSync framesync;

uint32_t correctFromFrameSync(FrameSync &framesync, TimePoint_t frameRecv, uint32_t frameID, uint32_t obsAdvance, uint32_t expAdvance, bool likelyDropped)
{
#ifdef LOG_TIMING
	const char *space = "";
	if (likelyDropped)
		space = "    ";
	#define DEBUG_TIME(str, ...) {\
		printf("%s" str, space, __VA_ARGS__);\
		space = "    ";\
	}
	#define HAS_LOGGED() (space[0] != '\0')
#else
	#define DEBUG_TIME(...) {}
	#define HAS_LOGGED() false
#endif

	std::unique_lock lock(framesync.access);

	// 7000us at least from trigger signal to a frame being received in the camera
	// Usually 7150us, with early timesync 7080us - just to be safe from timesync wonkyness, lower
	const unsigned int MIN_TRIGGER_TO_FRAME = 6800;

	uint32_t frameIDObs = frameID + obsAdvance;
	uint32_t frameIDExp = frameID + expAdvance;
	bool continualFrameSync = false; // If framesync is expected to be continual

	uint32_t frameIDNext = frameIDObs;
	long frameTimeUSNext = 0;

	while (!framesync.frameSOFs.empty())
	{
		auto SOF = framesync.frameSOFs.front();
		uint32_t sofFrameIDObs = frameIDObs + shortDiff<uint16_t, int>(frameIDObs&0xFF, SOF.first, 5, 0xFF);
		uint32_t sofFrameIDExp = frameIDExp + shortDiff<uint16_t, int>(frameIDExp&0xFF, SOF.first, 5, 0xFF);
		uint32_t SOFID = continualFrameSync? sofFrameIDExp : sofFrameIDObs;
		long frameTimeUS = dtUS(SOF.second, frameRecv);
		if (sofFrameIDExp != sofFrameIDObs)
		{ // Drop of frame sync longer than 255-bias frames
			DEBUG_TIME("Encountered large drop of frame sync - cannot recover frameID from last %u, with next either obs %u or exp %u, SOF either obs %u or exp %u\n",
				frameID, frameIDObs, frameIDExp, sofFrameIDObs, sofFrameIDExp);
		}
		else if (HAS_LOGGED())
		{
			DEBUG_TIME("Handling SOF %u with frame time %ldus, with current adjusted frame ID of %d!\n",
				SOFID, frameTimeUS, frameIDNext);
		}

		if (frameTimeUS <= MIN_TRIGGER_TO_FRAME && SOFID > frameIDObs)
		{ // Very likely a future SOF, take current frame ID
			DEBUG_TIME("Skipping future frame SOF %d (current observed %d), with frame time %ldus\n", SOFID, frameIDObs, frameTimeUS);
			break;
		}

		if (SOFID < frameIDObs)
		{ // Likely didn't receive SOF in time, so it wasn't accounted for before
			if (frameTimeUS > MIN_TRIGGER_TO_FRAME+MIN_TRIGGER_TO_FRAME)
			{ // This really should not be the current frame
				DEBUG_TIME("Only handling past SOF %d from %ldus ago now when expecting frame %d, with latest frame interval %.2fms!\n", SOFID, frameTimeUS, frameIDObs, newInterval);
				framesync.frameSOFs.pop();
				if (framesync.frameSOFs.empty())
					DEBUG_TIME("Expected obs frame ID %d has no SOF yet!\n", frameIDObs);
				continue;
			}
			DEBUG_TIME("Older SOF %d might actually be for current frame %d (frametime %ldus < %d) - adopting and then trying newer SOF!\n",
				SOFID, frameIDObs, frameTimeUS, MIN_TRIGGER_TO_FRAME+MIN_TRIGGER_TO_FRAME);

			framesync.frameSOFs.pop();
			frameIDNext = SOFID;
			frameTimeUSNext = frameTimeUS;
			continue;
		}

		// Consume this SOF, may take it or a future one
		framesync.frameSOFs.pop();
		frameIDNext = SOFID;
		frameTimeUSNext = frameTimeUS;

		// Check delay model of trigger to receiving the frame through V4L2
		if (!framesync.frameSOFs.empty() && frameTimeUS > framesync.SOF2RecvDelay.avg + framesync.SOF2RecvDelay.stdDev()*2)
		{ // Check next frame if it could be our SOF instead
			DEBUG_TIME("SOF %d (obs %d) is a bit old, frametime %ldus - adopting and then trying newer SOF!\n",
				SOFID, frameIDObs, frameTimeUS);
			continue;
		}

		// Take this SOF as reference
		break;
	}

	if (HAS_LOGGED())
	{
		int noTriggerFrames = frameIDNext - frameIDObs;
		DEBUG_TIME("-> Ended up with SOF %d, with %d frames passed and %d without trigger!\n", frameIDNext, advanceFrames + noTriggerFrames, noTriggerFrames);
	}

	if (frameTimeUSNext > 0)
		framesync.SOF2RecvDelay.update(frameTimeUSNext);

	return frameIDNext;
}