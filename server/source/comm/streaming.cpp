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

//#define LOG_MAX_LEVEL LTrace

#include "comm/streaming.hpp"

#include "device/tracking_camera.hpp"

#include "util/log.hpp"
#include "util/util.hpp"

#include <chrono>
#include <cmath>
#include <cassert>

/**
 * Streaming input from cameras in packets
 */

#define MEASURE_FRAME_RATE

// server.hpp
void ProcessStreamFrame(SyncGroup&, SyncedFrame&, bool);

/* Variables */

const int keepFramesFor = 100;
const int keepFramesMax = 10;
const int keepFramesMin = 2; // To be able to extrapolate SOFs, esp. for external sync inputs


/* Sync Group Management */

void ClearSyncGroup(SyncGroup &sync)
{
	assert(sync.source != SYNC_NONE);
	for (auto &c : sync.cameras)
	{
		if (!c) continue; // Removed while streaming
		c->sync = nullptr;
		c->syncIndex = -1;
	}
	sync.cameras.clear();
	ResetSyncGroup(sync);
}

void DeleteSyncGroup(StreamState &state, std::shared_ptr<Synchronised<SyncGroup>> &&sync)
{
	auto sg = std::find_if(state.syncGroups.begin(), state.syncGroups.end(), [&sync](const auto &sg) { return sg == sync; });
	state.syncGroups.erase(sg);
	auto sync_lock = sync->contextualLock();
	for (auto &c : sync_lock->cameras)
	{
		if (!c) continue; // Removed while streaming
		c->sync = nullptr;
		c->syncIndex = -1;
	}
	// May still exist afterwards if referenced elsewhere, e.g. as another controllers sync, so they should be deleted soon after
}

void RemoveCameraSync(StreamState &state, TrackingCameraState &camera)
{ // Sync is either external or only for camera, in which case it will be removed, too
	if (camera.sync)
	{
		auto sync_lock = camera.sync->contextualLock();
		if (sync_lock->source == SYNC_NONE)
		{ // No other camera in sync group
			auto sg = std::find_if(state.syncGroups.begin(), state.syncGroups.end(), [&camera](const auto &sg) { return sg == camera.sync; });
			state.syncGroups.erase(sg);
		}
		else
		{
			auto c = std::find_if(sync_lock->cameras.begin(), sync_lock->cameras.end(), [&camera](const auto &c) { return c && c->id == camera.id; });
			if (std::next(c) == sync_lock->cameras.end())
				sync_lock->cameras.erase(c);
			else // Don't reorder list so other syncIndices stay valid
			 	*c = nullptr;
		}
	}
	camera.sync = nullptr;
	camera.syncIndex = -1;
}

void SetCameraSyncNone(StreamState &state, std::shared_ptr<TrackingCameraState> &camera, float frameIntervalMS)
{ // Sync is either external or only for camera, in which case it will be removed, too
	if (camera->sync && camera->sync->contextualRLock()->source == SYNC_NONE)
	{ // Already have internal sync
		camera->sync->contextualLock()->frameIntervalMS = frameIntervalMS;
		return;
	}
	RemoveCameraSync(state, *camera);
	// Generate new internal sync
	camera->sync = std::make_shared<Synchronised<SyncGroup>>();
	state.syncGroups.push_back(camera->sync); // new shared_ptr
	auto sync_lock = camera->sync->contextualLock();
	sync_lock->source = SYNC_NONE;
	sync_lock->cameras.push_back(camera); // new shared_ptr
	sync_lock->frameIntervalMS = frameIntervalMS;
	camera->syncIndex = 0;
}

void SetCameraSync(StreamState &state, std::shared_ptr<TrackingCameraState> &camera, std::shared_ptr<Synchronised<SyncGroup>> &sync)
{ // Sync is either external or only for camera, in which case it will be removed, too
	if (camera->sync == sync) return; // Already have same external sync
	RemoveCameraSync(state, *camera);
	camera->sync = sync; // new shared_ptr
	auto sync_lock = camera->sync->contextualLock();
	int i = 0; 
	while (i < sync_lock->cameras.size() && sync_lock->cameras[i]) i++;
	if (i == sync_lock->cameras.size())
	{ // Add to end
		camera->syncIndex = sync_lock->cameras.size();
		sync_lock->cameras.push_back(camera); // new shared_ptr
	}
	else
	{ // Add in the middle
		camera->syncIndex = i;
		sync_lock->cameras[i] = camera; // new shared_ptr
	}
}

/**
 * Reset frame state for this sync group
 */
void ResetSyncGroup(SyncGroup &sync)
{
	sync.SOFincrease = std::chrono::microseconds(0);
	sync.SOFdiff.reset();
	sync.SOFSwitch.reset();

	sync.frames.clear();
	sync.frameCount = 0;
	sync.frameProcessedCount = 0;
	sync.packetMissingCount = 0;
	sync.frameOutdatedCount = 0;
	sync.frameDelayedCount = 0;
	sync.packetErroneousCount = 0;
	sync.procLatency.reset();
	sync.latency.reset();
	sync.lastStatUpdate = sclock::now();
}

/**
 * Reset the stream state to initial values
 * Requires the sync groups to be setup before
 */
void ResetStreamState(StreamState &state)
{
	for (auto &sync : state.syncGroups)
		ResetSyncGroup(*sync->contextualLock());
}

/* Frame Records Management */

/**
 * Finds the full FrameID of a recent or imminent frame via last frame records
 */
FrameID EstimateFullFrameID(const SyncGroup &sync, TruncFrameID frameID)
{
	if (sync.frames.empty())
		return frameID;
	return sync.frames.back().ID + shortDiff<TruncFrameID, int>(sync.frames.back().ID, frameID, std::numeric_limits<TruncFrameID>::max()/2, std::numeric_limits<TruncFrameID>::max()+1);
}

/**
 * Returns the frame record for frameID if it exists or NULL
 */
SyncedFrame *FindSyncedFrame(SyncGroup &sync, TruncFrameID frameID)
{
	for (auto frame = sync.frames.end(); frame != sync.frames.begin();)
	{ // Search in active frames
		frame--;
		if ((frame->ID&0xFF) == frameID)
			return frame.operator->();
	}
	return NULL;
}

static TimePoint_t EstimateSOF(SyncGroup &sync, FrameID frameID)
{
	if (sync.source == SYNC_EXTERNAL)
	{ // Try to estimate frame interval first
		auto frameStart = std::find_if(sync.frames.begin(), sync.frames.end(), [](const SyncedFrame &frame) { return !frame.approxSOF; });
		auto frameEnd = std::find_if(sync.frames.rbegin(), sync.frames.rend(), [](const SyncedFrame &frame) { return !frame.approxSOF; });
		if (frameStart != sync.frames.end() && frameEnd != sync.frames.rend() && frameEnd->ID > frameStart->ID)
		{ // Predicting frame interval
			sync.frameIntervalMS = 0.5f*sync.frameIntervalMS * 0.5f*dtMS<float>(frameStart->SOF, frameEnd->SOF) / (frameEnd->ID - frameStart->ID);
		}
		else
		{ // External, unpredictable sync and no way to estimate
			LOG(LSOF, LWarn, "Couldn't extrapolate externally synced SOF for frame ID %d since none of the %d previous stored frames had a good SOF\n", frameID, (int)sync.frames.size());
			return sclock::now();
		}
	}
	// With frame interval estimated (or known), estimate average SOF

	// Extrapolate SOF from past SOFs
	auto reference = sclock::now();
	StatValue<float, StatDistribution> diffUS = {};
	for (auto frame = sync.frames.rbegin(); frame != sync.frames.rend(); frame++)
	{
		if (frame->approxSOF) continue;
		int framesPassed = frameID-frame->ID;
		if (framesPassed < 0) continue;
		TimePoint_t predSOF = frame->SOF + std::chrono::microseconds((frameID-frame->ID) * (int)(sync.frameIntervalMS * 1000));
		diffUS.update(dtUS(reference, predSOF));
	}
	if (diffUS.num == 0)
	{ // Should only happen for first frames or if all frames (min keepFramesMin) had no SOF
		LOG(LSOF, LWarn, "Couldn't extrapolate SOF for frame ID %d since none of the %d previous stored frames had a good SOF\n", frameID, (int)sync.frames.size());
		return sclock::now();
	}
	if (diffUS.stdDev() > 100)
	{
		LOG(LSOF, LWarn, "Had troubles extrapolating SOF for frame ID %d with %d/%d previous frames providing a std deviation of %.2fms\n",
			frameID, diffUS.num, (int)sync.frames.size(), diffUS.stdDev()/1000.0f);
	}
	return reference + std::chrono::microseconds((int)diffUS.avg);
}

/**
 * Get the frame record for frameID or create a new one with approximate SOF time
 */
SyncedFrame *GetSyncedFrame(SyncGroup &sync, TruncFrameID frameID, bool create)
{
	for (auto frame = sync.frames.rbegin(); frame != sync.frames.rend(); frame++)
	{ // Search in active frames
		if ((frame->ID&0xFF) == frameID)
			return &*frame;
	}
	if (!create) return nullptr;
	
	FrameID lastSOFID = sync.frames.empty()? 0 : sync.frames.back().ID;
	if (sync.frames.size() > 1)
		lastSOFID = std::max(lastSOFID, std::prev(sync.frames.end(), 2)->ID);

	// No SOF was received for this frame yet, create new frame record with estimated SOF
	SyncedFrame frame = {};
	frame.cameras.resize(sync.cameras.size());
	frame.ID = EstimateFullFrameID(sync, frameID);

	if (frame.ID > lastSOFID+2)
	{
		LOG(LSOF, LError, "Tried to extrapolate %d >> lastSOFID %d\n", frame.ID, lastSOFID);
	}

	frame.approxSOF = true;
	frame.SOF = EstimateSOF(sync, frame.ID);
	LOG(LStreaming, LDebug, "Extrapolated frame ID from %d to %d using past frames!\n", frameID, frame.ID);
	sync.frames.push_back(std::move(frame));
	sync.frameCount++;
	return &sync.frames.back();
}


/* Register packets */

/**
 * Set start of frame with given ID
 */
void RegisterSOF(SyncGroup &sync, FrameID frameID, TimePoint_t SOF)
{
	// Update or create frame record with SOF time
	auto frame = sync.frames.begin();
	while (frame != sync.frames.end())
	{ // Find existing frame record
		if (frame->ID == frameID)
			break;
		frame++;
	}
	if (frame != sync.frames.end())
	{ // Retroactively update SOF
		LOG(LSOF, LTrace, "Already recorded frame %d for %fms before this SOF!\n",
			frame->ID, dtMS(frame->SOF, SOF));
		if (frame->receivedSOFs > 0)
		{ // Received SOF from another controller in the same sync group, merge SOF times
			int diffUS = dtUS(frame->SOF, SOF);
			if (std::abs(diffUS) > 100)
			{
				LOG(LSOF, LDarn, "--------- Additional SOF for frame %d, has a difference of %dus from prior SOF estimate %.2fms ago - likely bad time sync!",
					frame->ID, diffUS, dtMS(frame->SOF, sclock::now()));
			}
			frame->SOF += std::chrono::microseconds(diffUS/(frame->receivedSOFs+1));
		}
		else
		{ // Shouldn't happen, SOF should be first packet for frameID
			LOG(LSOF, LDarn, "--------- SOF wasn't first packet of frame %d with %d prior SOFs and %d blocks, retroactively changed SOF time from %s %.2fms ago to %.2fms ago!",
				frame->ID, frame->receivedSOFs, frame->blockCounter, frame->approxSOF? "approximately" : "exactly\n", dtMS(frame->SOF, sclock::now()), dtMS(SOF, sclock::now()));
			frame->SOF = SOF;
			frame->approxSOF = false;
		}
	}
	else
	{ // Register frame
		SyncedFrame frame = {};
		frame.cameras.resize(sync.cameras.size());
		frame.ID = frameID;
		frame.SOF = SOF;
		frame.approxSOF = false;
		frame.receivedSOFs = 1;
		LOG(LSOF, LTrace, "Registered SOF %d as new frame!\n", frame.ID);
		sync.frames.push_back(std::move(frame));
		sync.frameCount++;
	}
}

/**
 * Set frame with given ID to expect frame data from camera
 */
SyncedFrame *RegisterCameraFrame(SyncGroup &sync, int index, TruncFrameID frameID)
{
	// TODO: Keep frames as circular buffer
	// don't require the first camera to announce the frame to have a Frame Record already existing for it
	// Because currently, no cameras announcing => no SyncedFrame to... record the frame
	SyncedFrame *frame = GetSyncedFrame(sync, frameID, false);
	if (!frame)
	{ // Missed SOF
		LOG(LStreaming, LDarn, "Camera received streaming packet announcement for frame %d but frame SOF wasn't registered yet!\n", EstimateFullFrameID(sync, frameID));
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (frame->cameras[index].announced)
	{ // Duplicate packet from past frame?
		LOG(LStreaming, LWarn, "Camera announced packet for frame %d (SOF %fms ago) but it was already announced and frame is %s processed!\n",
			frameID, dtMS(frame->SOF, sclock::now()), frame->finallyProcessed? "finally" : (frame->previouslyProcessed? "partially" : "not"));
		return nullptr;
	}
	frame->expecting++;
	frame->cameras[index].announced = true;
	return frame;
}

/**
 * Register the stream packet from camera for frame with given ID
 */
SyncedFrame *RegisterStreamPacket(SyncGroup &sync, int index, TruncFrameID frameID, TimePoint_t packetTime)
{
	// Record that frame data is being received
	SyncedFrame *frame = GetSyncedFrame(sync, frameID, false);
	if (!frame)
	{ // Missed SOF
		LOG(LStreaming, LDarn, "Camera received streaming packet header for frame %d but frame SOF wasn't registered yet!\n", EstimateFullFrameID(sync, frameID));
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (!frame->cameras[index].announced)
	{ // Missed Announcement
		LOG(LStreaming, LDarn, "Camera received streaming packet header for frame %d but it wasn't announced!\n", frame->ID);
		return nullptr;
	}
	if (frame->cameras[index].receiving)
	{ // Duplicate packet from past frame?
		LOG(LStreaming, LWarn, "Camera received streaming packet header for frame %d but it was already receiving a packet!\n", frame->ID);
		return nullptr;
	}
	frame->cameras[index].receiving = true;
	frame->lastBlock = packetTime;
	if (!frame->receiving)
		frame->firstPacket = packetTime;
	frame->receiving++;
	if (frame->receiving > frame->expecting)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "ERROR: Frame %d (%d) has %d packets marked receiving but %d announced!\n",
			frame->ID, frameID, frame->receiving, frame->expecting);
	}

	// Update statistics
	float frameMS = dtMS(frame->SOF, packetTime);
	sync.procLatency.update(frameMS);
	sync.frameCount++;
	return frame;
}

/**
 * Register the stream block from camera for frame with given ID
 */
SyncedFrame *RegisterStreamBlock(SyncGroup &sync, int index, TruncFrameID frameID)
{
	SyncedFrame *frame = GetSyncedFrame(sync, frameID, false);
	if (!frame)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "Camera received complete streaming packet for frame %d but frame wasn't recorded at all!\n", EstimateFullFrameID(sync, frameID));
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (!frame->cameras[index].announced)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "Camera received streaming packet block for frame %d but it wasn't announced and header wasn't received!\n", frame->ID);
		frame->cameras[index].announced = true;
	}
	if (!frame->cameras[index].receiving)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "Camera received streaming packet block for frame %d but header wasn't received!\n", frame->ID);
		frame->cameras[index].receiving = true;
	}
	if (frame->finallyProcessed)
	{ // Probably delayed block so frame was processed without this camera
		LOG(LStreaming, LWarn, "Registering new block for camera %d when frame %d(%d) is already finally processed!",
			sync.cameras[index]->id, frame->ID, frameID);
		return nullptr;
	}
	// TODO: Reenable assert(!frame->finallyProcessed); - it should hold true, but doesn't always
	frame->lastBlock = sclock::now();
	frame->blockCounter++;
	frame->dataProcessed = false; // Set dirty flag
	return frame;
}

/**
 * Mark data from camera for frame with given ID as complete
 */
SyncedFrame *RegisterStreamPacketComplete(SyncGroup &sync, int index, TruncFrameID frameID, CameraFrameRecord &&cameraFrame, bool erroneous)
{
	SyncedFrame *frame = GetSyncedFrame(sync, frameID, false);
	if (!frame)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "Camera received complete streaming packet for frame %d but frame wasn't recorded at all!\n", EstimateFullFrameID(sync, frameID));
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (!frame->cameras[index].announced)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "Camera received complete streaming packet for frame %d but it wasn't announced and header wasn't received!\n", frame->ID);
		frame->cameras[index].announced = true;
	}
	if (!frame->cameras[index].receiving)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "Camera received complete streaming packet for frame %d but header wasn't received!\n", frame->ID);
		frame->cameras[index].receiving = true;
	}
	frame->cameras[index].complete = true;
	frame->cameras[index].erroneous = erroneous;
	frame->cameras[index].record = std::move(cameraFrame);
	frame->completed++;
	if (frame->completed > frame->expecting)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "ERROR: Frame %d (%d) has %d packets marked completed but %d announced!\n",
			frame->ID, frameID, frame->completed, frame->expecting);
	}
	return frame;
}


/* Frame/Stream Management */

/**
 * Check all sync groups for delayed and complete frames
 */
void MaintainStreamState(StreamState &state)
{
	TimePoint_t now = sclock::now();
	for (int s = 0; s < state.syncGroups.size(); s++)
	{
		auto sync = state.syncGroups[s]->contextualLock();
		// Check for old frames
		auto frame = sync->frames.begin();
		while (frame != sync->frames.end())
		{
			float frameMS = dtMS(frame->SOF, now);
			float packetLastMS = frame->receiving? dtMS(frame->lastBlock, now) : 0;

			auto processFrame = [&](bool premature)
			{
				assert(!frame->finallyProcessed);
				ProcessStreamFrame(*sync, *frame, premature);
				frame->previouslyProcessed = true;
				frame->dataProcessed = true; // Reset dirty flag
				frame->finallyProcessed = !premature;
			};
			auto registerFrameEnd = [&](bool complete)
			{
#ifdef MEASURE_FRAME_RATE
				sync->frameProcessedCount++;
				if (frame->previouslyProcessed) sync->frameDelayedCount++;
				if (frame->outdated) sync->frameOutdatedCount++;
				if (sync->frameCount > 10 && complete)
					sync->latency.update(frameMS);
				bool missing = false, erroneous = false;
				for (auto &packet : frame->cameras)
				{
					if (!packet.complete) sync->packetMissingCount++;
					if (packet.erroneous) sync->packetErroneousCount++;
				}
				const int deltaF = 1000;
				if (sync->frameProcessedCount % deltaF == 0)
				{
					float deltaT = dtMS(sync->lastStatUpdate, sclock::now());
					LOG(LStreaming, LInfo, "Group %d: Frame Rate: %.2f / Latency %.2fms +-%.2fms - Max %.2fms\n",
						s, sync->frameProcessedCount/deltaT*1000.0f, sync->latency.avg, sync->latency.stdDev(), sync->latency.max);
					LOG(LStreaming, LInfo, "    After %d processed frames: %d delayed, %d eventually outdated, caused by packets: missing %d, erroneous %d\n",
						sync->frameProcessedCount, sync->frameDelayedCount, sync->frameOutdatedCount,
						sync->packetMissingCount, sync->packetErroneousCount);
					sync->frameProcessedCount = 0;
					sync->frameOutdatedCount = 0;
					sync->frameDelayedCount = 0;
					sync->packetMissingCount = 0;
					sync->packetErroneousCount = 0;
					sync->latency.reset();
					sync->lastStatUpdate = sclock::now();
				}
#endif
			};

			if (frameMS > keepFramesFor && sync->frames.size() > keepFramesMin && (!frame->receiving || packetLastMS > 5))
			{ // Finally remove frame, whether completed or not

				if (!frame->finallyProcessed)
				{ // Never got a non-premature processing, do here for offline saving reasons only
					registerFrameEnd(false);
					processFrame(false);
				}

				frame = sync->frames.erase(frame);
				continue;
			}
			if (frame->previouslyProcessed && frame->dataProcessed)
			{ // Finished frame, or delayed frame already processed prematurely, now waiting for new data or to be deleted
				frame++;
				continue;
			}
			if (frame->finallyProcessed)
			{
				LOG(LStreaming, LError, "For some reason, already finallyProcessed with prev: %d, new data? %d", frame->previouslyProcessed? 1 : 0, frame->dataProcessed? 1 : 0);
			}
			assert(!frame->finallyProcessed);
			// Need to process frames with no data, unprocessed data, and frames with new data after premature processing

			//assert(frame->completed <= sync->cameras.size());
			if (frame->completed > frame->cameras.size())
			{
				LOG(LStreaming, LError, "Now got %d/%d completed~ skipping", frame->completed, (int)frame->cameras.size());
				frame++;
				continue;
			}
			if (frame->completed == frame->cameras.size())
			{ // Fully finished frame, and no new delayed frame announcements are expected
				LOG(LStreaming, frame->previouslyProcessed? LDebug : LTrace, "- Ending completed frame %d (%d) after %.2fms!\n", frame->ID, frame->ID&0xFF, frameMS);
				assert(!frame->finallyProcessed);
				registerFrameEnd(true);
				processFrame(false);
				
				if (!frame->outdated)
				{ // Mark older incomplete frames as outdated for realtime processing
					for (auto old = sync->frames.begin(); old != frame; old++)
					{
						if (old->ID < frame->ID)
						{
							old->outdated = true;
						}
						else
						{
							LOG(LStreaming, LDarn, "- Frame %d (%d) with %d packets received must've been announced before currently completed frame %d (%d)! Actual dt: %fms\n",
								old->ID, old->ID&0xFF, old->receiving, frame->ID, frame->ID&0xFF, dtMS(old->firstPacket, frame->firstPacket));
						}
					}
				}

				sync->lastFrameEndTime = now;
			}
			else if (frame->previouslyProcessed)
			{ // New data for already delayed frame, continue with another premature frame processing
				LOG(LStreaming, LDebug, "- Processing next part of delayed frame %d (%d) after %.2fms!\n", frame->ID, frame->ID&0xFF, frameMS);
				processFrame(true);
			}
			else if (sync->frameProcessedCount > 50)
			{ // Haven't processed anything yet, and frame is not complete yet, but may want to start prematurely processing for realtime purposes
				//float packetFirstMS = frame->receiving? dtMS(frame->firstPacket, now) : 0;
				//(frame->receiving && (frame->cameraPackets.size() == frame->completed) && (packetLastMS > 1 || packetFirstMS > 4))
				float maxLatency = sync->latency.avg + 5 * sync->latency.stdDev();
				if ((frameMS > maxLatency && packetLastMS > 2) || packetLastMS > 5)
				{ // Start prematurely processing partial data for low-latency use
					LOG(LStreaming, LDarn,
						"- Frame %d (%d) is %.2fms%s into the expected frame (avg %.2fms, allowed %.2fms) "
						"- %d(+%d delayed)/%d cameras, with %d blocks read and %.2fms since last packet!",
						frame->ID, frame->ID&0xFF, frameMS, frame->approxSOF? " (est.)" : "", sync->latency.avg, maxLatency,
						frame->completed, (int)frame->expecting-frame->completed, (int)frame->cameras.size(),
						frame->blockCounter, packetLastMS);
					processFrame(true);
				}
			}

			frame++;
		}

		// Check if new "empty" frame should be added
		if (sync->frames.empty() && sync->source == SYNC_VIRTUAL)
		{
			SyncedFrame frame = {};
			frame.cameras.resize(sync->cameras.size());
			frame.ID = 0;
			frame.approxSOF = false;
			frame.SOF = sclock::now();
			LOG(LStreaming, LInfo, "Started virtual frame generation!\n");
			sync->frames.push_back(std::move(frame));
			sync->frameCount++;
		}
		else if (dtMS(sync->frames.back().SOF, sclock::now()) > sync->frameIntervalMS*1.5f)
		{
			SyncedFrame frame = {};
			frame.cameras.resize(sync->cameras.size());
			frame.ID = sync->frames.back().ID + 1;
			frame.approxSOF = sync->source != SYNC_VIRTUAL;
			frame.SOF = EstimateSOF(*sync, frame.ID);
			sync->frames.push_back(std::move(frame));
			sync->frameCount++;
		}
	}
}