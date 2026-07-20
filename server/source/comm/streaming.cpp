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

void RemoveCameraSync(StreamState &state, TrackingCameraState &camera)
{ // Sync is either external or only for camera, in which case it will be removed, too
	if (!camera.sync)
		return;

	{ // Remove camera from its sync group
		auto sync_lock = camera.sync->lock();
		int last = 0;
		for (int c = 0; c < sync_lock->cameras.size(); c++)
		{
			if (!sync_lock->cameras[c]) continue;
			if (sync_lock->cameras[c].get() == &camera)
				sync_lock->cameras[c] = nullptr;
			else last = c;
		}
		sync_lock->cameras.resize(last);
	}

	// Remove group and source from StreamState if this was last reference
	if (camera.sync.use_count() == 2 && camera.sync->group.use_count() == 2)
		state.syncGroups.erase(std::remove(state.syncGroups.begin(), state.syncGroups.end(), camera.sync->group));
	if (camera.sync.use_count() == 2)
		state.syncSources.erase(std::remove(state.syncSources.begin(), state.syncSources.end(), camera.sync));

	camera.sync = nullptr;
	camera.syncIndex = -1;
}

void SetCameraSyncNone(StreamState &state, std::shared_ptr<TrackingCameraState> &camera, float frameIntervalMS)
{ // Sync is either external or only for camera, in which case it will be removed, too
	if (camera->sync && camera->sync->rlock()->type == SYNC_NONE)
	{ // Already have internal sync
		camera->sync->lock()->frameIntervalMS = frameIntervalMS;
		return;
	}
	RemoveCameraSync(state, *camera);
	// Generate new internal sync
	std::shared_ptr<SyncSource> sync = std::make_shared<SyncSource>(std::make_shared<Synchronised<SyncGroup>>());
	sync->generating = true;
	{
		auto sync_lock = sync->lock();
		sync_lock->type = SYNC_NONE;
		sync_lock->frameIntervalMS = frameIntervalMS;
		sync_lock->cameras.push_back(camera); // new shared_ptr
	}
	state.syncGroups.push_back(sync->group); // new shared_ptr
	state.syncSources.push_back(sync); // new shared_ptr
	camera->sync = std::move(sync);
	camera->syncIndex = 0;
}

void SetCameraSync(StreamState &state, std::shared_ptr<TrackingCameraState> &camera, std::shared_ptr<SyncSource> &sync)
{ // Sync is either external or only for camera, in which case it will be removed, too
	if (camera->sync == sync) return; // Already have same external sync
	RemoveCameraSync(state, *camera);
	camera->sync = sync; // new shared_ptr
	auto sync_lock = camera->sync->lock();
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
 * Finds the full FrameID of a recent or imminent frame via past frames of the SyncSource
 */
FrameID EstimateFullFrameID(const SyncSource &source, const SyncGroup &sync, TruncFrameID frameID)
{
	FrameID ref;
	if (source.generating)
	{
		if (sync.frames.empty()) return frameID;
		ref = sync.frames.back().ID;
	}
	else
	{
		if (source.frames.empty()) return frameID;
		ref = source.frames.back().sourceFrameID;
	}
	return ref + shortDiff<TruncFrameID, int>(ref, frameID, std::numeric_limits<TruncFrameID>::max()/2);
}

static TimePoint_t EstimateSOF(SyncGroup &sync, FrameID frameID)
{
	if (sync.type == SYNC_TRIG)
	{ // Try to estimate frame interval first
		auto frameStart = std::find_if(sync.frames.begin(), sync.frames.end(), [](const SyncedFrame &frame) { return !frame.approxSOF; });
		auto frameEnd = std::find_if(sync.frames.rbegin(), sync.frames.rend(), [](const SyncedFrame &frame) { return !frame.approxSOF; });
		if (frameStart != sync.frames.end() && frameEnd != sync.frames.rend() && frameEnd->ID != frameStart->ID)
		{ // Predicting frame interval
			if (frameEnd->ID <= frameStart->ID)
				LOG(LSOF, LWarn, "Can't easily extrapolate externally synced SOF for frame ID %d with only references %d and %d (diff %d) being %.2fms apart\n",
				frameID, frameStart->ID, frameEnd->ID, frameEnd->ID - frameStart->ID, dtMS<float>(frameStart->SOF, frameEnd->SOF));
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
 * Attempt to remap the given source frame to a groups SyncedFrame by SOF time
 * May fail if generating sources SOF was not handled yet (may consistently occur due to USB polling order)
 */
static SyncedFrame* RemapFrameID(SyncSource &source, SyncGroup &sync, FrameID frameID, TimePoint_t frameSOF)
{
	SyncedFrame *matchedFrame = nullptr;
	float matchedDiffMS = 10000;
	for (auto frame = sync.frames.rbegin(); frame != sync.frames.rend(); frame++)
	{
		float timeDiffMS = std::abs(dtMS(frame->SOF, frameSOF));
		if (timeDiffMS > matchedDiffMS) break;
		matchedFrame = &*frame;
		matchedDiffMS = timeDiffMS;
	}

	if (matchedDiffMS > sync.frameIntervalMS/4)
		return nullptr;

	if (matchedDiffMS > 1)
		LOG(LStreaming, LDarn, "Frame SOF of secondary SyncSource differs by %.3fms! Did sync propagation fail?", matchedDiffMS);

	matchedFrame->receivedSOFs++;
	return matchedFrame;
}

/**
 * Try to get the synced frame corresponding to the source frameID
 */
static SyncedFrame *GetSyncedFrame(SyncSource &source, SyncGroup &sync, TruncFrameID frameID)
{
	if (source.generating)
	{ // Generating SyncSource determines FrameID used for SyncGroup
		auto frame = std::find_if(sync.frames.rbegin(), sync.frames.rend(),
			[&](auto &f){ return (f.ID&0xFF) == frameID; });
		return frame == sync.frames.rend()? nullptr : &*frame;
	}

	// Other SyncSources FrameID may diverge and needs to be mapped
	auto frameMap = std::find_if(source.frames.rbegin(), source.frames.rend(),
		[&](auto &f){ return (f.sourceFrameID&0xFF) == frameID; });
	if (frameMap == source.frames.rend())
	{ // SOF of this source was likely dropped or severely delayed somewhere in USB stack
		// This will result in dropped or missing packets!
		LOG(LSOF, LDarn, "Attempted to to map secondary frame ID %d to group frame, but no SOF was received!", frameID);
		return nullptr;
	}

	if (frameMap->mapped)
	{ // Already mapped SOF of this source to SOF of generating source
		auto frame = std::find_if(sync.frames.rbegin(), sync.frames.rend(),
			[&](auto &f){ return f.ID == frameMap->groupFrameID; });
		if (frame != sync.frames.rend()) 
			return &*frame;
		// Should not happen, frames are only dropped from SyncGroup after they're finally processed
		LOG(LSOF, LError, "Secondary frame ID %d was mapped to group frame %d but that did not exist anymore!",
			frameMap->sourceFrameID, frameMap->groupFrameID);
		return nullptr;
	}

	// SOF has not been mapped yet, perhaps it arrived before SOF of generating source (very common)
	SyncedFrame *mappedFrame = RemapFrameID(source, sync, frameMap->sourceFrameID, frameMap->sourceFrameTime);
	if (mappedFrame)
	{ // Mapped correctly now that SOF of generating source also arrived
		LOG(LSOF, LTrace, "Finally remapped pending SOF %d to group frame %d!", frameMap->sourceFrameID, mappedFrame->ID);
		frameMap->groupFrameID = mappedFrame->ID;
		frameMap->mapped = true;
	}
	else
	{ // Perhaps time sync of either this or generating source was WAY off
		// Or SOF of generating source was dropped or severely delayed somewhere in USB stack
		LOG(LSOF, LWarn, "Pending SOF %d existed but failed to map to group frame AGAIN - %.2fms later!",
				frameMap->sourceFrameID, dtMS(frameMap->sourceFrameTime, sclock::now()));
	}
	return mappedFrame;
}


/* Register packets */

/**
 * Set start of frame with given ID
 */
void RegisterSOF(SyncSource &source, SyncGroup &sync, FrameID frameID, TimePoint_t SOF)
{
	if (!source.generating)
	{ // Map this SOF to that of the generating SyncSource by time alone
		SyncedFrame *mappedFrame = RemapFrameID(source, sync, frameID, SOF);
		source.frames.push_back({ SOF, frameID, mappedFrame? mappedFrame->ID : 0, mappedFrame != nullptr });
		if (!mappedFrame)
		{ // This SOF arrived earlier than the generating SyncSources frame SOF, record for later mapping (very common)
			LOG(LSOF, LTrace, "Received SOF %d for secondary source but generating source SOF was not handled yet!", frameID);
		}
		return;
	}

	// Sanity-check for any existing frame record
	auto frameIt = std::find_if(sync.frames.rbegin(), sync.frames.rend(),
		[&](auto &f){ return (f.ID&0xFF) == frameID; });
	if (frameIt != sync.frames.rend())
	{ // Don't support SOF arriving after a packet of that frame anymore. This should not happen.
		LOG(LSOF, LError, "Received SOF %d but already had that frame recorded!\n", frameID);
		return;
	}

	// Register frame
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

/**
 * Set frame with given ID to expect frame data from camera
 */
SyncedFrame *RegisterCameraFrame(SyncSource &source, SyncGroup &sync, int index, TruncFrameID frameID)
{
	SyncedFrame *frame = GetSyncedFrame(source, sync, frameID);
	if (!frame)
	{ // Missed SOF
		LOG(LStreaming, LDarn, "Camera %u received streaming packet announcement for source frame %d (%d) but frame SOF wasn't registered yet!\n",
			sync.cameras[index]->id, EstimateFullFrameID(source, sync, frameID), frameID);
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (frame->cameras[index].announced)
	{ // Duplicate packet from past frame?
		LOG(LStreaming, LWarn, "Camera %u announced packet for frame %d (SOF %fms ago) but it was already announced and frame is %s processed!\n",
			sync.cameras[index]->id, frame->ID, dtMS(frame->SOF, sclock::now()),
			frame->finallyProcessed? "finally" : (frame->previouslyProcessed? "partially" : "not"));
		return nullptr;
	}
	frame->expecting++;
	frame->cameras[index].announced = true;
	return frame;
}

/**
 * Register the stream packet from camera for frame with given ID
 */
SyncedFrame *RegisterStreamPacket(SyncSource &source, SyncGroup &sync, int index, TruncFrameID frameID, TimePoint_t packetTime)
{
	SyncedFrame *frame = GetSyncedFrame(source, sync, frameID);
	if (!frame)
	{ // Missed SOF
		LOG(LStreaming, LDarn, "Camera %u received streaming packet header for source frame %d (%d) but frame SOF wasn't registered yet!\n",
			sync.cameras[index]->id, EstimateFullFrameID(source, sync, frameID), frameID);
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (!frame->cameras[index].announced)
	{ // Missed Announcement
		LOG(LStreaming, LDarn, "Camera %u received streaming packet header for frame %d but it wasn't announced!\n",
			sync.cameras[index]->id, frame->ID);
		// The stream processing might not be waiting for this frame, but we may still process it
		// Just because the announcement packet had e.g. a wrong checksum, doesn't make the streaming data invalid
		//return nullptr;
		frame->expecting++;
		frame->cameras[index].announced = true;
	}
	if (frame->cameras[index].receiving)
	{ // Duplicate packet from past frame?
		LOG(LStreaming, LWarn, "Camera %u received streaming packet header for frame %d but it was already receiving a packet!\n",
			sync.cameras[index]->id, frame->ID);
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
			frame->ID, frame->ID&0xFF, frame->receiving, frame->expecting);
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
SyncedFrame *RegisterStreamBlock(SyncSource &source, SyncGroup &sync, int index, TruncFrameID frameID)
{
	SyncedFrame *frame = GetSyncedFrame(source, sync, frameID);
	if (!frame)
	{ // Shouldn't happen
		LOG(LStreaming, LWarn, "Camera %u, received complete streaming packet for source frame %d(%d) but frame wasn't recorded at all!\n",
			sync.cameras[index]->id, EstimateFullFrameID(source, sync, frameID), frameID);
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (!frame->cameras[index].announced)
	{ // Shouldn't happen
		LOG(LStreaming, LWarn, "Camera %u, received streaming packet block for frame %d(%d) but it wasn't announced and header wasn't received!\n",
			sync.cameras[index]->id, frame->ID, frame->ID&0xFF);
		return nullptr;
	}
	if (!frame->cameras[index].receiving)
	{ // Shouldn't happen
		LOG(LStreaming, LWarn, "Camera %u, received streaming packet block for frame %d(%d) but header wasn't received!\n",
			sync.cameras[index]->id, frame->ID, frame->ID&0xFF);
		return nullptr;
	}
	if (frame->finallyProcessed)
	{ // Probably delayed block so frame was processed without this camera
		LOG(LStreaming, LDarn, "Registering new block for camera %u %.2fms into frame %d(%d) after it was already finally processed %.2fms ago!",
			sync.cameras[index]->id, dtMS(frame->SOF, sclock::now()), frame->ID, frame->ID&0xFF, dtMS(frame->lastProcessed, sclock::now()));
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
SyncedFrame *RegisterStreamPacketComplete(SyncSource &source, SyncGroup &sync, int index, TruncFrameID frameID, CameraFrameRecord &&cameraFrame, bool erroneous)
{
	SyncedFrame *frame = GetSyncedFrame(source, sync, frameID);
	if (!frame)
	{ // Shouldn't happen
		LOG(LStreaming, LError, "Camera %u, received complete streaming packet for source frame %d (%d) but frame wasn't recorded at all!\n",
			sync.cameras[index]->id, EstimateFullFrameID(source, sync, frameID), frameID);
		return nullptr;
	}
	frame->cameras.resize(sync.cameras.size());
	if (!frame->cameras[index].announced)
	{ // Shouldn't happen
		LOG(LStreaming, LWarn, "Camera %u, received complete streaming packet for frame %d (%d) but it wasn't announced and header wasn't received!\n",
			sync.cameras[index]->id, frame->ID, frame->ID&0xFF);
		return nullptr;
	}
	if (!frame->cameras[index].receiving)
	{ // Shouldn't happen
		LOG(LStreaming, LWarn, "Camera %u, received complete streaming packet for frame %d (%d) but header wasn't received!\n",
			sync.cameras[index]->id, frame->ID, frame->ID&0xFF);
		return nullptr;
	}
	frame->cameras[index].complete = true;
	frame->cameras[index].erroneous = erroneous;
	frame->cameras[index].record = std::move(cameraFrame);
	frame->completed++;
	if (frame->completed > frame->expecting)
	{ // Shouldn't happen
		LOG(LStreaming, LWarn, "ERROR: Frame %d (%d) has %d packets marked completed but %d announced!\n",
			frame->ID, frame->ID&0xFF, frame->completed, frame->expecting);
	}
	return frame;
}


/* Frame/Stream Management */

/**
 * Check all sync groups for delayed and complete frames
 */
bool MaintainStreamState(StreamState &state)
{
	TimePoint_t now = sclock::now();
	state.lastMaintainTime = now;
	bool startedProcessing = false;

	for (int s = 0; s < state.syncSources.size(); s++)
	{
		assert(state.syncSources[s].use_count() >= 2);
	}

	for (int s = 0; s < state.syncGroups.size(); s++)
	{
		auto sync = state.syncGroups[s]->contextualLock();
		assert(state.syncGroups[s].use_count() >= (sync->type == SYNC_VIRTUAL? 1 : 2));

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
				frame->lastProcessed = sclock::now();
				frame->previouslyProcessed = true;
				frame->dataProcessed = true; // Reset dirty flag
				frame->finallyProcessed = !premature;
				startedProcessing = true;
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
					LOG(LStreaming, LInfo, "    After %d processed frames: %d delayed, %d eventually outdated, caused by packets: incomplete %d, erroneous %d\n",
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
				if (frame->completed == frame->expecting)
					LOG(LStreaming, LDarn, "Finally ending completed frame %d (%d) after %.2fms, when already finally processed %.2fms ago!\n", frame->ID, frame->ID&0xFF, frameMS, dtMS(frame->lastProcessed, sclock::now()));
				else
					LOG(LStreaming, LDarn, "Received new data after frame was already finally processed, ignoring!");
				frame->dataProcessed = true; // Just to not print this message repeatedly
				frame++;
				continue;
			}
			if (frame->expecting == 0)
			{ // Nothing arrived yet
				frame++;
				continue;
			}
			// Need to process frames with no data, unprocessed data, and frames with new data after premature processing

			//assert(frame->completed <= sync->cameras.size());
			if (frame->completed > frame->expecting)
			{
				LOG(LStreaming, LError, "Now got %d/%d completed~ skipping", frame->completed, (int)frame->cameras.size());
				frame++;
				continue;
			}
			if (frame->completed == frame->expecting)
			{ // Fully finished frame, and no new delayed frame announcements are expected
				if (frame->previouslyProcessed)
					LOG(LStreaming, LDarn, "Finally ending completed frame %d (%d) after %.2fms after initial processing!\n", frame->ID, frame->ID&0xFF, frameMS);
				else
				 	LOG(LStreaming, LTrace, "- Ending completed frame %d (%d) after %.2fms!\n", frame->ID, frame->ID&0xFF, frameMS);
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
							LOG(LStreaming, LDarn, "- Frame %d (%d) with %d packets received must've been announced before currently completed frame %d (%d)!"
								"SOF Delta %.2fms, but took %.2fms to receive this frame!\n",
								old->ID, old->ID&0xFF, old->receiving, frame->ID, frame->ID&0xFF, dtMS(frame->SOF, old->SOF), dtMS(frame->SOF, frame->lastBlock));
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
			else
			{ // Haven't processed anything yet, and frame is not complete yet, but may want to start prematurely processing for realtime purposes
				//float packetFirstMS = frame->receiving? dtMS(frame->firstPacket, now) : 0;
				//(frame->receiving && (frame->cameraPackets.size() == frame->completed) && (packetLastMS > 1 || packetFirstMS > 4))
				float maxLatency = sync->latency.num > 50? sync->latency.avg + 5 * sync->latency.stdDev() : 10000.0f;
				if ((frameMS > maxLatency && packetLastMS > 5) || packetLastMS > 10)
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

		//if (!sync->frames.empty() && sync->source != SYNC_EXTERNAL &&
		//	dtMS(sync->frames.back().SOF, sclock::now()) > sync->frameIntervalMS*1.5f)
		//{ // Either intentionally virtual, or haven't even received a SOF - bridge with empty frame
		// TODO: Disabled for now, would only be useful to update with IMU samples as there are no new optical samples
		if (!sync->frames.empty() && sync->type == SYNC_VIRTUAL &&
			dtMS(sync->frames.back().SOF, sclock::now()) > sync->frameIntervalMS)
		{ // Continue virtual frames
			SyncedFrame frame = {};
			frame.cameras.resize(sync->cameras.size());
			frame.ID = sync->frames.back().ID + 1;
			frame.approxSOF = sync->type != SYNC_VIRTUAL;
			frame.SOF = EstimateSOF(*sync, frame.ID);
			sync->frames.push_back(std::move(frame));
			sync->frameCount++;
			if (sync->type != SYNC_VIRTUAL)
				LOG(LStreaming, LDarn, "Replacing completely dropped frame with virtual frame!");
		}
	}
	return startedProcessing;
}