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

#ifndef COMM_STREAMING_H
#define COMM_STREAMING_H

#include "pipeline/record.hpp"

#include "util/util.hpp" // TimePoint_t
#include "util/stats.hpp"
#include "util/synchronised.hpp"

#include <vector>
#include <list>
#include <memory> // shared_ptr

/**
 * Streaming input from cameras in packets
 */


/* Structures */

typedef uint32_t FrameID;
typedef uint8_t TruncFrameID;

// Forward-declared opaque structs
struct TrackingCameraState;

enum SyncSource
{
	SYNC_NONE,		// Implies camera generates signal itself, actual SOF is guessed
	SYNC_VIRTUAL,	// Implies there is no cameras in sync group, generating frames merely for other hardware
	SYNC_INTERNAL,	// Implies controller generates consistent, trusted signal
	SYNC_EXTERNAL	// Implies sync timing and frameID is externally supllied and could be inconsistent
};

struct SyncedFrame
{
	FrameID ID;
	TimePoint_t SOF;
	bool approxSOF = true; // In case MC->Host drops a SOF (can be improved), or Sync is None (freerunning mode, SOF estimate)

	// Receiving state
	int receivedSOFs = 0; // Should usually equal the number of controllers involved
	int blockCounter = 0;
	TimePoint_t firstPacket;
	TimePoint_t lastBlock;

	// Camera state
	int expecting = 0, receiving = 0, completed = 0;
	struct CameraStatus
	{
		bool announced, receiving, complete, erroneous;
		CameraFrameRecord record;
	};
	std::vector<CameraStatus> cameras;

	// Processing state
	bool outdated = false;
	TimePoint_t lastProcessed;
	bool previouslyProcessed = false; // For premature processing
	bool dataProcessed = true; // For premature processing, acts as dirty flag
	bool finallyProcessed = false; // For processing after frame ended for delayed packets
};

/**
 * A group of cameras synced together and at the same framerate
 * Shared frame ID, SOF, and stats
 */
struct SyncGroup
{
	// Synced cameras in group
	SyncSource source;
	std::vector<std::shared_ptr<TrackingCameraState>> cameras;

	// Expected frame interval (may be estimated for external sync input)
	float frameIntervalMS;

	// Tracking frames
	std::list<SyncedFrame> frames;

	// Tracking SOF (Start of Frame)
	std::chrono::microseconds SOFincrease;
	StatDistf SOFdiff;
	StatDistf SOFSwitch;

	// Stats
	long long frameCount = 0;
	int frameProcessedCount; // Frames that had any data to be processed at all
	int packetMissingCount; // Frames that had missing packets from cameras
	int packetErroneousCount; // Frames that had erroneous packets from cameras
	int frameOutdatedCount; // Frames that didn't complete in time for the next frame to complete
	int frameDelayedCount; // Frames that didn't complete in time and were prematurely processed
	TimePoint_t lastFrameEndTime;
	StatAvgf procLatency; // Latency SOF->First Packet
	StatValue<float,StatDistribution|StatExtremas> latency; // Latency SOF->Last Packet
	TimePoint_t lastStatUpdate;
};
static bool operator==(const SyncGroup& a, const SyncGroup& b) { return &a == &b; }

/**
 * State of data streams from multiple cameras ports on multiple controllers
 * Each camera can be grouped in a sync group that shares sync and framerate
 */
struct StreamState
{
	std::vector<std::shared_ptr<Synchronised<SyncGroup>>> syncGroups;
};


/* Functions */

void ClearSyncGroup(SyncGroup &sync);
void DeleteSyncGroup(StreamState &state, std::shared_ptr<Synchronised<SyncGroup>> &&sync);
void RemoveCameraSync(StreamState &state, TrackingCameraState &camera);
void SetCameraSyncNone(StreamState &state, std::shared_ptr<TrackingCameraState> &camera, float frameIntervalMS);
void SetCameraSync(StreamState &state, std::shared_ptr<TrackingCameraState> &camera, std::shared_ptr<Synchronised<SyncGroup>> &sync);

/**
 * Reset frame state for this sync group
 */
void ResetSyncGroup(SyncGroup &sync);

/**
 * Reset the stream state to initial values
 * Requires controllers and their ports as well as the sync groups to be setup before
 */
void ResetStreamState(StreamState &state);

/**
 * Finds the full FrameID of a recent or imminent frame via last frame records
 */
FrameID EstimateFullFrameID(const SyncGroup &sync, TruncFrameID frameID);

/**
 * Returns the frame record for frameID if it exists or NULL
 */
SyncedFrame *FindSyncedFrame(SyncGroup &sync, TruncFrameID frameID);

/**
 * Set start of frame with given ID
 */
void RegisterSOF(SyncGroup &sync, FrameID frameID, TimePoint_t SOF);

/**
 * Set frame with given ID to expect frame data from camera
 */
SyncedFrame *RegisterCameraFrame(SyncGroup &sync, int index, TruncFrameID frameID);

/**
 * Register the stream packet from camera for frame with given ID
 */
SyncedFrame *RegisterStreamPacket(SyncGroup &sync, int index, TruncFrameID frameID, TimePoint_t packetTime);

/**
 * Mark data from camera for frame with given ID as complete
 */
SyncedFrame *RegisterStreamBlock(SyncGroup &sync, int index, TruncFrameID frameID);

/**
 * Mark data from camera for frame with given ID as complete
 */
SyncedFrame *RegisterStreamPacketComplete(SyncGroup &sync, int index, TruncFrameID frameID, CameraFrameRecord &&cameraFrame, bool erroneous);

/**
 * Check all sync groups for delayed and complete frames
 */
void MaintainStreamState(StreamState &state);

#endif // COMM_STREAMING_H