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

#include "pipeline.hpp"

#include "target/tracking2D.hpp"
#include "target/detection3D.hpp"
#include "target/detection2D.hpp"

#include "tracking/cluster.hpp"

#include "util/debugging.hpp"
#include "util/log.hpp"
#include "util/eigenutil.hpp"

#include "dbscan/dbscan.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <numeric>

// ----------------------------------------------------------------------------
// General 3D- and 2D Marker Tracking
// ----------------------------------------------------------------------------

void ResetTrackingPipeline(PipelineState &pipeline)
{
	pipeline.tracking.trackedMarkers.clear();
	pipeline.tracking.trackedTargets.clear();
	pipeline.tracking.dormantTargets.clear();
	pipeline.tracking.dormantMarkers.clear();
	pipeline.tracking.orphanedIMUs.clear();
}

void InitTrackingPipeline(PipelineState &pipeline)
{
	ResetTrackingPipeline(pipeline);
	for (auto &imu : pipeline.record.imus)
		pipeline.tracking.orphanedIMUs.emplace_back(imu, pipeline.params.track);
	// Trackers (both markers and targets) will be added when triggered, and IMUs reassigned
}

/**
 * Create a tracked target record for given target id
 */
static TrackerRecord& createTrackerRecord(std::shared_ptr<FrameRecord> &frame, int id, TrackingResult result)
{
	auto trackRecord = std::find_if(frame->trackers.begin(), frame->trackers.end(),
		[&](auto &t){ return t.id == id; });
	if (trackRecord == frame->trackers.end())
	{ // Enter into frame record
		frame->trackers.push_back({ id, result });
		return frame->trackers.back();
	}
	// May happen in replay mode when jumping back and re-tracking area
	trackRecord->result = result; // Update
	return *trackRecord;
}

/**
 * Update tracked target record with tracker target data
 */
static void recordTrackerTarget(TrackerRecord &record, const TrackerTarget &target, bool keepInternalData)
{
	record.error = target.match2D.error;
	if (keepInternalData)
	{
		record.match2D = target.match2D;
	}
	updateVisibleMarkers(record.visibleMarkers, target.match2D);
}

/**
 * Update tracked target record with inertial data
 */
static void recordTrackerInertial(TrackerRecord &record, const TrackerInertial &inertial, bool keepInternalData)
{
	if (inertial)
	{
		if (dtMS(inertial.fusion.lastIntegration, inertial.fusion.time) < 20)
		{
			if (inertial.calibration.phase < IMU_CALIB_DONE)
				record.imuState = TrackerInertialState::IMU_CALIBRATING;
			else
				record.imuState = TrackerInertialState::IMU_TRACKING;
		}
		else
			record.imuState = TrackerInertialState::IMU_LOST;

		record.imuSampleInterval = inertial.fusion.sampleInterval.floating;
		record.imuLastSample = inertial.fusion.lastIntegration;
	}
	else
		record.imuState = TrackerInertialState::NO_IMU;
}

/**
 * Update tracked target record with tracker pose
 */
static void recordTrackerObservation(TrackerRecord &record, const TrackerObservation &pose, bool keepInternalData)
{
	record.posePredicted = pose.predicted;
	record.covPredicted = pose.covPredicted;
	record.poseExtrapolated = pose.extrapolated;
	record.poseInertialIntegrated = pose.inertialIntegrated;
	record.poseInertialFused = pose.inertialFused;
	record.poseInertialFiltered = pose.inertialFiltered;
	if (record.result.isTracked())
	{
		record.poseObserved = pose.observed;
		record.covObserved = pose.covObserved;
	}
	else
	{
		record.poseObserved.setIdentity();
		record.covObserved.setZero();
	}
	record.poseFiltered = pose.filtered;
	record.covFiltered = pose.covFiltered;
}

/**
 * Consider recording tracked target into database for optimisation
 */
static void recordTrackingTargetData(PipelineState &pipeline, int trackerID, const TrackerTarget &tracker, std::shared_ptr<FrameRecord> &frame)
{
	// Consider recording target in optimisation database
	auto &params = pipeline.params.cont.targetObs;
	int sampleCount = 0, strongCameras = 0;
	for (auto &camPts : tracker.match2D.points2D)
	{
		if (camPts.size() >= params.minCameraSamples)
			sampleCount += camPts.size();
		if (camPts.size() >= params.minStrongCameraSamples)
			strongCameras++;
	}
	if (strongCameras >= params.minStrongCameras && sampleCount >= params.minTotalSamples)
	{
		auto db_lock = pipeline.obsDatabase.contextualLock();
		auto targetIt = std::find_if(db_lock->targets.begin(), db_lock->targets.end(),
			[&](auto &tgt){ return tgt.targetID == trackerID; });
		if (targetIt == db_lock->targets.end())
		{
			db_lock->targets.push_back({ trackerID });
			targetIt = std::prev(db_lock->targets.end());
			targetIt->markers.reserve(tracker.calib.markers.size());
			for (auto &marker : tracker.calib.markers)
			{
				targetIt->markerMap.emplace((int)targetIt->markers.size(), (int)targetIt->markers.size());
				targetIt->markers.push_back(marker.pos);
			}
		}
		if (!targetIt->frames.empty() && targetIt->frames.back().frame >= frame->num)
			return;
		targetIt->frames.push_back({});
		auto &tgtFrame = targetIt->frames.back();
		tgtFrame.error = tracker.match2D.error.mean;
		tgtFrame.pose = tracker.match2D.pose;
		tgtFrame.frame = frame->num;
		tgtFrame.samples.reserve(sampleCount);
		for (int c = 0; c < tracker.match2D.points2D.size(); c++)
		{
			if (tracker.match2D.points2D[c].size() < params.minCameraSamples)
			{ // TODO: Consider adding those where all points were a good, clear match (pass TrackedTarget in here?)
				continue;
			}
			// Easily add all
			for (auto &pt : tracker.match2D.points2D[c])
				tgtFrame.samples.emplace_back(pt.first, c, frame->cameras[c].rawPoints2D[pt.second]);
			targetIt->totalSamples += tracker.match2D.points2D[c].size();
		}

		/* ScopedLogCategory scopedLogCategory(LOptimisation);
		ScopedLogLevel scopedLogLevel(LInfo);
		LOGCL("Added frame to optimisation database, new error:");
		updateReprojectionErrors(*db_lock, pipeline.getCalibs()); */
	}
}

static TrackerRecord &recordTrackingResult(PipelineState &pipeline, std::shared_ptr<FrameRecord> &frame, const TrackedTarget &tracker, TrackingResult result)
{
	TrackerRecord &record = createTrackerRecord(frame, tracker.id, result);
	if (result.isTracked())
	{
		recordTrackerObservation(record, tracker.pose, pipeline.keepInternalData);
		recordTrackerTarget(record, tracker.target, pipeline.keepInternalData);
		if (!result.hasFlag(TrackingResult::FILTER_FAILED))
			recordTrackingTargetData(pipeline, tracker.id, tracker.target, frame);
	}
	else if (result.isDetected())
	{
		recordTrackerObservation(record, tracker.pose, pipeline.keepInternalData);
		recordTrackerTarget(record, tracker.target, pipeline.keepInternalData);
	}
	else if (result.isState(TrackingResult::NO_TRACK))
	{ // Failed to track, but not removed
		recordTrackerObservation(record, tracker.pose, pipeline.keepInternalData);
		recordTrackerTarget(record, tracker.target, pipeline.keepInternalData);
	}
	recordTrackerInertial(record, tracker.inertial, pipeline.keepInternalData);
	return record;
}

static TrackerRecord &retroactivelyTrackFrame(PipelineState &pipeline, TrackedTarget &tracker, std::shared_ptr<FrameRecord> &frame)
{
	frame->finishedProcessing = false; // TODO: Not entirely thread-safe. But don't want a mutex here if we can avoid it
	std::vector<CameraCalib> calibs = pipeline.getCalibs();
	// TODO: Only take calibs of cameras actually part of the frame!
	// Would need to iterate over frame->cameras and take cameras that have points
	std::vector<std::vector<Eigen::Vector2f> const *> points2D(calibs.size());
	std::vector<std::vector<BlobProperty> const *> properties(calibs.size());
	std::vector<std::vector<int>> remainingPoints2D(calibs.size());
	std::vector<std::vector<int> const *> relevantPoints2D(calibs.size());
	for (int c = 0; c < calibs.size(); c++)
	{
		points2D[c] = &frame->cameras[calibs[c].index].points2D;
		properties[c] = &frame->cameras[calibs[c].index].properties;
		// TODO: Get remainingPoints2D from FrameRecord
		remainingPoints2D[c].resize(points2D[c]->size());
		std::iota(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), 0);
		relevantPoints2D[c] = &remainingPoints2D[c];
	}

	if (tracker.inertial)
		integrateIMU(tracker.state, tracker.inertial, tracker.pose, frame->time, pipeline.params.track);

	TrackingResult result = trackTarget(tracker.state, tracker.target, tracker.pose,
		calibs, points2D, properties, relevantPoints2D,
		frame->time, frame->num, pipeline.cameras.size(), pipeline.params.track);

	if (result.isTracked() && tracker.inertial)
		postCorrectIMU(tracker, tracker.state, tracker.inertial, tracker.pose, frame->time, pipeline.params.track);

	result.setFlag(TrackingResult::CATCHING_UP);
	auto &record = recordTrackingResult(pipeline, frame, tracker, result);
	frame->finishedProcessing = true;
	return record;
}

static bool detectTargetAsync(std::stop_token stopToken, PipelineState &pipeline, std::shared_ptr<FrameRecord> &frame,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<int>> &detectionPoints2D,
	bool useProbe, int focus, Eigen::Vector3f pos, int probeCount, DormantTarget &&dormant)
{
	{ // Detect in the first frame
		std::vector<std::vector<Eigen::Vector2f> const *> points2D(calibs.size());
		std::vector<std::vector<BlobProperty> const *> properties(calibs.size());
		std::vector<std::vector<int> const *> relevantPoints2D(calibs.size());
		for (int c = 0; c < calibs.size(); c++)
		{
			points2D[c] = &frame->cameras[calibs[c].index].points2D;
			properties[c] = &frame->cameras[calibs[c].index].properties;
			relevantPoints2D[c] = &detectionPoints2D[c];
		}

		frame->finishedProcessing = false; // TODO: Not entirely thread-safe. But don't want a mutex here if we can avoid it
		if (useProbe)
		{ // Probe target against clusters points
			dormant.target.match2D = probeTarget2D(stopToken, dormant.target.calib, calibs, points2D, properties, relevantPoints2D,
				pos, pipeline.cameras.size(), probeCount, pipeline.params.detect, pipeline.params.track, dormant.target.data);
		}
		else
		{ // Detect target first in focusCameras 2D points, and then match with others
			dormant.target.match2D = searchTarget2D(stopToken, dormant.target.calib, calibs, points2D, properties, relevantPoints2D,
				focus, pipeline.cameras.size(), pipeline.params.detect, pipeline.params.track, dormant.target.data);
		}
		if (dormant.target.match2D.error.samples < pipeline.params.detect.minObservations.total)
		{
			createTrackerRecord(frame, dormant.id, useProbe? TrackingResult::PROBED_2D : TrackingResult::SEARCHED_2D);
			frame->finishedProcessing = true;
			return false;
		}

		LOG(LDetection2D, LInfo, useProbe?
			"    Detected target using probe in frame %d (now %d) with %d 2D points and %fpx mean error!\n" :
			"    Detected target using search in frame %d (now %d) with %d 2D points and %fpx mean error!\n",
			frame->num, (int)pipeline.frameNum.load(), dormant.target.match2D.error.samples, dormant.target.match2D.error.mean*PixelFactor);
	}

	Eigen::Isometry3f pose = dormant.target.match2D.pose;
	TrackedTarget tracker(std::move(dormant), pose, frame->time, frame->num, pipeline.params.track);
	recordTrackingResult(pipeline, frame, tracker, useProbe? TrackingResult::DETECTED_P2D : TrackingResult::DETECTED_S2D);
	frame->finishedProcessing = true;

	if (stopToken.stop_requested())
		return false;

	// Then continue on tracking to catch up with the latest realtime frame
	// TODO: Skip some frames to catch up faster, should handle some skips as long as prediction is accurate
	// But may also want intermediate frames tracked for recording purposes, so do those afterwards anyway

	TimePoint_t lastTimestamp = frame->time;
	std::size_t frameIndex = frame->num;

	{
		auto framesRecord = pipeline.record.frames.getView<false>();
		if (frameIndex >= framesRecord.endIndex())
		{ // FrameRecords were cleared while starting detection, abort
			LOG(LDetection2D, LDarn, "    Detection - Frame %d: Frames were cleared while detecting!\n", (int)frameIndex);
			return false;
		}
		auto frameRecordIt = framesRecord.pos(frameIndex+1);
		for (; frameRecordIt < framesRecord.end(); frameRecordIt++)
		{
			assert(frameRecordIt.accessible());
			if (!*frameRecordIt) continue;
			auto &frameRecord = *frameRecordIt->get();
			if (!frameRecord.finishedProcessing)
				break;
			for (auto &trackRecord : frameRecord.trackers)
			{
				if (trackRecord.id != tracker.id) continue;
				LOG(LDetection2D, LDebug, "    Detection - Frame %d: Caught up to a frame already tracked!\n", (int)frameRecordIt.index());
				return true; // Already tracked, maybe detected by 3D triangulation detection
			}
			TrackerRecord &trackRecord = retroactivelyTrackFrame(pipeline, tracker, *frameRecordIt);
			if (!trackRecord.result.isTracked())
			{ // Don't record as real loss since this is retroactive
				LOG(LDetection2D, LDarn, "    Detection - Frame %d: Failed to find continuation of target with %d observations and %.3fpx mean error!\n",
					frame->num, tracker.target.match2D.error.samples, tracker.target.match2D.error.mean*PixelFactor);
				return false;
			}
			LOG(LDetection2D, LTrace, "    Detection - Frame %d: Pixel Error after 2D target track: %fpx mean over %d points\n",
				frame->num, tracker.target.match2D.error.mean*PixelFactor, tracker.target.match2D.error.samples);
			lastTimestamp = frameRecord.time;
			if (stopToken.stop_requested())
				return false;
		}
		// Release view when leaving scope
		frameIndex = frameRecordIt.index()-1;
	}

	LOG(LDetection2D, LDebug, "    Detection - Frame %d: Caught up to snapshot, syncing with realtime!\n", (int)frameIndex);

	std::unique_lock pipeline_lock(pipeline.pipelineLock);
	if (stopToken.stop_requested())
		return false;

	{ // Finish the rest of the frames that were processed while waiting for the lock
		auto framesRecord = pipeline.record.frames.getView<false>();
		if (frameIndex >= framesRecord.endIndex())
		{ // FrameRecords were cleared while waiting for the lock, abort
			LOG(LDetection2D, LDarn, "    Detection - Frame %d: Frames were cleared while syncing with realtime!\n", (int)frameIndex);
			return false;
		}
		auto frameRecordIt = framesRecord.pos(frameIndex+1);
		for (; frameRecordIt < framesRecord.end(); frameRecordIt++)
		{
			assert(frameRecordIt.accessible());
			if (!*frameRecordIt) continue;
			auto &frameRecord = *frameRecordIt->get();
			if (!frameRecord.finishedProcessing)
				break;
			for (auto &trackRecord : frameRecord.trackers)
			{
				if (trackRecord.id != tracker.id) continue;
				LOG(LDetection2D, LDebug, "    Detection - Frame %d: Caught up to a frame already tracked!\n", (int)frameRecordIt.index());
				return true; // Already tracked, maybe detected by 3D triangulation detection
			}

			TrackerRecord &trackRecord = retroactivelyTrackFrame(pipeline, tracker, *frameRecordIt);
			if (!trackRecord.result.isTracked())
			{ // Don't record as real loss since this is retroactive
				LOG(LDetection2D, LDarn, "    Detection - Frame %d: Failed to find continuation of target with %d observations and %.3fpx mean error!\n",
					frame->num, tracker.target.match2D.error.samples, tracker.target.match2D.error.mean*PixelFactor);
				return false;
			}
			LOG(LDetection2D, LTrace, "    Detection - Frame %d: Pixel Error after 2D target track: %fpx mean over %d points\n",
				frame->num, tracker.target.match2D.error.mean*PixelFactor, tracker.target.match2D.error.samples);
			lastTimestamp = frameRecord.time;
			if (stopToken.stop_requested())
				return false;
		}
		// Release view when leaving scope
		frameIndex = frameRecordIt.index()-1;
	}

	LOG(LDetection2D, LDebug, "    Detection - Frame %d: Caught up to most recent processed frame!\n", (int)frameIndex);

	// Finally, no more frames to catch up on, register as new tracked target
	int erased = std::erase_if(pipeline.tracking.dormantTargets, [&](const auto &d){ return d.id == tracker.id; });
	assert(erased == 1);
	pipeline.tracking.trackedTargets.push_back(std::move(tracker));
	SignalTrackerDetected(tracker.id);
	return true;
}

void RetroactivelySimulateFilter(PipelineState &pipeline, std::size_t frameStart, std::size_t frameEnd)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	std::vector<CameraCalib> calibs = pipeline.getCalibs();
	std::vector<std::vector<Eigen::Vector2f> const *> points2D(pipeline.cameras.size());

	auto framesRecord = pipeline.record.frames.getView<false>();
	frameStart = std::max<unsigned int>(framesRecord.beginIndex(), frameStart-3);
	frameEnd = std::min<unsigned int>(framesRecord.endIndex(), frameEnd);
	auto frameRecordIt = framesRecord.pos(frameStart);
	while ((!*frameRecordIt || !frameRecordIt->get()->finishedProcessing) && frameRecordIt.index() < frameEnd) frameRecordIt++;
	for (; frameRecordIt.index() < frameEnd; frameRecordIt++)
	{
		if (!*frameRecordIt || !frameRecordIt->get()->finishedProcessing) continue;
		FrameRecord &frameRecord = *frameRecordIt->get();
		for (int c = 0; c < pipeline.cameras.size(); c++)
			points2D[c] = &frameRecord.cameras[calibs[c].index].points2D;
		for (auto &trackRecord : frameRecord.trackers)
		{
			auto &targets = pipeline.tracking.trackedTargets;
			auto targetIt = std::find_if(targets.begin(), targets.end(),
				[&](auto &e){ return e.id == trackRecord.id; });
			if (targetIt == targets.end()) continue; // Not interested in lost targets anyway
			if (targetIt->state.lastObsFrame > frameRecord.num)
			{ // Initialise filter as good as possible
				targetIt->state = TrackerState(trackRecord.poseFiltered, frameRecord.time, pipeline.params.track);
				targetIt->state.lastObsFrame = frameRecord.num;
				targetIt->state.lastObservation = frameRecord.time;
				targetIt->state.lastIMUSample = -1;
				continue;
			}

			if (targetIt->inertial)
				integrateIMU(targetIt->state, targetIt->inertial, targetIt->pose, frameRecord.time, pipeline.params.track);

			trackRecord.result = simulateTrackTarget(targetIt->state, targetIt->target, targetIt->pose,
				calibs, points2D, trackRecord, frameRecord.time, frameRecord.num, pipeline.params.track);

			if (trackRecord.result.isTracked() && targetIt->inertial)
				postCorrectIMU(*targetIt, targetIt->state, targetIt->inertial, targetIt->pose, frameRecord.time, pipeline.params.track);

			recordTrackerObservation(trackRecord, targetIt->pose, pipeline.keepInternalData);
			recordTrackerTarget(trackRecord, targetIt->target, pipeline.keepInternalData);
		}
	}
}

void UpdateTrackingPipeline(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras, std::shared_ptr<FrameRecord> &frame, bool trackTargets)
{
	pclock::time_point start, trk0, trk1, tri0, tri1, tri2, tri3, det0, det1, det2, tpt0, tpt1;
	start = pclock::now();

	// Aggregate points and camera calibrations
	std::vector<CameraCalib> calibs(cameras.size());
	std::vector<std::vector<Eigen::Vector2f> const *> points2D(cameras.size());
	std::vector<std::vector<BlobProperty> const *> properties(cameras.size());
	std::vector<std::vector<int>> remainingPoints2D(calibs.size());
	std::vector<std::vector<int> const *> relevantPoints2D(calibs.size());
	for (int c = 0; c < cameras.size(); c++)
	{
		calibs[c]= cameras[c]->calib;
		points2D[c] = &frame->cameras[calibs[c].index].points2D;
		properties[c] = &frame->cameras[calibs[c].index].properties;
		remainingPoints2D[c].resize(points2D[c]->size());
		std::iota(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), 0);
		relevantPoints2D[c] = &remainingPoints2D[c];
	}

	auto &track = pipeline.tracking;
	int camCount = pipeline.cameras.size();
	std::vector<int> triIndices;

	auto occupyTargetMatches = [&](const TargetMatch2D &targetMatch2D)
	{
		for (int c = 0; c < calibs.size(); c++)
		{
			int i = calibs[c].index;
			remainingPoints2D[c].erase(std::remove_if(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), [&](int p) {
				for (auto &match : targetMatch2D.points2D[i])
					if (match.second == p)
						return true;
				return false;
			}), remainingPoints2D[c].end());			
		}
		// Find and efficiently remove triangulated points if they are occupied
		triIndices.erase(std::remove_if(triIndices.begin(), triIndices.end(), [&](int p) {
			auto &tri = track.triangulations3D[p];
			for (int c = 0; c < calibs.size(); c++)
			{
				int i = calibs[c].index;
				for (auto &match : targetMatch2D.points2D[i])
					if (tri.blobs[i] == match.second)
						return true;
			}
			return false;
		}), triIndices.end());	
	};

	{ // Integrate orphaned IMU up until recent frame
		int i = 0;
		auto tracker = track.orphanedIMUs.begin();
		while (tracker != track.orphanedIMUs.end())
		{
			LOG(LTracking, LDebug, "Integrating orphaned IMU %d!\n", i++);
			integrateIMU(tracker->state, tracker->inertial, tracker->pose, frame->time, pipeline.params.track);
			tracker++;
		}
		// TODO: Associate orphaned IMU with tracked target
	}

	if (trackTargets)
	{ // 6-DOF Target Tracking

		trk0 = pclock::now();

		auto tracker = track.trackedTargets.begin();
		while (tracker != track.trackedTargets.end())
		{
			LOG(LTracking, LDebug, "Tracking target %d (name %s) with %d markers!\n", tracker->id, tracker->label.c_str(), (int)tracker->target.calib.markers.size());

			if (tracker->inertial)
				integrateIMU(tracker->state, tracker->inertial, tracker->pose, frame->time, pipeline.params.track);

			TrackingResult result = trackTarget(tracker->state, tracker->target, tracker->pose,
				calibs, points2D, properties, relevantPoints2D,
				frame->time, frame->num, camCount, pipeline.params.track);
			if (result.isTracked())
			{
				LOG(LTracking, LDebug, "    Found continuation of target %d (name %s) with %d observations and %.3fpx mean error!\n",
					tracker->id, tracker->label.c_str(), tracker->target.match2D.error.samples, tracker->target.match2D.error.mean*PixelFactor);

				if (tracker->inertial)
					postCorrectIMU(*tracker, tracker->state, tracker->inertial, tracker->pose, frame->time, pipeline.params.track);

				// Occupy all 2D points of tracked target
				occupyTargetMatches(tracker->target.match2D);
				recordTrackingResult(pipeline, frame, *tracker, result);

				/* if (targetMatch2D.error.mean*PixelFactor > 0.5 && IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				} */
			}
			else if (dtMS(tracker->state.lastObservation, frame->time) < pipeline.params.track.lostTargetCoastMS &&
					dtMS(tracker->state.firstObservation, frame->time) > pipeline.params.track.coastMinTrackTime)
			{
				LOG(LTracking, LDarn, "Failed to find continuation of target %d (name %s), got %d observations and %.3fpx mean error!\n",
					tracker->id, tracker->label.c_str(), tracker->target.match2D.error.samples, tracker->target.match2D.error.mean*PixelFactor);

				recordTrackingResult(pipeline, frame, *tracker, result);

				if (IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				}

				tracker++;
				continue;
			}
			else
			{
				LOG(LTracking, LWarn, "Failed to find continuation of target %d (name %s), got %d observations and %.3fpx mean error!\n",
					tracker->id, tracker->label.c_str(), tracker->target.match2D.error.samples, tracker->target.match2D.error.mean*PixelFactor);

				result.setFlag(TrackingResult::REMOVED);
				recordTrackingResult(pipeline, frame, *tracker, result);

				if (IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				}

				// Push to dormant targets for re-detection
				track.dormantTargets.emplace_back(std::move(*tracker));
				tracker = track.trackedTargets.erase(tracker);
				continue;
			}

			if (pipeline.isSimulationMode)
			{
				// Try to find ground truth pose of what it believes it is tracking (it may have detected wrong)
				Eigen::Isometry3f gtPose = pipeline.simulation.contextualRLock()->getGTPose(tracker->id);
				std::pair<double,double> GTErrorOpt = calculatePoseError(gtPose, tracker->pose.observed);
				LOG(LTracking, LDebug, "    GT Opt Tracking Error: %fmm, %fdeg\n", GTErrorOpt.first, GTErrorOpt.second);
				std::pair<double,double> GTError = calculatePoseError(gtPose, tracker->pose.filtered);
				LOG(LTracking, LDebug, "    GT Tracking Error: %fmm, %fdeg\n", GTError.first, GTError.second);
				if (GTError.first > 10 || GTError.second > 5)
					LOG(LTracking, LDarn, "=== WAY too high tracking error!!!\n");
			}

			tracker++;
		}

		trk1 = pclock::now();
	}

	/* { // Single Marker Tracking

		tpt0 = pclock::now();

		auto tracker = pipeline.tracking.trackedMarkers.begin();
		while (tracker != pipeline.tracking.trackedMarkers.end())
		{
			int matchedPoint = -1;
			// TODO: Use 2D Points
			TrackingResult result = trackMarker(tracker->state, tracker->marker, tracker->pose,
					pipeline.tracking.points3D, triIndices, &matchedPoint, frame->time, 3);
			if (result.isTracked())
			{
				tracker++;
			}
			else
			{
				LOG(LTracking, LDarn, "Failed to find continuation of tracked point!\n");
				tracker = pipeline.tracking.trackedMarkers.erase(tracker);
				// TODO: Add to dormant markers
			}
		}

		tpt1 = pclock::now();
	} */

	std::vector<TriCluster3D> tri3DClusters;

	const auto &detect = pipeline.params.detect;
	const auto &clustering = pipeline.params.cluster;

	{ // Triangulate unoccupied 2D points

		// Clear past frames' triangulations
		track.triangulations3D.clear();
		track.discarded3D.clear();
		track.points3D.clear();

		auto &params = pipeline.params.tri;

		tri0 = pclock::now();

		// Find potential point correspondences as TriangulatedPoints
		triangulateRayIntersections(calibs, points2D, relevantPoints2D, track.triangulations3D,
			params.maxIntersectError, params.minIntersectError);

		tri1 = pclock::now();

		// Resolve conflicts by assigning points based on confidences and reevaluating confidences
		// Note this uses internal data from triangulateRayIntersections to help resolve
		resolveTriangulationConflicts(track.triangulations3D, params.maxIntersectError);

		// Remove the least confident points
		filterTriangulatedPoints(track.triangulations3D, track.discarded3D,
			params.minIntersectionConfidence);
		LOG(LTracking, LDebug, "%d triangulated points detected", (int)track.triangulations3D.size());

		tri2 = pclock::now();

		// Refine point positions
		track.points3D.reserve(track.triangulations3D.size());
		for (int p = 0; p < track.triangulations3D.size(); p++)
		{
			refineTriangulationIterative<float>(points2D, calibs, track.triangulations3D[p], params.refineIterations);
			track.points3D.push_back(track.triangulations3D[p].pos);
		}

		// TODO: Approximate 3D size of each triangulated point
		/* for (int p = 0; p < track.triangulations3D.size(); p++)
		{
			TriangulatedPoint &tri = track.triangulations3D[p];
			for (int c = 0; c < calibs.size(); c++)
			{
				if (tri.blobs[c] == InvalidBlob) continue;
				auto &record = *pipeline.currentFrame->cameras[calibs[c].index];
				int i = tri.blobs[c];
				// Properly infer 3D size here, accounting for distortion is harder
				//tri.size += record.pointSizes[i];
			}
		} */

		if (pipeline.isSimulationMode && SHOULD_LOG(LTriangulation, LTrace))
		{
			std::vector<std::pair<int, Eigen::Vector3f>> gtTris;
			{ // Copy GT triangulations
				auto sim_lock = pipeline.simulation.contextualRLock();
				if (sim_lock->triangulatedPoints3D.frame == frame->num)
				{
					gtTris = sim_lock->triangulatedPoints3D.triangulation;
					LOG(LTriangulation, LTrace, "    Simulation generated %d triangulatable points!\n", (int)gtTris.size());
				}
			}
			for (int p = 0; p < track.points3D.size(); p++)
			{
				float bestDist = std::numeric_limits<float>::max();
				int bestIndex = -1, bestMarker = -1;
				Eigen::Vector3f tri = track.points3D[p];
				for (int g = 0; g < gtTris.size(); g++)
				{
					auto gtTri = gtTris[g];
					Eigen::Vector3f diff = tri - gtTri.second;
					float dist = diff.squaredNorm();
					if (dist < bestDist)
					{
						bestIndex = g;
						bestDist = dist;
						bestMarker = gtTri.first; // source target marker (if generated by a target)
					}
				}

				if (bestIndex >= 0)
				{
					LOG(LTriangulation, LTrace, "        Tri %d had closest GT marker %d with distance of %fmm\n",
						p, bestMarker, std::sqrt(bestDist)*1000);
				}
			}
		}

		tri3 = pclock::now();

		// TODO: Split point cloud into groups of nearby points - currently unused
		tri3DClusters = dbscan<3,float, int>(track.points3D, clustering.tri3DCluster.maxDistance, clustering.tri3DCluster.minPoints);
		// May also want to consider doing it before resolving conflicts, incase a ray actually passes through two clusters
		// - in which case, that ray would still be uncertain, I guess
		LOG(LTracking, LDebug, "Clustered 3D points in %d groups in %d points!", (int)tri3DClusters.size(), (int)track.points3D.size());
		for (auto &cluster : tri3DClusters)
			LOG(LTracking, LDebug, "    Cluster has %d 3D points!", (int)cluster.size());

		// Fill triangulated point list to pass to functions
		triIndices.resize(track.triangulations3D.size());
		std::iota(triIndices.begin(), triIndices.end(), 0);
	}

	det0 = pclock::now();

	if (trackTargets && pipeline.tracking.triangulations3D.size() >= detect.tri.minPointCount)
	{ // Target detection in 3D point cloud

		auto dormantIt = pipeline.tracking.dormantTargets.begin();
		while (dormantIt != pipeline.tracking.dormantTargets.end())
		{
			if (!dormantIt->target.detectionConfig.match3D)
			{
				dormantIt++;
				continue;
			}
			DormantTarget &dormant = *dormantIt;
			LOG(LTracking, LDebug, "Trying to detect target %d (name %s) with %d markers in 3D point cloud!\n",
				dormant.id, dormant.label.c_str(), (int)dormant.target.calib.markers.size());

			// Detect target in remaining 3D point cloud
			TargetCandidate3D candidate = detectTarget3D(dormant.target.calib,
				pipeline.tracking.triangulations3D, triIndices,
				detect.tri.sigmaError, detect.tri.poseSigmaError, detect.tri.quickAssignTargetMatches);

			bool acceptCandidate = candidate.points.size() >= detect.tri.minPointCount && candidate.MSE < detect.tri.maxErrorRMSE*detect.tri.maxErrorRMSE;
			if (acceptCandidate)
			{
				// Use pose candidate to track target with 2D points
				CovarianceMatrix covariance = pipeline.params.track.filter.getSyntheticCovariance<float>() * pipeline.params.track.filter.detectSigma;
				dormant.target.match2D = trackTarget2D(dormant.target.calib, candidate.pose, covariance,
					calibs, camCount, points2D, properties, relevantPoints2D, pipeline.params.track, dormant.target.data);

				acceptCandidate = dormant.target.match2D.error.samples >= pipeline.params.track.quality.minTotalObs
								&& dormant.target.match2D.error.mean < pipeline.params.track.quality.maxTotalError;
				if (acceptCandidate)
				{ // Register as tracked target
					{ // Make sure no async detection of this target is ongoing
						if (pipeline.tracking.asyncDetection && pipeline.tracking.asyncDetectTargetID == dormant.id)
						{
							LOG(LDetection2D, LDebug, "    Interrupting async detection because tracker has been found!\n");
							pipeline.tracking.asyncDetectionStop.request_stop();
						}
					}
					Eigen::Isometry3f pose = dormant.target.match2D.pose;
					TrackedTarget tracker(std::move(dormant), pose, frame->time, frame->num, pipeline.params.track);
					occupyTargetMatches(tracker.target.match2D);
					recordTrackingResult(pipeline, frame, tracker, TrackingResult::DETECTED_M3D);
					SignalTrackerDetected(tracker.id);

					LOG(LTracking, LInfo, "    Added dormant tracked target %s back with %d points and %fmm RMSE"
						", now with %d 2D points and %fpx mean error!\n",
						dormant.label.c_str(), (int)candidate.points.size(), std::sqrt(candidate.MSE),
						tracker.target.match2D.error.samples, tracker.target.match2D.error.mean*PixelFactor);

					pipeline.tracking.trackedTargets.push_back(std::move(tracker));
				}
				else
				{
					createTrackerRecord(frame, dormant.id, TrackingResult::MATCHED_3D);
					LOG(LTracking, LDebug, "    No good candidate, best has %d points and %fmm RMSE!\n",
						(int)candidate.points.size(), std::sqrt(candidate.MSE));
				}
				
				if (IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				}
			}
			else if (candidate.points.size() > 0)
			{
				LOG(LTracking, LDebug, "    No good candidate, best has %d points and %fmm RMSE!\n",
					(int)candidate.points.size(), std::sqrt(candidate.MSE));
			}

			if (pipeline.isSimulationMode)
			{
				// Try to find ground truth pose of what it believes it is tracking (it may have detected wrong)
				Eigen::Isometry3f gtPose = pipeline.simulation.contextualRLock()->getGTPose(dormant.id);
				std::pair<double,double> GTError = calculatePoseError(gtPose, candidate.pose);
				LOG(LTracking, LDebug, "    Best candidate has GT Error %fmm, %fdeg!\n", GTError.first, GTError.second);
			}

			if (acceptCandidate)
				dormantIt = pipeline.tracking.dormantTargets.erase(dormantIt);
			else
				dormantIt++;
		}
	}

	/* { // Add single markers to track

		// TODO: Rewrite this fully to consider DormantMarkers
		// Consider cues over multiple frames from associated IMUs, too
		// E.g. keep track of markers for a bit, and eventually associate with DormantMarker to turn to TrackedMarker

		for (int p : triIndices)
		{
			Eigen::Vector3f pos = pipeline.tracking.points3D[p];

			// TODO: Add tracked marker
			// TODO: Remove 2D points
		}
	} */

	det1 = pclock::now();

	// Cluster 2D points and calculate stats of each
	std::vector<std::vector<Cluster2D>> clusters2D(calibs.size());
	std::vector<std::vector<Cluster2DStats>> cluster2DStats(calibs.size());
	for (int c = 0; c < calibs.size(); c++)
	{ // 2D Point clustering
		clusters2D[c] = dbscanSubset<2,float,int>(*points2D[c], remainingPoints2D[c], clustering.blob2DCluster.maxDistance, clustering.blob2DCluster.minPoints);
		cluster2DStats[c].resize(clusters2D[c].size());
		for (int i = 0; i < clusters2D[c].size(); i++)
			cluster2DStats[c][i] = calculateClusterStats2D(clusters2D[c][i], *points2D[c]);

		LOG(LCluster, LDebug, "Camera %d: Clustered %d/%d relevant points into %d groups:", calibs[c].index, (int)remainingPoints2D[c].size(), (int)points2D[c]->size(), (int)clusters2D[c].size());
		for (int i = 0; i < clusters2D[c].size(); i++)
		{
			LOG(LCluster, LDebug, "    Cluster has %d 2D points, variance of %fx%fpx!", (int)clusters2D[c][i].size(),
				std::sqrt(cluster2DStats[c][i].covariance(0,0))*PixelFactor, std::sqrt(cluster2DStats[c][i].covariance(1,1))*PixelFactor);
		}
	}

	// Match those 2D clusters to potential 3D clusters using triangulation
	std::vector<Cluster2DTri3D> clusters2DTri = triangulateClusters2D(cluster2DStats, calibs, clustering);
	std::sort(clusters2DTri.begin(), clusters2DTri.end(), [](auto &a, auto &b){ return a.score > b.score; });

	// TODO: Track clusters in 3D (1/2)
	std::vector<Cluster3D> trackedClusters3D;
	trackedClusters3D.reserve(clusters2DTri.size());
	for (auto &clusterTri : clusters2DTri)
	{
		trackedClusters3D.emplace_back(clusterTri.score, clusterTri.center, Eigen::Matrix3f::Identity() * 0.1f);
	}

	if (trackTargets && !clusters2DTri.empty() && !pipeline.tracking.dormantTargets.empty() && !pipeline.tracking.asyncDetection)
	{
		// Select cluster to detect in
		Cluster2DTri3D &cluster = clusters2DTri.front();

		// Select target to detect
		pipeline.tracking.dormantTargets.splice(pipeline.tracking.dormantTargets.end(), pipeline.tracking.dormantTargets, pipeline.tracking.dormantTargets.begin());
		DormantTarget &dormant = pipeline.tracking.dormantTargets.back();
		TargetDetectionConfig config = dormant.target.detectionConfig;

		// Select best camera from that cluster
		int focusPoints = 0, focusCamera = -1;
		for (int c = 0; c < calibs.size(); c++)
		{
			int cIndex = cluster.camClusters[c];
			if (cIndex >= 0 && focusPoints < clusters2D[c][cIndex].size())
			{
				focusPoints = clusters2D[c][cIndex].size();
				focusCamera = c;
			}
		}

		// Cycle through methods to use (per target)
		std::array<bool,2> options = {
			config.search2D && focusPoints >= detect.minObservations.focus,
			config.probe2D
		};
		int opt = -1;
		for (int i = 0; i < options.size(); i++, dormant.target.detectionCycle++)
		{
			if (!options[dormant.target.detectionCycle%options.size()]) continue;
			opt = dormant.target.detectionCycle%options.size();
			break;
		}
		dormant.target.detectionCycle = (dormant.target.detectionCycle+1)%4;

		if (opt != -1)
		{
			bool useProbe = opt == 1;

			LOG(LDetection2D, LDarn, opt == 1?
				"Probing for target %d (name %s) with %d markers!\n" :
				"Searching for target %d (name %s) with %d markers!\n",
				dormant.id, dormant.label.c_str(), (int)dormant.target.calib.markers.size());

			// TODO: Figure out why (async) search sucks now - did it always suck or just because of new clustering? Or a regression?

			// Prepare selection of points
			std::vector<std::vector<int>> detectionPoints2DAsync(calibs.size());
			std::vector<Cluster2D const *> detectionPoints2DSync(calibs.size());
			for (int c = 0; c < calibs.size(); c++)
			{
				int cIndex = cluster.camClusters[c];
				if (cIndex < 0) continue;
				// TODO: Track clusters in 3D (1/2) - would allow nearby points below cluster limit to be used here
				detectionPoints2DAsync[c] = clusters2D[c][cIndex];
				detectionPoints2DSync[c] = &clusters2D[c][cIndex];
			}

			if (detect.useAsyncDetection)
			{
				pipeline.tracking.asyncDetection = true;
				pipeline.tracking.asyncDetectionStop = {};
				pipeline.tracking.asyncDetectTargetID = dormant.id;
				threadPool.push([&pipeline](int, std::stop_token stopToken, std::shared_ptr<FrameRecord> frameRec,
					std::vector<CameraCalib> &calibs, std::vector<std::vector<int>> &detectionPoints2D,
					bool useProbe, int focus, Eigen::Vector3f pos, int probeCount, DormantTarget dormant)
				{ // Working with copy of dormant tracker
					detectTargetAsync(stopToken, pipeline, frameRec, calibs, detectionPoints2D, 
						useProbe, focus, pos, probeCount, std::move(dormant));
					pipeline.tracking.asyncDetection = false;
				}, pipeline.tracking.asyncDetectionStop.get_token(), frame, std::move(calibs), std::move(detectionPoints2DAsync),
					useProbe, focusCamera, cluster.center, config.probeCount, dormant);
			}
			else
			{
				pipeline.tracking.syncDetectionStop = {};
				if (useProbe)
				{ // Probe target against clusters points
					dormant.target.match2D = probeTarget2D(pipeline.tracking.syncDetectionStop.get_token(),
						dormant.target.calib, calibs, points2D, properties, detectionPoints2DSync, cluster.center,
						pipeline.cameras.size(), config.probeCount, pipeline.params.detect, pipeline.params.track, dormant.target.data);
				}
				else
				{ // Detect target first in focusCameras 2D points, and then match with others
					dormant.target.match2D = searchTarget2D(pipeline.tracking.syncDetectionStop.get_token(),
						dormant.target.calib, calibs, points2D, properties, detectionPoints2DSync,
						focusCamera, pipeline.cameras.size(), pipeline.params.detect, pipeline.params.track, dormant.target.data);
					
				}
				if (dormant.target.match2D.error.samples >= pipeline.params.detect.minObservations.total)
				{
					LOG(LDetection2D, LInfo, useProbe?
					"    Detected target using probe in frame %d (now %d) with %d 2D points and %fpx mean error!\n" :
					"    Detected target using search in frame %d (now %d) with %d 2D points and %fpx mean error!\n",
						frame->num, (int)pipeline.frameNum.load(), dormant.target.match2D.error.samples, dormant.target.match2D.error.mean*PixelFactor);
					Eigen::Isometry3f pose = dormant.target.match2D.pose;
					TrackedTarget tracker(std::move(dormant), pose, frame->time, frame->num, pipeline.params.track);
					occupyTargetMatches(tracker.target.match2D);
					recordTrackingResult(pipeline, frame, tracker, useProbe? TrackingResult::DETECTED_P2D : TrackingResult::DETECTED_S2D);
					SignalTrackerDetected(tracker.id);
					pipeline.tracking.trackedTargets.push_back(std::move(tracker));
					pipeline.tracking.dormantTargets.pop_back();
				}
				else
				{
					createTrackerRecord(frame, dormant.id, TrackingResult::SEARCHED_2D);
				}
			}
		}
	}

	det2 = pclock::now();

	// TODO: Properly integrate triangulation records (2/3)
	frame->triangulations = track.points3D;
	frame->cluster2DTri = std::move(trackedClusters3D);
	for (int c = 0; c < calibs.size(); c++)
	{
		int i = calibs[c].index;
		frame->cameras[i].clusters2D = std::move(cluster2DStats[c]);
	}

	{ // Log timing
		float frameTime = dtMS(start, pclock::now());
		static float accumFrameTime = 0.0f;
		static int accumCount = 0;
		accumFrameTime += frameTime;
		accumCount++;
		LOG(LPipeline, LDebug, "Tracking took %.3fms - average after %d cycles %f\n", frameTime, accumCount, accumFrameTime/accumCount);
		LOG(LPipeline, LDebug, "Target Tracking %.3fms; Triangulation %f ms; Target Detection 3D %.3fms; Target Detection 2D %.3fms; Marker Tracking %.3fms\n",
			dtMS(trk0, trk1), dtMS(tri0, tri3), dtMS(det0, det1), dtMS(det1, det2), dtMS(tpt0, tpt1));
		LOG(LPipeline, LDebug, "Triangulation split up: Ray Intersection %.3fms; Filtering %.3fms; Refinement %.3fms\n",
			dtMS(tri0, tri1), dtMS(tri1, tri2), dtMS(tri2, tri3));
	}
}

void SetTrackedTarget(PipelineState &pipeline, int ID, std::string label, TargetCalibration3D calib, TargetDetectionConfig detectionConfig)
{
	std::unique_lock lock (pipeline.pipelineLock); // May already be in pipeline thread - make use of recursive mutex
	for (auto &tracker : pipeline.tracking.trackedTargets)
	{
		if (tracker.id != ID) continue;
		tracker.label = label;
		tracker.target.calib = calib;
		tracker.target.detectionConfig = detectionConfig;
		return;
	}
	for (auto &tracker : pipeline.tracking.dormantTargets)
	{
		if (tracker.id != ID) continue;
		tracker.label = label;
		tracker.target.calib = calib;
		tracker.target.detectionConfig = detectionConfig;
		return;
	}
	pipeline.tracking.dormantTargets.emplace_back(ID, label, TrackerTarget(std::move(calib), detectionConfig));
}

void SetTrackedMarker(PipelineState &pipeline, int ID, std::string label, float size)
{
	std::unique_lock lock (pipeline.pipelineLock); // May already be in pipeline thread - make use of recursive mutex
	for (auto &tracker : pipeline.tracking.trackedMarkers)
	{
		if (tracker.id != ID) continue;
		tracker.label = label;
		tracker.marker.size = size;
		return;
	}
	for (auto &tracker : pipeline.tracking.dormantMarkers)
	{
		if (tracker.id != ID) continue;
		tracker.label = label;
		tracker.marker.size = size;
		return;
	}
	pipeline.tracking.dormantMarkers.emplace_back(ID, label, TrackerMarker(size));
}

bool AssociateIMU(PipelineState &pipeline, std::shared_ptr<IMU> &imu, int trackerID, IMUCalib calib)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock); // May already be in pipeline thread - make use of recursive mutex
	OrphanIMU(pipeline, imu);
	for (auto &tracker : pipeline.tracking.trackedTargets)
	{
		if (tracker.id != trackerID) continue;
		tracker.inertial = TrackerInertial(imu, calib); // new shared_ptr
		tracker.state.lastIMUSample = -1;
		std::erase_if(pipeline.tracking.orphanedIMUs, [&](const auto &t){ return t.inertial.imu == imu; });
		return true;
	}
	for (auto &tracker : pipeline.tracking.dormantTargets)
	{
		if (tracker.id != trackerID) continue;
		tracker.inertial = TrackerInertial(imu, calib); // new shared_ptr
		std::erase_if(pipeline.tracking.orphanedIMUs, [&](const auto &t){ return t.inertial.imu == imu; });
		return true;
	}
	for (auto &tracker : pipeline.tracking.trackedMarkers)
	{
		if (tracker.id != trackerID) continue;
		tracker.inertial = TrackerInertial(imu, calib); // new shared_ptr
		tracker.state.lastIMUSample = -1;
		std::erase_if(pipeline.tracking.orphanedIMUs, [&](const auto &t){ return t.inertial.imu == imu; });
		return true;
	}
	for (auto &tracker : pipeline.tracking.trackedMarkers)
	{
		if (tracker.id != trackerID) continue;
		tracker.inertial = TrackerInertial(imu, calib); // new shared_ptr
		std::erase_if(pipeline.tracking.orphanedIMUs, [&](const auto &t){ return t.inertial.imu == imu; });
		return true;
	}
	// Ensure it's orphaned
	if (std::find_if(pipeline.tracking.orphanedIMUs.begin(), pipeline.tracking.orphanedIMUs.end(),
		[&](const auto &t){ return t.inertial.imu == imu; }) == pipeline.tracking.orphanedIMUs.end())
		pipeline.tracking.orphanedIMUs.emplace_back(imu, pipeline.params.track);
	return false;
}

void DisassociateIMU(PipelineState &pipeline, int trackerID)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock); // May already be in pipeline thread - make use of recursive mutex
	std::shared_ptr<IMU> imu = nullptr;
	for (auto &tracker : pipeline.tracking.trackedTargets)
	{
		if (tracker.id != trackerID) continue;
		imu = std::move(tracker.inertial.imu);
		tracker.inertial = {};
		tracker.state.lastIMUSample = -1;
		break;
	}
	for (auto &tracker : pipeline.tracking.dormantTargets)
	{
		if (tracker.id != trackerID) continue;
		imu = std::move(tracker.inertial.imu);
		tracker.inertial = {};
		break;
	}
	for (auto &tracker : pipeline.tracking.trackedMarkers)
	{
		if (tracker.id != trackerID) continue;
		imu = std::move(tracker.inertial.imu);
		tracker.inertial = {};
		tracker.state.lastIMUSample = -1;
		break;
	}
	for (auto &tracker : pipeline.tracking.dormantMarkers)
	{
		if (tracker.id != trackerID) continue;
		imu = std::move(tracker.inertial.imu);
		tracker.inertial = {};
		break;
	}
	if (imu)
	{
		// Ensure it's orphaned
		if (std::find_if(pipeline.tracking.orphanedIMUs.begin(), pipeline.tracking.orphanedIMUs.end(),
			[&](const auto &t){ return t.inertial.imu == imu; }) == pipeline.tracking.orphanedIMUs.end())
			pipeline.tracking.orphanedIMUs.emplace_back(imu, pipeline.params.track);
	}
}

void OrphanIMU(PipelineState &pipeline, std::shared_ptr<IMU> &imu)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock); // May already be in pipeline thread - make use of recursive mutex

	// Disconnect
	for (auto &tracker : pipeline.tracking.trackedTargets)
		if (tracker.inertial.imu == imu) tracker.inertial = {};
	for (auto &tracker : pipeline.tracking.dormantTargets)
		if (tracker.inertial.imu == imu) tracker.inertial = {};
	for (auto &tracker : pipeline.tracking.trackedMarkers)
		if (tracker.inertial.imu == imu) tracker.inertial = {};
	for (auto &tracker : pipeline.tracking.dormantMarkers)
		if (tracker.inertial.imu == imu) tracker.inertial = {};

	// Ensure it's orphaned
	if (std::find_if(pipeline.tracking.orphanedIMUs.begin(), pipeline.tracking.orphanedIMUs.end(),
		[&](const auto &t){ return t.inertial.imu == imu; }) == pipeline.tracking.orphanedIMUs.end())
		pipeline.tracking.orphanedIMUs.emplace_back(imu, pipeline.params.track);
}