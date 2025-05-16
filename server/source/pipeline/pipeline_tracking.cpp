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

void InitTrackingPipeline(PipelineState &pipeline)
{
	pipeline.tracking.dormantTargets.clear();
	for (auto &target : pipeline.tracking.targetCalibrations)
		pipeline.tracking.dormantTargets.emplace_back(&target);
	pipeline.tracking.orphanedIMUs.clear();
	for (auto &imu : pipeline.record.imus)
	{
		if (!imu->tracker || !ConnectIMU(pipeline, imu))
			pipeline.tracking.orphanedIMUs.emplace_back(imu, pipeline.params.track);
	}
}

void ResetTrackingPipeline(PipelineState &pipeline)
{
	pipeline.tracking.trackedMarkers.clear();
	pipeline.tracking.trackedTargets.clear();
	pipeline.tracking.dormantTargets.clear();
	pipeline.tracking.orphanedIMUs.clear();
}

static TrackedTargetRecord& recordTracking(std::shared_ptr<FrameRecord> &frame, const TargetCalibration3D &target)
{
	auto tgtRec = std::find_if(frame->tracking.targets.begin(), frame->tracking.targets.end(),
		[&](auto &t){ return t.id == target.id; });
	if (tgtRec == frame->tracking.targets.end())
	{ // Enter into frame record
		frame->tracking.targets.push_back({ target.id });
		return frame->tracking.targets.back();
	}
	return *tgtRec;
}

static void recordTrackingResults(std::shared_ptr<FrameRecord> &frame, TrackedTargetRecord &target, const TrackedTarget &tracker, PipelineState &pipeline)
{
	// Record target tracking results
	target.id = tracker.target.calib->id;
	target.tracked = true;
	target.posePredicted = tracker.pose.predicted;
	target.poseExtrapolated = tracker.pose.extrapolated;
	target.poseInertialIntegrated = tracker.pose.inertialIntegrated;
	target.poseInertialFused = tracker.pose.inertialFused;
	target.poseInertialFiltered = tracker.pose.inertialFiltered;
	target.poseObserved = tracker.pose.observed;
	target.poseFiltered = tracker.pose.filtered;
	target.covPredicted = tracker.pose.covPredicted;
	target.covObserved = tracker.pose.covObserved;
	target.covFiltered = tracker.pose.covFiltered;
	target.error = tracker.target.match2D.error;
	if (pipeline.keepInternalData)
	{
		target.match2D = tracker.target.match2D;
	}
	updateVisibleMarkers(target.visibleMarkers, tracker.target.match2D);

	// Consider recording target in optimisation database
	auto &params = pipeline.params.cont.targetObs;
	int sampleCount = 0, strongCameras = 0;
	for (auto &camPts : tracker.target.match2D.points2D)
	{
		if (camPts.size() >= params.minCameraSamples)
			sampleCount += camPts.size();
		if (camPts.size() >= params.minStrongCameraSamples)
			strongCameras++;
	}
	if (strongCameras >= params.minStrongCameras && sampleCount >= params.minTotalSamples)
	{
		auto db_lock = pipeline.obsDatabase.contextualLock();
		auto targetIt = std::find_if(db_lock->targets.begin(), db_lock->targets.end(), [&](auto &tgt){ return tgt.targetID == target.id; });
		if (targetIt == db_lock->targets.end())
		{
			db_lock->targets.push_back({ target.id });
			targetIt = std::prev(db_lock->targets.end());
			targetIt->markers.reserve(tracker.target.calib->markers.size());
			for (auto &marker : tracker.target.calib->markers)
			{
				targetIt->markerMap.emplace((int)targetIt->markers.size(), (int)targetIt->markers.size());
				targetIt->markers.push_back(marker.pos);
			}
		}
		if (!targetIt->frames.empty() && targetIt->frames.back().frame >= frame->num)
			return;
		targetIt->frames.push_back({});
		auto &tgtFrame = targetIt->frames.back();
		tgtFrame.error = tracker.target.match2D.error.mean;
		tgtFrame.pose = tracker.target.match2D.pose;
		tgtFrame.frame = frame->num;
		tgtFrame.samples.reserve(sampleCount);
		for (int c = 0; c < tracker.target.match2D.points2D.size(); c++)
		{
			if (tracker.target.match2D.points2D[c].size() < params.minCameraSamples)
			{ // TODO: Consider adding those where all points were a good, clear match (pass TrackedTarget in here?)
				continue;
			}
			// Easily add all
			for (auto &pt : tracker.target.match2D.points2D[c])
				tgtFrame.samples.emplace_back(pt.first, c, frame->cameras[c].rawPoints2D[pt.second]);
			targetIt->totalSamples += tracker.target.match2D.points2D[c].size();
		}

		/* ScopedLogCategory scopedLogCategory(LOptimisation);
		ScopedLogLevel scopedLogLevel(LInfo);
		LOGCL("Added frame to optimisation database, new error:");
		updateReprojectionErrors(*db_lock, pipeline.getCalibs()); */
	}
}

static void recordTrackingFailure(std::shared_ptr<FrameRecord> &frame, TrackedTargetRecord &target, const TrackedTarget &tracker, bool loss = true)
{
	// Record target tracking failure
	target.id = tracker.target.calib->id;
	target.tracked = false;
	target.posePredicted = tracker.pose.predicted;
	target.poseExtrapolated = tracker.pose.extrapolated;
	target.poseInertialIntegrated = tracker.pose.inertialIntegrated;
	target.poseInertialFused = tracker.pose.inertialFused;
	target.poseInertialFiltered = tracker.pose.inertialFiltered;
	target.poseObserved.setIdentity();
	target.poseFiltered = tracker.pose.filtered;
	target.covPredicted = tracker.pose.covPredicted;
	target.covObserved.setZero();
	target.covFiltered = tracker.pose.covFiltered;
	target.error = { 0, 0, 0, 0};
	// TODO: Properly record as loss
	if (loss)
		frame->tracking.trackingLosses++;
}

static bool retroactivelyTrackFrame(PipelineState &pipeline, TrackedTarget &tracker, std::shared_ptr<FrameRecord> &frame)
{
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

	// Enter into frame record
	TrackedTargetRecord &target = recordTracking(frame, *tracker.target.calib);

	if (tracker.inertial)
		integrateIMU(tracker.state, tracker.inertial, tracker.pose, frame->time, pipeline.params.track);

	// Track just like in a normal frame
	if (!trackTarget(tracker.state, tracker.target, tracker.pose,
		calibs, points2D, properties, relevantPoints2D,
		frame->time, frame->num, pipeline.cameras.size(), pipeline.params.track))
	{
		LOG(LDetection2D, LInfo, "    Detection - Frame %d: Failed to find continuation of target with %d observations and %.3fpx mean error!\n",
			frame->num, tracker.target.match2D.error.samples, tracker.target.match2D.error.mean*PixelFactor);
		// Don't record as real loss since this is retroactive
		recordTrackingFailure(frame, target, tracker);
		return false;
	}
	LOG(LDetection2D, LDebug, "    Detection - Frame %d: Pixel Error after 2D target track: %fpx mean over %d points\n",
		frame->num, tracker.target.match2D.error.mean*PixelFactor, tracker.target.match2D.error.samples);
	if (tracker.inertial)
		postCorrectIMU(tracker.state, tracker.inertial, tracker.pose, frame->time, pipeline.params.track);
	recordTrackingResults(frame, target, tracker, pipeline);
	return true;
}

static bool detectTarget(std::stop_token stopToken, PipelineState &pipeline, std::shared_ptr<FrameRecord> &frame,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	int focus, TrackerTarget &tracker)
{
	auto &params = pipeline.params;
	frame->tracking.searches2D++;

	tracker.data.init(pipeline.cameras.size());

	// Detect target first in this cameras 2D points, and then match with others
	tracker.match2D = detectTarget2D(stopToken, *tracker.calib, calibs, points2D, properties, relevantPoints2D,
		focus, pipeline.cameras.size(), params.detect, params.track, tracker.data);
	if (tracker.match2D.error.samples < params.detect.minObservations.total)
		return false;
	frame->tracking.detections2D++;

	LOG(LDetection2D, LInfo, "    Detected tracked target in frame %d (now %d) with %d 2D points and %fpx mean error!\n",
		frame->num, (int)pipeline.frameNum.load(), tracker.match2D.error.samples, tracker.match2D.error.mean*PixelFactor);
	return true;
}

static bool detectTargetAsync(std::stop_token stopToken, PipelineState &pipeline, std::shared_ptr<FrameRecord> &frame,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<int>> &detectionPoints2D,
	int focus, DormantTarget &&dormant)
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
		if (!detectTarget(stopToken, pipeline, frame, calibs, points2D, properties, relevantPoints2D, focus, dormant.target))
			return false;
	}
	if (stopToken.stop_requested())
		return false;

	TrackedTarget tracker(dormant.target.calib, dormant.target.match2D.pose, frame->time, frame->num, pipeline.params.track);
	tracker.target = std::move(dormant.target);
	tracker.inertial = std::move(dormant.inertial);

	// Then continue on tracking to catch up with the latest realtime frame
	// TODO: Skip some frames to catch up faster, should handle some skips as long as prediction is accurate
	// But may also want intermediate frames tracked for recording purposes, so do those afterwards anyway

	TimePoint_t lastTimestamp = frame->time;
	std::size_t frameIndex = frame->num;

	{
		auto frames = pipeline.record.frames.getView<false>();
		if (frames.size() < frameIndex)
		{ // FrameRecords were cleared while starting detection, abort
			return false;
		}
		auto frameRecIt = frames.pos(frameIndex+1);
		for (; frameRecIt < frames.end(); frameRecIt++)
		{
			assert(frameRecIt.accessible());
			if (!*frameRecIt) continue;
			auto &frameRec = *frameRecIt->get();
			if (!frameRec.finishedProcessing)
				break;
			for (auto &track : frameRec.tracking.targets)
			{
				if (track.id != tracker.target.calib->id) continue;
				LOG(LDetection2D, LInfo, "    Detection - Frame %d: Caught up to a frame already tracked!\n", (int)frameRecIt.index()-1);
				return true; // Already tracked, maybe detected by 3D triangulation detection
			}
			if (!retroactivelyTrackFrame(pipeline, tracker, *frameRecIt))
			{
				frameRec.tracking.trackingLosses++;
				return false;
			}
			frameRec.tracking.trackingCatchups++;
			lastTimestamp = frameRec.time;
			if (stopToken.stop_requested())
				return false;
		}
		// Release view when leaving scope
		frameIndex = frameRecIt.index();
	}

	std::unique_lock pipeline_lock(pipeline.pipelineLock);
	if (stopToken.stop_requested())
		return false;

	{ // Finish the rest of the frames that were processed while waiting for the lock
		auto frames = pipeline.record.frames.getView<false>();
		if (frames.size() < frameIndex)
		{ // FrameRecords were cleared while waiting for the lock, abort
			return false;
		}
		auto frameRecIt = frames.pos(frameIndex);
		for (; frameRecIt < frames.end(); frameRecIt++)
		{
			assert(frameRecIt.accessible());
			if (!*frameRecIt) continue;
			auto &frameRec = *frameRecIt->get();
			if (!frameRec.finishedProcessing)
				break;
			for (auto &track : frameRec.tracking.targets)
			{
				if (track.id != tracker.target.calib->id) continue;
				LOG(LDetection2D, LInfo, "    Detection - Frame %d: Caught up to a frame already tracked!\n", (int)frameRecIt.index()-1);
				return true; // Already tracked, maybe detected by 3D triangulation detection
			}
			if (!retroactivelyTrackFrame(pipeline, tracker, *frameRecIt))
			{
				frameRec.tracking.trackingLosses++;
				return false;
			}
			frameRec.tracking.trackingCatchups++;
			lastTimestamp = frameRec.time;
			if (stopToken.stop_requested())
				return false;
		}
		// Release view when leaving scope
		frameIndex = frameRecIt.index();
	}

	LOG(LDetection2D, LInfo, "    Detection - Frame %d: Caught up to most recent processed frame!\n", (int)frameIndex-1);

	// Finally, no more frames to catch up on, register as new tracked target
	int erased = std::erase_if(pipeline.tracking.dormantTargets, [&](const auto &d){ return d.target.calib == tracker.target.calib; });
	assert(erased == 1);
	pipeline.tracking.trackedTargets.push_back(std::move(tracker));
	return true;
}

void retroactivelySimulateFilter(PipelineState &pipeline, std::size_t frameStart, std::size_t frameEnd)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);

	std::vector<CameraCalib> calibs(pipeline.cameras.size());
	std::vector<std::vector<Eigen::Vector2f> const *> points2D(pipeline.cameras.size());
	for (int c = 0; c < pipeline.cameras.size(); c++)
		calibs[c]= pipeline.cameras[c]->calib;

	auto frames = pipeline.record.frames.getView<false>();
	frameStart = std::max<unsigned int>(frames.beginIndex(), frameStart-3);
	frameEnd = std::min<unsigned int>(frames.endIndex(), frameEnd);
	auto frameIt = frames.pos(frameStart);
	while ((!*frameIt || !frameIt->get()->finishedProcessing) && frameIt.index() < frameEnd) frameIt++;
	for (; frameIt.index() < frameEnd; frameIt++)
	{
		if (!*frameIt || !frameIt->get()->finishedProcessing) continue;
		FrameRecord &frame = *frameIt->get();
		for (int c = 0; c < pipeline.cameras.size(); c++)
			points2D[c] = &frame.cameras[calibs[c].index].points2D;
		for (auto &target : frame.tracking.targets)
		{
			auto &targets = pipeline.tracking.trackedTargets;
			auto targetIt = std::find_if(targets.begin(), targets.end(),
				[&](auto &e){ return e.target.calib->id == target.id; });
			if (targetIt == targets.end()) continue; // Not interested in lost targets anyway
			if (targetIt->state.lastObsFrame > frame.num)
			{ // Initialise filter as good as possible
				targetIt->state = TrackerState(target.poseFiltered, frame.time, pipeline.params.track);
				targetIt->state.lastObsFrame = frame.num;
				targetIt->state.lastObservation = frame.time;
				targetIt->state.lastIMUSample = -1;
				continue;
			}
			if (targetIt->inertial)
				integrateIMU(targetIt->state, targetIt->inertial, targetIt->pose, frame.time, pipeline.params.track);
			if (simulateTrackTarget(targetIt->state, targetIt->target, targetIt->pose,
				calibs, points2D, target, frame.time, frame.num, pipeline.params.track))
			{
				recordTrackingResults(*frameIt, target, *targetIt, pipeline);
				if (targetIt->inertial)
					postCorrectIMU(targetIt->state, targetIt->inertial, targetIt->pose, frame.time, pipeline.params.track);
			}
			else
				recordTrackingFailure(*frameIt, target, *targetIt, false);
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

	{ // 6-DOF Target Tracking

		trk0 = pclock::now();

		auto tracker = track.trackedTargets.begin();
		while (tracker != track.trackedTargets.end())
		{
			const TargetCalibration3D &target = *tracker->target.calib;
			LOG(LTracking, LDebug, "Tracking target %d (name %s) with %d markers!\n", target.id, target.label.c_str(), (int)target.markers.size());

			// Enter into frame record
			TrackedTargetRecord &targetRecord = recordTracking(frame, *tracker->target.calib);

			if (tracker->inertial)
				integrateIMU(tracker->state, tracker->inertial, tracker->pose, frame->time, pipeline.params.track);

			if (trackTarget(tracker->state, tracker->target, tracker->pose,
				calibs, points2D, properties, relevantPoints2D,
				frame->time, frame->num, camCount, pipeline.params.track))
			{
				LOG(LTracking, LDebug, "    Found continuation of target %d (name %s) with %d observations and %.3fpx mean error!\n",
					target.id, target.label.c_str(), tracker->target.match2D.error.samples, tracker->target.match2D.error.mean*PixelFactor);

				if (tracker->inertial)
					postCorrectIMU(tracker->state, tracker->inertial, tracker->pose, frame->time, pipeline.params.track);

				// Occupy all 2D points of tracked target
				occupyTargetMatches(tracker->target.match2D);
				recordTrackingResults(frame, targetRecord, *tracker, pipeline);

				/* if (targetMatch2D.error.mean*PixelFactor > 0.5 && IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				} */
			}
			else if (dtMS(tracker->state.lastObservation, frame->time) > pipeline.params.track.lostTargetCoastMS)
			{
				LOG(LTracking, LDarn, "Failed to find continuation of target %d (name %s), got %d observations and %.3fpx mean error!\n",
					target.id, target.label.c_str(), tracker->target.match2D.error.samples, tracker->target.match2D.error.mean*PixelFactor);
				recordTrackingFailure(frame, targetRecord, *tracker);

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
				LOG(LTracking, LDarn, "Failed to find continuation of target %d (name %s), got %d observations and %.3fpx mean error!\n",
					target.id, target.label.c_str(), tracker->target.match2D.error.samples, tracker->target.match2D.error.mean*PixelFactor);
				recordTrackingFailure(frame, targetRecord, *tracker, true);

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
				Eigen::Isometry3f gtPose = pipeline.simulation.contextualRLock()->getGTPose(target);
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

	std::vector<std::vector<int>> clusters3D;

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
		clusters3D = dbscan<3,float, int>(track.points3D, clustering.tri.maxDistance, clustering.tri.minPoints);
		// May also want to consider doing it before resolving conflicts, incase a ray actually passes through two clusters
		// - in which case, that ray would still be uncertain, I guess
		LOG(LTracking, LDebug, "Clustered 3D points in %d groups in %d points!", (int)clusters3D.size(), (int)track.points3D.size());
		for (auto &cluster : clusters3D)
			LOG(LTracking, LDebug, "    Cluster has %d 3D points!", (int)cluster.size());

		// Fill triangulated point list to pass to functions
		triIndices.resize(track.triangulations3D.size());
		std::iota(triIndices.begin(), triIndices.end(), 0);
	}

	det0 = pclock::now();

	if (trackTargets && detect.enable3D && pipeline.tracking.triangulations3D.size() >= detect.tri.minPointCount)
	{ // Target re-detection

		auto dormantIt = pipeline.tracking.dormantTargets.begin();
		while (dormantIt != pipeline.tracking.dormantTargets.end())
		{
			DormantTarget &dormant = *dormantIt;
			const TargetCalibration3D &target = *dormant.target.calib;
			LOG(LTracking, LDebug, "Trying to detect target %d (name %s) with %d markers in 3D point cloud!\n",
				target.id, target.label.c_str(), (int)target.markers.size());

			// Detect target in remaining 3D point cloud
			TargetCandidate3D candidate = detectTarget3D(target,
				pipeline.tracking.triangulations3D, triIndices,
				detect.tri.sigmaError, detect.tri.poseSigmaError, detect.tri.quickAssignTargetMatches);

			bool acceptCandidate = candidate.points.size() >= detect.tri.minPointCount && candidate.MSE < detect.tri.maxErrorRMSE*detect.tri.maxErrorRMSE;
			if (acceptCandidate)
			{
				// Use pose candidate to track target with 2D points
				CovarianceMatrix covariance = pipeline.params.track.filter.getSyntheticCovariance<float>() * pipeline.params.track.filter.detectSigma;
				dormant.target.match2D = trackTarget2D(target, candidate.pose, covariance,
					calibs, camCount, points2D, properties, relevantPoints2D, pipeline.params.track, dormant.target.data);

				acceptCandidate = dormant.target.match2D.error.samples >= pipeline.params.track.quality.minTotalObs
								&& dormant.target.match2D.error.mean < pipeline.params.track.quality.maxTotalError;
				if (acceptCandidate)
				{ // Register as tracked target
					{ // Make sure no async detection of this target is ongoing
						if (pipeline.tracking.asyncDetection && pipeline.tracking.asyncDetectTargetID == target.id)
							pipeline.tracking.asyncDetectionStop.request_stop();
					}
					TrackedTarget trackedTarget(std::move(dormant), dormant.target.match2D.pose, frame->time, frame->num, pipeline.params.track);
					occupyTargetMatches(trackedTarget.target.match2D);
					TrackedTargetRecord &targetRecord = recordTracking(frame, target);
					recordTrackingResults(frame, targetRecord, trackedTarget, pipeline);
					frame->tracking.detections3D++;

					LOG(LTracking, LInfo, "    Added dormant tracked target %s back with %d points and %fmm RMSE"
						", now with %d 2D points and %fpx mean error!\n",
						target.label.c_str(), (int)candidate.points.size(), std::sqrt(candidate.MSE),
						trackedTarget.target.match2D.error.samples, trackedTarget.target.match2D.error.mean*PixelFactor);

					pipeline.tracking.trackedTargets.push_back(std::move(trackedTarget));
				}
				else
				{
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
				Eigen::Isometry3f gtPose = pipeline.simulation.contextualRLock()->getGTPose(target);
				std::pair<double,double> GTError = calculatePoseError(gtPose, candidate.pose);
				LOG(LTracking, LDebug, "    Best candidate has GT Error %fmm, %fdeg!\n", GTError.first, GTError.second);
			}

			if (acceptCandidate)
				dormantIt = pipeline.tracking.dormantTargets.erase(dormantIt);
			else
				dormantIt++;
		}
	}

	det1 = pclock::now();

	std::vector<std::vector<std::vector<int>>> clusters(calibs.size());
	int clusterSelect = 0, clusterCamera = -1;
	for (int c = 0; c < calibs.size(); c++)
	{ // 2D Point clustering
		clusters[c] = dbscanSubset<2,float,int>(*points2D[c], remainingPoints2D[c], clustering.blobs.maxDistance, clustering.blobs.minPoints);
		if (!clusters[c].empty() && clusterSelect < clusters[c].front().size())
		{
			clusterSelect = clusters[c].front().size();
			clusterCamera = c;
		}

		LOG(LTracking, LDebug, "Camera %d: Clustered %d/%d relevant points into %d groups:", calibs[c].index, (int)remainingPoints2D[c].size(), (int)points2D[c]->size(), (int)clusters[c].size());
		for (auto &cluster : clusters[c])
			LOG(LTracking, LDebug, "    Cluster has %d 2D points!", (int)cluster.size());
	}

	if (trackTargets && (detect.enable2DSync || (detect.enable2DAsync && !pipeline.tracking.asyncDetection))
		&& clusterSelect > detect.minObservations.focus && !pipeline.tracking.dormantTargets.empty())
	{ // 2D Target Detection

		// Select cluster to detect in
		auto &cluster = clusters[clusterCamera].front();
		assert(cluster.size() > detect.minObservations.focus);

		// Select target to detect
		DormantTarget &dormant = pipeline.tracking.dormantTargets.front();
		const TargetCalibration3D &target = *dormant.target.calib;
		LOG(LDetection2D, LDarn, "Trying target %d (name %s) with %d markers!\n",
			target.id, target.label.c_str(), (int)target.markers.size());

		// Move to back in queue
		pipeline.tracking.dormantTargets.splice(pipeline.tracking.dormantTargets.end(), pipeline.tracking.dormantTargets, pipeline.tracking.dormantTargets.begin());

		if (detect.enable2DSync)
		{
			std::vector<std::vector<int> const *> detectionPoints2D = relevantPoints2D;
			detectionPoints2D[clusterCamera] = &cluster;

			pipeline.tracking.syncDetectionStop = {};
			if (detectTarget(pipeline.tracking.syncDetectionStop.get_token(), pipeline, frame, calibs,
				points2D, properties, detectionPoints2D, clusterCamera, dormant.target))
			{
				TrackedTarget tracker(&target, dormant.target.match2D.pose, frame->time, frame->num, pipeline.params.track);
				tracker.target = std::move(dormant.target);
				tracker.inertial = std::move(dormant.inertial);
				occupyTargetMatches(tracker.target.match2D);
				TrackedTargetRecord &targetRecord = recordTracking(frame, target);
				recordTrackingResults(frame, targetRecord, tracker, pipeline);
				pipeline.tracking.trackedTargets.push_back(std::move(tracker));
				pipeline.tracking.dormantTargets.pop_back();
			}
		}
		else if (detect.enable2DAsync)
		{
			std::vector<std::vector<int>> detectionPoints2D = remainingPoints2D;
			detectionPoints2D[clusterCamera] = cluster;

			pipeline.tracking.asyncDetection = true;
			pipeline.tracking.asyncDetectionStop = {};
			pipeline.tracking.asyncDetectTargetID = target.id;
			threadPool.push([&pipeline](int, std::stop_token stopToken, std::shared_ptr<FrameRecord> frameRec,
				std::vector<CameraCalib> calibs, std::vector<std::vector<int>> detectionPoints2D,
				DormantTarget dormant, int focus)
			{ // Working with copy of dormant tracker
				detectTargetAsync(stopToken, pipeline, frameRec, calibs, detectionPoints2D, focus, std::move(dormant));
				pipeline.tracking.asyncDetection = false;
			}, pipeline.tracking.asyncDetectionStop.get_token(), frame, calibs, detectionPoints2D, dormant, clusterCamera);
		}
	}

	det2 = pclock::now();

	tpt0 = pclock::now();

	{ // Single Marker Tracking

		auto tracker = pipeline.tracking.trackedMarkers.begin();
		while (tracker != pipeline.tracking.trackedMarkers.end())
		{
			int matchedPoint = trackMarker(tracker->state, tracker->marker, tracker->pose,
					pipeline.tracking.points3D, triIndices, frame->time, 3);
			if (matchedPoint >= 0)
			{
				triIndices.erase(std::find(triIndices.begin(), triIndices.end(), matchedPoint));
				tracker++;
			}
			else
			{
				LOG(LTracking, LDarn, "Failed to find continuation of tracked point!\n");
				tracker = pipeline.tracking.trackedMarkers.erase(tracker);
			}
		}
	}

	/*{ // Add single markers to track
		// TODO: Limit by size? Or stability over multiple frames
		for (int p = 0; p < pipeline.tracking.points3D.size(); p++)
		{
			if (!occupied3D[p])
			{
				Eigen::Vector3f pos = pipeline.tracking.points3D[p];
				// Add tracked marker
				pipeline.tracking.trackedMarkers.push_back({});
				auto &marker = pipeline.tracking.trackedMarkers.back();
				marker.state.setZero();
				marker.state.pos() = pos;
				marker.errorScaleT = 100000; // To allow for quick adaption of correct velocity and acceleration
				marker.measurements = 1;
				marker.time = frame->time;
				LOG(LTracking, LDebug, "Added new tracked point!\n");
				occupied3D[p] = true;
			}
		}
	}*/

	// TODO: Properly integrate
	frame->tracking.triangulations = track.points3D;

	tpt1 = pclock::now();

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

bool ConnectIMU(PipelineState &pipeline, std::shared_ptr<IMU> &imu)
{
	if (imu->tracker.id == 0) return false;
	std::unique_lock pipeline_lock(pipeline.pipelineLock);
	for (auto &tracker : pipeline.tracking.trackedTargets)
	{
		if (tracker.target.calib->id != imu->tracker.id) continue;
		tracker.inertial = TrackerInertial(imu); // new shared_ptr
		std::erase_if(pipeline.tracking.orphanedIMUs, [&](const auto &t){ return t.inertial.imu == imu; });
		tracker.state.lastIMUSample = -1;
		return true;
	}
	for (auto &tracker : pipeline.tracking.dormantTargets)
	{
		if (tracker.target.calib->id != imu->tracker.id) continue;
		tracker.inertial = TrackerInertial(imu); // new shared_ptr
		std::erase_if(pipeline.tracking.orphanedIMUs, [&](const auto &t){ return t.inertial.imu == imu; });
		return true;
	}
	return false;
}

bool ConnectIMU(PipelineState &pipeline, std::shared_ptr<IMU> &imu, TrackedMarker &tracker)
{
	if (!imu->tracker.isMarker()) return false;

	std::unique_lock pipeline_lock(pipeline.pipelineLock);
	// TODO: Properly implement automatic association of IMUs to markers 

	// Check marker against existing association
	if (tracker.inertial.imu)
		return false; // Don't handle here
	if (std::abs(tracker.marker.size/imu->tracker.size - 1) < 0.1f)
		return false; // Don't handle here

	tracker.inertial = TrackerInertial(imu); // new shared_ptr
	tracker.state.lastIMUSample = -1;
	std::erase_if(pipeline.tracking.orphanedIMUs, [&](const auto &t){ return t.inertial.imu == imu; });
	return true;
}

void DisassociateIMU(PipelineState &pipeline, std::shared_ptr<IMU> &imu)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);
	
	// Disconnect
	for (auto &tracker : pipeline.tracking.trackedTargets)
		if (tracker.inertial.imu == imu) tracker.inertial = {};
	for (auto &tracker : pipeline.tracking.dormantTargets)
		if (tracker.inertial.imu == imu) tracker.inertial = {};
	for (auto &tracker : pipeline.tracking.trackedMarkers)
		if (tracker.inertial.imu == imu) tracker.inertial = {};

	// Clear association
	imu->tracker = IMUTracker();

	// Ensure it's orphaned
	if (std::find_if(pipeline.tracking.orphanedIMUs.begin(), pipeline.tracking.orphanedIMUs.end(),
		[&](const auto &t){ return t.inertial.imu == imu; }) != pipeline.tracking.orphanedIMUs.end())
		return;
	pipeline.tracking.orphanedIMUs.emplace_back(imu, pipeline.params.track);
}

void DisassociateIMU(PipelineState &pipeline, int trackerID)
{
	std::unique_lock pipeline_lock(pipeline.pipelineLock);
	std::shared_ptr<IMU> imu = nullptr;
	// Disconnect
	for (auto &tracker : pipeline.tracking.trackedTargets)
	{
		if (tracker.target.calib->id != trackerID) continue;
		imu = std::move(tracker.inertial.imu);
		tracker.inertial = {};
		tracker.state.lastIMUSample = -1;
		break;
	}
	for (auto &tracker : pipeline.tracking.dormantTargets)
	{
		if (tracker.target.calib->id != trackerID) continue;
		imu = std::move(tracker.inertial.imu);
		tracker.inertial = {};
		break;
	}
	if (imu)
	{
		// Clear association
		imu->tracker = IMUTracker();
		// Ensure it's orphaned
		pipeline.tracking.orphanedIMUs.emplace_back(std::move(imu), pipeline.params.track);
	}

}