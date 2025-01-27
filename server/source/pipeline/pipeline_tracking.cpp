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

#include "dbscan/dbscan.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <numeric>

// ----------------------------------------------------------------------------
// General 3D- and 2D Marker Tracking
// ----------------------------------------------------------------------------

void InitTrackingPipeline(PipelineState &pipeline)
{
	pipeline.tracking.lastFrameTime = sclock::now(); // Timestep should not be used for first measurement, but still
	pipeline.tracking.trackedMarkers.clear();
	pipeline.tracking.trackedTargets.clear();
	pipeline.tracking.lostTargets.clear();
	pipeline.tracking.unusedTargets.clear();
	for (auto &targetTemplate : pipeline.tracking.targetTemplates3D)
		pipeline.tracking.unusedTargets.push_back(&targetTemplate);
}

static void recordTrackingResults(PipelineState &pipeline, std::shared_ptr<FrameRecord> &frame, const TrackedTargetFiltered &tracker)
{
	// Record target tracking results
	frame->tracking.targets.emplace_back();
	TrackedTargetRecord &target = frame->tracking.targets.back();
	target.id = tracker.match2D.targetTemplate->id;
	target.pose = tracker.match2D.pose;
	updateVisibleMarkers(target.visibleMarkers, tracker.match2D);
	target.prediction = tracker.filter.predPose;
	target.error2DAvg = tracker.match2D.error.mean;
	target.error2DMax = tracker.match2D.error.max;
	target.sampleCnt = tracker.match2D.pointCount;
	target.cameraCnt = 0;
	for (auto &camPts : tracker.match2D.points2D)
		if (!camPts.empty())
			target.cameraCnt++;

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
		auto targetIt = std::find_if(db_lock->targets.begin(), db_lock->targets.end(), [&](auto &tgt){ return tgt.targetID == target.id; });
		if (targetIt == db_lock->targets.end())
		{
			db_lock->targets.push_back({ target.id });
			targetIt = std::prev(db_lock->targets.end());
			targetIt->markers.reserve(tracker.match2D.targetTemplate->markers.size());
			for (auto &marker : tracker.match2D.targetTemplate->markers)
			{
				targetIt->markerMap.emplace(targetIt->markers.size(), targetIt->markers.size());
				targetIt->markers.push_back(marker.pos);
			}
		}
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

static bool retroactivelyTrackFrame(PipelineState &pipeline, TrackedTargetFiltered &tracker, std::shared_ptr<FrameRecord> &frame, float timestep)
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

	// Track just like in a normal frame
	if (!trackTarget(tracker, calibs, points2D, properties, relevantPoints2D,
		timestep, pipeline.cameras.size(), pipeline.params.track))
	{
		LOG(LDetection2D, LInfo, "    Detection - Frame %d: Failed to find continuation of target with %d observations and %.3fpx mean error!\n",
			frame->num, tracker.match2D.pointCount, tracker.match2D.error.mean*PixelFactor);
		return false;
	}
	LOG(LDetection2D, LDebug, "    Detection - Frame %d: Pixel Error after 2D target track: %fpx mean over %d points\n",
		frame->num, tracker.match2D.error.mean*PixelFactor, tracker.match2D.pointCount);

	// Record target
	recordTrackingResults(pipeline, frame, tracker);
	return true;
}

static bool detectTarget(std::stop_token stopToken, PipelineState &pipeline, std::shared_ptr<FrameRecord> &frame,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	int focus, const TargetTemplate3D *target,
	TrackedTargetFiltered &tracker)
{
	auto &params = pipeline.params;
	frame->tracking.searches2D++;

	// Detect target first in this cameras 2D points, and then match with others
	TargetTracking2DData tracking2DData(pipeline.cameras.size());
	TargetMatch2D targetMatch2D = detectTarget2D(stopToken, *target, calibs, points2D, properties, relevantPoints2D,
		focus, pipeline.cameras.size(), params.detect, params.track, tracking2DData);
	if (targetMatch2D.pointCount < params.detect.minObservations.total)
		return false;
	frame->tracking.detections2D++;

	LOG(LDetection2D, LInfo, "    Detected tracked target in frame %d (now %d) with %d 2D points and %fpx mean error!\n",
		frame->num, (int)pipeline.frameNum.load(), targetMatch2D.pointCount, targetMatch2D.error.mean*PixelFactor);

	tracker = {};
	tracker.tracking2DData = std::move(tracking2DData); // Reuse allocations
	tracker.match2D = std::move(targetMatch2D);
	tracker.target = target;
	tracker.filter.init(tracker.match2D.pose);
	return true;
}

static bool detectTargetAsync(std::stop_token stopToken, PipelineState &pipeline, std::shared_ptr<FrameRecord> &frame,
	const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<int>> &detectionPoints2D,
	int focus, const TargetTemplate3D *target)
{
	TrackedTargetFiltered tracker;
	// TODO: Add stopToken so a successfull detection3D can abort even the brute forcing, or the later catchup parts

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
		if (!detectTarget(stopToken, pipeline, frame, calibs, points2D, properties, relevantPoints2D, focus, target, tracker))
			return false;
	}
	if (stopToken.stop_requested())
		return false;

	// Then continue on tracking to catch up with the latest realtime frame
	// TODO: Skip some frames to catch up faster, should handle some skips as long as prediction is accurate
	// But may also want intermediate frames tracked for recording purposes, so do those afterwards anyway

	TimePoint_t lastTimestamp = frame->time;
	std::size_t frameIndex = frame->num;

	{
		auto frames = pipeline.frameRecords.getView<false>();
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
				if (track.id != target->id) continue;
				LOG(LDetection2D, LInfo, "    Detection - Frame %d: Caught up to a frame already tracked!\n", (int)frameRecIt.index()-1);
				return true; // Already tracked, maybe detected by 3D triangulation detection
			}
			float timestep = dt(lastTimestamp, frameRec.time);
			if (!retroactivelyTrackFrame(pipeline, tracker, *frameRecIt, timestep))
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
		auto frames = pipeline.frameRecords.getView<false>();
		if (frames.size() < frameIndex)
		{ // FrameRecords were cleared while waiting for the lock, abort
			return false;
		}
		auto frameRecIt = frames.pos(frameIndex);
		for (; frameRecIt < frames.end(); frameRecIt++)
		{
			assert(frameRecIt.accessible());
			auto &frameRec = *frameRecIt->get();
			if (!frameRec.finishedProcessing)
				break;
			for (auto &track : frameRec.tracking.targets)
			{
				if (track.id != target->id) continue;
				LOG(LDetection2D, LInfo, "    Detection - Frame %d: Caught up to a frame already tracked!\n", (int)frameRecIt.index()-1);
				return true; // Already tracked, maybe detected by 3D triangulation detection
			}
			float timestep = dt(lastTimestamp, frameRec.time);
			if (!retroactivelyTrackFrame(pipeline, tracker, *frameRecIt, timestep))
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
		tracker.lastTrackedFrame = frameIndex;
	}

	LOG(LDetection2D, LInfo, "    Detection - Frame %d: Caught up to most recent processed frame!\n", (int)frameIndex-1);

	// Finally, no more frames to catch up on, register as new tracked target
	int id = tracker.target->id;
	auto lostIt = std::find_if(pipeline.tracking.lostTargets.begin(), pipeline.tracking.lostTargets.end(),
		[&](auto &e){ return e.first->id == id; });
	bool lost = lostIt != pipeline.tracking.lostTargets.end();
	if (lost) pipeline.tracking.lostTargets.erase(lostIt);
	auto unusedIt = std::find_if(pipeline.tracking.unusedTargets.begin(), pipeline.tracking.unusedTargets.end(),
		[&](auto &e){ return e->id == id; });
	bool unused = unusedIt != pipeline.tracking.unusedTargets.end();
	if (unused) pipeline.tracking.unusedTargets.erase(unusedIt);
	if (lost || unused)
	{
		pipeline.tracking.trackedTargets.push_back(std::move(tracker));
		return true;
	}
	return false;
}

void UpdateTrackingPipeline(PipelineState &pipeline, std::vector<CameraPipeline*> &cameras, std::shared_ptr<FrameRecord> &frame)
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

	float timestep = dt(track.lastFrameTime, frame->time);
	track.lastFrameTime = frame->time;

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

	{ // 6-DOF Target Tracking

		trk0 = pclock::now();

		auto trackedTarget = track.trackedTargets.begin();
		while (trackedTarget != track.trackedTargets.end())
		{
			const TargetTemplate3D &target = *trackedTarget->target;
			LOG(LTracking, LDebug, "Tracking target %d (name %s) with %d markers!\n", target.id, target.label.c_str(), (int)target.markers.size());

			if (trackTarget(*trackedTarget, calibs, points2D, properties, relevantPoints2D,
				timestep, pipeline.cameras.size(), pipeline.params.track))
			{
				LOG(LTracking, LDebug, "    Found continuation of target %d (name %s) with %d observations and %.3fpx mean error!\n",
					target.id, target.label.c_str(), trackedTarget->match2D.pointCount, trackedTarget->match2D.error.mean*PixelFactor);

				// Occupy all 2D points of tracked target
				occupyTargetMatches(trackedTarget->match2D);
				recordTrackingResults(pipeline, frame, *trackedTarget);
				trackedTarget->lastTrackedFrame = frame->num;

				/* if (targetMatch2D.error.mean*PixelFactor > 0.5 && IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				} */
			}
			else if (frame->num - trackedTarget->lastTrackedFrame < 4)
			{
				LOG(LTracking, LDarn, "Failed to find continuation of target %d (name %s) after %d measurements, got with %d observations and %.3fpx mean error!\n",
					target.id, target.label.c_str(), trackedTarget->filter.measurements, trackedTarget->match2D.pointCount, trackedTarget->match2D.error.mean*PixelFactor);

				if (IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				}

				trackedTarget++;
				continue;
			}
			else
			{
				LOG(LTracking, LDarn, "Failed to find continuation of target %d (name %s) after %d measurements, got with %d observations and %.3fpx mean error!\n",
					target.id, target.label.c_str(), trackedTarget->filter.measurements, trackedTarget->match2D.pointCount, trackedTarget->match2D.error.mean*PixelFactor);

				if (IsDebugging())
				{
					pipeline.pipelineLock.unlock();
					Breakpoint();
					pipeline.pipelineLock.lock();
				}

				// Push to lost targets for re-detection
				// TODO: Wait another frame or two, target might have dropped out or temporarily been bad for the algorithm
				track.lostTargets.push_back({ trackedTarget->target, frame->num });
				trackedTarget = track.trackedTargets.erase(trackedTarget);
				frame->tracking.trackingLosses++;
				continue;
			}

			if (pipeline.isSimulationMode)
			{
				// Try to find ground truth pose of what it believes it is tracking (it may have detected wrong)
				Eigen::Isometry3f gtPose = pipeline.simulation.contextualRLock()->getGTPose(target);
				std::pair<double,double> GTErrorOpt = calculatePoseError(gtPose, trackedTarget->match2D.pose);
				LOG(LTracking, LDebug, "    GT Opt Tracking Error: %fmm, %fdeg\n", GTErrorOpt.first, GTErrorOpt.second);
				std::pair<double,double> GTError = calculatePoseError(gtPose, trackedTarget->filter.pose);
				LOG(LTracking, LDebug, "    GT Tracking Error: %fmm, %fdeg\n", GTError.first, GTError.second);
				if (GTError.first > 10 || GTError.second > 5)
					LOG(LTracking, LDarn, "=== WAY too high tracking error!!!\n");
			}

			trackedTarget++;
		}

		trk1 = pclock::now();
	}

	std::vector<std::vector<uint32_t>> clusters3D;

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
		clusters3D = dbscan<3,float>(track.points3D, clustering.tri.maxDistance, clustering.tri.minPoints);
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

	auto createTrackedTarget = [&pipeline](TargetTemplate3D const *target, TargetMatch2D &&targetMatch2D)
	{
		TrackedTargetFiltered tracker = {};
		tracker.target = target;
		tracker.filter.init(targetMatch2D.pose);
		tracker.match2D = std::move(targetMatch2D);
		return tracker;
	};
	auto expandTargetMatches2D = [&track, camCount, &calibs, &points2D](TargetTemplate3D const *target, TargetCandidate3D &candidate, const TargetOptimisationParameters &params)
	{
		// Setup candidate for 2D optimisation
		TargetMatch2D targetMatch2D;
		targetMatch2D.targetTemplate = target;
		targetMatch2D.pose = candidate.pose;
		targetMatch2D.pointCount = 0;
		targetMatch2D.points2D.resize(camCount);
		// Iterate over all 2D observations
		for (int p = 0; p < candidate.points.size(); p++)
		{
			int trIndex = candidate.points[p];
			int mkIndex = candidate.pointMap[trIndex];
			TriangulatedPoint &pt = track.triangulations3D[trIndex];
			// Handle each individual observation
			for (auto &calib : calibs)
			{
				int c = calib.index;
				if (pt.blobs[c] != InvalidBlob)
				{ // Register currently visible markers for each view
					targetMatch2D.points2D[c].push_back({ mkIndex, pt.blobs[c] });
					targetMatch2D.pointCount++;
				}
			}
		}
		// TODO: Add further 2D-only points using trackTarget2D
		// Optimise pose
		targetMatch2D.error = optimiseTargetPose<false>(calibs, points2D, targetMatch2D, params);
		return targetMatch2D;
	};

	if (detect.enable3D && pipeline.tracking.triangulations3D.size() >= detect.tri.minPointCount)
	{ // Target re-detection

		auto targetTemplate = pipeline.tracking.lostTargets.begin();
		while (targetTemplate != pipeline.tracking.lostTargets.end())
		{
			const TargetTemplate3D &target = *targetTemplate->first;
			LOG(LTracking, LDebug, "Trying lost target %d (name %s) with %d markers from %d frames ago!\n",
				target.id, target.label.c_str(), (int)target.markers.size(), frame->num-targetTemplate->second);

			// Detect target in remaining 3D point cloud
			TargetCandidate3D candidate = detectTarget3D(target,
				pipeline.tracking.triangulations3D, triIndices,
				detect.tri.sigmaError, detect.tri.poseSigmaError, detect.tri.quickAssignTargetMatches);

			bool acceptCandidate = candidate.points.size() >= detect.tri.minPointCount && candidate.MSE < detect.tri.maxErrorRMSE*detect.tri.maxErrorRMSE;
			if (acceptCandidate)
			{
				auto targetMatch2D = expandTargetMatches2D(&target, candidate, detect.opt);
				auto trackedTarget = createTrackedTarget(&target, std::move(targetMatch2D));
				occupyTargetMatches(trackedTarget.match2D);
				{ // Make sure no async detection of this target is ongoing
					if (pipeline.tracking.asyncDetection && pipeline.tracking.asyncDetectTargetID == target.id)
						pipeline.tracking.asyncDetectionStop.request_stop();
				}
				recordTrackingResults(pipeline, frame, trackedTarget);
				trackedTarget.lastTrackedFrame = frame->num;
				frame->tracking.detections3D++;

				LOG(LTracking, LInfo, "    Added lost tracked target back with %d points and %fmm RMSE"
					", now with %d 2D points and %fpx mean error!\n",
					(int)candidate.points.size(), std::sqrt(candidate.MSE),
					trackedTarget.match2D.pointCount, trackedTarget.match2D.error.mean*PixelFactor);

				pipeline.tracking.trackedTargets.push_back(std::move(trackedTarget));

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
				targetTemplate = pipeline.tracking.lostTargets.erase(targetTemplate);
			else
				targetTemplate++;
		}
	}

	if (detect.enable3D && pipeline.tracking.triangulations3D.size() >= detect.tri.minPointCount)
	{ // Target detection

		auto targetTemplate = pipeline.tracking.unusedTargets.begin();
		while (targetTemplate != pipeline.tracking.unusedTargets.end())
		{
			const TargetTemplate3D &target = **targetTemplate;
			LOG(LTracking, LDebug, "Trying unused %d (name %s) with %d markers!\n", target.id, target.label.c_str(), (int)target.markers.size());

			// Detect target in remaining 3D point cloud
			TargetCandidate3D candidate = detectTarget3D(target,
				pipeline.tracking.triangulations3D, triIndices,
				detect.tri.sigmaError, detect.tri.poseSigmaError, detect.tri.quickAssignTargetMatches);

			bool acceptCandidate = candidate.points.size() >= detect.tri.minPointCount && candidate.MSE < detect.tri.maxErrorRMSE*detect.tri.maxErrorRMSE;
			if (acceptCandidate)
			{
				auto targetMatch2D = expandTargetMatches2D(&target, candidate, detect.opt);
				auto trackedTarget = createTrackedTarget(&target, std::move(targetMatch2D));
				occupyTargetMatches(trackedTarget.match2D);
				{ // Make sure no async detection of this target is ongoing
					if (pipeline.tracking.asyncDetection && pipeline.tracking.asyncDetectTargetID == target.id)
						pipeline.tracking.asyncDetectionStop.request_stop();
				}
				recordTrackingResults(pipeline, frame, trackedTarget);
				trackedTarget.lastTrackedFrame = frame->num;
				frame->tracking.detections3D++;

				LOG(LTracking, LInfo, "    Added new tracked target with %d points and %fmm RMSE"
					", now with %d 2D points and %fpx mean error!\n",
					(int)candidate.points.size(), std::sqrt(candidate.MSE),
					trackedTarget.match2D.pointCount, trackedTarget.match2D.error.mean*PixelFactor);

				pipeline.tracking.trackedTargets.push_back(std::move(trackedTarget));

			}
			else if (candidate.points.size() > 0)
			{
				LOG(LTracking, LDebug, "    No good candidate, best has %d points and %fmm RMSE!\n",
					(int)candidate.points.size(), std::sqrt(candidate.MSE));
			}

			if (pipeline.isSimulationMode)
			{
				std::vector<std::pair<int, Eigen::Vector3f>> gtTris;
				{ // Copy GT triangulations
					auto sim_lock = pipeline.simulation.contextualRLock();
					// Try to find ground truth pose of what it believes it is tracking (it may have detected wrong)
					Eigen::Isometry3f gtPose = sim_lock->getGTPose(target);
					std::pair<double,double> GTError = calculatePoseError(gtPose, candidate.pose);
					LOG(LTracking, LDebug, "    Best candidate has GT Error %fmm, %fdeg!\n", GTError.first, GTError.second);
					if (sim_lock->triangulatedPoints3D.frame == frame->num)
					{
						gtTris = sim_lock->triangulatedPoints3D.triangulation;
						LOG(LTracking, LTrace, "Simulation generated %d triangulatable points!\n", (int)gtTris.size());
					}
				}
				for (int p = 0; p < candidate.points.size(); p++)
				{
					float bestDist = std::numeric_limits<float>::max();
					int bestMarker = -1;
					float matchedDist = std::numeric_limits<float>::max();
					int triPt = candidate.points[p];
					int tgtPt = candidate.pointMap[triPt];
					Eigen::Vector3f tri = pipeline.tracking.points3D[triPt];
					Eigen::Vector3f tgt = target.markers[tgtPt].pos;
					for (int g = 0; g < gtTris.size(); g++)
					{
						auto gtTri = gtTris[g];
						Eigen::Vector3f diff = tri - gtTri.second;
						float dist = diff.squaredNorm();
						if (dist < bestDist)
						{
							bestDist = dist;
							bestMarker = gtTri.first; // source target marker (if generated by a target)
						}
						if (gtTri.first == tgtPt)
						{
							matchedDist = std::sqrt(dist);
						}
					}

					if (bestMarker == tgtPt)
					{
						LOG(LTracking, LDebug, "        Best candidates point %d (tri %d) was probably correctly matched to tgt point %d with distance of %fmm, match error %fmm\n",
							p, triPt, tgtPt, std::sqrt(bestDist)*1000, (tri-tgt).norm()*1000);
					}
					else
					{
						LOG(LTracking, LDebug, "        Best candidates point %d (tri %d) was incorrectly matched to tgt point %d which is actually %fmm away\n",
							p, triPt, tgtPt, matchedDist*1000);
						LOG(LTracking, LDebug, "        Likely should have been matched to tgt point %d with distance %fmm, match error %fmm\n",
							bestMarker, std::sqrt(bestDist)*1000, (tri-tgt).norm()*1000);
					}
				}
			}
			if (acceptCandidate)
				targetTemplate = pipeline.tracking.unusedTargets.erase(targetTemplate);
			else
				targetTemplate++;
		}
	}

	det1 = pclock::now();

	for (int c = 0; c < calibs.size(); c++)
	{ // 2D Target Detection per camera
		if (!detect.enable2DSync && !detect.enable2DAsync) break;
		if (remainingPoints2D[c].size() < detect.minObservations.focus) continue;

		// Alread have asynchronous detection running
		if (pipeline.tracking.asyncDetection) break;

		// 2D Point clustering
		auto clusters2D = dbscan<2,float>(*points2D[c], clustering.blobs.maxDistance, clustering.blobs.minPoints);
		// May also want to consider doing it before resolving conflicts, incase a ray actually passes through two clusters
		// - in which case, that ray would still be uncertain, I guess
		LOG(LTracking, LDebug, "Clustered for camera %d 2D points in %d groups in %d points!", calibs[c].index, (int)clusters2D.size(), (int)points2D[c]->size());
		for (auto &cluster : clusters2D)
			LOG(LTracking, LDebug, "    Cluster has %d 2D points!", (int)cluster.size());

		// Target re-detection
		auto lostTarget = pipeline.tracking.lostTargets.begin();
		while (lostTarget != pipeline.tracking.lostTargets.end() &&
			remainingPoints2D[c].size() >= detect.minObservations.focus)
		{
			const TargetTemplate3D &target = *lostTarget->first;
			if (target.markers.size() < detect.minObservations.focus)
			{
				lostTarget++;
				continue;
			}

			LOG(LDetection2D, LInfo, "Trying lost target %d (name %s) with %d markers from %d frames ago!\n",
				target.id, target.label.c_str(), (int)target.markers.size(), frame->num-lostTarget->second);

			if (detect.enable2DSync)
			{
				std::vector<std::vector<int> const *> detectionPoints2D = relevantPoints2D;
				// TODO: Check specific cluster that is likely
				//detectionPoints2D[c] = clusters2D.front();
				pipeline.tracking.syncDetectionStop = {};
				TrackedTargetFiltered trackedTarget;
				if (detectTarget(pipeline.tracking.syncDetectionStop.get_token(), pipeline, frame, calibs,
					points2D, properties, detectionPoints2D, c, &target, trackedTarget))
				{
					occupyTargetMatches(trackedTarget.match2D);
					{ // Make sure no async detection of this target is ongoing
						if (pipeline.tracking.asyncDetection && pipeline.tracking.asyncDetectTargetID == target.id)
							pipeline.tracking.asyncDetectionStop.request_stop();
					}
					recordTrackingResults(pipeline, frame, trackedTarget);
					trackedTarget.lastTrackedFrame = frame->num;
					pipeline.tracking.trackedTargets.push_back(std::move(trackedTarget));
					lostTarget = pipeline.tracking.lostTargets.erase(lostTarget);
					break;
				}
			}
			else if (detect.enable2DAsync)
			{
				std::vector<std::vector<int>> detectionPoints2D = remainingPoints2D;
				// TODO: Check specific cluster that is likely
				//detectionPoints2D[c] = clusters2D.front();

				pipeline.tracking.asyncDetection = true;
				pipeline.tracking.asyncDetectionStop = {};
				pipeline.tracking.asyncDetectTargetID = target.id;
				threadPool.push([&pipeline](int, std::stop_token stopToken, std::shared_ptr<FrameRecord> frameRec,
					std::vector<CameraCalib> calibs, std::vector<std::vector<int>> detectionPoints2D,
					const TargetTemplate3D *target, int focus)
				{
					bool success = detectTargetAsync(stopToken, pipeline, frameRec, calibs, detectionPoints2D, focus, target);
					pipeline.tracking.asyncDetection = false;
				}, pipeline.tracking.asyncDetectionStop.get_token(), frame, calibs, detectionPoints2D, &target, c);
				break;
			}
			lostTarget++;
		}
		if (pipeline.tracking.asyncDetection) break;

		auto unusedTarget = pipeline.tracking.unusedTargets.begin();
		while (unusedTarget != pipeline.tracking.unusedTargets.end() &&
			remainingPoints2D[c].size() >= detect.minObservations.focus)
		{
			const TargetTemplate3D &target = **unusedTarget;
			if (target.markers.size() < detect.minObservations.focus)
			{
				unusedTarget++;
				continue;
			}

			LOG(LDetection2D, LInfo, "Trying unused target %d (name %s) with %d markers!\n",
				target.id, target.label.c_str(), (int)target.markers.size());
			
			if (detect.enable2DSync)
			{
				std::vector<std::vector<int> const *> detectionPoints2D = relevantPoints2D;
				// TODO: Check specific cluster that is likely
				//detectionPoints2D[c] = clusters2D.front();
				pipeline.tracking.syncDetectionStop = {};
				TrackedTargetFiltered trackedTarget;
				if (detectTarget(pipeline.tracking.syncDetectionStop.get_token(), pipeline, frame, calibs,
				points2D, properties, detectionPoints2D, c, &target, trackedTarget))
				{
					occupyTargetMatches(trackedTarget.match2D);
					recordTrackingResults(pipeline, frame, trackedTarget);
					trackedTarget.lastTrackedFrame = frame->num;
					{ // Make sure no async detection of this target is ongoing
						if (pipeline.tracking.asyncDetection && pipeline.tracking.asyncDetectTargetID == target.id)
							pipeline.tracking.asyncDetectionStop.request_stop();
					}
					pipeline.tracking.trackedTargets.push_back(std::move(trackedTarget));
					unusedTarget = pipeline.tracking.unusedTargets.erase(unusedTarget);
					break;
				}
			}
			else if (detect.enable2DAsync)
			{
				std::vector<std::vector<int>> detectionPoints2D = remainingPoints2D;
				// TODO: Check specific cluster that is likely
				//detectionPoints2D[c] = clusters2D.front();

				pipeline.tracking.asyncDetectionStop = {};
				pipeline.tracking.asyncDetection = true;
				pipeline.tracking.asyncDetectTargetID = target.id;
				threadPool.push([&pipeline](int, std::stop_token stopToken, std::shared_ptr<FrameRecord> frameRec,
					std::vector<CameraCalib> calibs, std::vector<std::vector<int>> detectionPoints2D,
					const TargetTemplate3D *target, int focus)
				{
					bool success = detectTargetAsync(stopToken, pipeline, frameRec, calibs, detectionPoints2D, focus, target);
					pipeline.tracking.asyncDetection = false;
				}, pipeline.tracking.asyncDetectionStop.get_token(), frame, calibs, detectionPoints2D, &target, c);
				break;
			}
			unusedTarget++;
		}
		if (pipeline.tracking.asyncDetection) break;
	}

	det2 = pclock::now();

	tpt0 = pclock::now();

	{ // Single Marker Tracking

		auto marker = pipeline.tracking.trackedMarkers.begin();
		while (marker != pipeline.tracking.trackedMarkers.end())
		{
			int matchedPoint = trackMarker(*marker, pipeline.tracking.points3D, triIndices, timestep, 3);
			if (matchedPoint >= 0)
			{
				triIndices.erase(std::find(triIndices.begin(), triIndices.end(), matchedPoint));
				marker++;
			}
			else
			{
				LOG(LTracking, LDarn, "Failed to find continuation of tracked point after %d measurements!\n", marker->measurements);
				marker = pipeline.tracking.trackedMarkers.erase(marker);
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
				LOG(LTracking, LDebug, "Added new tracked point!\n");
				occupied3D[p] = true;
			}
		}
	}*/

	tpt1 = pclock::now();

	{ // Log timing
		float frameTime = dt(start, pclock::now());
		static float accumFrameTime = 0.0f;
		static int accumCount = 0;
		accumFrameTime += frameTime;
		accumCount++;
		LOG(LPipeline, LDebug, "Tracking took %.3fms - average after %d cycles %f\n", frameTime, accumCount, accumFrameTime/accumCount);
		LOG(LPipeline, LDebug, "Target Tracking %.3fms; Triangulation %f ms; Target Detection 3D %.3fms; Target Detection 2D %.3fms; Marker Tracking %.3fms\n",
			dt(trk0, trk1), dt(tri0, tri3), dt(det0, det1), dt(det1, det2), dt(tpt0, tpt1));
		LOG(LPipeline, LDebug, "Triangulation split up: Ray Intersection %.3fms; Filtering %.3fms; Refinement %.3fms\n",
			dt(tri0, tri1), dt(tri1, tri2), dt(tri2, tri3));
	}
}