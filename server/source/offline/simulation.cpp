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

#include "simulation.hpp"
#include "pipeline/pipeline.hpp"

#include "util/eigenutil.hpp"

//#define LOG_MAX_LEVEL LTrace
#include "util/log.hpp"

#include <random>
#include <algorithm>

/*
 * Generate simulated data
 */


#define MAX_MARKER_POINTS 64

const bool recordPoints = true;

std::vector<MotionParameters> motionPresets = {
	MotionParameters { "Room", 2.0f, 0.0f, 0.00004f, 1.5f, true, 0.2f, 0.0f, 0.003f, 0.0f, 0.2f },
	MotionParameters { "Wide", 0.5f, 0.01f, 0.00005f, 1.2f, false, 0.2f, 0.05f, 0.005f, 0.01f, 0.2f },
	MotionParameters { "Center", 0.02f, 0.0015f, 0.00005f, 1.0f, false, 0.5f, 0.05f, 0.01f, 0.005f, 0.1f }
};


/* Variables */

static std::mt19937 gen = std::mt19937(std::random_device{}());


TargetCalibration3D PointCalibMarker({
	{
		Eigen::Vector3f::Zero(),
		Eigen::Vector3f::UnitZ(),
		330, // Some dead spots to simulate occlusions (360° for full coverage)
		0.04f // 4cm big marker
	}
});

#ifdef USE_LINE_LENGTH
TargetCalibration3D LineCalibMarker({
	{
		Eigen::Vector3f(-30,0,0),
		Eigen::Vector3f(-1,0,0),
		320, // Some dead spots to simulate occlusions (360° for full coverage)
		0.02f // 2cm big marker
	},
	{
		Eigen::Vector3f(+30,0,0),
		Eigen::Vector3f(+1,0,0),
		320, // Some dead spots to simulate occlusions (360° for full coverage)
		0.02f // 2cm big marker
	}
});
#endif


/* Functions */

static Eigen::Isometry3f genPoseInTrackingSpace(const PipelineState &pipeline);
static Eigen::Vector3f centeringForce(const PipelineState &pipeline, Eigen::Vector3f pos, float centerAttenuation, float centerForce, bool correctiveOnly);
static Eigen::Isometry3f generateFluidPose(const PipelineState &pipeline, SimulatedObject &object);

static Eigen::Vector2f generateNoise(const SimProjectionParameters &params);

static bool testBlobMerge(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::map<int,int> &mergeMap, const SimProjectionParameters &params);
static void applyMergeMap(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::map<int,int> &mergeMap, std::vector<int> &markerMap);
static void applyMergeMap(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, const std::map<int,int> &mergeMap);

static bool projectMarker(const CameraCalib &calib, const CameraMode &mode, const Eigen::Isometry3f &pose,
	const SimProjectionParameters &params, const Eigen::Vector3f &mkPoint, float mkSize, Eigen::Vector2f &ptPos, float &ptSize);

static void createTargetProjection(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::map<int,int> &mergeMap, std::vector<int> &markerMap,
	const TargetCalibration3D &target, const CameraCalib &calib, const CameraMode &mode, const Eigen::Isometry3f &pose, const SimProjectionParameters &params);

std::shared_ptr<FrameRecord> GenerateSimulationData(PipelineState &pipeline, FrameRecord &frameState)
{
	ScopedLogCategory optLogCategory(LSimulation);
	auto simLock = pipeline.simulation.contextualLock();
	SimulationState &simulation = *simLock;

	frameState.cameras.resize(pipeline.cameras.size());

	LOGC(LDebug, "Generating points for frame %" PRIu64 ":\n", frameState.num);

	std::shared_ptr<FrameRecord> simFrame = std::make_shared<FrameRecord>();
	simFrame->num = frameState.num;
	simFrame->triangulations.reserve(simulation.points.size());

	// ----- Generate 3D Point Cloud -----

	{ // Update and project triangulatable points
		const auto &params = simulation.pointSim;

		int oldCount = simulation.points.size();
		simulation.points.resize(params.pointCount);
		for (int i = oldCount; i < params.pointCount; i++)
		{
			simulation.points[i].pos = genPoseInTrackingSpace(pipeline).translation();
			simulation.points[i].vel.setZero();
		}

		// Motion update, with centering and pseudo-gravity simulation
		float szSquared = params.pointSize * params.pointSize;
		for (auto &pt : simulation.points)
		{
			pt.acc.setZero();
			for (auto &at : simulation.points)
			{
				auto diff = at.pos - pt.pos;
				if (diff.isZero()) continue;
				pt.acc += diff.cwiseSquare().cwiseMax(szSquared).cwiseInverse().cwiseProduct(diff.cwiseSign());
			}
			pt.acc *= params.pointAttraction;
			pt.acc += centeringForce(pipeline, pt.pos, params.centerAttenuation, params.centerForce, params.centerCorrectOnly);
		}
		for (auto &pt : simulation.points)
		{
			pt.pos += pt.vel + pt.acc/2;
			pt.vel += pt.acc;
			pt.vel *= 1 - params.pointDampening;
		}

		std::vector<std::array<int,2>> triSamples(simulation.points.size());
		std::map<int,int> mergeMap;
		std::vector<int> obsStatus, markerMap;
		for (auto &cam : pipeline.cameras)
		{
			if (cam->disabled) continue;

			auto &record = frameState.cameras[cam->index];
			record.received = true;

			mergeMap.clear();
			markerMap.clear();
			obsStatus.clear();
			obsStatus.resize(simulation.points.size(), 0); // Mark unused
			for (int i = 0; i < simulation.points.size(); i++)
			{
				auto &pt = simulation.points[i];

				// Project marker of given size
				Eigen::Vector2f ptPos;
				float ptSize;
				if (!projectMarker(cam->simulation.calib, cam->mode, Eigen::Isometry3f::Identity(),
					simulation.projectionParams, pt.pos, 0.01f, ptPos, ptSize))
					continue;

				// Register marker observation
				obsStatus[i] = 1; // Mark used, unconflicted
				markerMap.push_back(i);
				record.rawPoints2D.push_back(ptPos);
				record.properties.emplace_back(ptSize, 1000);

				// Test if it merges with any previous observation
				testBlobMerge(record.rawPoints2D, record.properties, mergeMap, simulation.projectionParams);
			}
			int originalPts = record.rawPoints2D.size();

			if (!mergeMap.empty())
			{ // Actually merge with prior observations
				for (auto &conflict : mergeMap)
				{ // Mark used, conflicted
					obsStatus[markerMap[conflict.first]] = 2;
					obsStatus[markerMap[conflict.second]] = 2;
				}

				applyMergeMap(record.rawPoints2D, record.properties, mergeMap);
			}

			for (int i = 0; i < simulation.points.size(); i++)
			{ // Count unconflicted and conflicted samples respectively
				if (obsStatus[i] == 1) triSamples[i][0]++;
				if (obsStatus[i] == 2) triSamples[i][1]++;
			}

			if (mergeMap.empty())
				LOGC(LDebug, "  Camera %u has %d visible tri points!\n", cam->id, (int)record.rawPoints2D.size());
			else
				LOGC(LDebug, "  Camera %u has %d visible tri points, with %d mergers on %d original points!\n", cam->id, (int)record.rawPoints2D.size(), (int)mergeMap.size(), originalPts);
		}

		for (int i = 0; i < simulation.points.size(); i++)
		{ // Register triangulations as they SHOULD be detected in simulated frame record
			auto &pt = simulation.points[i];
			int clean = triSamples[i][0], conflict = triSamples[i][1];
			//float confidence = (clean*clean)/(conflict+1);
			float confidence = clean*clean*2 + conflict;
			if (confidence >= pipeline.params.tri.minIntersectionConfidence)
				simFrame->triangulations.emplace_back(pt.pos, 0.001f, confidence);
		}
	}

	int i = -1;
	for (auto &object : simulation.objects)
	{
		i++;
		if (!object.enabled) continue;
		LOGC(LTrace, "    Generating object %d: %s!\n", object.id, object.label.c_str());

		// ----- Generate new pose -----

		Eigen::Isometry3f oldPose = object.pose;
		object.pose = generateFluidPose(pipeline, object);
		//if (object.logPose)
		{
			Eigen::Quaternionf quat(object.internalMotionState.RGT);
			LOGC(LDebug, "Object %s GT Pose: (%.4f, %.4f, %.4f), Quat (%.4f, %.4f, %.4f, %.4f) --"
				"Dynamic state %.3fcm/f, %.3fcm/f^2, %.3f°/f, Change: %.3fcm, %.3f°/f\n",
				object.label.c_str(),
				object.pose.translation().x(), object.pose.translation().y(), object.pose.translation().z(),
				quat.x(), quat.y(), quat.z(), quat.w(),
				object.internalMotionState.TD.norm()*100,
				object.internalMotionState.TA.norm()*100,
				Eigen::AngleAxisf(getQuaternionXYZ(object.internalMotionState.RD)).angle()/PI*180.0f,
				(object.pose.translation() - oldPose.translation()).norm()*100,
				Eigen::AngleAxisf(object.pose.rotation() * oldPose.rotation().transpose()).angle()/PI*180.0f
			);
		}

		// ----- Generate projection -----

		TrackerRecord tracker = {};
		tracker.id = object.id;
		tracker.pose.observed = object.pose;
		tracker.match2D = ptr::make_value<TargetMatch2D>();
		tracker.match2D->pose = object.pose;
		tracker.match2D->points2D.resize(pipeline.cameras.size());

		std::vector<int> markerObsCount;
		if (recordPoints)
			markerObsCount.resize(object.target.markers.size());

		for (auto &cam : pipeline.cameras)
		{
			if (cam->disabled) continue;

			LOGC(LTrace, "  Camera %u Frame %" PRIu64 ":\n", cam->id, frameState.num);
			auto &record = frameState.cameras[cam->index];
			record.received = true;

			// Project marker into camera view
			int startPts = record.rawPoints2D.size();
			std::vector<int> markerMap;
			std::map<int,int> mergeMap;
			createTargetProjection(record.rawPoints2D, record.properties, mergeMap, markerMap,
				object.target, cam->simulation.calib, cam->mode, object.pose, simulation.projectionParams);
			applyMergeMap(record.rawPoints2D, record.properties, mergeMap, markerMap);

			// Update TrackerRecord
			std::vector<int> validMarkerMap = markerMap;
			for (auto &conflict : mergeMap)
			{ // Mark used, conflicted
				validMarkerMap[conflict.first] = -2;
				validMarkerMap[conflict.second] = -2;
			}
			for (int i = 0; i < markerMap.size(); i++)
			{ // Enter valid samples into TargetMatch2D
				if (markerMap[i] < 0) continue; // Not visible or merged and should not be used
				tracker.match2D->points2D[cam->index].emplace_back(i, markerMap[i]);
				float error = simulation.projectionParams.blobNoiseStdDev * 2;
				tracker.match2D->error.mean += error;
				tracker.match2D->error.max = std::max(tracker.match2D->error.max, error);
				tracker.match2D->error.samples++;
			}

			if (recordPoints)
			{ // Invert GTMarkers2Point to points2GTMarker (merged points are not invertible)
				record.simulation.GTMarkers2Point.insert(record.simulation.GTMarkers2Point.end(),
					markerMap.begin(), markerMap.end());
				record.simulation.points2GTMarker.resize(record.rawPoints2D.size(), -1);
				for (int p = 0; p < markerMap.size(); p++)
				{
					int pt = markerMap[p];
					if (pt < 0) continue;
					markerObsCount[p]++;
					if (record.simulation.points2GTMarker[pt] == -1)
						record.simulation.points2GTMarker[pt] = p;
					else // Merged, multiple markers
						record.simulation.points2GTMarker[pt] = -2;
					LOGC(LTrace, "    Camera %u Point %d maps back to target point %d, recorded %d (-2 == multiple)\n",
						cam->id, pt, p, record.simulation.points2GTMarker[pt]);
				}
				assert(record.simulation.points2GTMarker.size() == record.rawPoints2D.size());
			}
			/* else
			{ // Shuffle points around
				std::shuffle(record.rawPoints2D.begin()+startPts, record.rawPoints2D.end(), gen);
			} */
		}

		if (tracker.match2D->error.samples > 0)
		{ // Record TrackerRecord on successful track
			tracker.result = tracker.match2D->error.samples >= 3? TrackingResult::TRACKED_POSE : TrackingResult::NO_TRACK;
			tracker.match2D->error.mean /= tracker.match2D->error.samples;
			simFrame->trackers.push_back(std::move(tracker));
		}

		// ----- Generate 3D Point Cloud -----

		if (SHOULD_LOGC(LTrace) && recordPoints)
		{ // Double check target projection via triangulation

			LOGC(LTrace, "Recording GT triangulations:\n");

			// Setup interfacing structures (reused for all points)
			std::vector<std::vector<Eigen::Vector2f>> blobContainer(pipeline.cameras.size());
			std::vector<std::vector<Eigen::Vector2f> const *> points2D(pipeline.cameras.size());
			std::vector<CameraCalib> calibs(pipeline.cameras.size());
			for (auto &cam : pipeline.cameras)
			{
				blobContainer[cam->index].resize(1); // Used to store a single blob for each camera involved with a point
				points2D[cam->index] = &blobContainer[cam->index];
				calibs[cam->index] = cam->calib;
			}
			TriangulatedPoint triPoint;

			for (int i = 0; i < object.target.markers.size(); i++)
			{
				if (markerObsCount[i] < 2)
					continue;
				Eigen::Vector3f gtPoint = object.pose * object.target.markers[i].pos;

				// Triangulate the point to double check
				triPoint.samples.clear();
				for (auto &cam : pipeline.cameras)
				{
					auto &record = frameState.cameras[cam->index];
					for (int j = 0; j < record.simulation.points2GTMarker.size(); j++)
					{
						if (record.simulation.points2GTMarker[j] == i)
						{
							blobContainer[cam->index][0] = undistortPoint(cam->calib, record.rawPoints2D[j]);
							triPoint.samples.emplace_back(cam->index, 0); // Working with full camera set
							break;
						}
					}
				}
				if (triPoint.samples.size() >= 2)
				{ // E.g. merged blobs might not be traceable
					Eigen::Vector3f point3D = refineTriangulationIterative<float>(points2D, calibs, triPoint);
					LOGC(LTrace, "  Triangulated target point %d (%.4f, %.4f, %.4f) from observations, error %.2fmm with %f error value\n",
						i, point3D.x(), point3D.y(), point3D.z(), (point3D-gtPoint).norm()*1000, triPoint.error);
				}
				else
				{
					LOGC(LTrace, "  Could not triangulate target point %d from observations since points weren't all traceable (may have been merged)", i);
				}
			}
		}
	}

	//simFrame->cameras = frameState.cameras;
	simFrame->finishedProcessing = true;

	return simFrame;
}

/**
 * Replace point data in frameState belonging to the given tracker record with simulated data
 */
void ReplaceTargetObservations(const PipelineState &pipeline, FrameRecord &frame, const std::vector<TrackerRecord> &trackers, bool keepUnmatchedObservations)
{
	ScopedLogCategory optLogCategory(LSimulation);
	auto sim_lock = pipeline.simulation.contextualRLock();
	const SimulationState &simulation = *sim_lock;
	const ReplaceParameters &params = simulation.replaceParams;
	// For temporarily showing original observations (e.g. to interactively compare)
	if (params.suspendReplacing) return;

	struct CameraReplace
	{
		std::vector<Eigen::Vector2f> rawPoints2D;
		std::vector<BlobProperty> properties;
		std::vector<bool> removedPoints;
	};
	std::vector<CameraReplace> cameraReplace(frame.cameras.size());
	for (int c = 0; c < frame.cameras.size(); c++)
		cameraReplace[c].removedPoints.resize(frame.cameras[c].rawPoints2D.size());

	for (const TrackerRecord &record : trackers)
	{
		if (!record.result.isDetected() && !record.result.isTracked()) continue;

		auto replaceIt = simulation.replace.find(record.id);
		if (replaceIt == simulation.replace.end()) continue;
		auto &replace = replaceIt->second;

		Eigen::Vector3f pos = record.pose.observed.translation()*100, rot = getEulerXYZ(record.pose.observed.rotation())*180/PI;
		LOGC(LDebug, "Replacing target %d with %d! Pos (%.2fcm, %.2fcm, %.2fcm), Rot (%.2f°, %.2f°, %.2f°)!",
			replace.srcID, replace.tgtID, pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z());

		for (int c = 0; c < frame.cameras.size(); c++)
		{
			if (pipeline.cameras[c]->disabled) continue;
			auto &camRec = frame.cameras[c];
			auto &camRep = cameraReplace[c];
			CameraCalib calib = pipeline.cameras[c]->calib;
			Eigen::Isometry3f mv = calib.view.cast<float>() * record.pose.observed;
			float targetDist = (record.pose.observed.translation() - calib.transform.translation().cast<float>()).norm();
			float paramScale = 5.0f / targetDist;
			float matchDist = paramScale * params.matchDist;
			// TODO: Rework all paramScale of expandAngle across all code
			// Needs more than one parameter to be fully expressive
			float expandViewAngle = params.expandMarkerViewAngle * paramScale;
			float occludeAngleTolerance = params.occludeAngleTolerance / paramScale; // Inverted to expandAngle so divide.

			// Reproject previously tracked tracker
			// Both to remove its samples from observations
			// And to determine where it was likely occluded
			int projected = 0, matched = 0;
			std::vector<std::pair<Eigen::Vector2f, float>> occlusionMap;
			occlusionMap.reserve(replace.srcCalib.markers.size());
			for (int i = 0; i < replace.srcCalib.markers.size(); i++)
			{
				Eigen::Vector2f proj2D;
				float facing = projectMarker(proj2D, replace.srcCalib.markers[i], calib, mv);
				if (facing + expandViewAngle < 0) continue;

				bool ptMatched = false;
				for (int p = 0; p < camRec.rawPoints2D.size(); p++)
				{
					Eigen::Vector2f obs2D = undistortPoint(calib, camRec.rawPoints2D[p]);
					if ((obs2D - proj2D).squaredNorm() < matchDist*matchDist)
					{ // may have multiple matches
						camRep.removedPoints[p] = true;
						ptMatched = true;
					}
				}
				if (ptMatched)
					matched++;
				projected++;

				// Build data point on occlusion map
				// If not matched, it might be occluded, or just 
				float occlusionFactor = ptMatched? -params.occlusionDiscredit : std::max(0.0f, facing - occludeAngleTolerance);
				occlusionMap.emplace_back(proj2D, occlusionFactor);
			}
			LOGC(LDebug, "    Camera %u: Matched %d / %d projected target markers, target distance %.2fm!", calib.id, matched, projected, targetDist);

			if (replace.tgtID == 0 || (matched == 0 && projected > 0))
				continue; // Don't want to replace or source target was fully occluded

			int preIndex = camRep.rawPoints2D.size();
			std::vector<int> markerMap;
			std::map<int,int> mergeMap;
			createTargetProjection(camRep.rawPoints2D, camRep.properties, mergeMap, markerMap,
				replace.tgtCalib, calib, pipeline.cameras[c]->mode, record.pose.observed, simulation.projectionParams);
			applyMergeMap(camRep.rawPoints2D, camRep.properties, mergeMap, markerMap);
			int replacePoints = camRep.rawPoints2D.size() - preIndex;

			// Remove points likely to be occluded based on actually occluded observations
			int pp = preIndex;
			for (int p = preIndex; p < camRep.rawPoints2D.size(); p++)
			{ // Check if projected point is likely occluded based on actual tracker matches
				Eigen::Vector2f proj = undistortPoint(calib, camRep.rawPoints2D[p]);
				float occlusionFactor = 0.0f;
				for (auto &occlusionSample : occlusionMap)
					occlusionFactor += occlusionSample.second / (proj - occlusionSample.first).squaredNorm();
				if (occlusionFactor > 0)
				{
					LOGC(LTrace, "        Camera %u: Marker occluded with factor %.2f!", calib.id, occlusionFactor);
					continue;
				}
				camRep.rawPoints2D[pp] = camRep.rawPoints2D[p];
				camRep.properties[pp] = camRep.properties[p];
				pp++;
			}
			LOGC(LDebug, "    Camera %u: Added %d / %d projected replacement target markers!",
				calib.id, pp-preIndex, (int)camRep.rawPoints2D.size()-preIndex);
			camRep.rawPoints2D.resize(pp);
			camRep.properties.resize(pp);
		}
	}

	for (int c = 0; c < frame.cameras.size(); c++)
	{
		if (pipeline.cameras[c]->disabled) continue;
		auto &camRec = frame.cameras[c];
		auto &camRep = cameraReplace[c];

		if (keepUnmatchedObservations)
		{ // Filter out matched points from recorded observations
			int pp = 0;
			for (int p = 0; p < camRec.rawPoints2D.size(); p++)
			{
				if (camRep.removedPoints[p]) continue;
				camRec.rawPoints2D[pp] = camRec.rawPoints2D[p];
				camRec.properties[pp] = camRec.properties[p];
				pp++;
			}
			LOGC(LDebug, "Camera %u: %d/%d observations remain, with %d newly projected!",
				pipeline.cameras[c]->calib.id, pp, (int)camRec.rawPoints2D.size(), (int)camRep.rawPoints2D.size());
			camRec.rawPoints2D.resize(pp);
			camRec.properties.resize(pp);
			camRec.points2D.clear();
		}
		else
		{ // Delete all existing, act like simulation mode with movement determined by recorded trackers
			camRec.rawPoints2D.clear();
			camRec.properties.clear();
			camRec.points2D.clear();
		}

		std::move(std::begin(camRep.rawPoints2D), std::end(camRep.rawPoints2D), std::back_inserter(camRec.rawPoints2D));
		std::move(std::begin(camRep.properties), std::end(camRep.properties), std::back_inserter(camRec.properties));
	}
}

static Eigen::Isometry3f genPoseInTrackingSpace(const PipelineState &pipeline)
{
	// Min forced on the groundplane
	Eigen::Vector3f min = Eigen::Vector3f::Zero(), max = Eigen::Vector3f::Zero();
	for (const auto &cam : pipeline.cameras)
	{
		if (cam->disabled) continue;
		min = min.array().min(cam->simulation.calib.transform.translation().cast<float>().array());
		max = max.array().max(cam->simulation.calib.transform.translation().cast<float>().array());
	}
	Eigen::Vector3f diff = max-min;
	min += diff*0.2f;
	diff *= 0.6f;
	Eigen::Isometry3f pose;
	pose.translation() = Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * diff.x() + min.x(),
		(rand()%10000 / 10000.0f) * diff.y() + min.y(),
		(rand()%10000 / 10000.0f) * diff.z() + min.z());
	pose.linear() = getRotationZYX(Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * 360 - 180,
		(rand()%10000 / 10000.0f) * 360 - 180,
		(rand()%10000 / 10000.0f) * 360 - 180));
	return pose;
}

static Eigen::Vector3f centeringForce(const PipelineState &pipeline, Eigen::Vector3f pos, float centerAttenuation, float centerForce, bool correctiveOnly)
{
	auto attenuate = [](float val, float att){
		att = std::pow(std::abs(val), att);
		return val < 0? -att : att;
	};
	Eigen::Vector3f force = Eigen::Vector3f::Zero();
	for (const auto &cam : pipeline.cameras)
	{
		if (cam->disabled) continue;
		// Calculate corrective target position in 2D camera view
		Eigen::Vector2f tgt2D = Eigen::Vector2f::Zero();
		if (correctiveOnly)
		{ // Project and clamp to view bounds to find corrective target
			tgt2D = projectPoint2D(cam->simulation.calib.camera, pos);
			if (!std::isfinite(tgt2D.x()) || !std::isfinite(tgt2D.y()))
				tgt2D = Eigen::Vector2f::Zero();
			if (std::abs(tgt2D.x()) > cam->mode.sizeW)
				tgt2D *= cam->mode.sizeW / std::abs(tgt2D.x());
			if (std::abs(tgt2D.y()) > cam->mode.sizeH)
				tgt2D *= cam->mode.sizeH / std::abs(tgt2D.y());
		}
		// Determine closest 3D target
		Ray3f ray3D = castRay<float>(tgt2D, cam->simulation.calib);
		Eigen::Vector3f tgt3D = ray3D.pos + ray3D.dir * getRaySection(ray3D, pos);
		float deviation3D = (tgt3D - pos).norm();
		if (!std::isfinite(deviation3D))
			continue;
		if (deviation3D < 0.1)
			continue;
		Eigen::Vector3f tgtDir = (tgt3D - pos) / deviation3D;
		// Apply force to correct to that target position
		force += tgtDir * centerForce * attenuate(deviation3D, centerAttenuation);
		assert(!force.hasNaN());
	}
	return force;
}

static Eigen::Isometry3f generateFluidPose(const PipelineState &pipeline, SimulatedObject &object)
{
	auto &motion = object.internalMotionState;
	auto &params = motionPresets[object.motionPreset];
	// Generate consistent movement (positional acceleration is in mm)
	motion.TA += Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * params.accT - params.accT/2,
		(rand()%10000 / 10000.0f) * params.accT - params.accT/2,
		(rand()%10000 / 10000.0f) * params.accT - params.accT/2)/1000;
	motion.RA += Eigen::Vector3f(
		(rand()%10000 / 10000.0f) * params.accR - params.accR/2,
		(rand()%10000 / 10000.0f) * params.accR - params.accR/2,
		(rand()%10000 / 10000.0f) * params.accR - params.accR/2);
	if (motion.TA.norm() < params.minAcc/1000)
		motion.TA = motion.TA.normalized() * params.minAcc/1000;
	// Dampen movement
	motion.TA *= 1.0-params.dampT;
	motion.RA *= 1.0-params.dampR;
	motion.TA -= motion.TD * params.slowT;
	motion.RA -= motion.RD * params.slowR;
	// Move towards center
	motion.TA += centeringForce(pipeline, motion.TGT, params.centerAttenuation, params.centerForce, params.centerCorrectOnly);
	// Apply
	motion.TGT += motion.TD+motion.TA/2;
	motion.RGT = motion.RGT * getRotationXYZ(motion.RD+motion.RA/2);;
	motion.TD += motion.TA;
	motion.RD += motion.RA;
	// Finalise
	return createModelMatrix(motion.TGT, motion.RGT);
}

static Eigen::Vector2f generateNoise(const SimProjectionParameters &params)
{
	// Generate noise
	std::normal_distribution<float> noise(0, params.blobNoiseStdDev);
	float noiseX = noise(gen), noiseY = noise(gen); // NOTE: Noise select, first unpredictable, second predictable
	//float noiseX = rand()%10000 / 10000.0f * params.blobNoiseStdDev*2, noiseY = rand()%10000 / 10000.0f * params.blobNoiseStdDev*2;
	const float maxNoise = params.blobNoiseMaxSigma * params.blobNoiseStdDev;
	if (std::abs(noiseX) > maxNoise) noiseX /= std::ceil(std::abs(noiseX)/maxNoise);
	if (std::abs(noiseY) > maxNoise) noiseY /= std::ceil(std::abs(noiseY)/maxNoise);
	return Eigen::Vector2f(noiseX, noiseY);
}

/**
 * Tests if the latest blob should be merged with another prior nearby blob, and record it in mergeMap
 */
static bool testBlobMerge(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::map<int,int> &mergeMap, const SimProjectionParameters &params)
{
	Eigen::Vector2f ptPos = points2D.back();
	float ptSize = properties.back().size;
	for (int p = 0; p < points2D.size()-1; p++)
	{
		auto &prop = properties[p];
		float distSq = (points2D[p] - ptPos).squaredNorm();
		float radius = prop.size + ptSize;
		if (distSq < radius*radius * params.mergeFactor*params.mergeFactor)
		{ // Unless they are overlapping really bad, the algorithm could potentially discern them still, so mergeFactor < 1 is fine
			LOGC(LTrace, "     -> Point merging into %d!", p);
			LOGC(LDebug, "Merged with point %d at distance %.2fpx with size %.2fpx and %.2fpx\n",
				p, std::sqrt(distSq)*PixelFactor, prop.size*PixelFactor, ptSize*PixelFactor);
			LOGC(LTrace, "        While projecting point %d (%f, %f), decided to merge into point %d instead. "
				"Distance %f, sizes %f and %f, new size %f, pos (%f, %f)\n",
					(int)points2D.size()-1, ptPos.x()*PixelFactor, ptPos.y()*PixelFactor, p,
					std::sqrt(distSq)*PixelFactor, ptSize*PixelFactor, (radius-ptSize)*PixelFactor,
					prop.size*PixelFactor, points2D[p].x()*PixelFactor, points2D[p].y()*PixelFactor);
			mergeMap[points2D.size()-1] = mergeMap.contains(p)? mergeMap[p] : p;
			return true;
		}
		else if (distSq < radius*radius * 2*2)
		{
			LOGC(LTrace, "        While projecting point %d (%f, %f), nearly merged into point %d instead. Distance %f, sizes %f and %f\n",
				(int)points2D.size()-1, ptPos.x()*PixelFactor, ptPos.y()*PixelFactor, p, std::sqrt(distSq)*PixelFactor, ptSize*PixelFactor, properties[p].size*PixelFactor);
		}
	}
	return false;
}

/**
 * Apply merges in mergeMap to points2D and properties, as well as a source marker -> blob list
 */
static void applyMergeMap(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::map<int,int> &mergeMap, std::vector<int> &markerMap)
{
	if (mergeMap.empty())
		return;
	for (auto &merge : mergeMap)
	{ // Apply merges, retaining original points to not disturb indices
		float dist = (points2D[merge.first] - points2D[merge.second]).norm();
		float radius = properties[merge.first].size + properties[merge.second].size;
		points2D[merge.second] = (properties[merge.first].size*points2D[merge.first] + properties[merge.second].size*points2D[merge.second]) / radius;
		properties[merge.second].size = (radius + dist) / 2;
	}

	int pp = -1;
	for (int i = 0; i < markerMap.size(); i++)
	{
		if (markerMap[i] < 0) continue;
		if (pp < 0) pp = markerMap[i];
		int p = markerMap[i];
		auto map = mergeMap.find(p);
		if (map != mergeMap.end())
		{ // Update markerMap and delete point p, was merged into an earlier one
			markerMap[i] = map->second;
			for (auto &merge : mergeMap)
			{ // Update indices of mergeMap
				if (merge.second > p)
					merge.second--;
			}
		}
		else
		{ // Keep point p, move it to new location
			points2D[pp] = points2D[p];
			properties[pp] = properties[p];
			markerMap[i] = pp;
			pp++;
		}
		p++;
	}
	if (pp < 0) return;
	points2D.resize(pp);
	properties.resize(pp);
}

/**
 * Apply merges in mergeMap to points2D and properties
 */
static void applyMergeMap(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, const std::map<int,int> &mergeMap)
{
	for (auto &merge : mergeMap)
	{ // Apply merges, retaining original points to not disturb indices
		float dist = (points2D[merge.first] - points2D[merge.second]).norm();
		float radius = properties[merge.first].size + properties[merge.second].size;
		points2D[merge.second] = (properties[merge.first].size*points2D[merge.first] + properties[merge.second].size*points2D[merge.second]) / radius;
		properties[merge.second].size = (radius + dist) / 2;
	}

	int pp = mergeMap.begin()->first; // Replace from first merged marker
	for (int p = pp; p < points2D.size(); p++)
	{
		if (!mergeMap.contains(p))
		{ // Keep point, move it to new location
			points2D[pp] = points2D[p];
			properties[pp] = properties[p];
			pp++;
		}
	}
	points2D.resize(pp);
	properties.resize(pp);
}

static bool projectMarker(const CameraCalib &calib, const CameraMode &mode, const Eigen::Isometry3f &pose,
	const SimProjectionParameters &params, const Eigen::Vector3f &mkPoint, float mkSize, Eigen::Vector2f &ptPos, float &ptSize)
{
	// Project point
	Eigen::Projective3f mvp = calib.camera.cast<float>() * pose;
	Eigen::Vector4f proj = mvp * mkPoint.homogeneous();
	if (proj.w() < 0)
		return false;
	proj.head<2>() /= proj.w();
	if (proj.hasNaN())
		return false;
	// TODO: This acts as an upper limit to simulated distortion
	// But it serves to filter before an unstable distortion below
	if (proj.x() < -mode.sizeW*2 || proj.y() < -mode.sizeH*2 || proj.x() > mode.sizeW*2 || proj.y() > mode.sizeH*2)
		return false;

	// Apply distortion
	Eigen::Vector2f projDist = distortPointUnstable<float>(calib, proj.head<2>(), 1000, 0.001f*PixelSize);
	Eigen::Vector2f check = undistortPoint(calib, projDist);
	float diff = (proj.head<2>()-check).squaredNorm();
	if (diff > 1*1*PixelSize*PixelSize)
		LOGC(LDarn, "Simulated blob distortion is unstable in camera #%u: Error of %.2fpx", calib.id, std::sqrt(diff)*PixelFactor);
	ptPos = projDist + generateNoise(params);

	// Clip
	if (ptPos.x() < -mode.sizeW || ptPos.y() < -mode.sizeH || ptPos.x() > mode.sizeW || ptPos.y() > mode.sizeH)
		return false;
	if (ptPos.hasNaN())
		return false;

	// Determine point size (as radius in -1 to 1 space or diameter in 0 to 1 space)
	//float ptSize = 1.0f + 0.1f/camPoint.z();
	Eigen::Vector3f camUpVec = calib.transform.matrix().col(2).head<3>().cast<float>();
	Eigen::Vector3f distVec = mkPoint - calib.transform.translation().cast<float>();
	Eigen::Vector3f sideVec = distVec.cross(camUpVec).normalized() * mkSize/2;
	Eigen::Vector2f sideProj = projectPoint2D(mvp, mkPoint + sideVec);
	Eigen::Vector2f sideDist = distortPointUnstable<float>(calib, sideProj, 1000, 0.001f*PixelSize);
	ptSize = (sideDist - projDist).norm() * 2;
	if (SHOULD_LOGC(LTrace) && ptSize < 2*PixelFactor)
	{
		float ptSizeUndist = (sideProj - proj.head<2>()).norm();
		float ptDist = (mkPoint-calib.transform.translation().cast<float>()).norm();
		float sideDist = (mkPoint+sideVec-calib.transform.translation().cast<float>()).norm();
		LOGC(LTrace, "        Point size is %.2fpx with %.4fmm source size, f of %f, at distance of %.4fm",
			ptSize*PixelFactor, mkSize*1000, calib.f, ptDist);
		LOGC(LTrace, "        CamUpVec %f, sideVec %fmm, side point dist diff of %.4fmm, size undist %.2fpx",
			camUpVec.norm(), sideVec.norm()*1000, (sideDist-ptDist)*1000, ptSizeUndist * PixelFactor);
	}
	return true;
}

/**
 * Projects target into camera view, clipping out-of-view points, merging closeby points, and applying noise
 */
static void createTargetProjection(std::vector<Eigen::Vector2f> &points2D, std::vector<BlobProperty> &properties, std::map<int,int> &mergeMap, std::vector<int> &markerMap,
	const TargetCalibration3D &target, const CameraCalib &calib, const CameraMode &mode, const Eigen::Isometry3f &pose, const SimProjectionParameters &params)
{
	// Reserve space for projected points
	int baseMarker = markerMap.size();
	points2D.reserve(points2D.size() + target.markers.size());
	properties.reserve(properties.size() + target.markers.size());
	markerMap.resize(markerMap.size() + target.markers.size(), -1);
	// Merge from points2D -> points2D
	Eigen::Isometry3f mv = calib.view.cast<float>() * pose;
	for (int i = 0; i < target.markers.size(); i++)
	{
		const TargetMarker &markerPt = target.markers[i];

		// Cull back
		Eigen::Vector3f camPoint = mv * markerPt.pos;
		if (camPoint.z() < 0)
			continue;

		// Calculate and clip marker points not facing the camera in regards to their field of view
		Eigen::Vector3f ptNrm = mv.linear() * markerPt.nrm;
		float facing = -ptNrm.dot(camPoint.normalized());
		if (facing + params.expandMarkerViewAngle < markerPt.viewAngle)
			continue;

		// Project marker with given size
		Eigen::Vector2f ptPos;
		float ptSize;
		if (!projectMarker(calib, mode, pose, params, markerPt.pos, markerPt.size, ptPos, ptSize))
			continue;

		// Adjust size by view angle
		if (params.grazingAngleDiminishSize && markerPt.viewAngle > 0.1)
		{ // Well below 180° FoV - very likely flat marker that has less light reflected on tight view angles
			float sizeDiminish = (facing - markerPt.viewAngle - params.grazingAngleLower) / (params.grazingAngleUpper-params.grazingAngleLower);
			ptSize *= std::max(0.0f, std::min(1.0f, sizeDiminish));
		}
		if (ptSize < params.minSourceBlobSize)
			continue;
		ptSize = ptSize*params.blobVisualSizeFactor + params.blobVisualSizeFlare;

		// Register projected marker point
		markerMap[baseMarker+i] = points2D.size();
		points2D.push_back(ptPos);
		properties.emplace_back(ptSize, 1000);

		// Test if it merges with any previous blob
		testBlobMerge(points2D, properties, mergeMap, params);

		LOGC(LTrace, "    Camera %u: Done projecting point %d (%f, %f), size %f\n", calib.id, (int)points2D.size(), ptPos.x()*PixelFactor, ptPos.y()*PixelFactor, ptSize*PixelFactor);
	}
}