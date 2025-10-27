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

#include "ui/system/vis.hpp"
#include "ui/ui.hpp"
#include "ui/gl/visualisation.hpp"
#include "point/sequence_data.inl" // resolveGTMarker


/* Functions */

VisTargetLock VisualisationState::lockVisTarget() const
{
	VisTargetLock state = {};
	if (targetCalib.edit)
	{
		state.editRef = targetCalib.edit;
		state.obs = &targetCalib.edit->target;
		state.calib = &targetCalib.edit->targetCalib;
		state.hasPose = true;
		state.targetGT = targetCalib.edit->simulation.targetGT;
	}
	else if (targetCalib.view)
	{
		state.target_lock = targetCalib.view->target.contextualRLock();
		state.obs = &*state.target_lock;
		thread_local TargetCalibration3D tempTargetCalib = {};
		tempTargetCalib.initialise(state.obs->markers);
		state.calib = &tempTargetCalib;
		//state.calib = &targetCalib.view->calib;
		state.hasPose = targetCalib.view->state.calibrated;
		state.targetGT = targetCalib.view->simulation.targetGT;
	}
	else if (targetCalib.stage)
	{
		state.obs = &targetCalib.stage->base.target;
		state.calib = &targetCalib.stage->base.targetCalib;
		//if (targetCalib.highlightedStageTarget >= 0 && targetCalib.highlightedStageTarget < targetCalib.selStage->mergeTests.size())
		//	state.target = &targetCalib.selStage->mergeTests[targetCalib.highlightedStageTarget].newBase.optTarget;
		state.hasPose = true;
		state.targetGT = targetCalib.stage->base.simulation.targetGT;
	}
	else if (target.inspectingTrackerID != 0)
	{
		// Get TargetCalibration3D
		auto trackerIt = std::find_if(GetState().trackerConfigs.begin(), GetState().trackerConfigs.end(),
			[&](auto &tgt){ return tgt.id == target.inspectingTrackerID; });
		if (trackerIt != GetState().trackerConfigs.end() && trackerIt->type == TrackerConfig::TRACKER_TARGET)
			state.calib = &trackerIt->calib;
		else if (!target.inspectingTargetCalib.markers.empty())
			state.calib = &target.inspectingTargetCalib;
		else
			return state;
		state.targetGT = nullptr;
		// Get ObsTarget if tracking data exists
		auto db_lock = GetState().pipeline.obsDatabase.contextualRLock();
		auto obsIt = std::find_if(db_lock->targets.begin(), db_lock->targets.end(),
			[&](auto &tgt){ return tgt.trackerID == target.inspectingTrackerID; });
		if (obsIt == db_lock->targets.end())
			return state;
		state.db_lock = std::move(db_lock);
		state.obs = &*obsIt;
		state.hasPose = true;
	}
	else // No target to visualise
		return state;
	assert(state.calib);
	assert(state.obs);
	assert(!state.obs->frames.empty());
	state.frameIdx = targetCalib.frameIdx % state.obs->frames.size();
	return state;
}

bool VisualisationState::resetVisTarget(bool keepFrame)
{
	if (targetCalib.edit)
		return false;
	// Allowed to edit
	if (keepFrame)
	{ // Try to store real frame to restore later
		targetCalib.frameNum = -1;
		VisTargetLock visTarget = lockVisTarget();
		if (visTarget.hasObs())
			targetCalib.frameNum = visTarget.obs->frames[targetCalib.frameIdx % visTarget.obs->frames.size()].frame;
	}
	// Reset state
	targetCalib.edit = nullptr;
	targetCalib.view = nullptr;
	targetCalib.stage = nullptr;
	targetCalib.stageSubIndex = -1;
	targetCalib.stageSubSubIndex = -1;
	target.inspectingSource = 'N';
	target.inspectingTrackerID = 0;
	target.inspectingTargetCalib = {};
	target.markerSelect.clear();
	target.editMarkerFoV = 0.0f;
	return true;
}

void VisualisationState::updateVisTarget(const VisTargetLock &visTarget)
{
	if (visTarget.hasObs() && targetCalib.frameNum >= 0)
	{ // Try to re-focus on the same frame while switching
		for (targetCalib.frameIdx = 0; targetCalib.frameIdx < visTarget.obs->frames.size(); targetCalib.frameIdx++)
		{
			if (visTarget.obs->frames[targetCalib.frameIdx].frame >= targetCalib.frameNum)
				break;
		}
		targetCalib.frameNum = -1;
	}
	if (visTarget.hasObs())
		targetCalib.frameIdx = targetCalib.frameIdx % visTarget.obs->frames.size();
	target.markerSelect.resize(visTarget.calib->markers.size(), false);
}

void VisualisationState::updateVisTarget()
{
	VisTargetLock visTarget = lockVisTarget();
	if (visTarget)
		updateVisTarget(visTarget);
}

thread_local std::vector<VisPoint> markerPoints;
thread_local std::vector<int> markerIndices;
std::vector<VisPoint>& visualiseVisTargetMarkers(const PipelineState &pipelineGT, const VisualisationState &visState, const VisTargetLock &visTarget)
{
	assert(visTarget);
	Eigen::Isometry3f tgtPose = visTarget.getPose();

	float focusSizeFactor = 2.0f;
	float visibleAlphaFactor = 1.5f;

	float gtSize = 0.003f;
	Color8 gtColor = Color{ 1.0f, 0.0f, 0.8f, 0.8f };
	Color8 gtHighlight = Color{ 0.6f, 0.0f, 0.5f, 1.0f };
	float auxSize = 0.004f;
	Color8 auxColor = Color{ 0.0f, 0.0f, 1.0f, 0.4f };
	Color8 mappedColor = Color{ 0.1f, 0.8f, 0.1f, 0.8f };
	float mkSize = 0.006f;
	Color8 mkColor = Color{ 0.6f, 0.6f, 0.6f, 0.4f };
	Color8 mkHighlight = Color{ 0.6f, 0.4f, 0.1f, 0.4f };
	
	// Get markers that were recorded in a sequence this frame
	std::vector<bool> markersVisible(visTarget.calib->markers.size());
	if (visTarget.hasObs())
	{
		assert(markersVisible.size() == visTarget.obs->markers.size());
		auto &frame = visTarget.obs->frames[visTarget.frameIdx];
		for (auto &sample : frame.samples)
			markersVisible[visTarget.obs->markerMap.at(sample.marker)] = true;
	}

	markerPoints.clear();
	markerIndices.clear();

	if (visTarget.hasObs() && visTarget.hasPose && visTarget.targetGT && pipelineGT.isSimulationMode)
	{ // Draw markers of GT target
		Eigen::Isometry3f pose;
		int mBase = markerPoints.size();
		{
			auto sim_lock = pipelineGT.simulation.contextualRLock();
			auto &frame = visTarget.obs->frames[visTarget.frameIdx];
			pose = sim_lock->framePoses[frame.frame]; // Have to rely on it being primary object at the time of calibration
			for (const auto &marker : visTarget.targetGT->markers)
			{
				markerPoints.emplace_back(pose * marker.pos, gtColor, gtSize);
				markerIndices.push_back(-1);
			}
		}
		{
			auto obs_lock = pipelineGT.seqDatabase.contextualRLock();
			for (auto &map : visTarget.obs->markerMap)
			{
				auto gtMarker = obs_lock->markers[map.first].resolveGTMarker();
				auto &markerPt = markerPoints[mBase+gtMarker.first];
				markerIndices[mBase+gtMarker.first] = map.second;
				if (visState.target.markerSelect[map.second])
					markerPt.color = gtHighlight;
				if (visState.target.markerFocussed == map.second)
					markerPt.size *= focusSizeFactor;
				if (markersVisible[map.second])
					markerPt.color.a = std::min(255, (int)(markerPt.color.a * visibleAlphaFactor));
			}
		}
		visualisePose(pose, gtColor, 0.1f, 2.0f);
	}

	if (visTarget.hasPose && visState.targetCalib.stage && visState.targetCalib.stageSubIndex >= 0 &&
		visState.targetCalib.stageSubIndex < visState.targetCalib.stage->alignResults.size() && visState.targetCalib.stageSubSubIndex >= 0)
	{ // Draw markers of aligning result of selected assembly stage
		auto &viewRes = visState.targetCalib.stage->alignResults[visState.targetCalib.stageSubIndex];
		auto &cand = viewRes.candidates[visState.targetCalib.stageSubSubIndex];
		Eigen::Isometry3f pose = tgtPose * cand.pose.inverse();
		int mBase = markerPoints.size();
		for (int m = 0; m < viewRes.markers.size(); m++)
		{
			auto &marker = viewRes.markers[m];
			// Highlight markers used for aligning
			int map = cand.pointMap[m];
			markerPoints.emplace_back(pose * marker, map < 0? auxColor : mappedColor, auxSize);
			markerIndices.push_back(map);
		}
		if (visState.target.markerFocussed >= 0 && visState.target.markerFocussed < viewRes.markers.size())
			markerPoints[mBase+visState.target.markerFocussed].size *= focusSizeFactor;
		visualisePose(pose, auxColor, 0.1f, 2.0f);
	}

	if (visTarget.hasPose && visState.targetCalib.stage && visState.targetCalib.stageSubIndex >= 0 &&
		visState.targetCalib.stageSubIndex < visState.targetCalib.stage->mergeTests.size())
	{ // Draw markers of merging result of selected assembly stage
		auto &viewRes = visState.targetCalib.stage->mergeTests[visState.targetCalib.stageSubIndex];
		int mBase = markerPoints.size();
		for (int m = 0; m < viewRes.markers.size(); m++)
		{
			auto &marker = viewRes.markers[m];
			// Highlight markers used for merging
			int map = viewRes.pointMap[m];
			markerPoints.emplace_back(tgtPose * marker, map < 0? auxColor : mappedColor, auxSize);
			markerIndices.push_back(map);
		}
		if (visState.target.markerFocussed >= 0 && visState.target.markerFocussed < viewRes.markers.size())
			markerPoints[mBase+visState.target.markerFocussed].size *= focusSizeFactor;
	}

	{ // Draw current markers of actively selected target (target view or assembly stage)
		for (int m = 0; m < visTarget.calib->markers.size(); m++)
		{
			VisPoint markerPt = {};
			markerPt.pos = tgtPose * visTarget.calib->markers[m].pos;
			markerPt.color = visState.target.markerSelect[m]? mkHighlight : mkColor;
			if (markersVisible[m])
				markerPt.color.a = std::min(255, (int)(markerPt.color.a * visibleAlphaFactor));
			markerPt.size = mkSize;
			if (visState.target.markerFocussed == m)
				markerPt.size *= focusSizeFactor;
			markerPoints.push_back(markerPt);
			markerIndices.push_back(m);
		}
		visualisePose(tgtPose, mkColor, 0.1f, 2.0f);
	}

	return markerPoints;
}

static int interactWithPointCloud(const std::vector<VisPoint> &points, Eigen::Isometry3f view, Eigen::Projective3f proj, Eigen::Vector2f mouse)
{
	// Enter valid points with their distance
	thread_local std::vector<std::pair<int, float>> order;
	order.clear();
	for (int i = 0; i < points.size(); i++)
	{
		auto &pt = points[i];
		if (pt.color.a == 0) continue;
		float dist = (view.inverse() * pt.pos).z();
		order.emplace_back(i, dist - pt.size);
	}
	// Sort front-to-back
	std::sort(order.begin(), order.end(), 
		[&](auto &a, auto &b) { return a.second < b.second; });
	// Find frontmost circle hit (not proper sphere raycasting but whatever)
	Eigen::Projective3f vp = proj * view.inverse();
	for (int i = 0; i < order.size(); i++)
	{
		auto &pt = points[order[i].first];
		Eigen::Vector3f viewUpAxis = view.matrix().col(2).head<3>().cast<float>();
		Eigen::Vector3f sideVec = (pt.pos - view.translation().cast<float>()).cross(viewUpAxis);
		Eigen::Vector3f sidePos = pt.pos + (pt.size/2 * sideVec.normalized());
		Eigen::Vector2f pSide = (vp * sidePos.homogeneous()).hnormalized().head<2>();
		Eigen::Vector2f pCenter = (vp * pt.pos.homogeneous()).hnormalized().head<2>();
		float sizeSq = (pCenter - pSide).squaredNorm() * 2*2;
		float distSq = (pCenter - mouse).squaredNorm();
		if (distSq < sizeSq)
			return order[i].first;
	}
	return -1;
}

std::pair<int,int> interactWithVisTargetMarker(Eigen::Isometry3f view, Eigen::Projective3f proj, Eigen::Vector2f mouse)
{
	int interacting = interactWithPointCloud(markerPoints, view, proj, mouse);
	if (interacting < 0) return { -1, -1 };
	return { interacting, markerIndices[interacting] };
}

void visualiseVisTargetObservations(const std::vector<CameraCalib> &calibs, const VisualisationState &visState, const VisTargetLock &visTarget)
{
	assert(visTarget.hasObs() && visTarget.hasPose);
	auto &curFrame = visTarget.obs->frames[visTarget.frameIdx];

	thread_local std::vector<std::pair<VisPoint, VisPoint>> rayLines;
	rayLines.clear();
	std::vector<std::vector<Eigen::Vector3f>> markerRays(visTarget.obs->markers.size());
	auto enterFrameObsRay = [&](const ObsTargetFrame &frame, bool enter, float length, Color cam, Color marker)
	{
		for (auto &sample : frame.samples)
		{
			Eigen::Vector2f point = undistortPoint(calibs[sample.camera], sample.point);
			bool seqHighlight = visState.targetCalib.highlightedObservation == sample.marker;
			Ray3f ray = castRay<float>(point, calibs[sample.camera]);
			int m = visTarget.obs->markerMap.at(sample.marker);
			float len = seqHighlight? length*2 : length;
			Eigen::Vector3f mkPos = frame.pose * visTarget.obs->markers[m];
			Eigen::Vector3f mkIx = ray.pos + ray.dir * getRaySection(ray, mkPos);
			Eigen::Vector3f mkCam = mkIx + (ray.pos-mkIx).normalized()*len;
			mkIx = frame.pose.inverse() * mkIx;
			mkCam = frame.pose.inverse() * mkCam;
			if (enter) markerRays[m].push_back((mkCam-mkIx)/len);
			mkIx = curFrame.pose * mkIx;
			mkCam = curFrame.pose * mkCam;
			rayLines.emplace_back(
				VisPoint{ mkCam, cam },
				VisPoint{ mkIx, marker }
			);
		}
	};

	// Show all samples determining a marker as rays, with the current frame highlighted
	for (auto &frame : visTarget.obs->frames)
		enterFrameObsRay(frame, true, 0.01f, { 0.3f, 0.9f, 0.3f, 1.0f }, { 0.9f, 0.4f, 0.3f, 1.0f });
	visualiseLines(rayLines, 1.0f);

	rayLines.clear();
	enterFrameObsRay(curFrame, false, 0.02f, { 1.0f, 0.2f, 0.7f, 1.0f }, { 1.0f, 0.2f, 0.7f, 1.0f });

	visualiseLines(rayLines, 3.0f);
}

void visualiseVisTargetMarkerFoV(const std::vector<CameraCalib> &calibs, const VisualisationState &visState, const VisTargetLock &visTarget)
{
	assert(visTarget);
	Eigen::Isometry3f tgtPose = visTarget.getPose();

	thread_local std::vector<std::pair<VisPoint, VisPoint>> rayLines;
	rayLines.clear();

	for (auto &marker : visTarget.calib->markers)
	{
		Eigen::Vector3f pos = tgtPose * marker.pos;
		Eigen::Vector3f nrm = tgtPose * (marker.pos + marker.nrm * 0.02f);
		rayLines.emplace_back(
			VisPoint{ pos, Color{ 0.2f, 0.8f, 0.7f, 1.0f } },
			VisPoint{ nrm, Color{ 0.2f, 0.8f, 0.7f, 1.0f } }
		);
	}
	visualiseLines(rayLines, 3.0f);

	thread_local std::vector<VisPoint> coneMesh;
	const int coneRes = 20;
	coneMesh.clear();
	coneMesh.resize(1+coneRes+1, VisPoint{ Eigen::Vector3f::Zero(), Color{ 0.6f, 0.6f, 0.6f, 0.6f } });
	auto visCone = [&](Eigen::Vector3f pos, Eigen::Vector3f nrm, float angleLimit)
	{
		coneMesh[0].pos = pos;
		Eigen::Vector3f perp1(1, 0, 0);
		perp1 = (perp1 - perp1.dot(nrm) * nrm).normalized();
		Eigen::Vector3f perp2 = perp1.cross(nrm);
		pos += nrm * 0.02f * angleLimit;
		float side = 0.02f * std::sin(std::acos(angleLimit));
		for (int i = 0; i <= coneRes; i++)
		{
			float c = 2*(float)PI*(float)i/coneRes;
			coneMesh[i+1].pos = pos + side * (perp1 * std::sin(c) + perp2 * std::cos(c));
		}
		visualiseMesh(coneMesh, 6); // GL_TRIANGLE_FAN
	};
	for (auto &marker : visTarget.calib->markers)
	{
		visCone(tgtPose * marker.pos, tgtPose.linear() * marker.nrm, marker.angleLimit);
	}
	if (visState.target.editMarkerFoV != 0.0f)
	{
		coneMesh.clear();
		coneMesh.resize(1+coneRes+1, VisPoint{ Eigen::Vector3f::Zero(), Color{ 0.5f, 0.8f, 0.5f, 0.6f } });
		for (auto &marker : visTarget.calib->markers)
		{
			float angleLimit = std::min(1.0f, std::max(-1.0f, marker.angleLimit + visState.target.editMarkerFoV));
			visCone(tgtPose * marker.pos, tgtPose.linear() * marker.nrm, angleLimit);
		}
	}
}

void visualiseVisTargetObsCameraRays(const std::vector<CameraCalib> &calibs, const VisualisationState &visState, const VisTargetLock &visTarget)
{
	assert(visTarget.hasObs() && visTarget.hasPose);
	auto &curFrame = visTarget.obs->frames[visTarget.frameIdx];

	Color rayColor = { 0.3f, 0.4f, 0.9f, 1.0f };
	Color perpColor = { 0.9f, 0.4f, 0.2f, 1.0f };
	thread_local std::vector<std::pair<VisPoint, VisPoint>> rayLines;
	rayLines.clear();
	for (auto &sample : curFrame.samples)
	{
		if (visState.target.cameraRays.size() <= sample.camera || !visState.target.cameraRays[sample.camera]) continue;
		Eigen::Vector2f point = undistortPoint(calibs[sample.camera], sample.point);
		Ray3f ray = castRay<float>(point, calibs[sample.camera]);
		int m = visTarget.obs->markerMap.at(sample.marker);
		Eigen::Vector3f mkPos = curFrame.pose * visTarget.obs->markers[m];
		Eigen::Vector3f mkIx = ray.pos + ray.dir * getRaySection(ray, mkPos);
		rayLines.emplace_back(
			VisPoint{ ray.pos, rayColor },
			VisPoint{ mkIx, rayColor }
		);
		rayLines.emplace_back(
			VisPoint{ mkIx, perpColor },
			VisPoint{ mkPos, perpColor }
		);
	}
	visualiseLines(rayLines, 2.0f);
}

void visualiseMarkerSequenceRays(const std::vector<CameraCalib> &calibs, const VisTargetLock &visTarget, const MarkerSequences &seq, int sequenceMarker)
{
	assert(visTarget.hasObs() && visTarget.hasPose);
	auto &curFrame = visTarget.obs->frames[visTarget.frameIdx];
	auto m = visTarget.obs->markerMap.find(sequenceMarker);
	if (m == visTarget.obs->markerMap.end()) return;
	Eigen::Vector3f marker = visTarget.obs->markers[m->second];

	Color inlierColor = { 0.3f, 0.4f, 0.9f, 1.0f };
	Color outlierColor = { 0.9f, 0.4f, 0.2f, 1.0f };
	float length = 0.1f; // 10cm
	thread_local std::vector<std::pair<VisPoint, VisPoint>> rayLines;
	rayLines.clear();
	for (int c = 0; c < seq.cameras.size(); c++)
	{
		if (seq.cameras[c].sequences.empty()) continue;
		auto frameIt = std::lower_bound(visTarget.obs->frames.begin(), visTarget.obs->frames.end(), seq.cameras[c].begin().frame(),
			[](auto &a, int f){ return a.frame < f; });
		for (auto frame = seq.cameras[c].begin(); frame != seq.cameras[c].end(); frame++)
		{
			if (frameIt == visTarget.obs->frames.end()) break;
			if (frameIt->frame > frame.frame()) continue;
			Ray3f ray = castRay<float>(*frame, calibs[c]);
			Eigen::Vector3f mkPos = frameIt->pose * marker;
			Eigen::Vector3f midpoint = ray.pos + ray.dir * getRaySection(ray, mkPos);
			Eigen::Vector3f r1 = midpoint - ray.dir*length;
			Eigen::Vector3f r2 = midpoint + ray.dir*length;
			r1 = curFrame.pose * frameIt->pose.inverse() * r1;
			midpoint = curFrame.pose * frameIt->pose.inverse() * midpoint;
			r2 = curFrame.pose * frameIt->pose.inverse() * r2;
			rayLines.emplace_back(
				VisPoint{ r1, inlierColor },
				VisPoint{ midpoint, outlierColor }
			);
			rayLines.emplace_back(
				VisPoint{ midpoint, outlierColor },
				VisPoint{ r2, outlierColor }
			);
			frameIt++;
		}
	}
	visualiseLines(rayLines, 2.0f);
}

void visualiseTarget2DMatchingStages(VisualisationState &visState, const CameraCalib &calib, const CameraFrameRecord &frame,
	const TargetCalibration3D &target, const TargetTracking2DData::CameraMatchingStages &matchingData, float expandMarkerFoV)
{
	thread_local std::vector<Eigen::Vector2f> projected2D;
	thread_local std::vector<int> relevantProjected2D;
	auto &trkVis = visState.tracking;

	int camCount = trkVis.debug.targetMatch2D.points2D.size();
	trkVis.debug.targetBounds.resize(camCount);
	trkVis.debug.priLabels.resize(camCount);
	trkVis.debug.secLabels.resize(camCount);
	trkVis.debug.editButtons.resize(camCount);
	auto &priLabels = trkVis.debug.priLabels[calib.index];
	auto &secLabels = trkVis.debug.secLabels[calib.index];
	auto &buttons = trkVis.debug.editButtons[calib.index];
	priLabels.clear();
	secLabels.clear();
	buttons.clear();

	trkVis.debug.targetBounds[calib.index] = {};
	for (int i = 0; i < trkVis.debug.initialMatch2D.points2D[calib.index].size(); i++)
	{
		int pt = trkVis.debug.initialMatch2D.points2D[calib.index][i].second;
		trkVis.debug.targetBounds[calib.index].include(frame.points2D[pt]);
	}
	for (int i = 0; i < trkVis.debug.targetMatch2D.points2D[calib.index].size(); i++)
	{
		int pt = trkVis.debug.targetMatch2D.points2D[calib.index][i].second;
		trkVis.debug.targetBounds[calib.index].include(frame.points2D[pt]);
	}

	std::vector<std::pair<VisPoint,VisPoint>> matchLines;
	auto reprojection = [&calib, &target](const TargetMatchingData::MarkerMatchingData &markerMatch, const Eigen::Isometry3f &pose)
	{
		Eigen::Isometry3f mv = calib.view.cast<float>() * pose;
		return applyProjection2D(calib, mv * target.markers[markerMatch.marker].pos);
	};
	auto recordMatchCandidateLines = [&](const TargetMatchingData::MarkerMatchingData &mk, Eigen::Vector2f projected, float lower, float upper, float alpha = 1)
	{
		for (const auto &match : mk.matches)
		{
			Eigen::Vector2f pos = frame.points2D[match.index];
			Color col;
			if (match.value < lower)
				col = lerp({ 0, 1, 0, 1.0f*alpha }, { 1, 0, 0, 0.9f*alpha }, match.value/lower);
			else
				col = lerp({ 1, 0, 0, 0.9f*alpha }, { 1, 0, 1, 0.8f*alpha }, match.value/upper);
			matchLines.push_back({
				{ pos.homogeneous(), col },
				{ projected.homogeneous(), col }
			});
		}
	};
	auto createLabel = [&](const TargetMatchingData::MarkerMatchingData &mk, Eigen::Vector2f projected, const TargetMatchingData::Point2DMatchCandidate &match, Color col)
	{
		SceneLabel label;
		label.position.head<2>() = (frame.points2D[match.index] + projected)/2;
		label.radius = 1.0f*PixelSize;
		label.text = asprintf_s("Value: %.3f\nDiff: %.3f (dist: %.3f)\nMis: %.3f (sim: %.3f, infl: %.3f)",
			match.value,
			match.difference, std::sqrt(match.distSq)*PixelFactor,
			match.mismatch, match.similarity, match.influence);
		label.color = col;
		return label;
	};
	auto createEditToggle = [&](Eigen::Vector2f pos, int mkIdx, int ptIdx, bool toggle)
	{
		SceneButton button;
		button.position.head<2>() = pos;
		button.radius = 1.0f*PixelSize;
		button.context = (intptr_t)(mkIdx*0x35D0C2+ptIdx*0x44A7F3+pos.x()*0x854EF7);
		int cam = calib.index;
		button.callback = [cam, mkIdx, ptIdx](intptr_t){
			auto &edit = GetUI().visState.tracking.debug.editedMatch2D;
			auto &pts = edit.points2D[cam];
			for (auto m = pts.begin(); m != pts.end(); m++)
			{
				if (m->first != mkIdx) continue;
				if (m->second == ptIdx)
				{ // Remove
					pts.erase(m);
					edit.error.samples--;
				}
				else // Else, replace
					m->second = ptIdx;
				return;
			}
			// Nothing found, add new
			pts.push_back({ mkIdx, ptIdx });
			edit.error.samples++;
		};
		button.color = toggle? Color{ 0.6f, 0.6f, 1.0f, 1 } : Color{ 1.0f, 0.6f, 0.6f, 1 };
		return button;
	};

	if (trkVis.debug.showEditTools)
	{
		const auto &tgtMatch = trkVis.debug.editedMatch2D;
		const auto &matches = tgtMatch.points2D[calib.index];

		// Visualise target markers that should be visible
		projectTarget(projected2D,
			target, calib, tgtMatch.pose, expandMarkerFoV);
		visualisePoints2D(projected2D, Color{ 0.0, 0.8, 0.2, 0.3f }, 2.0f);

		// Then get the same marker projections but with proper indexes
		projectTarget(projected2D, relevantProjected2D,
			target, calib, tgtMatch.pose, expandMarkerFoV);

		// Now visualise all "closeby" points of each such target marker, with a button if it's not already matched
		matchLines.clear();
		for (int m : relevantProjected2D)
		{ // Find all closeby blobs to potentially match with
			for (int p = 0; p < frame.points2D.size(); p++)
			{
				auto &pt = frame.points2D[p];
				float distSq = (projected2D[m] - pt).squaredNorm();
				if (distSq > 10*10*PixelSize*PixelSize)
					continue; // Too far away to show
				if (std::find(matches.begin(), matches.end(), std::make_pair(m, p)) != matches.end())
					continue; // Already used in edited target match, visualise later
				matchLines.push_back({
					{ pt.homogeneous(), Color(0.0f, 0.5f, 0.2f, 1.0f) },
					{ projected2D[m].homogeneous(), Color(0.0f, 0.5f, 0.2f, 1.0f) }
				});
				buttons.push_back(createEditToggle((pt+projected2D[m])/2, m, p, false));
			}
		}
		visualiseLines(matchLines, 1.0f);

		// Then visualise all matches that are already used for the edited target match
		for (auto match : matches)
		{
			Eigen::Vector2f pt = frame.points2D[match.second];
			Eigen::Vector2f proj = projected2D[match.first];
			matchLines.push_back({
				{ pt.homogeneous(), Color(0.0f, 0.5f, 1.0f, 1.0f) },
				{ proj.homogeneous(), Color(0.0f, 0.5f, 1.0f, 1.0f) }
			});
			buttons.push_back(createEditToggle((pt+proj)/2, match.first, match.second, true));
		}
		visualiseLines(matchLines, 4.0f);
	}
	else if (trkVis.debugFocusStage == 0)
	{
		const auto &tgtMatch = trkVis.debug.showInitial? trkVis.debug.initialMatch2D :
			(trkVis.debug.showEdited? trkVis.debug.editedMatch2D : trkVis.debug.targetMatch2D);
		const auto &matches = tgtMatch.points2D[calib.index];

		// Visualise target points that were considered (since they should've been visible assuming the pose is about right)
		projectTarget(projected2D, target, calib, tgtMatch.pose, expandMarkerFoV);
		visualisePoints2D(projected2D, Color{ 0.0, 0.8, 0.2, 0.3f }, 2.0f);

		// Gather relevant points that were tracked
		relevantProjected2D.clear();
		relevantProjected2D.reserve(matches.size());
		for (auto match : matches)
			relevantProjected2D.push_back(match.first);

		// Visualise target points that were tracked this frame
		projectTarget(projected2D, target, calib, relevantProjected2D, tgtMatch.pose);
		visualisePoints2D(projected2D, Color{ 0.8, 0.0, 0.2, 0.6f }, 2.0f);

		// Show final accepted matches
		projectTarget(projected2D, target, calib, tgtMatch.pose);
		assert(projected2D.size() == target.markers.size());
		matchLines.clear();
		matchLines.reserve(matches.size());
		for (auto match : matches)
		{
			Eigen::Vector2f pt = frame.points2D[match.second];
			Eigen::Vector2f proj = projected2D[match.first];
			matchLines.push_back({
				{ pt.homogeneous(), Color(0.0f, 0.5f, 1.0f, 1.0f) },
				{ proj.homogeneous(), Color(0.0f, 0.5f, 1.0f, 1.0f) }
			});
		}
		visualiseLines(matchLines, 4.0f);
	}
	else if (trkVis.debugFocusStage > 0 && trkVis.debugFocusStage-1 < matchingData.numStages
		&& matchingData.stages[trkVis.debugFocusStage-1].identifier >= 0)
	{
		auto &targetMatch = matchingData.stages[trkVis.debugFocusStage-1];
	
		// Copy projected of that state
		projected2D.clear();
		for (int m = 0; m < targetMatch.markerCount; m++)
			projected2D.push_back(targetMatch.markers[m].projected);
		visualisePoints2D(projected2D, Color{ 0.0, 0.8, 0.2, 0.3f }, 2.0f);

		// Show all considered match pairs
		if (trkVis.debugFocusPoint >= 0 && trkVis.debugFocusPoint < targetMatch.markerCount)
		{
			if (!trkVis.onlyFocusPoint)
			{ // Also show other match lines in background
				matchLines.clear();
				for (int m = 0; m < targetMatch.markerCount; m++)
				{
					if (trkVis.debugFocusPoint != m)
						recordMatchCandidateLines(targetMatch.markers[m], targetMatch.markers[m].projected, targetMatch.lower, targetMatch.upper, 0.4f);
				}
				visualiseLines(matchLines, 1.0f);
			}

			// Show focused match lines in foreground
			const auto &markerMatch = targetMatch.markers[trkVis.debugFocusPoint];
			matchLines.clear();
			recordMatchCandidateLines(markerMatch, markerMatch.projected, targetMatch.lower, targetMatch.upper, 1.0f);
			visualiseLines(matchLines, 4.0f);
		}
		else 
		{ // Show all match lines, faintly
			matchLines.clear();
			for (int m = 0; m < targetMatch.markerCount; m++)
				recordMatchCandidateLines(targetMatch.markers[m], targetMatch.markers[m].projected, targetMatch.lower, targetMatch.upper, 0.6f);
			visualiseLines(matchLines, 1.0f);
		}

		// Add labels, depending on whether a match was accepted, validated in manual editing, or if neither, a discarded match of the focus point
		auto &editMatches = (trkVis.debug.showEdited? trkVis.debug.editedMatch2D : trkVis.debug.targetMatch2D).points2D[calib.index];
		for (int m = 0; m < targetMatch.markerCount; m++)
		{
			if (trkVis.onlyFocusPoint && m != trkVis.debugFocusPoint)
				continue; // Only show matches related to the focus point
			const auto &markerMatch = targetMatch.markers[m];
			for (const auto &match : markerMatch.matches)
			{
				bool valid = trkVis.debug.showEdited && std::find(editMatches.begin(), editMatches.end(), std::make_pair(m, match.index)) != editMatches.end();
				if (match.accepted || valid)
				{ // Match that has either already been accepted or deemed valid through manual editing
					priLabels.push_back(createLabel(markerMatch, markerMatch.projected, match, 
						Color{ 0.4f, valid? 1.0f : 0.4f, match.accepted? 1.0f : 0.4f, 1 }));
				}
				else if (trkVis.showAllLabels || trkVis.debugFocusPoint == m)
				{ // Secondary match that has neither been accepted nor deemed valid through manual editing
					secLabels.push_back(createLabel(markerMatch, markerMatch.projected, match,
						Color{ std::clamp(0.5f - 0.1f*std::log(match.value), 0.0f, 1.0f), 0.6f, 0.6f, 1 }));
				}
			}
		}
	}
}

void visualiseTarget2DUncertaintyAxis(UncertaintyAxisAlignment uncertaintyAxis)
{
	if (!uncertaintyAxis.rayDir.hasNaN())
	{
		Eigen::Vector2f pos1 = uncertaintyAxis.rayCam,
			pos2 = uncertaintyAxis.rayTgt,
			pos3 = pos1 + uncertaintyAxis.rayDir * 10;
		visualiseLines({
			{
				{ Eigen::Vector3f(pos1.x(), pos1.y(), 0.1f), Color{ 1.0f, 0.5f, 0.5f } },
				{ Eigen::Vector3f(pos3.x(), pos3.y(), 0.1f), Color{ 1.0f, 0.5f, 0.5f } }
			},
			{
				{ Eigen::Vector3f(pos1.x(), pos1.y(), 0.1f), Color{ 0.5f, 1.0f, 0.5f } },
				{ Eigen::Vector3f(pos2.x(), pos2.y(), 0.1f), Color{ 0.5f, 1.0f, 0.5f } }
			}
		});
		if (!std::isnan(uncertaintyAxis.projMin))
		{
			visualisePoints2D({
				pos1 + uncertaintyAxis.rayDir * uncertaintyAxis.projMin,
				pos1 + uncertaintyAxis.rayDir * uncertaintyAxis.projMax,
			}, Color{ 1.0f, 0.5f, 0.5f }, 2.0f);
		}
		if (!std::isnan(uncertaintyAxis.obsMin))
		{
			visualisePoints2D({
				pos1 + uncertaintyAxis.rayDir * uncertaintyAxis.obsMin,
				pos1 + uncertaintyAxis.rayDir * uncertaintyAxis.obsMax,
			}, Color{ 0.5f, 1.0f, 0.5f }, 2.0f);
		}
	}
	else if (!uncertaintyAxis.rayTgt.hasNaN())
	{
		visualisePointsSprites({
			{ Eigen::Vector3f(uncertaintyAxis.rayTgt.x(), uncertaintyAxis.rayTgt.y(), 0.1f), Color{ 0.5f, 1.0f, 0.5f } }
		});
	}
}