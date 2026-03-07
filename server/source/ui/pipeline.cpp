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

#include "ui.hpp"

#include "pipeline/pipeline.hpp"
#include "calib_target/assembly.hpp"

#include "ui/system/vis.hpp"
#include "point/sequences2D.hpp"
#include "point/sequence_data.inl"
#include "calib/camera_system.inl"

#include "util/debugging.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <numeric>


void InterfaceState::UpdatePipeline(InterfaceWindow &window)
{
	auto discardTargetSelection = []
	{
		if (GetUI().visState.target.inspectingSource == 'O')
			GetUI().visState.resetVisTarget();
	};
	if (!window.open)
	{
		discardTargetSelection();
		return;
	}
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		discardTargetSelection();
		ImGui::End();
		return;
	}
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	{ // Display information on calibration state
		ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0xDD, 0x88, 0x44, 0xFF));
		if (calibState.numUncalibrated)
		{
			ImGui::TextWrapped("%d / %d cameras are still uncalibrated",
				calibState.numUncalibrated, calibState.numUncalibrated+calibState.numCalibrated);
		}
		if (calibState.relUncertain)
		{
			ImGui::TextWrapped("%d / %d calibrated camera relations still need verification",
				calibState.relUncertain, calibState.relUncertain+calibState.relCertain);
			ImGui::SetItemTooltip("Simply wave some ball markers around so that cameras can build confidence in their calibration between each other.");
		}
		ImGui::PopStyleColor();

		if (calibState.relUncertain && ImGui::Button("Assume calibrations to be valid", SizeWidthFull()))
		{ // Skip manual verification (by providing samples) and instead write bogus values
			AssumeCalibrationsValid(pipeline, *pipeline.calibration.contextualLock());
			UpdateCalibrations();
		}
	}

	ImGui::BeginDisabled(state.mode == MODE_None);

	BeginSection("Phase");

	const PipelinePhase phaseMap[] = { PHASE_Idle, PHASE_Automatic, PHASE_Calibration_Point, PHASE_Calibration_Target, PHASE_Tracking };
	const char* phaseLabels = "Idle""\0"
		"Automatic""\0"
		"Camera Calibration""\0"
		"Target Calibration""\0"
		"Tracking""\0\0";

	int phaseSelection = pipeline.phase;
	ImGui::SetNextItemWidth(LineWidth() - ImGui::GetFrameHeight() - ImGui::GetStyle().ItemSpacing.x);
	if (ImGui::Combo("##Phase", &phaseSelection, phaseLabels))
	{
		pipeline.phase = phaseMap[phaseSelection];
	}

	ImGui::SameLine();
	if (CrossButton("Return##Phase"))
	{ // Return to automatic or idle
		pipeline.phase = PHASE_Idle;
	}
	EndSection();

	ImGui::EndDisabled();
	if (state.mode == MODE_None)
	{
		discardTargetSelection();
		ImGui::End();
		return;
	}

	ImVec2 ButtonSize = ImVec2(std::min(100.0f, SizeWidthDiv3().x), ImGui::GetFrameHeight());

	bool displayInternalDebug = (state.mode == MODE_Replay || state.mode == MODE_Simulation) && (state.simAdvance.load() == 0 || dbg_isBreaking);

	if (pipeline.phase == PHASE_Tracking)
	{
		ImGui::PushID("Trk");

		// Gather tracked targets from latest frame and update UI-local record of tracked targets
		OptFrameNum frameNum = pipeline.frameNum;
		VisFrameLock visFrame = visState.lockVisFrame(pipeline, true, -1, true);
		if (visFrame)
		{
			frameNum = visFrame.frameIt.index();
			auto &frameRecord = *visFrame.frameIt->get();
			for (auto &trackRecord : frameRecord.trackers)
			{
				auto findIt = visState.tracking.targets.find(trackRecord.id);
				auto &t = visState.tracking.targets[trackRecord.id];
				if (findIt == visState.tracking.targets.end())
				{ // Setup tracked target for the first time
					auto trackConfig = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
						[&](auto &t){ return t.id == trackRecord.id; });
					assert(trackConfig != state.trackerConfigs.end());
					t.label = trackConfig->label;
					t.imuSampleRate = StatFloatingf(1000);
				}
				if (trackRecord.result.isTracked())
					t.lastTrackedFrame = frameNum;
				t.trackState = trackRecord.result;
				t.imuState = trackRecord.imuState;
				t.imuSampleRate.update(std::round(1/trackRecord.imuSampleInterval));
				t.imuSampleAgo = dtS(trackRecord.imuLastSample, frameRecord.time);
			}
		}

		bool foundFocused = false;
		auto SelectableTracker = [&](int id, std::string label)
		{
			bool selected = visState.tracking.focusedTrackerID == id;
			if (ImGui::Selectable(label.c_str(), &selected,
				ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{
				visState.tracking.focusedTrackerID = selected? id : 0;
				visState.tracking.focusTrackingInsights = selected;
			}
			if (selected) foundFocused = true;
		};

		auto ShowIMUStatus = [&](const auto &tracker)
		{
			if (tracker.imuState == TrackerInertialState::NO_IMU)
				return;
			SameLineTrailing(iconSize().x + ImGui::GetStyle().ItemSpacing.x + ImGui::GetFrameHeight());
			ImGui::SetCursorPosY(ImGui::GetCursorPosY()+ImGui::GetStyle().FramePadding.y);
			if (tracker.imuState == TrackerInertialState::IMU_CALIBRATING)
			{
				ImGui::Image(icons().imu_calib, iconSize());
				ImGui::SetItemTooltip("Calibrating with %dHz sample rate", (int)tracker.imuSampleRate.floating);
			}
			else if (tracker.imuState == TrackerInertialState::IMU_TRACKING)
			{
				ImGui::Image(icons().imu_track, iconSize());
				ImGui::SetItemTooltip("Tracking with %dHz sample rate", (int)tracker.imuSampleRate.floating);
			}
			else if (tracker.imuState == TrackerInertialState::IMU_LOST)
			{
				ImGui::Image(icons().imu_lost, iconSize());
				ImGui::SetItemTooltip("No IMU samples received for %.1fs", tracker.imuSampleAgo);
			}
		};

		bool hasDormant = false, hasTracked = false;
		for (auto &tracker : state.trackerConfigs)
		{
			if (!tracker.triggered) continue;
			auto trackerRec = visState.tracking.targets.find(tracker.id);
			if (trackerRec == visState.tracking.targets.end()) { hasDormant = true; continue; }
			OptFrameNum trackedAgo = frameNum - trackerRec->second.lastTrackedFrame;
			if (trackedAgo >= 500) { hasDormant = true; continue; }
			hasTracked = true;

			ImGui::PushID(tracker.id);
			ImGui::AlignTextToFramePadding();

			std::string label;
			if (trackedAgo < 10)
				label = asprintf_s("Tracking '%s' (%d)###TgtTrk", tracker.label.c_str(), tracker.id);
			else
				label = asprintf_s("Lost '%s' (%d), %ld frames ago###TgtTrk", tracker.label.c_str(), tracker.id, trackedAgo);
			SelectableTracker(tracker.id, label);

			ShowIMUStatus(trackerRec->second);
			SameLineTrailing(ImGui::GetFrameHeight());
			if (ImGui::ArrowButton("Unset", ImGuiDir_Down))
				ServerUpdateTrackerConditions(state, tracker, false, -1, 0);
			ImGui::SetItemTooltip("Manually reset trigger to remove tracker from tracking.");
			ImGui::PopID();
		}

		if (hasDormant && hasTracked)
			ImGui::Separator();

		for (auto &tracker : state.trackerConfigs)
		{
			if (!tracker.triggered) continue;
			auto trackerRec = visState.tracking.targets.find(tracker.id);
			if (trackerRec != visState.tracking.targets.end())
			{ // Skip recently tracked trackers
				OptFrameNum trackedAgo = frameNum - trackerRec->second.lastTrackedFrame;
				if (trackedAgo < 500)
					continue;
			}
			ImGui::PushID(tracker.id);
			ImGui::AlignTextToFramePadding();
			SelectableTracker(tracker.id, asprintf_s("Dormant '%s' (%d)", tracker.label.c_str(), tracker.id));
			if (trackerRec != visState.tracking.targets.end())
				ShowIMUStatus(trackerRec->second);
			SameLineTrailing(ImGui::GetFrameHeight());
			if (ImGui::ArrowButton("Unset", ImGuiDir_Down))
				ServerUpdateTrackerConditions(state, tracker, false, -1, 0);
			ImGui::SetItemTooltip("Manually reset trigger to remove tracker from tracking.");
			ImGui::PopID();
		}

		if (!foundFocused)
			visState.tracking.focusedTrackerID = 0;

		if (ImGui::CollapsingHeader("Unused Trackers"))
		{
			bool hasUnused = false;
			for (auto &tracker : state.trackerConfigs)
			{
				if (tracker.triggered) continue;
				hasUnused = true;
				ImGui::PushID(tracker.id);
				ImGui::AlignTextToFramePadding();
				ImGui::Text("'%s' (%d)", tracker.label.c_str(), tracker.id);
				SameLineTrailing(ImGui::GetFrameHeight());
				if (ImGui::ArrowButton("Trigger", ImGuiDir_Up))
					ServerUpdateTrackerConditions(state, tracker, false, 1, 0);
				ImGui::SetItemTooltip("Manually trigger tracker to consider it for tracking.");
				ImGui::PopID();
			}
			if (!hasUnused)
			{
				ImGui::TextWrapped("No unused trackers, all are expected to be in the tracking volume.\n"
					"You may set trigger conditions based on cues that they are in use.");
				if (ImGui::Button("Open Tracker Configuration", SizeWidthFull()))
					windows[WIN_TRACKERS].open = true;
			}
		}
		ImGui::PopID();
		ImGui::Dummy(ImVec2(0, 4));

		if (displayInternalDebug && visFrame && visState.tracking.focusedTrackerID != 0)
		{
			// Temporary debug tools, data, etc.

			auto &frameRecord = *visFrame.frameIt->get();
			auto trackRecord = std::find_if(frameRecord.trackers.begin(), frameRecord.trackers.end(),
						[&](auto &t){ return t.id == visState.tracking.focusedTrackerID; });
			auto trackConfig = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
						[&](auto &t){ return t.id == visState.tracking.focusedTrackerID; });
			if (trackRecord != frameRecord.trackers.end() && trackConfig != state.trackerConfigs.end()
				&& trackConfig->type == TrackerConfig::TRACKER_TARGET
				&& BeginCollapsingRegion("Target Tracking Debug"))
			{
				auto &debugVis = visState.tracking.debug;

				std::vector<std::vector<Eigen::Vector2f> const *> points2D(pipeline.cameras.size());
				std::vector<std::vector<BlobProperty> const *> properties(pipeline.cameras.size());
				std::vector<std::vector<int>> remainingPoints2D(pipeline.cameras.size());
				std::vector<std::vector<int> const *> relevantPoints2D(pipeline.cameras.size());
				for (int c = 0; c < pipeline.cameras.size(); c++)
				{
					points2D[c] = &frameRecord.cameras[c].points2D;
					properties[c] = &frameRecord.cameras[c].properties;
					relevantPoints2D[c] = &remainingPoints2D[c];
					// Consider all points for tracking, as would the original target tracking
					remainingPoints2D[c].resize(points2D[c]->size());
					std::iota(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), 0);
				}

				if (ImGui::Button("Debug target matching", SizeWidthFull()))
				{ // Manually trigger update - might also be automatic
					debugVis.needsUpdate = true;
				}
				if (debugVis.needsUpdate)
				{
					debugVis.needsUpdate = false;
					debugVis.showInitial = trackRecord->match2D.error.samples > 0 && trackRecord->result.isProbe();
					debugVis.initialMatch2D = trackRecord->match2D;
					debugVis.internalData.init(pipeline.cameras.size());
					debugVis.targetMatch2D = trackTarget2D(trackConfig->calib,
						trackRecord->posePredicted, trackRecord->covPredicted,
						pipeline.getCalibs(), pipeline.cameras.size(),
						points2D, properties, relevantPoints2D, pipeline.params.track, debugVis.internalData);
					debugVis.editedMatch2D = debugVis.targetMatch2D;
					debugVis.trackerID = visState.tracking.focusedTrackerID;
					debugVis.calib = &trackConfig->calib;
					debugVis.frameNum = frameNum;
				}

				if (debugVis.frameNum == frameNum)
				{
					ImGui::Text("Original tracking error: %.2fpx +- %.2fpx, %.2fpx max, %d matches",
						trackRecord->error.mean*PixelFactor, trackRecord->error.stdDev*PixelFactor, trackRecord->error.max*PixelFactor, trackRecord->error.samples);
					ImGui::Text("Redone tracking error: %.2fpx +- %.2fpx, %.2fpx max, %d matches",
						debugVis.targetMatch2D.error.mean*PixelFactor, debugVis.targetMatch2D.error.stdDev*PixelFactor, debugVis.targetMatch2D.error.max*PixelFactor, debugVis.targetMatch2D.error.samples);
					ImGui::Text("Edited tracking error: %.2fpx +- %.2fpx, %.2fpx max, %d matches",
						debugVis.editedMatch2D.error.mean*PixelFactor, debugVis.editedMatch2D.error.stdDev*PixelFactor, debugVis.editedMatch2D.error.max*PixelFactor, debugVis.editedMatch2D.error.samples);

					if (trackRecord->result.isProbe())
						ImGui::TextUnformatted("Detection attempt!");
					ImGui::BeginDisabled(debugVis.showEdited);
					ImGui::Checkbox(debugVis.showInitial?
						"Showing Initial Matching###DetectToggle" :
						"Showing Redone Matching###DetectToggle",
						&debugVis.showInitial);
					ImGui::EndDisabled();

					if (ImGui::Button(debugVis.showEdited? "Hide" : "Edit", SizeWidthDiv4()))
					{
						debugVis.showEdited = !debugVis.showEdited;
						debugVis.showEditTools = false;
					}
					ImGui::SameLine();
					ImGui::BeginDisabled(!debugVis.showEdited);
					if (ImGui::Button("Matches", SizeWidthDiv4()))
					{
						debugVis.showEditTools = !debugVis.showEditTools;
					}
					ImGui::SameLine();
					if (ImGui::Button("Optimise", SizeWidthDiv4()))
					{
						debugVis.editedMatch2D.error = optimiseTargetPose<true>(pipeline.getCalibs(), points2D, 
							debugVis.editedMatch2D, *debugVis.calib, trackRecord->posePredicted, pipeline.params.track.opt);
					}
					ImGui::SameLine();
					if (ImGui::Button("Reset", SizeWidthDiv4()))
					{
						debugVis.editedMatch2D = debugVis.targetMatch2D;
					}
					ImGui::EndDisabled();
				}

				EndCollapsingRegion();
			}
		}
	}

	if (pipeline.phase == PHASE_Tracking)
	{
		if (BeginCollapsingRegion("Optimisation Database"))
		{
			auto getTargetCalib = [&](ObsTarget &target, TargetCalibration3D &calib) -> TargetCalibration3D&
			{
				auto track = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
					[&](auto &t){ return t.id == target.trackerID; });
				if (track == state.trackerConfigs.end())
				{ // Not used right now since all targets in obsDatabase should already be fully calibrated
					calib = initialiseTargetCalib(pipeline.getCalibs(), target, pipeline.targetCalib.params.post);
					return calib;
				}
				return track->calib;
			};

			auto db_lock = pipeline.obsDatabase.lock();
			{
				ImGui::AlignTextToFramePadding();
				ImGui::Text("%d points, %d samples (%d outliers)", (int)db_lock->points.points.size(), db_lock->points.totalSamples, db_lock->points.outlierSamples);
				// Button to delete
				SameLineTrailing(ImGui::GetFrameHeight());
				if (CrossButton("Del"))
					db_lock->points = {};
			}
			for (auto tgtIt = db_lock->targets.begin(); tgtIt != db_lock->targets.end();)
			{
				auto tgt = *tgtIt;
				ImGui::PushID(tgt.trackerID);
				std::string label = asprintf_s("Target %d: %d frames, %d samples, %d outliers###TgtCont",
					tgt.trackerID, (int)tgt.frames.size(), tgt.totalSamples, tgt.outlierSamples);
				bool selected = visState.target.inspectingTrackerID == tgt.trackerID;
				ImGui::AlignTextToFramePadding();
				if (ImGui::Selectable(label.c_str(), &selected, ImGuiSelectableFlags_AllowOverlap))
				{
					if (visState.resetVisTarget(false) && selected)
					{ // Set new selected target ID
						visState.target.inspectingTargetCalib = {};
						getTargetCalib(tgt, visState.target.inspectingTargetCalib);
						visState.target.inspectingTrackerID = tgt.trackerID;
						visState.target.inspectingSource = 'O';
						auto unlock = db_lock.scopedUnlock(); // for lockVisTarget
						visState.updateVisTarget();
					}
				}
				// Button to delete
				SameLineTrailing(ImGui::GetFrameHeight());
				bool del = CrossButton("Del");
				if (del)
				{
					if (visState.target.inspectingTrackerID == tgt.trackerID)
						visState.resetVisTarget(false);
					tgtIt = db_lock->targets.erase(tgtIt);
				}
				else tgtIt++;
				ImGui::PopID();
			}

			auto &ptCalib = pipeline.pointCalib;
			auto &tgtCalib = pipeline.targetCalib;
			ImGui::BeginDisabled(ptCalib.control && ptCalib.control->running());
			if (ImGui::Button("Optimise Cameras", SizeWidthDiv2()))
			{
				ptCalib.settings.typeFlags = 0b1000;
				ptCalib.settings.maxSteps = ptCalib.state->numSteps + 10;
				ptCalib.planned = true;
			}
			ImGui::EndDisabled();
			ImGui::SameLine();
			ImGui::BeginDisabled(visState.target.inspectingTrackerID == 0 || tgtCalib.optPlannedForTargetID != 0);
			if (ImGui::Button("Optimise Target", SizeWidthDiv2()))
			{
				auto tgtOptLock = tgtCalib.targetOptimisations.contextualRLock();
				auto trackerIt = std::find_if(tgtOptLock->begin(), tgtOptLock->end(),
					[&](auto &t){ return t->targetID == visState.target.inspectingTrackerID; });
				if (trackerIt == tgtOptLock->end())
					tgtCalib.optPlannedForTargetID = visState.target.inspectingTrackerID;
				else
					SignalErrorToUser("Already optimising the selected target!");
			}
			ImGui::EndDisabled();
 
			ImGui::BeginDisabled(visState.target.inspectingTrackerID == 0);

			if (ImGui::Button("Edit Target", SizeWidthDiv2()))
			{
				auto trackerIt = std::find_if(db_lock->targets.begin(), db_lock->targets.end(),
					[&](auto &t){ return t.trackerID == visState.target.inspectingTrackerID; });
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock(); 
				if (trackerIt == db_lock->targets.end())
					SignalErrorToUser("Selected tracker not found in target database!");
				else if (!stages_lock->empty())
					SignalErrorToUser("Target Calibration already has stages from prior calibration.");
					// TODO: Could ask user whether to Cancel, Clear, or Append
				else
				{
					auto &data = *trackerIt;
					LOG(LGUI, LInfo, "Setting tracking data as stage with %d frames, %d markers, %d sequences",
						(int)data.frames.size(), (int)data.markers.size(), (int)data.markerMap.size());
					TargetAssemblyBase base = {};
					base.initialViewID = -1;
					base.assignedTrackerID = data.trackerID;
					base.target = data; // Copy
					base.errors = getTargetErrorDist(pipeline.getCalibs(), base.target);
					base.targetCalib = getTargetCalib(base.target, base.targetCalib);
					stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(base), STAGE_LOADED, 1, "Edit Tracked"));
					visState.targetCalib.stage = stages_lock->back();
					pipeline.targetCalib.assignedTrackerID = data.trackerID;
					pipeline.phase = PHASE_Calibration_Target;
				}
			}

			ImGui::SameLine();

			if (ImGui::Button("Update View Cones", SizeWidthDiv2()))
			{
				auto trackerIt = std::find_if(db_lock->targets.begin(), db_lock->targets.end(),
					[&](auto &t){ return t.trackerID == visState.target.inspectingTrackerID; });
				auto track = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
					[&](auto &t){ return t.id == visState.target.inspectingTrackerID; });
				if (trackerIt == db_lock->targets.end())
					SignalErrorToUser("Selected tracker not found in target database!");
				else if (track == state.trackerConfigs.end())
					SignalErrorToUser("Selected tracker not found in trackers!");
				else threadPool.push([&](int, int id, TargetCalibration3D calib, ObsTarget obsTarget)
				{
					updateMarkerViewAngles(calib.markers, pipeline.getCalibs(), obsTarget, pipeline.targetCalib.params.post);
					SignalTargetCalibUpdate(id, calib);
				}, track->id, track->calib, *trackerIt);
			}
			ImGui::SetItemTooltip("Automatically recalculate view cones using new tracking samples (use as many as possible).");

			ImGui::EndDisabled();

			if (SaveButton("Save Cameras", SizeWidthDiv2(), state.cameraCalibsDirty))
			{
				auto error = storeCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
				if (error) SignalErrorToUser(error.value());
				else state.cameraCalibsDirty = false;
			}
			ImGui::SameLine();
			if (SaveButton("Save Targets", SizeWidthDiv2(), state.trackerConfigDirty || state.trackerCalibsDirty || state.trackerIMUsDirty))
			{
				auto error = storeTrackerConfigurations("store/trackers.json", state.trackerConfigs);
				if (error) SignalErrorToUser(error.value());
				else state.trackerConfigDirty = state.trackerCalibsDirty = state.trackerIMUsDirty = false;
			}

			if (ptCalib.control && ptCalib.control->running())
			{ // Shared with main point calibration
				if (ImGui::Button(ptCalib.control->stopping()? "Stopping..." : "Stop", ButtonSize))
				{
					ptCalib.control->stop_source.request_stop();
				}
				ImGui::SameLine();
				if (ptCalib.settings.typeFlags & 0b0001)
				{
					ImGui::TextUnformatted("Reconstructing Camera Geometry...");
				}
				else if (ptCalib.settings.typeFlags & 0b0010)
				{
					ImGui::Text("Optimising Cameras %d/%d...", ptCalib.state->numSteps, ptCalib.settings.maxSteps);
				}
				else if (ptCalib.settings.typeFlags & 0b1000)
				{ // Target-specific, started from tracking database optimisation
					ImGui::Text("Optimising Cameras %d/%d...", ptCalib.state->numSteps, ptCalib.settings.maxSteps);
				}
			}
			{
				auto tgtOptLock = tgtCalib.targetOptimisations.contextualRLock();
				for (auto &tgt : *tgtOptLock)
				{
					ImGui::PushID(tgt->targetID);
					if (ImGui::Button(tgt->control.stopping()? "Stopping..." : "Stop", ButtonSize))
						tgt->control.stop_source.request_stop();
					ImGui::SameLine();
					auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
						[&](auto &t){ return t.id == tgt->targetID; });
					if (trackerIt == state.trackerConfigs.end()) continue;
					ImGui::Text("Optimising '%s' %d/%d...", trackerIt->label.c_str(), tgt->numSteps, tgt->maxSteps);
					ImGui::PopID();
				}
			}

			EndCollapsingRegion();
		}
		else
			discardTargetSelection();
	}	

	if (pipeline.phase == PHASE_Tracking && state.mode == MODE_Replay)
	{
		if (BeginCollapsingRegion("Tracking Results"))
		{
			if (ImGui::Button("Update on disk", SizeWidthDiv3()))
			{
				assert(state.recording.segments.size() == state.recording.tracking.size());
				for (int i = 0; i < state.recording.segments.size(); i++)
				{
					auto &segment = state.recording.segments[i];
					if (segment.frameCount == 0) continue; // Invalid or missing segment
					if (state.recording.tracking[i].empty())
						state.recording.tracking[i] = state.recording.captures[i];
					auto error = dumpTrackingResults(state.recording.tracking[i], pipeline.record,
						segment.frameStart, segment.frameStart+segment.frameCount, segment.frameOffset);
					if (error) SignalErrorToUser(error.value());
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Load from disk", SizeWidthDiv3()))
			{
				assert(state.recording.segments.size() == state.recording.tracking.size());
				for (int i = 0; i < state.recording.segments.size(); i++)
				{
					auto &segment = state.recording.segments[i];
					if (segment.frameCount == 0) continue; // Invalid or missing segment
					if (state.recording.tracking[i].empty())
						state.recording.tracking[i] = state.recording.captures[i];
					auto error = parseTrackingResults(state.recording.tracking[i], state.stored, segment.frameOffset);
					if (error) SignalErrorToUser(error.value());
				}
			}
			ImGui::SameLine();
			if (ImGui::Button("Set temporarily", SizeWidthDiv3()))
			{
				auto framesRecord = pipeline.record.frames.getView();
				auto framesStored = state.stored.frames.getView();
				for (auto &frameStored : framesStored)
				{
					if (!frameStored) continue;
					frameStored->trackers.clear();
					frameStored->triangulations.clear();
				}
				for (const auto &frameRecord : framesRecord)
				{
					if (!frameRecord) continue;
					if (framesStored.size() <= frameRecord->num) break;
					framesStored[frameRecord->num]->trackers = frameRecord->trackers;
					framesStored[frameRecord->num]->triangulations = frameRecord->triangulations;
				}
			}

			// Tracking quality assessment;
			struct EventChange
			{
				uint32_t loaded, current, added, removed;
			};
			static EventChange losses, search2D, detect2D, detect3D, tracked;
			struct TrackingSamples
			{
				std::string label;
				EventChange tracked;
				StatValue<float,StatExtremas|StatDistribution> samplesLoaded, samplesCurrent, samplesAdded, samplesRemoved;
				StatValue<float,StatExtremas|StatDistribution> errorLoaded, errorCurrent, errorAdded, errorRemoved;
				StatValue<float,StatDistribution> trackTimeLoaded, trackTimeCurrent;
			};
			static std::map<int, TrackingSamples> trackers;
			static uint32_t coveredFrames; // All following code relies on frame number
			if (ImGui::Button("Update Tracking Results", SizeWidthFull()))
			{
				auto countEvents = [](const FrameRecord &record, TrackingResult::State result)
				{
					return std::count_if(record.trackers.begin(), record.trackers.end(),
						[&](auto &t){ return t.result.isState(result); });
				};
				auto updateEventChange = [](EventChange &event, uint32_t loaded, uint32_t current)
				{
					event.loaded += loaded;
					event.current += current;
					event.added += current > loaded? current - loaded : 0;
					event.removed += loaded > current? loaded - current : 0;
				};
				losses = {};
				search2D = {};
				detect2D = {};
				detect3D = {};
				tracked = {};
				trackers.clear();
				coveredFrames = 0;
				auto framesRecord = pipeline.record.frames.getView<true>();
				auto framesStored = state.stored.frames.getView<true>();
				for (const auto &frameRecord : framesRecord)
				{
					if (!frameRecord || !frameRecord->finishedProcessing) continue;
					if (framesStored.size() <= frameRecord->num) break;
					coveredFrames++;
					auto &stored = *framesStored[frameRecord->num];
					auto &record = *frameRecord;
					// Update changes to all tracking events
					updateEventChange(losses, countEvents(stored, TrackingResult::NO_TRACK), countEvents(record, TrackingResult::NO_TRACK));
					updateEventChange(search2D, countEvents(stored, TrackingResult::SEARCHED_2D), countEvents(record, TrackingResult::SEARCHED_2D));
					updateEventChange(detect2D, countEvents(stored, TrackingResult::DETECTED_S2D), countEvents(record, TrackingResult::DETECTED_S2D));
					updateEventChange(detect2D, countEvents(stored, TrackingResult::DETECTED_P2D), countEvents(record, TrackingResult::DETECTED_P2D));
					updateEventChange(detect3D, countEvents(stored, TrackingResult::DETECTED_M3D), countEvents(record, TrackingResult::DETECTED_M3D));
					updateEventChange(tracked, stored.trackers.size(), record.trackers.size());
					// Accumulate results for each target for this frame from loaded and current results
					struct FrameTrackers {
						uint32_t samplesLoaded = 0, samplesCurrent = 0;
						float errorLoaded = 0, errorCurrent = 0;
						float timeLoaded = 0.0f, timeCurrent = 0.0f;
					};
					std::map<int, FrameTrackers> frameTrackers;
					for (auto &trackRecord : record.trackers)
					{
						auto &trkFrame = frameTrackers[trackRecord.id];
						trkFrame.samplesCurrent = trackRecord.error.samples;
						trkFrame.errorCurrent = trackRecord.error.mean;
						if (trackRecord.result.isTracked())
							trkFrame.timeCurrent = trackRecord.procTimeMS;
					}
					for (auto &trackStored : stored.trackers)
					{
						auto &trkFrame = frameTrackers[trackStored.id];
						trkFrame.samplesLoaded = trackStored.error.samples;
						trkFrame.errorLoaded = trackStored.error.mean;
						if (trackStored.result.isTracked())
							trkFrame.timeLoaded = trackStored.procTimeMS;
					}
					// Update changes to each targets tracking results
					for (auto &trk : frameTrackers)
					{
						auto &tracker = trackers[trk.first];
						auto &trkFrame = trk.second;
						updateEventChange(tracker.tracked, std::min(1u, trkFrame.samplesLoaded), std::min(1u, trkFrame.samplesCurrent));
						if (trkFrame.samplesLoaded)
						{
							tracker.samplesLoaded.update(trkFrame.samplesLoaded);
							tracker.errorLoaded.update(trkFrame.errorLoaded);
							tracker.trackTimeLoaded.update(trkFrame.timeLoaded);
						}
						if (trkFrame.samplesCurrent)
						{
							tracker.samplesCurrent.update(trkFrame.samplesCurrent);
							tracker.errorCurrent.update(trkFrame.errorCurrent);
							tracker.trackTimeCurrent.update(trkFrame.timeCurrent);
						}
						if (trkFrame.samplesCurrent > trkFrame.samplesLoaded)
							tracker.samplesAdded.update(trkFrame.samplesCurrent-trkFrame.samplesLoaded);
						if (trkFrame.samplesCurrent < trkFrame.samplesLoaded)
							tracker.samplesRemoved.update(trkFrame.samplesLoaded-trkFrame.samplesCurrent);
						if (trkFrame.samplesLoaded && trkFrame.samplesCurrent)
						{
							if (trkFrame.errorCurrent > trkFrame.errorLoaded)
								tracker.errorAdded.update(trkFrame.errorCurrent-trkFrame.errorLoaded);
							if (trkFrame.errorCurrent < trkFrame.errorLoaded)
								tracker.errorRemoved.update(trkFrame.errorLoaded-trkFrame.errorCurrent);
						}
					}
				}
				for (auto &tracker: state.trackerConfigs)
				{
					auto it = trackers.find(tracker.id);
					if (it != trackers.end())
						it->second.label = tracker.label;
				}
			}
			if (!trackers.empty() && ImGui::TreeNode("Target Results"))
			{
				ImGui::Text("Tracking Losses: %d  [%d]  +%d -%d", losses.current, losses.loaded, losses.added, losses.removed);
				ImGui::Text("2D Searches: %d  [%d]  +%d -%d", search2D.current, search2D.loaded, search2D.added, search2D.removed);
				ImGui::Text("2D Detections: %d  [%d]  +%d -%d", detect2D.current, detect2D.loaded, detect2D.added, detect2D.removed);
				ImGui::Text("3D Detections: %d  [%d]  +%d -%d", detect3D.current, detect3D.loaded, detect3D.added, detect3D.removed);
				ImGui::Text("Tracked Tgt/Frame: %d  [%d]  +%d -%d", tracked.current, tracked.loaded, tracked.added, tracked.removed);

				for (auto &trackedTarget : trackers)
				{
					ImGui::PushID(trackedTarget.first);
					if (ImGui::TreeNode(trackedTarget.second.label.c_str()))
					{
						auto &tgt = trackedTarget.second;
						ImGui::Text("Frames: %u  [%u]  +%u -%u", tgt.tracked.current, tgt.tracked.loaded, tgt.tracked.added, tgt.tracked.removed);
						ImGui::Text("Times: %.2fms +- %.2f  [%.2fms +- %.2f]", tgt.trackTimeCurrent.avg, tgt.trackTimeCurrent.stdDev()*2, tgt.trackTimeLoaded.avg, tgt.trackTimeLoaded.stdDev()*2);
						ImGui::Text("Samples:");
						ImGui::Text("    sum: %u  [%u]  +%u -%u", (uint32_t)tgt.samplesCurrent.sum, (uint32_t)tgt.samplesLoaded.sum, (uint32_t)tgt.samplesAdded.sum, (uint32_t)tgt.samplesRemoved.sum);
						ImGui::Text("     avg: %.2f +- %.2f  [%.2f +- %.2f]", tgt.samplesCurrent.avg, tgt.samplesCurrent.stdDev()*2, tgt.samplesLoaded.avg, tgt.samplesLoaded.stdDev()*2);
						ImGui::Text("	 diff: +%.2f (%u) -%.2f (%u), %u unchanged", tgt.samplesAdded.avg, tgt.samplesAdded.num, tgt.samplesRemoved.avg, tgt.samplesRemoved.num, coveredFrames-(tgt.samplesAdded.num+tgt.samplesRemoved.num));
						ImGui::Text("      >/<: %u/%u  [%u/%u]", (uint32_t)tgt.samplesCurrent.min, (uint32_t)tgt.samplesCurrent.max, (uint32_t)tgt.samplesLoaded.min, (uint32_t)tgt.samplesLoaded.max);
						ImGui::Text("Errors :");
						ImGui::Text("     avg: %.2fpx +- %.2fpx  [%.2fpx +- %.2fpx]", tgt.errorCurrent.avg*PixelFactor, tgt.errorCurrent.stdDev()*2*PixelFactor, tgt.errorLoaded.avg*PixelFactor, tgt.errorLoaded.stdDev()*2*PixelFactor);
						ImGui::Text("	 diff: +%.2fpx (%u) -%.2fpx (%u), %u unchanged", tgt.errorAdded.avg*PixelFactor, tgt.errorAdded.num, tgt.errorRemoved.avg*PixelFactor, tgt.errorRemoved.num, coveredFrames-(tgt.errorAdded.num+tgt.errorRemoved.num));
						ImGui::Text("      >/<: %.3fpx/%.2fpx  [%.3fpx/%.2fpx]", tgt.errorCurrent.min*PixelFactor, tgt.errorCurrent.max*PixelFactor, tgt.errorLoaded.min*PixelFactor, tgt.errorLoaded.max*PixelFactor);
						ImGui::TreePop();
					}
					ImGui::PopID();
				}
				ImGui::TreePop();
			}

			EndCollapsingRegion();
		}
	}
	else if (pipeline.phase == PHASE_Calibration_Point)
	{
		discardTargetSelection();
		UpdatePipelineCalibSection();
		UpdatePipelineObservationSection();
		UpdatePipelinePointCalib();
	}
	else if (pipeline.phase == PHASE_Calibration_Target)
	{
		discardTargetSelection();
		UpdatePipelineTargetCalib();
	}
	else
		discardTargetSelection();

	ImGui::End();
}