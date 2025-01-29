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
#include "point/sequence_data.inl"
#include "calib/camera_system.inl"

#include "util/debugging.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <numeric>


void InterfaceState::UpdatePipeline(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
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
			{
				auto lastFrame = pipeline.seqDatabase.contextualRLock()->lastRecordedFrame;
				auto calib_lock = pipeline.calibration.contextualLock();
				for (int i = 1; i < pipeline.cameras.size(); i++)
				{
					if (pipeline.cameras[i]->calib.invalid())
						continue;
					for (int j = 0; j < i; j++)
					{
						if (pipeline.cameras[j]->calib.invalid())
							continue;
						auto &FMcand = calib_lock->relations.getFMEntry(i, j);
						if (FMcand.candidates.empty())
						{
							FMcand.candidates.push_back({});
							FMcand.candidates.front().matrix = calculateFundamentalMatrix<float>(pipeline.cameras[i]->calib, pipeline.cameras[j]->calib);
							FMcand.candidates.front().precalculated = true;
						}
						auto &FM = FMcand.getBestCandidate();
						FM.floatingTrust = 6000.0f;
						FM.lastTrustUpdate = lastFrame;
						FM.lastCalculation = lastFrame;
						FM.stats.num = 6000;
						FM.stats.avg = 0.01f;
						FM.stats.M2 = 0.02f*0.02f*FM.stats.num;
					}
				}
			}
			UpdateCalibrations(false);
		}
	}

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

	ImVec2 ButtonSize = ImVec2(std::min(100.0f, SizeWidthDiv3().x), ImGui::GetFrameHeight());

	bool displayInternalDebug = (state.mode == MODE_Replay || state.mode == MODE_Simulation) && (state.simAdvanceCount.load() == 0 || dbg_isBreaking);

	if (pipeline.phase == PHASE_Tracking)
	{
		BeginSection("Tracking (?)");
		ImGui::SetItemTooltip("Any calibrated target will be detected and tracked automatically.\n");

		// Gather tracked targets from latest frame and update UI-local record of tracked targets
		long frameNum = pipeline.frameNum;
		VisFrameLock visFrame = visState.lockVisFrame(pipeline, true);
		if (visFrame)
		{
			frameNum = visFrame.frameIt.index();
			auto &tracking = visFrame.frameIt->get()->tracking;
			for (auto &tracked : tracking.targets)
			{
				auto &t = visState.tracking.targets[tracked.id];
				if (!t.first)
				{
					auto target = std::find_if(pipeline.tracking.targetTemplates3D.begin(), pipeline.tracking.targetTemplates3D.end(),
						[&](auto &t){ return t.id == tracked.id; });
					assert(target != pipeline.tracking.targetTemplates3D.end());
					t.first = &*target;
				}
				t.second = frameNum;
			}
		}

		// Display list of trackable targets
		for (auto &target : visState.tracking.targets)
		{
			ImGui::PushID(target.first);
			long ago = frameNum - target.second.second;
			TargetTemplate3D const *tgtTemplate = target.second.first;
			std::string label;
			if (ago < 5)
				label = asprintf_s("Tracking '%s' (%d)###TgtTrk", tgtTemplate->label.c_str(), target.first);
			else if (ago < 1000)
				label = asprintf_s("Lost '%s' (%d), %ld frames ago###TgtTrk", tgtTemplate->label.c_str(), target.first, ago);
			else
				label = asprintf_s("Dormant '%s' (%d)###TgtTrk", tgtTemplate->label.c_str(), target.first);
			if (ImGui::Selectable(label.c_str(), visState.tracking.focusedTargetID == target.first, ImGuiSelectableFlags_SpanAllColumns))
				visState.tracking.focusedTargetID = target.first;
			ImGui::PopID();
		}

		EndSection();

		if (displayInternalDebug && visFrame && visState.tracking.focusedTargetID != 0 &&
			visState.tracking.targets[visState.tracking.focusedTargetID].second == frameNum)
		{
			BeginSection("Tracking Debug");

			// Temporary debug tools, data, etc.

			auto &frame = *visFrame.frameIt->get();
			auto tracked = std::find_if(frame.tracking.targets.begin(), frame.tracking.targets.end(),
						[&](auto &t){ return t.id == visState.tracking.focusedTargetID; });
			assert(tracked != frame.tracking.targets.end());
			auto &target = *visState.tracking.targets[visState.tracking.focusedTargetID].first;
			auto &debugVis = visState.tracking.debug;

			std::vector<std::vector<Eigen::Vector2f> const *> points2D(pipeline.cameras.size());
			std::vector<std::vector<BlobProperty> const *> properties(pipeline.cameras.size());
			std::vector<std::vector<int>> remainingPoints2D(pipeline.cameras.size());
			std::vector<std::vector<int> const *> relevantPoints2D(pipeline.cameras.size());
			for (int c = 0; c < pipeline.cameras.size(); c++)
			{
				points2D[c] = &frame.cameras[c].points2D;
				properties[c] = &frame.cameras[c].properties;
				remainingPoints2D[c].resize(points2D[c]->size());
				std::iota(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), 0);
				relevantPoints2D[c] = &remainingPoints2D[c];
			}

			if (ImGui::Button("Redo tracking for this frame", SizeWidthFull()) || debugVis.needsUpdate)
			{
				debugVis.needsUpdate = false;
				visState.tracking.retrackData.init(pipeline.cameras.size());
				debugVis.targetMatch2D = trackTarget2D(target,
					tracked->prediction, tracked->predStdDev,
					pipeline.getCalibs(), pipeline.cameras.size(),
					points2D, properties, relevantPoints2D, pipeline.params.track, visState.tracking.retrackData);
				debugVis.editedMatch2D = debugVis.targetMatch2D;
				debugVis.trackerID = visState.tracking.focusedTargetID;
				debugVis.frameNum = frameNum;
			}

			if (debugVis.frameNum == frameNum)
			{
				ImGui::Text("Original tracking error: %.2fpx +- %.2fpx, %.2fpx max",
					tracked->error.mean*PixelFactor, tracked->error.stdDev*PixelFactor, tracked->error.max*PixelFactor);
				ImGui::Text("Redone tracking error: %.2fpx +- %.2fpx, %.2fpx max",
					debugVis.targetMatch2D.error.mean*PixelFactor, debugVis.targetMatch2D.error.stdDev*PixelFactor, debugVis.targetMatch2D.error.max*PixelFactor);
				ImGui::Text("Edited tracking error: %.2fpx +- %.2fpx, %.2fpx max",
					debugVis.editedMatch2D.error.mean*PixelFactor, debugVis.editedMatch2D.error.stdDev*PixelFactor, debugVis.editedMatch2D.error.max*PixelFactor);

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
						debugVis.editedMatch2D, pipeline.params.track.opt);
				}
				ImGui::SameLine();
				if (ImGui::Button("Reset", SizeWidthDiv4()))
				{
					debugVis.editedMatch2D = debugVis.targetMatch2D;
				}
				ImGui::EndDisabled();
			}

			EndSection();
		}
	}

	if (pipeline.phase == PHASE_Tracking)
	{
		BeginSection("Settings");

		if (ImGui::TreeNode("Detection Method"))
		{
			CheckboxInput("3D Detection in Triangulations", &pipeline.params.detect.enable3D);
			CheckboxInput("2D Brute-Force Searching Async", &pipeline.params.detect.enable2DAsync);
			CheckboxInput("2D Brute-Force Searching Sync (!)", &pipeline.params.detect.enable2DSync);
			ImGui::TreePop();
		}
		// TODO: Can we make filtering methods configurable? Currently selected at compile time in tracking3D.hpp
		// Maybe make TrackedTarget::filter object an opaque pointer, and have a few method calls abstracted away
		// Then predPose and pose should be accessible in TrackedTarget directly

		EndSection();

		BeginSection("Optimisation Database");
		{
			auto db_lock = pipeline.obsDatabase.lock();
			ImGui::Text("%d points, %d samples (%d outliers)", (int)db_lock->points.points.size(), db_lock->points.totalSamples, db_lock->points.outlierSamples);
			for (auto tgtIt = db_lock->targets.begin(); tgtIt != db_lock->targets.end(); tgtIt++)
			{
				auto tgt = *tgtIt;
				ImGui::PushID(tgt.targetID);
				std::string label = asprintf_s("Target %d: %d frames, %d samples, %d outliers###TgtCont",
					tgt.targetID, (int)tgt.frames.size(), tgt.totalSamples, tgt.outlierSamples);
				bool selected = visState.targetCalib.contCalibTargetID == tgt.targetID;
				if (ImGui::Selectable(label.c_str(), visState.targetCalib.contCalibTargetID == tgt.targetID, ImGuiSelectableFlags_AllowOverlap))
				{
					if (visState.targetCalib.contCalibTargetID != tgt.targetID)
					{
						auto track = std::find_if(pipeline.tracking.targetTemplates3D.begin(), pipeline.tracking.targetTemplates3D.end(),
							[&](auto &t){ return t.id == tgt.targetID; });
						if (track != pipeline.tracking.targetTemplates3D.end())
							visState.targetCalib.contCalibTargetTemplate = *track;
						else
						{ // Not used right now since all targets in obsDatabase should already be fully calibrated
							visState.targetCalib.contCalibTargetTemplate = TargetTemplate3D(tgt.targetID, "Temp",
								finaliseTargetMarkers(pipeline.getCalibs(), tgt, pipeline.targetCalib.params.post));
						}
						visState.targetCalib.contCalibTargetID = tgt.targetID;
					}
					else visState.resetVisTarget(false);
				}
				// Button to delete
				SameLineTrailing(ImGui::GetFrameHeight());
				bool del = CrossButton("Del");
				if (del)
				{
					if (visState.targetCalib.contCalibTargetID == tgt.targetID)
						visState.resetVisTarget(false);
					tgtIt = db_lock->targets.erase(tgtIt);
				}
				ImGui::PopID();
			}

			if (ImGui::Button("Optimise Cameras", SizeWidthDiv3()))
			{
				threadPool.push([&](int, ObsData data)
				{
					auto lastIt = pclock::now();
					int numSteps = 0, maxSteps = 10;
					OptErrorRes lastError;
					auto itUpdate = [&](OptErrorRes errors){
						lastError = errors;
						LOGC(LInfo, "    Reprojection rmse %.2fpx, %.2fpx +- %.2fpx, max %.2fpx, after %.2fms!",
							errors.rmse*PixelFactor, errors.mean*PixelFactor, errors.stdDev*PixelFactor, errors.max*PixelFactor, dt(lastIt, pclock::now()));
						lastIt = pclock::now();
						return numSteps++ < maxSteps;
					};
					auto calibs = pipeline.getCalibs();
					auto calibsOpt = calibs;

					ScopedLogCategory scopedLogCategory(LOptimisation);
					ScopedLogLevel scopedLogLevel(LInfo);
					LOGCL("Before optimising with optimisation database:");
					updateReprojectionErrors(data, calibs);

					LOGCL("Optimising camera position:");
					OptimisationOptions options(false, true, false, false, false);
					optimiseData<OptSparse>(options, data, calibsOpt, itUpdate, 1);
					for (int c = 0; c < calibs.size(); c++)
						LOGCL("        Camera %d was moved by %.2fmm", calibs[c].id, (calibs[c].transform.translation() - calibsOpt[c].transform.translation()).norm()*1000);

					{ // Update calibration
						std::unique_lock pipeline_lock(pipeline.pipelineLock);
						for (int c = 0; c < calibsOpt.size(); c++)
							pipeline.cameras[calibsOpt[c].index]->calib = calibsOpt[c];
					}

					// Update fundamental matrices using calibration
					UpdateCalibrationRelations(pipeline, *pipeline.calibration.contextualLock(), lastError, data.getValidSamples());
					UpdateCalibrations();
				}, *db_lock);
			}
			ImGui::SameLine();
			if (ImGui::Button("Optimise Targets", SizeWidthDiv3()))
			{
				threadPool.push([&](int, ObsData data)
				{
					auto lastIt = pclock::now();
					int numSteps = 0, maxSteps = 5;
					auto itUpdate = [&](OptErrorRes errors){
						LOGC(LInfo, "    Reprojection rmse %.2fpx, %.2fpx +- %.2fpx, max %.2fpx, after %.2fms!",
							errors.rmse*PixelFactor, errors.mean*PixelFactor, errors.stdDev*PixelFactor, errors.max*PixelFactor, dt(lastIt, pclock::now()));
						lastIt = pclock::now();
						return numSteps++ < maxSteps;
					};

					auto calibs = GetState().pipeline.getCalibs();

					ScopedLogCategory scopedLogCategory(LOptimisation);
					ScopedLogLevel scopedLogLevel(LInfo);
					LOGCL("Before optimising with optimisation database:");
					updateReprojectionErrors(data, calibs);

					LOGCL("Optimising target markers:");
					OptimisationOptions options(true, false, false, false, false);
					optimiseData<OptSparse>(options, data, calibs, itUpdate, 1);

					LOGCL("Done optimising! Applying to targets!");

					{ // Update Targets
						std::unique_lock pipeline_lock(pipeline.pipelineLock);
						for (auto &tgtNew : data.targets)
						{
							bool found = false;
							for (auto &tgt : pipeline.tracking.targetTemplates3D)
							{
								if (std::abs(tgt.id) != std::abs(tgtNew.targetID)) continue;
								updateMarkerOrientations(tgt.markers, calibs, tgtNew, pipeline.targetCalib.params.post);
								tgt.updateMarkers();
								found = true;
								break;
							}
							if (!found)
							{
								state.pipeline.tracking.targetTemplates3D.push_back(TargetTemplate3D(std::abs(tgtNew.targetID), "New Target",
									finaliseTargetMarkers(pipeline.getCalibs(), tgtNew, pipeline.targetCalib.params.post)));
							}
						}
					}

					{ // Update Targets in Database
						auto db_lock = pipeline.obsDatabase.contextualLock();
						for (auto &tgtNew : data.targets)
						{
							for (auto &tgtOld : db_lock->targets)
							{
								if (std::abs(tgtOld.targetID) != std::abs(tgtNew.targetID)) continue;
								tgtOld.markers = tgtNew.markers;
								auto frameIt = tgtNew.frames.begin();
								for (auto &frame : tgtOld.frames)
								{
									while (frameIt != tgtNew.frames.end() && frameIt->frame < frame.frame)
										frameIt++;
									while (frameIt == tgtNew.frames.end()) break;
									assert (frameIt->frame == frame.frame);
									// Apply new outliers, new pose, and error
									tgtOld.outlierSamples += frameIt->samples.size() - frame.samples.size();
									*frameIt = std::move(frame);
								}
								break;
							}
						}
					}
				}, *db_lock);
			}
			ImGui::SameLine();
			if (ImGui::Button("Set Marker FoV", SizeWidthDiv3()))
			{
				threadPool.push([&](int, ObsData data)
				{
					auto calibs = GetState().pipeline.getCalibs();

					ScopedLogCategory scopedLogCategory(LOptimisation);
					ScopedLogLevel scopedLogLevel(LInfo);
					LOGCL("Errors:");
					updateReprojectionErrors(data, calibs);

					{ // Update Targets
						std::unique_lock pipeline_lock(pipeline.pipelineLock);
						for (auto &tgtNew : data.targets)
						{
							for (auto &tgt : pipeline.tracking.targetTemplates3D)
							{
								if (tgt.id != tgtNew.targetID) continue;
								updateMarkerOrientations(tgt.markers, calibs, tgtNew, pipeline.targetCalib.params.post);
								tgt.updateMarkers();
								break;
							}
						}
					}

				}, *db_lock);
			}

			if (ImGui::Button("Save Cameras", SizeWidthDiv3()))
			{
				ServerStoreCameraCalib(state);
			}
			ImGui::SameLine();
			if (ImGui::Button("Save Targets", SizeWidthDiv3()))
			{
				ServerStoreTargetCalib(state);
			}
			ImGui::SameLine();
			if (ImGui::Button("Clear Database", SizeWidthDiv3()))
			{
				db_lock.unlock();
				*pipeline.obsDatabase.contextualLock() = {};
			}
		}
		EndSection();

		if (state.mode == MODE_Replay)
		{
			BeginSection("Tracking Results for this replay");
			if (ImGui::Button("Update on disk", SizeWidthDiv3()))
			{
				auto records = pipeline.frameRecords.getView();
				dumpTrackingResults(state.loadedFramePath, records.begin(), records.end(), state.loadedFrameOffset);
			}
			ImGui::SameLine();
			if (ImGui::Button("Load from disk", SizeWidthDiv3()))
			{
				for (auto &record : state.loadedFrameRecords)
					record.tracking = {};
				parseTrackingResults(state.loadedFramePath, state.loadedFrameRecords, state.loadedFrameOffset);
			}
			ImGui::SameLine();
			if (ImGui::Button("Set temporarily", SizeWidthDiv3()))
			{
				for (auto &record : state.loadedFrameRecords)
					record.tracking = {};
				for (const auto &record : pipeline.frameRecords.getView())
				{
					if (!record) continue;
					if (state.loadedFrameRecords.size() <= record->num) break;
					state.loadedFrameRecords[record->num].tracking = record->tracking;
				}
			}

			// Tracking quality assessment;
			struct EventChange
			{
				int loaded, current, added, removed;
			};
			static EventChange losses, search2D, detect2D, detect3D, tracked;
			struct TargetTracking
			{
				std::string label;
				EventChange tracked;
				StatValue<float,StatExtremas|StatDistribution> samplesLoaded, samplesCurrent, samplesAdded, samplesRemoved;
				StatValue<float,StatExtremas|StatDistribution> errorLoaded, errorCurrent, errorAdded, errorRemoved;
			};
			static std::map<int, TargetTracking> targets;
			static long coveredFrames;
			if (ImGui::Button("Update Tracking Results", SizeWidthFull()))
			{
				auto updateEventChange = [](EventChange &event, int loaded, int current)
				{
					event.loaded += loaded;
					event.current += current;
					event.added += std::max(0, current-loaded);
					event.removed += std::max(0, loaded-current);
				};
				losses = {};
				search2D = {};
				detect2D = {};
				detect3D = {};
				tracked = {};
				targets.clear();
				coveredFrames = 0;
				for (const auto &record : pipeline.frameRecords.getView())
				{
					if (!record) continue;
					if (state.loadedFrameRecords.size() <= record->num) break;
					coveredFrames++;
					auto &loaded = state.loadedFrameRecords[record->num].tracking;
					auto &current = record->tracking;
					// Update changes to all tracking events
					updateEventChange(losses, loaded.trackingLosses, current.trackingLosses);
					updateEventChange(search2D, loaded.searches2D, current.searches2D);
					updateEventChange(detect2D, loaded.detections2D, current.detections2D);
					updateEventChange(detect3D, loaded.detections3D, current.detections3D);
					updateEventChange(tracked, loaded.targets.size(), current.targets.size());
					// Accumulate results for each target for this frame from loaded and current results
					struct TargetTrackingFrame {
						int samplesLoaded = 0, samplesCurrent = 0;
						float errorLoaded = 0, errorCurrent = 0;
					};
					std::map<int, TargetTrackingFrame> targetsFrame;
					for (auto &tracker : current.targets)
					{
						auto &tgt = targetsFrame[tracker.id];
						tgt.samplesCurrent = tracker.error.samples;
						tgt.errorCurrent = tracker.error.mean;
					}
					for (auto &tracker : loaded.targets)
					{
						auto &tgt = targetsFrame[tracker.id];
						tgt.samplesLoaded = tracker.error.samples;
						tgt.errorLoaded = tracker.error.mean;
					}
					// Update changes to each targets tracking results
					for (auto &tgt : targetsFrame)
					{
						auto &tgtRec = targets[tgt.first];
						auto &tgtFrame = tgt.second;
						updateEventChange(tgtRec.tracked, std::min(1, tgtFrame.samplesLoaded), std::min(1, tgtFrame.samplesCurrent));
						if (tgtFrame.samplesLoaded)
						{
							tgtRec.samplesLoaded.update(tgtFrame.samplesLoaded);
							tgtRec.errorLoaded.update(tgtFrame.errorLoaded);
						}
						if (tgtFrame.samplesCurrent)
						{
							tgtRec.samplesCurrent.update(tgtFrame.samplesCurrent);
							tgtRec.errorCurrent.update(tgtFrame.errorCurrent);
						}
						if (tgtFrame.samplesCurrent > tgtFrame.samplesLoaded)
							tgtRec.samplesAdded.update(tgtFrame.samplesCurrent-tgtFrame.samplesLoaded);
						if (tgtFrame.samplesCurrent < tgtFrame.samplesLoaded)
							tgtRec.samplesRemoved.update(tgtFrame.samplesLoaded-tgtFrame.samplesCurrent);
						if (tgtFrame.samplesLoaded && tgtFrame.samplesCurrent)
						{
							if (tgtFrame.errorCurrent > tgtFrame.errorLoaded)
								tgtRec.errorAdded.update(tgtFrame.errorCurrent-tgtFrame.errorLoaded);
							if (tgtFrame.errorCurrent < tgtFrame.errorLoaded)
								tgtRec.errorRemoved.update(tgtFrame.errorLoaded-tgtFrame.errorCurrent);
						}
					}
				}
				for (auto &tgt : state.pipeline.tracking.targetTemplates3D)
				{
					auto it = targets.find(tgt.id);
					if (it != targets.end())
						it->second.label = tgt.label;
				}
			}
			if (!targets.empty() && ImGui::TreeNode("Target Results"))
			{
				ImGui::Text("Tracking Losses: %d  [%d]  +%d -%d", losses.current, losses.loaded, losses.added, losses.removed);
				ImGui::Text("2D Searches: %d  [%d]  +%d -%d", search2D.current, search2D.loaded, search2D.added, search2D.removed);
				ImGui::Text("2D Detections: %d  [%d]  +%d -%d", detect2D.current, detect2D.loaded, detect2D.added, detect2D.removed);
				ImGui::Text("3D Detections: %d  [%d]  +%d -%d", detect3D.current, detect3D.loaded, detect3D.added, detect3D.removed);
				ImGui::Text("Tracked Tgt/Frame: %d  [%d]  +%d -%d", tracked.current, tracked.loaded, tracked.added, tracked.removed);

				for (auto &trackedTarget : targets)
				{
					ImGui::PushID(trackedTarget.first);
					if (ImGui::TreeNode(trackedTarget.second.label.c_str()))
					{
						auto &tgt = trackedTarget.second;
						ImGui::Text("Frames: %d  [%d]  +%d -%d", tgt.tracked.current, tgt.tracked.loaded, tgt.tracked.added, tgt.tracked.removed);
						ImGui::Text("Samples:");
						ImGui::Text("    sum: %ld  [%ld]  +%ld -%ld", (long)tgt.samplesCurrent.sum, (long)tgt.samplesLoaded.sum, (long)tgt.samplesAdded.sum, (long)tgt.samplesRemoved.sum);
						ImGui::Text("     avg: %.2f +- %.2f  [%.2f +- %.2f]", tgt.samplesCurrent.avg, tgt.samplesCurrent.stdDev()*2, tgt.samplesLoaded.avg, tgt.samplesLoaded.stdDev()*2);
						ImGui::Text("	 diff: +%.2f (%d) -%.2f (%d), %ld unchanged", tgt.samplesAdded.avg, tgt.samplesAdded.num, tgt.samplesRemoved.avg, tgt.samplesRemoved.num, coveredFrames-(tgt.samplesAdded.num+tgt.samplesRemoved.num));
						ImGui::Text("      >/<: %d/%d  [%d/%d]", (int)tgt.samplesCurrent.min, (int)tgt.samplesCurrent.max, (int)tgt.samplesLoaded.min, (int)tgt.samplesLoaded.max);
						ImGui::Text("Errors :");
						ImGui::Text("     avg: %.2fpx +- %.2fpx  [%.2fpx +- %.2fpx]", tgt.errorCurrent.avg*PixelFactor, tgt.errorCurrent.stdDev()*2*PixelFactor, tgt.errorLoaded.avg*PixelFactor, tgt.errorLoaded.stdDev()*2*PixelFactor);
						ImGui::Text("	 diff: +%.2fpx (%d) -%.2fpx (%d), %ld unchanged", tgt.errorAdded.avg*PixelFactor, tgt.errorAdded.num, tgt.errorRemoved.avg*PixelFactor, tgt.errorRemoved.num, coveredFrames-(tgt.errorAdded.num+tgt.errorRemoved.num));
						ImGui::Text("      >/<: %.3fpx/%.2fpx  [%.3fpx/%.2fpx]", tgt.errorCurrent.min*PixelFactor, tgt.errorCurrent.max*PixelFactor, tgt.errorLoaded.min*PixelFactor, tgt.errorLoaded.max*PixelFactor);
						ImGui::TreePop();
					}
					ImGui::PopID();
				}
				ImGui::TreePop();
			}

			EndSection();
		}
	}
	else if (pipeline.phase == PHASE_Calibration_Point)
	{
		UpdatePipelineCalibSection();
		UpdatePipelineObservationSection();
		UpdatePipelinePointCalib();
	}
	else if (pipeline.phase == PHASE_Calibration_Target)
	{
		UpdatePipelineTargetCalib();
	}

	ImGui::End();
}