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
		ImGui::End();
		return;
	}

	ImVec2 ButtonSize = ImVec2(std::min(100.0f, SizeWidthDiv3().x), ImGui::GetFrameHeight());

	bool displayInternalDebug = (state.mode == MODE_Replay || state.mode == MODE_Simulation) && (state.simAdvance.load() == 0 || dbg_isBreaking);

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

		// Display list of trackable targets
		for (auto &trackerIt : visState.tracking.targets)
		{
			int trackerID = trackerIt.first;
			auto &tracker = trackerIt.second;
			ImGui::PushID(trackerID);
			long trackedAgo = frameNum - tracker.lastTrackedFrame;
			std::string label;
			if (trackedAgo < 5)
				label = asprintf_s("Tracking '%s' (%d)###TgtTrk", tracker.label.c_str(), trackerID);
			else if (trackedAgo < 1000)
				label = asprintf_s("Lost '%s' (%d), %ld frames ago###TgtTrk", tracker.label.c_str(), trackerID, trackedAgo);
			else
				label = asprintf_s("Dormant '%s' (%d)###TgtTrk", tracker.label.c_str(), trackerID);
			ImGui::AlignTextToFramePadding();
			if (ImGui::Selectable(label.c_str(), visState.tracking.focusedTrackerID == trackerID, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
				visState.tracking.focusedTrackerID = trackerID;

			if (tracker.imuState != TrackerInertialState::NO_IMU)
			{
				SameLineTrailing(iconSize().x);
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
			}
			ImGui::PopID();
		}

		EndSection();

		if (displayInternalDebug && visFrame && visState.tracking.focusedTrackerID != 0)
		{
			// Temporary debug tools, data, etc.

			auto &frameRecord = *visFrame.frameIt->get();
			auto trackRecord = std::find_if(frameRecord.trackers.begin(), frameRecord.trackers.end(),
						[&](auto &t){ return t.id == visState.tracking.focusedTrackerID; });
			auto trackConfig = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
						[&](auto &t){ return t.id == visState.tracking.focusedTrackerID; });
			if (trackRecord != frameRecord.trackers.end() && trackConfig != state.trackerConfigs.end()
				&& trackConfig->type == TrackerConfig::TRACKER_TARGET)
			{
				BeginSection("Tracking Debug");
					
				auto &debugVis = visState.tracking.debug;

				std::vector<std::vector<Eigen::Vector2f> const *> points2D(pipeline.cameras.size());
				std::vector<std::vector<BlobProperty> const *> properties(pipeline.cameras.size());
				std::vector<std::vector<int>> remainingPoints2D(pipeline.cameras.size());
				std::vector<std::vector<int> const *> relevantPoints2D(pipeline.cameras.size());
				for (int c = 0; c < pipeline.cameras.size(); c++)
				{
					points2D[c] = &frameRecord.cameras[c].points2D;
					properties[c] = &frameRecord.cameras[c].properties;
					remainingPoints2D[c].resize(points2D[c]->size());
					std::iota(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), 0);
					relevantPoints2D[c] = &remainingPoints2D[c];
				}

				if (ImGui::Button("Debug target matching", SizeWidthFull()))
				{ // Manually trigger update - might also be automatic
					debugVis.needsUpdate = true;
				}
				if (debugVis.needsUpdate)
				{
					debugVis.needsUpdate = false;
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
					ImGui::Text("Original tracking error: %.2fpx +- %.2fpx, %.2fpx max",
						trackRecord->error.mean*PixelFactor, trackRecord->error.stdDev*PixelFactor, trackRecord->error.max*PixelFactor);
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
							debugVis.editedMatch2D, trackRecord->posePredicted, pipeline.params.track.opt);
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
	}

	if (pipeline.phase == PHASE_Tracking)
	{
		if (ImGui::CollapsingHeader("Optimisation Database"))
		{
			ImGui::PushID("OptDb");

			auto db_lock = pipeline.obsDatabase.lock();
			ImGui::Text("%d points, %d samples (%d outliers)", (int)db_lock->points.points.size(), db_lock->points.totalSamples, db_lock->points.outlierSamples);
			for (auto tgtIt = db_lock->targets.begin(); tgtIt != db_lock->targets.end(); tgtIt++)
			{
				auto tgt = *tgtIt;
				ImGui::PushID(tgt.targetID);
				std::string label = asprintf_s("Target %d: %d frames, %d samples, %d outliers###TgtCont",
					tgt.targetID, (int)tgt.frames.size(), tgt.totalSamples, tgt.outlierSamples);
				bool selected = visState.target.selectedTargetID == tgt.targetID;
				if (ImGui::Selectable(label.c_str(), visState.target.selectedTargetID == tgt.targetID, ImGuiSelectableFlags_AllowOverlap))
				{
					if (visState.target.selectedTargetID != tgt.targetID)
					{ // Set new selected target ID
						visState.target.selectedTargetCalib = {};
						auto track = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
							[&](auto &t){ return t.id == tgt.targetID; });
						if (track == state.trackerConfigs.end())
						{ // Not used right now since all targets in obsDatabase should already be fully calibrated
							visState.target.selectedTargetCalib = TargetCalibration3D(finaliseTargetMarkers(
								pipeline.getCalibs(), tgt, pipeline.targetCalib.params.post));
						}
						visState.target.selectedTargetID = tgt.targetID;
					}
					else visState.resetVisTarget(false);
				}
				// Button to delete
				SameLineTrailing(ImGui::GetFrameHeight());
				bool del = CrossButton("Del");
				if (del)
				{
					if (visState.target.selectedTargetID == tgt.targetID)
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
							errors.rmse*PixelFactor, errors.mean*PixelFactor, errors.stdDev*PixelFactor, errors.max*PixelFactor, dtMS(lastIt, pclock::now()));
						lastIt = pclock::now();
						return numSteps++ < maxSteps;
					};
					auto calibs = pipeline.getCalibs();

					ScopedLogCategory scopedLogCategory(LOptimisation);
					ScopedLogLevel scopedLogLevel(LInfo);
					LOGCL("Before optimising with optimisation database:");
					updateReprojectionErrors(data, calibs);

					LOGCL("Optimising camera position:");
					OptimisationOptions options(false, true, false, false, false);
					optimiseDataSparse(options, data, calibs, itUpdate, 1);

					// Update calibration
					AdoptNewCalibrations(pipeline, calibs, true);
					UpdateCalibrationRelations(pipeline, *pipeline.calibration.contextualLock(), data);
					SignalCameraCalibUpdate(calibs);

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
							errors.rmse*PixelFactor, errors.mean*PixelFactor, errors.stdDev*PixelFactor, errors.max*PixelFactor, dtMS(lastIt, pclock::now()));
						lastIt = pclock::now();

						LOGCL("Applying to targets!");

						// Update Targets
						for (auto &tgtNew : data.targets)
						{
							for (auto &tracker : state.trackerConfigs)
							{ // For now, fetch from trackerConfig - may also fetch from trackedTargets/dormantTargets
								if (tracker.id != tgtNew.targetID) continue;
								// Copy and update target calibration
								TargetCalibration3D calib = tracker.calib;
								calib.markers = finaliseTargetMarkers(pipeline.getCalibs(), tgtNew, pipeline.targetCalib.params.post);
								calib.updateMarkers();
								// Signal server to update calib
								SignalTargetCalibUpdate(tgtNew.targetID, calib);
								break;
							}
						}

						return numSteps++ < maxSteps;
					};

					auto calibs = GetState().pipeline.getCalibs();

					ScopedLogCategory scopedLogCategory(LOptimisation);
					ScopedLogLevel scopedLogLevel(LInfo);
					LOGCL("Before optimising with optimisation database:");
					updateReprojectionErrors(data, calibs);

					LOGCL("Optimising target markers:");
					OptimisationOptions options(true, false, false, false, false);
					optimiseDataSparse(options, data, calibs, itUpdate, 1);

					LOGCL("Done optimising! Applying to database1");

					{ // Update Targets in Database
						auto db_lock = pipeline.obsDatabase.contextualLock();
						for (auto &tgtOpt : data.targets)
						{
							for (auto &tgtDB : db_lock->targets)
							{
								if (std::abs(tgtDB.targetID) != std::abs(tgtOpt.targetID)) continue;
								tgtDB.markers = tgtOpt.markers;
								auto frameOpt = tgtOpt.frames.begin();
								for (auto &frameDB : tgtDB.frames)
								{
									while (frameOpt != tgtOpt.frames.end() && frameOpt->frame < frameDB.frame)
										frameOpt++;
									if (frameOpt == tgtOpt.frames.end()) break;
									assert (frameOpt->frame == frameDB.frame);
									// Apply new outliers, new pose, and error
									tgtDB.outlierSamples += frameDB.samples.size() - frameOpt->samples.size();
									frameDB = std::move(*frameOpt);
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

					// Update Targets
					for (auto &tgtNew : data.targets)
					{
						for (auto &tracker : state.trackerConfigs)
						{ // For now, fetch from trackerConfig - may also fetch from trackedTargets/dormantTargets
							if (tracker.id != tgtNew.targetID) continue;
							// Copy and update target calibration
							TargetCalibration3D calib = tracker.calib;
							updateMarkerOrientations(calib.markers, calibs, tgtNew, pipeline.targetCalib.params.post);
							calib.updateMarkers();
							// Signal server to update calib
							SignalTargetCalibUpdate(tgtNew.targetID, calib);
							break;
						}
					}

				}, *db_lock);
			}

			if (SaveButton("Save Cameras", SizeWidthDiv3(), state.cameraCalibsDirty))
			{
				storeCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
				state.cameraCalibsDirty = false;
			}
			ImGui::SameLine();
			if (SaveButton("Save Targets", SizeWidthDiv3(), state.trackerConfigDirty || state.trackerCalibsDirty || state.trackerIMUsDirty))
			{
				storeTrackerConfigurations("store/trackers.json", state.trackerConfigs);
				state.trackerConfigDirty = state.trackerCalibsDirty = state.trackerIMUsDirty = false;
			}
			ImGui::SameLine();
			if (ImGui::Button("Clear Database", SizeWidthDiv3()))
			{
				db_lock.unlock();
				*pipeline.obsDatabase.contextualLock() = {};
			}

			ImGui::PopID();
		}

		if (state.mode == MODE_Replay)
		{
			BeginSection("Tracking Results for this replay");
			if (ImGui::Button("Update on disk", SizeWidthDiv3()))
			{
				assert(state.recording.segments.size() == state.recording.tracking.size());
				for (int i = 0; i < state.recording.segments.size(); i++)
				{
					auto &segment = state.recording.segments[i];
					if (segment.frameCount == 0) continue; // Invalid or missing segment
					if (state.recording.tracking[i].empty())
						state.recording.tracking[i] = state.recording.captures[i];
					dumpTrackingResults(state.recording.tracking[i], pipeline.record,
						segment.frameStart, segment.frameStart+segment.frameCount, segment.frameOffset);
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
					parseTrackingResults(state.recording.tracking[i], state.stored, segment.frameOffset);
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
				int loaded, current, added, removed;
			};
			static EventChange losses, search2D, detect2D, detect3D, tracked;
			struct TrackingSamples
			{
				std::string label;
				EventChange tracked;
				StatValue<float,StatExtremas|StatDistribution> samplesLoaded, samplesCurrent, samplesAdded, samplesRemoved;
				StatValue<float,StatExtremas|StatDistribution> errorLoaded, errorCurrent, errorAdded, errorRemoved;
			};
			static std::map<int, TrackingSamples> trackers;
			static long coveredFrames;
			if (ImGui::Button("Update Tracking Results", SizeWidthFull()))
			{
				auto countEvents = [](const FrameRecord &record, TrackingResult::State result)
				{
					return std::count_if(record.trackers.begin(), record.trackers.end(),
						[&](auto &t){ return t.result.isState(result); });
				};
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
						int samplesLoaded = 0, samplesCurrent = 0;
						float errorLoaded = 0, errorCurrent = 0;
					};
					std::map<int, FrameTrackers> frameTrackers;
					for (auto &trackRecord : record.trackers)
					{
						auto &trkFrame = frameTrackers[trackRecord.id];
						trkFrame.samplesCurrent = trackRecord.error.samples;
						trkFrame.errorCurrent = trackRecord.error.mean;
					}
					for (auto &trackStored : stored.trackers)
					{
						auto &trkFrame = frameTrackers[trackStored.id];
						trkFrame.samplesLoaded = trackStored.error.samples;
						trkFrame.errorLoaded = trackStored.error.mean;
					}
					// Update changes to each targets tracking results
					for (auto &trk : frameTrackers)
					{
						auto &tracker = trackers[trk.first];
						auto &trkFrame = trk.second;
						updateEventChange(tracker.tracked, std::min(1, trkFrame.samplesLoaded), std::min(1, trkFrame.samplesCurrent));
						if (trkFrame.samplesLoaded)
						{
							tracker.samplesLoaded.update(trkFrame.samplesLoaded);
							tracker.errorLoaded.update(trkFrame.errorLoaded);
						}
						if (trkFrame.samplesCurrent)
						{
							tracker.samplesCurrent.update(trkFrame.samplesCurrent);
							tracker.errorCurrent.update(trkFrame.errorCurrent);
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