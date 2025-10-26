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
#include "calib/obs_data.inl"

// This is just a cached value to reduce the amount sequences needs to be saved/loaded
static int savedSequencesLastFrame = 0;

static bool ensureSequencesSaved(const PipelineState &pipeline, const SequenceData &sequences)
{
	if (savedSequencesLastFrame == sequences.lastRecordedFrame)
		return true;
	// Copy camera ids
	std::vector<int> cameraIDs;
	for (auto &cam : pipeline.cameras)
		cameraIDs.push_back(cam->id);
	// Write accompanying sequences
	auto error = dumpSequenceDatabase("dump/target_calib_sequences.json", sequences, cameraIDs);
	if (error) GetState().errors.push(error.value());
	else savedSequencesLastFrame = sequences.lastRecordedFrame;
	return !error;
}

static bool checkSequencesLoad(PipelineState &pipeline, SequenceData &sequences)
{
	if (savedSequencesLastFrame == 0 || savedSequencesLastFrame > sequences.lastRecordedFrame)
	{
		std::vector<int> cameraIDs;
		SequenceData loadedSequences;
		auto error = parseSequenceDatabase("dump/target_calib_sequences.json", cameraIDs, loadedSequences);
		if (error)
		{
			LOG(LTargetCalib, LError, "%s", error->c_str());
			GetState().errors.push(error.value());
			return false;
		}
		bool valid = cameraIDs.size() == pipeline.cameras.size();
		if (valid)
		{
			std::vector<int> camMap(cameraIDs.size(), -1);
			for (auto &cam : pipeline.cameras)
			{
				bool found = false;
				for (int c = 0; c < cameraIDs.size(); c++)
				{
					if (cameraIDs[c] == cam->id)
					{
						camMap[c] = cam->index;
						found = true;
						break;
					}
				}
				if (!found) valid = false;
			}
		}
		if (!valid)
		{
			LOG(LTargetCalib, LError, "Cameras mismatch those in the accompanying sequence database - perhaps you loaded the wrong recording?");
			GetState().errors.push("Cameras mismatch those in the accompanying sequence database - perhaps you loaded the wrong recording?");
			return false;
		}
		sequences = std::move(loadedSequences);
		savedSequencesLastFrame = sequences.lastRecordedFrame;
		if (GetState().mode == MODE_Replay && pipeline.frameNum == -1)
		{ // Automatically load stored frames into current record if we haven't already started playback
			ServerState &state = GetState();
			std::unique_lock pipeline_lock(pipeline.pipelineLock);
			auto storedFrames = state.stored.frames.getView();
			LOG(LTargetCalib, LInfo, "Automatically 'replaying' %d stored frames for loaded data!", (int)storedFrames.size());
			state.recording.replayTime = sclock::now();
			for (auto record : storedFrames)
			{
				std::shared_ptr<FrameRecord> frameRecord = std::make_shared<FrameRecord>();
				frameRecord->num = record->num;
				frameRecord->ID = record->ID;
				frameRecord->time = state.recording.replayTime + (record->time - state.recording.replayTime);
				frameRecord->cameras = record->cameras;
				pipeline.record.frames.insert(record->num, std::move(frameRecord));
			}
		}
		else
			LOG(LTargetCalib, LInfo, "Relying on existing frames records to cover loaded data!");
		if (GetState().mode == MODE_Replay && pipeline.record.frames.getView().size() < sequences.lastRecordedFrame)
		{ // Check that the frames are actually covering the loaded sequences
			LOG(LTargetCalib, LError, "Current frames aren't covering loaded data fully! Either recording mismatches or you started playback but didn't wait for it to finish!");
			GetState().errors.push("Current frames aren't covering loaded data fully! Either recording mismatches or you started playback but didn't wait for it to finish!");
			return false;
		}
	}
	return true;
}

void InterfaceState::UpdatePipelineTargetCalib()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	ImVec2 ButtonSize = ImVec2(std::min(100.0f, SizeWidthDiv3().x), ImGui::GetFrameHeight());

	// Set lighter background for all the tables
	ImVec4 color = ImGui::GetStyleColorVec4(ImGuiCol_ChildBg);
	color.x *= 1.2f;
	color.y *= 1.2f;
	color.z *= 1.2f;
	ImGui::PushStyleColor(ImGuiCol_ChildBg, color);

	bool sectionTargetViews = ImGui::CollapsingHeader("Target Views (?)", ImGuiTreeNodeFlags_DefaultOpen);
	ImGui::SetItemTooltip("Move around a target with sufficient flat markers on it, not too fast, and in good view of several cameras.\n"
		"The system will look for good views (short segments of around one second where the view of the markers is good.\n"
		"These TargetViews will then be reconstructed to get an estimate of the 3D structure of the markers.");
	ImGui::PushID("TV");

	if (sectionTargetViews)
	{ // Target Observations to acquire Target Views
		ImGui::Checkbox("Record Target Observations", &pipeline.recordSequences);
		ImGui::SetItemTooltip("Record visible markers into observations as continous sequences.");

		if (ImGui::Button("Clear##Observations", SizeWidthDiv3()))
		{
			ResetTargetCalibration(pipeline);
			
			// Clean up UI state
			targetViewsSorted.clear();
			visState.targetCalib.edit = nullptr;
			visState.resetVisTarget();

			// Reset observations
			pipeline.seqDatabase.contextualLock()->clear();
			UpdateSequences(true);
		}
		ImGui::SameLine();
		if (ImGui::Button("Save##Observations", SizeWidthDiv3()))
		{
			// Make sure the relevant sequence data is saved alongside
			if (ensureSequencesSaved(pipeline, *pipeline.seqDatabase.contextualRLock()))
			{
				auto error = dumpTargetViewRecords("dump/target_calib_views.json", *pipeline.targetCalib.views.contextualRLock());
				if (error) GetState().errors.push(error.value());
			}
		}
		ImGui::SameLine();
		if (ImGui::Button("Load##Observations", SizeWidthDiv3()))
		{
			// Check if we need to load sequence database from checkpoint, too
			if (checkSequencesLoad(pipeline, *pipeline.seqDatabase.contextualLock()))
			{
				auto views_lock = pipeline.targetCalib.views.contextualLock();
				auto error = parseTargetViewRecords("dump/target_calib_views.json", pipeline.record.frames, *views_lock);
				if (error) GetState().errors.push(error.value());
				else
				for (auto &view : *views_lock)
				{
					{ // Fill target observations and calculate errors
						auto view_lock = view->target.contextualLock();
						updateTargetObservations(*view_lock, pipeline.seqDatabase.contextualRLock()->markers);
						view->state.errors = getTargetErrorDist(pipeline.getCalibs(), *view_lock);
					}
					view->state.calibrated = true;
					// Also start optimisation
					view->plan = { TargetView::OPTIMISE_COARSE };
					view->planned = true;
				}
			}
		}
	}

	if (sectionTargetViews && ImGui::BeginTable("Target Views Table", 6,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_Sortable | ImGuiTableFlags_ScrollY | ImGuiTableFlags_PadOuterX,
		ImVec2(0, ImGui::GetFrameHeight()*10)))
	{
		ImGui::TableSetupScrollFreeze(0, 1); // Fix top row even with scrolling
		ImGui::TableSetupColumn("##Selection", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("Markers");
		ImGui::TableSetupColumn("Samples");
		ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch | ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_PreferSortAscending, 2.0f);
		ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableHeadersRow();

		auto sorting = ImGui::TableGetSortSpecs();
		bool newSorting = sorting->SpecsDirty || targetViewsDirty;
		targetViewsDirty = false;
		{
			auto views_lock = pipeline.targetCalib.views.contextualRLock();	
			if (targetViewsSorted.size() != views_lock->size() || newSorting)
			{
				targetViewsSorted.clear();
				for (auto &view : *views_lock)
					targetViewsSorted.push_back(view);
				newSorting = true;
			}
		}
		if (newSorting)
		{
			for (int i = 0; i < sorting->SpecsCount; i++)
			{
				int sortIndex = sorting->Specs[i].ColumnIndex;
				auto valueFunc = [sortIndex](const TargetView &view)
				{
					switch(sortIndex)
					{
						case 0: return view.selected? 1.0f : 	0.0f;
						case 1: return (float)view.id;
						case 2: return (float)view.stats.markerCount;
						case 3: return (float)view.stats.sampleCount;
						case 4: return view.state.calibrated? (float)view.state.errors.rmse : 100000000.0f;
						case 5: return 0.0f;
						default: return 0.0f;
					}
				};
				bool sortAsc = sorting->Specs[i].SortDirection == ImGuiSortDirection_Ascending;
				std::stable_sort(targetViewsSorted.begin(), targetViewsSorted.end(), [sortAsc, &valueFunc](const auto &v1, const auto &v2){
					float f1 = valueFunc(*v1), f2 = valueFunc(*v2);
					if (std::isnan(f1)) f1 = std::numeric_limits<float>::infinity();
					if (std::isnan(f2)) f2 = std::numeric_limits<float>::infinity();
					if (sortAsc) return f1 < f2;
					else return f1 > f2;
				});
			}
			sorting->SpecsDirty = false;
			targetViewsDirty = false;
		}

		for (int i = 0; i < targetViewsSorted.size(); i++)
		{
			auto &viewPtr = targetViewsSorted[i];
			TargetView &view = *viewPtr;
			ImGui::PushID(view.beginFrame);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Checkbox("##Select", &view.selected);
			ImGui::TableNextColumn();
			ImGui::Text("%d", view.id);
			ImGui::TableNextColumn();
			ImGui::Text("%d", view.stats.markerCount);
			ImGui::TableNextColumn();
			ImGui::Text("%d", view.stats.sampleCount);
			if (ImGui::BeginItemTooltip())
			{
				auto tgt_lock = view.target.contextualRLock();
				ImGui::Text("%d samples, %d inliers, %d outliers",
					tgt_lock->totalSamples+tgt_lock->outlierSamples, tgt_lock->totalSamples, tgt_lock->outlierSamples);
				ImGui::EndTooltip();
			}
			ImGui::TableNextColumn();
			if (view.control.running())
			{
				switch (view.state.step)
				{
					case TargetView::RECONSTRUCT:
						ImGui::TextUnformatted("Reconstructing...");
						break;
					case TargetView::REEVALUATE_MARKERS:
					case TargetView::TEST_REEVALUATE_MARKERS:
						ImGui::TextUnformatted("Reevaluating...");
						break;
					case TargetView::OPTIMISE_COARSE:
					case TargetView::OPTIMISE_FINE:
					ImGui::Text("%.3fpx (%d/%d)", view.state.errors.rmse*PixelFactor, view.state.numSteps, view.state.maxSteps);
						break;
					default:
						ImGui::TextUnformatted("...");
				}
			}
			else if (view.state.calibrated && view.state.complete)
				ImGui::Text("%.3fpx [%d]", view.state.errors.rmse*PixelFactor, view.state.numSteps);
			else if (view.state.calibrated && view.state.numSteps > 0)
				ImGui::Text("%.3fpx (%d)", view.state.errors.rmse*PixelFactor, view.state.numSteps);
			else if (view.state.calibrated)
				ImGui::Text("%.3fpx", view.state.errors.rmse*PixelFactor);
			else
				ImGui::Text("---");
			ImGui::SetItemTooltip("%.4fpx RMSE, %.4fpx +- %.4fpx (avg +- std deviation), %.4fpx max - all without outliers",
				view.state.errors.rmse*PixelFactor,
				view.state.errors.mean*PixelFactor,
				view.state.errors.stdDev*PixelFactor,
				view.state.errors.max*PixelFactor);
			ImGui::TableNextColumn();
			bool deleteView = CrossButton("Del");
			ImGui::SameLine();
			bool select = visState.targetCalib.view == viewPtr;
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{
				if (visState.resetVisTarget())
				{
					visState.targetCalib.view = viewPtr;
					visState.updateVisTarget();
				}
			}
			if (deleteView)
			{
				if (visState.targetCalib.view == viewPtr)
				{
					visState.targetCalib.view = nullptr;
					visState.resetVisTarget();
				}
				auto views_lock = pipeline.targetCalib.views.contextualLock();
				auto viewIt = std::find_if(views_lock->begin(), views_lock->end(),
					[&](const auto &v){ return v == viewPtr; });
				if (viewIt != views_lock->end())
				{
					if (viewIt->get()->control.running())
						viewIt->get()->control.stop_source.request_stop();
					views_lock->erase(viewIt);
				}
				targetViewsSorted.erase(targetViewsSorted.begin()+i);
				i--;
			}
			ImGui::PopID();
		}
		ImGui::EndTable();
	}

	if (sectionTargetViews && visState.targetCalib.view)
	{ // Details and actions on selected Target View
		auto &view = *visState.targetCalib.view;
		if (view.state.calibrated)
		{
			ImGui::Text("%.4fpx RMSE, %.4fpx +- %.4fpx, %.4fpx max",
				view.state.errors.rmse*PixelFactor,
				view.state.errors.mean*PixelFactor,
				view.state.errors.stdDev*PixelFactor,
				view.state.errors.max*PixelFactor);
			ImGui::SetItemTooltip("RMSE, avg error +- std deviation, max error - all without outliers");
		}
		else
			ImGui::TextUnformatted("View not calibrated.");

		if (pipeline.targetCalib.assemblyStages.contextualRLock()->empty())
		{
			if (pipeline.targetCalib.baseView == visState.targetCalib.view)
			{
				if (ImGui::Button("Auto-Select Base View", SizeWidthDiv2()))
					pipeline.targetCalib.baseView = nullptr;
			}
			else
			{
				if (ImGui::Button("Pick as Base View", SizeWidthDiv2()))
					pipeline.targetCalib.baseView = visState.targetCalib.view; // new shared_ptr
			}
		}
		else if (pipeline.targetCalib.baseView == visState.targetCalib.view)
		{
			ImGui::TextUnformatted("View has been selected as Base View.");
		}

		SameLineTrailing(SizeWidthDiv2().x);
		if (ImGui::Button("Save OBJ", SizeWidthDiv2()))
		{
			const char* tgtPathFmt = "dump/target_%d.obj";
			std::string tgtPath = asprintf_s(tgtPathFmt, findLastFileEnumeration(tgtPathFmt)+1);
			auto error = writeTargetObjFile(tgtPath, TargetCalibration3D(view.target.contextualRLock()->markers));
			if (error) GetState().errors.push(error.value());
		}

		ImGui::BeginDisabled(view.control.stopping());
		if (view.deleted)
		{
			ImGui::TextUnformatted("Deleted view due to insufficient data!");
		}
		else if (view.control.running())
		{
			if (ImGui::Button(view.control.stopping()? "Stopping..." : "Stop", ButtonSize))
			{
				view.control.stop_source.request_stop();
			}
			ImGui::SameLine();
			switch (view.state.step)
			{
				case TargetView::RECONSTRUCT:
					ImGui::TextUnformatted("Reconstructing Target View...");
					break;
				case TargetView::REEVALUATE_MARKERS:
				case TargetView::TEST_REEVALUATE_MARKERS:
					ImGui::TextUnformatted("Reevaluating Markers Sequences...");
					break;
				case TargetView::OPTIMISE_COARSE:
				case TargetView::OPTIMISE_FINE:
					ImGui::Text("Optimising %d/%d...", view.state.numSteps, view.state.maxSteps);
					break;
				default:
					ImGui::TextUnformatted("Working...");
			}
		}
		else
		{
			if (ImGui::Button("Retry", SizeWidthDiv2()))
			{ // Reconstruct and optimise view again, and if good, even reevaluate to merge markers
				view.selected = false; // Reset, may be automatically selected if next try is good
				// Currently, TEST_REEVALUATE_MARKERS actually checks selected state to test
				view.plan = {
					TargetView::RECONSTRUCT, TargetView::OPTIMISE_COARSE,
					TargetView::TEST_REEVALUATE_MARKERS, TargetView::OPTIMISE_COARSE,
					TargetView::TEST_REEVALUATE_MARKERS, TargetView::OPTIMISE_COARSE,
					TargetView::TEST_REEVALUATE_MARKERS, TargetView::OPTIMISE_COARSE };
				view.planned = true;
			}
			ImGui::SetItemTooltip("Calculates an initial estimate of the target structure and motion.");
			ImGui::BeginDisabled(!view.state.calibrated);
			ImGui::SameLine();
			if (ImGui::Button("Optimise", SizeWidthDiv2()))
			{
				view.plan = { TargetView::OPTIMISE_FINE };
				view.planned = true;
			}
			
			if (ImGui::Button("Reevaluate Markers", SizeWidthDiv2()))
			{
				view.plan = { TargetView::REEVALUATE_MARKERS, TargetView::OPTIMISE_COARSE };
				view.planned = true;
			}
			ImGui::SameLine();
			if (ImGui::Button("Expand Frames", SizeWidthDiv2()))
			{
				view.plan = { TargetView::EXPAND_FRAMES, TargetView::OPTIMISE_COARSE };
				view.planned = true;
			}
			ImGui::EndDisabled();
		}
		ImGui::EndDisabled();

		/* auto writeToCSV = [](std::ofstream &out, const TargetView &view)
		{
			std::stringstream csvData, csvDataStdDev, csvDataMax, csvHeader;
			auto stats = view.stats;
			auto its = view.iterationStates;
			csvHeader << "Sample Count, Frame Count, Marker Count, Min Marker Observations, Min Frame Observations, Min Shared Markers";
			csvData << stats.sampleCount << ", " << stats.frameCount << ", " << stats.markerCount << ", " << stats.minMarkerObs << ", " << stats.minFrameObs << ", " << stats.minSharedMarkers;
			csvHeader << ", Param Count, Data Count, Code";
			csvData << ", " << stats.paramCnt << ", " << stats.dataCnt << ", " << view.calibration.contextualRLock()->lastStopCode;
			csvDataStdDev << ",,,,,,,";
			csvDataMax << ",,,,,,,";

			{
				csvHeader << ", Final Error";
				csvData << ", " << its.back().error.mean*PixelFactor;
				csvDataStdDev << ", " << its.back().error.stdDev*PixelFactor;
				csvDataMax << ", " << its.back().error.max*PixelFactor;
			}

			csvHeader << ", Reconstruction Error";
			csvData << ", " << its[0].error.mean*PixelFactor;
			csvDataStdDev << ", " << its[0].error.stdDev*PixelFactor;
			csvDataMax << ", " << its[0].error.max*PixelFactor;
			for (int i = 1; i < its.size(); i++)
			{
				csvHeader << ", Iteration " << i;
				csvData << ", " << its[i].error.mean*PixelFactor;
				csvDataStdDev << ", " << its[i].error.stdDev*PixelFactor;
				csvDataMax << ", " << its[i].error.max*PixelFactor;
			}

			out << csvHeader.str() << std::endl;
			out << csvData.str() << std::endl;
			out << csvDataStdDev.str() << std::endl;
			out << csvDataMax.str() << std::endl;
		};

		ImGui::SetNextItemWidth(SizeWidthDiv3().x);
		ImGui::Checkbox("Debug", &visState.targetCalib.targetView->debugIterations);

		ImGui::BeginDisabled(!visState.targetCalib.targetView->debugIterations || visState.targetCalib.targetView->iterationStates.empty());
		ImGui::SameLine(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);
		if (ImGui::Button("Write to CSV", SizeWidthDiv3()))
		{
			std::ofstream file("target_view.csv", std::ios::trunc);
			writeToCSV(file, *visState.targetCalib.targetView);
		}
		ImGui::SameLine();
		if (ImGui::Button("Write all to CSV", SizeWidthDiv3()))
		{
			std::ofstream file("target_views.csv", std::ios::trunc);
			for (const auto &view : pipeline.targetCalib.viewRanges)
				writeToCSV(file, *view);
		}
		ImGui::EndDisabled();

		if (visState.targetCalib.targetView->calibrated && visState.targetCalib.targetView->debugIterations)
		{
			auto &its = visState.targetCalib.targetView->iterationStates;
			for (int i = 0; i < its.size(); i++)
			{
				if (i == 0)
					ImGui::Text("Reconstruction Errors: %fpx, max %fpx", its[i].error.mean * PixelFactor, its[i].error.stdDev * PixelFactor);
				else
					ImGui::Text("Iteration %d errors: %fpx, max %fpx", i, its[i].error.mean * PixelFactor, its[i].error.stdDev * PixelFactor);
			}
			ImGui::Text("Current Status Code: %d", calState.lastStopCode);
		} */
	}

	ImGui::PopID();

	bool sectionTargetAssembly = ImGui::CollapsingHeader("Target Assembly (?)", ImGuiTreeNodeFlags_DefaultOpen);
	ImGui::SetItemTooltip("Individual TargetViews are an incomplete, error-prone view of a target.\n"
		"The next phase aims to combine these short segments by correlating marker observations.\n"
		"TargetViews are aligned, some markers merged outright, and the rest adopted and merged as more data becomes available.\n"
		"You can at any point stop automatic assembly, issue own directives, and then resume automatic assembly.\n"
		"You can also manually edit the target (merging/splitting markers, swapping/removing observation sequences).");
	ImGui::PushID("TA");

	if (sectionTargetAssembly)
	{ // Target assembly controls
		auto &assembly = pipeline.targetCalib.assembly;

		int assemblyProgress;
		{ // State of assembly
			assemblyProgress = [&]()
			{
				if (!pipeline.targetCalib.assemblyStages.contextualRLock()->empty()) return 0;
				auto views_lock = pipeline.targetCalib.views.contextualRLock();
				if (views_lock->empty()) return 3;
				int selected = 0;
				for (auto view : *views_lock)
					if (view->selected) selected++;
				if (selected < 2) return 2;
				return 1;
			}();

			if (assemblyProgress == 0)
			{
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualRLock();
				TargetAssemblyBase &base = stages_lock->back()->base;
				ImGui::Text("%ld markers, %.4fpx += %.4fpx, %.4fpx max",
					base.target.markers.size(),
					base.errors.mean*PixelFactor,
					base.errors.stdDev*PixelFactor,
					base.errors.max*PixelFactor);
			}
			else if (assemblyProgress == 1)
				ImGui::TextUnformatted("Start target auto-assembly");
			else if (assemblyProgress == 2)
				ImGui::TextUnformatted("Select at least 2 good Target Views");
			else if (assemblyProgress == 3)
				ImGui::TextUnformatted("Record good views of the target to be calibrated");
		}

		if (assembly.control && assembly.state)
		{ // Assembly thread status + controls
			auto state = assembly.state;
			assert(state);
			ImGui::BeginDisabled(assembly.control->stopping());
			if (ImGui::Button(assembly.control->stopping()? "Stopping...##Stop" : "Stop##Stop", ButtonSize))
			{
				assembly.control->stop_source.request_stop();
			}
			ImGui::SameLine();
			if (assemblyProgress > 0) // No stage yet - still selecting base view
				ImGui::TextUnformatted("Assembling - selecting base view...");
			else if (state->optimising)
				ImGui::Text("%s %d/%d...", state->currentStage.c_str(), state->numSteps, state->maxSteps);
			else
				ImGui::Text("%s...", state->currentStage.c_str());
			ImGui::EndDisabled();
		}

		if (visState.targetCalib.edit && !assembly.control)
		{ // Direct editing tools
			auto &editTarget = visState.targetCalib.edit;
			auto &selection = visState.target.markerSelect;
			auto &markers = editTarget->target.markers;
			int selectCnt = 0;
			for (auto h : selection)
				if (h) selectCnt++;
			unsigned int markerA = -1, markerB = -1;
			if (selectCnt > 0 && selectCnt <= 2)
			{
				for (markerA = 0; markerA < selection.size(); markerA++)
					if (selection[markerA]) break;
				for (markerB = markerA+1; markerB < selection.size(); markerB++)
					if (selection[markerB]) break;
			}
			auto eraseMarker = [&](int marker, int replacement = -1)
			{
				markers.erase(markers.begin()+marker);
				// Update markerMap
				for (auto map = editTarget->target.markerMap.begin(); map != editTarget->target.markerMap.end();)
				{
					if (map->second > marker) map->second--;
					else if (map->second == marker)
					{
						if (replacement >= 0) map->second = replacement;
						else
						{
							map = editTarget->target.markerMap.erase(map);
							continue;
						}
					}
					map++;
				}
				// Update selection
				if (selection[marker]) selectCnt--;
				selection.erase(selection.begin()+marker);
			};

			ImGui::AlignTextToFramePadding();
			ImGui::Text("%d markers", selectCnt);
			SameLinePos(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);
			ImGui::BeginDisabled(selectCnt != 2);
			if (ImGui::Button("Merge##Edit", SizeWidthDiv3()))
			{
				assert(markerB < markers.size());
				// Accumulate marker sample count
				int samplesA = 0, samplesB = 0;
				for (auto &frame : editTarget->target.frames)
				{
					for (auto &sample : frame.samples)
					{
						int m = editTarget->target.markerMap[sample.marker];
						if (m == markerA) samplesA++;
						else if (m == markerB) samplesB++;
					}
				}
				// Update markers
				float weightA = samplesA*samplesA, weightB = samplesB*samplesB;
				Eigen::Vector3f &mkA = markers[markerA], &mkB = markers[markerB];
				mkA = (mkA * weightA + mkB * weightB) / (weightA+weightB);
				// Erase old marker
				eraseMarker(markerB, markerA);
				editTarget->targetCalib = TargetCalibration3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
			}
			ImGui::EndDisabled();
			ImGui::SameLine();
			ImGui::BeginDisabled(selectCnt != 1);
			if (ImGui::Button("Split##Edit", SizeWidthDiv3()))
			{
				assert(markerA < markers.size());
				markerB = markers.size();
				markers.push_back(markers[markerA] + Eigen::Vector3f(0.01f, 0, 0));
				editTarget->targetCalib = TargetCalibration3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
				// Update selection
				selection.push_back(true);
				selectCnt++;
			}
			ImGui::EndDisabled();

			// Show observations of selected markers
			int selObs = visState.targetCalib.selectedObservation;
			visState.targetCalib.selectedObservation = -1;
			visState.targetCalib.highlightedObservation = -1;
			if (selectCnt > 0 && selectCnt <= 2 &&
				ImGui::BeginTable("Observations Table", 1))
			{
				for (auto map = editTarget->target.markerMap.begin(); map != editTarget->target.markerMap.end();)
				{
					if (!selection[map->second])
					{
						map++;
						continue;
					}
					ImGui::PushID(map->first);
					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::Text("Observation %d -> Marker %d", map->first, map->second);

					if (selectCnt == 2)
					{ // Button to swap marker assignment
						SameLineTrailing(GetBarWidth(ImGui::GetFrameHeight(), 2));
						if (ImGui::ArrowButton("Swap", ImGuiDir_Right))
						{
							int oldMarker = map->second;
							if (map->second == markerA)
								map->second = markerB;
							else if (map->second == markerB)
								map->second = markerA;
							if (!std::any_of(editTarget->target.markerMap.begin(), editTarget->target.markerMap.end(),
								[&](auto &m){ return m.second == oldMarker; }))
							{ // Erase marker completely if it has no other observation
								eraseMarker(oldMarker);
							}
							editTarget->targetCalib = TargetCalibration3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
						}
					}

					// Select and highlight observation
					ImGui::SameLine();
					bool selected = selObs == map->first;
					ImGui::Selectable("", &selected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap);
					if (ImGui::IsItemHovered())
						visState.targetCalib.highlightedObservation = map->first;
					if (selected)
						visState.targetCalib.selectedObservation = selObs = map->first;
					
					// Button to delete
					SameLineTrailing(ImGui::GetFrameHeight());
					bool del = CrossButton("Del");
					if (del)
					{ // Perform deletion
						// Clear observation from marker
						for (auto &frame : editTarget->target.frames)
						{
							for (auto sampleIt = frame.samples.begin(); sampleIt != frame.samples.end();)
							{
								if (sampleIt->marker == map->first)
								{
									sampleIt = frame.samples.erase(sampleIt);
									editTarget->target.totalSamples--;
								}
								else sampleIt++;
							}
						}
						// No way to know how many outliers were in that observation to remove from outlierSamples
						// Delete association to observation
						int oldMarker = map->second;
						map = editTarget->target.markerMap.erase(map);
						if (!std::any_of(editTarget->target.markerMap.begin(), editTarget->target.markerMap.end(),
							[&](auto &m){ return m.second == oldMarker; }))
						{ // Erase marker completely if it has no other observation
							eraseMarker(oldMarker);
						}
						// Clear references to observation
						if (visState.targetCalib.highlightedObservation == map->first) visState.targetCalib.highlightedObservation = -1;
						if (selObs == map->first) visState.targetCalib.selectedObservation = selObs = -1;
						editTarget->targetCalib = TargetCalibration3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
					}
					else
						map++;
					ImGui::PopID();
				}
				ImGui::EndTable();
			}

			ImGui::AlignTextToFramePadding();
			ImGui::Text("Editing");
			SameLinePos(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);
			if (ImGui::Button("Discard##Edit", SizeWidthDiv3()))
			{
				visState.targetCalib.edit = nullptr;
				visState.targetCalib.selectedObservation = -1;
				visState.targetCalib.highlightedObservation = -1;
			}
			ImGui::SameLine();
			if (ImGui::Button("Apply##Edit", SizeWidthDiv3()))
			{
				editTarget->errors = getTargetErrorDist(pipeline.getCalibs(), editTarget->target);
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
				stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(*editTarget), STAGE_EDITED, 1, "Edited"));
				visState.targetCalib.edit = nullptr;
				visState.targetCalib.selectedObservation = -1;
				visState.targetCalib.highlightedObservation = -1;
			}
		}
		
		if (!visState.targetCalib.edit && !assembly.control)
		{ // Main assembly directives and tools

			ImGui::BeginDisabled(assemblyProgress > 1);
			if (ImGui::Button("Auto-Assembly", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control);
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
				if (targetDiscardAfterStage >= 0 && targetDiscardAfterStage < stages_lock->size())
					stages_lock->resize(targetDiscardAfterStage);
				assembly.settings = {};
				assembly.planned = true;
			}
			ImGui::SetItemTooltip("Attempts to assembles selected target views into a common target structure.");
			ImGui::EndDisabled();

			ImGui::BeginDisabled(assemblyProgress > 0);

			ImGui::SameLine();

			if (ImGui::Button("Optimise", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control);
				assembly.settings = {};
				assembly.settings.followAlgorithm = false;
				assembly.settings.instructions = { STAGE_OPTIMISATION };
				assembly.settings.maxSteps = pipeline.targetCalib.params.view.manualOptLimitIncrease;
				assembly.settings.tolerances = pipeline.targetCalib.params.view.manualOptTolerance;
				assembly.planned = true;
			}
			ImGui::SetItemTooltip("Start an optimisation on the current target data.");

			if (ImGui::Button("Reevaluate Markers", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control);
				assembly.settings = {};
				assembly.settings.followAlgorithm = false;
				assembly.settings.instructions = { STAGE_REEVALUATE_MARKERS };
				assembly.settings.maxSteps = pipeline.targetCalib.params.assembly.optMaxIt;
				assembly.settings.tolerances = pipeline.targetCalib.params.assembly.optTolerance;
				assembly.planned = true;
			}
			ImGui::SetItemTooltip("Reevaluate observation sequences for each marker.\n"
				"This might add new observation sequences and merge markers with agreeing observations.\n"
				"This operation resets detected outliers.");

			ImGui::SameLine();

			if (ImGui::Button("Expand Frames", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control);
				assembly.settings = {};
				assembly.settings.followAlgorithm = false;
				assembly.settings.instructions = { STAGE_EXPAND_FRAMES };
				assembly.settings.maxSteps = pipeline.targetCalib.params.assembly.optMaxIt;
				assembly.settings.tolerances = pipeline.targetCalib.params.assembly.optTolerance;
				assembly.planned = true;
			}
			ImGui::SetItemTooltip("Expand frames to immediate neighbours by trying to track the target.");


			if (ImGui::Button("Subsample Data", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control);
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
				TargetAssemblyBase base = stages_lock->back()->base;
				auto obs_lock = pipeline.seqDatabase.contextualRLock();
				base.target = subsampleTargetObservations(pipeline.record.frames, obs_lock->markers, base.target, pipeline.targetCalib.params.assembly.subsampling);
				base.errors = getTargetErrorDist(pipeline.getCalibs(), base.target);
				base.targetCalib = TargetCalibration3D(finaliseTargetMarkers(pipeline.getCalibs(), base.target, pipeline.targetCalib.params.post));
				stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(base), STAGE_EDITED, 1, "Subsampled"));
			}
			ImGui::SetItemTooltip("Subsample current target data for all followup stages to reduce computation times.");

			ImGui::SameLine();

			if (ImGui::Button("Determine Outliers", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control);
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
				auto calibs = pipeline.getCalibs();
				TargetAssemblyBase base = stages_lock->back()->base;
				base.errors = getTargetErrorDist(calibs, base.target);
				determineTargetOutliers(calibs, base.target, SigmaToErrors(pipeline.targetCalib.params.assembly.outlierSigmas, base.errors), pipeline.targetCalib.params.aquisition);
				base.errors = getTargetErrorDist(calibs, base.target);
				base.targetCalib = TargetCalibration3D(finaliseTargetMarkers(pipeline.getCalibs(), base.target, pipeline.targetCalib.params.post));
				stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(base), STAGE_EDITED, 1, "Added outliers"));
			}
			ImGui::SetItemTooltip("Determine new outlier samples and remove them from the current target data.");

			if (ImGui::Button("Manually Edit Target", SizeWidthFull()))
			{
				assert(!assembly.planned && !assembly.control);
				visState.resetVisTarget();
				visState.targetCalib.edit = std::make_shared<TargetAssemblyBase>(pipeline.targetCalib.assemblyStages.contextualRLock()->back()->base);
				visState.updateVisTarget();
			}
			ImGui::SetItemTooltip("Edit the latest stage directly to merge/split markers and reassign/remove erroneous observation sequences.");

			ImGui::EndDisabled();
		}

		if (!visState.targetCalib.edit && !assembly.control)
		{ // Assembly stages input
			ImGui::AlignTextToFramePadding();
			ImGui::Text("Stages");
			SameLinePos(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);

			if (ImGui::Button("Discard", SizeWidthDiv3()))
			{
				pipeline.targetCalib.assemblyStages.contextualLock()->clear();
				visState.targetCalib.stage = nullptr;
			}

			ImGui::SameLine();

			if (ImGui::Button("Load Last", SizeWidthDiv3()))
			{
				// Check if we need to load sequence database from checkpoint, too
				if (checkSequencesLoad(pipeline, *pipeline.seqDatabase.contextualLock()))
				{
					// Load assembly stage checkpoint
					const char* obsPathFmt = "dump/assembly_stage_%d.json";
					std::string obsPath = asprintf_s(obsPathFmt, findLastFileEnumeration(obsPathFmt));
					TargetAssemblyBase base;
					auto error = parseTargetAssemblyStage(obsPath, base);
					if (error) GetState().errors.push(error.value());
					else
					{
						auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock(); 
						stages_lock->clear();
						visState.targetCalib.stage = nullptr;
						LOG(LGUI, LInfo, "Loaded %d frames, %d markers, %d sequences",
							(int)base.target.frames.size(), (int)base.target.markers.size(), (int)base.target.markerMap.size());
						auto obs_lock = pipeline.seqDatabase.contextualRLock();
						updateTargetObservations(base.target, obs_lock->markers);
						base.errors = getTargetErrorDist(pipeline.getCalibs(), base.target);
						base.targetCalib = TargetCalibration3D(finaliseTargetMarkers(pipeline.getCalibs(), base.target, pipeline.targetCalib.params.post));
						stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(base), STAGE_LOADED, 1, "Loaded"));
					}
				}
			}
		}

		if (!visState.targetCalib.edit)
		{ // Assembly stages output
			auto getSelectedStage = [&]()
			{
				return visState.targetCalib.stage? visState.targetCalib.stage : pipeline.targetCalib.assemblyStages.contextualRLock()->back();
			};
			ImGui::BeginDisabled(assemblyProgress > 0);

			ImGui::AlignTextToFramePadding();
			ImGui::Text("Save as");
			SameLinePos(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);

			if (ImGui::Button("Target", SizeWidthDiv3()))
			{
				// Find max occupied target id (excluding testing markers because they are negative)
				int id = 0;
				for (auto &tracker : state.trackerConfigs)
					id = std::max(id, tracker.id);
				id++;
				std::string label = std::string("Target ID ") + (char)((int)'0' + id);
				// Take latest assembly stage
				auto stage = getSelectedStage();
				LOG(LTargetCalib, LInfo, "Registered target with %d markers as id %d!\n", (int)stage->base.target.markers.size(), id);
				// Register as calibrated target
				TargetCalibration3D targetCalib(finaliseTargetMarkers(
					pipeline.getCalibs(), stage->base.target, pipeline.targetCalib.params.post));
				state.trackerConfigs.emplace_back(id, label, std::move(targetCalib), TargetDetectionConfig());
				auto error = storeTrackerConfigurations("store/trackers.json", state.trackerConfigs);
				if (error) GetState().errors.push(error.value());
				else state.trackerConfigDirty = state.trackerCalibsDirty = state.trackerIMUsDirty = false;
			}

			ImGui::SameLine();

			if (ImGui::Button("Stage", SizeWidthDiv3()))
			{
				if (ensureSequencesSaved(pipeline, *pipeline.seqDatabase.contextualRLock()))
				{
					const char* obsPathFmt = "dump/assembly_stage_%d.json";
					std::string obsPath = asprintf_s(obsPathFmt, findLastFileEnumeration(obsPathFmt)+1);
					auto error = dumpTargetAssemblyStage(obsPath, getSelectedStage()->base);
					if (error) GetState().errors.push(error.value());
				}
			}

			ImGui::EndDisabled();

		}
	}

	if (sectionTargetAssembly && ImGui::BeginTable("Target Assembly Stage Table", 8,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX | ImGuiTableFlags_RowBg,
		ImVec2(0, ImGui::GetFrameHeight()*10)))
	{
		ImGui::TableSetupColumn("It", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("Views");
		ImGui::TableSetupColumn("Markers");
		ImGui::TableSetupColumn("Samples", ImGuiTableColumnFlags_WidthStretch, 1.5f);
		ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_WidthStretch, 1.5f);
		ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 3.0f);
		ImGui::TableSetupColumn("Update", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("Reset", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableHeadersRow();

		// Prepare default and "marked" background color
		ImVec4 tempBGCol = ImGui::GetStyleColorVec4(ImGuiCol_ChildBg);
		ImGui::PushStyleColor(ImGuiCol_TableRowBg, tempBGCol);
		ImGui::PushStyleColor(ImGuiCol_TableRowBgAlt, tempBGCol);
		tempBGCol.x *= 0.6f;
		tempBGCol.y *= 0.6f;
		tempBGCol.z *= 0.6f;

		int newDiscardPos = targetDiscardAfterStage;
		auto stages_lock = pipeline.targetCalib.assemblyStages.contextualRLock();
		int resize = -1;
		for (int i = 0; i < stages_lock->size(); i++)
		{
			TargetAssemblyStage &stage = *stages_lock->at(i);
			ImGui::PushID(i);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Text("%d", i);
			ImGui::TableNextColumn();
			ImGui::Text("%ld", stage.base.merged.size());
			ImGui::TableNextColumn();
			ImGui::Text("%ld", stage.base.target.markers.size());
			ImGui::TableNextColumn();
			ImGui::Text("%d", stage.base.target.totalSamples);
			ImGui::SetItemTooltip("%d samples, %d inliers, %d outliers",
				stage.base.target.totalSamples+stage.base.target.outlierSamples,
				stage.base.target.totalSamples, stage.base.target.outlierSamples);
			ImGui::TableNextColumn();
			ImGui::Text("%.3fpx", stage.base.errors.rmse*PixelFactor);
			ImGui::SetItemTooltip("%.4fpx RMSE, %.4fpx +- %.4fpx (avg +- std deviation), %.4fpx max - all without outliers",
				stage.base.errors.rmse*PixelFactor,
				stage.base.errors.mean*PixelFactor,
				stage.base.errors.stdDev*PixelFactor,
				stage.base.errors.max*PixelFactor);
			ImGui::TableNextColumn();
			ImGui::Text("%s", stage.label.c_str());
			ImGui::SetItemTooltip("Step ID %d", stage.step);
			ImGui::TableNextColumn();
			ImGui::BeginDisabled(pipeline.targetCalib.assembly.control && pipeline.targetCalib.assembly.control->running());
			if (ImGui::ArrowButton("Update", ImGuiDir_Down))
			{ // Discard this stages when updating
				if (targetDiscardAfterStage == i+1)
					newDiscardPos = -1;
				else
					newDiscardPos = i+1;
			}
			ImGui::SetItemTooltip("Set starting position to this stage, so starting Auto-Assembly will always reset to this stage before continuing. Press again to disable.");
			ImGui::TableNextColumn();
			if (ImGui::ArrowButton("Reset", ImGuiDir_Up))
			{ // Discard later stages
				resize = i+1;
			}
			ImGui::EndDisabled();
			ImGui::SetItemTooltip("Reset assembly to this stage, discarding all stages past this one.");
			ImGui::SameLine();
			bool select = visState.targetCalib.stage == stages_lock->at(i);
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{ // Switch target calib vis to stage
				if (visState.resetVisTarget() && !stage.base.target.frames.empty())
				{
					visState.targetCalib.stage = stages_lock->at(i);
					visState.updateVisTarget();
				}
			}
			if (i == targetDiscardAfterStage)
			{ // Draw with new background from this row
				ImGui::PushStyleColor(ImGuiCol_TableRowBg, tempBGCol);
				ImGui::PushStyleColor(ImGuiCol_TableRowBgAlt, tempBGCol);
			}
			ImGui::PopID();
		}
		ImGui::TableEndRow(ImGui::GetCurrentContext()->CurrentTable);
		if (targetDiscardAfterStage >= 0)
		{
			if (targetDiscardAfterStage < stages_lock->size())
				ImGui::PopStyleColor(2);
		}
		targetDiscardAfterStage = newDiscardPos;
		ImGui::PopStyleColor(2);
		ImGui::EndTable();

		stages_lock.unlock();
		if (resize)
		{
			auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
			if (stages_lock->size() > resize)
				stages_lock->resize(resize);
		}
	}

	if (sectionTargetAssembly && visState.targetCalib.stage && !visState.targetCalib.stage->alignResults.empty()
		&& ImGui::CollapsingHeader("Align Tests"))
	{ // Details of selected stage alignments
		ImGui::PushID("AT");
		if (ImGui::BeginTable("TargetBase Align Tests Table", 4,
			ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
		{
			ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
			ImGui::TableSetupColumn("Matched Markers");
			ImGui::TableSetupColumn("MSE [mm]");
			ImGui::TableSetupColumn("Aligned", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
			ImGui::TableHeadersRow();

			for (int i = 0; i < visState.targetCalib.stage->alignResults.size(); i++)
			{
				auto &test = visState.targetCalib.stage->alignResults[i];
				ImGui::PushID(test.id);
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::Text("%d", test.id);
				ImGui::TableNextColumn();
				ImGui::Text("%ld", test.bestCandidate < 0? 0 : test.candidates[test.bestCandidate].points.size());
				ImGui::TableNextColumn();
				ImGui::Text("%f", test.RMSE);
				ImGui::TableNextColumn();
				ImGui::Text("%s", test.success? "X" : "");
				ImGui::SameLine();
				ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // selected
				ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.2f, 0.2f, 0.2f, 1.0f));
				ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
				bool select = visState.targetCalib.stageSubIndex == i;
				if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
				{
					visState.targetCalib.stageSubIndex = i;
					visState.targetCalib.stageSubSubIndex = test.bestCandidate;
				}
				ImGui::PopStyleColor(3);

				ImGui::PopID();
			}
			ImGui::EndTable();
		}

		if (visState.targetCalib.stageSubIndex >= 0)
		{
			auto &test = visState.targetCalib.stage->alignResults[visState.targetCalib.stageSubIndex];
			ImGui::Text("Alignment Candidates (%ld):", test.candidates.size());
			if (ImGui::BeginTable("TargetBase Merge Candidates Table", 3, //4,
				ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_PadOuterX,
				ImVec2(0, ImGui::GetFrameHeight()*10)))
			{
				ImGui::TableSetupColumn("Num");
				ImGui::TableSetupColumn("Matched Markers");
				ImGui::TableSetupColumn("Best", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
				ImGui::TableHeadersRow();

				for (int i = 0; i < test.candidates.size(); i++)
				{
					auto &cand = test.candidates[i];
					ImGui::PushID(i);
					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					ImGui::Text("%d", i);
					ImGui::TableNextColumn();
					ImGui::Text("%ld", cand.points.size());
					ImGui::TableNextColumn();
					/* ImGui::Text("%f", cand.RMSE);
					ImGui::TableNextColumn(); */
					ImGui::Text("%s", i == test.bestCandidate? "X" : "");
					ImGui::SameLine();
					bool select = visState.targetCalib.stageSubSubIndex == i;
					if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
					{
						visState.targetCalib.stageSubSubIndex = i;
					}
					ImGui::PopID();
				}
				ImGui::EndTable();
			}
		}
		ImGui::PopID();
	}

	if (sectionTargetAssembly && visState.targetCalib.stage && !visState.targetCalib.stage->mergeTests.empty()
		&& ImGui::CollapsingHeader("Merge Tests"))
	{ // Details of selected stage merge attempts
		ImGui::PushID("MT");
		if (ImGui::BeginTable("TargetBase Merge Targets Table", 5,
			ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
		{
			ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
			ImGui::TableSetupColumn("Matched Markers");
			ImGui::TableSetupColumn("New Markers");
			ImGui::TableSetupColumn("Error 2D [px]");
			ImGui::TableSetupColumn("Merged", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
			ImGui::TableHeadersRow();

			for (int i = 0; i < visState.targetCalib.stage->mergeTests.size(); i++)
			{
				auto &test = visState.targetCalib.stage->mergeTests[i];
				ImGui::PushID(test.id);
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::Text("%d", test.id);
				ImGui::TableNextColumn();
				ImGui::Text("%ld", test.pointMap.size());
				ImGui::TableNextColumn();
				ImGui::Text("%ld", test.markers.size() - test.pointMap.size());
				ImGui::TableNextColumn();
				ImGui::Text("%.4fpx", test.error.rmse*PixelFactor);
				ImGui::TableNextColumn();
				ImGui::Text("%s", test.merged? "X" : "");
				ImGui::SameLine();
				ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // selected
				ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.2f, 0.2f, 0.2f, 1.0f));
				ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
				bool select = visState.targetCalib.stageSubIndex == i;
				if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
				{
					visState.targetCalib.stageSubIndex = i;
				}
				ImGui::PopStyleColor(3);

				ImGui::PopID();
			}
			ImGui::EndTable();
		}
		ImGui::PopID();
	}

	ImGui::PopID();

	ImGui::PopStyleColor();
}