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

	if (sectionTargetViews)
	{ // Target Observations to acquire Target Views
		ImGui::Checkbox("Record Target Observations", &pipeline.recordSequences);
		ImGui::SetItemTooltip("Record visible markers into observations as continous sequences.");

		if (ImGui::Button("Clear##Observations", SizeWidthDiv3()))
		{
			// Request all calibration threads to stop if they are still running
			pipeline.targetCalib.assembly.control.stop_source.request_stop();
			for (auto &view : *pipeline.targetCalib.views.contextualRLock())
				view->control.stop_source.request_stop();
			// Wait for threads to stop - not entirely necessary, but else threads will never be deleted
			pipeline.targetCalib.assembly.control.stop();
			// Destructor of views will join and thus block
			// TODO: Make that async somehow? Can't have UI blocking for an optimisation to finish

			// Clean up the rest
			pipeline.targetCalib.views.contextualLock()->clear();
			pipeline.targetCalib.assemblyStages.contextualLock()->clear();
			
			// Clean up UI state
			targetViewsSorted.clear();
			visState.targetCalib.markerSelect.clear();
			visState.targetCalib.edit = nullptr;
			visState.resetVisTarget();

			// Reset observations
			pipeline.seqDatabase.contextualLock()->clear();
			UpdateSequences(true);
			UpdateErrorFromObservations(pipeline);
		}
		ImGui::SameLine();
		if (ImGui::Button("Save##Observations", SizeWidthDiv3()))
		{
			dumpTargetViewRecords("dump/targetViewsDebug.json", *pipeline.targetCalib.views.contextualRLock());
		}
		ImGui::SameLine();
		if (ImGui::Button("Load##Observations", SizeWidthDiv3()))
		{
			auto views_lock = pipeline.targetCalib.views.contextualLock();
			*views_lock = parseTargetViewRecords("dump/targetViewsDebug.json", pipeline.frameRecords);
			for (auto &view : *views_lock)
			{
				{ // Fill target observations and calculate errors
					auto view_lock = view->target.contextualLock();
					updateTargetObservations(*view_lock, pipeline.seqDatabase.contextualRLock()->markers);
					view->state.errors = getTargetErrorDist(pipeline.getCalibs(), *view_lock);
				}
				// Also start optimisation
				view->settings.typeFlags = 0b10; // Optimise only
				view->settings.maxSteps = view->state.numSteps + pipeline.targetCalib.params.view.initialOptLimit;
				view->settings.tolerances = pipeline.targetCalib.params.view.initialOptTolerance;
				view->planned = true;
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
		ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 2.0f);
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
			if (view.control.running() && (view.settings.typeFlags & 0b01))
				ImGui::Text("...");
			else if (view.control.running() && (view.settings.typeFlags & 0b10))
				ImGui::Text("%.3fpx (%d/%d)", view.state.errors.rmse*PixelFactor, view.state.numSteps, view.settings.maxSteps);
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
			bool select = visState.targetCalib.view == viewPtr;
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{
				if (visState.resetVisTarget())
				{
					visState.targetCalib.view = viewPtr;
					visState.updateVisTarget();
				}
			}
			ImGui::SameLine();
			if (CrossButton("Del"))
			{
				if (visState.targetCalib.view == viewPtr)
				{
					visState.targetCalib.view = nullptr;
					visState.targetCalib.markerSelect.clear();
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
				if (ImGui::Button("Auto-Select Base View"))
					pipeline.targetCalib.baseView = nullptr;
			}
			else
			{
				if (ImGui::Button("Pick as Base View"))
					pipeline.targetCalib.baseView = visState.targetCalib.view; // new shared_ptr
			}
		}
		else if (pipeline.targetCalib.baseView == visState.targetCalib.view)
		{
			ImGui::TextUnformatted("View has been selected as Base View.");
		}

		ImGui::SameLine();
		if (ImGui::Button("Save OBJ"))
		{
			const char* tgtPathFmt = "dump/target_%d.obj";
			std::string tgtPath = asprintf_s(tgtPathFmt, findLastFileEnumeration(tgtPathFmt)+1);
			writeTargetObjFile(tgtPath, TargetTemplate3D(view.target.contextualRLock()->markers));
		}

		ImGui::BeginDisabled(view.control.stopping());
		if (view.control.running())
		{
			if (ImGui::Button(view.control.stopping()? "Stopping..." : "Stop", ButtonSize))
			{
				view.control.stop_source.request_stop();
			}
			ImGui::SameLine();
			if (view.settings.typeFlags & 0b01)
				ImGui::TextUnformatted("Reconstructing Target View...");
			else if (view.settings.typeFlags & 0b10)
				ImGui::Text("Optimising %d/%d...", view.state.numSteps, view.settings.maxSteps);
		}
		else
		{
			if (ImGui::Button("Reconstruct", SizeWidthDiv2()))
			{
				view.settings.typeFlags = 0b01; // Reconstruct only
				view.planned = true;
			}
			ImGui::SetItemTooltip("Calculates an initial estimate of the target structure and motion.");
			ImGui::BeginDisabled(!view.state.calibrated);
			ImGui::SameLine();
			if (ImGui::Button("Optimise", SizeWidthDiv2()))
			{
				view.settings.typeFlags = 0b10; // Assembly phases
				view.settings.maxSteps = view.state.numSteps + pipeline.targetCalib.params.view.manualOptLimitIncrease;
				view.settings.tolerances = pipeline.targetCalib.params.view.manualOptTolerance;
				view.planned = true;
			}
			
			if (ImGui::Button("Reevaluate Markers", SizeWidthDiv2()))
			{
				std::vector<CameraCalib> calibs = pipeline.getCalibs();
				auto obs_lock = pipeline.seqDatabase.contextualRLock();
				auto target_lock = visState.targetCalib.view->target.contextualLock();
				reevaluateMarkerSequences<true>(calibs, obs_lock->markers, *target_lock, { 0.5f, 10, 3, 3 });
				reevaluateMarkerSequences<false>(calibs, obs_lock->markers, *target_lock, { 0.5f, 10, 3, 3 });
				updateTargetObservations(*target_lock, obs_lock->markers);
				view.state.errors = getTargetErrorDist(calibs, *target_lock);
				// Plan optimisation
				view.settings.outlierSigma = 10.0f;
				view.settings.typeFlags = 0b10; // Assembly phases
				view.settings.maxSteps = view.state.numSteps + pipeline.targetCalib.params.view.manualOptLimitIncrease;
				view.settings.tolerances = pipeline.targetCalib.params.view.manualOptTolerance;
				view.planned = true;
			}
			ImGui::SameLine();
			if (ImGui::Button("Expand Frames", SizeWidthDiv2()))
			{
				std::vector<CameraCalib> calibs = pipeline.getCalibs();
				auto obs_lock = pipeline.seqDatabase.contextualRLock();
				auto target_lock = visState.targetCalib.view->target.contextualLock();
				TargetTemplate3D trkTarget(finaliseTargetMarkers(calibs, *target_lock, pipeline.targetCalib.params.post));
				expandFrameObservations(calibs, pipeline.frameRecords, *target_lock, trkTarget, pipeline.targetCalib.params.assembly.trackFrame);
				reevaluateMarkerSequences<true>(calibs, obs_lock->markers, *target_lock, { 0.5f, 10, 3, 10 });
				updateTargetObservations(*target_lock, obs_lock->markers);
				view.state.errors = getTargetErrorDist(calibs, *target_lock);
				// Plan optimisation
				view.settings.outlierSigma = 10.0f;
				view.settings.typeFlags = 0b10; // Assembly phases
				view.settings.maxSteps = view.state.numSteps + pipeline.targetCalib.params.view.manualOptLimitIncrease;
				view.settings.tolerances = pipeline.targetCalib.params.view.manualOptTolerance;
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

	bool sectionTargetAssembly = ImGui::CollapsingHeader("Target Assembly (?)", ImGuiTreeNodeFlags_DefaultOpen);
	ImGui::SetItemTooltip("Individual TargetViews are an incomplete, error-prone view of a target.\n"
		"The next phase aims to combine these short segments by correlating marker observations.\n"
		"TargetViews are aligned, some markers merged outright, and the rest adopted and merged as more data becomes available.\n"
		"You can at any point stop automatic assembly, issue own directives, and then resume automatic assembly.\n"
		"You can also manually edit the target (merging/splitting markers, swapping/removing observation sequences).");

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

		if (assembly.control.running())
		{ // Assembly thread status + controls
			ImGui::BeginDisabled(assembly.control.stopping());
			if (ImGui::Button(assembly.control.stopping()? "Stopping...##Stop" : "Stop##Stop", ButtonSize))
			{
				assembly.control.stop_source.request_stop();
			}
			ImGui::SameLine();
			if (assemblyProgress > 0) // No stage yet - still selecting base view
				ImGui::TextUnformatted("Assembling - selecting base view...");
			else if (assembly.state.optimising)
				ImGui::Text("%s %d/%d...", assembly.state.currentStage.c_str(), assembly.state.numSteps, assembly.state.maxSteps);
			else
				ImGui::Text("%s...", assembly.state.currentStage.c_str());
			ImGui::EndDisabled();
		}

		if (visState.targetCalib.edit && !assembly.control.running())
		{ // Direct editing tools
			auto &editTarget = visState.targetCalib.edit;
			auto &selection = visState.targetCalib.markerSelect;
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
				editTarget->targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
			}
			ImGui::EndDisabled();
			ImGui::SameLine();
			ImGui::BeginDisabled(selectCnt != 1);
			if (ImGui::Button("Split##Edit", SizeWidthDiv3()))
			{
				assert(markerA < markers.size());
				markerB = markers.size();
				markers.push_back(markers[markerA] + Eigen::Vector3f(0.01f, 0, 0));
				editTarget->targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
				// Update selection
				selection.push_back(true);
				selectCnt++;
			}
			ImGui::EndDisabled();

			// Show sequences of selected markers
			int selSeq = visState.targetCalib.selectedSequence;
			visState.targetCalib.selectedSequence = -1;
			visState.targetCalib.highlightedSequence = -1;
			if (selectCnt > 0 && selectCnt <= 2 &&
				ImGui::BeginTable("Sequences Table", 1))
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
					ImGui::Text("Sequence %d -> Marker %d", map->first, map->second);

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
							{ // Erase marker completely if it has no other sequence
								eraseMarker(oldMarker);
							}
							editTarget->targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
						}
					}

					// Select and highlight sequence
					ImGui::SameLine();
					bool selected = selSeq == map->first;
					ImGui::Selectable("", &selected, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap);
					if (ImGui::IsItemHovered())
						visState.targetCalib.highlightedSequence = map->first;
					if (selected)
						visState.targetCalib.selectedSequence = selSeq = map->first;
					
					// Button to delete
					SameLineTrailing(ImGui::GetFrameHeight());
					bool del = CrossButton("Del");
					if (del)
					{ // Perform deletion
						// Clear observations of sequence
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
						// No way to know how many outliers were in that sequence to remove from outlierSamples
						// Delete sequence association
						int oldMarker = map->second;
						map = editTarget->target.markerMap.erase(map);
						if (!std::any_of(editTarget->target.markerMap.begin(), editTarget->target.markerMap.end(),
							[&](auto &m){ return m.second == oldMarker; }))
						{ // Erase marker completely if it has no other sequence
							eraseMarker(oldMarker);
						}
						// Clean sequences
						if (visState.targetCalib.highlightedSequence == map->first) visState.targetCalib.highlightedSequence = -1;
						if (selSeq == map->first) visState.targetCalib.selectedSequence = selSeq = -1;
						editTarget->targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline.getCalibs(), editTarget->target, pipeline.targetCalib.params.post));
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
				visState.targetCalib.selectedSequence = -1;
				visState.targetCalib.highlightedSequence = -1;
			}
			ImGui::SameLine();
			if (ImGui::Button("Apply##Edit", SizeWidthDiv3()))
			{
				editTarget->errors = getTargetErrorDist(pipeline.getCalibs(), editTarget->target);
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
				stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(*editTarget), STAGE_EDITED, 1, "Edited"));
				visState.targetCalib.edit = nullptr;
				visState.targetCalib.selectedSequence = -1;
				visState.targetCalib.highlightedSequence = -1;
			}
		}
		
		if (!visState.targetCalib.edit && !assembly.control.running())
		{ // Main assembly directives and tools

			ImGui::BeginDisabled(assemblyProgress > 1);
			if (ImGui::Button("Auto-Assembly", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control.running());
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
				assert(!assembly.planned && !assembly.control.running());
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
				assert(!assembly.planned && !assembly.control.running());
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
				assert(!assembly.planned && !assembly.control.running());
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
				assert(!assembly.planned && !assembly.control.running());
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
				TargetAssemblyBase base = stages_lock->back()->base;
				auto obs_lock = pipeline.seqDatabase.contextualRLock();
				base.target = subsampleTargetObservations(pipeline.frameRecords, obs_lock->markers, base.target, pipeline.targetCalib.params.assembly.subsampling);
				base.errors = getTargetErrorDist(pipeline.getCalibs(), base.target);
				base.targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline.getCalibs(), base.target, pipeline.targetCalib.params.post));
				stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(base), STAGE_EDITED, 1, "Subsampled"));
			}
			ImGui::SetItemTooltip("Subsample current target data for all followup stages to reduce computation times.");

			ImGui::SameLine();

			if (ImGui::Button("Determine Outliers", SizeWidthDiv2()))
			{
				assert(!assembly.planned && !assembly.control.running());
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock();
				auto calibs = pipeline.getCalibs();
				TargetAssemblyBase base = stages_lock->back()->base;
				base.errors = getTargetErrorDist(calibs, base.target);
				determineTargetOutliers(calibs, base.target, SigmaToErrors(pipeline.targetCalib.params.assembly.outlierSigmas, base.errors), pipeline.targetCalib.params.aquisition);
				base.errors = getTargetErrorDist(calibs, base.target);
				base.targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline.getCalibs(), base.target, pipeline.targetCalib.params.post));
				stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(base), STAGE_EDITED, 1, "Added outliers"));
			}
			ImGui::SetItemTooltip("Determine new outlier samples and remove them from the current target data.");

			if (ImGui::Button("Manually Edit Target", SizeWidthFull()))
			{
				assert(!assembly.planned && !assembly.control.running());
				visState.resetVisTarget();
				visState.targetCalib.edit = std::make_shared<TargetAssemblyBase>(pipeline.targetCalib.assemblyStages.contextualRLock()->back()->base);
				visState.updateVisTarget();
			}
			ImGui::SetItemTooltip("Edit the latest stage directly to merge/split markers and reassign/remove erroneous observation sequences.");

			ImGui::EndDisabled();
		}

		if (!visState.targetCalib.edit && !assembly.control.running())
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
				const char* obsPathFmt = "dump/assembly_stage_%d.json";
				std::string obsPath = asprintf_s(obsPathFmt, findLastFileEnumeration(obsPathFmt));
				TargetAssemblyBase base;
				bool success = parseTargetAssemblyStage(obsPath, base);
				auto stages_lock = pipeline.targetCalib.assemblyStages.contextualLock(); 
				stages_lock->clear();
				visState.targetCalib.stage = nullptr;
				if (success)
				{
					LOG(LGUI, LInfo, "Loaded %d frames, %d markers, %d sequences",
						(int)base.target.frames.size(), (int)base.target.markers.size(), (int)base.target.markerMap.size());
					auto obs_lock = pipeline.seqDatabase.contextualRLock();
					updateTargetObservations(base.target, obs_lock->markers);
					base.errors = getTargetErrorDist(pipeline.getCalibs(), base.target);
					base.targetTemplate = TargetTemplate3D(finaliseTargetMarkers(pipeline.getCalibs(), base.target, pipeline.targetCalib.params.post));
					stages_lock->push_back(std::make_shared<TargetAssemblyStage>(std::move(base), STAGE_LOADED, 1, "Loaded"));
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
				int id = -1;
				for (int i = 0; i < pipeline.tracking.targetTemplates3D.size(); i++)
					id = std::max(id, pipeline.tracking.targetTemplates3D[i].id);
				id++;
				// Register latest assembly stage as tracking target
				auto stage = getSelectedStage();
				TargetTemplate3D targetTemplate(id, std::string("Target ID ") + (char)((int)'0' + id), 
					finaliseTargetMarkers(pipeline.getCalibs(), stage->base.target, pipeline.targetCalib.params.post));
				pipeline.tracking.targetTemplates3D.push_back(std::move(targetTemplate));
				LOG(LTargetCalib, LInfo, "Registered target with %d markers as id %d!\n", (int)targetTemplate.markers.size(), id);
				ServerStoreTargetCalib(state);
			}

			ImGui::SameLine();

			if (ImGui::Button("Stage", SizeWidthDiv3()))
			{
				const char* obsPathFmt = "dump/assembly_stage_%d.json";
				std::string obsPath = asprintf_s(obsPathFmt, findLastFileEnumeration(obsPathFmt)+1);
				dumpTargetAssemblyStage(obsPath, getSelectedStage()->base);
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
			ImGui::TableNextColumn();
			ImGui::BeginDisabled(pipeline.targetCalib.assembly.control.running());
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
		ImGui::TreePop();
	}

	if (sectionTargetAssembly && visState.targetCalib.stage && !visState.targetCalib.stage->mergeTests.empty()
		&& ImGui::CollapsingHeader("Merge Tests"))
	{ // Details of selected stage merge attempts
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
		ImGui::TreePop();
	}

	ImGui::PopStyleColor();
}