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

#define IMGUI_DEFINE_MATH_OPERATORS
#include "ui.hpp"

#include "pipeline/pipeline.hpp"

#include "imgui/imgui_onDemand.hpp"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include "nativefiledialog-extended/nfd.h"
#include "nativefiledialog-extended/nfd_glfw3.h"

#include <filesystem>
#include <numeric>

void InterfaceState::UpdateTrackers(InterfaceWindow &window)
{
	auto discardSelection = []
	{
		GetUI().visState.target.selectedTrackerID = 0;
		if (GetUI().visState.target.inspectingSource == 'T')
			GetUI().visState.resetVisTarget();
	};
	if (!window.open)
	{
		discardSelection();
		return;
	}
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		discardSelection();
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	const char* objImportExportGuide =
		"Import/Export this .obj in e.g. Blender with Y as Forward Axis and Z as Up Axis.\n"
		"Mind the Vertex Groups, they mark which faces to use and store the label.\n"
		"That way, you can demark individual faces of a tracker model as markers.";

	// Popup to import target from .obj
	const ImGuiID loadObjPopupID = ImGui::GetID("ImportTargetObj");
	static struct
	{
		int trackerID = 0;
		std::string path;
		bool markerMerge = true;
		const float defaultMarkerFoV = 180, defaultMarkerSize = 6;
		float markerFoV = defaultMarkerFoV, markerSize = defaultMarkerSize;
	} loadObj;

	ImGui::SetNextWindowSize(ImVec2(30*ImGui::GetFontSize(), -1), ImGuiCond_Appearing);
	if (BeginPopup(loadObjPopupID))
	{
		ImGui::AlignTextToFramePadding();
		if (std::filesystem::path(loadObj.path).has_filename())
			ImGui::Text("%s", std::filesystem::path(loadObj.path).filename().generic_string().c_str());
		else
			ImGui::Text("%s", loadObj.path.c_str());
		ImGui::SetItemTooltip("%s", loadObj.path.c_str());
		SameLineTrailing(SizeWidthDiv4().x);
		if (ImGui::Button("Select", SizeWidthDiv4()))
		{
			threadPool.push([](int id)
			{
				if (!NFD_Init())
				{ // Thread-specific init
					SignalErrorToUser(asprintf_s("Failed to initialise File Picker: %s", NFD_GetError()));
					return;
				}

				const int filterLen = 1;
				nfdfilteritem_t filterList[filterLen] = {
					{"Object Target", "obj"},
				};
				nfdchar_t *outPath;
				std::string defPath = std::filesystem::absolute(temporaryStoreFolder);
				nfdopendialogu8args_t args;
				args.filterList = filterList;
				args.filterCount = filterLen;
				args.defaultPath = defPath.c_str();
				NFD_GetNativeWindowFromGLFWWindow(GetUI().glfwWindow, &args.parentWindow);
				nfdresult_t result = NFD_OpenDialogU8_With(&outPath, &args);
				if (result == NFD_OKAY)
				{
					loadObj.path = outPath;
					NFD_FreePath(outPath);
				}
				else if (result == NFD_ERROR)
				{
					SignalErrorToUser(asprintf_s("Failed to use File Picker: %s", NFD_GetError()));
				}

				NFD_Quit();

				GetUI().RequestUpdates();
			});
		}

		ScalarProperty("Marker FoV", "°", &loadObj.markerFoV, &loadObj.defaultMarkerFoV, 0.0f, 360.0f);
		ScalarProperty("Marker Size", "mm", &loadObj.markerSize, &loadObj.defaultMarkerSize, 0.1f, 100.0f);

		if (loadObj.trackerID != 0)
		{
			BooleanProperty("Merge Markers", &loadObj.markerMerge, nullptr);
		}

		ImGui::TextUnformatted(objImportExportGuide);

		if (ImGui::Button("Cancel", SizeWidthDiv2()))
			ImGui::CloseCurrentPopup();
		ImGui::SameLine();
		if (ImGui::Button(loadObj.trackerID == 0? "Load as New" : "Load and Overwrite", SizeWidthDiv2()))
		{
			std::map<std::string, TargetCalibration3D> targets;
			auto error = parseTargetObjFile(loadObj.path, targets, 180, 5);
			if (error)
				SignalErrorToUser(error.value());
			else if (loadObj.trackerID != 0)
			{
				if (targets.size() > 1)
					SignalErrorToUser("Object file contains multiple targets! Will not override.");
				else if (targets.size() == 0)
					SignalErrorToUser("Object file contains no targets!");
				else if (loadObj.markerMerge)
				{
					auto &loadCalib = targets.begin()->second;
					auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
						[&](auto &t){ return t.id == loadObj.trackerID; });
					if (trackerIt == state.trackerConfigs.end())
					{
						SignalErrorToUser(asprintf_s("Could not find tracker by ID %d to import markers into!", loadObj.trackerID));
					}
					else
					{
						auto &refCalib = trackerIt->calib;
						auto &params = state.pipeline.targetCalib.params.assembly;

						// Prepare interface for target detection
						std::vector<TriangulatedPoint> triPoints;
						triPoints.reserve(refCalib.markers.size());
						for (const auto &mk : refCalib.markers)
							triPoints.push_back(TriangulatedPoint(mk.pos, params.alignPointError/1000, 10.0f));
						std::vector<int> triIndices(triPoints.size());
						std::iota(triIndices.begin(), triIndices.end(), 0);

						// Match markers between the two targets
						auto match = detectTarget3D(loadCalib, triPoints, triIndices, params.alignPointSigma, params.alignPoseSigma, false);

						// Copy marker parameters for matched markers
						for (int m = 0; m < match.pointMap.size(); m++)
						{
							if (match.pointMap[m] < 0) continue;
							auto &mkRef = refCalib.markers[m];
							auto &mkLoad = loadCalib.markers[match.pointMap[m]];
							mkLoad.size = mkRef.size;
							mkLoad.viewAngle = mkRef.viewAngle;
						}
					}
					SignalTargetCalibUpdate(loadObj.trackerID, loadCalib);
				}
				else
				{
					SignalTargetCalibUpdate(loadObj.trackerID, targets.begin()->second);
				}
			}
			else
			{ // Import as new target
				auto selectNewID = [&]()
				{ // Find max occupied target id
					int id = 0;
					for (auto &tracker : state.trackerConfigs)
						id = std::max(id, tracker.id);
					return id + 1;
				};
				for (auto &target : targets)
					state.trackerConfigs.push_back(TrackerConfig(selectNewID(), target.first, std::move(target.second), TargetDetectionConfig()));
			}
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}

	bool isDirty = false;
	for (auto &tracker : state.trackerConfigs)
		isDirty |= tracker.configDirty | tracker.targetDirty;
	if (SaveButton("Save Tracker Configuration", SizeWidthFull(), isDirty))
	{
		auto error = storeTrackerConfigurations(trackerConfigFolder, state.trackerConfigs);
		if (error) SignalErrorToUser(error.value());
	}

	if (ImGui::Button("Import Target from OBJ", SizeWidthFull()))
	{
		loadObj.trackerID = 0;
		ImGui::OpenPopup(loadObjPopupID);
	}

	if (ImGui::Button("Add Virtual Tracker", SizeWidthFull()))
	{ // Import as new target
		auto selectNewID = [&]()
		{ // Find max occupied target id
			int id = 0;
			for (auto &tracker : state.trackerConfigs)
				id = std::max(id, tracker.id);
			return id + 1;
		};
		int newID = selectNewID();
		state.trackerConfigs.push_back(TrackerConfig(newID, asprintf_s("Target %d", newID), TrackerConfig::TRACKER_VIRTUAL));
		visState.target.selectedTrackerID = newID;
	}

	if (!ImGui::BeginChild("EditChild", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding))
	{
		discardSelection();
		ImGui::EndChild();
		ImGui::End();
		return;
	}

	// Popup to delete tracker from database
	const ImGuiID delTrackerPopupID = ImGui::GetID("ConfirmDeleteTracker");
	static int delTrackerID = 0;
	static std::string delTrackerLabel;

	/* static float childSize = 100; // Auto-Scale to fit view and keep bottom control visible
	childSize = std::max(CalcTableHeight(3), childSize + GetWindowContentRegionHeight() - GetWindowActualContentHeight());
	childSize = std::min(CalcTableHeight(state.trackerConfigs.size()), childSize); */
	float childSize = CalcTableHeight(5); // Adaptive sizing adapts TOO well to differences in UI
	if (ImGui::BeginTable("Tracker", 5,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX | ImGuiTableFlags_ScrollY,
		ImVec2(0, childSize)))
	{
		ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("Detail", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("Change", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("Del", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		//ImGui::TableHeadersRow();

		for (auto &tracker : state.trackerConfigs)
		{
			ImGui::PushID(tracker.id);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::AlignTextToFramePadding();

			bool select = visState.target.selectedTrackerID == tracker.id;
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{
				if (visState.target.inspectingSource == 'T' && visState.target.inspectingTrackerID == visState.target.selectedTrackerID)
					visState.resetVisTarget();
				if (visState.target.selectedTrackerID == tracker.id)
					visState.target.selectedTrackerID = 0;
				else
					visState.target.selectedTrackerID = tracker.id;
			}
			ImGui::SameLine(0, 0);

			ImGui::Text("%d", tracker.id);
			ImGui::TableNextColumn();

			ImGui::Text("'%s'", tracker.label.c_str());
			ImGui::TableNextColumn();

			if (tracker.type == TrackerConfig::TRACKER_TARGET)
				ImGui::Text("Target with %d markers", (int)tracker.calib.markers.size());
			else if (tracker.type == TrackerConfig::TRACKER_MARKER)
				ImGui::Text("Marker of size %.1fmm", tracker.markerSize*1000.0f);
			else if (tracker.type == TrackerConfig::TRACKER_VIRTUAL)
				ImGui::Text("Virtual Tracker (%d)", (int)tracker.virtConfig.ids.size());
			else
			 	ImGui::TextUnformatted("Tracker of unknown type.");

			ImGui::TableNextColumn();

			if (tracker.configDirty || tracker.targetDirty)
			{
				if (BeginIconDropdown("Reload", icons().visual, iconSize(), ImGuiComboFlags_PopupAlignLeft))
				{
					if (ImGui::MenuItem("Reload Tracker"))
					{
						std::optional<TrackerConfig> trackerConfig;
						auto error = parseTrackerConfiguration(trackerConfigFolder, tracker.id, trackerConfig);
						if (error) SignalErrorToUser(error.value());
						else if (trackerConfig)
						{
							tracker = trackerConfig.value();
							bool updatedIMU = ServerUpdateTrackerIMU(state, tracker);
							ServerUpdateTrackerConditions(state, tracker, true);
							ServerUpdateTrackerConfig(state, tracker, updatedIMU);
						}
					}
					ImGui::EndCombo();
				}
				if (ImGui::BeginItemTooltip())
				{
					if (tracker.configDirty)
						ImGui::TextUnformatted("Tracker Config has been changed.");
					if (tracker.targetDirty)
						ImGui::TextUnformatted("Tracker Calibration has been changed.");
					ImGui::EndTooltip();
				}
			}
			ImGui::TableNextColumn();

			if (CrossButton("Del"))
			{
				delTrackerID = tracker.id;
				delTrackerLabel = tracker.label;
				ImGui::OpenPopup(delTrackerPopupID);
			}
			ImGui::PopID();
		}

		ImGui::EndTable();
	}

	if (BeginPopup(delTrackerPopupID))
	{
		ImGui::Text("Delete tracker '%s' (%d) from disk immediately (No Undo)?", delTrackerLabel.c_str(), delTrackerID);
		if (SaveButton("Delete", SizeWidthDiv2(), true))
		{
			std::filesystem::path trackerFolder = std::filesystem::path(trackerConfigFolder),
				trackerFile = trackerFolder / asprintf_s("tracker_%d.json", delTrackerID),
				targetFile = trackerFolder / asprintf_s("target_%d.json", delTrackerID);
			if (std::filesystem::is_regular_file(trackerFile))
				std::filesystem::remove(trackerFile);
			if (std::filesystem::is_regular_file(targetFile))
				std::filesystem::remove(targetFile); // Only for target type trackers
			// Remove from database
			auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
				[&](auto &t){ return t.id == delTrackerID; });
			if (trackerIt == state.trackerConfigs.end())
				SignalErrorToUser(asprintf_s("Failed to find target '%s' (%d) in database!", delTrackerLabel.c_str(), delTrackerID));
			else
				state.trackerConfigs.erase(trackerIt);
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel", SizeWidthDiv2()))
		{
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
	else
	{
		delTrackerID = 0;
		delTrackerLabel.clear();
	}

	auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
		[&](auto &t){ return t.id == visState.target.selectedTrackerID; });
	if (trackerIt == state.trackerConfigs.end())
	{
		discardSelection();
		ImGui::EndChild();
		ImGui::End();
		return;
	}
	auto &tracker = *trackerIt;

	{
		BeginSection("Selected Tracker");

		bool changed = false;

		ImGui::SetNextItemWidth(LineWidthRemaining());
		changed |= ImGui::InputText("##TrkLabel", &tracker.label);

		ImGui::AlignTextToFramePadding();
		BeginLabelledGroup("Trigger Conditions");
		std::array<const char*,7> triggerLabels;
		triggerLabels.fill("Invalid State");
		triggerLabels[TrackerConfig::TRIGGER_BY_DEFAULT] = "By Default";
		triggerLabels[TrackerConfig::TRIGGER_ONLY_MANUALLY] = "Only Manually";
		triggerLabels[TrackerConfig::TRIGGER_ON_IMU_CONNECT] = "On IMU Connect";
		triggerLabels[TrackerConfig::TRIGGER_ON_IO_CONNECT] = "On IO Connect";
		triggerLabels[TrackerConfig::TRIGGER_ON_IMU_CONNECT | TrackerConfig::TRIGGER_ON_IO_CONNECT] = "On IMU or IO Connect";
		if (ImGui::BeginCombo("##TrgCond", triggerLabels[tracker.trigger]))
		{
			bool updated = false;
			if (ImGui::Selectable("By Default", tracker.trigger == TrackerConfig::TRIGGER_BY_DEFAULT))
			{
				updated = tracker.trigger != TrackerConfig::TRIGGER_BY_DEFAULT;
				tracker.trigger = TrackerConfig::TRIGGER_BY_DEFAULT;
			}
			if (ImGui::Selectable("Only Manually", tracker.trigger == TrackerConfig::TRIGGER_ONLY_MANUALLY))
			{
				updated = tracker.trigger != TrackerConfig::TRIGGER_ONLY_MANUALLY;
				tracker.trigger = TrackerConfig::TRIGGER_ONLY_MANUALLY;
			}
			bool triggerIMU = tracker.trigger & TrackerConfig::TRIGGER_ON_IMU_CONNECT;
			if (ImGui::Selectable("On IMU Connect", triggerIMU))
			{
				updated = !triggerIMU;
				if (triggerIMU)
					tracker.trigger = (TrackerConfig::TrackerTrigger)((tracker.trigger & TrackerConfig::TRIGGER_FLAG_MASK) & ~TrackerConfig::TRIGGER_ON_IMU_CONNECT);
				else
					tracker.trigger = (TrackerConfig::TrackerTrigger)((tracker.trigger & TrackerConfig::TRIGGER_FLAG_MASK) | TrackerConfig::TRIGGER_ON_IMU_CONNECT);
			}
			bool triggerIO = tracker.trigger & TrackerConfig::TRIGGER_ON_IO_CONNECT;
			if (ImGui::Selectable("On IO Connect", triggerIO))
			{
				updated = !triggerIO;
				if (triggerIO)
					tracker.trigger = (TrackerConfig::TrackerTrigger)((tracker.trigger & TrackerConfig::TRIGGER_FLAG_MASK) & ~TrackerConfig::TRIGGER_ON_IO_CONNECT);
				else
					tracker.trigger = (TrackerConfig::TrackerTrigger)((tracker.trigger & TrackerConfig::TRIGGER_FLAG_MASK) | TrackerConfig::TRIGGER_ON_IO_CONNECT);
			}
			ImGui::EndCombo();
			if (updated)
				ImGui::MarkItemEdited(ImGui::GetItemID());
			changed |= updated;
		}
		ImGui::EndGroup();
		ImGui::SetItemTooltip("Conditions to trigger a tracker. After that it is expected to be in the tracking volume and actively searched for.");

		ImGui::AlignTextToFramePadding();
		BeginLabelledGroup("Expose Conditions");
		const TrackerConfig::TrackerExpose exposeMap[] = { TrackerConfig::EXPOSE_BY_DEFAULT, TrackerConfig::EXPOSE_ONCE_TRIGGERED, TrackerConfig::EXPOSE_ONCE_TRACKED };
		std::array<const char*,3> exposeLabel;
		exposeLabel[TrackerConfig::EXPOSE_BY_DEFAULT] = "By Default";
		exposeLabel[TrackerConfig::EXPOSE_ONCE_TRIGGERED] = "Once Triggered";
		exposeLabel[TrackerConfig::EXPOSE_ONCE_TRACKED] = "Once Tracked";
		int exposeCond = tracker.expose;
		if (ImGui::Combo("##ExpCond", &exposeCond, exposeLabel.data(), 3))
		{
			tracker.expose = (TrackerConfig::TrackerExpose)exposeCond;
			changed = true;
		}
		ImGui::EndGroup();
		ImGui::SetItemTooltip("Conditions to expose the tracker on IO. After that, external clients can see/connect to the tracker.");

		if (state.isStreaming)
		{ // Could remove these, this can be done in the integrations window and pipeline tracking UI, too
			ImGui::BeginDisabled(tracker.triggered);
			if (ImGui::Button("Trigger Manually", SizeWidthDiv2()))
			{ // Manually trigger tracker and notify pipeline
				ServerUpdateTrackerConditions(state, tracker, false, 1, 0);
			}
			ImGui::EndDisabled();
			ImGui::SameLine();
			ImGui::BeginDisabled(tracker.exposed);
			if (ImGui::Button("Expose Manually", SizeWidthDiv2()))
			{ // Manually expose tracker, CheckTrackingIO will update integrations accordingly
				ServerUpdateTrackerConditions(state, tracker, false, 0, 1);
			}
			ImGui::EndDisabled();
		}

		if (changed)
		{
			tracker.configDirty = true;
			ServerUpdateTrackerConfig(state, tracker);
		}

		EndSection();
	}

	if (tracker.type == TrackerConfig::TRACKER_VIRTUAL)
	{
		auto trackerLabel = [&state](int trackerID, const char* none = nullptr)
		{
			auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
					[&](auto &t){ return t.id == trackerID; });
			return trackerIt == state.trackerConfigs.end()? (none? none : "None") : trackerIt->label;
		};

		auto targetSelector = [&](const char* label, int &trackerID, const char* noneOption = nullptr)
		{
			bool changed = false;
			if (ImGui::BeginCombo(label, trackerLabel(trackerID, noneOption).c_str()))
			{
				if (noneOption && ImGui::Selectable(noneOption, trackerID < 0))
				{
					changed = true;
					trackerID = -1;
				}
				for (const auto &trackerConfig : state.trackerConfigs)
				{
					if (trackerConfig.type != TrackerConfig::TRACKER_TARGET) continue;
					if (!ImGui::Selectable(trackerConfig.label.c_str(), trackerID == trackerConfig.id)) continue;
					changed = true;
					trackerID = trackerConfig.id;
				}
				ImGui::EndCombo();
				if (changed)
					ImGui::MarkItemEdited(ImGui::GetItemID());
			}
			return changed;
		};

		auto subtrackerSelector = [&](const char* label, int &subtrackerIdx, const std::vector<int> &subtrackers, const char* noneOption = nullptr)
		{
			int trackerID = subtrackerIdx < 0 || subtrackerIdx >= subtrackers.size()? -1 : subtrackers[subtrackerIdx];

			bool changed = false;
			if (ImGui::BeginCombo(label, trackerLabel(trackerID, noneOption).c_str()))
			{
				if (noneOption && ImGui::Selectable(noneOption, subtrackerIdx < 0))
				{
					changed = true;
					subtrackerIdx = -1;
				}
				for (int t = 0; t < subtrackers.size(); t++)
				{
					if (!ImGui::Selectable(trackerLabel(subtrackers[t]).c_str(), subtrackerIdx == t)) continue;
					changed = true;
					subtrackerIdx = t;
				}
				ImGui::EndCombo();
				if (changed)
					ImGui::MarkItemEdited(ImGui::GetItemID());
			}
			return changed;
		};

		auto axisLabel = [](const TrackerAxis axis)
		{
			const char *axisLabel[] = { "X", "Y", "Z", "INVALID" };
			return asprintf_s("Axis %s%s", axis & TrackerAxis::AXIS_SIGN? "-" : "+", axisLabel[axis & TrackerAxis::AXIS_MASK]);;
		};

		auto axisSelector = [&](const char* label, TrackerAxis &axis)
		{
			bool changed = false;
			if (ImGui::BeginCombo(label, axisLabel(axis).c_str()))
			{
				for (int s = 0; s <= TrackerAxis::AXIS_SIGN; s += TrackerAxis::AXIS_SIGN)
				{
					for (int a = 0; a < 3; a++)
					{
						TrackerAxis axisOpt = (TrackerAxis)(a | s);
						if (ImGui::Selectable(axisLabel(axisOpt).c_str(), axis == axisOpt))
						{
							changed = true;
							axis = axisOpt;
						}
					}
				}
				ImGui::EndCombo();
				if (changed)
					ImGui::MarkItemEdited(ImGui::GetItemID());
			}
			return changed;
		};

		BeginSection("Virtual Target");
		bool virtConfigChanged = false;
		auto &config = tracker.virtConfig;

		ImGui::AlignTextToFramePadding();
		ImGui::Text("Subtrackers");
		SameLineTrailing();
		if (PlusButton("AddSubTrk"))
		{
			config.ids.push_back(-1);
			virtConfigChanged = true;
		}
		ImGui::SetItemTooltip("Add a new subtracker entry.");

		if (ImGui::BeginTable("Subtrackers", 2))
		{
			ImGui::TableSetupColumn("Select", ImGuiTableColumnFlags_WidthStretch, 1);
			ImGui::TableSetupColumn("Edit", ImGuiTableColumnFlags_WidthFixed, GetBarWidth(ImGui::GetFrameHeight(), 1));

			for (int i = 0; i < config.ids.size(); i++)
			{
				ImGui::PushID(i); // May technically be used already if last entry was deleted
				ImGui::PushID(config.ids[i]);
				ImGui::TableNextRow();
				ImGui::TableNextColumn();

				ImGui::SetNextItemWidth(LineWidthRemaining());
				virtConfigChanged |= targetSelector("##Tracker", config.ids[i]);

				ImGui::TableNextColumn();

				if (CrossButton("Delete"))
				{
					config.ids.erase(config.ids.begin()+i);
					virtConfigChanged = true;
					i--;
				}
				ImGui::SetItemTooltip("Delete the subtracker entry.");

				ImGui::PopID();
				ImGui::PopID();
			}
			ImGui::EndTable();
		}

		if (ImGui::TreeNode("Position (Center)"))
		{
			ImGui::Text("Weighted Average Center");
			float defaultWeight = 1;
			config.centerWeights.resize(config.ids.size(), defaultWeight);
			std::vector<float> defaultWeights(config.ids.size(), defaultWeight);
			for (int i = 0; i < config.ids.size(); i++)
			{
				std::string label = asprintf_s("Subtracker %s##%d", trackerLabel(config.ids[i]).c_str(), i);
				virtConfigChanged |= ScalarProperty<float>(label.c_str(), nullptr, &config.centerWeights[i], &defaultWeight, 0, 100, 0);
			}

			virtConfigChanged |= ScalarFieldsN<float>("###Offset", "mm", 3, config.centerOffset.data(), nullptr, -1000, 1000, 1000, "%.1f");

			ImGui::TreePop();	
		}

		if (ImGui::TreeNode("Rotation (Alignment)"))
		{
			{
				virtConfigChanged |= subtrackerSelector("Copy Rotation", config.copyRotationFromTracker, config.ids, "Don't Copy");
			}

			ImGui::BeginDisabled(config.copyRotationFromTracker >= 0);

			{
				ImGui::AlignTextToFramePadding();
				ImGui::Text("Copy Axis (Average)");
				SameLineTrailing();
				if (PlusButton("AddAxis"))
				{
					config.copyAxis.sources.push_back({});
					virtConfigChanged = true;
				}

				if (ImGui::BeginTable("CopyAxis", 2))
				{
					ImGui::TableSetupColumn("Select", ImGuiTableColumnFlags_WidthStretch, 1);
					ImGui::TableSetupColumn("Edit", ImGuiTableColumnFlags_WidthFixed, GetBarWidth(ImGui::GetFrameHeight(), 1));

					for (int i = 0; i < config.copyAxis.sources.size(); i++)
					{
						ImGui::PushID(i); // May technically be used already if last entry was deleted
						ImGui::TableNextRow();
						ImGui::TableNextColumn();

						ImGui::SetNextItemWidth(SizeWidthDiv2().x);
						virtConfigChanged |= subtrackerSelector("##Trk", config.copyAxis.sources[i].tracker, config.ids);

						ImGui::SameLine();

						ImGui::SetNextItemWidth(SizeWidthDiv2().x);
						virtConfigChanged |= axisSelector("##Axis", config.copyAxis.sources[i].axis);

						ImGui::TableNextColumn();

						if (CrossButton("Delete"))
						{
							config.copyAxis.sources.erase(config.copyAxis.sources.begin()+i);
							virtConfigChanged = true;
							i--;
						}

						ImGui::PopID();
					}
					ImGui::EndTable();
				}

				ImGui::BeginDisabled(config.copyAxis.sources.empty());
				virtConfigChanged |= axisSelector("Target Axis", config.copyAxis.axis);
				ImGui::EndDisabled();
			}

			{
				ImGui::AlignTextToFramePadding();
				ImGui::Text("Align Axis");

				ImGui::SetNextItemWidth(SizeWidthFull().x);
				virtConfigChanged |= subtrackerSelector("Align Target", config.alignAxis.tracker, config.ids, "Don't Align");

				ImGui::BeginDisabled(config.alignAxis.tracker < 0);
				virtConfigChanged |= axisSelector("Aligned Axis", config.alignAxis.axis);
				ImGui::EndDisabled();
			}

			ImGui::EndDisabled();

			ImGui::TreePop();
		}

		if (virtConfigChanged)
		{
			tracker.configDirty = true;
			ServerUpdateTrackerConfig(state, tracker);
		}

		EndSection();
	}

	if (tracker.type == TrackerConfig::TRACKER_TARGET)
	{
		BeginSection("Target Calibration");

		if (visState.target.inspectingTrackerID == tracker.id)
		{
			if (ImGui::Button("Stop###Inspect", SizeWidthFull()))
				visState.resetVisTarget();
		}
		else if (ImGui::Button("Inspect", SizeWidthFull()))
		{
			if (visState.resetVisTarget())
			{
				visState.target.inspectingTrackerID = tracker.id;
				visState.target.inspectingSource = 'T';
				visState.updateVisTarget();
			}
		}

		static float adjustScale = 1.0f;
		ImGui::SetNextItemWidth(SizeWidthDiv2().x);
		ImGui::InputFloat("##Scale", &adjustScale, 0.0f, 0.0f, "%.8f");
		ImGui::SameLine();
		if (ImGui::Button("Adjust scale", SizeWidthDiv2()))
		{
			LOG(LTargetCalib, LInfo, "Adjusting scale of target '%s' by %f!", tracker.label.c_str(), adjustScale);
			TargetCalibration3D calib = tracker.calib;
			for (auto &mk : calib.markers)
				mk.pos *= adjustScale;
			calib.updateMarkers();
			auto obs_lock = state.pipeline.obsDatabase.contextualLock();
			for (auto &tgt : obs_lock->targets)
			{
				if (tgt.trackerID != visState.target.inspectingTrackerID) continue;
				for (auto &mk : tgt.markers)
					mk *= adjustScale;
			}
			adjustScale = 1.0f/adjustScale;
			// Signal server to update calib
			SignalTargetCalibUpdate(tracker.id, calib);
		}

		ImGui::SetNextItemWidth(SizeWidthDiv2().x);
		ImGui::SliderFloat("##ViewAngle", &visState.target.adjViewAngle, -2.0f, 2.0f);
		ImGui::SameLine();
		if (ImGui::Button("Adjust View Angle", SizeWidthDiv2()))
		{
			LOG(LTargetCalib, LInfo, "Adjusting marker view angle of target '%s' by %f!", tracker.label.c_str(), visState.target.adjViewAngle);
			TargetCalibration3D calib = tracker.calib;
			for (auto &mk : calib.markers)
				mk.viewAngle = std::min(1.0f, std::max(-1.0f, mk.viewAngle + visState.target.adjViewAngle));
			calib.updateMarkers();
			visState.target.adjViewAngle = 0;
			// Signal server to update calib
			SignalTargetCalibUpdate(tracker.id, calib);
		}

		if (ImGui::Button("Import OBJ", SizeWidthDiv2()))
		{
			loadObj.trackerID = tracker.id;
			ImGui::OpenPopup(loadObjPopupID);
		}
		ImGui::SameLine();
		if (ImGui::Button("Export OBJ", SizeWidthDiv2()))
		{
			std::string tgtPathFmt = asprintf_s("/target_%s_%%d.obj", tracker.label.c_str());
			int tgtPathID = findLastFileEnumeration(temporaryStoreFolder + tgtPathFmt)+1;
			std::string tgtPath = temporaryStoreFolder + asprintf_s(tgtPathFmt.c_str(), tgtPathID);
			auto error = writeTargetObjFile(tgtPath, tracker.label, TargetCalibration3D(tracker.calib));
			if (error) SignalErrorToUser(error.value());
		}
		if (ImGui::BeginItemTooltip())
		{
			ImGui::Text("Export target calibration to '%s' folder as target_%s_X.obj.", temporaryStoreFolder.c_str(), tracker.label.c_str());
			ImGui::TextUnformatted(objImportExportGuide);
			ImGui::EndTooltip();
		}

		ImGui::BeginDisabled(!state.isStreaming);
		static OptFrameNum orientationCollectionFrame = -1;
		if (orientationCollectionFrame >= 0)
		{
			if (!state.isStreaming)
				orientationCollectionFrame = -1; // Stopped streaming
			else if (state.pipeline.frameNum < orientationCollectionFrame)
				orientationCollectionFrame = -1; // Reset
			else if (state.pipeline.frameNum - orientationCollectionFrame > 200)
				orientationCollectionFrame = -1; // Lag / UI not active throughout
			else if (state.pipeline.frameNum - orientationCollectionFrame > 100)
			{ // Apply
				auto frames = state.pipeline.record.frames.getView();
				if (orientationCollectionFrame < frames.beginIndex() || orientationCollectionFrame+100 > frames.endIndex())
					SignalErrorToUser("Frames are unexpectedly not available anymore!");
				else
				{
					Matrix3<double> accum = Matrix3<double>::Zero();
					int samples = 0;
					for (auto frame = frames.pos(orientationCollectionFrame); frame.index() < frames.endIndex() && samples < 100; frame++)
					{
						for (auto &trackRecord : frame->get()->trackers)
						{
							if (trackRecord.result.isTracked())
							{
								accum += trackRecord.poseFiltered.rotation().cast<double>();
								samples++;
							}
						}
					}
					if (samples < 80)
					{
						SignalErrorToUser("Only found %d tracked frames of tracker '%s'!\n"
							"Ensure you are tracking, the tracker is well visible,\n"
							"and you aren't dropping an excessive number of frames.");
					}
					else
					{
						// Average rotation (and ensure it really is a rotation matrix)
						Eigen::Matrix3f restRot = (accum / samples).cast<float>();
						auto svd = restRot.jacobiSvd<Eigen::ComputeFullU | Eigen::ComputeFullV>();
						restRot = svd.matrixU() * svd.matrixV().transpose();

						LOG(LTargetCalib, LInfo, "Adjusting target orientation of target '%s' to align with current pose!", tracker.label.c_str());
						TargetCalibration3D calib = tracker.calib;
						for (auto &mk : calib.markers)
						{
							mk.pos = restRot * mk.pos;
							mk.nrm = restRot * mk.nrm;
						}
						calib.updateMarkers();
						// Signal server to update calib
						SignalTargetCalibUpdate(tracker.id, calib);
						// Due to sudden change in calibration tracking will likely get lost, but hard to interfere
					}
				}
				orientationCollectionFrame = -1;
			}
		}
		if (ImGui::Button("Apply currently tracked orientation", SizeWidthFull()))
		{
			orientationCollectionFrame = state.pipeline.frameNum;
		}
		ImGui::SetItemTooltip("Place the tracker so that it is level with the floor.\n"
			"This button will sample that orientation and apply it.\n"
			"The forward axis is not clearly defined.");
		ImGui::EndDisabled();

		EndSection();
	}

	{
		BeginSection("IMU Config");

		std::string NoIMULabel = "No IMU";
		auto getIMULabel = [](const IMU &imu)
		{
			return asprintf_s("IMU %s (%d) [%d] - %s", imu.id.string.c_str(), imu.id.driver, imu.index, imu.isFused? "fused" : "raw");
		};
		auto getIMUIdentLabel = [&](const IMUIdent &imu)
		{
			if (imu)
				return asprintf_s("IMU %s (%d)", imu.string.c_str(), imu.driver);
			else
				return NoIMULabel;
		};
		ImGui::AlignTextToFramePadding();
		BeginLabelledGroup("IMU");
		if (ImGui::BeginCombo("##IMUSel", getIMUIdentLabel(tracker.imuIdent).c_str()))
		{
			bool changed = false;
			if (ImGui::Selectable(NoIMULabel.c_str(), !tracker.imuIdent))
			{
				changed = tracker.imuIdent;
				SignalIMUCalibUpdate(tracker.id, {}, {});
			}
			for (auto &imu : state.pipeline.record.imus)
			{
				ImGui::PushID(imu->index);
				if (ImGui::Selectable(getIMULabel(*imu).c_str(), imu->id == tracker.imuIdent))
				{
					changed = tracker.imuIdent != imu->id;
					if (changed)
						SignalIMUCalibUpdate(tracker.id, imu->id, {});
				}
				ImGui::PopID();
			}
			ImGui::EndCombo();
			if (changed)
				ImGui::MarkItemEdited(ImGui::GetItemID());
		}
		ImGui::EndGroup();

		if (tracker.imuIdent)
		{
			if (tracker.imuCalib.orientation.coeffs().hasNaN())
			{
				ImGui::TextUnformatted("IMU not calibrated!");
				ImGui::SetItemTooltip("IMU orientation and offset to optical tracker have to be calibrated.\n"
					"Use the tracker (and/or place it in various static positions) to calibrate.");
			}
			else if (tracker.imuCalib.offset.isZero())
			{
				ImGui::TextUnformatted("IMU partially calibrated!");
				ImGui::SetItemTooltip("IMU orientation to optical tracker is calibrated, but offset is not yet.\n"
					"Keep using the tracker to continue calibration.");
			}
			else
			{
				ImGui::TextUnformatted("IMU fully calibrated!");
				ImGui::SetItemTooltip("IMU orientation and offset to optical tracker have been fully calibrated.");
			}
			int noOffset = 0;

			bool changed = ScalarProperty<int>("Timestamp Offset", "us", &tracker.imuCalib.timestampOffsetUS, &noOffset, -10000, +10000);
			if (changed) SignalIMUCalibUpdate(tracker.id, tracker.imuIdent, tracker.imuCalib);
		}

		EndSection();
	}

	if (tracker.type == TrackerConfig::TRACKER_TARGET)
	{
		BeginSection("Detection Config");

		TargetDetectionConfig defaultConfig = {};
		bool changed = false;

		changed |= BooleanProperty("3D Matching with Triangulations", &tracker.detectionConfig.match3D, &defaultConfig.match3D);
		ImGui::SetItemTooltip("Quick 3D detection, but may not work on targets with primarily flat markers");

		changed |= BooleanProperty("2D Brute-Force Searching", &tracker.detectionConfig.search2D, &defaultConfig.search2D);
		ImGui::SetItemTooltip("Somewhat slow 2D search, may not work on targets with few markers visible at once");

		changed |= BooleanProperty("2D Brute-Force Probing", &tracker.detectionConfig.probe2D, &defaultConfig.probe2D);
		ImGui::SetItemTooltip("2D probing which should work well on all targets, but slower than 2D searching like-for-like\nTune probe number to get optimal speed/detection ratio for each target.");

		ImGui::BeginDisabled(!tracker.detectionConfig.probe2D && !tracker.detectionConfig.probe2D);
		changed |= ScalarProperty<int>("Probe Count", "", &tracker.detectionConfig.probeCount, &defaultConfig.probeCount, 10, 10000);
		ImGui::SetItemTooltip("Determines the number of rotations probed. A higher number increases processing time, but also increases reliability of detection - up until a point.");
		ImGui::EndDisabled();

		if (changed)
		{
			tracker.configDirty = true;
			ServerUpdateTrackerConfig(state, tracker);
		}

		EndSection();
		// TODO: Can we make filtering methods configurable? Currently selected at compile time in tracking3D.hpp
		// Maybe make TrackedTarget::filter object an opaque pointer, and have a few method calls abstracted away
	}

	ImGui::EndChild();

	ImGui::End();
}