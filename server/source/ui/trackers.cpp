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

void InterfaceState::UpdateTrackers(InterfaceWindow &window)
{
	static int selectedTrackerID = 0;
	auto discardSelection = []
	{
		selectedTrackerID = 0;
		if (GetUI().visState.target.inspectingSource == 'T')
		{
			GetUI().visState.target.inspectingTrackerID = 0;
			GetUI().visState.target.inspectingSource = 'N';
		}
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

	if (SaveButton("Save Tracker Configuration", SizeWidthFull(), state.trackerConfigDirty || state.trackerCalibsDirty || state.trackerIMUsDirty))
	{
		auto error = storeTrackerConfigurations("store/trackers.json", state.trackerConfigs);
		if (error) SignalErrorToUser(error.value());
		else state.trackerConfigDirty = state.trackerCalibsDirty = state.trackerIMUsDirty = false;
	}
	if ((state.trackerConfigDirty || state.trackerCalibsDirty || state.trackerIMUsDirty) && ImGui::BeginItemTooltip())
	{
		if (state.trackerConfigDirty)
			ImGui::TextUnformatted("Tracker Config has been changed.");
		if (state.trackerCalibsDirty)
			ImGui::TextUnformatted("Tracker Calibration has been changed.");
		if (state.trackerIMUsDirty)
			ImGui::TextUnformatted("Tracker IMU Configuration has been changed.");
		ImGui::EndTooltip();
	}

	if (!ImGui::BeginChild("EditChild", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding))
	{
		discardSelection();
		ImGui::EndChild();
		ImGui::End();
		return;
	}

	/* static float childSize = 100; // Auto-Scale to fit view and keep bottom control visible
	childSize = std::max(CalcTableHeight(3), childSize + GetWindowContentRegionHeight() - GetWindowActualContentHeight());
	childSize = std::min(CalcTableHeight(state.trackerConfigs.size()), childSize); */
	float childSize = CalcTableHeight(5); // Adaptive sizing adapts TOO well to differences in UI
	if (ImGui::BeginTable("Tracker", 4,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX | ImGuiTableFlags_ScrollY,
		ImVec2(0, childSize)))
	{
		ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("Detail", ImGuiTableColumnFlags_WidthStretch, 1);
		ImGui::TableSetupColumn("Del", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		//ImGui::TableHeadersRow();

		for (auto trackerIt = state.trackerConfigs.begin(); trackerIt != state.trackerConfigs.end();)
		{
			auto &tracker = *trackerIt;
			ImGui::PushID(tracker.id);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::AlignTextToFramePadding();

			ImGui::Text("%d", tracker.id);
			ImGui::TableNextColumn();

			ImGui::Text("'%s'", tracker.label.c_str());
			ImGui::TableNextColumn();

			if (tracker.type == TrackerConfig::TRACKER_TARGET)
				ImGui::Text("Target with %d markers", (int)tracker.calib.markers.size());
			else if (tracker.type == TrackerConfig::TRACKER_MARKER)
				ImGui::Text("Marker of size %.1fmm", tracker.markerSize*1000.0f);
			else
			 	ImGui::TextUnformatted("Tracker of unknown type.");
			ImGui::TableNextColumn();

			bool select = selectedTrackerID == tracker.id;
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{
				if (visState.target.inspectingSource == 'T' && visState.target.inspectingTrackerID == selectedTrackerID)
					visState.resetVisTarget();
				if (selectedTrackerID == tracker.id)
					selectedTrackerID = 0;
				else
					selectedTrackerID = tracker.id;
			}
			ImGui::SameLine(0, 0);
			if (CrossButton("Del"))
				trackerIt = state.trackerConfigs.erase(trackerIt);
			else
				trackerIt++;
			ImGui::PopID();
		}

		ImGui::EndTable();
	}

	auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
		[&](auto &t){ return t.id == selectedTrackerID; });
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

		ImGui::AlignTextToFramePadding();
		BeginLabelledGroup("Trigger Conditions");
		std::array<const char*,7> triggerLabels;
		triggerLabels.fill("Invalid State");
		triggerLabels[TrackerConfig::TRIGGER_ALWAYS] = "Always";
		triggerLabels[TrackerConfig::TRIGGER_ONLY_MANUALLY] = "Only Manually";
		triggerLabels[TrackerConfig::TRIGGER_ON_IMU_CONNECT] = "On IMU Connect";
		triggerLabels[TrackerConfig::TRIGGER_ON_IO_CONNECT] = "On IO Connect";
		triggerLabels[TrackerConfig::TRIGGER_ON_IMU_CONNECT | TrackerConfig::TRIGGER_ON_IO_CONNECT] = "On IMU or IO Connect";
		if (ImGui::BeginCombo("##TrgCond", triggerLabels[tracker.trigger]))
		{
			bool updated = false;
			if (ImGui::Selectable("Always", tracker.trigger == TrackerConfig::TRIGGER_ALWAYS))
			{
				updated = tracker.trigger != TrackerConfig::TRIGGER_ALWAYS;
				tracker.trigger = TrackerConfig::TRIGGER_ALWAYS;
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
		const TrackerConfig::TrackerExpose exposeMap[] = { TrackerConfig::EXPOSE_ALWAYS, TrackerConfig::EXPOSE_ONCE_TRIGGERED, TrackerConfig::EXPOSE_ONCE_TRACKED };
		std::array<const char*,3> exposeLabel;
		exposeLabel[TrackerConfig::EXPOSE_ALWAYS] = "Always";
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
		{
			ImGui::BeginDisabled(tracker.triggered);
			if (ImGui::Button("Trigger Manually", SizeWidthDiv2()))
			{ // ServerUpdatedTrackerConfig will update with triggered set
				tracker.triggered = true;
				ServerUpdatedTrackerConfig(state, tracker);
			}
			ImGui::EndDisabled();
			ImGui::SameLine();
			ImGui::BeginDisabled(tracker.exposed);
			if (ImGui::Button("Expose Manually", SizeWidthDiv2()))
			{ // CheckTrackingIO in supervisor thread will act with exposed set
				tracker.exposed = true;
			}
			ImGui::EndDisabled();
		}

		if (changed)
		{
			ServerUpdatedTrackerConfig(state, tracker);
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
				view3D.orbit = true;
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
			// Signal server to update calib
			SignalTargetCalibUpdate(tracker.id, calib);
		}

		if (ImGui::Button("Save Target as OBJ", SizeWidthFull()))
		{
			const char* tgtPathFmt = "dump/target_%d.obj";
			std::string tgtPath = asprintf_s(tgtPathFmt, findLastFileEnumeration(tgtPathFmt)+1);
			auto error = writeTargetObjFile(tgtPath, TargetCalibration3D(tracker.calib));
			if (error) SignalErrorToUser(error.value());
		}

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
			ServerUpdatedTrackerConfig(state, tracker);
		}

		EndSection();
		// TODO: Can we make filtering methods configurable? Currently selected at compile time in tracking3D.hpp
		// Maybe make TrackedTarget::filter object an opaque pointer, and have a few method calls abstracted away
	}

	ImGui::EndChild();

	ImGui::End();
}