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

void InterfaceState::UpdateTargets(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	ImGui::SeparatorText("Target Database");

	if (ImGui::BeginTable("Targets", 5,
		ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
	{
		ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthStretch, 3);
		ImGui::TableSetupColumn("Markers");
		ImGui::TableSetupColumn("Detail");
		ImGui::TableSetupColumn("Del", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
		//ImGui::TableHeadersRow();

		for (auto &target : state.trackerConfigs)
		{
			if (target.type != TrackerConfig::TRACKER_TARGET)
				continue;

			ImGui::PushID(target.id);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::AlignTextToFramePadding();

			ImGui::Text("%d", target.id);
			ImGui::TableNextColumn();

			ImGui::Text("'%s'", target.label.c_str());
			ImGui::TableNextColumn();

			ImGui::Text("%d", (int)target.calib.markers.size());
			ImGui::TableNextColumn();

			ImGui::Text("%d", (int)target.calib.markers.size());
			ImGui::TableNextColumn();

			bool select = visState.target.selectedTargetID == target.id;
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{
				if (visState.target.selectedTargetID == target.id)
					visState.target.selectedTargetID = 0;
				else if (visState.resetVisTarget())
				{
					if (visState.target.selectedTargetID == target.id)
						visState.target.selectedTargetID = 0;
					else
					{
						visState.target.selectedTargetID = target.id;
						visState.updateVisTarget();
					}
				}
			}
			ImGui::SameLine(0, 0);
			if (CrossButton("Del"))
			{
				
			}
			ImGui::PopID();
		}

		ImGui::EndTable();
	}

	auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
		[&](auto &t){ return t.id == visState.target.selectedTargetID; });
	if (trackerIt == state.trackerConfigs.end())
	{
		ImGui::End();
		return;
	}
	auto &tracker = *trackerIt;

	if (tracker.type == TrackerConfig::TRACKER_TARGET)
	{
		ImGui::Text("Selected %s target %d '%s'", tracker.isSimulated? "simulated" : "calibrated", tracker.id, tracker.label.c_str());

		static float adjustScale = 1.0f;
		ImGui::SetNextItemWidth(SizeWidthDiv2().x);
		ImGui::InputFloat("##Scale", &adjustScale, 0.0f, 0.0f, "%.8f");
		ImGui::SameLine();
		if (ImGui::Button("Adjust scale", SizeWidthDiv2()))
		{
			LOG(LTargetCalib, LInfo, "Adjusting scale of target '%s' by %f!", tracker.label.c_str(), adjustScale);
			for (auto &mk : tracker.calib.markers)
				mk.pos *= adjustScale;
			tracker.calib.updateMarkers();
			auto obs_lock = state.pipeline.obsDatabase.contextualLock();
			for (auto &tgt : obs_lock->targets)
			{
				if (tgt.targetID != visState.target.selectedTargetID) continue;
				for (auto &mk : tgt.markers)
					mk *= adjustScale;
			}
			ServerUpdatedTrackerConfig(state, tracker);
		}

		if (ImGui::Button("Save Target as OBJ", SizeWidthFull()))
		{
			const char* tgtPathFmt = "dump/target_%d.obj";
			std::string tgtPath = asprintf_s(tgtPathFmt, findLastFileEnumeration(tgtPathFmt)+1);
			writeTargetObjFile(tgtPath, TargetCalibration3D(tracker.calib));
		}
	}

	{
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
		if (ImGui::BeginCombo("IMU", getIMUIdentLabel(tracker.imuIdent).c_str()))
		{
			bool changed = false;
			if (ImGui::Selectable(NoIMULabel.c_str(), !tracker.imuIdent))
			{
				tracker.imuIdent = {};
				tracker.imuCalib = {};
				ServerUpdatedTrackerConfig(state, *trackerIt);
			}
			for (auto &imu : state.pipeline.record.imus)
			{
				ImGui::PushID(imu->index);
				if (ImGui::Selectable(getIMULabel(*imu).c_str(), imu->id == tracker.imuIdent))
				{
					tracker.imuIdent = imu->id;
					tracker.imuCalib = {};
					ServerUpdatedTrackerConfig(state, *trackerIt);
					changed = true;
				}
				ImGui::PopID();
			}
			ImGui::EndCombo();
			if (changed)
				ImGui::MarkItemEdited(ImGui::GetItemID());
		}
	}

	ImGui::End();
}