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

		for (auto &target : state.pipeline.tracking.targetTemplates3D)
		{
			ImGui::PushID(target.id);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::AlignTextToFramePadding();

			ImGui::Text("%d", target.id);
			ImGui::TableNextColumn();

			ImGui::Text("'%s'", target.label.c_str());
			ImGui::TableNextColumn();

			ImGui::Text("%d", (int)target.markers.size());
			ImGui::TableNextColumn();

			ImGui::Text("%d", (int)target.markers.size());
			ImGui::TableNextColumn();

			bool select = visState.target.selectedTargetID == target.id;
			
			if (ImGui::Selectable("", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
			{
				if (visState.resetVisTarget())
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

	TargetTemplate3D *target = nullptr, *targetSim = nullptr;
	if (visState.target.selectedTargetID > 0)
	{
		auto targetIt = std::find_if(state.pipeline.tracking.targetTemplates3D.begin(), state.pipeline.tracking.targetTemplates3D.end(),
			[&](auto &t){ return t.id == visState.target.selectedTargetID; });
		if (targetIt != state.pipeline.tracking.targetTemplates3D.end())
			target = &*targetIt;
	}
	else if (visState.target.selectedTargetID < 0)
	{
		auto targetIt = std::find_if(state.config.simulation.trackingTargets.begin(), state.config.simulation.trackingTargets.end(),
			[&](auto &t){ return t.id == visState.target.selectedTargetID; });
		if (targetIt != state.pipeline.tracking.targetTemplates3D.end())
			target = &*targetIt;
	}
	// TODO: Improve handling of simulated and calibrated target IDs
	// may have a simulated target that's calibrated, so exists as both simulated and calibrated variant
	// Currently the former is < 0, latter > 0, what about calibrated simulated targets? Clarify

	// TODO: Allow adoption of simulated target into pipeline.tracking.targetTemplates3D
	// Would allow for kick-starting calibrations with a designed calibration

	if (target)
	{
		ImGui::Text("Selected calibrated target %d '%s'", target->id, target->label.c_str());

		static float adjustScale = 1.0f;
		ImGui::SetNextItemWidth(SizeWidthDiv2().x);
		ImGui::InputFloat("##Scale", &adjustScale, 0.0f, 0.0f, "%.8f");
		ImGui::SameLine();
		if (ImGui::Button("Adjust scale", SizeWidthDiv2()))
		{
			LOG(LTargetCalib, LInfo, "Adjusting scale of target '%s' by %f!", target->label.c_str(), adjustScale);
			for (auto &mk : target->markers)
				mk.pos *= adjustScale;
			target->updateMarkers();
			auto obs_lock = state.pipeline.obsDatabase.contextualLock();
			for (auto &tgt : obs_lock->targets)
			{
				if (tgt.targetID != visState.target.selectedTargetID) continue;
				for (auto &mk : tgt.markers)
					mk *= adjustScale;
			}
		}

		if (ImGui::Button("Save Target as OBJ", SizeWidthFull()))
		{
			const char* tgtPathFmt = "dump/target_%d.obj";
			std::string tgtPath = asprintf_s(tgtPathFmt, findLastFileEnumeration(tgtPathFmt)+1);
			writeTargetObjFile(tgtPath, TargetTemplate3D(target->markers));
		}
	}
	else if (targetSim)
	{
		ImGui::Text("Selected simulated target %d '%s'", targetSim->id, targetSim->label.c_str());
	}

	ImGui::End();
}