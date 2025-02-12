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

#include "util/debugging.hpp"

void InterfaceState::UpdateVisualisationSettings(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;
 
	if (ImGui::TreeNode("Image Adjustment"))
	{
		SliderInput("Brightness", &visState.image.brightness, -0.4f, 0.4f);
		SliderInput("Contrast", &visState.image.contrast, 0.0f, 5.0f);
		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Lens Calibration Vis"))
	{
		float checkboxIndent = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x*2;
		ImGui::Checkbox("Show FoV Circle", &visState.calib.showFoVCircle);
		ImGui::Indent(checkboxIndent);
		ImGui::SliderFloat("##FOV", &visState.calib.circularFoV, 0.0f, 160.0f);
		ImGui::Unindent(checkboxIndent);
		ImGui::Checkbox("Show FoV Bounds (H, V, D)", &visState.calib.showFoVBounds);
		ImGui::Indent(checkboxIndent);
		ImGui::InputFloat3("##HVD", visState.calib.boundsFoV.data(), "%.3fdg");
		ImGui::Unindent(checkboxIndent);
		ImGui::TreePop();
	}

	if (pipeline.phase == PHASE_Tracking && (state.mode == MODE_Replay || state.mode == MODE_Simulation)
		&& ImGui::TreeNode("Target Tracking"))
	{
		ImGui::Checkbox("Show Axis of Uncertainty", &visState.tracking.showUncertaintyAxis);
		ImGui::Checkbox("Show Predicted Target Pose", &visState.tracking.showPredictedTarget);
		ImGui::Checkbox("Show Search Bounds", &visState.tracking.showSearchBounds);

		bool displayInternalDebug = state.simAdvance.load() == 0 || dbg_isBreaking;
		if (displayInternalDebug && visState.tracking.debug.frameNum >= 0)
		{
			ImGui::Checkbox("Debug Matching Algorithm", &visState.tracking.debugMatchingState);
			ImGui::InputInt("Matching Stage", &visState.tracking.debugFocusStage);
			ImGui::BeginDisabled(visState.tracking.debugFocusStage <= 0);
			ImGui::Checkbox("Show all labels", &visState.tracking.showAllLabels);
			ImGui::InputInt("Focus on Point", &visState.tracking.debugFocusPoint);
			ImGui::Checkbox("Only focus point", &visState.tracking.onlyFocusPoint);
			ImGui::EndDisabled();
		}

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Room References"))
	{
		ImGui::Checkbox("Origin", &visState.room.showOrigin);
		ImGui::SameLine();
		ImGui::BeginDisabled(!visState.room.showOrigin);
		ImGui::InputFloat3("##Origin", visState.room.origin.data(), "%.3f");
		ImGui::EndDisabled();

		ImGui::TreePop();
	}

	ImGui::End();
}