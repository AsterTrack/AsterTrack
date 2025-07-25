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

#include "ui/system/vis.hpp"

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
		ImGui::Checkbox("Show Oprhaned IMUs", &visState.tracking.showOrphanedIMUs);
		ImGui::Checkbox("Show Search Bounds", &visState.tracking.showSearchBounds);
		ImGui::Checkbox("Show Observed Target", &visState.tracking.showTargetObserved);
		ImGui::Checkbox("Show Predicted Target", &visState.tracking.showTargetPredicted);
		ImGui::Checkbox("Show Extrapolated Pose", &visState.tracking.showPoseExtrapolated);
		ImGui::Checkbox("Show Inertial Integrated", &visState.tracking.showInertialIntegrated);
		ImGui::Checkbox("Show Inertial Fused", &visState.tracking.showInertialFused);
		ImGui::Checkbox("Show Inertial Filtered", &visState.tracking.showInertialFiltered);
		ImGui::Checkbox("Show Filtered Target", &visState.tracking.showTargetFiltered);
		ImGui::Checkbox("Show Filtered Target in Camera", &visState.tracking.showTargetFilteredCamera);
		ImGui::SliderInt("Trail Length", &visState.tracking.trailLength, 0, 100);


		if (ImGui::TreeNode("Covariance"))
		{
			ImGui::Checkbox("Show Positional Covariance", &visState.tracking.showCovariancePos);
			ImGui::Checkbox("Show Rotational Covariance", &visState.tracking.showCovariancePos);
			ImGui::SliderFloat("Covariance Sigma", &visState.tracking.scaleCovariance, 1, 100);
			ImGui::Checkbox("Show Covariance Samples", &visState.tracking.showCovarianceSamples);

			bool displayInternalDebug = state.simAdvance.load() == 0 || dbg_isBreaking;
			VisFrameLock visFrame = visState.lockVisFrame(pipeline);
			if (displayInternalDebug && visFrame)
			{
				auto trackRecord = std::find_if(visFrame.frameIt->get()->trackers.begin(), visFrame.frameIt->get()->trackers.end(),
					[&](auto &tgt){ return tgt.id == visState.tracking.focusedTrackerID; });
				if (trackRecord != visFrame.frameIt->get()->trackers.end())
				{
					Eigen::Matrix3f covariance = trackRecord->covFiltered.topLeftCorner<3,3>().transpose();
					ImGui::InputFloat3("##CovT1", covariance.data()+0, "%.8f");
					ImGui::InputFloat3("##CovT2", covariance.data()+3, "%.8f");
					ImGui::InputFloat3("##CovT3", covariance.data()+6, "%.8f");
				}
			}

			ImGui::TreePop();
		}
		else visState.tracking.showCovariancePos = visState.tracking.showCovarianceRot = visState.tracking.showCovarianceSamples = false;

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
			ImGui::Checkbox("Show Axis of Uncertainty", &visState.tracking.showUncertaintyAxis);
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

	if (ImGui::TreeNode("Rotation Sphere"))
	{
		ImGui::Checkbox("Visualise", &visState.rotationSphere.visualise);

		auto &gen = pipeline.params.detect.rotGen;


		ImGui::SeparatorText("Visualisation");
		auto &sphere = visState.rotationSphere;

		ImGui::SliderFloat("Point Size", &sphere.pointSize, 0, 50.0f);

		ImGui::SliderFloat("Shell Radius Increase", &sphere.shellRadiusIncrease, 1, 2);

		sphere.hideRollShells = std::min(sphere.hideRollShells, gen.rollAxisShells);
		ImGui::SliderInt("Hide Roll Shells", &sphere.hideRollShells, 0, gen.rollAxisShells);

		sphere.hideShellPoints = std::min(sphere.hideShellPoints, gen.shellPoints);
		ImGui::SliderInt("Hide Shell Points", &sphere.hideShellPoints, 0, gen.shellPoints);

		ImGui::InputFloat3("Sphere Origin", sphere.sphereOrigin.data(), "%.3f");

		ImGui::InputFloat3("Box Origin", sphere.boxOrigin.data(), "%.3f");
		ImGui::SliderFloat("Box Size", &sphere.boxScale, 0, 100);

		ImGui::SliderFloat("Min Neighbour Angle", &sphere.minNeighbourAngle, 0, 90.0f);

		ImGui::TreePop();
	}

	ImGui::End();
}