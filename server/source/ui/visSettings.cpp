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
	if (!window.open)
		return;
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;
 
	bool isTracking = pipeline.phase == PHASE_Tracking || pipeline.phase == PHASE_Automatic;
	bool isTargetCalib = pipeline.phase == PHASE_Calibration_Target;
	bool isTesting = state.mode == MODE_Replay || state.mode == MODE_Simulation;
	bool isDebug = (isTesting && state.simAdvance.load() == 0) || dbg_isBreaking;

	if (BeginCollapsingRegion("Pipeline"))
	{
		ImGui::Checkbox("Show Marker Rays", &visState.pipeline.showMarkerRays);
		ImGui::Checkbox("Show 3D Tri Clusters in 3D", &visState.pipeline.showClustersTri3D);
		ImGui::Checkbox("Show Tri 2D Clusters in 3D", &visState.pipeline.showClusters2DTri);
		ImGui::Checkbox("Show 2D Clusters in Camera", &visState.pipeline.showClusters2D);

		EndCollapsingRegion();
	}

	ImGui::BeginDisabled(!isTracking);
	if (BeginCollapsingRegion("Markers"))
	{
		ImGui::Checkbox("Show Covariance in 3D View", &visState.markers.showCovarianceIn3DView);
		ImGui::Checkbox("Show Covariance 3D in Camera", &visState.markers.showCovarianceInCam3D);
		ImGui::Checkbox("Show Covariance 2D in Camera", &visState.markers.showCovarianceInCam2D);
		ImGui::SliderFloat("Covariance Sigma", &visState.markers.scaleCovariance, 1, 100);

		ImGui::Checkbox("Show All Searches in 3D View", &visState.markers.showAllSearchesIn3DView);
		ImGui::Checkbox("Show Missing Searches in 3D View", &visState.markers.showMissingSearchesIn3DView);

		EndCollapsingRegion();
	}
	ImGui::EndDisabled();

	ImGui::BeginDisabled(!isTracking);
	if (BeginCollapsingRegion("Trackers"))
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

		BeginSection("Covariance");
		ImGui::Checkbox("Show Positional Covariance", &visState.tracking.showCovariancePos);
		ImGui::Checkbox("Show Rotational Covariance", &visState.tracking.showCovarianceRot);
		ImGui::SliderFloat("Covariance Sigma", &visState.tracking.scaleCovariance, 1, 100);
		ImGui::Checkbox("Show Covariance Samples", &visState.tracking.showCovarianceSamples);
		ImGui::SliderFloat("Sample Size", &visState.tracking.covSamplesSize, 0, 1);
		ImGui::SliderFloat("Sample Scaling", &visState.tracking.covSamplesScaling, 0, 100);

		VisFrameLock visFrame = visState.lockVisFrame(pipeline, false, true);
		if (isDebug && visFrame)
		{
			auto trackRecord = std::find_if(visFrame.frameIt->get()->trackers.begin(), visFrame.frameIt->get()->trackers.end(),
				[&](auto &tgt){ return tgt.id == visState.tracker.focusedID; });
			if (trackRecord != visFrame.frameIt->get()->trackers.end())
			{
				Eigen::Matrix3f covariance = trackRecord->pose.filteredCov.topLeftCorner<3,3>().transpose();
				ImGui::InputFloat3("##CovT1", covariance.data()+0, "%.8f");
				ImGui::InputFloat3("##CovT2", covariance.data()+3, "%.8f");
				ImGui::InputFloat3("##CovT3", covariance.data()+6, "%.8f");
			}
		}
		EndSection();

		BeginSection("Virtual Tracker");
		ImGui::Checkbox("Show Relation to Subtrackers", &visState.virtTrackers.showRelations);
		ImGui::Checkbox("Debug relation from Subtrackers", &visState.virtTrackers.debugRelationsReverse);
		ImGui::Checkbox("Debug Up Vectors", &visState.virtTrackers.debugUpVectors);
		EndSection();

		EndCollapsingRegion();
	}
	ImGui::EndDisabled();

	ImGui::BeginDisabled(visState.targetMatching.debug.frameNum < 0);
	ImGui::SetNextItemOpen(visState.targetMatching.debug.frameNum >= 0, ImGuiCond_Appearing);
	if (isTracking && isDebug && BeginCollapsingRegion("Target Matching Debug"))
	{
		ImGui::Checkbox("Debug Matching Algorithm", &visState.targetMatching.debugMatchingState);
		ImGui::InputInt("Matching Stage", &visState.targetMatching.debugFocusStage);
		ImGui::BeginDisabled(visState.targetMatching.debugFocusStage <= 0);
		ImGui::Checkbox("Show all labels", &visState.targetMatching.showAllLabels);
		ImGui::InputInt("Focus on Point", &visState.targetMatching.debugFocusPoint);
		ImGui::Checkbox("Only focus point", &visState.targetMatching.onlyFocusPoint);
		ImGui::EndDisabled();
		ImGui::Checkbox("Show Axis of Uncertainty", &visState.targetMatching.showUncertaintyAxis);

		EndCollapsingRegion();
	}
	ImGui::EndDisabled();

	ImGui::BeginDisabled(!isTargetCalib);
	if (isTesting && BeginCollapsingRegion("Target Calibration"))
	{
		BeginSection("Target View Aquisition (Camera View)");
		ImGui::Checkbox("Show Marker Trails", &visState.incObsUpdate.showSeq2DTrail);
		ImGui::Checkbox("Show Marker Labels", &visState.incObsUpdate.showSeq2DLabels);
		EndSection();

		EndCollapsingRegion();
	}
	ImGui::EndDisabled();

	if (BeginCollapsingRegion("Room References"))
	{
		ImGui::Checkbox("Origin", &visState.room.showOrigin);
		ImGui::SameLine();
		ImGui::BeginDisabled(!visState.room.showOrigin);
		ImGui::InputFloat3("##Origin", visState.room.origin.data(), "%.3f");
		ImGui::EndDisabled();

		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Image Adjustment"))
	{
		SliderInput("Brightness", &visState.image.brightness, -0.4f, 0.4f);
		SliderInput("Contrast", &visState.image.contrast, 0.0f, 5.0f);

		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Lens Calibration Vis (Camera View)"))
	{
		float checkboxIndent = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x*2;
		ImGui::Checkbox("Show FoV Circle", &visState.calib.showFoVCircle);
		ImGui::Indent(checkboxIndent);
		ImGui::SliderFloat("##FOV", &visState.calib.circularFoV, 0.0f, 160.0f);
		ImGui::Unindent(checkboxIndent);
		ImGui::Checkbox("Show FoV Bounds (H, V, D)", &visState.calib.showFoVBounds);
		ImGui::Indent(checkboxIndent);
		ImGui::InputFloat3("##HVD", visState.calib.boundsFoV.data(), "%.3f°");
		ImGui::Unindent(checkboxIndent);

		EndCollapsingRegion();
	}

	if (isTesting && BeginCollapsingRegion("Rotation Generation Debug (3D View)"))
	{
		ImGui::Checkbox("Visualise", &visState.rotationGeneration.visualise);

		auto &gen = pipeline.params.detect.rotGen;

		ImGui::SeparatorText("Visualisation");
		auto &sphere = visState.rotationGeneration;

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

		EndCollapsingRegion();
	}

	ImGui::End();
}