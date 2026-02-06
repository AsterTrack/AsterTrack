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

#include "ui/ui.hpp"

#include "calib_target/parameters.hpp"

#include <cmath>

void InterfaceState::UpdateSubsamplingParameters(SubsampleTargetParameters &params, const SubsampleTargetParameters &standard)
{
	bool sub = false;
	sub |= ScalarProperty<int>("Opt Resample Interval", "", &params.resampleInterval, &standard.resampleInterval, 0, 100);
	sub |= ScalarProperty<int>("Target Marker Samples", "", &params.targetMarkerSamples, &standard.targetMarkerSamples, 0, 10000, 100);
	sub |= ScalarProperty<int>("Target Camera Samples", "", &params.targetCameraSamples, &standard.targetCameraSamples, 0, 100000, 100);
	sub |= ScalarProperty<int>("Frame Batch Size", "", &params.frameBatchSize, &standard.frameBatchSize, 0, 100);
	sub |= ScalarProperty<float>("Percentile", "%", &params.percentile, &standard.percentile, 1, 100, 5.0f, 100, "%.1f");
	sub |= ScalarProperty<float>("Marker Value Distribution", "", &params.markerValueDist, &standard.markerValueDist, -10, 10, 0.1f);
	sub |= ScalarProperty<float>("Camera Value Distribution", "", &params.cameraValueDist, &standard.cameraValueDist, -10, 10, 0.1f);
	sub |= ScalarProperty<float>("Dynamic Marker Factor", "", &params.dynamicMarkerFactor, &standard.dynamicMarkerFactor, 0, 1000, 0.01f);
	sub |= ScalarProperty<float>("Dynamic Camera Factor", "", &params.dynamicCameraFactor, &standard.dynamicCameraFactor, 0, 1000, 0.01f);
	sub |= ScalarProperty<float>("Noise Std Dev", "", &params.randomStdDev, &standard.randomStdDev, 0.0001f, 100, 0.01f);
	if (sub && visState.target.inspectingTrackerID != 0)
	{
		for (auto &tgt  : GetState().pipeline.obsDatabase.contextualRLock()->targets)
		{
			if (tgt.trackerID != visState.target.inspectingTrackerID) continue;
			subsampleTargetObservations(GetState().pipeline.record.frames, tgt, params);
		}
	}
}

void InterfaceState::UpdateTargetCalibParameters(InterfaceWindow &window)
{
	if (!window.open)
		return;
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();
	TargetCalibParameters &calibParams = state.pipeline.targetCalib.params;
	const static TargetCalibParameters defaultParams = {};

	if (BeginCollapsingRegion("Aquiring TargetViews"))
	{
		auto &aquisition = calibParams.aquisition;
		const auto &standard = defaultParams.aquisition;
		ScalarProperty<int>("Min Length", "", &aquisition.minLength, &standard.minLength, 1, 500);
		ScalarProperty<int>("Min Frame Obs Count", "", &aquisition.minFrameObsCount, &standard.minFrameObsCount, 1, 100);
		ScalarProperty<int>("Min Marker Obs Count", "", &aquisition.minMarkerObsCount, &standard.minMarkerObsCount, 1, 100);
		ScalarProperty<int>("Min Shared Marker Count", "", &aquisition.minSharedMarkerCount, &standard.minSharedMarkerCount, 1, 100);
		ScalarProperty<float>("Ratio Hard limit", "x", &aquisition.ratioHardLimit, &standard.ratioHardLimit, 0, 10, 0.1f);
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Optimising TargetViews"))
	{
		auto &view = calibParams.view;
		const auto &standard = defaultParams.view;
		ScalarProperty<float>("Initial Tolerance", "x", &view.initialOptTolerance, &standard.initialOptTolerance, 0, 100, 0.1f);
		ScalarProperty<int>("Initial Iterations Max", "", &view.initialOptLimit, &standard.initialOptLimit, 1, 100);
		ScalarProperty<float>("Manual Tolerance", "x", &view.manualOptTolerance, &standard.manualOptTolerance, 0, 100, 0.01f);
		ScalarProperty<int>("Manual Iterations Max", "", &view.manualOptLimitIncrease, &standard.manualOptLimitIncrease, 1, 100);
		ScalarProperty<float>("Trigger Outlier Sigma", "o", &view.outlierSigmas.trigger, &standard.outlierSigmas.trigger, 1, 10, 0.1f);
		ScalarProperty<float>("Sample Outlier Sigma", "o", &view.outlierSigmas.sample, &standard.outlierSigmas.sample, 1, 10, 0.1f);
		ScalarProperty<float>("Frame Outlier Sigma", "o", &view.outlierSigmas.frame, &standard.outlierSigmas.frame, 1, 10, 0.1f);
		ScalarProperty<float>("Marker Outlier Sigma", "o", &view.outlierSigmas.marker, &standard.outlierSigmas.marker, 1, 10, 0.1f);
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Aligning TargetViews"))
	{
		auto &assembly = calibParams.assembly;
		const auto &standard = defaultParams.assembly;
		ScalarProperty<float>("Point Error", "mm", &assembly.alignPointError, &standard.alignPointError, 0, 10, 0.1f);
		ScalarProperty<float>("Point Sigma", "o", &assembly.alignPointSigma, &standard.alignPointSigma, 0, 10, 0.1f);
		ScalarProperty<float>("Pose Sigma", "o", &assembly.alignPoseSigma, &standard.alignPoseSigma, 0, 10, 0.1f);
		ScalarProperty<float>("Max RMSE", "mm", &assembly.alignMaxRMSE, &standard.alignMaxRMSE, 0, 20, 0.5f);
		ScalarProperty<int>("Min Points", "", &assembly.alignMinPoints, &standard.alignMinPoints, 1, 20);
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Merging TargetView"))
	{
		auto &assembly = calibParams.assembly;
		const auto &standard = defaultParams.assembly;
		ScalarProperty<float>("Marker Merge Dist", "mm", &assembly.mergePointLimit, &standard.mergePointLimit, 0, 20, 0.5f);
		ScalarProperty<int>("Min Points", "", &assembly.mergeMinPoints, &standard.mergeMinPoints, 1, 20);
		ScalarProperty<int>("Min Frame Obs", "", &assembly.mergeMinFrameObs, &standard.mergeMinFrameObs, 1, 20);
		ScalarProperty<float>("Acceptable Error Increase", "x", &assembly.viewThresh, &standard.viewThresh, 1, 10, 0.1f);
		ScalarProperty<float>("Max Error Increase", "x", &assembly.viewMax, &standard.viewMax, 1, 100, 0.1f);
		ImGui::SeparatorText("Marker Matching Algorithm");
		ScalarProperty<float>("Primary Advantage", "", &assembly.match.primAdvantage, &standard.match.primAdvantage, 0, 10, 0.1f);
		ScalarProperty<float>("Competitive Advantage", "", &assembly.match.compAdvantage, &standard.match.compAdvantage, 0, 10, 0.1f);
		ScalarProperty<float>("Uncertainty", "mm", &assembly.match.uncertainty, &standard.match.uncertainty, 0, 10, 0.1f, 1000);
		ScalarProperty<int>("Conservative Level", "", &assembly.match.conservativeLevel, &standard.match.conservativeLevel, 0, 3);
		ScalarProperty<int>("Compete Range", "", &assembly.match.competeRange, &standard.match.competeRange, 0, 10);
		ImGui::SeparatorText("Conservative Optimisation After");
		ScalarProperty<float>("Trigger Outlier Sigma", "o", &assembly.outlierSigmasConservative.trigger, &standard.outlierSigmasConservative.trigger, 1, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Sample Outlier Sigma", "o", &assembly.outlierSigmasConservative.sample, &standard.outlierSigmasConservative.sample, 1, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Frame Outlier Sigma", "o", &assembly.outlierSigmasConservative.frame, &standard.outlierSigmasConservative.frame, 1, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Marker Outlier Sigma", "o", &assembly.outlierSigmasConservative.marker, &standard.outlierSigmasConservative.marker, 1, 10, 0.1f, 1, "%.2f");
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Optimising Target"))
	{
		auto &assembly = calibParams.assembly;
		const auto &standard = defaultParams.assembly;
		ScalarProperty<float>("Trigger Error Increase", "x", &assembly.optThresh, &standard.optThresh, 1, 10, 0.1f, 1, "%.2f");
		ImGui::SetItemTooltip("Sets limit the error may increase before an optional optimisation is triggered.");
		ScalarProperty<float>("Tolerances", "x", &assembly.optTolerance, &standard.optTolerance, 0, 100, 0.1f, 1, "%.2f");
		ScalarProperty<int>("Max Iterations", "", &assembly.optMaxIt, &standard.optMaxIt, 0, 100);
		ScalarProperty<float>("Trigger Outlier Sigma", "o", &assembly.outlierSigmas.trigger, &standard.outlierSigmas.trigger, 1, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Sample Outlier Sigma", "o", &assembly.outlierSigmas.sample, &standard.outlierSigmas.sample, 1, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Frame Outlier Sigma", "o", &assembly.outlierSigmas.frame, &standard.outlierSigmas.frame, 1, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Marker Outlier Sigma", "o", &assembly.outlierSigmas.marker, &standard.outlierSigmas.marker, 1, 10, 0.1f, 1, "%.2f");
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Subsampling Target Data"))
	{
		auto &params = calibParams.assembly.subsampling;
		const auto &standard = defaultParams.assembly.subsampling;
		UpdateSubsamplingParameters(params, standard);
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Reevaluating Markers Sequences"))
	{
		auto &assembly = calibParams.assembly;
		const auto &standard = defaultParams.assembly;
		ScalarProperty<int>("Min Check Length", "", &assembly.reevaluation.minOverlap, &standard.reevaluation.minOverlap, 0, 100);
		ScalarProperty<float>("Uncertainty##Markers", "px", &assembly.reevaluation.uncertainty, &standard.reevaluation.uncertainty, 0, 10, 0.01f, 1, "%.2f");
		ScalarProperty<float>("Max Sequence RMSE", "px", &assembly.reevaluation.maxSeqRMSE, &standard.reevaluation.maxSeqRMSE, 0, 20, 0.5f, 1, "%.2f");
		ScalarProperty<float>("Add Sequence Sigma", "o", &assembly.reevaluation.sigmaAddSeq, &standard.reevaluation.sigmaAddSeq, 0, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Merge Markers Sigma", "o", &assembly.reevaluation.sigmaMerge, &standard.reevaluation.sigmaMerge, 0, 10, 0.1f, 1, "%.2f");
		ImGui::SeparatorText("Sequence->Marker Matching Algorithm");
		ScalarProperty<float>("Primary Advantage", "", &assembly.reevaluation.match.primAdvantage, &standard.reevaluation.match.primAdvantage, 0, 10, 0.1f);
		ScalarProperty<float>("Competitive Advantage", "", &assembly.reevaluation.match.compAdvantage, &standard.reevaluation.match.compAdvantage, 0, 10, 0.1f);
		ScalarProperty<float>("Uncertainty##ReSeqMatch", "px", &assembly.reevaluation.match.uncertainty, &standard.reevaluation.match.uncertainty, 0, 10, 0.1f, 1, "%.2f");
		ScalarProperty<int>("Conservative Level", "", &assembly.reevaluation.match.conservativeLevel, &standard.reevaluation.match.conservativeLevel, 0, 3);
		ScalarProperty<int>("Compete Range", "", &assembly.reevaluation.match.competeRange, &standard.reevaluation.match.competeRange, 0, 10);
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Tracking Additional Frames"))
	{
		auto &assembly = calibParams.assembly;
		const auto &standard = defaultParams.assembly;
		ScalarProperty<int>("Min Tracked Points", "", &assembly.trackFrame.minTrackPts, &standard.trackFrame.minTrackPts, 0, 20);
		ScalarProperty<float>("Uncertainty", "px", &assembly.trackFrame.uncertainty, &standard.trackFrame.uncertainty, 0, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Fill Frame Sigma", "o", &assembly.trackFrame.sigmaFill, &standard.trackFrame.sigmaFill, 0, 10, 0.1f, 1, "%.2f");
		ScalarProperty<float>("Add Frame Sigma", "o", &assembly.trackFrame.sigmaAdd, &standard.trackFrame.sigmaAdd, 0, 10, 0.1f, 1, "%.2f");
		EndCollapsingRegion();
	}

	if (BeginCollapsingRegion("Determining Marker View"))
	{
		auto &post = calibParams.post;
		const auto &standard = defaultParams.post;

		bool assumeSize = !std::isnan(post.assumedSize);
		if (ImGui::Checkbox("##Sz", &assumeSize))
			post.assumedSize = assumeSize? 0.005f : NAN;
		ImGui::SameLine();
		ImGui::BeginDisabled(!assumeSize);
		ScalarProperty<float>("Assume Size", "mm", &post.assumedSize, &standard.assumedSize, 0, 20, 0.1f, 1000);
		ImGui::EndDisabled();

		bool assumeAngle = !std::isnan(post.assumedAngle);
		if (ImGui::Checkbox("##Ang", &assumeAngle))
			post.assumedAngle = assumeAngle? 180.0f : NAN;
		ImGui::SameLine();
		ImGui::BeginDisabled(!assumeAngle);
		ScalarProperty<float>("Assume Angle", "°", &post.assumedAngle, &standard.assumedAngle, 0, 360, 5);
		ImGui::EndDisabled();

		ImGui::BeginDisabled(assumeAngle);
		ScalarProperty<float>("Extremity Centering Power", "", &post.viewCone.extremityCenteringPower, &standard.viewCone.extremityCenteringPower, 0, 10);
		ImGui::InputFloat3("Own Sample Extremes", post.viewCone.ownSampleExtremes.data(), "%.2f");
		ImGui::InputFloat3("Top Marker Extremes", post.viewCone.topMarkerExtremes.data(), "%.2f");
		ImGui::InputFloat3("Own/Top/Compensated", post.viewCone.markerLimitMix.data(), "%.2f");
		ImGui::EndDisabled();

		EndCollapsingRegion();
	}

	ImGui::End();
}