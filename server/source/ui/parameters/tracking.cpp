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

#include "target/parameters.hpp"

#include "implot/implot.h"

#include <cmath>

void InterfaceState::UpdateTrackingParameters(InterfaceWindow &window)
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

	const static TrackingParameters defaultParams = {};

	if (ImGui::CollapsingHeader("Clustering"))
	{
		ImGui::PushID("Cluster");
		auto &params = state.pipeline.params.cluster;
		const auto &standard = defaultParams.cluster;

		BeginSection("2D Clustering of Blobs");
		ScalarProperty<int>("Min Points", "", &params.blob2DCluster.minPoints, &standard.blob2DCluster.minPoints, 1, 10);
		ScalarProperty<float>("Max Distance", "px", &params.blob2DCluster.maxDistance, &standard.blob2DCluster.maxDistance, 0, 1000, 5.0f, PixelFactor, "%.1f");
		EndSection();

		BeginSection("Triangulating of 2D Clusters");
		ScalarProperty<int>("Min Focus Cluster Points", "", &params.clusterTri.minFocusClusterPoints, &standard.clusterTri.minFocusClusterPoints, 1, 10);
		ScalarProperty<float>("Min 2D Cluster Overlap", "", &params.clusterTri.min2DClusterOverlap, &standard.clusterTri.min2DClusterOverlap, 0, 1, 0.02f);
		ScalarProperty<float>("Min 3D Cluster Score", "px", &params.clusterTri.min3DClusterScore, &standard.clusterTri.min3DClusterScore, 0, 100, 1.0f, 1, "%.1f");
		BooleanProperty("Allow Competing Triangulations", &params.clusterTri.allowCompeting, &standard.clusterTri.allowCompeting);
		EndSection();

		BeginSection("3D Clustering of Triangulations");
		ScalarProperty<int>("Min Points", "", &params.tri3DCluster.minPoints, &standard.tri3DCluster.minPoints, 1, 10);
		ScalarProperty<float>("Max Distance", "mm", &params.tri3DCluster.maxDistance, &standard.tri3DCluster.maxDistance, 0, 1000, 10.0f, 1000, "%.1f");
		EndSection();

		ImGui::PopID();
	}

	auto plotSquaredFalloffGraph = [](const char *label, float radius, int power, float cutoff = 0.05f)
	{
		float radiusSq = radius*radius;
		float middleRadiusSq = radiusSq/std::pow(1/cutoff-1, 1.0f/power);
		float middleRadiusPx = std::sqrt(middleRadiusSq)*PixelFactor;
		ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(1,1));
		if (ImPlot::BeginPlot(label, ImVec2(-1, ImGui::GetFrameHeight()*2)))
		{
			ImPlot::SetupAxis(ImAxis_X1, "X1",ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels);
			ImPlot::SetupAxisLimits(ImAxis_X1, 0, radius*PixelFactor, ImPlotCond_Always);
			ImPlot::SetupAxis(ImAxis_Y1, "Y1",ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoLabel | ImPlotAxisFlags_NoTickMarks | ImPlotAxisFlags_NoTickLabels);
			ImPlot::SetupAxisLimits(ImAxis_Y1, -0.01, 1.01, ImPlotCond_Always);
			std::array<float, 20> data;
			for (int i = 0; i < data.size(); i++)
			{
				float r = radius*i/(data.size()-1);
				data[i] = 1.0f / (1.0f + std::pow(r*r/middleRadiusSq, power));
			}
			ImPlot::PlotLine("##Falloff", data.data(), data.size(), radius*PixelFactor/(data.size()-1));
			ImPlot::PlotInfLines("##Middle", &middleRadiusPx, 1);
			ImPlot::EndPlot();
		}
		ImPlot::PopStyleVar();
	};

	auto matchAlgParamUI = [&](MatchingParameters &params, const MatchingParameters &standard)
	{
		bool modified = false;
		modified |= ScalarProperty<float>("Primary Advantage", "", &params.primAdvantage, &standard.primAdvantage, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Competitive Advantage", "", &params.compAdvantage, &standard.compAdvantage, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Uncertainty", "", &params.uncertainty, &standard.uncertainty, 0, 10, 0.02f);
		modified |= ScalarProperty<int>("Conservative Level", "", &params.conservativeLevel, &standard.conservativeLevel, 0, 3);
		modified |= ScalarProperty<int>("Compete Range", "", &params.competeRange, &standard.competeRange, 0, 10);

		return modified;
	};

	auto matchParamUI = [&plotSquaredFalloffGraph, &matchAlgParamUI](TargetMatchingParametersSlow &params, const TargetMatchingParametersSlow &standard)
	{
		bool modified = false;

		modified |= ScalarProperty<int>("Max Match Candidates", "", &params.maxCandidates, &standard.maxCandidates, 0, 20);
		modified |= ScalarProperty<float>("Match Radius", "px", &params.matchRadius, &standard.matchRadius, 0, 100, 0.5f, PixelFactor, "%.2f");
		modified |= ScalarProperty<float>("Max Value", "", &params.maxValue, &standard.maxValue, 0, 100, 0.1f);
		modified |= ScalarProperty<float>("Difference Factor", "x", &params.differenceFactor, &standard.differenceFactor, 0, 100, 0.1f);
		modified |= ScalarProperty<float>("Mismatch Factor", "x", &params.mismatchFactor, &standard.mismatchFactor, 0, 100, 0.1f);

		ImGui::SeparatorText("Difference");
		modified |= ScalarProperty<float>("Difference Radius", "px", &params.differenceRadius, &standard.differenceRadius, 0, 100, 1, PixelFactor, "%.2f");
		modified |= ScalarProperty<int>("Difference Falloff", "", &params.differenceFalloff, &standard.differenceFalloff, 1, 16);
		plotSquaredFalloffGraph("##Difference", params.differenceRadius, params.differenceFalloff);

		ImGui::SeparatorText("Mismatch");
		modified |= ScalarProperty<float>("Influence Radius", "px", &params.influenceRadius, &standard.influenceRadius, 0, 100, 5.0f, PixelFactor, "%.2f");
		modified |= ScalarProperty<int>("Influence Falloff", "", &params.influenceFalloff, &standard.influenceFalloff, 1, 16);
		plotSquaredFalloffGraph("##Influence", params.influenceRadius, params.influenceFalloff);
		modified |= ScalarProperty<float>("Similarity Radius", "px", &params.similarityRadius, &standard.similarityRadius, 0, 100, 1.0f, PixelFactor, "%.2f");
		modified |= ScalarProperty<int>("Similarity Falloff", "", &params.similarityFalloff, &standard.similarityFalloff, 1, 16);
		plotSquaredFalloffGraph("##Similarity", params.similarityRadius, params.similarityFalloff);
		modified |= ScalarProperty<float>("Mismatch Power", "x", &params.mismatchPower, &standard.mismatchPower, 0, 100, 0.1f);

		ImGui::SeparatorText("Point Matching Algorithm");
		modified |= matchAlgParamUI(params.match, standard.match);

		return modified;
	};

	if (ImGui::CollapsingHeader("Target Detection"))
	{
		ImGui::PushID("Detect");
		auto &params = state.pipeline.params.detect;
		const auto &standard = defaultParams.detect;
		bool modified = false;

		BooleanProperty("Use Async Search/Probe", &params.useAsyncDetection, &standard.useAsyncDetection);

		BeginSection("2D Search");
		modified |= ScalarProperty<float>("Error Sigma over Best", "o", &params.search.errorSigma, &standard.search.errorSigma, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Error Max", "px", &params.search.errorMax, &standard.search.errorMax, 0, 10, 0.5f, PixelFactor, "%.1f");
		modified |= ScalarProperty<int>("Max Candidates", "", &params.search.maxCandidates, &standard.search.maxCandidates, 1, 100);
		BooleanProperty("Allow Single Camera Search", &params.search.allowSingleCamera, &standard.search.allowSingleCamera);
		EndSection();

		BeginSection("2D Probe");
		modified |= ScalarProperty<int>("Min Observations", "", &params.probe.minObs, &standard.probe.minObs, 0, 20);
		modified |= ScalarProperty<float>("Initial Error Max", "px", &params.probe.errorInitialMax, &standard.probe.errorInitialMax, 0, 30, 0.5f, PixelFactor, "%.1f");
		modified |= ScalarProperty<float>("Error Max", "px", &params.probe.errorMax, &standard.probe.errorMax, 0, 10, 0.5f, PixelFactor, "%.1f");
		modified |= ScalarProperty<int>("Max Candidates", "", &params.probe.maxCandidates, &standard.probe.maxCandidates, 1, 100);
		if (ImGui::TreeNode("Rotation Generation"))
		{
			auto &gen = pipeline.params.detect.rotGen;

			ImGui::SliderInt("Shell Points", &gen.shellPoints, 0, 1000);
			ImGui::SliderInt("Roll Axis Shells", &gen.rollAxisShells, 1, 100);

			int shellCount = gen.shells.size();
			if (ImGui::SliderInt("Repeating Shells", &shellCount, 1, 10))
				gen.shells.resize(shellCount, Eigen::Vector2f::Zero());
			for (int i = 0; i < gen.shells.size(); i++)
			{
				ImGui::PushID(i);
				ImGui::InputFloat2("    Shell Offset", &gen.shells[i].x(), "%.5f");
				ImGui::PopID();
			}

			if (ImGui::TreeNode("Rotational Spread (Debug)"))
			{ // These are test parameters for future features (e.g. generating rotations according to a covariance)
				ImGui::SliderFloat("Spread Floor", &gen.spreadFloor, 0, 1.0f);
				ImGui::SliderFloat("Spread Ceil", &gen.spreadCeil, 0, 1.0f);
				ImGui::SliderFloat("Spread Variance", &gen.spreadVariance, 0.0f, 10.0f);
				gen.spreadVariance = std::max(gen.spreadVariance, 0.00001f);
				ImGui::TreePop();
			}

			ImGui::TreePop();
		}
		EndSection();

		BeginSection("2D Point Matching");
		modified |= ScalarProperty<float>("Expand Marker FoV", "", &params.expandMarkerFoV, &standard.expandMarkerFoV, -2.0f, 2.0f, 0.02f);
		modified |= ScalarProperty<float>("Normalise for Distance", "m", &params.normaliseDistance, &standard.normaliseDistance, 0, 10, 0.5f);
		if (ImGui::TreeNode("Algorithm"))
		{
			modified |= matchParamUI(params.match, standard.match);
			ImGui::TreePop();
		}
		EndSection();

		BeginSection("Quality");
		modified |= ScalarProperty<int>("Min Obs in Focus Cam", "", &params.minObservations.focus, &standard.minObservations.focus, 0, 50);
		modified |= ScalarProperty<int>("Min total Observations", "", &params.minObservations.total, &standard.minObservations.total, 6, 50); // 6 needed to optimise 6 parameters
		EndSection();

		BeginSection("Optimisation");
		modified |= ScalarProperty<int>("Max Refine Iterations", "", &params.opt.maxRefineIterations, &standard.opt.maxRefineIterations, 0, 100);
		modified |= ScalarProperty<float>("Refine Tolerances", "x", &params.opt.refineTolerances, &standard.opt.refineTolerances, 0, 1000, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<int>("Max Outlier Iterations", "", &params.opt.maxOutlierIterations, &standard.opt.maxOutlierIterations, 0, 100);
		modified |= ScalarProperty<float>("Outlier Tolerances", "x", &params.opt.outlierTolerances, &standard.opt.outlierTolerances, 0, 1000, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Outlier Error Limit", "px", &params.opt.outlierErrorLimit, &standard.opt.outlierErrorLimit, 0, 10, 0.1f, PixelFactor, "%.2f");
		modified |= ScalarProperty<float>("Outlier Grouping", "x", &params.opt.outlierGrouping, &standard.opt.outlierGrouping, 0, 1, 0.05f, 1, "%.4f");
		modified |= ScalarProperty<float>("Outlier Sigma", "o", &params.opt.outlierSigma, &standard.opt.outlierSigma, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Outlier Min Variance", "px", &params.opt.outlierVarMin, &standard.opt.outlierVarMin, 0, 10, 0.1f, PixelFactor, "%.2f");
		modified |= ScalarProperty<float>("Prediction Influence", "x", &params.opt.predictionInfluence, &standard.opt.predictionInfluence, 0, 100, 0.01f, 1, "%.8f");
		EndSection();

		if (modified)
			frameRelevantParametersDirty = true;
		//if (modified && visState.tracking.debug.frameNum == state.pipeline.frameNum)
		//	visState.tracking.debug.needsUpdate = true;

		ImGui::PopID();
	}

	if (ImGui::CollapsingHeader("Target Tracking"))
	{
		ImGui::PushID("Track");
		auto &params = state.pipeline.params.track;
		const auto &standard = defaultParams.track;
		bool modified = false, trustMod = false, filterMod = false;

		BeginSection("Prediction");
		modified |= ScalarProperty<float>("Min 3D Uncertainty", "mm", &params.minUncertainty3D, &standard.minUncertainty3D, 0, 100, 1.0f, 1000, "%.1f");
		modified |= ScalarProperty<float>("Uncertainty Sigma", "o", &params.uncertaintySigma, &standard.uncertaintySigma, 0, 100, 0.1f, 1, "%.1f");
		EndSection();

		BeginSection("2D Point Matching");
		modified |= ScalarProperty<float>("Expand Marker FoV", "", &params.expandMarkerFoV, &standard.expandMarkerFoV, -2.0f, 2.0f, 0.02f);
		modified |= ScalarProperty<float>("Normalise for Distance", "m", &params.normaliseDistance, &standard.normaliseDistance, 0, 10, 0.5f);
		if (ImGui::TreeNode("Fast Matching"))
		{
			modified |= ScalarProperty<float>("Match Radius", "px", &params.matchFast.matchRadius, &standard.matchFast.matchRadius, 0, 100, 1.0f, PixelFactor);
			modified |= matchAlgParamUI(params.matchFast.match, standard.matchFast.match);
			ImGui::TreePop();
		}
		ImGui::SetItemTooltip("Fast matching used initially, works well when changes were predicted accurately.");
		if (ImGui::TreeNode("Select Recover Matching"))
		{
			modified |= ScalarProperty<int>("Min Good Cameras", "", &params.selectRecover.minCamerasGood, &standard.selectRecover.minCamerasGood, 0, 10);
			modified |= ScalarProperty<int>("Max Recover Stages", "", &params.selectRecover.maxRecoverStages, &standard.selectRecover.maxRecoverStages, 0, 10);
			ImGui::TreePop();
		}
		ImGui::SetItemTooltip("These parameters influence whether Recover Matching is used.");
		if (ImGui::TreeNode("Recover Matching"))
		{
			modified |= matchParamUI(params.matchRecover, standard.matchRecover);
			ImGui::TreePop();
		}
		ImGui::SetItemTooltip("A complex matching algorithm that may recover from severe prediction errors (mostly translational).\n"
			"It may also perform worse than fast matching in simpler cases, so it is only used as a fallback if needed.");
		if (ImGui::TreeNode("Select Single Camera Uncertainty"))
		{
			modified |= ScalarProperty<int>("Min Good Cameras", "", &params.selectUncertain.maxCamerasGood, &standard.selectUncertain.maxCamerasGood, 0, 10);
			modified |= ScalarProperty<float>("Max Dominant Factor", "x", &params.selectUncertain.maxDominantFactor, &standard.selectUncertain.maxDominantFactor, 0, 50, 1.0f);
			ImGui::TreePop();
		}
		ImGui::SetItemTooltip("These parameters influence whether Single Camera Uncertainty Matching is used.");
		if (ImGui::TreeNode("Single Camera Uncertainty Axis"))
		{
			modified |= ScalarProperty<float>("Axis Perp Deviation", "", &params.matchUncertain.perpDeviation, &standard.matchUncertain.perpDeviation, 1.0f, 2.0f, 0.1f, 1, "%.1f");
			modified |= ScalarProperty<float>("Step Length", "px", &params.matchUncertain.stepLength, &standard.matchUncertain.stepLength, 1.0f, 10, 0.5f, PixelFactor, "%.1f");
			modified |= ScalarProperty<int>("Max Steps", "", &params.matchUncertain.maxSteps, &standard.matchUncertain.maxSteps, 5, 100);
			ImGui::SeparatorText("Matching in other cameras");
			modified |= ScalarProperty<float>("Match Radius", "px", &params.matchUncertain.subMatch.matchRadius, &standard.matchUncertain.subMatch.matchRadius, 0, 100, 1.0f, PixelFactor);
			modified |= matchAlgParamUI(params.matchUncertain.subMatch.match, standard.matchUncertain.subMatch.match);
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Final Fast Matching"))
		{
			modified |= ScalarProperty<float>("Match Radius", "px", &params.matchFastFinal.matchRadius, &standard.matchFastFinal.matchRadius, 0, 100, 1.0f, PixelFactor);
			modified |= matchAlgParamUI(params.matchFastFinal.match, standard.matchFastFinal.match);
			ImGui::TreePop();
		}
		ImGui::SetItemTooltip("Fast matching used at the end to increase points matched.");
		EndSection();

		BeginSection("Quality");
		modified |= ScalarProperty<int>("Good Camera Min Obs", "", &params.quality.cameraGoodObs, &standard.quality.cameraGoodObs, 0, 20);
		modified |= ScalarProperty<float>("Good Camera Sample Ratio", "px", &params.quality.cameraGoodRatio, &standard.quality.cameraGoodRatio, 0, 1, 0.05f);
		modified |= ScalarProperty<int>("Min Points To Improve", "", &params.quality.minImprovePoints, &standard.quality.minImprovePoints, 0, 20);
		modified |= ScalarProperty<float>("Min Factor To Improve", "x", &params.quality.minImproveFactor, &standard.quality.minImproveFactor, 0, 10, 0.1f);
		modified |= ScalarProperty<int>("Min total Observations", "", &params.quality.minTotalObs, &standard.quality.minTotalObs, 1, 50); // 6 needed to optimise 6 parameters
		modified |= ScalarProperty<int>("Min camera Observations", "", &params.quality.minCameraObs, &standard.quality.minCameraObs, 0, 50);
		modified |= ScalarProperty<float>("Max total Error", "px", &params.quality.maxTotalError, &standard.quality.maxTotalError, 0, 10, 0.1f, PixelFactor, "%.2f");
		EndSection();

		BeginSection("Mistrust");
		trustMod |= ScalarProperty<float>("Tracked Marker Trust", "x", &params.mistrust.matchedMarkerFactor, &standard.mistrust.matchedMarkerFactor, 0, 1, 0.01f);
		trustMod |= ScalarProperty<float>("High Error Threshold", "px", &params.mistrust.highErrorThreshold, &standard.mistrust.highErrorThreshold, 0, 10, 0.1f, PixelFactor, "%.2f");
		trustMod |= ScalarProperty<float>("High Error Mistrust", "x", &params.mistrust.highErrorFactor, &standard.mistrust.highErrorFactor, 0, 1000, 1.0f);
		trustMod |= ScalarProperty<float>("No-Tracking Mistrust", "", &params.mistrust.noTrackingAccum, &standard.mistrust.noTrackingAccum, 0, 1, 0.1f);
		trustMod |= ScalarProperty<float>("Marker Conflict Mistrust", "", &params.mistrust.conflictedMarkerAccum, &standard.mistrust.conflictedMarkerAccum, 0, 1, 0.01f);
		trustMod |= ScalarProperty<float>("Unmatched Pair Mistrust", "", &params.mistrust.unmatchedCertainAccum, &standard.mistrust.unmatchedCertainAccum, 0, 1, 0.001f);
		trustMod |= ScalarProperty<float>("Free Observations Mistrust", "", &params.mistrust.unmatchedObservationsAccum, &standard.mistrust.unmatchedObservationsAccum, 0, 1, 0.001f);
		trustMod |= ScalarProperty<float>("Free Projections Mistrust", "", &params.mistrust.unmatchedProjectionsAccum, &standard.mistrust.unmatchedProjectionsAccum, 0, 1, 0.0001f, 1, "%.4f");
		trustMod |= ScalarProperty<float>("Closeby Observation Range", "x", &params.mistrust.closebyObservationsRange, &standard.mistrust.closebyObservationsRange, 0, 10, 0.01f, 1, "%.4f");
		trustMod |= ScalarProperty<float>("Maximum Mistrust", "", &params.mistrust.maxMistrust, &standard.mistrust.maxMistrust, 0, 10, 0.1f);
		trustMod |= ScalarProperty<float>("Start Maximum Mistrust", "", &params.mistrust.maxMistrustStart, &standard.mistrust.maxMistrustStart, 0, 10, 0.1f);
		trustMod |= ScalarProperty<int>("Start Ease Period", "frames", &params.mistrust.mistrustEasePeriod, &standard.mistrust.mistrustEasePeriod, 0, 100, 10);
		EndSection();

		BeginSection("Optimisation");
		modified |= ScalarProperty<int>("Max Refine Iterations", "", &params.opt.maxRefineIterations, &standard.opt.maxRefineIterations, 0, 100);
		modified |= ScalarProperty<float>("Refine Tolerances", "x", &params.opt.refineTolerances, &standard.opt.refineTolerances, 0, 1000, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<int>("Max Outlier Iterations", "", &params.opt.maxOutlierIterations, &standard.opt.maxOutlierIterations, 0, 100);
		modified |= ScalarProperty<float>("Outlier Tolerances", "x", &params.opt.outlierTolerances, &standard.opt.outlierTolerances, 0, 1000, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Outlier Error Limit", "px", &params.opt.outlierErrorLimit, &standard.opt.outlierErrorLimit, 0, 10, 0.1f, PixelFactor, "%.2f");
		modified |= ScalarProperty<float>("Outlier Grouping", "x", &params.opt.outlierGrouping, &standard.opt.outlierGrouping, 0, 1, 0.05f, 1, "%.4f");
		modified |= ScalarProperty<float>("Outlier Sigma", "o", &params.opt.outlierSigma, &standard.opt.outlierSigma, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Outlier Min Variance", "px", &params.opt.outlierVarMin, &standard.opt.outlierVarMin, 0, 10, 0.1f, PixelFactor, "%.2f");
		modified |= ScalarProperty<float>("Prediction Influence", "x", &params.opt.predictionInfluence, &standard.opt.predictionInfluence, 0, 100, 0.01f, 1, "%.8f");
		EndSection();

		BeginSection("Filtering");

		filterMod |= ScalarProperty<float>("Sigma Init State", "x", &params.filter.sigmaInitState, &standard.filter.sigmaInitState, 0, 10000000, 1.0f, 1, "%.1f");
		filterMod |= ScalarProperty<float>("Sigma Init Change", "x", &params.filter.sigmaInitChange, &standard.filter.sigmaInitChange, 0, 10000000, 1.0f, 1, "%.1f");
		filterMod |= ScalarProperty<float>("Sigma Detect", "x", &params.filter.detectSigma, &standard.filter.detectSigma, 0, 10000000, 1.0f, 1, "%.1f");
		filterMod |= ScalarProperty<float>("Sigma Track", "x", &params.filter.trackSigma, &standard.filter.trackSigma, 0, 10000000, 1.0f, 1, "%.1f");
		filterMod |= ScalarProperty<float>("Dampening Pos", "x", &params.filter.dampeningPos, &standard.filter.dampeningPos, 0, 1, 0.1f, 1, "%.4f");
		filterMod |= ScalarProperty<float>("Dampening Rot", "x", &params.filter.dampeningRot, &standard.filter.dampeningRot, 0, 1, 0.1f, 1, "%.4f");

		ImGui::Separator();

		filterMod |= ScalarProperty<float>("Unscented Alpha", "", &params.filter.sigmaAlpha, &standard.filter.sigmaAlpha, 0, 1, 0.1f, 1, "%.4f");
		filterMod |= ScalarProperty<float>("Unscented Beta", "", &params.filter.sigmaBeta, &standard.filter.sigmaBeta, 0, 10, 0.1f, 1, "%.4f");
		filterMod |= ScalarProperty<float>("Unscented Kappa", "", &params.filter.sigmaKappa, &standard.filter.sigmaKappa, 0, 10, 0.1f, 1, "%.4f");

		ImGui::SeparatorText("Full Target Pose Update");
		ImGui::BeginDisabled(true);
		filterMod |= BooleanProperty("Use Unscented (UKF)##Pose", &params.filter.pose.useUnscented, &standard.filter.pose.useUnscented);
		ImGui::EndDisabled();
		filterMod |= BooleanProperty("Use Numerical Covariance", &params.filter.pose.useNumericCov, &standard.filter.pose.useNumericCov);
		ImGui::BeginDisabled(params.filter.pose.useNumericCov);
		filterMod |= BooleanProperty("Pos Numerical Covariance", &params.filter.pose.useNumericCovPos, &standard.filter.pose.useNumericCovPos);
		ImGui::BeginDisabled(params.filter.pose.useNumericCovPos);
		filterMod |= ScalarProperty<float>("StdDev Pos", "mm", &params.filter.pose.stdDevPos, &standard.filter.pose.stdDevPos, 0, 10, 0.01f, 1000, "%.4f");
		ImGui::EndDisabled();
		filterMod |= ScalarProperty<float>("StdDev Rot", "", &params.filter.pose.stdDevEXP, &standard.filter.pose.stdDevEXP, 0, 10, 0.05f, 1000, "%.4f");
		ImGui::EndDisabled();

		ImGui::SeparatorText("Partial Target Point Update");
		filterMod |= ScalarProperty<int>("Max Observations", "", &params.filter.point.obsLimit, &standard.filter.point.obsLimit, 1, 50);
		filterMod |= BooleanProperty("Use Unscented (UKF)##Point", &params.filter.point.useUnscented, &standard.filter.point.useUnscented);
		filterMod |= BooleanProperty("Use Numerical Jacobian", &params.filter.point.useNumericJac, &standard.filter.point.useNumericJac);
		filterMod |= BooleanProperty("Use Separate Corrections", &params.filter.point.separateCorrections, &standard.filter.point.separateCorrections);
		filterMod |= ScalarProperty<float>("StdDev Error", "px", &params.filter.point.stdDev, &standard.filter.point.stdDev, 0, 10, 0.01f, PixelFactor, "%.4f");

		ImGui::SeparatorText("IMU Integration");
		filterMod |= BooleanProperty("Use IMU Prediction", &params.filter.imu.useForPrediction, &standard.filter.imu.useForPrediction);
		filterMod |= ScalarProperty<float>("StdDev IMU Quat", "", &params.filter.imu.stdDevIMU, &standard.filter.imu.stdDevIMU, 0, 10, 0.05f, 1000, "%.4f");
		filterMod |= ScalarProperty<float>("StdDev IMU Accel", "", &params.filter.imu.stdDevAccel, &standard.filter.imu.stdDevAccel, 0, 10, 0.01f, 1000, "%.4f");

		EndSection();

		modified |= trustMod;
		modified |= filterMod;
		if (modified)
			frameRelevantParametersDirty = true;
		if (state.mode == MODE_Replay || state.mode == MODE_Simulation)
		{ // In replay/simulation
			if (modified && state.simAdvance == 0 && visState.tracking.debug.frameNum == pipeline.frameNum)
			{ // Debugging tracking, automatically track frame again
				visState.tracking.debug.needsUpdate = true;
			}
			if (trustMod)
			{ // Debugging trust values, simulate new filter parameters on recent history
				RetroactivelySimulateMistrust(pipeline, 0, pipeline.frameNum+1);
			}
			else if (filterMod && state.simAdvance == 0 && visState.tracking.trailLength > 0)
			{ // Debugging filter, simulate new filter parameters on recent history
				RetroactivelySimulateFilter(pipeline, pipeline.frameNum-visState.tracking.trailLength, pipeline.frameNum);
			}
		}

		ImGui::PopID();
	}

	if (ImGui::CollapsingHeader("Triangulation Algorithms"))
	{
		ImGui::PushID("Tri");
		TrackingParameters &params = state.pipeline.params;
		const auto &standard = defaultParams;

		BeginSection("Triangulation");
		ScalarProperty<float>("Min Intersection Error", "mm", &params.tri.minIntersectError, &standard.tri.minIntersectError, 0, 10, 0.1f, 1000);
		ScalarProperty<float>("Max Intersection Error", "mm", &params.tri.maxIntersectError, &standard.tri.maxIntersectError, 0, 10, 0.1f, 1000);
		ScalarProperty<float>("Min Intersection Confidence", "", &params.tri.minIntersectionConfidence, &standard.tri.minIntersectionConfidence, 0, 10, 0.5f, 1, "%.1f");
		ScalarProperty<int>("Max Refine Iterations", "", &params.tri.refineIterations, &standard.tri.refineIterations, 0, 50);
		EndSection();

		BeginSection("3D Target Detection / Registration");
		ImGui::Checkbox("Quick Assign Target Matches", &params.detect.tri.quickAssignTargetMatches);
		ScalarProperty<float>("Sigma Error", "o", &params.detect.tri.sigmaError, &standard.detect.tri.sigmaError, 0, 10, 0.1f);
		ScalarProperty<float>("Pose Sigma Error", "o", &params.detect.tri.poseSigmaError, &standard.detect.tri.poseSigmaError, 0, 10, 0.1f);
		ScalarProperty<int>("Min Point Count", "", &params.detect.tri.minPointCount, &standard.detect.tri.minPointCount, 0, 50);
		ScalarProperty<float>("Max Error RMSE", "mm", &params.detect.tri.maxErrorRMSE, &standard.detect.tri.maxErrorRMSE, 0, 50, 0.1f);
		ImGui::Spacing();
		ScalarInput<int>("Min Relations Count", "", &triTargetMinRelations, 0, 100);
		ScalarInput<float>("Min Relations Range", "mm", &triTargetMinDistance, 0, 100, 5.0f, 1000, "%.1f");
		EndSection();

		ImGui::PopID();
	}

	if (ImGui::CollapsingHeader("Continuous Calibration"))
	{
		ImGui::PushID("Cont");
		auto &params = state.pipeline.params.cont;
		const auto &standard = defaultParams.cont;

		BeginSection("Adding Target Observations");
		ScalarProperty<int>("Min Strong Camera Samples ", "", &params.targetObs.minStrongCameraSamples, &standard.targetObs.minStrongCameraSamples, 0, 100);
		ScalarProperty<int>("Min Strong Cameras", "", &params.targetObs.minStrongCameras, &standard.targetObs.minStrongCameras, 0, 10);
		ScalarProperty<int>("Min Camera Samples ", "", &params.targetObs.minCameraSamples, &standard.targetObs.minCameraSamples, 0, 100);
		ScalarProperty<int>("Min Total Samples", "", &params.targetObs.minTotalSamples, &standard.targetObs.minTotalSamples, 0, 100);
		EndSection();

		BeginSection("Camera Optimisation Subsampling");
		{
			auto &params = state.pipeline.params.cont.cameraOptSubsampling;
			const auto &standard = defaultParams.cont.cameraOptSubsampling;
			UpdateSubsamplingParameters(params, standard);
		}
		EndSection();

		BeginSection("Target Optimisation Subsampling");
		{
			auto &params = state.pipeline.params.cont.targetOptSubsampling;
			const auto &standard = defaultParams.cont.targetOptSubsampling;
			UpdateSubsamplingParameters(params, standard);
		}
		EndSection();

		ImGui::PopID();
	}

	ImGui::End();
}