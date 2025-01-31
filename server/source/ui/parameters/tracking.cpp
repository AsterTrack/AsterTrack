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
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	const static TrackingParameters defaultParams = {};

	if (ImGui::CollapsingHeader("Clustering"))
	{
		ImGui::PushID("Cluster");
		auto &params = state.pipeline.params.cluster;
		const auto &standard = defaultParams.cluster;

		BeginSection("2D Point Clustering");
		ScalarProperty<int>("Min Points", "", &params.blobs.minPoints, &standard.blobs.minPoints, 1, 10);
		ScalarProperty<float>("Max Distance", "px", &params.blobs.maxDistance, &standard.blobs.maxDistance, 0, 1000, 5.0f, PixelFactor, "%.1f");
		EndSection();

		BeginSection("3D Point Clustering");
		ScalarProperty<int>("Min Points", "", &params.tri.minPoints, &standard.tri.minPoints, 1, 10);
		ScalarProperty<float>("Max Distance", "mm", &params.tri.maxDistance, &standard.tri.maxDistance, 0, 1000, 10.0f, 1000, "%.1f");
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

		BeginSection("Brute Force");
		modified |= ScalarProperty<float>("Error Sigma over Best", "o", &params.search.errorSigma, &standard.search.errorSigma, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Error Max", "px", &params.search.errorMax, &standard.search.errorMax, 0, 10, 0.5f, PixelFactor, "%.1f");
		modified |= ScalarProperty<int>("Max Candidates", "", &params.search.maxCandidates, &standard.search.maxCandidates, 1, 100);
		EndSection();

		BeginSection("2D Point Matching");
		modified |= ScalarProperty<float>("Expand Marker FoV", "", &params.expandMarkerFoV, &standard.expandMarkerFoV, 0, 2.0f, 0.02f);
		modified |= ScalarProperty<float>("Normalise for Distance", "m", &params.normaliseDistance, &standard.normaliseDistance, 0, 10, 0.5f);
		if (ImGui::TreeNode("Algorithm"))
		{
			modified |= matchParamUI(params.match, standard.match);
			ImGui::TreePop();
		}
		EndSection();

		BeginSection("Quality");
		modified |= ScalarProperty<int>("Min Obs in Focus Cam", "", &params.minObservations.focus, &standard.minObservations.focus, 0, 50);
		modified |= ScalarProperty<int>("Min Obs in Sec Cam", "", &params.minObservations.sec, &standard.minObservations.sec, 0, 50);
		modified |= ScalarProperty<int>("Min total Observations", "", &params.minObservations.total, &standard.minObservations.total, 6, 50); // 6 needed to optimise 6 parameters
		EndSection();

		BeginSection("Optimisation");
		modified |= ScalarProperty<int>("Max Iterations", "", &params.opt.maxIterations, &standard.opt.maxIterations, 0, 100);
		modified |= ScalarProperty<float>("Tolerances", "x", &params.opt.tolerances, &standard.opt.tolerances, 0, 10, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Outlier Sigma", "o", &params.opt.outlierSigma, &standard.opt.outlierSigma, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Outlier Min Variance", "px", &params.opt.outlierVarMin, &standard.opt.outlierVarMin, 0, 10, 0.1f, PixelFactor, "%.2f");
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
		bool modified = false;

		BeginSection("Prediction");
		modified |= ScalarProperty<float>("Min 3D Uncertainty", "mm", &params.minUncertainty3D, &standard.minUncertainty3D, 0, 100, 1.0f, 1000, "%.1f");
		modified |= ScalarProperty<float>("Add. 2D Uncertainty", "px", &params.addUncertaintyPx, &standard.addUncertaintyPx, 0, 200, 1.0f, PixelFactor, "%.1f");
		modified |= ScalarProperty<float>("Uncertainty Sigma", "o", &params.uncertaintySigma, &standard.uncertaintySigma, 0, 10, 0.1f, 1, "%.1f");
		EndSection();

		BeginSection("2D Point Matching");
		modified |= ScalarProperty<float>("Expand Marker FoV", "", &params.expandMarkerFoV, &standard.expandMarkerFoV, 0, 2.0f, 0.02f);
		if (ImGui::TreeNode("Fast Path"))
		{
			modified |= ScalarProperty<float>("Match Radius", "px", &params.matchFast.matchRadius, &standard.matchFast.matchRadius, 0, 100, 1.0f, PixelFactor);
			modified |= matchAlgParamUI(params.matchFast.match, standard.matchFast.match);
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Slow Path"))
		{
			modified |= matchParamUI(params.matchSlow, standard.matchSlow);
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Single Camera Uncertainty Axis"))
		{
			modified |= ScalarProperty<float>("Max Dominant Factor", "px", &params.matchUncertain.maxDominantFactor, &standard.matchUncertain.maxDominantFactor, 0, 50, 1.0f);
			modified |= ScalarProperty<float>("Axis Perp Deviation", "", &params.matchUncertain.perpDeviation, &standard.matchUncertain.perpDeviation, 1.0f, 2.0f, 0.1f, 1, "%.1f");
			modified |= ScalarProperty<float>("Step Length", "px", &params.matchUncertain.stepLength, &standard.matchUncertain.stepLength, 1.0f, 10, 0.5f, PixelFactor, "%.1f");
			modified |= ScalarProperty<int>("Max Steps", "", &params.matchUncertain.maxSteps, &standard.matchUncertain.maxSteps, 5, 100);
			ImGui::SeparatorText("Matching in other cameras");
			modified |= ScalarProperty<float>("Match Radius", "px", &params.matchUncertain.subMatch.matchRadius, &standard.matchUncertain.subMatch.matchRadius, 0, 100, 1.0f, PixelFactor);
			modified |= matchAlgParamUI(params.matchUncertain.subMatch.match, standard.matchUncertain.subMatch.match);
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Slow Path For Final Match"))
		{
			modified |= matchParamUI(params.matchSlowSecond, standard.matchSlowSecond);
			ImGui::TreePop();
		}
		EndSection();

		BeginSection("Quality");
		modified |= ScalarProperty<int>("Good Camera Sample Count", "", &params.cameraGoodObs, &standard.cameraGoodObs, 0, 50);
		modified |= ScalarProperty<float>("Good Camera Sample Ratio", "px", &params.cameraGoodRatio, &standard.cameraGoodRatio, 0, 1, 0.05f);
		modified |= ScalarProperty<int>("Min total Observations", "", &params.minTotalObs, &standard.minTotalObs, 6, 50); // 6 needed to optimise 6 parameters
		modified |= ScalarProperty<int>("Min camera Observations", "", &params.minCameraObs, &standard.minCameraObs, 0, 50);
		modified |= ScalarProperty<float>("Max total Error", "px", &params.maxTotalError, &standard.maxTotalError, 0, 10, 0.1f, PixelFactor, "%.2f");
		EndSection();

		BeginSection("Optimisation");
		modified |= ScalarProperty<int>("Max Iterations", "", &params.opt.maxIterations, &standard.opt.maxIterations, 0, 100);
		modified |= ScalarProperty<float>("Tolerances", "x", &params.opt.tolerances, &standard.opt.tolerances, 0, 10, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Outlier Sigma", "o", &params.opt.outlierSigma, &standard.opt.outlierSigma, 0, 10, 0.1f);
		modified |= ScalarProperty<float>("Outlier Min Variance", "px", &params.opt.outlierVarMin, &standard.opt.outlierVarMin, 0, 10, 0.1f, PixelFactor, "%.2f");
		EndSection();


		BeginSection("Filtering");
		modified |= ScalarProperty<float>("Uncertainty Pos", "mm", &params.uncertaintyPos, &standard.uncertaintyPos, 0, 1, 0.002f, 1000, "%.4f");
		modified |= ScalarProperty<float>("Uncertainty Rot", "dg", &params.uncertaintyRot, &standard.uncertaintyRot, 0, 1, 0.002f, 180, "%.4f");
		// Don't actually know if uncertainty is in degrees, but it's approximately right and allows for easier editing, so whatever
		modified |= ScalarProperty<float>("Sigma Alpha", "", &params.sigmaAlpha, &standard.sigmaAlpha, 0, 1, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Sigma Beta", "", &params.sigmaBeta, &standard.sigmaBeta, 0, 10, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Sigma Kappa", "", &params.sigmaKappa, &standard.sigmaKappa, 0, 10, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Dampening Pos", "x", &params.dampeningPos, &standard.dampeningPos, 0, 1, 0.1f, 1, "%.4f");
		modified |= ScalarProperty<float>("Dampening Rot", "x", &params.dampeningRot, &standard.dampeningRot, 0, 1, 0.1f, 1, "%.4f");
		EndSection();

		if (modified)
			frameRelevantParametersDirty = true;
		if (modified && visState.tracking.debug.frameNum == state.pipeline.frameNum)
			visState.tracking.debug.needsUpdate = true;

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

		ImGui::PopID();
	}

	ImGui::End();
}