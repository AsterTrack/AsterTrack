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

#include "point/sequences2D.hpp"

#include <cmath>

void InterfaceState::UpdateSequenceParameters(InterfaceWindow &window)
{
	ServerState &state = GetState();
	auto &params = state.pipeline.sequenceParams;
	if (!window.open)
	{
		params.selected = -1;
		return;
	}
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		params.selected = -1;
		ImGui::End();
		return;
	}

	auto displaySettingsUI = [&](int settingIndex)
	{
		auto &setting = params.sequence[settingIndex];
		const SequenceAquisitionParameters standard = settingIndex < params.standard.size()? params.standard[settingIndex] : SequenceAquisitionParameters{};

		if (BeginCollapsingRegion("Matching of points each frame"))
		{
			ScalarProperty<float>("Max Acceleration", "px", &setting.maxAcceleration, &standard.maxAcceleration, 0, 200, 1.0f, PixelFactor, "%.1f");
			ScalarProperty<float>("Max Value Difference", "", &setting.maxValueDiff, &standard.maxValueDiff, 0, 1000, 10.0f, 1, "%.0f");
			ScalarProperty<float>("Value Factor", "", &setting.valueDiffFactor, &standard.valueDiffFactor, 0, 10, 0.05f, 1000*PixelFactor, "%.2f");
			ScalarProperty<int>("Value Flowing Average", "", &setting.valueFlowingAverage, &standard.valueFlowingAverage, 1, 1000);
			
			BeginSection("Matching Algorithm");
			ScalarProperty<float>("Primary Advantage", "", &setting.match.primAdvantage, &standard.match.primAdvantage, 0, 10, 0.1f);
			ScalarProperty<float>("Competitive Advantage", "", &setting.match.compAdvantage, &standard.match.compAdvantage, 0, 10, 0.1f);
			ScalarProperty<float>("Uncertainty", "px", &setting.match.uncertainty, &standard.match.uncertainty, 0, 10, 0.1f, PixelFactor, "%.2f");
			ScalarProperty<int>("Conservative Level", "", &setting.match.conservativeLevel, &standard.match.conservativeLevel, 0, 3);
			ScalarProperty<int>("Compete Range", "", &setting.match.competeRange, &standard.match.competeRange, 0, 10);
			EndSection();

			BeginSection("Inactive points");
			ScalarProperty<int>("Min Inactive Frames", "", &setting.minInactivityLength, &standard.minInactivityLength, 1, 1000);
			ScalarProperty<float>("Max Movement", "px", &setting.maxInactivityMovement, &standard.maxInactivityMovement, 0, 20, 0.1f, PixelFactor, "%.2f");
			ScalarProperty<float>("Max Deviation", "o", &setting.maxSigmaInactivity, &standard.maxSigmaInactivity, 0, 10, 0.1f);
			EndSection();

			BeginSection("Allowed frame drops / occlusions");
			ScalarProperty<int>("Established Sequences", "", &setting.allowedDrops, &standard.allowedDrops, 0, 10);
			ScalarProperty<int>("New Sequences", "", &setting.allowedDropsTemp, &standard.allowedDropsTemp, 0, 10);
			ScalarProperty<int>("Inactive Points", "", &setting.allowedDropsTempInactive, &standard.allowedDropsTempInactive, 0, 10);
			EndSection();

			EndCollapsingRegion();
		}

		if (BeginCollapsingRegion("Sequence Length Limits"))
		{
			ImGui::BeginDisabled(true);
			int stableLength = stableSequenceDelay;
			ScalarProperty<int>("Sequence Stable Limit", "", &stableLength, nullptr, stableSequenceDelay, stableSequenceDelay);
			ImGui::SetItemTooltip("Hardcoded delay (in frames from current frame) used by other algorithms after which they assume sequences to be stable and fully corresponded.");
			ImGui::EndDisabled();
			ScalarProperty<int>("Correspondence Match Length", "", &setting.sequenceCorrespondenceLength, &standard.sequenceCorrespondenceLength, 0, stableSequenceDelay);
			ImGui::SetItemTooltip("Length at which correspondence matching is performed - to set it below the hardcoded stable limit.");
			ScalarProperty<int>("Minimum Length", "", &setting.minSequenceLength, &standard.minSequenceLength, 0, setting.sequenceCorrespondenceLength);

			EndCollapsingRegion();
		}

		if (BeginCollapsingRegion("Fundamental Matrices"))
		{
			ScalarProperty<float>("Upper Error", "", &setting.FM.UpperError, &standard.FM.UpperError, 0, 1, 0.05f);
			ScalarProperty<int>("Confidence Min Samples", "", &setting.FM.ConfidentMinSamples, &standard.FM.ConfidentMinSamples, 0, 10000, 10);
			ScalarProperty<float>("Confidence Min Trust", "", &setting.FM.ConfidentMinTrust, &standard.FM.ConfidentMinTrust, 0, 100000, 100);
			ScalarProperty<float>("Trust Decay Rate", "", &setting.FM.TrustDecayRate, &standard.FM.TrustDecayRate, 0, 1, 0.001f, 1000, "%.5f");
			ScalarProperty<float>("Maximum Deviation", "o", &setting.FM.FitPointSigma, &standard.FM.FitPointSigma, 0, 10, 0.1f);
			ImGui::SetItemTooltip("Maximum sigma deviation allowed for points during continuous cross-correspondence checks.");

			EndCollapsingRegion();
		}

		if (BeginCollapsingRegion("Correspondence Matching"))
		{
			ScalarProperty<float>("Trust Base for Supporting", "", &setting.correspondences.TrustBaseSupporting, &standard.correspondences.TrustBaseSupporting, 0, 10000, 10);
			ScalarProperty<float>("Trust Base for Discrediting", "", &setting.correspondences.TrustBaseDiscrediting, &standard.correspondences.TrustBaseDiscrediting, 0, 10000, 10);
			ScalarProperty<float>("Minimum Supporting Weight", "", &setting.correspondences.minWeight, &standard.correspondences.minWeight, 0, 1000, 10);
			ScalarProperty<float>("Maximum Discrediting Weight", "%", &setting.correspondences.maxDiscrediting, &standard.correspondences.maxDiscrediting, 0, 100, 1, 100, "%.0f");
			ScalarProperty<float>("Primary Advantage", "", &setting.correspondences.minPrimAdvantage, &standard.correspondences.minPrimAdvantage, 0, 10, 0.1f);
			ScalarProperty<float>("Uncertainty", "", &setting.correspondences.errorUncertainty, &standard.correspondences.errorUncertainty, 0, 1000, 0.01f, 1000);

			BeginSection("with existing FM");
			ScalarProperty<float>("Selecting Fit Deviation", "o", &setting.FM.FitSeqSigmaSelect, &standard.FM.FitSeqSigmaSelect, 0, 10, 0.1f);
			ImGui::SetItemTooltip("Selects the error value of a sequence as avg + sigma deviation when using a confident Fundamental Matrix for the correspondence check.");
			ScalarProperty<float>("Maximum Fit Deviation", "o", &setting.FM.FitSeqSigmaMax, &standard.FM.FitSeqSigmaMax, 0, 10, 0.1f);
			ImGui::SetItemTooltip("Maximum sigma deviation allowed for the chosen sequence error value when using a confident Fundamental Matrix for the correspondence check.");
			EndSection();

			BeginSection("with new FM");
			ScalarProperty<int>("Minimum Overlap", "", &setting.correspondences.minOverlap, &standard.correspondences.minOverlap, 1, setting.sequenceCorrespondenceLength);
			ScalarProperty<float>("Maximum Error", "", &setting.correspondences.maxError, &standard.correspondences.maxError, 0, 1, 0.01f);
			ScalarProperty<float>("Minimum Confidence", "", &setting.correspondences.minConfidence, &standard.correspondences.minConfidence, 0, 1000);
			ScalarProperty<float>("Selecting Fit Deviation", "o", &setting.FM.FitNewSigma, &standard.FM.FitNewSigma, 0, 10, 0.1f);
			ImGui::SetItemTooltip("Selects the error value of a sequence as avg + sigma deviation when for judging a brand new Fundamental Matrix.");
			BooleanProperty("Allow Marker Merging", &setting.correspondences.allowMarkerMerging, &standard.correspondences.allowMarkerMerging);
			ImGui::SetItemTooltip("Merge markers if correspondence matches two different markers closely. Less conservative than normal correspondence matching.");
			EndSection();

			EndCollapsingRegion();
		}
	};

	// Tab bar to select settings to edit
	if (ImGui::BeginTabBar("SeqSettings", ImGuiTabBarFlags_AutoSelectNewTabs | 
		ImGuiTabBarFlags_FittingPolicyScroll | ImGuiTabBarFlags_TabListPopupButton | ImGuiTabBarFlags_NoTabListScrollingButtons))
	{
		for (int i = 0; i < params.sequence.size(); i++)
		{
			bool retain = true;
			if (ImGui::BeginTabItem(params.sequence[i].label.c_str(), &retain))
			{
				params.selected = i;
				ImGui::EndTabItem();
			}
			if (!retain)
			{
				params.sequence.erase(std::next(params.sequence.begin(), i));
				if (i == params.selected && i == params.sequence.size())
					params.selected = 0;
				i--;
				continue;
			}
		}

		if (ImGui::TabItemButton("+", ImGuiTabItemFlags_Trailing))
		{
			if (params.selected >= 0)
				params.sequence.push_back(params.sequence[params.selected]);
			else
				params.sequence.push_back({});
			params.sequence.back().label = asprintf_s("Setting %d", (int)params.sequence.size()-1);
			params.selected = params.sequence.size()-1;
		}

		if (ImGui::GetCurrentTabBar()->ScrollingSpeed != 0.0f)
			RequestUpdates(1);
		ImGui::EndTabBar();
	}
	if (params.selected >= 0)
		displaySettingsUI(params.selected);

	ImGui::End();
}