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

#include "calib_point/parameters.hpp"

#include <cmath>

void InterfaceState::UpdatePointCalibParameters(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();
	const static PointCalibParameters defaultParams = {};

	if (ImGui::CollapsingHeader("Reconstruction"))
	{
		ImGui::PushID("Recon");
		const auto &standard = defaultParams.reconstruction;
		auto &params = state.pipeline.pointCalib.params.reconstruction;

		BeginSection("Fundamental Matrix");
		ScalarProperty<int>("Min Correspondence", "", &params.FM.minPairwiseCorrespondences, &standard.FM.minPairwiseCorrespondences, 1, stableSequenceDelay);
		ScalarProperty<float>("Min Confidence", "", &params.FM.minConfidence, &standard.FM.minConfidence, 0, 10, 0.1f);
		EndSection();

		BeginSection("Data Extrapolation");
		ScalarProperty<float>("Max Columns Factor", "", &params.basis.nColMaxFactor, &standard.basis.nColMaxFactor, 0, 10, 0.1f);
		ScalarProperty<float>("Min Columns Factor", "", &params.basis.nColMinFactor, &standard.basis.nColMinFactor, 0, 10, 0.1f);
		ScalarProperty<int>("Min Columns", "", &params.basis.nColMin, &standard.basis.nColMin, 1, 10000, 10);
		ScalarProperty<float>("Max Tuples Factor", "", &params.basis.tupleMaxFactor, &standard.basis.tupleMaxFactor, 0, 10, 0.1f);
		ScalarProperty<float>("Max Tested Factor", "", &params.basis.tupleTestMaxFactor, &standard.basis.tupleTestMaxFactor, params.basis.tupleMaxFactor, 1000, 5.0f);
		ScalarProperty<float>("Min Rank", "", &params.basis.minRankFactor, &standard.basis.minRankFactor, 0, 100, 0.5f);
		EndSection();

		ImGui::PopID();
	}

	if (ImGui::CollapsingHeader("Optimisation"))
	{
		ImGui::PushID("Opt");
		const auto &standard = defaultParams.outliers;
		auto &params = state.pipeline.pointCalib.params.outliers;

		BeginSection("Outliers");
		ScalarInputN<int>("Bucket Grid Size", "", &params.gridSize.x(), &params.gridSize.y(), &standard.gridSize.x(), 1, 100);
		ScalarProperty<float>("Trigger Outlier Sigma", "", &params.sigma.trigger, &standard.sigma.trigger, 0, 10, 0.1f);
		ScalarProperty<float>("Minimum Outlier Sigma", "", &params.sigma.consider, &standard.sigma.consider, 0, 10, 0.1f);
		ScalarProperty<float>("Force Outlier Sigma", "", &params.sigma.force, &standard.sigma.force, 0, 10, 0.1f);
		ScalarProperty<float>("Bucket Outlier Sigma", "", &params.sigma.bucket, &standard.sigma.bucket, 0, 10, 0.1f);
		EndSection();

		ImGui::PopID();
	}

	ImGui::End();
}