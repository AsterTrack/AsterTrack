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

#include "simulation.hpp"


void InterfaceState::UpdateSimulationParameters(InterfaceWindow &window)
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
	auto sim_lock = pipeline.simulation.contextualLock();

	bool modified = false;

	if (BeginCollapsingRegion("Projection Parameters"))
	{
		auto &params = sim_lock->projectionParams;
		const static SimProjectionParameters standard = {};

		ImGui::SeparatorText("Blob Projection");
		modified |= ScalarProperty<float>("Expand Marker View Angle", "", &params.expandMarkerViewAngle, &standard.expandMarkerViewAngle, -2.0f, 2.0f, 0.02f);
		modified |= ScalarProperty<float>("Noise Std Dev", "px", &params.blobNoiseStdDev, &standard.blobNoiseStdDev, 0, 10, 0.02f, PixelFactor, "%.2f");
		modified |= ScalarProperty<float>("Noise Max Sigma", "o", &params.blobNoiseMaxSigma, &standard.blobNoiseMaxSigma, 1, 10, 0.2f);

		ImGui::SeparatorText("Blob Size");
		modified |= ScalarProperty<float>("Min Source Size", "px", &params.minSourceBlobSize, &standard.minSourceBlobSize, 0, 10, 0.02f, PixelFactor, "%.2f");
		modified |= ScalarProperty<float>("Visual Size Factor", "x", &params.blobVisualSizeFactor, &standard.blobVisualSizeFactor, 0, 100, 0.05f);
		modified |= ScalarProperty<float>("Visual Size Flare", "px", &params.blobVisualSizeFlare, &standard.blobVisualSizeFlare, 0, 10, 0.05f, PixelFactor, "%.2f");
		modified |= BooleanProperty("Diminish Size at Grazing View Angle", &params.grazingAngleDiminishSize, &standard.grazingAngleDiminishSize);
		ImGui::BeginDisabled(!params.grazingAngleDiminishSize);
		modified |= ScalarProperty<float>("Grazing Angle Lower Offset", "", &params.grazingAngleLower, &standard.grazingAngleLower, -2.0f, 2.0f, 0.02f);
		modified |= ScalarProperty<float>("Grazing Angle Upper Offset", "", &params.grazingAngleUpper, &standard.grazingAngleUpper, -2.0f, 2.0f, 0.02f);
		ImGui::EndDisabled();

		ImGui::SeparatorText("Merge Nearby Blobs");
		modified |= ScalarProperty<float>("Merge Distance Factor", "", &params.mergeFactor, &standard.mergeFactor, 0.0f, 1000.0f, 0.05f);

		EndCollapsingRegion();
	}

	if (modified && state.mode == MODE_Replay && state.simAdvance.load() == 0)
	{ // Simulation mode will loose simulation state, and doesn't benefit much from this anyway
		// Load last frame if currently halted
		AdoptFrameRecordState(pipeline, *pipeline.record.frames.getView().at(pipeline.frameNum.load()-1));
		// Then proceed by one frame to re-process it entirely
		state.simAdvance = 1;
		state.simAdvance.notify_all();
	}

	ImGui::End();
}