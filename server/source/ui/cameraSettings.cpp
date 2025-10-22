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

#include "device/tracking_camera.hpp"
#include "device/tracking_controller.hpp"
#include "comm/usb.hpp"

#include <cmath>

void InterfaceState::UpdateCameraSettings(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	auto displayConfigurationUI = [&](int configIndex)
	{
		auto &config = state.cameraConfig.configurations[configIndex];

		{ // Label for configuration
			ImGui::AlignTextToFramePadding(); // Align text vertically in line with controls
			ImGui::TextUnformatted("Label");
			SameLinePos(SizeWidthDiv3().x+ImGui::GetStyle().ItemSpacing.x);
			ImGui::SetNextItemWidth(SizeWidthDiv3_2().x);
			ImGui::InputText("##Label", &config.label);
			if (ImGui::IsItemDeactivatedAfterEdit())
				state.cameraConfigDirty = true;
		}

		{
			int numInUse = std::count_if(state.cameras.begin(), state.cameras.end(), 
				[&](const auto &cam){ return state.cameraConfig.getCameraConfigIndex(cam->id) == configIndex; });
			int numTotal = std::count_if(state.cameraConfig.cameraConfigs.begin(), state.cameraConfig.cameraConfigs.end(), 
				[&](const auto &map){ return map.second == configIndex; });
			ImGui::AlignTextToFramePadding();
			ImGui::TextWrapped("%d cameras (%d connected) are using this configuration", numTotal, numInUse);
			if (ImGui::Button("Assign in Devices panel", SizeWidthFull()))
			{
				windows[WIN_DEVICES].open = true;
				ImGui::SetWindowFocus(windows[WIN_DEVICES].title.c_str());
			}
		}

		bool updateDevice = false; // Need to update devices with new configuration if it changed

		BeginSection("Camera");
		ImGui::BeginDisabled(state.isStreaming);
		updateDevice |= ScalarInput2<int>("Resolution", "", &config.width, &config.height, 64, 9999);
		// Synchronised toggle to attach or detach it from it's host controllers framerate
		updateDevice |= CheckboxInput("Synchronised", &config.synchronised);
		ImGui::EndDisabled();
		if (!config.synchronised)
		{ // Show framerate field, but cannot set it yet
			ImGui::BeginDisabled(true);
			config.framerate = 144; // Free-running camera currently runs at max framerate
			updateDevice |= ScalarInput<int>("Framerate", "Hz", &config.framerate, 20, 144);
			// Low framerates currently result in an integer overflow because the timer is precise
			ImGui::EndDisabled();
		}
		{
			float fac = 7.62f, off = 5.2f; // Experimentally determined by visually comparing brightness between strobe changes
			float exposureUS = config.exposure*fac + off;
			updateDevice |= ScalarInput<float>("Exposure", "us", &exposureUS, off+4*fac, 255*fac+off, fac, 1, "%.1f");
			config.exposure = (int)std::round((exposureUS-off)/fac);
		}
		{
			float fac = 8.0f/3.0f, offLen = fac; // Experimentally determined with Oscilloscope
			float offsetUS = config.strobeOffset*fac, lengthUS = config.strobeLength*fac + offLen;
			updateDevice |= ScalarInput<float>("Strobe Length", "us", &lengthUS, offLen, 255*fac+offLen, fac, 1, "%.1f");
			updateDevice |= ScalarInput<float>("Strobe Offset", "us", &offsetUS, 0, 255*fac, fac, 1, "%.1f");
			// TODO: Negative strobe offset should also work, but doesn't currently - min: -255*fac
			config.strobeOffset = int(offsetUS/fac + (std::signbit(offsetUS)? -0.5f : 0.5f));
			config.strobeLength = std::lround((lengthUS-offLen)/fac);
		}
		updateDevice |= ScalarInput<int>("Gain", "x", &config.gain, 1, 16);
		EndSection();

		auto &proc = config.blobProcessing;
		bool updateProc = false;
		if (CheckboxInput("Share Blob Processing Parameters", &config.shareBlobProcessing))
		{
			if (config.shareBlobProcessing)
			{
				for (auto &cfg : state.cameraConfig.configurations)
				{
					if (cfg.shareBlobProcessing)
						config.blobProcessing = cfg.blobProcessing;
				}
				updateDevice = true;
			}
		}

		BeginSection("Blob Segmentation");
		{
			updateProc |= ScalarInput<int>("Center", "", &proc.thresholds.absolute, 1, 255);
			updateProc |= ScalarInput<int>("Edge", "", &proc.thresholds.edge, 1, 255);
		}
		EndSection();

		BeginSection("Blob Resegmentation");

		{
			updateProc |= ScalarInput<int>("Discard Cluster Threshold", "", &proc.classification.blobTinyThreshold, 1, 1000);
			updateProc |= ScalarInput<int>("Resegmentation Threshold", "", &proc.classification.resegmentationThreshold, 1, 10000);
			updateProc |= CheckboxInput("Resegment Single Clusters", &proc.classification.resegmentSingleClusters);
		}

		if (ImGui::CollapsingHeader("Base Parameters"))
		{
			updateProc |= CheckboxInput("Blur base image", &proc.base.blur);
			updateProc |= SliderInput<int>("Radius [px]", &proc.base.radius, 1, 5);
			updateProc |= SliderInput<float>("Sigma", &proc.base.sigma, 0.0f, 2);
		}

		if (ImGui::CollapsingHeader("SSR Parameters"))
		{
			updateProc |= SliderInput<int>("Scale Steps", &proc.ssr.sigmaSteps, 1, 10);
			updateProc |= SliderInput<float>("Sigma Min", &proc.ssr.sigmaMin, 0.0f, proc.ssr.sigmaMax);
			updateProc |= SliderInput<float>("Sigma Max", &proc.ssr.sigmaMax, proc.ssr.sigmaMin, 10.0f);
			updateProc |= SliderInput<float>("Sigma Curve", &proc.ssr.sigmaCurve, 0.0f, 2.0f);
			updateProc |= SliderInput<float>("Truncation", &proc.ssr.sigmaTrunc, 1.0f, 10.0f);
		}

		if (ImGui::CollapsingHeader("Maxima Hint Parameters"))
		{
			updateProc |= SliderInput<int>("Blob Plateau Threshold", &proc.maximaHints.plateauThreshold, 0, 255);
			updateProc |= SliderInput<int>("Blob Plateau Fill Offset", &proc.maximaHints.plateauFillOffset, 0, 100);
			updateProc |= SliderInput<int>("Min Stable Scales", &proc.maximaHints.minStable, 0, 10);
			updateProc |= SliderInput<int>("Min Scales", &proc.maximaHints.minScale, 0, 10);
		}

		if (ImGui::CollapsingHeader("Floodfilling Parameters"))
		{
			updateProc |= SliderInput<int>("Target Minimum", &proc.floodfilling.threshold.min, 1, 100);
			updateProc |= SliderInput<int>("Target Step", &proc.floodfilling.threshold.step, 1, 100);
			updateProc |= SliderInput<int>("Target Min Substep", &proc.floodfilling.threshold.minSubStep, 1, 100);
			updateProc |= SliderInput<int>("Target Loss Threshold", &proc.floodfilling.threshold.acceptableLoss, 1, 20);

			updateProc |= SliderInput<int>("Bounds Expansion", &proc.floodfilling.blob.allowBoundsExpansion, 0, 5);
			updateProc |= SliderInput<float>("Peak/Minimum Ratio", &proc.floodfilling.blob.peakMinimumRatio, 1.0f, 10.0f);
			updateProc |= SliderInput<float>("Limit Expansion Factor", &proc.floodfilling.blob.limitExpansionFactor, 1.0f, 2.0f);
			updateProc |= SliderInput<float>("Limit Expansion Base", &proc.floodfilling.blob.limitExpansionBase, 0.0f, 10.0f);
		}

		if (ImGui::CollapsingHeader("Filtering Parameters"))
		{
			updateProc |= SliderInput<int>("Min Unconflicted Pixels", &proc.filtering.minContributingPixels, 1, 50);
			updateProc |= SliderInput<float>("Min Contrast", &proc.filtering.minContrastValue, 1, 100);
			// TODO: Parameters for filtering blobs based on shape (e.g. circularity)
			// Ofc, that filtering itself is also not implemented yet
		}

		EndSection();

		BeginSection("Blob Refinement");
		{
			updateProc |= ScalarInput<int>("Refinement Threshold", "", &proc.classification.blobRefinementThreshold, 1, 10000);
			updateProc |= SliderInput<int>("Refining Edge Target Value", &proc.refinement.targetEdgeVal, 0, 255);
			updateProc |= SliderInput<float>("Max Edge Offset [px]", &proc.refinement.maxEdgeOffsetPX, 0.0f, 10.0f);
		}
		EndSection();

		ImGui::Separator();

		updateDevice |= updateProc;
		if (updateDevice)
		{
			state.cameraConfigDirty = true;
			if (updateProc && config.shareBlobProcessing)
			{
				for (auto &cfg : state.cameraConfig.configurations)
				{
					if (cfg.shareBlobProcessing && &config != &cfg)
						cfg.blobProcessing = config.blobProcessing;
				}
			}
		}
		if (updateDevice && state.mode == MODE_Device)
		{
			LOG(LGUI, LDebug, "Updating setup with new values!\n");
			for (auto &cam : state.cameras)
			{
				auto cfgMap = state.cameraConfig.cameraConfigs.find(cam->id);
				if (cfgMap == state.cameraConfig.cameraConfigs.end())
					continue;
				auto &cfg = state.cameraConfig.configurations[cfgMap->second];
				if (cfgMap->second != configIndex && !(updateProc && config.shareBlobProcessing && cfg.shareBlobProcessing))
					continue;
				auto mode = getCameraMode(state, cam->id);
				if (state.isStreaming)
					assert(cam->pipeline->mode.widthPx == mode.widthPx && cam->pipeline->mode.heightPx == mode.heightPx);
				else
					cam->pipeline->mode = mode;
				CameraUpdateSetup(state, *cam);
			}
			// TODO:: Implement UI for assigning controllers and their cameras to sync groups
			// Currently, updating "synchronised" status also needs to re-setup sync groups
			// See "Devices" window, should probably happen there
		}
		else if (updateDevice && state.mode == MODE_Replay)
		{
			for (auto &view : cameraViews)
			{
				if (view.second.vis.emulation.enabled)
					view.second.vis.emulation.update = true;
			}
		}
	};

	if (SaveButton("Save Configurations", SizeWidthFull(), state.cameraConfigDirty))
	{
		auto error = storeCameraConfigFile("store/camera_config.json", state.cameraConfig);
		if (error) GetState().errors.push(error.value());
		else state.cameraConfigDirty = false;
	}

	// Tab bar to select configuration to edit
	int selectedConfiguration = -1;
	if (ImGui::BeginTabBar("CameraConfigs", ImGuiTabBarFlags_AutoSelectNewTabs | 
		ImGuiTabBarFlags_FittingPolicyScroll | ImGuiTabBarFlags_TabListPopupButton | ImGuiTabBarFlags_NoTabListScrollingButtons))
	{
		for (int i = 0; i < state.cameraConfig.configurations.size(); i++)
		{
			// Display number of cameras that use this configuration
			int num = std::count_if(state.cameras.begin(), state.cameras.end(), 
				[&](const auto &cam){ return state.cameraConfig.getCameraConfigIndex(cam->id) == i; });

			// Create label for tab
			std::string label;
			if (state.cameraConfig.configurations[i].label.empty())
				label = asprintf_s("Config %d (%d)###%d", i, num, i);
			else
				label = asprintf_s("%s (%d)###%d", state.cameraConfig.configurations[i].label.c_str(), num, i);

			bool retain = true;
			if (ImGui::BeginTabItem(label.c_str(), &retain))
			{
				selectedConfiguration = i;
				ImGui::EndTabItem();
			}
			if (!retain)
			{
				state.cameraConfig.configurations.erase(std::next(state.cameraConfig.configurations.begin(), i));
				if (i == selectedConfiguration && i == state.cameraConfig.configurations.size())
					selectedConfiguration = 0;
				i--;
				// TODO: Reassign camera indices of this and all configurations after this
				// TODO: Confirm or provide undo - don't immediately irreversibly delete
				continue;
			}
		}

		if (ImGui::TabItemButton("+", ImGuiTabItemFlags_Trailing))
		{
			if (selectedConfiguration >= 0)
				state.cameraConfig.configurations.push_back(state.cameraConfig.configurations[selectedConfiguration]);
			else
				state.cameraConfig.configurations.push_back({});
			selectedConfiguration = state.cameraConfig.configurations.size()-1;
		}

		if (ImGui::GetCurrentTabBar()->ScrollingSpeed != 0.0f)
			RequestUpdates(1);
		ImGui::EndTabBar();
	}

	if (selectedConfiguration >= 0)
	{
		static float childSize = 100; // Auto-Scale to fit view and keep bottom control visible
		childSize = std::max(100.0f, childSize + GetWindowContentRegionHeight() - GetWindowActualContentHeight());
		if (ImGui::BeginChild("CamSettings", ImVec2(0, childSize), ImGuiChildFlags_AlwaysUseWindowPadding))
		{
			displayConfigurationUI(selectedConfiguration);
		}
		ImGui::EndChild();
	}

	BeginSection("Controller Configuration");

	bool updateFPS = ScalarInput<int>("Framerate", "Hz", &state.controllerConfig.framerate, 20, 144);

	// But this will not be handled here in the future anyway, framerate/sync group should be handled by a separate window for device setup
	if (updateFPS && state.isStreaming)
	{ // Streaming, have to update FPS across all systems
		if (state.mode == MODE_Device)
		{
			for (auto &controller : state.controllers)
			{
				if (!controller->syncGen) continue;
				controller->syncGen->contextualLock()->frameIntervalMS = 1000.0f / state.controllerConfig.framerate;
				// TODO: Temp workaround to get any frames at all for free-running cameras
				comm_submit_control_data(controller->comm, COMMAND_OUT_SYNC_GENERATE, state.controllerConfig.framerate, 0);
			}
		}
	}

	EndSection();

	ImGui::End();
}