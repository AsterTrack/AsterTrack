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

#include "app.hpp"
#include "config.hpp"

#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <filesystem>


void InterfaceState::UpdateMainMenuBar()
{
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10,6));
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10,10));

	static bool openAboutPopup = false;

	if (!ImGui::BeginMainMenuBar())
	{
		ImGui::PopStyleVar(3);
		return;
	}
	// Warning: Only use with useCustomHeader, doesn't play well with libdecor
	bool mouseInHeader = ImGui::IsWindowHovered(); 
	bool focusOnUIElement = false;

	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	if (ImGui::BeginMenu("AsterTrack"))
	{
		ImGui::SetNextItemShortcut(ImGuiKeyChord(ImGuiMod_Ctrl | ImGuiKey_R));
		if (ImGui::MenuItem("Reload Interface"))
		{
			setCloseInterface = true;
		}
		ImGui::SetItemTooltip("Reload the Interface - allows for updates to astertrack_interface.so to be applied");
		ImGui::Separator();
		if (ImGui::MenuItem(state.io.useVRPN? "Stop VRPN Server###VRPN" : "Start VRPN Server###VRPN"))
		{
			if (state.io.useVRPN)
				ResetIO(state);
			else
				SetupIO(state);
		}
		ImGui::SetItemTooltip("Start/Stop the server for the VRPN interface");
		ImGui::Separator();
		if (ImGui::MenuItem("About"))
		{
			openAboutPopup = true;
		}
		ImGui::SetItemTooltip("About AsterTrack");
		if (ImGui::MenuItem("Quit"))
		{
			GetApp().SignalQuitApp();
		}
		ImGui::SetItemTooltip("Quit the AsterTrack Application");
		ImGui::EndMenu();
	}
	focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
	if (ImGui::BeginMenu("View"))
	{
		if (ImGui::MenuItem("Reset Layout"))
			ResetWindowLayout();
		ImGui::Separator();

		auto addWindowMenuItem = [](InterfaceWindow &window)
		{
			if (!ImGui::MenuItem(window.title.c_str(), nullptr, &window.open))
				return;
			if (!window.open)
			{
				ImGuiWindow* imgui_window = ImGui::FindWindowByID(window.id);
				if (imgui_window)
					ImGui::DockContextProcessUndockWindow(ImGui::GetCurrentContext(), imgui_window, 0);
				ImGui::SetTabItemClosed(window.title.c_str());
			}
		};
		addWindowMenuItem(windows[WIN_CAMERA_VIEWS]);
		addWindowMenuItem(windows[WIN_3D_VIEW]);
		addWindowMenuItem(windows[WIN_VISUALISATION]);
		ImGui::Separator();
		addWindowMenuItem(windows[WIN_PIPELINE]);
		addWindowMenuItem(windows[WIN_CONTROL]);
		ImGui::Separator();
		addWindowMenuItem(windows[WIN_LOGGING]);
		addWindowMenuItem(windows[WIN_INSIGHTS]);
		ImGui::Separator();
		addWindowMenuItem(windows[WIN_TARGETS]);
		ImGui::Separator();
		addWindowMenuItem(windows[WIN_DEVICES]);
		addWindowMenuItem(windows[WIN_CAMERA_SETTINGS]);
		addWindowMenuItem(windows[WIN_WIRELESS]);
		ImGui::Separator();
		if (ImGui::BeginMenu("Parameters"))
		{
			addWindowMenuItem(windows[WIN_SEQUENCE_PARAMS]);
			addWindowMenuItem(windows[WIN_POINT_CALIB_PARAMS]);
			addWindowMenuItem(windows[WIN_TARGET_CALIB_PARAMS]);
			addWindowMenuItem(windows[WIN_TRACKING_PARAMS]);
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Tools"))
		{
			addWindowMenuItem(windows[WIN_LENS_SELECTION_TOOL]);
			ImGui::EndMenu();
		}
		ImGui::Separator();
		if (ImGui::BeginMenu("Dear ImGUI"))
		{
			addWindowMenuItem(windows[WIN_STYLE_EDITOR]);
			addWindowMenuItem(windows[WIN_IMGUI_DEMO]);
			addWindowMenuItem(windows[WIN_IMPLOT_DEMO]);
			ImGui::EndMenu();
		}
		ImGui::EndMenu();
	}
	focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
	if (ImGui::BeginMenu("Device"))
	{
		if (state.mode == MODE_Device)
		{
			if (ImGui::MenuItem("Disonnect###DeviceToggle"))
				StopDeviceMode(state);
		}
		else
		{
			if (ImGui::MenuItem("Connect###DeviceToggle", nullptr, nullptr, state.mode == MODE_None))
				StartDeviceMode(state);
		}
		ImGui::EndMenu();
	}
	focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
	bool cachePaths = false;
	static bool cachingCalibPaths = false, cachingCapturePaths = false;
	static std::vector<std::filesystem::path> pointCalibPaths, frameCapturePaths;
	if (ImGui::BeginMenu("Simulation"))
	{
		if (state.mode == MODE_Simulation)
		{
			if (ImGui::MenuItem("Stop Simulation###SimulationToggle"))
				StopSimulation(state);
		}
		else if (state.mode == MODE_Replay)
		{
			if (ImGui::MenuItem("Stop Replay###SimulationToggle"))
				StopReplay(state);
		}
		else
		{
			if (ImGui::MenuItem("Start Simulation###SimulationToggle", nullptr, nullptr, state.mode == MODE_None))
			{
				windows[WIN_CONTROL].open = true;
				StartSimulation(state);
			}
		}
		ImGui::Separator();

		if (ImGui::BeginMenu("Load Calibration Capture", state.mode == MODE_None))
		{
			cachePaths = true;
			if (!cachingCalibPaths)
			{
				cachingCalibPaths = true;
				pointCalibPaths.clear();
				std::string prefix = "dump/point_observations_";
				if (std::filesystem::exists(std::filesystem::path("dump")))
				{
					for (const auto &file : std::filesystem::directory_iterator("dump"))
					{
						if (file.path().string().compare(0, prefix.length(), prefix) == 0)
							pointCalibPaths.push_back(file.path());
					}
				}
				std::sort(pointCalibPaths.begin(), pointCalibPaths.end(), [&](auto &a, auto &b){
					int aNum, bNum;
					if (sscanf(a.string().data()+prefix.length(), "%d", &aNum) == 1
					 && sscanf(b.string().data()+prefix.length(), "%d", &bNum))
					 	return aNum > bNum;
					return a.string().compare(b.string()) > 0;
				});
			}
			if (pointCalibPaths.empty())
			{
				ImGui::MenuItem("No stored observation data", nullptr, nullptr, false);
			}
			for (const auto &file : pointCalibPaths)
			{
				if (!ImGui::MenuItem(file.filename().string().c_str())) continue;
				// Perform read in a separate thread to prevent blocking UI
				// TODO: Show indication that sample data is still loading - very bad without
				// e.g. global popup (then OpenPopup)
				// TODO: Memory Leak. Thread never joined
				new std::thread([](auto file)
				{
					ServerState &state = GetState();

					std::vector<CameraID> cameraIDs;
					SequenceData observations = parseSequenceDatabase(file.string(), cameraIDs);

					LOG(LGUI, LInfo, "Loading observations for %d cameras with %d markers!\n",
						(int)observations.temporary.size(), (int)observations.markers.size());
					if (state.mode != MODE_None)
					{
						LOG(LGUI, LWarn, "Already entered a mode, will not start replay!\n");
						return;
					}

					std::vector<CameraConfigRecord> cameras;
					for (int id : cameraIDs)
						cameras.emplace_back(id, 1280, 800);

					// Setup replay mode with relevant cameras
					StartReplay(state, cameras);

					// Load in observations
					state.pipeline.frameNum = observations.lastRecordedFrame;
					*state.pipeline.seqDatabase.contextualLock() = std::move(observations);

					UpdateErrorFromObservations(state.pipeline);
					GetUI().UpdateSequences(true);
				}, file);
			}
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Replay raw capture", state.mode == MODE_None))
		{
			cachePaths = true;
			if (!cachingCapturePaths)
			{
				cachingCapturePaths = true;
				frameCapturePaths.clear();
				std::string prefix = "dump/frame_capture_";
				if (std::filesystem::exists(std::filesystem::path("dump")))
				{
					for (const auto &file : std::filesystem::directory_iterator("dump"))
					{
						if (file.path().string().compare(0, prefix.length(), prefix) == 0
							&& file.path().extension().compare(".json") == 0)
							frameCapturePaths.push_back(file.path());
					}
				}
				std::sort(frameCapturePaths.begin(), frameCapturePaths.end(), [&](auto &a, auto &b){
					int aNum, bNum;
					if (sscanf(a.string().data()+prefix.length(), "%d", &aNum) == 1
					 && sscanf(b.string().data()+prefix.length(), "%d", &bNum))
					 	return aNum > bNum;
					return a.string().compare(b.string()) > 0;
				});
			}
			if (frameCapturePaths.empty())
			{
				ImGui::MenuItem("No stored observation data", nullptr, nullptr, false);
			}
			for (const auto &file : frameCapturePaths)
			{
				if (!ImGui::MenuItem(file.filename().string().c_str())) continue;
				// Perform read in a separate thread to prevent blocking UI
				// TODO: Show indication that sample data is still loading - very bad without
				// e.g. global popup (then OpenPopup)
				// TODO: Memory Leak. Thread never joined
				threadPool.push([](int, std::string file)
				{
					ServerState &state = GetState();
					std::vector<CameraConfigRecord> cameras;
					state.loadedFramePath = file;
					state.loadedFrameOffset = parseFrameRecords(state.loadedFramePath, cameras, state.loadedFrameRecords);
					parseTrackingResults(state.loadedFramePath, state.loadedFrameRecords, state.loadedFrameOffset);
					state.frameRecordReplayPos = 0;

					LOG(LGUI, LInfo, "Loaded %ld frames for replay!\n", state.loadedFrameRecords.size());
					if (state.mode != MODE_None)
					{
						LOG(LGUI, LWarn, "Already entered a mode, will not start replay!\n");
						return;
					}

					// Setup replay mode with relevant cameras
					StartReplay(state, cameras);
				}, file.string());
			}
			ImGui::EndMenu();
		}
		ImGui::EndMenu();
	}
	focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
	if (!cachePaths)
	{
		pointCalibPaths.clear();
		cachingCalibPaths = false;
		frameCapturePaths.clear();
		cachingCapturePaths = false;
	}

	ImGui::Dummy(ImVec2(20, ImGui::GetFrameHeight()));

	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.157f, 0.157f, 0.157f, 1.000f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.251f, 0.251f, 0.251f, 1.000f));

		ImGui::BeginDisabled(state.mode == MODE_None);
		if (ImGui::Button(state.isStreaming? "Stop Streaming###Streaming" : "Start Streaming###Streaming"))
		{
			if (state.isStreaming)
				StopStreaming(state);
			else
				StartStreaming(state);
		}
		focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);
		ImGui::SetItemTooltip("Once cameras are connected or a simulation has been started, pressing 'Start Streaming' will enable the cameras.");
		ImGui::EndDisabled();

		ImGui::PopStyleColor(2);
	}

	if (useCustomHeader)
	{
		ImVec2 maxIconSize = ImVec2(ImGui::GetFrameHeight()-ImGui::GetStyle().FramePadding.x*2, ImGui::GetFrameHeight()-ImGui::GetStyle().FramePadding.y*2);
		/* ImVec2 iconSize = ImVec2(24, 20); // For aspect ratio
		float factor = std::min(maxIconSize.x/iconSize.x, maxIconSize.y/iconSize.y);
		iconSize = ImVec2(iconSize.x*factor, iconSize.y*factor); */
		//if (ImGui::ImageButton("##WindowMinButton", darkModeIcons.visual, maxIconSize))

		ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_MenuBarBg));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.251f, 0.251f, 0.251f, 1.000f));

		ImGui::SetCursorPosX(ImGui::GetWindowWidth() - ImGui::GetFrameHeight()*3);
		if (ImGui::ArrowButton("##WindowMinButton", ImGuiDir_Down))
		{
			glfwIconifyWindow(glfwWindow);
		}
		focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);

		ImGui::SetCursorPosX(ImGui::GetWindowWidth() - ImGui::GetFrameHeight()*2);
		if (ImGui::ArrowButton("##WindowMaxButton", glfwGetWindowAttrib(glfwWindow, GLFW_MAXIMIZED)? ImGuiDir_Left : ImGuiDir_Up))
		{
			if (glfwGetWindowAttrib(glfwWindow, GLFW_MAXIMIZED))
				glfwRestoreWindow(glfwWindow);
			else
				glfwMaximizeWindow(glfwWindow);
		}
		focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);

		ImGui::SetCursorPosX(ImGui::GetWindowWidth() - ImGui::GetFrameHeight());
		if (CrossButton("##WindowCloseButton"))
		{
			GetApp().SignalQuitApp();
		}
		focusOnUIElement |= ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled);


		if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && mouseInHeader && !focusOnUIElement)
		{
			glfwDragWindow(glfwWindow);
		}

		if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && mouseInHeader && !focusOnUIElement)
		{
			if (glfwGetWindowAttrib(glfwWindow, GLFW_MAXIMIZED))
				glfwRestoreWindow(glfwWindow);
			else
				glfwMaximizeWindow(glfwWindow);
		}

		ImGui::PopStyleColor(2);
	}


	ImGui::EndMainMenuBar();

	ImGui::PopStyleVar(3);

	if (glfwGetWindowAttrib(glfwWindow, GLFW_HOVERED) && !glfwGetWindowAttrib(glfwWindow, GLFW_MAXIMIZED))
	{
		const float border = 5, wide = 10;
		ImVec2 mouse = ImGui::GetMousePos(), size = ImGui::GetMainViewport()->Size;
		float l = mouse.x, r = size.x-mouse.x, t = mouse.y, b = size.y-mouse.y;

		int borderFlag = 0;
		if ((l < wide || r < wide) && (t < wide || b < wide))
		{ // Two adjacent wide borders were triggered (wide corners)
			if (l < wide) borderFlag |= GLFW_WINDOW_LEFT;
			else if (r < wide) borderFlag |= GLFW_WINDOW_RIGHT;
			if (t < wide) borderFlag |= GLFW_WINDOW_TOP;
			else if (b < wide) borderFlag |= GLFW_WINDOW_BOTTOM;

			if (borderFlag == GLFW_WINDOW_TOPLEFT)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNW);
			else if (borderFlag == GLFW_WINDOW_BOTTOMRIGHT)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeSE);
			else if (borderFlag == GLFW_WINDOW_TOPRIGHT)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNE);
			else if (borderFlag == GLFW_WINDOW_BOTTOMLEFT)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeSW);
		}
		else
		{ // Check thin border
			if (l < border) borderFlag |= GLFW_WINDOW_LEFT;
			else if (r < border) borderFlag |= GLFW_WINDOW_RIGHT;
			if (t < border) borderFlag |= GLFW_WINDOW_TOP;
			else if (b < border) borderFlag |= GLFW_WINDOW_BOTTOM;

			if (borderFlag == GLFW_WINDOW_TOP)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeN);
			else if (borderFlag == GLFW_WINDOW_BOTTOM)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeS);
			else if (borderFlag == GLFW_WINDOW_RIGHT)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeE);
			else if (borderFlag == GLFW_WINDOW_LEFT)
				ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeW);
		}
		
		if (borderFlag != 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			glfwResizeWindow(glfwWindow, borderFlag);
	}

	static bool aboutPopupOpened = false;
	if (openAboutPopup)
	{
		openAboutPopup = false;
		ImGui::OpenPopup("About AsterTrack");
		aboutPopupOpened = true;
	}
	if (ImGui::BeginPopupModal("About AsterTrack", &aboutPopupOpened, ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoResize))
	{
		ImGui::Text("AsterTrack AsterTrack\n\n"
			"Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors\n"
			"\n"
			"This program is free software: you can redistribute it and/or modify\n"
			"it under the terms of the GNU General Public License as published by\n"
			"the Free Software Foundation, either version 3 of the License, or\n"
			"(at your option) any later version.\n"
			"\n"
			"This program is distributed in the hope that it will be useful,\n"
			"but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
			"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
			"GNU General Public License for more details.\n"
			"\n"
			"You should have received a copy of the GNU General Public License\n"
			"along with this program.  If not, see <https://www.gnu.org/licenses/>.");
		ImGui::EndPopup();
	}


	// Could have a secondary toolbar
	/* ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;
	float height = ImGui::GetFrameHeight();
	ImVec2 iconSize(24, 20);
	//float width = ImGui::GetCurrentContext()->Style.FramePadding.x * 4.0f + iconSize.x;
	//float height = ImGui::GetCurrentContext()->Style.FramePadding.y * 2.0f + iconSize.y;

	if (ImGui::BeginViewportSideBar("##SecondaryMenuBar", NULL, ImGuiDir_Up, height, window_flags))
	{
		if (ImGui::BeginMenuBar())
		{
			if (ImGui::ImageButton(darkModeIcons.wireless, iconSize))
			{
				LOG(LGUI, LDebug, "Pressed Test Button\n");
			}
			ImGui::EndMenuBar();
		}
	}
	ImGui::End(); */

}