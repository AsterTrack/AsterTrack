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
#include "ui/shared.hpp" // Signals
#include "ui/system/vis.hpp"

#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"

#include "app.hpp"

#include "stb/stb_image.h"
#include "gl/visualisation.hpp" // initVisualisation/cleanVisualisation
#include "imgui/imgui_onDemand.hpp"

#include "comm/usb.hpp"
#include "device/tracking_controller.hpp" // TrackingControllerState

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include "implot/implot.h"

#include <filesystem>

struct wl_display;
struct wl_resource;
#include "GL/glew.h"

#if defined(__unix__)
	#define ALLOW_CUSTOM_HEADER 1
#elif defined(_WIN32)
	#define ALLOW_CUSTOM_HEADER 1
	#if ALLOW_CUSTOM_HEADER
	#define GLFW_EXPOSE_NATIVE_WIN32
	#include "GLFW/glfw3native.h"
	#endif
#endif

#if defined(__GNUC__) // GCC
	#define EXPORT __attribute__((visibility("default")))
#else // Clang, MSVC
	#define EXPORT __declspec(dllexport)
#endif


/*
 * Interface for the AsterTrack application
 */

InterfaceState *InterfaceInstance;

const int maxRefreshRateHz = 60; // This is further limited to display refresh rate
const int minRefreshRateHz = 10; // Can also set to 0 and set waitIntervalS to 50.0f/1000.0f

// Minimum render/update interval, no faster allowed
const long minIntervalUS = 1000000/maxRefreshRateHz;
// Maximum interval between updates
const double waitIntervalS = minRefreshRateHz? (1.0f/minRefreshRateHz - 1.0f/maxRefreshRateHz) : (0.1f);

/* Function prototypes */

// GLFW Platform Window
static GLFWwindow* setupPlatformWindow(bool &useHeader);
static void closePlatformWindow(GLFWwindow *windowHandle);
static void RefreshGLFWWindow(GLFWwindow *window);
// ImGui Code
static std::string loadedFont, selectedFont;
static void updateFont();
static void readSettingsFile(ImGuiContext *context, ImGuiSettingsHandler *settings);
static void* readCustomSettingsHeader(ImGuiContext *context, ImGuiSettingsHandler *settings, const char *header);
static void readCustomSettingsLine(ImGuiContext *context, ImGuiSettingsHandler *settings, void *entry, const char *line);
static void writeCustomSettings(ImGuiContext *context, ImGuiSettingsHandler *settings, ImGuiTextBuffer *output);

/**
 * Main loop of UI
 */


static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Error: %s\n", description);
	LOG(LGUI, LError, "Error: %s\n", description);
}
extern "C" {

EXPORT bool _InterfaceThread()
{
	InterfaceState ui;
	if (!InterfaceInstance)
		return false;

	// Open platform window
	glfwSetErrorCallback(glfw_error_callback);
	ui.glfwWindow = setupPlatformWindow(ui.useCustomHeader);
	glfwMakeContextCurrent(ui.glfwWindow);
	glfwSetWindowRefreshCallback(ui.glfwWindow, RefreshGLFWWindow);

	// Initialise UI resources (e.g. ImGui, Themes, Icons and OpenGL)
	if (!ui.Init())
	{
		glfwMakeContextCurrent(nullptr);
		GetApp().SignalInterfaceClosed();
		GetApp().SignalQuitApp();
		return false;
	}

	// Sync state with server if necessary
	ui.UpdateCameras();
	ui.UpdateCalibrations();
	ui.UpdateSequences(true);

	// Render loop, starting off with 3 full UI update iterations
	ui.requireUpdates = 3;
	while (!glfwWindowShouldClose(ui.glfwWindow) && !ui.setCloseInterface)
	{
		if (ui.cameraListDirty)
			ui.UpdateCameras();

		if (ui.calibrationsDirty)
			ui.UpdateCalibrations();

		// Record render time
		auto now = sclock::now();
		ui.deltaTime = dtMS(ui.renderTime, now)/1000.0f;
		ui.renderTime = now;

		if (dtS(ui.updateTime, now) > waitIntervalS)
			ui.requireUpdates = std::max(ui.requireUpdates, 1);

		// Update/render UI as requested
		if (ui.requireUpdates)
		//if (ui.requireUpdates || ui.requireRender)
		{ // ImGui UI needs updates
			ui.requireUpdates--;
			ui.requireRender = false;
			ui.updateTime = now;
			ui.UpdateUI();
			ui.RenderUI(true);
		}
		else if (ui.requireRender)
		{ // Render parts of the screen with OnDemand rendered items
			ui.requireRender = false;
			ui.RenderUI(false);
		}

		// Wait a minimum amount to limit maximum refresh rate
		long curIntervalUS = dtUS(ui.renderTime, sclock::now());
		if (curIntervalUS < minIntervalUS)
			std::this_thread::sleep_for(std::chrono::microseconds(minIntervalUS-curIntervalUS));

		// Wait for input events or update/render requests while updating general state
		glfwPollEvents();
		ui.GeneralUpdate();
		while (!ui.requireRender && ui.requireUpdates == 0
			&& ImGui::GetCurrentContext()->InputEventsQueue.empty())
		{ // This timeout greatly influences idle power consumption
			glfwWaitEventsTimeout(waitIntervalS);
			ui.GeneralUpdate();
			if (minRefreshRateHz > 0) break;
		}
		if (!ImGui::GetCurrentContext()->InputEventsQueue.empty())
			ui.requireUpdates = std::max(ui.requireUpdates, 3);
	}

	glfwSetWindowRefreshCallback(ui.glfwWindow, nullptr);

	// Clean up UI resources
	ui.Exit();

	// If native decorations were used to close window, we haven't notified App yet to quit
	if (glfwWindowShouldClose(ui.glfwWindow))
		GetApp().SignalQuitApp();

	// Close platform window
	glfwMakeContextCurrent(nullptr);
	closePlatformWindow(ui.glfwWindow);
	GetApp().SignalInterfaceClosed();

	return true;
}

}

static void RefreshGLFWWindow(GLFWwindow *window)
{
	if (!InterfaceInstance) return;
	InterfaceState &ui = GetUI();
	if (!ui.init) return;

	// Request more renders after resizing is done, since we early out sometimes and might leave an invalid buffer 
	ui.RequestUpdates(3);

	// Check render time
	auto now = sclock::now();
	if (dtUS(ui.renderTime, now) < minIntervalUS) return;
	ui.deltaTime = dtMS(ui.renderTime, now)/1000.0f;
	ui.renderTime = now;

	// Update/render UI
	ui.GeneralUpdate();
	ui.UpdateUI();
	ui.RenderUI(true);
}


/**
 * UI Logic
 */

void InterfaceState::UpdateUI()
{
	updateFont();

	std::shared_lock dev_lock(GetState().deviceAccessMutex);

	// Start new UI frame
	ImGui_ImplGlfw_NewFrame();
	ImGui_ImplOpenGL3_NewFrame();
	ImGui::NewFrame();

	OnDemandNewFrame();

	GeneralInput();

	// Settings will have been loaded in NewFrame if an imgui.ini file existed
	static bool firstIteration = true;
	if (firstIteration)
	{
		firstIteration = false;
		if (!loadedSettingsIni)
		{
			ResetWindowLayout();
		}
	}

	ImVec4 baseCol = ImGui::GetStyleColorVec4(ImGuiCol_Text);
	isDarkMode = baseCol.x + baseCol.y + baseCol.z > 1.5f;

	UpdateMainMenuBar();

	// Dockspace for other windows to dock to, covering whole viewport
	ImGui::DockSpaceOverViewport(dockspaceID, ImGui::GetMainViewport());

	for (auto &window : windows)
	{
		if (window.open || window.alwaysUpdate)
			(this->*window.updateWindow)(window);
	}

	// Windows for detached camera views
	for (auto &viewIt : cameraViews)
	{
		if (viewIt.second.isDetached)
			UpdateDetachedCameraView(viewIt.second);
		else
		 	viewIt.second.detachedIndex = -1;
	}

	static bool handlingErrors = false;
	if (!handlingErrors && !GetState().errors.empty())
	{
		// TODO: Improve error handling when they occurred long before interface opened
		// This is currently not possible, but may want to use a better way to show older errors that happened while interface was closed than flooding the user
		handlingErrors = true;
		ImGui::OpenPopup("ErrorPop");
	}
	if (ImGui::BeginPopup("ErrorPop"))
	{
		ErrorMessage &error = GetState().errors.front();
		ImGui::Text("%s", error.c_str());
		ImVec2 buttonWidth((LineWidth()-ImGui::GetStyle().ItemSpacing.x)/2, ImGui::GetFrameHeight());
		if (ImGui::Button("Ok", SizeWidthFull()))
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
	if (handlingErrors && !ImGui::IsPopupOpen("ErrorPop"))
	{
		// TODO: Handle popups for multiple errors better
		// - keep next popup in same placed as last one (or never really close it if possible)
		// - show number of errors and allow to OK/Clear all
		GetState().errors.pop();
		handlingErrors = false;
	}

	// Render UI out to a DrawData list, for later GL rendering
	ImGui::Render();
}

void InterfaceState::RenderUI(bool fullUpdate)
{
	if (fullUpdate)
	{
		// Render 2D UI with callbacks at appropriate places for 3D GL
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}
	else
	{
		// Render areas of the screen with on-demand items - rest will be discarded
		for (OnDemandItem &onDemandState : onDemandStack)
		{
			if (onDemandState.renderOwn)
				ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData(), onDemandState.clip.Min, onDemandState.clip.Max);
		}
	}

	glfwMakeContextCurrent(glfwWindow);
	glfwSwapBuffers(glfwWindow);

	if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	{
		ImGui::UpdatePlatformWindows();
		ImGui::RenderPlatformWindowsDefault();
	}
}

void InterfaceState::GeneralInput()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	{
		VisTargetLock visTarget = visState.lockVisTarget();
		if (visTarget.hasObs())
		{ // Have a target with observation data to visualise and locked it
			int frameJumpInterval = 1;
			if (ImGui::IsKeyPressed(ImGuiKey_Space))
			{
				RequestUpdates();
				targetFramesAdvancing = !targetFramesAdvancing;
			}
			if (ImGui::IsKeyPressed(ImGuiKey_Minus) || ImGui::IsKeyPressed(ImGuiKey_LeftArrow))
			{
				RequestUpdates();
				visState.targetCalib.frameIdx -= frameJumpInterval;
			}
			if (ImGui::IsKeyPressed(ImGuiKey_KeypadAdd) || ImGui::IsKeyPressed(ImGuiKey_RightArrow))
			{
				RequestUpdates();
				visState.targetCalib.frameIdx += frameJumpInterval;
			}
			visState.updateVisTarget(visTarget);
		}
	}

	if (state.mode == MODE_Device)
	{
		if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_1))
		{
			for (auto &controller : state.controllers)
			{
				comm_submit_control_data(controller->comm, COMMAND_OUT_TEST, 0, 0);
			}
		}
		else if (ImGui::GetIO().KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_2))
		{
			for (auto &controller : state.controllers)
			{
				comm_submit_control_data(controller->comm, COMMAND_OUT_TEST, 1, 0);
			}
		}
	}
}

void InterfaceState::GeneralUpdate()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	{
		VisTargetLock visTarget = visState.lockVisTarget();
		if (visTarget.hasObs() && targetFramesAdvancing)
		{
			static TimePoint_t lastFrameIncrease;
			if (dtMS(lastFrameIncrease, sclock::now()) > 1000.0f/60)
			{
				lastFrameIncrease = sclock::now();

				visState.targetCalib.frameIdx++;
				visState.updateVisTarget(visTarget);

				RequestUpdates();
			}
		} 
	}

	if ((state.mode == MODE_Device || state.simAdvance.load() != 0) &&
		(state.mode != MODE_Replay || pipeline.frameNum < state.recording.frames) &&
		(pipeline.phase == PHASE_Calibration_Point || pipeline.phase == PHASE_Calibration_Target) && pipeline.recordSequences)
	{
		static TimePoint_t lastObservationUpdate, lastSequenceReset;
		if (dtMS(lastObservationUpdate, sclock::now()) > 100 || visState.incObsUpdate.resetFirstFrame >= 0 || calibError.triggered)
		{ // Incrementally update sequence observations for visualisation / stats in UI

			if (pipeline.phase == PHASE_Calibration_Point && dtMS(lastSequenceReset, sclock::now()) > 1000)
			{ // Regularly trigger full reset of observations
				// TODO: Fix incremental observation update (2/3)
				lastSequenceReset = sclock::now();
				visState.incObsUpdate.resetFirstFrame = 0;
			}

			lastObservationUpdate = sclock::now();
			UpdateSequences();
		}
	}
}

void InterfaceState::ResetWindowLayout()
{
	// Create root dockspace covering the whole main viewport (will be replaced if it already exists)
	ImGui::DockBuilderAddNode(dockspaceID, ImGuiDockNodeFlags_DockSpace);
	ImGui::DockBuilderSetNodePos(dockspaceID, ImGui::GetMainViewport()->WorkPos);
	ImGui::DockBuilderSetNodeSize(dockspaceID, ImGui::GetMainViewport()->WorkSize);

	// Define layout panels
	ImGuiID mainPanelID, bottomPanelID, sidePanelID, auxPanelID, edgePanelID;
	ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Right, 0.4f, &sidePanelID, &mainPanelID);
	ImGui::DockBuilderSplitNode(sidePanelID, ImGuiDir_Right, 0.3f, &edgePanelID, &sidePanelID);
	ImGui::DockBuilderSplitNode(sidePanelID, ImGuiDir_Down, 0.3f, &auxPanelID, &sidePanelID);	
	ImGui::DockBuilderSplitNode(mainPanelID, ImGuiDir_Down, 0.5f, &bottomPanelID, &mainPanelID);

	// Associate all static windows with a panel by default

	ImGui::DockBuilderDockWindow(windows[WIN_3D_VIEW].title.c_str(), mainPanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_CAMERA_VIEWS].title.c_str(), mainPanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_VISUALISATION].title.c_str(), auxPanelID);

	ImGui::DockBuilderDockWindow(windows[WIN_PIPELINE].title.c_str(), sidePanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_CONTROL].title.c_str(), auxPanelID);
	
	ImGui::DockBuilderDockWindow(windows[WIN_LOGGING].title.c_str(), bottomPanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_INSIGHTS].title.c_str(), bottomPanelID);

	ImGui::DockBuilderDockWindow(windows[WIN_TRACKERS].title.c_str(), auxPanelID);

	ImGui::DockBuilderDockWindow(windows[WIN_DEVICES].title.c_str(), edgePanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_CAMERA_SETTINGS].title.c_str(), sidePanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_WIRELESS].title.c_str(), auxPanelID);

	ImGui::DockBuilderDockWindow(windows[WIN_SEQUENCE_PARAMS].title.c_str(), sidePanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_POINT_CALIB_PARAMS].title.c_str(), sidePanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_TARGET_CALIB_PARAMS].title.c_str(), sidePanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_TRACKING_PARAMS].title.c_str(), sidePanelID);

	ImGui::DockBuilderDockWindow(windows[WIN_LENS_SELECTION_TOOL].title.c_str(), sidePanelID);

	ImGui::DockBuilderDockWindow(windows[WIN_STYLE_EDITOR].title.c_str(), sidePanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_IMGUI_DEMO].title.c_str(), mainPanelID);
	ImGui::DockBuilderDockWindow(windows[WIN_IMPLOT_DEMO].title.c_str(), mainPanelID);

	ImGui::DockBuilderFinish(dockspaceID);
}


/* Default ImGUI windows */

void InterfaceState::UpdateStyleUI(InterfaceWindow &window)
{
	if (ImGui::Begin(window.title.c_str(), &window.open))
		ImGui::ShowStyleEditor();
	ImGui::End();
}

void InterfaceState::UpdateImGuiDemoUI(InterfaceWindow &window)
{
	ImGui::ShowDemoWindow(&window.open);
}

void InterfaceState::UpdateImPlotDemoUI(InterfaceWindow &window)
{
	ImPlot::ShowDemoWindow(&window.open);
}


/**
 * UI Setup
 */

bool InterfaceState::Init()
{
	// ImGui Init

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();

	dockspaceID = ImHashStr("MainDockSpace");

	{ // Configure ImGui
		ImGuiIO& io = ImGui::GetIO();
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;		// Enable Keyboard Controls
		//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;		// Enable Gamepad Controls
		io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;			// Enable Docking
		//io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;		// Enable Multi-Viewport / Platform Windows

		// Viewport breaks both on Wayland and X11
		// Wayland: Doesn't support screenspace coordinates, which viewport window explicitly switches to and requires
		// 			So inputs are offset, and windows can't move themselves - resizing from edge will change size, but only bottom/right, not top/left
		// X11: TBD
		// VERDICT: Fixable in user code on X11, harder to fix on wayland (either backend or GLFW fix)

		//io.ConfigViewportsNoAutoMerge = true;
		//io.ConfigViewportsNoTaskBarIcon = true;
		//io.ConfigWindowsResizeFromEdges = true;
		io.ConfigWindowsMoveFromTitleBarOnly = true;
	}

	StyleSizingAsterDark();
	StyleColorsAsterDark();
	StyleSelectFont();
	updateFont();

	{ // Read/Write custom UI state settings
		ImGuiSettingsHandler uiStateHandler;
		uiStateHandler.TypeName = "UIState";
		uiStateHandler.TypeHash = ImHashStr("UIState");
		uiStateHandler.ReadOpenFn = readCustomSettingsHeader;
		uiStateHandler.ReadLineFn = readCustomSettingsLine;
		uiStateHandler.WriteAllFn = writeCustomSettings;
		uiStateHandler.ApplyAllFn = readSettingsFile;
		ImGui::AddSettingsHandler(&uiStateHandler); // Does not keep the pointer, so safe, just... odd
	}

	if (!ImGui_ImplGlfw_InitForOpenGL(glfwWindow, true))
		return false;
	//const char* glsl_version = "#version 150";
	if (!ImGui_ImplOpenGL3_Init())
		return false;

	// OpenGL configuration

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBlendEquation(GL_FUNC_ADD);

	glEnable(GL_SCISSOR_TEST);

	glEnable(GL_MULTISAMPLE);

	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glEnable(GL_POINT_SPRITE);

	// GL Visualisation Init

	initVisualisation();

	// GL Icons Init

	int iconLoadFailures = 0;
	auto loadIcon = [&iconLoadFailures](const char *path)
	{
		int x, y, n;
		uint8_t *imageData = stbi_load(path, &x, &y, &n, 4);
		if (!imageData)
		{
			printf("Failed to load '%s': %s\n", path, stbi_failure_reason());
			LOG(LGUI, LError, "Failed to load '%s': %s", path, stbi_failure_reason());
			iconLoadFailures++;
			return (ImTextureID)0;
		}


		GLuint tex;
		glGenTextures(1, &tex);
		glBindTexture(GL_TEXTURE_2D, tex);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, x, y, 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData);

		glGenerateMipmap(GL_TEXTURE_2D);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		free(imageData);
		return (ImTextureID)(intptr_t)tex;
	};

	#define ICON_LOAD(NAME) \
		lightModeIcons.NAME = loadIcon("resources/icons/" #NAME ".png"); \
		darkModeIcons.NAME = loadIcon("resources/icons/" #NAME "_d.png"); \

	if (std::filesystem::exists("resources/icons/"))
	{
		ICON_LOAD(frame_wireless)
		ICON_LOAD(frame_hdmi)
		ICON_LOAD(visual)
		ICON_LOAD(mode)

		ICON_LOAD(orbit)
		ICON_LOAD(detach)
		ICON_LOAD(vdots)

		ICON_LOAD(controller)
		ICON_LOAD(camera)

		ICON_LOAD(imu_calib)
		ICON_LOAD(imu_track)
		ICON_LOAD(imu_lost)

		ICON_LOAD(wireless)
		ICON_LOAD(no_wireless)
		ICON_LOAD(server)
		ICON_LOAD(no_server)
		ICON_LOAD(ssh)
		ICON_LOAD(no_ssh)

		if (iconLoadFailures > 0)
			SignalErrorToUser(asprintf_s("Failed to load %d icons - check Logging view!", iconLoadFailures));
	}
	else if (std::filesystem::exists("../resources/"))
	{
		SignalErrorToUser("'resources' folder not found in working directory but in parent directory! Make sure to run AsterTrack in the program root directory!");
	}
	else
	{
		SignalErrorToUser("'resources' folder not found in working directory! Make sure to run AsterTrack in the program root directory!");
	}


	// Initialise all static UI windows

	// Scene visualisation
	windows[WIN_3D_VIEW] = InterfaceWindow("3D View", &InterfaceState::Update3DViewUI, true);
	windows[WIN_CAMERA_VIEWS] = InterfaceWindow("Camera Views", &InterfaceState::UpdateCameraViews, true);
	windows[WIN_VISUALISATION] = InterfaceWindow("Visualisation", &InterfaceState::UpdateVisualisationSettings, true);
	// Primary interface
	windows[WIN_PIPELINE] = InterfaceWindow("Pipeline", &InterfaceState::UpdatePipeline, true);
	windows[WIN_CONTROL] = InterfaceWindow("Control", &InterfaceState::UpdateControl, true);
	// Development
	windows[WIN_LOGGING] = InterfaceWindow("Logging", &InterfaceState::UpdateLogging, true);
	windows[WIN_INSIGHTS] = InterfaceWindow("Insights", &InterfaceState::UpdateInsights, true);
	// Database
	windows[WIN_TRACKERS] = InterfaceWindow("Trackers", &InterfaceState::UpdateTrackers, false);
	// Device mode
	windows[WIN_DEVICES] = InterfaceWindow("Devices", &InterfaceState::UpdateDevices, false);
	windows[WIN_CAMERA_SETTINGS] = InterfaceWindow("Camera Settings", &InterfaceState::UpdateCameraSettings, false);
	windows[WIN_WIRELESS] = InterfaceWindow("Wireless Setup", &InterfaceState::UpdateWirelessSetup, false);
	// Parameters
	windows[WIN_SEQUENCE_PARAMS] = InterfaceWindow("Sequence2D Params", &InterfaceState::UpdateSequenceParameters, false);
	windows[WIN_POINT_CALIB_PARAMS] = InterfaceWindow("PointCalib Params", &InterfaceState::UpdatePointCalibParameters, false);
	windows[WIN_TARGET_CALIB_PARAMS] = InterfaceWindow("TargetCalib Params", &InterfaceState::UpdateTargetCalibParameters, false);
	windows[WIN_TRACKING_PARAMS] = InterfaceWindow("Tracking Params", &InterfaceState::UpdateTrackingParameters, false);
	// Tools
	windows[WIN_LENS_SELECTION_TOOL] = InterfaceWindow("Lens Selection", &InterfaceState::UpdateLensSelectionTool, false);
	// Shortcut to ImGui's built-in style editor
	windows[WIN_STYLE_EDITOR] = InterfaceWindow("Style Editor", &InterfaceState::UpdateStyleUI, false, false);
	// Useful tool to debug and research UI
	windows[WIN_IMGUI_DEMO] = InterfaceWindow("Dear ImGui Demo", &InterfaceState::UpdateImGuiDemoUI, false, false);
	windows[WIN_IMPLOT_DEMO] = InterfaceWindow("ImPlot Demo", &InterfaceState::UpdateImPlotDemoUI, false, false);

	// Initialise 3D View
	view3D.pitch = (float)PI/2.0f;
	view3D.heading = +(float)PI;
	view3D.distance = 1.0f;
	view3D.viewTransform.translation() = Eigen::Vector3f(0, -4, 1.8f);
	view3D.viewTransform.linear() = getRotationXYZ(Eigen::Vector3f(view3D.pitch, 0, view3D.heading));

	// Initialise log
	for (int i = 0; i < LMaxCategory; i++)
		LogFilterTable[i] = LInfo;

	init = true;

	return true;
}

void InterfaceState::Exit()
{
	init = false;

	// Cleanup GL
	cleanVisualisation();
	// TODO: Properly clean icon GL Textures (loaded above, never unloaded)

	// Cleanup OnDemand drawing system
	CleanupOnDemand();

	// Exit ImGui
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();
}

void InterfaceState::RequestUpdates(int count)
{
	requireUpdates = std::max(requireUpdates, count);
	glfwPostEmptyEvent(); // Wake up UI thread
}

void InterfaceState::RequestRender()
{
	requireRender = true;
	glfwPostEmptyEvent(); // Wake up UI thread
}


/**
 * Signals for UI
 */

extern "C" {

EXPORT void _SignalShouldClose()
{
	if (!InterfaceInstance || InterfaceInstance->setCloseInterface || !ImGui::GetCurrentContext())
		return; // UI not initialised
	GetUI().setCloseInterface = true;
	glfwPostEmptyEvent(); // Wake up UI thread
}

EXPORT void _SignalLogUpdate()
{
	if (!InterfaceInstance || InterfaceInstance->setCloseInterface || !ImGui::GetCurrentContext())
		return; // UI not initialised
	if (!GetUI().logsStickToNew)
		return; // Don't need to update with new logs
	ImGuiWindow* imguiWindow = ImGui::FindWindowByName(GetUI().windows[WIN_LOGGING].title.c_str());
	if (!(imguiWindow && !imguiWindow->Hidden && imguiWindow->DockTabIsVisible))
		return; // Log is not visible
	//GetUI().RequestUpdates(1);
	// Not interactive, no need to wake UI thread to render now, especially if this gets spammed a lot
	GetUI().requireUpdates = std::max(GetUI().requireUpdates, 1);
}

EXPORT void _SignalCameraRefresh(CameraID id)
{
	if (!InterfaceInstance || InterfaceInstance->setCloseInterface || !ImGui::GetCurrentContext())
		return; // UI not initialised
	// Update parts of the screen with OnDemand rendered items
	if (id == 0)
		GetUI().RequestRender();
	else // TODO: Update one camera? Doesn't seem reasonable nor necessary
		GetUI().RequestRender();
}

EXPORT void _SignalPipelineUpdate()
{
	if (!InterfaceInstance || InterfaceInstance->setCloseInterface || !ImGui::GetCurrentContext())
		return; // UI not initialised
	GetUI().targetViewsDirty = true;
	GetUI().RequestUpdates();
}

EXPORT void _SignalObservationReset(long firstFrame = 0)
{
	if (!InterfaceInstance || InterfaceInstance->setCloseInterface || !ImGui::GetCurrentContext())
		return; // UI not initialised
	GetUI().visState.incObsUpdate.resetFirstFrame = firstFrame;
	GetUI().RequestUpdates();
}

static void CleanVisualisationState()
{
	GetUI().visState.observations = {};
	GetUI().visState.target = {};
	GetUI().visState.targetCalib = {};
	GetUI().visState.incObsUpdate = {};
	GetUI().visState.tracking.targets = {};
	GetUI().visState.tracking.debug = {};
}

EXPORT void _SignalServerEvent(ServerEvents event)
{
	if (!InterfaceInstance || InterfaceInstance->setCloseInterface || !ImGui::GetCurrentContext())
		return; // UI not initialised
	switch(event)
	{
		case EVT_DEVICE_DISCONNECT:
			break;
		case EVT_MODE_DEVICE_START:
			GetUI().windows[WIN_DEVICES].open = true;
			break;
		case EVT_MODE_DEVICE_STOP:
			CleanVisualisationState();
			break;
		case EVT_MODE_SIMULATION_START:
			break;
		case EVT_MODE_SIMULATION_STOP:
			CleanVisualisationState();
			break;
		case EVT_START_STREAMING:
			GetUI().recordSections.clear();
			GetUI().recordSectionStart = -1;
			break;
		case EVT_STOP_STREAMING:
			// TODO: Might need UI drawing lock, not in UI thread right now
			GetUI().visState.tracking.targets.clear();
			GetUI().visState.tracking.focusedTrackerID = 0;
			break;
		case EVT_UPDATE_CAMERAS:
			GetUI().cameraListDirty = true;
			GetUI().RequestUpdates();
			break;
		case EVT_UPDATE_CALIBS:
			GetUI().calibrationsDirty = true;
			GetUI().RequestUpdates();
			break;
		case EVT_UPDATE_EVENTS:
			if (GetUI().seqEvents)
			{ // A controller event sequence tab is active
				// But no need to wake UI thread to update immediately
				GetUI().requireUpdates = std::max(GetUI().requireUpdates, 1);
			}
			break;
		case EVT_UPDATE_INTERFACE:
			GetUI().RequestUpdates();
			break;
		default:
			break;
	}
}

}


/**
 * GLFW Platform Window
 */

static GLFWwindow* setupPlatformWindow(bool &useHeader)
{

	/* Select platform-specific feature set */

#ifdef __unix__

#if ALLOW_CUSTOM_HEADER
	// Custom header works for both X11 and wayland with a customised glfw
	useHeader = true;
#endif

	// Use wayland if available and desired, X11 otherwise
	const char *backendHint = std::getenv("GDK_BACKEND");
	if (backendHint && strcasecmp(backendHint, "x11") == 0)
	{ // We're not gtk, but this is a common method, so why not
		glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
		if (!glfwInit())
			return nullptr;
	}
	else
	{ // Try to initialise with wayland, fall back to X11
		glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_WAYLAND);
		if (useHeader)
			glfwInitHint(GLFW_WAYLAND_LIBDECOR, false);
		if (!glfwInit())
		{ // No wayland
			glfwInitHint(GLFW_PLATFORM, GLFW_ANY_PLATFORM);
			if (!glfwInit())
				return nullptr;
		}
	}

	// OpenGL 3.0 works just fine, 3.3 drops some simpler functions
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#elif _WIN32

#if ALLOW_CUSTOM_HEADER
	// Moving window worked with a glfw patch, but automatic docking/fullscreening didn't
	// Sizing options are available with a further call though
	// Still, disable until it works better
	useHeader = false;
#endif

	if (!glfwInit())
		return nullptr;

	// Colors didn't work on 3.0 context
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#else

	#error "Platform currently not supported!"

#endif

	/* Setup GLFW window */

	{
		GLFWmonitor *monitor = glfwGetPrimaryMonitor();
		const GLFWvidmode *mode = glfwGetVideoMode(monitor);
		glfwWindowHint(GLFW_RED_BITS, mode->redBits);
		glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
		glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);
		glfwWindowHint(GLFW_DOUBLEBUFFER, true);
		glfwWindowHint(GLFW_SAMPLES, 4);
		//glfwWindowHint(GLFW_MAXIMIZED, true); // Keeps specified size, so problem of "what is fullscreen - taskbar" is not solved
	}

#if ALLOW_CUSTOM_HEADER
	if (useHeader)
	{
		glfwWindowHint(GLFW_DECORATED, false);
		glfwWindowHint(GLFW_RESIZABLE, true);
	}
#else
	useHeader = false;
#endif

	// Open main window
	GLFWwindow *glfwWindow = glfwCreateWindow(1280, 720, "AsterTrack", nullptr, nullptr);
	if (!glfwWindow)
	{
		glfwTerminate();
		return nullptr;
	}
	glfwMaximizeWindow(glfwWindow);

#if ALLOW_CUSTOM_HEADERS && defined(_WIN32)
	if (useHeader)
	{ // Enable resizing handles, should disable in fullscreen
		HWND hwnd = glfwGetWin32Window(glfwWindow);
		long style = GetWindowLongA(hwnd, GWL_STYLE);
		SetWindowLongA(hwnd, GWL_STYLE, style | WS_SIZEBOX);
	}
#endif

	// GLEW Init (temporarily grabbing window coontext for this thread)
	glfwMakeContextCurrent(glfwWindow);
	GLenum err = glewContextInit();
	glfwMakeContextCurrent(nullptr);
	if (GLEW_OK != err)
	{
		LOG(LGUI, LError, "GLEW Init Error %d\n", err);
		fprintf(stderr, "GLEW Init Error: '%s'\n", glewGetErrorString(err));
		glfwTerminate();
		return nullptr;
	}

	return glfwWindow;
}

static void closePlatformWindow(GLFWwindow *windowHandle)
{
	if (windowHandle)
		glfwDestroyWindow((GLFWwindow*)windowHandle);
	glfwTerminate();
}


/**
 * ImGui code
 */

void InterfaceState::StyleSizingDefault(ImGuiStyle *dst)
{
	ImGuiStyle newStyle;
	std::memcpy(&newStyle.Colors, &ImGui::GetStyle().Colors, sizeof(newStyle.Colors));
	ImGuiStyle &style = dst ? *dst : ImGui::GetStyle();
	style = newStyle;
}

void InterfaceState::StyleSizingAsterDark(ImGuiStyle *dst)
{
	ImGuiStyle newStyle;
	std::memcpy(&newStyle.Colors, &ImGui::GetStyle().Colors, sizeof(newStyle.Colors));
	ImGuiStyle &style = dst ? *dst : ImGui::GetStyle();
	style = newStyle;

	//style.Alpha							= 1.0f;					// Global alpha applies to everything in Dear ImGui.
	//style.DisabledAlpha					= 0.60f;				// Additional alpha multiplier applied by BeginDisabled(). Multiply over current value of Alpha.
	style.WindowPadding					= ImVec2(7,7);		// Padding within a window
	//style.WindowRounding				= 0.0f;					// Radius of window corners rounding. Set to 0.0f to have rectangular windows. Large values tend to lead to variety of artifacts and are not recommended.
	//style.WindowBorderSize				= 1.0f;					// Thickness of border around windows. Generally set to 0.0f or 1.0f. Other values not well tested.
	//style.WindowBorderHoverPadding		= 4.0f;					// Hit-testing extent outside/inside resizing border. Also extend determination of hovered window. Generally meaningfully larger than WindowBorderSize to make it easy to reach borders.
	//style.WindowMinSize					= ImVec2(32,32);		// Minimum window size
	//style.WindowTitleAlign				= ImVec2(0.0f,0.5f);	// Alignment for title bar text
	style.WindowMenuButtonPosition		= ImGuiDir_None;		// Position of the collapsing/docking button in the title bar (left/right). Defaults to ImGuiDir_Left.
	style.ChildRounding					= 2.0f;					// Radius of child window corners rounding. Set to 0.0f to have rectangular child windows
	//style.ChildBorderSize				= 1.0f;					// Thickness of border around child windows. Generally set to 0.0f or 1.0f. Other values not well tested.
	//style.PopupRounding					= 0.0f;					// Radius of popup window corners rounding. Set to 0.0f to have rectangular child windows
	//style.PopupBorderSize				= 1.0f;					// Thickness of border around popup or tooltip windows. Generally set to 0.0f or 1.0f. Other values not well tested.
	style.FramePadding					= ImVec2(8,4);	 	// Padding within a framed rectangle (used by most widgets)
	style.FrameRounding					= 2.0f;					// Radius of frame corners rounding. Set to 0.0f to have rectangular frames (used by most widgets).
	//style.FrameBorderSize				= 0.0f;					// Thickness of border around frames. Generally set to 0.0f or 1.0f. Other values not well tested.
	style.ItemSpacing					= ImVec2(6,6);	 	// Horizontal and vertical spacing between widgets/lines
	style.ItemInnerSpacing				= ImVec2(6,6);	 	// Horizontal and vertical spacing between within elements of a composed widget (e.g. a slider and its label)
	style.CellPadding					= ImVec2(4,4);	 	// Padding within a table cell. Cellpadding.x is locked for entire table. CellPadding.y may be altered between different rows.
	//style.TouchExtraPadding				= ImVec2(0,0);	 	// Expand reactive bounding box for touch-based system where touch position is not accurate enough. Unfortunately we don't sort widgets so priority on overlap will always be given to the first widget. So don't grow this too much!
	style.IndentSpacing					= 20.0f;				// Horizontal spacing when e.g. entering a tree node. Generally == (FontSize + FramePadding.x*2).
	style.ColumnsMinSpacing				= 8.0f;					// Minimum horizontal spacing between two columns. Preferably > (FramePadding.x + 1).
	style.ScrollbarSize					= 20.0f;				// Width of the vertical scrollbar, Height of the horizontal scrollbar
	style.ScrollbarRounding				= 10.0f;					// Radius of grab corners rounding for scrollbar
	style.GrabMinSize					= 8.0f;				// Minimum width/height of a grab box for slider/scrollbar
	style.GrabRounding					= 4.0f;					// Radius of grabs corners rounding. Set to 0.0f to have rectangular slider grabs.
	//style.LogSliderDeadzone				= 4.0f;					// The size in pixels of the dead-zone around zero on logarithmic sliders that cross zero.
	//style.ImageBorderSize				= 0.0f;					// Thickness of border around tabs.
	style.TabRounding					= 4.0f;					// Radius of upper corners of a tab. Set to 0.0f to have rectangular tabs.
	//style.TabBorderSize					= 0.0f;					// Thickness of border around tabs.
	style.TabCloseButtonMinWidthSelected = 0.0f;				// -1: always visible. 0.0f: visible when hovered. >0.0f: visible when hovered if minimum width.
	//style.TabCloseButtonMinWidthUnselected = 0.0f;				// -1: always visible. 0.0f: visible when hovered. >0.0f: visible when hovered if minimum width. FLT_MAX: never show close button when unselected.
	//style.TabBarBorderSize				= 1.0f;					// Thickness of tab-bar separator, which takes on the tab active color to denote focus.
	//style.TabBarOverlineSize			= 1.0f;					// Thickness of tab-bar overline, which highlights the selected tab-bar.
	//style.TableAngledHeadersAngle		= 35.0f * (IM_PI / 180.0f);	// Angle of angled headers (supported values range from -50 degrees to +50 degrees).
	//style.TableAngledHeadersTextAlign	= ImVec2(0.5f,0.0f);	// Alignment of angled headers within the cell
	//style.TreeLinesFlags				= ImGuiTreeNodeFlags_DrawLinesNone;
	//style.TreeLinesSize					= 1.0f;					// Thickness of outlines when using ImGuiTreeNodeFlags_DrawLines.
	//style.TreeLinesRounding				= 0.0f;					// Radius of lines connecting child nodes to the vertical line.
	//style.ColorButtonPosition			= ImGuiDir_Right;		// Side of the color button in the ColorEdit4 widget (left/right). Defaults to ImGuiDir_Right.
	//style.ButtonTextAlign				= ImVec2(0.5f,0.5f);	// Alignment of button text when button is larger than text.
	//style.SelectableTextAlign			= ImVec2(0.0f,0.0f);	// Alignment of selectable text. Defaults to (0.0f, 0.0f) (top-left aligned). It's generally important to keep this left-aligned if you want to lay multiple items on a same line.
	style.SeparatorTextBorderSize		= 2.0f;					// Thickness of border in SeparatorText()
	//style.SeparatorTextAlign			= ImVec2(0.0f,0.5f);	// Alignment of text within the separator. Defaults to (0.0f, 0.5f) (left aligned, center).
	//style.SeparatorTextPadding			= ImVec2(20.0f,3.f);	// Horizontal offset of text from each edge of the separator + spacing on other axis. Generally small values. .y is recommended to be == FramePadding.y.
	//style.DisplayWindowPadding			= ImVec2(19,19);		// Window position are clamped to be visible within the display area or monitors by at least this amount. Only applies to regular windows.
	style.DisplaySafeAreaPadding		= ImVec2(0,0);		// If you cannot see the edge of your screen (e.g. on a TV) increase the safe area padding. Covers popups/tooltips as well regular windows.
	style.DockingSeparatorSize			= 3.0f;					// Thickness of resizing border between docked windows
	//style.MouseCursorScale				= 1.0f;					// Scale software rendered mouse cursor (when io.MouseDrawCursor is enabled). May be removed later.
	//style.AntiAliasedLines				= true;					// Enable anti-aliased lines/borders. Disable if you are really tight on CPU/GPU.
	//style.AntiAliasedLinesUseTex		= true;					// Enable anti-aliased lines/borders using textures where possible. Require backend to render with bilinear filtering (NOT point/nearest filtering).
	//style.AntiAliasedFill				= true;					// Enable anti-aliased filled shapes (rounded rectangles, circles, etc.).
	//style.CurveTessellationTol			= 1.25f;				// Tessellation tolerance when using PathBezierCurveTo() without a specific number of segments. Decrease for highly tessellated curves (higher quality, more polygons), increase to reduce quality.
	//style.CircleTessellationMaxError	= 0.30f;				// Maximum error (in pixels) allowed when using AddCircle()/AddCircleFilled() or drawing rounded corner rectangles with no explicit segment count specified. Decrease for higher quality but more geometry.
}

void InterfaceState::StyleColorsAsterDark(ImGuiStyle *dst)
{
	ImGuiStyle* style = dst ? dst : &ImGui::GetStyle();
	ImVec4* colors = style->Colors;

	// Interesting colors
	#define DARK_GREEN_RGB		0.26f, 0.59f, 0.98f

	#define LOG_COL(ID) LOG(LGUI, LInfo, "colors["#ID"] = = ImVec4(%ff, %ff, %ff, %ff)", colors[ID].x, colors[ID].y, colors[ID].z, colors[ID].w)
	#define ACCENT_1_RGB		0.26f, 0.59f, 0.98f
	#define ACCENT_2_RGB		0.11f, 0.64f, 0.92f

	colors[ImGuiCol_Text]					= ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	colors[ImGuiCol_TextDisabled]			= ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
	colors[ImGuiCol_WindowBg]				= ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
	colors[ImGuiCol_ChildBg]				= colors[ImGuiCol_WindowBg];
	colors[ImGuiCol_PopupBg]				= colors[ImGuiCol_WindowBg];
	colors[ImGuiCol_Border]					= ImVec4(0.25f, 0.25f, 0.25f, 0.50f);
	colors[ImGuiCol_BorderShadow]			= ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_FrameBg]				= ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
	colors[ImGuiCol_FrameBgHovered]			= ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
	colors[ImGuiCol_FrameBgActive]			= ImVec4(0.67f, 0.67f, 0.67f, 0.39f);
	colors[ImGuiCol_TitleBg]				= ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
	colors[ImGuiCol_TitleBgActive]			= colors[ImGuiCol_TitleBg];
	colors[ImGuiCol_TitleBgCollapsed]		= ImVec4(0.00f, 0.00f, 0.00f, 0.51f);
	colors[ImGuiCol_MenuBarBg]				= ImVec4(0.027f, 0.027f, 0.027f, 1.0f);
	colors[ImGuiCol_ScrollbarBg]			= ImVec4(0.02f, 0.02f, 0.02f, 0.53f);
	colors[ImGuiCol_ScrollbarGrab]			= ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
	colors[ImGuiCol_ScrollbarGrabHovered]	= ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
	colors[ImGuiCol_ScrollbarGrabActive]	= ImVec4(0.51f, 0.51f, 0.51f, 1.00f);
	colors[ImGuiCol_CheckMark]				= ImVec4(ACCENT_2_RGB, 1.00f);
	colors[ImGuiCol_SliderGrab]				= ImVec4(ACCENT_2_RGB, 1.00f);
	colors[ImGuiCol_SliderGrabActive]		= ImVec4(0.08f, 0.50f, 0.72f, 1.00f);
	colors[ImGuiCol_Button]					= colors[ImGuiCol_FrameBg];
	colors[ImGuiCol_ButtonHovered]			= colors[ImGuiCol_FrameBgHovered];
	colors[ImGuiCol_ButtonActive]			= colors[ImGuiCol_FrameBgActive];
	colors[ImGuiCol_Header]					= ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
	colors[ImGuiCol_HeaderHovered]			= ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
	colors[ImGuiCol_HeaderActive]			= ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
	colors[ImGuiCol_Separator]				= colors[ImGuiCol_Border];
	colors[ImGuiCol_SeparatorHovered]		= ImVec4(0.41f, 0.42f, 0.44f, 1.00f);
	colors[ImGuiCol_SeparatorActive]		= ImVec4(ACCENT_1_RGB, 0.95f);
	colors[ImGuiCol_ResizeGrip]				= ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_ResizeGripHovered]		= ImVec4(0.29f, 0.30f, 0.31f, 0.67f);
	colors[ImGuiCol_ResizeGripActive]		= ImVec4(ACCENT_1_RGB, 0.95f);
	colors[ImGuiCol_InputTextCursor]		= colors[ImGuiCol_Text];
	colors[ImGuiCol_TabHovered]				= ImVec4(0.33f, 0.34f, 0.36f, 0.83f);
	colors[ImGuiCol_Tab]					= ImVec4(0.08f, 0.08f, 0.09f, 0.83f);
	colors[ImGuiCol_TabSelected]			= ImVec4(0.23f, 0.23f, 0.24f, 1.00f);
	colors[ImGuiCol_TabSelectedOverline]	= ImVec4(ACCENT_1_RGB, 1.00f);
	colors[ImGuiCol_TabDimmed]				= ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
	colors[ImGuiCol_TabDimmedSelected]		= ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
	colors[ImGuiCol_TabDimmedSelectedOverline] = ImVec4(0.50f, 0.50f, 0.50f, 0.00f);
	colors[ImGuiCol_DockingPreview]			= ImVec4(ACCENT_1_RGB, 0.70f);
	colors[ImGuiCol_DockingEmptyBg]			= ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
	colors[ImGuiCol_PlotLines]				= ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
	colors[ImGuiCol_PlotLinesHovered]		= ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
	colors[ImGuiCol_PlotHistogram]			= ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
	colors[ImGuiCol_PlotHistogramHovered]	= ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
	colors[ImGuiCol_TableHeaderBg]			= ImVec4(0.19f, 0.19f, 0.20f, 1.00f);
	colors[ImGuiCol_TableBorderStrong]		= ImVec4(0.31f, 0.31f, 0.35f, 1.00f);	// Prefer using Alpha=1.0 here
	colors[ImGuiCol_TableBorderLight]		= ImVec4(0.23f, 0.23f, 0.25f, 1.00f);	// Prefer using Alpha=1.0 here
	colors[ImGuiCol_TableRowBg]				= ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	colors[ImGuiCol_TableRowBgAlt]			= ImVec4(1.00f, 1.00f, 1.00f, 0.06f);
	colors[ImGuiCol_TextLink]				= ImVec4(ACCENT_1_RGB, 1.00f);
	colors[ImGuiCol_TextSelectedBg]			= ImVec4(ACCENT_1_RGB, 0.35f);
	colors[ImGuiCol_TreeLines]				= ImVec4(0.43f, 0.43f, 0.50f, 0.50f);
	colors[ImGuiCol_DragDropTarget]			= ImVec4(ACCENT_2_RGB, 1.00f);
	colors[ImGuiCol_NavCursor]				= ImVec4(ACCENT_1_RGB, 1.00f);
	colors[ImGuiCol_NavWindowingHighlight]	= ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
	colors[ImGuiCol_NavWindowingDimBg]		= ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
	colors[ImGuiCol_ModalWindowDimBg]		= ImVec4(0.80f, 0.80f, 0.80f, 0.35f);
}

void InterfaceState::StyleSelectFont(std::string name)
{
	if (name.empty())
		selectedFont = "Karla";
	else selectedFont = name;
}

static void updateFont()
{
	if (loadedFont == selectedFont) return;
	loadedFont = selectedFont;
	ImFontConfig config;
	config.OversampleH = 2;
	config.OversampleV = 2;
	std::string latinFont = "resources/fonts/" + selectedFont + "-Regular-Latin.ttf";
	std::string iconFont = "resources/fonts/LineAwesome-Regular-Icon.ttf";
	if (std::filesystem::exists(latinFont))
	{
		auto &io = ImGui::GetIO(); 
		io.Fonts->Clear();
		io.Fonts->AddFontFromFileTTF(latinFont.c_str(), 17, &config);
		io.Fonts->AddFontFromFileTTF(iconFont.c_str(), 17, &config);
	}
	else if (std::filesystem::exists("../" + latinFont))
	{
		printf("'%s' not found in working directory but in parent directory! Make sure to run AsterTrack in the program root directory!\n", latinFont.c_str());
		LOG(LDefault, LError, "'%s' not found in working directory but in parent directory! Make sure to run AsterTrack in the program root directory!", latinFont.c_str());
	}
	else
	{
		printf("'%s' not found in working directory! Make sure to run AsterTrack in the program root directory!\n", latinFont.c_str());
		LOG(LDefault, LError, "'%s' not found in working directory! Make sure to run AsterTrack in the program root directory!", latinFont.c_str());
	}
}

/* ImGui Settings for UI-related persistency */

static void readSettingsFile(ImGuiContext *context, ImGuiSettingsHandler *settings)
{
	GetUI().loadedSettingsIni = true;
}

static void* readCustomSettingsHeader(ImGuiContext *context, ImGuiSettingsHandler *settings, const char *header)
{
	int cmp = strncmp("UIState", header, 8); // With \0
	if (cmp != 0)
		return (void*)(intptr_t)0;
	return (void*)(intptr_t)1;
}

static void readCustomSettingsLine(ImGuiContext *context, ImGuiSettingsHandler *settings, void *entry, const char *line)
{
	if ((intptr_t)entry == 0)
		return; // Invalid header name
	int d1, d2;
	if (sscanf(line, "L%d=%d", &d1, &d2) == 2)
	{
		if (d1 >= 0 && d1 < LMaxCategory && d2 >= 0 && d2 < LMaxLevel)
		{
			LogFilterTable[d1] = (LogLevel)d2;
			LogMaxLevelTable[d1] = std::min(LDebug, LogFilterTable[d1]);
		}
	}
}

static void writeCustomSettings(ImGuiContext *context, ImGuiSettingsHandler *settings, ImGuiTextBuffer *output)
{
	output->reserve(output->size() + LMaxCategory * 7); // LXX=XX\n
	output->appendf("[%s][%s]\n", settings->TypeName, "UIState");
	for (int i = 0; i < LMaxCategory; i++)
	{
		output->appendf("L%d=%d\n", i, (int)LogFilterTable[i]);
	}
}
