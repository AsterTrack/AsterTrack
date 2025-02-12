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

const long targetIntervalUS = 1000000/60; // glfwSwapBuffer will further limit it to display refresh rate


/* Function prototypes */

// GLFW Platform Window
static GLFWwindow* setupPlatformWindow(bool &useHeader);
static void closePlatformWindow(GLFWwindow *windowHandle);
static void checkDPIUpdate(GLFWwindow *glfwWindow);
static void RefreshGLFWWindow(GLFWwindow *window);
// ImGui Code
static bool setupImGuiTheme();
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
	ui.UpdateCalibrations(false);
	ui.UpdateSequences(true);

	// Render loop, starting off with 3 full UI update iterations
	ui.requireUpdates = 3;
	while (!glfwWindowShouldClose(ui.glfwWindow) && !ui.setCloseInterface)
	{
		// Record render time
		auto now = sclock::now();
		ui.deltaTime = dtMS(ui.renderTime, now)/1000.0f;
		ui.renderTime = now;

		// Update/render UI as requested
		if (ui.requireUpdates)
		{ // ImGui UI needs updates
			ui.requireUpdates--;
			ui.requireRender = false;
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
		if (curIntervalUS < targetIntervalUS)
			std::this_thread::sleep_for(std::chrono::microseconds(targetIntervalUS-curIntervalUS));

		// Wait for input events or update/render requests while updating general state
		glfwPollEvents();
		ui.GeneralUpdate();
		while (!ui.requireRender && ui.requireUpdates == 0
			&& ImGui::GetCurrentContext()->InputEventsQueue.empty())
		{ // This timeout greatly influences idle power consumption
			glfwWaitEventsTimeout(50.0f/1000.0f);
			ui.GeneralUpdate();
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
	if (dtUS(ui.renderTime, now) < targetIntervalUS) return;
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
	std::shared_lock dev_lock(GetState().deviceAccessMutex);

	checkDPIUpdate(glfwWindow);

	// Start new UI frame
	ImGui_ImplGlfw_NewFrame();
	ImGui_ImplOpenGL3_NewFrame();
	ImGui::NewFrame();

	OnDemandNewFrame();

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

	UpdateMainMenuBar();

	// Dockspace for other windows to dock to, covering whole viewport
	ImGui::DockSpaceOverViewport(dockspaceID, ImGui::GetMainViewport());

	for (auto &window : windows)
	{
		if (window.open)
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

	GeneralInput();

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
		(state.mode != MODE_Replay || state.frameRecordReplayPos < state.loadedFrameRecords.size()) &&
		(pipeline.phase == PHASE_Calibration_Point || pipeline.phase == PHASE_Calibration_Target) && pipeline.recordSequences)
	{
		static TimePoint_t lastObservationUpdate;
		if (dtMS(lastObservationUpdate, sclock::now()) > 100 || visState.incObsUpdate.dirty)
		{
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

	ImGui::DockBuilderDockWindow(windows[WIN_TARGETS].title.c_str(), auxPanelID);

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

	setupImGuiTheme();

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

	auto loadIcon = [](const char *path)
	{
		int x, y, n;
		uint8_t *imageData = stbi_load(path, &x, &y, &n, 4);

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

	lightModeIcons.wireless = loadIcon("resources/wireless.png");
	lightModeIcons.server = loadIcon("resources/server.png");
	lightModeIcons.frameWireless = loadIcon("resources/frame_wireless.png");
	lightModeIcons.frameHDMI = loadIcon("resources/frame_hdmi.png");
	lightModeIcons.visual = loadIcon("resources/visual.png");
	lightModeIcons.detach = loadIcon("resources/detach.png");
	lightModeIcons.context = loadIcon("resources/dots.png");
	lightModeIcons.controller = loadIcon("resources/controller.png");
	lightModeIcons.camera = loadIcon("resources/camera.png");
	darkModeIcons.wireless = loadIcon("resources/wireless_d.png");
	darkModeIcons.server = loadIcon("resources/server_d.png");
	darkModeIcons.frameWireless = loadIcon("resources/frame_wireless_d.png");
	darkModeIcons.frameHDMI = loadIcon("resources/frame_hdmi_d.png");
	darkModeIcons.visual = loadIcon("resources/visual_d.png");
	darkModeIcons.detach = loadIcon("resources/detach_d.png");
	darkModeIcons.context = loadIcon("resources/dots_d.png");
	darkModeIcons.controller = loadIcon("resources/controller_d.png");
	darkModeIcons.camera = loadIcon("resources/camera_d.png");
	

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
	windows[WIN_TARGETS] = InterfaceWindow("Targets", &InterfaceState::UpdateTargets, false);
	// Device mode
	windows[WIN_DEVICES] = InterfaceWindow("Devices", &InterfaceState::UpdateDevices, false);
	windows[WIN_CAMERA_SETTINGS] = InterfaceWindow("Camera Settings", &InterfaceState::UpdateCameraSettings, true);
	windows[WIN_WIRELESS] = InterfaceWindow("Wireless Setup", &InterfaceState::UpdateWirelessSetup, false);
	// Parameters
	windows[WIN_SEQUENCE_PARAMS] = InterfaceWindow("Sequence2D Params", &InterfaceState::UpdateSequenceParameters, false);
	windows[WIN_POINT_CALIB_PARAMS] = InterfaceWindow("PointCalib Params", &InterfaceState::UpdatePointCalibParameters, false);
	windows[WIN_TARGET_CALIB_PARAMS] = InterfaceWindow("TargetCalib Params", &InterfaceState::UpdateTargetCalibParameters, false);
	windows[WIN_TRACKING_PARAMS] = InterfaceWindow("Tracking Params", &InterfaceState::UpdateTrackingParameters, false);
	// Tools
	windows[WIN_LENS_SELECTION_TOOL] = InterfaceWindow("Lens Selection", &InterfaceState::UpdateLensSelectionTool, false);
	// Shortcut to ImGui's built-in style editor
	windows[WIN_STYLE_EDITOR] = InterfaceWindow("Style Editor", &InterfaceState::UpdateStyleUI);
	// Useful tool to debug and research UI
	windows[WIN_IMGUI_DEMO] = InterfaceWindow("Dear ImGui Demo", &InterfaceState::UpdateImGuiDemoUI);
	windows[WIN_IMPLOT_DEMO] = InterfaceWindow("ImPlot Demo", &InterfaceState::UpdateImPlotDemoUI);

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
	if (!(imguiWindow && imguiWindow->Active && imguiWindow->DockTabIsVisible))
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
			break;
		case EVT_MODE_SIMULATION_START:
			break;
		case EVT_MODE_SIMULATION_STOP:
			break;
		case EVT_START_STREAMING:
			break;
		case EVT_STOP_STREAMING:
			// TODO: Might need UI drawing lock, not in UI thread right now
			GetUI().visState.tracking.targets.clear();
			GetUI().recordSections.clear();
			GetUI().recordSectionStart = -1;
			break;
		case EVT_UPDATE_CAMERAS:
			GetUI().UpdateCameras();
			break;
		case EVT_UPDATE_OBSERVATIONS:
			GetUI().visState.incObsUpdate.dirty = true;
			GetUI().RequestUpdates();
			break;
		case EVT_UPDATE_CALIBS:
			GetUI().UpdateCalibrations();
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

static void checkDPIUpdate(GLFWwindow *glfwWindow)
{
	int winX, winY;
	int fbX, fbY;
	float csX, csY;
	glfwGetWindowSize(glfwWindow, &winX, &winY);
	glfwGetFramebufferSize(glfwWindow, &fbX, &fbY);
	glfwGetWindowContentScale(glfwWindow, &csX, &csY);

	static float renderPixelRatio;
	static float hidpiScale;
	float t_renderPixelRatio = std::max((float)fbX/winX, (float)fbY/winY);
	float t_hidpiScale = std::max(csX, csY);
	if (renderPixelRatio != t_renderPixelRatio || hidpiScale != t_hidpiScale)
	{
		renderPixelRatio = t_renderPixelRatio;
		hidpiScale = t_hidpiScale;

		LOG(LGUI, LDebug, "Window %dx%d, Framebuffer %dx%d, ContentScale %fx%f\n"
			"So pixel Ratio of %f, hidpi scale of %f, imgui reports dpi scale of %f",
			winX, winY, fbX, fbY, csX, csY,
			renderPixelRatio, hidpiScale, ImGui::GetWindowDpiScale());

		ImFontConfig config;
		config.OversampleH = 2;
		config.OversampleV = 2;
		config.GlyphExtraSpacing.x = 1.0f;
		ImGui::GetIO().Fonts->Clear();
		ImGui::GetIO().Fonts->AddFontFromFileTTF("resources/Karla-Regular.ttf", 17*renderPixelRatio, &config);
		ImGui::GetIO().FontGlobalScale = 1.0f / renderPixelRatio;

		ImGui_ImplOpenGL3_DestroyFontsTexture();
		ImGui_ImplOpenGL3_CreateFontsTexture();
	}
}


/**
 * ImGui code
 */

static bool setupImGuiTheme()
{
	// Setup Dear ImGui style
	ImGuiStyle& style = ImGui::GetStyle();

	style.WindowPadding = ImVec2(7, 7);				// Padding within a window.
	style.WindowRounding = 0.0f;					// Radius of window corners rounding. Set to 0.0f to have rectangular windows. Large values tend to lead to variety of artifacts and are not recommended.
	style.WindowBorderSize = 1.0f;					// Thickness of border around windows. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
	//style.WindowMinSize;					// Minimum window size. This is a global setting. If you want to constrain individual windows, use SetNextWindowSizeConstraints().
	//style.WindowTitleAlign;				// Alignment for title bar text. Defaults to (0.0f,0.5f) for left-aligned,vertically centered.
	style.WindowMenuButtonPosition = ImGuiDir_None;	// Side of the collapsing/docking button in the title bar (None/Left/Right). Defaults to ImGuiDir_Left.
	style.ChildRounding = 8.0f;						// Radius of child window corners rounding. Set to 0.0f to have rectangular windows.
	style.ChildBorderSize = 1.0f;					// Thickness of border around child windows. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
	style.PopupRounding = 0.0f;						// Radius of popup window corners rounding. (Note that tooltip windows use WindowRounding)
	style.PopupBorderSize = 1.0f;					// Thickness of border around popup/tooltip windows. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
	style.FramePadding = ImVec2(8,4);				// Padding within a framed rectangle (used by most widgets).
	style.FrameRounding = 2.0f;						// Radius of frame corners rounding. Set to 0.0f to have rectangular frame (used by most widgets).
	style.FrameBorderSize = 0.0f;					// Thickness of border around frames. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
	style.ItemSpacing = ImVec2(8,6);				// Horizontal and vertical spacing between widgets/lines.
	style.ItemInnerSpacing = ImVec2(6,6);			// Horizontal and vertical spacing between within elements of a composed widget (e.g. a slider and its label).
	style.CellPadding = ImVec2(4,4);				// Padding within a table cell. CellPadding.y may be altered between different rows.
	style.TouchExtraPadding = ImVec2(0, 0);			// Expand reactive bounding box for touch-based system where touch position is not accurate enough. Unfortunately we don't sort widgets so priority on overlap will always be given to the first widget. So don't grow this too much!
	style.IndentSpacing = 20.0f;					// Horizontal indentation when e.g. entering a tree node. Generally == (FontSize + FramePadding.x*2).
	style.ColumnsMinSpacing = 8;					// Minimum horizontal spacing between two columns. Preferably > (FramePadding.x + 1).
	style.ScrollbarSize = 20.0f;					// Width of the vertical scrollbar, Height of the horizontal scrollbar.
	style.ScrollbarRounding = 10.0f;				// Radius of grab corners for scrollbar.
	style.GrabMinSize = 8.0f;						// Minimum width/height of a grab box for slider/scrollbar.
	style.GrabRounding = 4.0f;						// Radius of grabs corners rounding. Set to 0.0f to have rectangular slider grabs.
	//style.LogSliderDeadzone;				// The size in pixels of the dead-zone around zero on logarithmic sliders that cross zero.
	style.TabRounding = 4.0f;						// Radius of upper corners of a tab. Set to 0.0f to have rectangular tabs.
	style.TabBorderSize = 0.0f;						// Thickness of border around tabs.
	style.TabMinWidthForCloseButton = 0.0f;			// Minimum width for close button to appear on an unselected tab when hovered. Set to 0.0f to always show when hovering, set to FLT_MAX to never show close button unless selected.
	style.TabBarBorderSize = 1.0f;					// Thickness of tab-bar separator, which takes on the tab active color to denote focus.
	//style.ColorButtonPosition;			// Side of the color button in the ColorEdit4 widget (left/right). Defaults to ImGuiDir_Right.
	//style.ButtonTextAlign;				// Alignment of button text when button is larger than text. Defaults to (0.5f, 0.5f) (centered).
	//style.SelectableTextAlign;			// Alignment of selectable text. Defaults to (0.0f, 0.0f) (top-left aligned). It's generally important to keep this left-aligned if you want to lay multiple items on a same line.
	style.SeparatorTextBorderSize = 2.0f;			// Thickkness of border in SeparatorText()
	//style.SeparatorTextAlign;				// Alignment of text within the separator. Defaults to (0.0f, 0.5f) (left aligned, center).
	//style.SeparatorTextPadding;			// Horizontal offset of text from each edge of the separator + spacing on other axis. Generally small values. .y is recommended to be == FramePadding.y.
	//style.DisplayWindowPadding;			// Window position are clamped to be visible within the display area or monitors by at least this amount. Only applies to regular windows.
	style.DisplaySafeAreaPadding = ImVec2(0,0);		// If you cannot see the edges of your screen (e.g. on a TV) increase the safe area padding. Apply to popups/tooltips as well regular windows. NB: Prefer configuring your TV sets correctly!
	style.DockingSeparatorSize = 3.0f;				// Thickness of resizing border between docked windows
	//style.MouseCursorScale;				// Scale software rendered mouse cursor (when io.MouseDrawCursor is enabled). We apply per-monitor DPI scaling over this scale. May be removed later.
	style.AntiAliasedLines = true;					// Enable anti-aliased lines/borders. Disable if you are really tight on CPU/GPU. Latched at the beginning of the frame (copied to ImDrawList).
	style.AntiAliasedLinesUseTex = true;			// Enable anti-aliased lines/borders using textures where possible. Require backend to render with bilinear filtering (NOT point/nearest filtering). Latched at the beginning of the frame (copied to ImDrawList).
	style.AntiAliasedFill = true;					// Enable anti-aliased edges around filled shapes (rounded rectangles, circles, etc.). Disable if you are really tight on CPU/GPU. Latched at the beginning of the frame (copied to ImDrawList).
	//style.CurveTessellationTol;			// Tessellation tolerance when using PathBezierCurveTo() without a specific number of segments. Decrease for highly tessellated curves (higher quality, more polygons), increase to reduce quality.
	//style.CircleTessellationMaxError;		// Maximum error (in pixels) allowed when using AddCircle()/AddCircleFilled() or drawing rounded corner rectangles with no explicit segment count specified. Decrease for higher quality but more geometry.


	// Interesting colors
	//(11,36,36,255)   (0.043f, 0.141f, 0.141f, 1.000f)

	// Default dark colors
	style.Colors[ImGuiCol_Text]						= ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	style.Colors[ImGuiCol_TextDisabled]				= ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_WindowBg]					= ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_ChildBg]					= ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_PopupBg]					= ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_Border]					= ImVec4(0.25f, 0.25f, 0.25f, 0.50f);
	style.Colors[ImGuiCol_BorderShadow]				= ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	style.Colors[ImGuiCol_FrameBg]					= ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
	style.Colors[ImGuiCol_FrameBgHovered]			= ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
	style.Colors[ImGuiCol_FrameBgActive]			= ImVec4(0.67f, 0.67f, 0.67f, 0.39f);
	style.Colors[ImGuiCol_TitleBg]					= ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
	style.Colors[ImGuiCol_TitleBgActive]			= ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
	style.Colors[ImGuiCol_TitleBgCollapsed]			= ImVec4(0.00f, 0.00f, 0.00f, 0.51f);
	style.Colors[ImGuiCol_MenuBarBg]				= ImVec4(0.027f, 0.027f, 0.027f, 1.0f);
	style.Colors[ImGuiCol_ScrollbarBg]				= ImVec4(0.02f, 0.02f, 0.02f, 0.53f);
	style.Colors[ImGuiCol_ScrollbarGrab]			= ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabHovered]		= ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
	style.Colors[ImGuiCol_ScrollbarGrabActive]		= ImVec4(0.51f, 0.51f, 0.51f, 1.00f);
	style.Colors[ImGuiCol_CheckMark]				= ImVec4(0.11f, 0.64f, 0.92f, 1.00f);
	style.Colors[ImGuiCol_SliderGrab]				= ImVec4(0.11f, 0.64f, 0.92f, 1.00f);
	style.Colors[ImGuiCol_SliderGrabActive]			= ImVec4(0.08f, 0.50f, 0.72f, 1.00f);
	style.Colors[ImGuiCol_Button]					= ImVec4(0.25f, 0.25f, 0.25f, 1.00f);
	style.Colors[ImGuiCol_ButtonHovered]			= ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
	style.Colors[ImGuiCol_ButtonActive]				= ImVec4(0.67f, 0.67f, 0.67f, 0.39f);
	style.Colors[ImGuiCol_Header]					= ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
	style.Colors[ImGuiCol_HeaderHovered]			= ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_HeaderActive]				= ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
	style.Colors[ImGuiCol_Separator]				= style.Colors[ImGuiCol_Border];
	style.Colors[ImGuiCol_SeparatorHovered]			= ImVec4(0.41f, 0.42f, 0.44f, 1.00f);
	style.Colors[ImGuiCol_SeparatorActive]			= ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
	style.Colors[ImGuiCol_ResizeGrip]				= ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	style.Colors[ImGuiCol_ResizeGripHovered]		= ImVec4(0.29f, 0.30f, 0.31f, 0.67f);
	style.Colors[ImGuiCol_ResizeGripActive]			= ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
	style.Colors[ImGuiCol_Tab]						= ImVec4(0.08f, 0.08f, 0.09f, 0.83f);
	style.Colors[ImGuiCol_TabHovered]				= ImVec4(0.33f, 0.34f, 0.36f, 0.83f);
	style.Colors[ImGuiCol_TabSelected]				= ImVec4(0.23f, 0.23f, 0.24f, 1.00f);
	style.Colors[ImGuiCol_TabDimmed]				= ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
	style.Colors[ImGuiCol_TabDimmedSelected]		= ImVec4(0.13f, 0.14f, 0.15f, 1.00f);
	style.Colors[ImGuiCol_DockingPreview]			= ImVec4(0.26f, 0.59f, 0.98f, 0.70f);
	style.Colors[ImGuiCol_DockingEmptyBg]			= ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
	style.Colors[ImGuiCol_PlotLines]				= ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
	style.Colors[ImGuiCol_PlotLinesHovered]			= ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
	style.Colors[ImGuiCol_PlotHistogram]			= ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
	style.Colors[ImGuiCol_PlotHistogramHovered]		= ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
	style.Colors[ImGuiCol_TextSelectedBg]			= ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
	style.Colors[ImGuiCol_DragDropTarget]			= ImVec4(0.11f, 0.64f, 0.92f, 1.00f);
	style.Colors[ImGuiCol_NavCursor]				= ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
	style.Colors[ImGuiCol_NavWindowingHighlight]	= ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
	style.Colors[ImGuiCol_NavWindowingDimBg]		= ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
	style.Colors[ImGuiCol_ModalWindowDimBg]			= ImVec4(0.80f, 0.80f, 0.80f, 0.35f);

	//ImGui::StyleColorsLight();

	return true;
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
