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

#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#endif

#include "ui.hpp"
#include "pipeline/pipeline.hpp"

#include "gl/visualisation.hpp"
#include "system/vis.hpp"
#include "imgui/imgui_onDemand.hpp"

#include "dbscan/dbscan.hpp" // For focusing on clusters
#include "emulation/integration.hpp" // Emulating TrackingCamera BlobDetection on host for visualisation
#include "comm/usb.hpp" // "Start Streaming" button
#include "device/parsing.hpp" // decompressCameraImageRecord
#include "device/tracking_camera.hpp"
#include "device/tracking_controller.hpp"
#include "calib/opt/covariance.hpp"

#include "util/eigenalg.hpp"
#include "util/debugging.hpp" // For checking debugging state

struct wl_display;
struct wl_resource;
#include "GL/glew.h"


static void visualiseCamera(const ServerState &state, VisualisationState &visState, const TrackingCameraState &camera, CameraVisState &visCamera, Eigen::Vector2i viewSize);

static bool updateAutoZoom(const VisualisationState &visState, const TrackingCameraState &camera, CameraVisState &visCamera, const CameraFrameRecord &frame, const VisFrameLock &visFrame);
static void assureValidView(CameraVisState &visCamera, const CameraMode &mode);
static Eigen::Isometry3f generateProjectionMatrix(CameraVisState &visCamera, const CameraMode &mode);
static float updateUndistortedBorders(CameraVisState &visCamera, const CameraCalib &calibHost, const CameraCalib &calib, const CameraMode &mode,
	std::vector<VisPoint> &fullBorder, std::vector<VisPoint> &usedBorder);
static void calculateCameraDistortionMap(const TrackingCameraState &camera, CameraVisState &visCamera);

static bool updateAdaptiveImageStreaming(Bounds2i &bounds, const CameraVisState &visCamera, const CameraPipeline &camera);

static bool drawOverlayMessage(ImRect rect, int index, int count, ImGuiID id, bool button, const std::string &message);

static CameraVisMode getVisMode(CameraView &view)
{
	if (!view.vis.errors.hasMap)
		view.vis.errors.show = false;
	if (view.vis.errors.show)
		return VIS_ERROR_VIS;
	if (!view.vis.imageVis.show)
		view.vis.emulation.enabled = false;
	if (view.vis.emulation.enabled)
		return VIS_EMULATION;
	if (view.camera->hasSetMode(TRCAM_MODE_BGCALIB))
		return VIS_BACKGROUND_TILES;
	if (view.camera->hasSetMode(TRCAM_MODE_VISUAL))
		return VIS_VISUAL_DEBUG;
	return VIS_BLOB;
}

void InterfaceState::ShowConnectingCameraUI(int index)
{
	// TODO: Rework connecting cameras (2/2)
	// This is only for cameras that we KNOW are there, but we don't know their persistent ID yet
	// Currently, all connecting cameras with unknown IDs (e.g. 0) would appear as one and the same, and we'd get a garbage camera in pipeline
	// So until we make the MCU (which communicates while Pi is starting) aware of the ID of the Pi, do this temporarily
	// Not super easy to both make MCU ID-aware, make 100% sure Pi that is booting will use the same ID, and keep backwards compatibility to cameras without MCU

	ImRect rect = ImGui::GetCurrentWindowRead()->InnerRect;
	// Black background
	ImGui::RenderFrame(rect.Min, rect.Max, IM_COL32(0,0,0,255), false, 0);
	// Single message
	drawOverlayMessage(rect, 0, 1, ImGui::GetID("Connecting"), false, "Camera is connected and starting up...");
}

void InterfaceState::UpdateCameraUI(CameraView &view)
{
	auto posToUI = [](Eigen::Vector2f pos)
	{
		auto window = ImGui::GetCurrentWindowRead();
		ImVec2 rel((pos.x()/2+0.5f), (pos.y()/2+0.5f));
		return rel*window->InnerRect.GetSize() + window->InnerRect.Min - window->Pos;
	};

	ImRect rect = ImGui::GetCurrentWindowRead()->InnerRect;
	AddOnDemandRender(rect, [](const ImDrawList* dl, const ImDrawCmd* dc)
	{
		OnDemandItem &render = *static_cast<OnDemandItem*>(dc->UserCallbackData);
		CameraID id = (CameraID)(intptr_t)render.userData;

		// Get camera from id (to make sure it's still valid)
		auto viewIt = GetUI().cameraViews.find(id);
		if (viewIt == GetUI().cameraViews.end())
			return; // Just removed, but UI hasn't been updated yet

		ImVec2 size = SetOnDemandRenderArea(render, dc->ClipRect);
		glClearColor(0.0, 0.0, 0.0, 0.0);
		glClear(GL_COLOR_BUFFER_BIT);

		// Update and render visualisations
		CameraView &view = viewIt->second;
		visualiseCamera(GetState(), GetUI().visState,
			*view.camera, view.vis, Eigen::Vector2i(size.x, size.y));
	}, (void*)(intptr_t)view.camera->id);

	float flip = view.vis.view.rotate180? -1 : +1;

	bool viewHovered, viewHeld;
	bool viewPressed = InteractionSurface("CameraView", rect, viewHovered, viewHeld);
	bool viewFocused = ImGui::IsItemFocused();

	ServerState &state = GetState();

	bool device = state.mode == MODE_Device;
	bool bgCalib = device && view.camera->hasSetMode(TRCAM_MODE_BGCALIB);
	bool visDebug = device && view.camera->hasSetMode(TRCAM_MODE_VISUAL);
	bool updateImageStreaming = false;

	/**
	 * Camera View Mouse Interaction
	 */

	auto &io = ImGui::GetIO();

	{ // Zoom controls

		auto &viewCtrl = view.vis.view;

		Eigen::Vector2f viewFactor(
			+2.0f*view.camera->pipeline->mode.sizeW * flip,
			-2.0f*view.camera->pipeline->mode.sizeH * flip);

		if (viewPressed && io.KeyCtrl)
			viewCtrl.isDragging = true;
		if (viewCtrl.isDragging && (!viewHeld || !io.KeyCtrl))
			viewCtrl.isDragging = false;

		if (viewHovered && io.KeyCtrl && std::abs(io.MouseWheel) > 0.0001f)
		{
			// Determine offset from center to temporary zoom target (mouse pos)
			ImVec2 offset = (io.MousePos - rect.GetCenter()) / rect.GetSize();
			Eigen::Vector2f correction(offset.x * viewFactor.x(), offset.y * viewFactor.y());
			// Correct zooming by keeping mouse at the same position
			float oldZoomFactor = viewCtrl.zoom;
			viewCtrl.zoom = std::max(viewCtrl.maxZoom, viewCtrl.zoom*(1.0f+io.MouseWheel*0.1f) + io.MouseWheel*0.2f);
			if ((oldZoomFactor > 1.0f && viewCtrl.zoom < 1.0f) ||
				(oldZoomFactor < 1.0f && viewCtrl.zoom > oldZoomFactor))
			{ // Constrain to default scale when crossing it
				viewCtrl.zoom = 1.0f;
				viewCtrl.center.setZero();
			}
			else
			{
				viewCtrl.center -= correction * (1.0f/viewCtrl.zoom - 1.0f/oldZoomFactor);
				assureValidView(view.vis, view.camera->pipeline->mode);
			}
		}
	
		if (viewCtrl.isDragging && (io.MouseDelta.x != 0 || io.MouseDelta.y != 0))
		{ // Drag zoomed-in view
			ImVec2 offset = io.MouseDelta / rect.GetSize();
			viewCtrl.center.x() -= offset.x * viewFactor.x() / viewCtrl.zoom;
			viewCtrl.center.y() -= offset.y * viewFactor.y() / viewCtrl.zoom;
			assureValidView(view.vis, view.camera->pipeline->mode);
		}

		viewCtrl.prevCenter = viewCtrl.center;
		viewCtrl.prevZoom = viewCtrl.zoom;
	}

	{ // Shortcut controls for individual cameras
		bool takeKeyInput = viewFocused || viewHovered;
		if (takeKeyInput)
			ImGui::SetNextFrameWantCaptureKeyboard(true);
		if (takeKeyInput) // !io.WantCaptureKeyboard
		{ // Process input
			if (ImGui::IsKeyPressed(ImGuiKey_H))
			{
				if (state.mode == MODE_Replay)
				{
					view.vis.imageVis.show = !view.vis.imageVis.show;
				}
				else if (device && state.isStreaming)
				{
					view.camera->config.imageStreaming.enabled = !view.camera->config.imageStreaming.enabled;
					updateImageStreaming = true;
				}
				RequestUpdates();
			}
		}

	}

	/**
	 * Camera GL View Overlays
	 */

	auto displaySceneLabels = [&](std::vector<SceneLabel> &labels, bool alwaysShow = false)
	{
		auto cursorPos = ImGui::GetCursorPos();
		for (SceneLabel &label : labels)
		{
			Eigen::Vector2f center(
				(label.position.x() - view.vis.view.center.x()) * +flip,
				(label.position.y() - view.vis.view.center.y()) * -flip);
			Eigen::Vector2f outside = center + Eigen::Vector2f::Constant(M_SQRT1_2 * label.radius);
			center = (center * view.vis.view.zoom).cwiseProduct(view.camera->pipeline->mode.factor.cast<float>());
			outside = (outside * view.vis.view.zoom).cwiseProduct(view.camera->pipeline->mode.factor.cast<float>());

			ImGui::PushID(label.text.c_str());
			if (!alwaysShow)
			{
				float size = ImGui::GetFrameHeight()/2;
				ImGui::SetCursorPos(posToUI(center) - ImVec2(size/2, size/2));
				if (CircularButton("#Label", size, ImVec4(2*label.color.r, 2*label.color.g, 2*label.color.b, 2*label.color.a)))
					label.toggle = !label.toggle;
				if (!label.toggle && (ImGui::IsItemHovered() || ImGui::IsItemActive()))
				{
					ImGuiWindowFlags flags = ImGuiWindowFlags_Tooltip | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoDocking;
					ImGui::SetNextWindowPos(ImGui::GetWindowPos() + posToUI(outside));
					ImGui::Begin(label.text.c_str(), NULL, flags);
					ImGui::TextUnformatted(label.text.c_str());
					ImGui::End();
				}
			}
			if (alwaysShow || label.toggle)
			{
				ImGui::SetCursorPos(posToUI(outside));
				ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(label.color.r, label.color.g, label.color.b, label.color.a));
				ImGui::TextUnformatted(label.text.c_str());
				ImGui::PopStyleColor();
			}
			ImGui::PopID();
		}
		ImGui::SetCursorPos(cursorPos);
	};
	auto displaySceneButtons = [&](std::vector<SceneButton> &buttons)
	{
		auto cursorPos = ImGui::GetCursorPos();
		for (SceneButton &button : buttons)
		{
			Eigen::Vector2f center(
				(button.position.x() - view.vis.view.center.x()) * +flip,
				(button.position.y() - view.vis.view.center.y()) * -flip);
			center = (center * view.vis.view.zoom).cwiseProduct(view.camera->pipeline->mode.factor.cast<float>());

			ImGui::PushID((intptr_t)button.context);
			float size = ImGui::GetFrameHeight()/2;
			ImGui::SetCursorPos(posToUI(center) - ImVec2(size/2, size/2));
			if (CircularButton("#Btn", size, ImVec4(2*button.color.r, 2*button.color.g, 2*button.color.b, button.color.a)))
				button.callback(button.context);
			ImGui::PopID();
		}
		ImGui::SetCursorPos(cursorPos);
	};

	if (view.vis.visMode == VIS_EMULATION)
	{
		// Fetch new result and update cached visualisation resources
		auto result = std::move(view.vis.emulation.newResults);
		if (result)// && vis.image && result->frameID == vis.image->frameID)
		{
			view.vis.emulation.result = std::move(result);
			if (!view.vis.emulation.vis)
				view.vis.emulation.vis = std::make_shared<BlobEmulationVis>();
			updateEmulationVis(view.vis.emulation.vis, view.vis.emulation.result);
		}

		if (view.vis.emulation.result && view.vis.emulation.options.labels)
		{
			displaySceneLabels(view.vis.emulation.result->labels);
		}
	}
	if (visState.showMarkerTrails)
	{
		displaySceneLabels(view.vis.observations.labels);
	}
	if (state.pipeline.phase == PHASE_Tracking && visState.tracking.debug.frameNum > 0 && visState.tracking.debugMatchingState)
	{
		auto &trkVis = visState.tracking;
		int camera = view.camera->pipeline->index;
		if (trkVis.debugFocusStage > 0)
		{
			auto &matchingData = trkVis.debug.internalData.matching.at(camera);
			if (trkVis.debugFocusStage-1 < matchingData.numStages
				&& matchingData.stages[trkVis.debugFocusStage-1].identifier >= 0)
			{
				auto &targetMatch = matchingData.stages[trkVis.debugFocusStage-1];
				SceneLabel label;
				label.position.head<2>() = trkVis.debug.targetBounds[camera].center() - trkVis.debug.targetBounds[camera].extends();
				label.radius = 1.0f*PixelSize;
				label.text = targetMatch.label;
				label.color = Color{ 1.0f, 1.0f, 1.0f, 1 };
				std::vector<SceneLabel> labels = { label };
				displaySceneLabels(labels, true);
			}
		}

		if (trkVis.debug.priLabels.size() > camera)
			displaySceneLabels(trkVis.debug.priLabels[camera], false);
		if (trkVis.debug.secLabels.size() > camera)
			displaySceneLabels(trkVis.debug.secLabels[camera], false);
		if (trkVis.debug.editButtons.size() > camera)
			displaySceneButtons(trkVis.debug.editButtons[camera]);
	}

	/**
	 * Camera View Toolbars
	 */

	BeginViewToolbar();

	if (!view.isDetached)
	{ // Show detach button and title within view
		if (ImGui::ImageButton("DetachView", darkModeIcons.detach, iconSize()))
		{
			view.isDetached = true;
			cameraGridDirty = true;
		}
		ImGui::SameLine();
		ImGui::AlignTextToFramePadding();
		if (view.camera->pipeline)
			ImGui::Text("#%d (%d)", view.camera->id, view.camera->pipeline->index);
		else
			ImGui::Text("#%d", view.camera->id);
		ImGui::SameLine();
	}

	if (device)
	{ // Show device status icons

		// Show camera streaming reliability icon
		ImVec2 statusSize(ImGui::GetTextLineHeight(), ImGui::GetTextLineHeight());
		OnDemandItem *icon = AddOnDemandIcon("StreamingIcon", 
			iconSize()+ImGui::GetStyle().FramePadding*2, statusSize, 
			[](const ImDrawList* dl, const ImDrawCmd* dc)
		{
			OnDemandItem &render = *static_cast<OnDemandItem*>(dc->UserCallbackData);
			CameraID id = (CameraID)(intptr_t)render.userData;

			// Get camera from id (to make sure it's still valid)
			auto viewIt = GetUI().cameraViews.find(id);
			if (viewIt == GetUI().cameraViews.end())
				return; // Just removed, but UI hasn't been updated yet

			ImVec2 size = SetOnDemandRenderArea(render, dc->ClipRect);
			Color color = getStatusColor(*viewIt->second.camera);
			visSetupProjection(Eigen::Isometry3f::Identity());
			visualiseCircle<true>(Eigen::Vector2f::Zero(), 0.8f, color);
		}, (void*)(intptr_t)view.camera->id);
		if (icon)
			icon->renderOwn = false; // Part of full camera view render
		if (ImGui::BeginItemTooltip())
		{
			ImGui::TextUnformatted(getStatusText(*view.camera).c_str());
			ImGui::EndTooltip();
		}
		ImGui::SameLine();

		// Show wireless status icon
		auto &wireless = view.camera->config.wireless;
		if (wireless.wifiStatus == WIRELESS_STATUS_CONNECTED)
		{ // Have to manually add frame padding
			ImGui::SetCursorPosY(ImGui::GetCursorPosY()+ImGui::GetStyle().FramePadding.y);
			ImGui::Image(darkModeIcons.wireless, ImVec2(ImGui::GetFontSize()*6/5, ImGui::GetFontSize()));
			if (ImGui::BeginItemTooltip())
			{
				ImGui::Text("Camera is connected to wireless network '%s' with IP '%s'", wireless.SSID.c_str(), wireless.IP.c_str());
				if (wireless.serverStatus == WIRELESS_STATUS_ENABLED || wireless.serverStatus == WIRELESS_STATUS_CONNECTED)
				{
					if (view.camera->client && view.camera->client->ready)
						ImGui::Text("The server is running and connected.");
					else
						ImGui::Text("The server is running but not connected.");
				}
				ImGui::EndTooltip();
			}
			ImGui::SameLine(0.0f, ImGui::GetStyle().FramePadding.x + ImGui::GetStyle().ItemSpacing.x);
		}
	}

	// Right-aligned bar of 2 buttons
	ImGui::SetCursorPosX(GetRightAlignedCursorPos(GetBarWidth(GetIconWidth(iconSize()), 3)));

	if (ImGui::ImageButton("ImgStr", darkModeIcons.frame_wireless, iconSize()))
	{
		if (state.mode == MODE_Replay)
		{
			view.vis.imageVis.show = !view.vis.imageVis.show;
		}
		else if (device && state.isStreaming)
		{
			view.camera->config.imageStreaming.enabled = !view.camera->config.imageStreaming.enabled;
			updateImageStreaming = true;
		}
		RequestUpdates();
	}
	ImGui::SetItemTooltip("Toggle image streaming - Shortcut 'H'");
	ImGui::SameLine();

	// One Dropdown for modes the camera can be in:
	if (BeginIconDropdown("Mode", darkModeIcons.mode, iconSize(), ImGuiComboFlags_PopupAlignLeft))
	{
		// TODO: Allow these buttons to start streaming for individual cameras? Not sure
		// Would need to properly set up sync groups if not done already (also see "Start Streaming" button that has to do something similar)
		// Probably would want this to start the cameras as free-running if the rest of the system doesn't stream yet
		if (ImGui::MenuItem("Background Calibration", nullptr, 
			&bgCalib, device && state.isStreaming && !visDebug))
		{
			if (bgCalib)
				view.camera->updateBackgroundCalib(TrackingCameraState::BG_CALIB);
			else
				view.camera->updateBackgroundCalib(TrackingCameraState::BG_DISCARD);
			view.camera->receiving.background.contextualLock()->tiles.clear();
		}
		if (ImGui::MenuItem("Visual Debug", nullptr, 
			&visDebug, device && state.isStreaming && !bgCalib))
		{
			if (visDebug)
			{ // Enable visual debug
				view.camera->sendModeSet(TRCAM_FLAG_STREAMING | TRCAM_MODE_VISUAL);
			}
			else if (state.isStreaming)
			{ // Return to normal tracking
				view.camera->sendModeSet(TRCAM_FLAG_STREAMING | TRCAM_MODE_BLOB);
			}
			else
			{ // Return to standby
				view.camera->sendModeSet(TRCAM_STANDBY);
			}
		}
		if (ImGui::MenuItem("HDMI Visualisation", nullptr, 
			&view.camera->config.hdmiVis.enabled, device))
		{
			CameraUpdateVis(*view.camera);
		}

		ImGui::EndCombo();
	}
	ImGui::SameLine();

	// One Dropdown for all toggles for camera visualisation
	if (BeginIconDropdown("Toggle", darkModeIcons.vdots, iconSize(), ImGuiComboFlags_PopupAlignLeft))
	{
		ImGui::MenuItem("Rotate 180", nullptr, &view.vis.view.rotate180);

		ImGui::MenuItem("Show Error Map", nullptr, &view.vis.errors.show, view.vis.errors.hasMap);

		ImGui::Separator();

		if (state.mode == MODE_Replay)
		{
			ImGui::MenuItem("Show Camera Images", nullptr, &view.vis.imageVis.show);
		}
		else
		{
			updateImageStreaming |= ImGui::MenuItem("Stream Camera Images", nullptr,
				&view.camera->config.imageStreaming.enabled, device && state.isStreaming);
		}

		if (ImGui::MenuItem("Emulate Blob Detection", nullptr, &view.vis.emulation.enabled, view.vis.imageVis.show))
		{
			if (view.vis.emulation.enabled)
				view.vis.imageVis.undistort = false;
		}

		ImGui::MenuItem("Show Marker Trails", nullptr, &visState.showMarkerTrails);

		ImGui::EndCombo();
	}

	EndViewToolbar();

	if (view.camera->config.imageStreaming.enabled && device)
	{ // Show controls for camera image during device mode
		auto &config = getCameraConfig(*view.camera);
		auto &stream = view.camera->config.imageStreaming;
		auto &request = stream.request;
		auto &vis = view.vis.imageVis;
		auto &mode = view.camera->pipeline->mode;
		vis.show = true;

		BeginViewToolbar();
		ImGui::PushID("IMG");

		ImGui::Checkbox("Undistort", &vis.undistort);
		ImGui::SameLine();
		ImGui::Checkbox("Blob", &vis.showBlobs);
		ImGui::SameLine();
		ImGui::Checkbox("Sync", &vis.syncVis);
		ImGui::SameLine();
		updateImageStreaming |= ImGui::Checkbox("View", &stream.adaptive);
		ImGui::SameLine();
		updateImageStreaming |= ImGui::Checkbox("Quality", &stream.focusQuality);

		if (stream.adaptive)
		{ // Adapt to zoom
			if (updateAdaptiveImageStreaming(request.bounds, view.vis, *view.camera->pipeline))
			{
				updateImageStreaming = true;
			}
		}
		else
		{
			Bounds2i imageBounds = Bounds2i(0, 0, mode.widthPx, mode.heightPx);
			if (request.bounds != imageBounds)
			{
				request.bounds = imageBounds;
				updateImageStreaming = true;
			}
		}

		// Size the resolution level slider
		ImGui::SameLine();
		float labelWidth = ImGui::CalcTextSize("1XXXxXXX, XX (XXkb)").x;
		float sizeRight = labelWidth + ImGui::GetFrameHeight() + 2*ImGui::GetStyle().ItemSpacing.x; // GetIconWidth(iconSize())
		ImGui::SetNextItemWidth(LineWidthRemaining() - sizeRight);

		// Select quality level
		updateImageStreaming |= ImGui::SliderInt("##qualityLevel", 
			&stream.resolutionScale, 0, 5, "");

		// Select best frame streaming settings
		if (updateImageStreaming || view.resized)
		{ // Dynamically select frame streaming quality based on resolutionScale and requested bounds
			updateImageStreaming = true;

			// Pixel counts to consider
			unsigned int frameSize = mode.widthPx * mode.heightPx;
			unsigned int viewSize = rect.GetWidth() * rect.GetHeight(); // Consider scaling?
			unsigned int streamSize = request.bounds.extends().prod();

			// Select preference for pixel count (and profile) based on quality level
			unsigned int maxFrameSizes[] = {
				frameSize,
				frameSize/3,
				frameSize/12,
				std::min((unsigned int)960*600, viewSize/10*10),
				std::min((unsigned int)640*400, viewSize/10*6),
				std::min((unsigned int)480*300, viewSize/10*4)
			};
			unsigned int maxFrameSize = maxFrameSizes[stream.resolutionScale];

			// Reduce resolution to just above minimum of preference
			int subsampling = 1;
			while (streamSize/(subsampling*subsampling) > maxFrameSize)
				subsampling++;
			unsigned int encPixels = streamSize/(subsampling*subsampling);

			// Map desired pixel count to frame streaming settings absed on profile (focus)
			// Quality doesn't affect processing speed, just transfer speed (and thus, to a degree, minimum interval)
			// Pixel count determines processing cost for the camera
			struct {
				int maxPixelCount, quality, interval;
			} profiles[2][6] = {
				{ // Quality profile
					{ 1280*800, 90, 20 },
					{ 960*600, 90, 18 },
					{ 640*400, 94, 15 },
					{ 480*300, 96, 12 },
					{ 320*200, 99, 10 },
					{ 240*150, 100, 8 }
				},
				{ // Low Bandwidth profile, slightly higher framerate
					// Sadly it doesn't scale well with such high res, need to slow down anyway, so take good quality
					{ 1280*800, 70, 20 },
					{ 960*600, 60, 14 },
					{ 640*400, 70, 10 },
					{ 480*300, 82, 8 },
					{ 320*200, 90, 6 },
					{ 240*150, 95, 4 }
				}
			}; // Both profiles converge towards the end where restrictions are less severe

			// Select best settings from selected profile for this resolution
			auto &profile = profiles[stream.focusQuality? 0 : 1];
			int set;
			for (set = 5; set > 0; set--)
			{
				if (profile[set].maxPixelCount >= encPixels)
					break;
			}

			// Apply new settings if they changed
			auto &setting = profile[set];
			if (request.subsampling != subsampling
				|| request.jpegQuality != setting.quality
				|| request.frame != setting.interval)
			{
				updateImageStreaming = true;
				request.subsampling = subsampling;
				request.jpegQuality = setting.quality;
				request.frame = setting.interval;
			}
		}

		ImGui::SameLine();
		Eigen::Vector2i size = request.bounds.extends() / request.subsampling;
		if (view.vis.image)
			ImGui::Text("%dx%d, %d (%dkb)", size.x(), size.y(), request.jpegQuality, view.vis.image->jpegSize/1000);
		else
			ImGui::Text("%dx%d, %d", size.x(), size.y(), request.jpegQuality);

		SameLineTrailing(ImGui::GetFrameHeight());
		ImGui::SetNextItemAllowOverlap();
		if (CrossButton("Discard"))
		{
			view.camera->config.imageStreaming.enabled = false;
			updateImageStreaming = true;
		}

		ImGui::PopID();
		EndViewToolbar();
	}
	else if (state.mode == MODE_Replay && view.vis.imageVis.show)
	{ // Show controls for camera image during replay mode
		auto &vis = view.vis.imageVis;

		BeginViewToolbar();
		ImGui::PushID("IMG");
	
		ImGui::Checkbox("Undistort", &vis.undistort);
		ImGui::SameLine();
		ImGui::Checkbox("Blob", &vis.showBlobs);
		ImGui::SameLine();
		ImGui::Checkbox("Sync", &vis.syncVis);
		ImGui::SameLine();
		ImGui::Checkbox("Follow", &vis.followFrame);

		if (view.vis.image)
		{
			float labelWidth = ImGui::CalcTextSize("1XXXxXXX (XXkb)").x;
			float sizeRight = labelWidth + ImGui::GetFrameHeight() + ImGui::GetStyle().ItemSpacing.x;
			SameLineTrailing(sizeRight);
			ImGui::Text("%dx%d (%dkb)", view.vis.image->width, view.vis.image->height, view.vis.image->jpegSize/1000);
		}

		SameLineTrailing(ImGui::GetFrameHeight());
		if (CrossButton("Discard"))
		{
			vis.show = false;
		}

		ImGui::PopID();
		EndViewToolbar();
	}
	else
	{
		view.vis.imageVis.show = false;
	}

	if (bgCalib)
	{
		BeginViewToolbar();
		ImGui::PushID("BG");
		ImGui::TextUnformatted("Background Calibration");
		if (ImGui::BeginItemTooltip())
		{
			ImGui::TextUnformatted("Clear the field of view of any markers to only leave the light sources and reflective surfaces that should be ignored as background.\n"
				"Then wait until all background tiles are detected, before accepting and thus saving the background for this session.");
			ImGui::EndTooltip();
		}
		SameLineTrailing(GetBarWidth(ImGui::GetFrameHeight(), 3));
		if (RetryButton("Retry"))
			view.camera->updateBackgroundCalib(TrackingCameraState::BG_RESET);
		ImGui::SameLine();
		if (CheckButton("Accept"))
			view.camera->updateBackgroundCalib(TrackingCameraState::BG_ACCEPT);
		ImGui::SameLine();
		if (CrossButton("Discard"))
			view.camera->updateBackgroundCalib(TrackingCameraState::BG_DISCARD);

		ImGui::PopID();
		EndViewToolbar();
	}

	/**
	 * Misc UI controls
	 */

	if (view.vis.emulation.enabled)
	{ // Insert into new window shared between all cameras
		ImGui::Begin("Emulation Visualisation");
		BeginSection(asprintf_s("Camera %d", view.camera->id).c_str());
		updateEmulationVisUI(view.vis);
		EndSection();
		ImGui::End();
	}

	if (view.vis.view.zoom > view.vis.view.camZoom)
	{ // Zoom visualisation and control
		// Show frame for bounds of full view
		float boundsSizeW = std::min(rect.GetWidth()/4.0f, std::max(100.0f, rect.GetWidth()/10.0f));
		ImVec2 boundsMax = rect.Max - ImVec2(5, 5);
		ImVec2 boundsMin = boundsMax - ImVec2(boundsSizeW, boundsSizeW*view.size.y/view.size.x);
		Bounds2f bounds(boundsMin.x, boundsMin.y, boundsMax.x, boundsMax.y);
		ImGui::RenderFrame(boundsMin, boundsMax, IM_COL32(0x55, 0x55, 0x55, 0x33), false, 5);
		// Show frame for bounds of zoomed view
		Bounds2f viewBounds(
			bounds.center() + view.vis.view.center.cwiseProduct(Eigen::Vector2f(+flip, -flip)) / 2.0f * boundsSizeW * view.vis.view.camZoom,
			bounds.extends() * view.vis.view.camZoom / view.vis.view.zoom);
		uint8_t val = std::max(std::min((int)(0x33*std::pow(view.vis.view.zoom, 0.8f)), 0xFF), 0x33);
		ImGui::RenderFrame(ImVec2(viewBounds.min.x(), viewBounds.min.y()), ImVec2(viewBounds.max.x(), viewBounds.max.y()), IM_COL32(val, val, val, 0x33), false, 5);
		// Provide interaction to move the view
		bool boundsHovered, boundsHeld;
		if (InteractionSurface("ZoomView", ImRect(boundsMin, boundsMax), boundsHovered, boundsHeld) || boundsHeld)
		{
			auto &mode = view.camera->pipeline->mode;
			ImVec2 pos = (ImGui::GetMousePos() - boundsMin) / (boundsMax-boundsMin);
			view.vis.view.center = Eigen::Vector2f(+flip * (pos.x-0.5f) * 2.0f * mode.sizeW / view.vis.view.camZoom, -flip * (pos.y-0.5f) * 2.0f * mode.sizeH / view.vis.view.camZoom);
			assureValidView(view.vis, mode);
		}
	}

	
	// Get abnormal state to display
	bool abnormalStreamingState;
	std::vector<std::string> status = getAbnormalStatus(*view.camera, abnormalStreamingState);
	int msgCount = status.size() + abnormalStreamingState;
	if (msgCount)
	{
		//LOGC(LInfo, "Drawing %d msgs in window at %.2fx%.2f of size %.2fx%.2f", msgCount, ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, ImGui::GetWindowSize().x, ImGui::GetWindowSize().y);
		int msgIndex = 0;
		for (int i = 0; i < status.size(); i++)
		{
			drawOverlayMessage(rect, msgIndex++, msgCount, ImGui::GetID(i), false, status[i]);
		}
		if (abnormalStreamingState)
		{
			const std::string restartStreaming = "Start Streaming";
			if (drawOverlayMessage(rect, msgIndex++, msgCount, ImGui::GetID("streamBtn"), true, restartStreaming))
			{
				CameraRestartStreaming(GetState(), view.camera);
			}
		}
	}

	/**
	 * State Update
	 */

	// Update visualisation mode for GL rendering
	view.vis.visMode = getVisMode(view);

	if (updateImageStreaming)
	{ // Update settings if desired
		CameraUpdateStream(*view.camera);
	}

}

static void visualiseCamera(const ServerState &state, VisualisationState &visState, const TrackingCameraState &camera, CameraVisState &visCamera, Eigen::Vector2i viewSize)
{
	const PipelineState &pipeline = state.pipeline;

	/**
	 * Gather camera data
	 */
	CameraMode mode = camera.pipeline->mode;
	CameraCalib calib = camera.pipeline->calib;
	if (pipeline.isSimulationMode && GetUI().calibState.numUncalibrated > 0)
		calib = camera.pipeline->simulation.calib;
	float pixelRatio = ImGui::GetIO().DisplayFramebufferScale.y;
	float flip = visCamera.view.rotate180? -1 : +1;

	/**
	 * Update undistorted camera borders
	 */
	if (calib.valid())
	{
		if (!visCamera.calibration.precalculated)
		{
			calculateCameraDistortionMap(camera, visCamera);
		}
	}
	else
	{
		visCamera.calibration.precalculated = false;
		visCamera.view.camZoom = 1.0f;
	}
	visCamera.view.maxZoom = 0.99f*visCamera.view.camZoom;
	if (visState.calib.showFoVCircle)
		visCamera.view.maxZoom = std::min<float>(std::min(0.5f, visCamera.view.maxZoom), 0.99f*calib.fInv/std::tan(visState.calib.circularFoV / 180 * PI / 2));
	if (visState.calib.showFoVBounds)
		visCamera.view.maxZoom = std::min<float>(std::min(0.5f, visCamera.view.maxZoom), 0.99f*calib.fInv/std::tan(visState.calib.boundsFoV[2] / 180 * PI / 2));
	if (visState.calib.showDesignCalib)
	{
		for (auto &dCalib : visState.calib.designCalibs)
		{
			auto corner = undistortPoint(dCalib, Eigen::Vector2f(1, mode.aspect))*dCalib.fInv*calib.f;
			float maxDist = std::max(corner.x(), corner.y()/(float)mode.aspect);
			visCamera.view.maxZoom = std::min<float>(std::min(0.5f, visCamera.view.maxZoom), 0.99f/maxDist);
		}
	}

	auto drawCalibrationGuides = [&]()
	{
		if (visCamera.calibration.precalculated)
		{
			visualiseLines(visCamera.calibration.fullBorder, 2.0f);
			visualiseLines(visCamera.calibration.usedBorder, 2.0f);
		}
		if (visState.calib.showDesignCalib)
		{
			for (auto &dCalib : visState.calib.designCalibs)
			{
				thread_local std::vector<VisPoint> fullBorder, usedBorder;
				updateUndistortedBorders(visCamera, calib, dCalib, mode, fullBorder, usedBorder);
				visualiseLines(fullBorder, 1.0f);
				visualiseLines(usedBorder, 1.0f);
			}
		}

		if (visState.calib.showFoVCircle)
		{
			float fov = visState.calib.circularFoV / 180 * PI;
			float radius = calib.f * std::tan(fov/2);

			const int SEG = 100;
			thread_local std::vector<VisPoint> circle;
			circle.resize(SEG+1);
			Color8 col = Color{ 1.0f, 0.0f, 0.0f, 1.0f };
			for (int i = 0; i < circle.size(); i++)
			{
				float p = 2*(float)PI*(float)i/SEG;
				circle[i].pos.x() = std::cos(p)*radius;
				circle[i].pos.y() = std::sin(p)*radius;
				circle[i].pos.z() = 0.1f;
				circle[i].color = col;
			}
			visualiseLines(circle, 2.0f);
		}
		if (visState.calib.showFoVBounds)
		{
			float fovH = calib.f * std::tan(visState.calib.boundsFoV[0] / 180 * PI / 2);
			float fovV = calib.f * std::tan(visState.calib.boundsFoV[1] / 180 * PI / 2);
			float fovD = calib.f * std::tan(visState.calib.boundsFoV[2] / 180 * PI / 2);

			float len = 0.1f, d = (float)M_SQRT2;
			Color8 col = Color{ 1.0f, 1.0f, 0.0f, 1.0f };
			std::vector<std::pair<VisPoint,VisPoint>> lines = {
				{ { Eigen::Vector3f(-len, +fovV, 0.1f), col }, { Eigen::Vector3f(+len, +fovV, 0.1f), col } },
				{ { Eigen::Vector3f(-len, -fovV, 0.1f), col }, { Eigen::Vector3f(+len, -fovV, 0.1f), col } },
				{ { Eigen::Vector3f(-fovH, -len, 0.1f), col }, { Eigen::Vector3f(-fovH, +len, 0.1f), col } },
				{ { Eigen::Vector3f(+fovH, -len, 0.1f), col }, { Eigen::Vector3f(+fovH, +len, 0.1f), col } },
			};
			visualiseLines(lines, 2.0f);

			const int SEG = 100;
			thread_local std::vector<VisPoint> circle;
			circle.resize(SEG+1);
			for (int i = 0; i < circle.size(); i++)
			{
				float p = 2*(float)PI*(float)i/SEG;
				circle[i].pos.x() = std::cos(p)*fovD;
				circle[i].pos.y() = std::sin(p)*fovD;
				circle[i].pos.z() = 0.1f;
				circle[i].color = col;
			}
			visualiseLines(circle, 2.0f);
		}
	};
	auto drawWithoutFrame = [&]()
	{
		visSetupProjection(generateProjectionMatrix(visCamera, mode));
		drawCalibrationGuides();
	};

	/**
	 * Prepare frame to visualise
	 */
	auto visFrame = visState.lockVisFrame(pipeline, true, camera.pipeline->index);
	if (!visFrame.hasFrame || !visFrame.hasCamera)
	{
		drawWithoutFrame();
		return;
	}
	// Find accompanying image (and potentially change frame)
	std::shared_ptr<FrameRecord> imageFrameState = nullptr;
	if (visCamera.imageVis.show)
	{ // Find recent processed frame with image
		auto imageIt = visFrame.frameIt;
		bool foundImage = false;
		for (int i = 0; i < 1000; i++)
		{
			if (*imageIt)
			{ // Got a frame to check
				if (imageIt->get()->cameras.size() <= camera.pipeline->index)
					break; // No frame before this will have more cameras
				if (imageIt->get()->finishedProcessing && imageIt->get()->cameras[camera.pipeline->index].image)
				{ // Frame has an image
					imageFrameState = *imageIt; // new shared_ptr
					foundImage = true;
					break;
				}
			}
			if (imageIt == visFrame.frames.begin()) break;
			imageIt--;
		}

		if (visCamera.imageVis.syncVis)
		{ // Overwrite frame to visualise
			if (!foundImage)
			{
				drawWithoutFrame();
				return;
			}
			visFrame.frameIt = imageIt;

			if (visFrame.target.hasObs())
			{ // Make sure it is in the target view, else pick next
				if (visFrame.target.obs->frames[visFrame.target.frameIdx].frame != visFrame.frameIt.index())
				{ // Backtrack to frame with image (if possible, not when at beginning)
					while (visFrame.target.obs->frames[visFrame.target.frameIdx].frame > visFrame.frameIt.index() && visFrame.target.frameIdx > 0)
						visFrame.target.frameIdx--;
					if (visFrame.target.obs->frames[visFrame.target.frameIdx].frame < visFrame.frameIt.index() && visFrame.target.frameIdx < visFrame.target.obs->frames.size())
						visFrame.target.frameIdx++; // Pick next if there's no exact match
					visFrame.frameIt = visFrame.frames.pos(visFrame.target.obs->frames[visFrame.target.frameIdx].frame);
				}
			}
		}
	}
	// Gather further frame data
	assert(visFrame.hasFrame);
	assert(visFrame.frameIt.accessible() && *visFrame.frameIt);
	std::shared_ptr<const FrameRecord> frame = *visFrame.frameIt; // new shared_ptr
	assert(frame->finishedProcessing);
	auto &camFrame = frame->cameras[camera.pipeline->index];
	PipelinePhase phase = pipeline.phase.load();

	/**
	 * Update camera image
	 */
	if (visCamera.imageVis.show && imageFrameState && (!visCamera.image || imageFrameState->ID != visCamera.image->frameID))
	{ // Got a different camera image to load
		if (camera.receiving.latestFrameImage && imageFrameState->ID == camera.receiving.latestFrameImage->frameID)
		{ // Already cached recent frame image
			visCamera.image = camera.receiving.latestFrameImage; // new shared_ptr
		}
		else
		{ // Have to load image from JPEG record
			std::shared_ptr<CameraImageRecord> imageRec = imageFrameState->cameras[camera.pipeline->index].image; // new shared_ptr
			visCamera.image = decompressCameraImageRecord(imageRec);
		}
		loadGrayscaleFrame(visCamera.imageVis.texID, visCamera.image->image.data(), visCamera.image->width, visCamera.image->height);
		visCamera.emulation.update = true;
		LOG(LGUI, LTrace, "Loading image for visualisation with bounds (%d,%d)(%d,%d)",
			visCamera.image->boundsPx.min.x(), visCamera.image->boundsPx.min.y(), visCamera.image->boundsPx.max.x(), visCamera.image->boundsPx.max.y());
	}

	/**
	 * Update zoom target if necessary
	 */
	if (visCamera.view.autoZoom)
	{ // Control zoom target based on phase
		if (updateAutoZoom(visState, camera, visCamera, camFrame, visFrame))
		{ // Set new target, update UI
			float diffRel = (visCamera.view.prevCenter-visCamera.view.center).norm()*mode.widthPx/2.0f * visCamera.view.zoom;
			float zoomRel = visCamera.view.zoom / visCamera.view.prevZoom;
			if (diffRel > 50 || zoomRel > 1.1f || zoomRel < 0.9f)
				GetUI().RequestUpdates();
			// Will both update the zoom visualisation in the bottom right
			// And update streaming request to camera if adaptive image streaming is enabled
		}
	}

	/**
	 * Build projection matrix
	 */
	Eigen::Isometry3f postProjMat = generateProjectionMatrix(visCamera, mode);
	//setupCamera(postProjMat, calib, mode, viewSize);
	visSetupProjection(postProjMat);

	/**
	 * Visualise camera image in background
	 */
	if (visCamera.imageVis.show && visCamera.imageVis.texID > 0 && visCamera.image)
	{ // Display in appropriate part of the frame
		if (visCamera.imageVis.undistort && calib.valid())
		{ // Use special shader that undistorts and correctly positions image
			//showGrayscaleFrameUndistorted(vis.imageVis.texID, vis.image->boundsRel,
			//	mode, calib, vis.calibration.undistortMapScale,
			//	Color{1,1,1,0}, 1.0f, visState.image.brightness, visState.image.contrast);
			showGrayscaleFrameUndistorted(visCamera.imageVis.texID, visCamera.calibration.undistortionTexID,
				visCamera.image->boundsRel, mode, visCamera.calibration.undistortMapScale,
				Color{1,1,1,0}, 1.0f, visState.image.brightness, visState.image.contrast);
		}
		else
		{ // Display image in appropriate part of the frame
			Eigen::Affine3f bgProjMat = Eigen::Affine3f(
				// 3. Correct for aspect ratio already applied in projection matrix (from 1,1 to 1,aspect)
				Eigen::AlignedScaling3f((float)mode.sizeW, (float)mode.sizeH, 1.0f)
				// 2. Move image
				* Eigen::Translation3f(
					+(visCamera.image->boundsRel.center().x()-0.5f)*2.0f,
					-(visCamera.image->boundsRel.center().y()-0.5f)*2.0f,
					0.0f)
				// 1. Scale image (flip y axis from pixel coordinates to image coordinates)
				* Eigen::AlignedScaling3f(
					+visCamera.image->boundsRel.extends().x(),
					-visCamera.image->boundsRel.extends().y(),
					1.0f)
			);
			showGrayscaleFrame(visCamera.imageVis.texID, bgProjMat,
				Color{1,1,1,0}, 1.0f, visState.image.brightness, visState.image.contrast);
		}
	}

	/**
	 * Visualise outline of full frame and used area
	 */
	if (!(visCamera.imageVis.show && visCamera.image && !visCamera.imageVis.undistort) && calib.valid())
	{
		drawWithoutFrame();
	}

	/**
	 * Update observation error visualisation
	 */
	if (camera.pipeline->errorVisDirty.exchange(false))
	{
		auto &errorVis = visCamera.errors;
		auto errors = camera.pipeline->errorVis.contextualRLock();
		{
			updateErrorPointsVBO(errorVis.errorPtsVBO, errors->pointErrors,
				Color{ 1, 1, 1, 0.1f }, Color{ 1, 0, 0, 0.8f }, Color{ 1, 0, 1, 1.0f });
			errorVis.errorPtsCnt = errors->pointErrors.size();
		}
		if (errors->errorMap.size() == errors->mapSize.x()*errors->mapSize.y())
		{
			errorVis.mapSize = errors->mapSize;
			// Doesn't seem to work, always invisible. Changing colorA.a to 0.2 does show all at 0.2 alpha
			float PixelSizeRel = 1.0f/errorVis.mapSize.x() * 2; // *2 because of -1 to 1 space
			updatePixelVBO(errorVis.errorMapVBO,    errors->errorMap,    errorVis.mapSize, Eigen::Vector2f::Zero(), 
				PixelSizeRel, 1.0f, Color{ 1,0,0,0.5f }, Color{ 1,0,0,0.5f });
			updatePixelVBO(errorVis.outlierMapVBO,  errors->outlierMap,  errorVis.mapSize, Eigen::Vector2f::Zero(), 
				PixelSizeRel, 1.0f, Color{ 0,0,0,0.2f }, Color{ 0,0,1,0.2f });
			updatePixelVBO(errorVis.coverageMapVBO, errors->coverageMap, errorVis.mapSize, Eigen::Vector2f::Zero(), 
				PixelSizeRel, 1.0f, Color{ 0,0,0,0.2f }, Color{ 0,1,0,0.5f });
			errorVis.hasMap = true;
		}
		else
		{
			errorVis.hasMap = false;
		}
	}

	/**
	 * Visualise frame based on phase
	 */
	bool displayInternalDebug = (GetState().mode == MODE_Replay || GetState().mode == MODE_Simulation) && (GetState().simAdvance.load() == 0 || dbg_isBreaking);
	float blobAlpha = visCamera.image && visCamera.imageVis.show? 0.6f : 1.0f, blobCross = 2.0f / mode.widthPx;
	if (visCamera.visMode == VIS_BACKGROUND_TILES)
	{
		auto bgCalib = camera.receiving.background.contextualRLock();
		float PixelSizeCur = bgCalib->tileSize * viewSize.x() * pixelRatio;
		visualisePoints2D(bgCalib->tiles, Color{ 0, 0, 1, 0.2f }, PixelSizeCur, 0.5f, false);
	}
	else if (visCamera.visMode == VIS_BLOB)
	{
		bool drawDistorted = phase == PHASE_Calibration_Point || !visCamera.imageVis.undistort;
		bool hideBlobsOnFrame = visCamera.imageVis.show && visCamera.image && !visCamera.imageVis.showBlobs;
		if (!hideBlobsOnFrame)
		{
			if (drawDistorted)
				visualiseBlobs2D(camFrame.rawPoints2D, camFrame.properties, Color{ 1, 0, 0, blobAlpha }, viewSize.x(), blobCross);
			else
				visualiseBlobs2D(camFrame.points2D, camFrame.properties, Color{ 1.0, 1.0, 0.2, blobAlpha }, viewSize.x(), blobCross);
			if (visState.showMarkerTrails && visFrame.isRealtimeFrame)
			{ // Only works with current frame, not past frames
				visualisePoints2D(visCamera.observations.ptsUnstable.begin(), visCamera.observations.ptsUnstable.end(), Color{ 0.4f, 0.4f, 1.0f, 0.2f }, 3.0f*pixelRatio);
				visualisePoints2D(visCamera.observations.ptsTemp.begin(), visCamera.observations.ptsTemp.end(), Color{ 1.0f, 0.2f, 0.2f, 0.2f }, 3.0f*pixelRatio);
				visualisePoints2D(visCamera.observations.ptsInactive.begin(), visCamera.observations.ptsInactive.end(), Color{ 0.6f, 0.8f, 0.2f, 0.2f }, 3.0f*pixelRatio);
			}
		}

		if (phase == PHASE_Automatic)
		{
			visSetupCamera(postProjMat, calib);
			for (auto &trackRecord : frame->trackers)
			{
				if (trackRecord.id > 0)
					visualisePose(trackRecord.poseFiltered, Color{ 0.8, 0.8, 0, 1.0f }, 0.1f, 1.0f);
				// TODO: Track individual large markers (3/4) - visualisation
				/* else if (trackRecord.hasIMU)
					visualisePose(trackRecord.poseFiltered, Color{ 0.8, 0.8, 0, 1.0f }, 0.1f, 1.0f);
				else
					visualisePointsSpheres({ VisPoint{ trackRecord.poseFiltered.translation(), Color{ 0.8, 0.8, 0, 1.0f }, 10.0f } }); */
			}
			visSetupProjection(postProjMat);
		}
		else if (phase == PHASE_Tracking)
		{
			if (visState.show2DClusters)
			{
				for (auto &cluster : camFrame.clusters2D)
				{
					Eigen::Matrix2f axis = sampleCovarianceExtremes(cluster.covariance, 3);
					visualiseEllipse<false>(cluster.center, axis.col(0), axis.col(1), Color{ 1.0f, 0.0f, 0.0f, 1.0f });
				}
			}

			auto &debugVis = visState.tracking.debug;
			if (debugVis.frameNum > 0 && debugVis.frameNum < frame->num)
				debugVis = {}; // Any one camera can reset
			if (displayInternalDebug && visState.tracking.debugMatchingState &&
				debugVis.frameNum == frame->num && debugVis.trackerID == visState.tracking.focusedTrackerID)
			{ // Visualise internal tracking debug instead of normal vis

				visualiseTarget2DMatchingStages(visState, calib, camFrame, *debugVis.calib,
					debugVis.internalData.matching.at(camera.pipeline->index), pipeline.params.track.expandMarkerFoV);

				if (visState.tracking.showUncertaintyAxis)
				{ // Visualise uncertainty axis of dominant camera from internal tracking debug data
					visualiseTarget2DUncertaintyAxis(debugVis.internalData.uncertaintyAxis.at(camera.pipeline->index));
				}

				return;
			}

			// Display poses in 3D
			visSetupCamera(postProjMat, calib);
			for (auto &record : frame->trackers)
				visualisePose(record.poseObserved, Color{ 0.8, 0.8, 0, 1.0 }, 0.1f, 1.0f);
			if (visState.room.showOrigin)
				visualiseOrigin(visState.room.origin, 1, 5);
			visSetupProjection(postProjMat);

			thread_local std::vector<Eigen::Vector2f> projected2D;
			for (auto &record : frame->trackers)
			{
				auto trackerIt = std::find_if(state.trackerConfigs.begin(), state.trackerConfigs.end(),
					[&](auto &t){ return t.id == record.id; });
				if (trackerIt == state.trackerConfigs.end()) continue;
				auto tracker = *trackerIt;
				if (tracker.type == TrackerConfig::TRACKER_MARKER)
				{
					// TODO: Render marker
					continue;
				}
				if (tracker.type != TrackerConfig::TRACKER_TARGET) continue;

				if (visState.tracking.showSearchBounds)
				{ // Show search bounds
					// Add positional uncertainty in target-space (rotated by prediction) to target-local bounds
					Eigen::Vector3f uncertainty = sampleCovarianceUncertainty<float,3>(record.covPredicted.topLeftCorner<3,3>(),
						pipeline.params.track.uncertaintySigma, record.posePredicted.rotation());
					uncertainty += Eigen::Vector3f::Constant(pipeline.params.track.minUncertainty3D);
					auto bounds = tracker.calib.bounds.extendedBy(uncertainty);
					Eigen::Projective3f mvp = calib.camera.cast<float>() * record.posePredicted;
					visualiseBounds2D(projectBounds(mvp, bounds));
				}

				Color colVisible = Color{ 0.0, 0.8, 0.2, 0.3f };
				Color colMatched = Color{ 0.8, 0.0, 0.2, 0.6f };
				Color colFiltered = Color{ 0.4f, 0.1f, 0.5f, 0.8f };
				Color colPredicted = Color{ 0.2, 0.5, 0.7, 0.4f };

				if (visState.tracking.showTargetObserved && record.result.isTracked())
				{
					// Visualise target points that were considered (since they should've been visible assuming the pose is about right)
					projectTarget(projected2D, tracker.calib, calib,
						record.poseObserved, pipeline.params.track.expandMarkerFoV);
					visualisePoints2D(projected2D, colVisible, 2.0f);

					// Visualise target points that are tracked this frame
					if (record.visibleMarkers.size() > camera.pipeline->index)
					{
						projectTarget(projected2D, tracker.calib, calib,
							record.visibleMarkers[camera.pipeline->index], record.poseObserved);
						visualisePoints2D(projected2D, colMatched, 2.0f);
					}
				}

				if (visState.tracking.showTargetPredicted)
				{
					projectTarget(projected2D, tracker.calib, calib,
						record.posePredicted, pipeline.params.track.expandMarkerFoV);
					visualisePoints2D(projected2D, colPredicted, 2.0f);
				}

				if (visState.tracking.showTargetFilteredCamera && record.result.isTracked())
				{
					projectTarget(projected2D, tracker.calib, calib,
						record.poseFiltered, pipeline.params.track.expandMarkerFoV);
					visualisePoints2D(projected2D, colFiltered, 2.0f);
				}
			}

			// Display triangulated points
			// TODO: Move triangulatedPoint as trackingResult to frameRecords
			/* static unsigned int triPointVBO = 0;
			thread_local std::vector<VisPoint> vertices;
			vertices.clear();
			Color colorNC = Color{ 1, 0, 0, 0.5f }, colorC = Color{ 0.5f, 0, 1, 0.5f };
			for (const auto &pt : pipeline.tracking.triangulations3D)
				vertices.emplace_back(pt.pos, pt.confidence < 4? colorNC : colorC, 4.0f);
			visualisePointsVBOSprites(triPointVBO, pipeline.tracking.triangulations3D.size(), true); */
		}
		else if (phase == PHASE_Calibration_Target)
		{
			if (visFrame.target.hasObs())
			{
				visSetupCamera(postProjMat, calib);

				// Visualise all markers at once to z-sort them appropriately
				auto &markerPoints = visualiseVisTargetMarkers(pipeline, visState, visFrame.target);
				visualisePointsSpheresDepthSorted(markerPoints);

				// Reinstate projection after 3D
				visSetupProjection(postProjMat);
			}
		}
		else if (phase == PHASE_Calibration_Point)
		{
			if (visState.observations.visSavedObs >= 0 && visState.observations.visSavedObs < visState.observations.savedObs.size())
			{
				const auto &obsCmp = visState.observations.savedObs[visState.observations.visSavedObs];
				if (camera.pipeline->index < obsCmp.visPoints.size())
				{
					const auto &pts = obsCmp.visPoints[camera.pipeline->index];
					visualisePoints2D(pts.begin(), pts.end(), Color{ 1.0f, 1.0f, 1.0f, 0.2f }, 3.0f*pixelRatio);
				}
			}
			else if (!pipeline.recordSequences && calib.valid() && visCamera.errors.hasMap)
			{
				visualisePointsVBOSprites(visCamera.errors.errorPtsVBO, visCamera.errors.errorPtsCnt, true, 3.0f*pixelRatio);
			}
			else
			{
				visualisePoints2D(visCamera.observations.ptsStable.begin(), visCamera.observations.ptsStable.end(), Color{ 1.0f, 1.0f, 1.0f, 0.2f }, 3.0f*pixelRatio);
				visualisePoints2D(visCamera.observations.ptsUnstable.begin(), visCamera.observations.ptsUnstable.end(), Color{ 0.4f, 0.4f, 1.0f, 0.2f }, 3.0f*pixelRatio);
				visualisePoints2D(visCamera.observations.ptsTemp.begin(), visCamera.observations.ptsTemp.end(), Color{ 1.0f, 0.2f, 0.2f, 0.2f }, 3.0f*pixelRatio);
				visualisePoints2D(visCamera.observations.ptsInactive.begin(), visCamera.observations.ptsInactive.end(), Color{ 0.6f, 0.8f, 0.2f, 0.2f }, 3.0f*pixelRatio);
			}

			auto room = pipeline.pointCalib.room.contextualRLock();
			if (!room->floorPoints.empty())
			{ // TODO: Accessing pipeline in vis without lock
				visSetupCamera(postProjMat, calib);
				thread_local std::vector<VisPoint> markerPoints;
				markerPoints.clear();
				Color col = { 0.6f, 1.0f, 0.1f, 1.0f };
				for (auto &pt : room->floorPoints)
				{
					if (pt.sampleCount > 3)
						markerPoints.emplace_back(pt.pos.cast<float>(), (Color8)col, 0.01f);
				}
				visualisePointsSpheres(markerPoints);
				visSetupProjection(postProjMat);
			}
		}
	}
	else if (visCamera.visMode == VIS_ERROR_VIS)
	{ // TODO: Proper UI controls to enable/disable error visualisation as desired
		auto &errorVis = visCamera.errors;
		if (calib.valid() && errorVis.hasMap)
		{
			//visualisePointsVBO(errorVis.errorPtsVBO, errorVis.errorPtsCnt, true);
			float PixelSize = (float)viewSize.x()/errorVis.mapSize.x();
			visualisePointsVBOSprites(errorVis.errorMapVBO		, errorVis.mapSize.prod(), false, PixelSize);
			visualisePointsVBOSprites(errorVis.outlierMapVBO	, errorVis.mapSize.prod(), false, PixelSize);
			visualisePointsVBOSprites(errorVis.coverageMapVBO	, errorVis.mapSize.prod(), false, PixelSize);
			//visualiseDistortion(calib, pipeline.isSimulationMode? camera.pipeline->simulation.calib : calib, mode, 0.5f, 0.8f, 0.0f);
		}
		else
		{
			//visualiseDistortion(calib, pipeline.isSimulationMode? camera.pipeline->simulation.calib : calib, mode, 0.5f, 0.8f, 0.0f);
		}
		visualiseBlobs2D(camFrame.rawPoints2D, camFrame.properties, Color{ 1, 0, 0, blobAlpha }, viewSize.x(), blobCross);
	}
	else if (visCamera.visMode == VIS_VISUAL_DEBUG)
	{
		// Show other blobs next to it normally
		visualiseBlobs2D(camFrame.points2D, camFrame.properties, Color{ 1, 1.0, 0.2, blobAlpha }, viewSize.x(), blobCross);

		auto vis_lock = camera.receiving.visualDebug.contextualRLock();
		auto &visual = *vis_lock;
		if (!visual.hasBlob) return;

		float PixelSize = (float)viewSize.x() / mode.widthPx;
		float PixelStride = 2.0f / mode.widthPx; // 2 because of -1 to 1 space

		// Bounds
		Eigen::Vector2i extendsPX = visual.bounds.extends();
		Eigen::Vector2f boundsCenterPX = visual.bounds.center<float>();
		Eigen::Vector2f boundsCenter = pix2cam(mode, visual.bounds.center<float>());

		// Actual detected blob
		Eigen::Vector2f centroidPX = cam2pix(mode, visual.pos);
		//Eigen::Vector2f centroidOffsetPX = centroidPX - boundsCenterPX;
		//float blobRadiusPX = visual.size;
		Eigen::Vector2f vizCenterPX = boundsCenterPX-centroidPX;

		// Adjust for size of visualisation pixels
		auto vizCenterPXOld = vizCenterPX;
		vizCenterPX *= PixelStride;

		LOG(LGUI, LDebug, "VisualDebug: viewSize %dx%d, PixelStride %f", viewSize.x(), viewSize.y(), PixelStride);
		LOG(LGUI, LDebug, "   VizCenterPX %fx%f then %fx%f, boundsCenterPX %fx%f, centroid %fx%f, centroidPX %fx%f, ",
			vizCenterPXOld.x(), vizCenterPXOld.y(), vizCenterPX.x(), vizCenterPX.y(),
			boundsCenterPX.x(), boundsCenterPX.y(), visual.pos.x(), visual.pos.y(), centroidPX.x(), centroidPX.y());

		// Visualise grayscale image
		thread_local unsigned int imagePixelVBO = 0;
		updatePixelVBO(imagePixelVBO, visual.image, extendsPX, boundsCenter, 
			PixelStride, PixelSize, Color{ 0,0,0,1.0f }, Color{ 1,1,1,1.0f });
		visualisePointsVBOSprites(imagePixelVBO, extendsPX.prod(), false);

		// Visualise binary mask
		thread_local unsigned int maskPixelVBO = 0;
		updatePixelVBO(maskPixelVBO, visual.mask, extendsPX, boundsCenter, 
			PixelStride, PixelSize, Color{ 0,0,0,0 }, Color{ 0,1,0,0.4f });
		visualisePointsVBOSprites(maskPixelVBO, extendsPX.prod(), false);

		// Visualise detected blob
		visualiseBlobCircle(visual.pos, visual.size*PixelStride, Color{ 1, 0, 0, 1 }, PixelStride);

		// Visualise refined edge
		Eigen::Vector2f boundsOrigin = pix2cam<float>(mode, visual.bounds.min.cast<float>());
		thread_local std::vector<VisPoint> edgePoints;
		edgePoints.clear();
		for (auto &pt : visual.boundPoints)
			edgePoints.push_back(VisPoint{ (boundsOrigin + pt*PixelStride).homogeneous(), Color{ 0,1,1,1.0f }, PixelSize/2 });
		visualisePointsSprites(edgePoints);
	}
	else if (visCamera.visMode == VIS_EMULATION)
	{
		updateEmulationVisualisation(camera, visCamera, camFrame, viewSize);
	}
}

static bool updateAutoZoom(const VisualisationState &visState, const TrackingCameraState &camera, CameraVisState &visCamera, const CameraFrameRecord &frame, const VisFrameLock &visFrame)
{
	CameraMode &mode = camera.pipeline->mode;
	CameraCalib &calib = camera.pipeline->calib;
	if (visCamera.visMode == VIS_VISUAL_DEBUG)
	{ // Zoom into blob and snap to it if it moves off-screen
		auto vis_lock = camera.receiving.visualDebug.contextualRLock();
		if (vis_lock->hasBlob)
		{ // Need to move center so that full bounds are visible
			Eigen::Vector2i center = cam2pix(mode, visCamera.view.center).cast<int>();
			Eigen::Vector2i size(mode.widthPx/visCamera.view.zoom, mode.heightPx/visCamera.view.zoom);
			if (!Bounds2i(center, size).includes(vis_lock->bounds))
			{
				visCamera.view.center = vis_lock->pos;
				return true;
			}
		}
		return false;
	}

	// Try to find a 3D target to zoom into
	Eigen::Vector3f target3D = visState.getPreferredTarget(visFrame);
	if (!target3D.hasNaN())
	{
		visCamera.view.center = projectPoint2D(calib.camera, target3D);
		return true;
	}

	if (GetState().mode == MODE_Replay && visCamera.imageVis.followFrame && visCamera.image)
	{ // Follow the frame that was sent, which roughly matches the view in the recorded live session
		visCamera.view.center = Eigen::Vector2f(
			+(visCamera.image->boundsRel.center().x()-0.5f)*2.0f*mode.sizeW,
			-(visCamera.image->boundsRel.center().y()-0.5f)*2.0f*mode.sizeH);
		visCamera.view.center = distortPointUnstable(calib, visCamera.view.center);
		visCamera.view.zoom = 1.0f/visCamera.image->boundsRel.extends().x();
		return true;
	}

	Bounds2f focusBounds;
	auto &points = visCamera.imageVis.undistort? frame.points2D : frame.rawPoints2D;
	bool gotBounds = false;

	{ // Try to find a big nearby cluster to keep focus on
		auto clusters2D = dbscan<2,float, int>(points, 50.0f*PixelSize, 3);
		int bestCluster = -1;
		float bestScore = 0;
		for (int i = 0; i < clusters2D.size(); i++)
		{
			Eigen::Vector2f center = Eigen::Vector2f::Zero();
			for (int p = 0; p < clusters2D[i].size(); p++)
				center += points[clusters2D[i][p]];
			center /= clusters2D[i].size();
			float dist = (center-visCamera.view.center).norm() * PixelFactor;
			float score = clusters2D[i].size() / dist;
			if (bestScore < score)
			{
				bestScore = score;
				bestCluster = i;
			}
		}

		if (bestCluster >= 0)
		{
			for (int i : clusters2D[bestCluster])
				focusBounds.include(Bounds2f(points[i], Eigen::Vector2f::Constant(frame.properties[i].size)));
			gotBounds = true;
			LOG(LGUI, LTrace, "Focusing on initial cluster of %d points!", (int)clusters2D[bestCluster].size());
		}
	}

	if (!gotBounds)
	{ // Try to find a nearby blob to keep focus on
		int bestBlob = -1;
		float bestScore = 0;
		for (int i = 0; i < points.size(); i++)
		{
			float dist = (points[i] - visCamera.view.center).norm();
			float score = frame.properties[i].value / dist;
			if (bestScore < score)
			{
				bestScore = score;
				bestBlob = i;
			}
		}

		if (bestBlob >= 0)
		{
			focusBounds.include(Bounds2f(points[bestBlob], Eigen::Vector2f::Constant(frame.properties[bestBlob].size)));
			gotBounds = true;
		}
	}

	if (gotBounds)
	{
		// If we actually track a group of blobs, keep all of them in view as good as possible
		for (int j = 0; j < 2; j++)
		{
			Bounds2f bestView = focusBounds;
			bestView.extendBy(bestView.extends() + Eigen::Vector2f::Constant(50.0f * PixelSize));
			focusBounds = Bounds2f();
			int ptCnt = 0;
			for (int i = 0; i < points.size(); i++)
			{
				if (bestView.includes(points[i]))
				{
					focusBounds.include(Bounds2f(points[i], Eigen::Vector2f::Constant(frame.properties[i].size)));
					ptCnt++;
				}
			}
			LOG(LGUI, LTrace, "   Expanded to %d points in iteration %d", ptCnt, j);
		}

		focusBounds.extendBy(Eigen::Vector2f::Constant(50.0f * PixelSize));
		float diff = (focusBounds.center() - visCamera.view.center).norm();
		if (diff > 200*PixelSize)
		{ // Delay switch if it is far away, likely our cluster disappeared and might reappear soon
			int delay = diff / (10*PixelSize);
			if (visCamera.view.autoZoomDelay++ < delay)
				return false;
			LOG(LGUI, LDebug, "Accepting switch to a target %fpx away after %d frames delay!", diff*PixelFactor, delay);
		}
		else if (visCamera.view.autoZoomDelay > 0)
			visCamera.view.autoZoomDelay--;
		const float lerpCenter = 0.2f, lerpZoomIn = 0.01f, lerpZoomOut = 0.02f;
		visCamera.view.center = focusBounds.center() * lerpCenter + visCamera.view.center * (1-lerpCenter);
		float zoom = std::min(2.0f * mode.sizeW / focusBounds.extends().x(), 2.0f * mode.sizeH / focusBounds.extends().y()) * 0.7f;
		if (zoom > visCamera.view.zoom)
			visCamera.view.zoom = zoom*lerpZoomIn + visCamera.view.zoom*(1-lerpZoomIn);
		else
			visCamera.view.zoom = zoom*lerpZoomOut + visCamera.view.zoom*(1-lerpZoomOut);
		return true;
	}
	return false;
}

static void assureValidView(CameraVisState &visCamera, const CameraMode &mode)
{
	// Clamp zoom to range
	visCamera.view.zoom = std::max(visCamera.view.maxZoom, std::min(50.0f, visCamera.view.zoom));
	// Clamp view rect to frame size
	Bounds2f allowableRect = Bounds2f(
		-mode.sizeW/visCamera.view.maxZoom + mode.sizeW/visCamera.view.zoom,
		-mode.sizeH/visCamera.view.maxZoom + mode.sizeH/visCamera.view.zoom,
		+mode.sizeW/visCamera.view.maxZoom - mode.sizeW/visCamera.view.zoom,
		+mode.sizeH/visCamera.view.maxZoom - mode.sizeH/visCamera.view.zoom
	);
	visCamera.view.center = allowableRect.clamp(visCamera.view.center);
}

static Eigen::Isometry3f generateProjectionMatrix(CameraVisState &visCamera, const CameraMode &mode)
{
	Eigen::Isometry3f postProjMat = Eigen::Isometry3f::Identity();
	// Build post-projection matrix
	assureValidView(visCamera, mode);
	// Build zoom / flip projection matrix
	float flip = visCamera.view.rotate180? -1 : +1;
	postProjMat(0,0) = visCamera.view.zoom * flip * (float)mode.factorW;
	postProjMat(1,1) = visCamera.view.zoom * flip * (float)mode.factorH;
	postProjMat.translation().x() = -visCamera.view.center.x()*postProjMat(0,0);
	postProjMat.translation().y() = -visCamera.view.center.y()*postProjMat(1,1);
	return postProjMat;
}

static float updateUndistortedBorders(CameraVisState &visCamera, const CameraCalib &calibHost, const CameraCalib &calib, const CameraMode &mode,
	std::vector<VisPoint> &fullBorder, std::vector<VisPoint> &usedBorder)
{
	float maxZoom = 0.0f;
	float correction = calib.fInv/calibHost.fInv;

	// Calculate rect actually used for blob detection
	ProgramLayout layout = SetupProgramLayout(mode.widthPx, mode.heightPx, 8, false);
	Eigen::Vector2f tl = layout.validMaskRect.min.cast<float>() * 2.0f / mode.widthPx;
	Eigen::Vector2f br = layout.validMaskRect.max.cast<float>() * 2.0f / mode.widthPx;
	tl = Eigen::Vector2f(tl.x()-1, mode.aspect-tl.y());
	br = Eigen::Vector2f(br.x()-1, mode.aspect-br.y());

	// Pepare corners of both rects
	std::array<Eigen::Vector2f, 4> cornersFull = {
		Eigen::Vector2f(-1,  mode.aspect), Eigen::Vector2f( 1,  mode.aspect),
		Eigen::Vector2f( 1, -mode.aspect), Eigen::Vector2f(-1, -mode.aspect)
	};
	std::array<Eigen::Vector2f, 4> cornersUsed = {
		tl, Eigen::Vector2f( br.x(),  tl.y()),
		br, Eigen::Vector2f( tl.x(), br.y())
	};

	// Generate visualisation points
	int sideRes = 40;
	Color8 colFull = Color{ 0.5f, 0.5f, 0.5f, 0.4f };
	Color8 colUsed = Color{ 0.4f, 0.6f, 0.4f, 0.4f };
	fullBorder.clear();
	usedBorder.clear();
	fullBorder.reserve(sideRes*4 - 4);
	usedBorder.reserve(sideRes*4 - 4);
	for (int s = 0; s < 4; s++)
	{
		Eigen::Vector2f dirFull = cornersFull[(s+1)%4] - cornersFull[s];
		Eigen::Vector2f dirUsed = cornersUsed[(s+1)%4] - cornersUsed[s];
		for (int i = 0; i < sideRes; i++)
		{
			float side = (float)i/sideRes;
			Eigen::Vector2f posFull = cornersFull[s] + dirFull*side;
			Eigen::Vector2f posUsed = cornersUsed[s] + dirUsed*side;
			posFull = undistortPoint(calib, posFull)*calib.fInv*calibHost.f;
			posUsed = undistortPoint(calib, posUsed)*calib.fInv*calibHost.f;
			fullBorder.push_back({ Eigen::Vector3f(posFull.x(), posFull.y(), 0.1f), colFull });
			usedBorder.push_back({ Eigen::Vector3f(posUsed.x(), posUsed.y(), 0.1f), colUsed });
			// Also expand zoom area if it reaches outside
			maxZoom = std::max(maxZoom, posFull.x());
			maxZoom = std::max(maxZoom, posFull.y()/(float)mode.aspect);
		}
	}
	return 1.0f / maxZoom;
}

#pragma GCC push_options
#pragma GCC optimize ("-O3")

static void __attribute__ ((optimize("3"))) calculateCameraDistortionMap(const TrackingCameraState &camera, CameraVisState &visCamera)
{
	CameraMode &mode = camera.pipeline->mode;
	CameraCalib &calib = camera.pipeline->calib;
	visCamera.view.camZoom = updateUndistortedBorders(visCamera, calib, calib, mode, visCamera.calibration.fullBorder, visCamera.calibration.usedBorder);
	// Update a map from undistorted to distorted
	// Maximum iterations should only ever be necessary in very distorted areas, tolerance is the real threshold
	// But a few highly distorted areas can tank performance, resulting in lags, so hope this is fine
	// Currently, everything <10ms, so all good
#if PERF >= 2
	int tgtSize = 200, itCount = 100;
#elif PERF == 1
	int tgtSize = 100, itCount = 50;
#elif PERF == 0
	int tgtSize = 20, itCount = 10;
#endif
	int tgtHeight = std::ceil(tgtSize*mode.aspect);
	int width = tgtSize/visCamera.view.camZoom, height = tgtHeight/visCamera.view.camZoom;
	thread_local std::vector<Eigen::Vector2f> undistortionTex;
	undistortionTex.clear();
	undistortionTex.resize(width*height);
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
		{
			Eigen::Vector2f pos(((x+0.5f)*2.0f-width)/tgtSize, ((y+0.5f)*2.0f-height)/tgtSize);
			undistortionTex[y*width + x] = distortPointUnstable(calib, pos, itCount, 0.5f*PixelSize);
		}
	loadVectorField(visCamera.calibration.undistortionTexID, undistortionTex, width, height);
	visCamera.calibration.undistortMapScale = Eigen::Vector2f((float)width/tgtSize, (float)height/tgtSize);
	visCamera.calibration.precalculated = true;
}

#pragma GCC pop_options

static bool updateAdaptiveImageStreaming(Bounds2i &bounds, const CameraVisState &visCamera, const CameraPipeline &camera)
{ // Adapt requested image to zoomed view
	Bounds2i fullFrame(0, 0, camera.mode.widthPx, camera.mode.heightPx);
	float expansionFactor = 1.2f; // To reduce image load, disable expansion
	Bounds2f view(visCamera.view.center, 
		Eigen::Vector2f(camera.mode.sizeW*2.0f/visCamera.view.zoom, camera.mode.sizeH*2.0f/visCamera.view.zoom));
	Bounds2f imageBounds;
	if (visCamera.imageVis.undistort)
	{ // Undistort roughly - since values outside of image could be used, it could be even more unstable than usual
		imageBounds.include(distortPointUnstable(camera.calib, Eigen::Vector2f(view.max.x(), view.max.y()), 100, 0.5f*PixelSize));
		imageBounds.include(distortPointUnstable(camera.calib, Eigen::Vector2f(view.max.x(), view.min.y()), 100, 0.5f*PixelSize));
		imageBounds.include(distortPointUnstable(camera.calib, Eigen::Vector2f(view.min.x(), view.max.y()), 100, 0.5f*PixelSize));
		imageBounds.include(distortPointUnstable(camera.calib, Eigen::Vector2f(view.min.x(), view.min.y()), 100, 0.5f*PixelSize));
	}
	else
	 	imageBounds = view;
	Bounds2f normBounds(Eigen::Vector2f(
			+imageBounds.center().x()*camera.mode.factorW/2.0f + 0.5f,
			-imageBounds.center().y()*camera.mode.factorH/2.0f + 0.5f),
		Eigen::Vector2f(
			imageBounds.extends().x()*camera.mode.factorW/2.0f * expansionFactor,
			imageBounds.extends().y()*camera.mode.factorH/2.0f * expansionFactor));
	Bounds2i pixelBounds(
		normBounds.min.x()*camera.mode.widthPx, normBounds.min.y()*camera.mode.heightPx,
		normBounds.max.x()*camera.mode.widthPx, normBounds.max.y()*camera.mode.heightPx);
	pixelBounds.overlapWith(fullFrame); // Due to expansionFactor and undistortion
	if (bounds != pixelBounds)
	{
		bounds = pixelBounds;
		LOG(LGUI, LTrace, "Requesting image bounds (%d,%d)(%d,%d)",
			bounds.min.x(), bounds.min.y(), bounds.max.x(), bounds.max.y());
		return true;
	}
	return false;
}

/**
 * Show an overlay message or button in the center of the given rect, in position index of count
 */
static bool drawOverlayMessage(ImRect rect, int index, int count, ImGuiID id, bool button, const std::string &message)
{ // Draw background box and
	ImGuiContext& g = *ImGui::GetCurrentContext();

	// Modify Frame Layout
	ImVec2 FramePadding =  ImVec2(10, 10);
	float FrameRounding = 10;

	float msgHeight = g.FontSize + FramePadding.y * 2;
	float msgInterval = msgHeight+g.Style.ItemSpacing.y;
	float msgStackHeight = msgInterval*count - g.Style.ItemSpacing.y;
	float msgStartY = rect.GetCenter().y - msgStackHeight/2 + msgInterval*index;

	const char* text_start = message.c_str(), *text_end = message.c_str() + message.size();
	float msgWidth = ImGui::CalcTextSize(text_start, text_end).x + FramePadding.x*2;
	float msgStartX = rect.GetCenter().x - msgWidth/2;

	ImVec2 msgStart(msgStartX, msgStartY);
	ImVec2 msgSize(msgWidth, msgHeight);
	//LOGC(LInfo, "Drawing msg %d at %.2fx%.2f of size %.2fx%.2f", index, msgStartX, msgStartY, msgWidth, msgHeight);

	const ImRect bb(msgStartX, msgStartY, msgStartX+msgWidth, msgStartY+msgHeight);
	ImGui::ItemSize(msgSize, FramePadding.y);
	if (!ImGui::ItemAdd(bb, id))
		return false;

	bool hovered = false, held = false, pressed = false;
	if (button)
	{
		pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held, 0);
	}

	// Modify Frame Style
	auto frameBG = ImGui::GetStyleColorVec4(ImGuiCol_FrameBg);
	frameBG.w *= 0.5f;
	ImGui::PushStyleColor(ImGuiCol_FrameBg, frameBG);

	// Render
	const ImU32 bg_col = ImGui::GetColorU32(button? ((held && hovered) ? ImGuiCol_ButtonActive : hovered ? ImGuiCol_ButtonHovered : ImGuiCol_Button) : ImGuiCol_FrameBg);
	const ImU32 text_col = ImGui::GetColorU32(ImGuiCol_Text);
	ImGui::RenderNavCursor(bb, id);
	ImGui::RenderFrame(bb.Min, bb.Max, bg_col, true, FrameRounding);
	ImGui::RenderText(bb.Min+FramePadding, text_start, text_end, false);

	// Reset Frame Style
	ImGui::PopStyleColor(1);
	return pressed;
}