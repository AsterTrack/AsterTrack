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

#include "pipeline/pipeline.hpp"
#include "gl/visualisation.hpp"
#include "gl/sharedGL.hpp"
#include "system/vis.hpp"

#include "util/debugging.hpp"

#include "imgui/imgui_onDemand.hpp"

#include "target/rotationGen.hpp"
#include "calib/opt/covariance.hpp"
#include "util/eigenalg.hpp"

#include "implot/implot.h"

struct wl_display;
struct wl_resource;
#include "GL/glew.h"

//#define VIEW_CAPTURE_MOUSE_CURSOR // Wayland/GLFW has a bug where it doesn't update the mouse pos in between captures when not moved 
//#define VIEW_RAW_MOUSE_MOVEMENT // Can't really adjust sensitivity, but allows simultaneous use of touchpad and keys on laptops that prevent that

static void visualiseState3D(const ServerState &state, VisualisationState &vis, View3D &view3D, Eigen::Vector2i viewSize, float dT);
static void visualRotationGenAnalysis(const VisualisationState &visState, const RotationGenerationParameters &gen);

void InterfaceState::Update3DViewUI(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open, ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();

	auto posToUI = [](Eigen::Vector2f pos)
	{
		auto window = ImGui::GetCurrentWindowRead();
		ImVec2 rel((pos.x()/2+0.5f), (pos.y()/2+0.5f));
		return rel*window->InnerRect.GetSize() + window->InnerRect.Min - window->Pos;
	};

	auto viewWin = ImGui::GetCurrentWindowRead();
	AddOnDemandRender(viewWin->InnerRect, [](const ImDrawList* dl, const ImDrawCmd* dc)
	{
		OnDemandItem &state = *static_cast<OnDemandItem*>(dc->UserCallbackData);
		ImVec2 size = SetOnDemandRenderArea(state, dc->ClipRect);
		glClearColor(0.2f, 0.0, 0.2f, 0.0);
		glClearDepth(0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Enable Depth for 3D scene
		glDepthMask(GL_TRUE);
		glDepthFunc(GL_GEQUAL);
		glEnable(GL_DEPTH_TEST);

		// Update and render visualisations
		visualiseState3D(GetState(), GetUI().visState,
			GetUI().view3D, Eigen::Vector2i(size.x, size.y), GetState().isStreaming? GetUI().deltaTime : 0.0f);
	}, nullptr);

	std::string viewLabel = "";
	ImVec2 toolbarPos = ImGui::GetCursorPos();
	ImVec2 viewOrigin = ImGui::GetCursorPos() - ImGui::GetStyle().WindowPadding;

	bool viewBGHovered, viewHeld;
	bool viewPressed = InteractionSurface("3DView", viewWin->InnerRect, viewBGHovered, viewHeld);
	bool viewFocused = ImGui::IsItemFocused();
	bool viewHovered = ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenOverlappedByItem);

	/**
	 * 3D View Key Input
	 */

	ImGuiIO &io = ImGui::GetIO();
	if (viewFocused || viewHovered)
	{ // Process input
		float dT = io.DeltaTime > 0.05f? 0.016f : io.DeltaTime;
		float dM = dT *10.0f;
		auto &transform = view3D.viewTransform;

		// Movement
		if (view3D.orbit)
		{
			if (ImGui::IsKeyDown(ImGuiKey_S))
			{
				RequestUpdates();
				view3D.distance += 1.0f*dT;
			}
			if (ImGui::IsKeyDown(ImGuiKey_W))
			{
				RequestUpdates();
				view3D.distance -= 1.0f*dT;
			}
			if (ImGui::IsKeyDown(ImGuiKey_U))
			{
				view3D.orbit = false;
			}
		}
		else
		{
			float wasd = 2.0f, ud = 1.0f;
			if (ImGui::IsKeyDown(ImGuiKey_A))
			{
				RequestUpdates();
				transform.translation() += transform.rotation() * Eigen::Vector3f(-wasd*dT, 0, 0);
			}
			if (ImGui::IsKeyDown(ImGuiKey_D))
			{
				RequestUpdates();
				transform.translation() += transform.rotation() * Eigen::Vector3f(+wasd*dT, 0, 0);
			}
			if (ImGui::IsKeyDown(ImGuiKey_S))
			{
				RequestUpdates();
				transform.translation() += transform.rotation() * Eigen::Vector3f(0, 0, -wasd*dT);
			}
			if (ImGui::IsKeyDown(ImGuiKey_W))
			{
				RequestUpdates();
				transform.translation() += transform.rotation() * Eigen::Vector3f(0, 0, +wasd*dT);
			}
			if (ImGui::IsKeyDown(ImGuiKey_PageDown) || ImGui::IsKeyDown(ImGuiKey_Q))
			{
				RequestUpdates();
				transform.translation() += /*transform.rotation() **/ Eigen::Vector3f(0, 0, -ud*dT);
			}
			if (ImGui::IsKeyDown(ImGuiKey_PageUp) || ImGui::IsKeyDown(ImGuiKey_E))
			{
				RequestUpdates();
				transform.translation() += /*transform.rotation() **/ Eigen::Vector3f(0, 0, +ud*dT);
			}
			if (ImGui::IsKeyDown(ImGuiKey_Z))
			{
				view3D.orbit = true;
			}
		}
	}
	if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)
	 || ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows))
	{
		if (ImGui::IsKeyPressed(ImGuiKey_N))
		{
			RequestUpdates();
			view3D.sidePanelOpen = !view3D.sidePanelOpen;
		}
	}

	/**
	 * 3D View Mouse Interaction
	 */

	if (viewHovered && std::abs(io.MouseWheel) > 0.0001f)
	{
		if (view3D.orbit)
		{
			view3D.distance = std::max(0.001f, view3D.distance/(1.0f+io.MouseWheel*0.05f) - io.MouseWheel*0.0001f);
		}
	}

	if (viewPressed)
	{
		view3D.isDragging = true;
#ifdef VIEW_CAPTURE_MOUSE_CURSOR // Wayland/GLFW has a bug where it doesn't update the mouse pos in between captures when not moved 
		glfwSetInputMode(glfwWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	#ifdef VIEW_RAW_MOUSE_MOVEMENT
		if (glfwRawMouseMotionSupported()) glfwSetInputMode(glfwWindow, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
	#endif
#endif
	}
	if (view3D.isDragging && !viewHeld)
	{
		view3D.isDragging = false;
#ifdef VIEW_CAPTURE_MOUSE_CURSOR // Wayland/GLFW has a bug where it doesn't update the mouse pos in between captures when not moved 
		glfwSetInputMode(glfwWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	#ifdef VIEW_RAW_MOUSE_MOVEMENT
		if (glfwRawMouseMotionSupported()) glfwSetInputMode(glfwWindow, GLFW_RAW_MOUSE_MOTION, GLFW_FALSE);
	#endif
#endif
	}
	if (view3D.isDragging)
	{
#ifndef VIEW_CAPTURE_MOUSE_CURSOR // Wayland/GLFW has a bug where it doesn't update the mouse pos in between captures when not moved 
		ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
#endif
		float mouseSensitivity = 0.005;
		view3D.pitch += mouseSensitivity*io.MouseDelta.y;
		view3D.pitch = std::clamp(view3D.pitch, 0.0f, (float)PI);
		view3D.heading += mouseSensitivity*io.MouseDelta.x;
		view3D.heading = std::fmod(view3D.heading, (float)PI*2);
		view3D.viewTransform.linear() = getRotationXYZ(Eigen::Vector3f(view3D.pitch, 0, view3D.heading));

		if (view3D.orbit)
		{
			VisFrameLock visFrame = visState.lockVisFrame(state.pipeline);
			view3D.target = visState.getPreferredTarget(visFrame);
			if (!view3D.target.hasNaN())
				view3D.viewTransform.translation() = view3D.target + view3D.viewTransform.linear() * Eigen::Vector3f(0, 0, -view3D.distance); 
		}
	}

	//LOG(LGUI, LDebug, "Want capture mouse: %s, keyboard: %s", io.WantCaptureMouse? "true" : "false\n", io.WantCaptureKeyboard? "true" : "false");


	/**
	 * 3D View Interaction
	 */

	if (viewHovered)
	{ // Convert to projected space useful for 3D interactions
		ImVec2 mouseRel = (ImGui::GetMousePos() - viewWin->InnerRect.Min) / viewWin->InnerRect.GetSize();
		view3D.mousePos = Eigen::Vector2f(mouseRel.x*2-1, -(mouseRel.y*2-1));
	}
	else
		view3D.mousePos.setConstant(NAN);

	PipelineState &pipeline = state.pipeline;
	{
		VisTargetLock visTarget = visState.lockVisTarget();
		if (visTarget)
			visState.updateVisTarget(visTarget);

		if (visTarget.hasObs())
		{
			auto &frame = visTarget.obs->frames[visState.targetCalib.frameIdx % visTarget.obs->frames.size()];
			viewLabel = asprintf_s("Frame %d", frame.frame);
			if (visTarget.hasPose)
			{
				Eigen::Projective3f proj = view3D.getProj(viewWin->InnerRect.GetHeight() / viewWin->InnerRect.GetWidth()) * view3D.viewTransform.inverse();
				float radiusPx = 8.0f;
				for (auto &cam : pipeline.cameras)
				{
					Eigen::Vector3f dir = (cam->calib.transform.translation().cast<float>() - frame.pose.translation()).normalized();
					Eigen::Vector3f pos3D = frame.pose.translation() + dir*0.2f;
					Eigen::Vector3f pos2D = (proj * pos3D.homogeneous()).hnormalized();
					if (pos2D.z() < 0) continue;
					// GL z and y are inverted, so just invert y
					ImVec2 posUI = posToUI(Eigen::Vector2f(pos2D.x(), -pos2D.y()));
					//viewWin->DrawList->AddCircleFilled(posUI + viewWin->Pos, radiusPx, IM_COL32(100, 100, 150, 255));
					ImGui::SetCursorPos(posUI - ImVec2(radiusPx, radiusPx));
					ImGui::SetNextItemAllowOverlap();
					if (ImGui::Button(asprintf_s("##CamBtn%d", cam->index).c_str(), ImVec2(radiusPx*2, radiusPx*2)))
					{
						visState.target.cameraRays.resize(pipeline.cameras.size());
						visState.target.cameraRays[cam->index] = !visState.target.cameraRays[cam->index];
					}
				}
			}
		}

		if (visTarget && visState.target.markerHovered >= 0)
		{
			ImGuiID id = ImGui::GetID("TargetCalibMarker");
			ImVec2 pos = ImGui::GetMousePos();
			ImRect bb(pos - ImVec2(10,10), pos + ImVec2(10,10));
			ImGui::SetNextItemAllowOverlap();
			if (ImGui::ItemAdd(bb, id))
			{
				bool hovered, held;
				bool pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held);
				if (pressed)
				{
					visState.target.markerSelect[visState.target.markerHovered] =
						!visState.target.markerSelect[visState.target.markerHovered]; 
				}
			}
		}
		visState.target.markerHovered = -1;
	}

	/**
	 * Overlay UI Layout
	 */

	static float sidePanelWidth = 200;
	float areaEnd = ImGui::GetContentRegionAvail().x + ImGui::GetStyle().WindowPadding.x; // To use instead of GetRightAlignedStartPos
	ImRect area3D(viewOrigin, viewOrigin + viewWin->InnerRect.GetSize()), areaSide;
	if (view3D.sidePanelOpen)
	{
		areaSide = area3D;
		areaSide.Min.x = area3D.Max.x - (sidePanelWidth + ImGui::GetStyle().WindowPadding.x);
		areaSide.Max.x -= ImGui::GetStyle().WindowPadding.x;
		areaSide.Min.y += ImGui::GetStyle().WindowPadding.y;
		areaSide.Max.y = 0;
		assert(areaSide.GetWidth() == sidePanelWidth);

		areaEnd -= sidePanelWidth + ImGui::GetStyle().WindowPadding.x;
		area3D.Max.x -= sidePanelWidth + ImGui::GetStyle().WindowPadding.x;
	}


	/**
	 * 3D View Toolbar
	 */

	ImGui::SetCursorPos(toolbarPos);

	BeginViewToolbar();

	ImVec2 iconSize(ImGui::GetFontSize()*6/5, ImGui::GetFontSize());
	if (ImGui::ImageButton("Orbit", icons().visual, iconSize))
	{
		view3D.orbit = !view3D.orbit;
	}
	ImGui::SetItemTooltip("Toggle Orbit View");
	ImGui::SameLine();

	if (!viewLabel.empty())
	{
		ImGui::TextUnformatted(viewLabel.c_str());
		ImGui::SameLine();
	}

	ImGui::SetCursorPosX(areaEnd - GetBarWidth(ImGui::GetFrameHeight(), 1));
	ImGui::BeginDisabled(true);
	ImGui::Button("?", ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight()));
	ImGui::EndDisabled();
	ImGui::SetItemTooltip("Move around with WASD/Arrow Keys\nMove Up/Down with E/Q\nLook around with Left Mouse Drag");

	EndViewToolbar();


	/**
	 * Side Panel
	 */

	bool showWindow = false;
	if (view3D.sidePanelOpen)
	{
		ImGui::SetCursorPos(areaSide.Min);
		ImVec4 sideBG = ImGui::GetStyleColorVec4(ImGuiCol_ChildBg);
		sideBG.w = 0.3f;
		ImGui::PushStyleColor(ImGuiCol_ChildBg, sideBG);
		ImGui::SetNextWindowSizeConstraints(ImVec2(150, 100), ImVec2(FLT_MAX,FLT_MAX));
		showWindow = ImGui::BeginChild("Visualisation", ImVec2(0,0),
			ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AutoResizeX |
			ImGuiChildFlags_Borders | ImGuiChildFlags_TitleBar |
			ImGuiChildFlags_AlwaysUseWindowPadding, ImGuiWindowFlags_NoScrollbar, &view3D.sidePanelOpen);
		ImGui::PopStyleColor();
		if (!showWindow)
			ImGui::EndChild();
	}
	if (showWindow)
	{ // Side Panel
		sidePanelWidth = ImGui::GetWindowWidth();

		ImGui::Checkbox("Show Marker Rays", &visState.showMarkerRays);
		ImGui::Checkbox("Show 3D Clusters", &visState.show3DClusters);
		ImGui::Checkbox("Show 2D Clusters", &visState.show2DClusters);

		VisTargetLock visTarget = visState.lockVisTarget();
		if (visTarget && ImGui::TreeNode("Target Calibration"))
		{
			ImGui::Checkbox("Show Marker FoV", &visState.target.markerViewCones);
			ImGui::Checkbox("Show Marker Observations", &visState.target.markerObservations);
			ImGui::Checkbox("Focus on Selection", &visState.target.focusOnMarkerSelection);
			ImGui::TreePop();
		}

		ImGui::EndChild();
	}

	ImGui::End();
}

static void visualiseState3D(const ServerState &state, VisualisationState &visState, View3D &view3D, Eigen::Vector2i viewSize, float dT)
{
	const PipelineState &pipeline = state.pipeline;
	float visAspect = (float)viewSize.y()/viewSize.x();

	VisFrameLock visFrame = visState.lockVisFrame(pipeline);
	if (view3D.orbit)
	{
		view3D.target = visState.getPreferredTarget(visFrame);
		if (!view3D.target.hasNaN())
			view3D.viewTransform.translation() = view3D.target + view3D.viewTransform.linear() * Eigen::Vector3f(0, 0, -view3D.distance); 
	}
	visSetupView(view3D.getProj(visAspect), view3D.viewTransform.inverse());

	static float time = 15.0f;
	time += dT/6;

	visualiseSkybox(time);
	visualiseFloor();
	if (visState.room.showOrigin)
		visualiseOrigin(visState.room.origin, 1, 5);

	for (auto &camera : pipeline.cameras)
		visualiseCamera(camera->calib.transform.cast<float>());

	if (pipeline.isSimulationMode)
	{
		for (auto &camera : pipeline.cameras)
			visualiseCamera(camera->simulation.calib.transform.cast<float>(), { 0.2f, 0.6f, 0.2f, 1.0f });
	}

	if (visState.showMarkerRays && visFrame)
	{
		auto &frame = *visFrame.frameIt->get();
		for (auto &camera : pipeline.cameras)
			visualiseRays(camera->calib, frame.cameras[camera->index].points2D, Color{ 0.6f, 0.6f, 0.6f, 1.0f });
	}

	if (visState.show3DClusters && visFrame)
	{
		thread_local std::vector<VisModel> clusters;
		clusters.clear();
		for (auto &cluster : visFrame.frameIt->get()->cluster2DTri)
		{
			Eigen::Isometry3f pose(Eigen::Translation3f(cluster.center));
			clusters.emplace_back(
				composeCovarianceTransform(pose, cluster.covariance, 1),
				Color{ 0.4f, 0.8f, 0.2f, 0.6f });
		}
		if (!clusters.empty())
		{
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_CULL_FACE);
			glCullFace(GL_BACK);
			visualiseMeshesDepthSorted(clusters, smoothSphereMesh);
			glDisable(GL_CULL_FACE);
			glEnable(GL_DEPTH_TEST);
		}
	}

	if (visState.rotationSphere.visualise)
		visualRotationGenAnalysis(visState, pipeline.params.detect.rotGen);

	if (visFrame.target)
	{ // Only show target calibration data, not real-time state
		visState.updateVisTarget(visFrame.target);

		if (visFrame.target.hasObs() && visFrame.target.hasPose && visState.target.markerObservations)
		{ // Show show observations ray for each marker
			visualiseVisTargetObservations(pipeline.getCalibs(), visState, visFrame.target);
		}

		if (visFrame.target.hasObs() && visFrame.target.hasPose)
		{ // Draw observation rays for focused cameras, if any
			visualiseVisTargetObsCameraRays(pipeline.getCalibs(), visState, visFrame.target);
		}

		if (visFrame.target.hasObs() && visFrame.target.hasPose && visState.targetCalib.selectedSequence >= 0)
		{ // Draw all observation rays of a selected sequence
			auto obs_lock = pipeline.seqDatabase.contextualRLock();
			if (obs_lock->markers.size() > visState.targetCalib.selectedSequence)
			{
				auto &seq = obs_lock->markers[visState.targetCalib.selectedSequence];
				visualiseMarkerSequenceRays(pipeline.getCalibs(), visFrame.target, seq, visState.targetCalib.selectedSequence);
			}
		}

		// Draw marker visualisation of the current target and its related state
		auto &markerPoints = visualiseVisTargetMarkers(pipeline, visState, visFrame.target);
		if (!view3D.mousePos.hasNaN())
		{ // Inject UI interaction into 3D marker spheres before rendering
			auto interacting = interactWithVisTargetMarker(view3D.viewTransform, view3D.getProj(visAspect), view3D.mousePos);
			if (interacting.first >= 0)
			{
				VisPoint &vis = markerPoints[interacting.first];
				auto adapt = [](uint8_t &val) { val = std::min(255, std::max(100, val*2)); };
				if (vis.color.a < 150) adapt(vis.color.a);
				else
				{ // Adapt lowest color value
					int min = std::min(vis.color.r, std::min(vis.color.g, vis.color.b));
					if (vis.color.r == min) adapt(vis.color.r);
					else if (vis.color.g == min) adapt(vis.color.g);
					else if (vis.color.b == min) adapt(vis.color.b);
				}
			}
			if (interacting.second >= 0)
				visState.target.markerHovered = interacting.second;
		}
		// Draw transparent spheres
		visualisePointsSpheresDepthSorted(markerPoints);

		// Draw transparent cones sticking out of spheres
		if (visState.target.markerViewCones)
		{ // Show calculated directionality and field of view of each marker
			visualiseVisTargetMarkerFoV(pipeline.getCalibs(), visState, visFrame.target);
		}

		return;
	}

	thread_local std::vector<std::pair<VisPoint, VisPoint>> imuAccelLines;
	imuAccelLines.clear();
	Color colIMUQuatRaw = Color{ 0.1f, 0.1f, 1.0f, 1.0f };
	Color colIMUQuatFiltered = Color{ 1.0f, 0.1f, 0.1f, 1.0f };
	Color colIMUDir = Color{ 1.0f, 0.1f, 0.1f, 1.0f };
	float accelScale = 0.1f;
	if (!GetState().isStreaming)
	{
		for (auto &imu : pipeline.record.imus)
		{
			if (!imu || !imu->isFused) continue;
			auto view = imu->samplesFused.getView();
			if (view.empty()) continue;
			Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
			pose.linear() = view.back().quat.toRotationMatrix();
			pose.translation() = Eigen::Vector3f(0,0,1);
			visualisePose(pose, colIMUQuatRaw, 0.2f, 4.0f);
			imuAccelLines.emplace_back(
				VisPoint{ pose.translation(), colIMUDir },
				VisPoint{ pose.translation()+view.back().accel*accelScale, colIMUDir }
		 	);
		}
	}
	else if (visState.tracking.showOrphanedIMUs)
	{
		std::shared_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(50));
		if (pipeline_lock.owns_lock())
		{
			for (auto &tracker : pipeline.tracking.orphanedIMUs)
			{
				visualisePose(tracker.pose.filtered, colIMUQuatFiltered, 0.2f, 4.0f);
				Eigen::Vector3f dir = tracker.inertial.fusion.accel.cast<float>();
				imuAccelLines.emplace_back(
					VisPoint{ tracker.pose.filtered.translation(), colIMUDir },
					VisPoint{ tracker.pose.filtered.translation()+dir*accelScale, colIMUDir }
				);
			}
			
			for (auto &tracker : pipeline.tracking.trackedTargets)
			{
				{
					Eigen::Vector3f dir = tracker.state.state.velocity().cast<float>();
					Eigen::Vector3f pos = tracker.pose.filtered.translation();
					imuAccelLines.emplace_back(
						VisPoint{ pos, colIMUDir },
						VisPoint{ pos+dir*accelScale, colIMUDir }
					);
				}
				if (tracker.inertial)
				{
					Eigen::Vector3f dir = tracker.inertial.fusion.imuVelocity.cast<float>();
					Eigen::Vector3f pos = tracker.pose.filtered.translation();
					imuAccelLines.emplace_back(
						VisPoint{ pos, Color{ 0.5f, 0.1f, 0.1f, 1.0f } },
						VisPoint{ pos+dir*accelScale, Color{ 0.5f, 0.1f, 0.1f, 1.0f } }
					);
				}
				/* if (tracker.inertial)
				{
					Eigen::Vector3f dir = tracker.inertial.fusion.accel.cast<float>();
					Eigen::Vector3f pos = tracker.pose.filtered.translation();
					imuAccelLines.emplace_back(
						VisPoint{ pos, Color{ 0.1f, 0.5f, 0.1f, 1.0f } },
						VisPoint{ pos+dir*accelScale, Color{ 0.1f, 0.5f, 0.1f, 1.0f } }
					);
				} */
				/* if (tracker.inertial)
				{
					Eigen::Vector3f dir = tracker.state.state.getCombinedQuaternion().cast<float>().conjugate() * -Eigen::Vector3f::UnitZ();
					Eigen::Vector3f pos = tracker.pose.filtered.translation();
					imuAccelLines.emplace_back(
						VisPoint{ pos, Color{ 0.1f, 0.1f, 1.0f, 1.0f } },
						VisPoint{ pos+dir*accelScale, Color{ 0.1f, 0.1f, 1.0f, 1.0f } }
					);
				}
				if (tracker.inertial)
				{
					Eigen::Vector3f dir = tracker.inertial.fusion.accelLocal.cast<float>();
					Eigen::Vector3f pos = tracker.pose.filtered.translation();
					imuAccelLines.emplace_back(
						VisPoint{ pos, Color{ 0.1f, 0.1f, 0.5f, 1.0f } },
						VisPoint{ pos+dir*accelScale, Color{ 0.1f, 0.1f, 0.5f, 1.0f } }
					);
				} */
				/* if (tracker.inertial)
				{
					Eigen::Vector3f dir = tracker.inertial.fusion.tangentialVelocity.cast<float>();
					Eigen::Vector3f pos = tracker.pose.filtered * tracker.inertial.calibration.offset.cast<float>();
					imuAccelLines.emplace_back(
						VisPoint{ pos, Color{ 1.0f, 1.0f, 0.1f, 1.0f } },
						VisPoint{ pos+dir*accelScale, Color{ 1.0f, 1.0f, 0.1f, 1.0f } }
					);
				} */
				/* if (tracker.inertial)
				{
					Eigen::Vector3f dir = tracker.state.state.angularVelocity().cast<float>();
					Eigen::Vector3f pos = tracker.pose.filtered.translation();
					imuAccelLines.emplace_back(
						VisPoint{ pos, Color{ 0.1f, 0.5f, 0.1f, 1.0f } },
						VisPoint{ pos+dir*accelScale, Color{ 0.1f, 0.5f, 0.1f, 1.0f } }
					);
				}
				if (tracker.inertial)
				{
					Eigen::Vector3f dir = tracker.inertial.fusion.angularVelocity.cast<float>();
					Eigen::Vector3f pos = tracker.pose.filtered.translation();
					imuAccelLines.emplace_back(
						VisPoint{ pos, Color{ 1.0f, 0.1f, 1.0f, 1.0f } },
						VisPoint{ pos+dir*accelScale, Color{ 1.0f, 0.1f, 1.0f, 1.0f } }
					);
				} */
			}
		}
	}

	visualiseLines(imuAccelLines, 2.0f);

	// Else, show real-time situation
	if (!visFrame) return;
	auto &frame = *visFrame.frameIt->get();

	for (auto &record : frame.trackers)
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
		auto &target = tracker.calib;

		Color colPredicted = Color{ 0.2f, 0.2f, 0.8f, 1.0f };
		Color colObserved = Color{ 0.4f, 0.8f, 0.2f, 0.6f };
		Color colFiltered = Color{ 0.5f, 0.1f, 0.1f, 0.6f };

		// Visualise visible target markers used for tracking
		thread_local std::vector<VisPoint> markers;
		markers.resize(target.markers.size()*3 + visState.tracking.trailLength*3);
		for (auto &pt : markers) pt.color.a = 0.0f;

		if (visState.tracking.showTargetPredicted)
		{ // Show target markers in predicted pose
			if (record.result.isTracked())
				updateTargetMarkerVis(pipeline, target, record.visibleMarkers,
					record.posePredicted, colPredicted, 0.5f, &markers[target.markers.size()*1]);
			visualisePose(record.posePredicted, colPredicted, 0.2f, 2.0f);
		}
		if (visState.tracking.showInertialIntegrated)
			visualisePose(record.poseInertialIntegrated, Color{ 0.5f, 0.1f, 1.0f, 1.0f }, 0.2f, 2.0f);
		if (visState.tracking.showInertialFused)
			visualisePose(record.poseInertialFused, Color{ 0.1f, 0.6f, 1.0f, 1.0f }, 0.2f, 2.0f);
		if (visState.tracking.showInertialFiltered)
			visualisePose(record.poseInertialFiltered, Color{ 0.1f, 1.0f, 0.6f, 1.0f }, 0.2f, 2.0f);
		if (visState.tracking.showPoseExtrapolated)
			visualisePose(record.poseExtrapolated, colIMUQuatFiltered, 0.2f, 2.0f);

		if (visState.tracking.showTargetObserved && record.result.isTracked())
		{ // Show target markers in observed pose
			updateTargetMarkerVis(pipeline, target, record.visibleMarkers,
				record.poseObserved, colObserved, 0.7f, &markers[target.markers.size()*0]);
			visualisePose(record.poseObserved, colObserved, 0.2f, 2.0f);
		}

		if (visState.tracking.showTargetFiltered && record.result.isTracked())
		{ // Show target markers in filtered pose
			updateTargetMarkerVis(pipeline, target, record.visibleMarkers,
				record.poseFiltered, colFiltered, 1.0f, &markers[target.markers.size()*2]);
			visualisePose(record.poseFiltered, colFiltered, 0.2f, 4.0f);
		}

		if (visState.tracking.showSearchBounds)
		{ // Show search bounds
			// Add positional uncertainty in target-space (rotated by prediction) to target-local bounds
			Eigen::Vector3f uncertainty = sampleCovarianceUncertainty<float,3>(record.covPredicted.topLeftCorner<3,3>(),
				pipeline.params.track.uncertaintySigma, record.posePredicted.rotation());
			uncertainty += Eigen::Vector3f::Constant(pipeline.params.track.minUncertainty3D);
			auto bounds = target.bounds.extendedBy(uncertainty);
			auto corners = transformBounds(record.posePredicted, bounds);

			Color8 col = Color{ 1.0, 0.1, 0.1, 1.0 };
			std::vector<std::pair<VisPoint,VisPoint>> edges = {
				// Face cols[0]
				{ { corners[0], col }, { corners[1], col } },
				{ { corners[1], col }, { corners[2], col } },
				{ { corners[2], col }, { corners[3], col } },
				{ { corners[3], col }, { corners[0], col } },
				// Face cols[3]
				{ { corners[4], col }, { corners[5], col } },
				{ { corners[5], col }, { corners[6], col } },
				{ { corners[6], col }, { corners[7], col } },
				{ { corners[7], col }, { corners[4], col } },
				// "Struts"
				{ { corners[0], col }, { corners[4], col } },
				{ { corners[1], col }, { corners[5], col } },
				{ { corners[2], col }, { corners[6], col } },
				{ { corners[3], col }, { corners[7], col } }
			};
			visualiseLines(edges, 2);
		}

		// For covariances and trails
		colPredicted.a = 0.4f;
		colObserved.a = 0.4f;
		colFiltered.a = 0.4f;

		thread_local std::vector<VisModel> covariances;
		covariances.clear();
		auto enterCovariances = [&](TrackerRecord &trk)
		{
			if (visState.tracking.showTargetFiltered)
				covariances.emplace_back(composeCovarianceTransform(
					trk.poseFiltered, trk.covFiltered.topLeftCorner<3,3>(),
					visState.tracking.scaleCovariance), colFiltered);
			if (visState.tracking.showTargetObserved)
				covariances.emplace_back(composeCovarianceTransform(
					trk.poseObserved, trk.covObserved.topLeftCorner<3,3>(),
					visState.tracking.scaleCovariance), colObserved);
			if (visState.tracking.showTargetPredicted)
				covariances.emplace_back(composeCovarianceTransform(
					trk.posePredicted, trk.covPredicted.topLeftCorner<3,3>(),
					visState.tracking.scaleCovariance), colPredicted);
		};

		auto trailIt = visFrame.frameIt;
		for (int i = 0; i < visState.tracking.trailLength; i++)
		{
			if (trailIt == visFrame.frames.begin()) break;
			trailIt--;
			if (!trailIt->get() || !trailIt->get()->finishedProcessing) continue;
			auto trailPt = &markers[target.markers.size()*3+i*3];
			auto pastTrack = std::find_if(trailIt->get()->trackers.begin(), trailIt->get()->trackers.end(),
				[&](auto &t){ return t.id == record.id; });
			if (pastTrack == trailIt->get()->trackers.end()) continue;
			if (visState.tracking.showCovariancePos)
			{ // Show covariances, not just points
				enterCovariances(*pastTrack);
				continue;
			}
			if (visState.tracking.showTargetPredicted)
				trailPt[0] = { pastTrack->posePredicted.translation(), colPredicted, 0.001f };
			if (visState.tracking.showTargetObserved)
				trailPt[1] = { pastTrack->poseObserved.translation(), colObserved, 0.001f };
			if (visState.tracking.showTargetFiltered)
				trailPt[2] = { pastTrack->poseFiltered.translation(), colFiltered, 0.002f };
		}

		if (visState.tracking.showCovarianceSamples)
		{ // Visualise samples on covariance ellipsoid shell
			thread_local std::vector<VisPoint> samples;
			samples.resize(record.match2D.deviations.size());
			Color devCol = { 1.0f, 1.0f, 1.0f, 1.0f };
			Color hypCol = { 0.5f, 1.0f, 1.0f, 0.4f };
			for (int i = 0; i < record.match2D.deviations.size(); i++)
			{
				bool hyperdimensional = record.match2D.deviations[i].tail<3>().cwiseAbs().sum() > 0.00000001f;
				samples[i].pos = record.poseObserved * (record.match2D.deviations[i].head<3>() * visState.tracking.scaleCovariance);
				samples[i].size = 0.00001f*visState.tracking.scaleCovariance;
				samples[i].color = hyperdimensional? hypCol : devCol;
			}
			visualisePointsSpheres(samples);

			if (visState.tracking.showCovariancePos)
			{ // This should be the same, but it is not
				Eigen::Matrix3f covariance = fitCovarianceToSamples<3,float>(record.match2D.deviations);
				covariances.emplace_back(composeCovarianceTransform(
					record.poseObserved, covariance,
					visState.tracking.scaleCovariance), Color{ 0.2f, 0.8f, 0.2f, 0.4f });
			}
			if (visState.tracking.showCovariancePos)
			{ // This is the old fixed covariance
				Eigen::Matrix3f covariance = pipeline.params.track.filter.getSyntheticCovariance<float>().topLeftCorner<3,3>() * pipeline.params.track.filter.trackSigma;
				covariances.emplace_back(composeCovarianceTransform(
					record.poseObserved, covariance,
					visState.tracking.scaleCovariance), Color{ 0.2f, 0.5f, 0.8f, 0.4f });
			}
		}
		if (visState.tracking.showCovariancePos)
		{ // Visualise positional covariance ellipsoids
			glDisable(GL_DEPTH_TEST);
			glEnable(GL_CULL_FACE);
			glCullFace(GL_BACK);
			enterCovariances(record);
			visualiseMeshesDepthSorted(covariances, smoothSphereMesh);
			glDisable(GL_CULL_FACE);
			glEnable(GL_DEPTH_TEST);
		}
		if (visState.tracking.showCovarianceRot)
		{ // Visualise rotational covariance rings around pose cross
			// TODO: Show different rotational covariances somehow (predicted, observed, filtered)
			visualiseRotationalCovariance(record.poseFiltered, record.covFiltered.bottomRightCorner<3,3>(),
				visState.tracking.scaleCovariance, 0.2f);
		}

		visualisePointsSpheresDepthSorted(markers);
	}

	if (pipeline.phase == PHASE_Calibration_Point)
	{ // TODO: Accessing pipeline in vis without lock
		thread_local std::vector<VisPoint> markerPoints;
		markerPoints.clear();

		if (!pipeline.pointCalib.room.floorPoints.empty())
		{
			Color col = { 0.6f, 1.0f, 0.1f, 1.0f };
			for (auto &pt : pipeline.pointCalib.room.floorPoints)
			{
				if (pt.sampleCount > 3)
					markerPoints.emplace_back(pt.pos.cast<float>(), (Color8)col, 0.01f);
			}
		}

		Color col = { 0.1f, 0.6f, 1.0f, 1.0f };
		for (auto &tri : frame.triangulations)
		{
			markerPoints.emplace_back(tri.cast<float>(), (Color8)col, 0.01f);
		}

		visualisePointsSpheres(markerPoints);
	}

	if (pipeline.isSimulationMode)
	{
		auto sim_lock = pipeline.simulation.contextualRLock();
		thread_local std::vector<VisPoint> markerPoints;
		markerPoints.clear();
		Color gtCol = { 1.0f, 0.0f, 0.8f, 0.6f };
		for (const auto &object : sim_lock->objects)
		{
			if (!object.enabled) continue;
			for (const auto &pt : object.target.markers)
				markerPoints.emplace_back(object.pose * pt.pos, (Color8)gtCol, 0.01f*0.5f);
		}
		visualisePointsSpheres(markerPoints);
	}
}

static void visualRotationGenAnalysis(const VisualisationState &visState, const RotationGenerationParameters &gen)
{
	auto sphere = visState.rotationSphere;

	std::vector<Eigen::Quaternionf> rotations(gen.rollAxisShells*gen.shellPoints);
	for (int r = 0; r < gen.rollAxisShells; r++)
	{
		float rollAngle = 2*PI * ((float)r)/gen.rollAxisShells; // TODO: Affect with spread
		for (int i = 0; i < gen.shellPoints; i++)
			rotations[r*gen.shellPoints + i] = generateRotation(gen, r % gen.shells.size(), i, rollAngle);
	}

	StatDistf all = {};
	StatDistf min1 = {}, min2 = {}, min3 = {};
	StatDistf roll1 = {}, roll2 = {}, roll3 = {};
	std::vector<MultipleExtremum<float, 3>> minXYAngles(rotations.size(), 10000);
	std::vector<MultipleExtremum<float, 3>> minRollAngles(rotations.size(), 10000);
	for (int i = 0; i < rotations.size(); i++)
	{
		for (int j = i+1; j < rotations.size(); j++)
		{
			float angle = Eigen::AngleAxisf(rotations[i] * rotations[j].conjugate()).angle() * 180/PI;
			assert(!std::isnan(angle));
			assert(angle > 0.001f);
			if (i/gen.shellPoints == j/gen.shellPoints)
			{
				minXYAngles[i].min(angle);
				minXYAngles[j].min(angle);
			}
			else
			{
				minRollAngles[i].min(angle);
				minRollAngles[j].min(angle);
			}
			all.update(angle);
		}
		min1.update(minXYAngles[i].rank[0]);
		min2.update(minXYAngles[i].rank[1]);
		min3.update(minXYAngles[i].rank[2]);
		roll1.update(minRollAngles[i].rank[0]);
		roll2.update(minRollAngles[i].rank[1]);
		roll3.update(minRollAngles[i].rank[2]);
	}
	LOG(LGUI, LInfo, "%d rotations, %d pairwise distances, minimums:",
		(int)rotations.size(), all.num);
	LOG(LGUI, LInfo, "XY  : 1: [%f - %f - %f]  2: [%f - %f - %f]  3: [%f - %f - %f]",
		min1.min, min1.avg, min1.max, min2.min, min2.avg, min2.max, min3.min, min3.avg, min3.max);
	LOG(LGUI, LInfo, "Roll: 1: [%f - %f - %f]  2: [%f - %f - %f]  3: [%f - %f - %f]",
		roll1.min, roll1.avg, roll1.max, roll2.min, roll2.avg, roll2.max, roll3.min, roll3.avg, roll3.max);

	std::vector<VisPoint> visPoints;
	std::vector<VisPoint> visAngles;
	std::vector<std::pair<VisPoint,VisPoint>> visLines;
	visPoints.reserve((gen.rollAxisShells-sphere.hideRollShells) * (gen.shellPoints-sphere.hideShellPoints));
	visAngles.reserve((gen.rollAxisShells-sphere.hideRollShells) * (gen.shellPoints-sphere.hideShellPoints));
	for (int r = 0; r < gen.rollAxisShells-sphere.hideRollShells; r++)
	{
		float radius = std::pow(sphere.shellRadiusIncrease, r);

		ImVec4 col = ImPlot::GetColormapColor(r);
		for (int i = 0; i < gen.shellPoints-sphere.hideShellPoints; i++)
		{
			int index = r*gen.shellPoints + i;
			auto point = generateSpherePoint(gen, r % gen.shells.size(), i);
			auto quat = rotations[index];

			Eigen::Vector3f tip = (quat * Eigen::Vector3f(0,0,1));
			Eigen::Vector3f side = (quat * Eigen::Vector3f(0.05f,0,1));
			visLines.emplace_back(
				VisPoint { sphere.sphereOrigin + tip * radius, Color{col.x, col.y, col.z,1.0f} },
				VisPoint { sphere.sphereOrigin + side * radius, Color{col.x, col.y, col.z,1.0f} }
			);

			float exc = minXYAngles[index].rank[0] > sphere.minNeighbourAngle? 0.3f : 0.0f;
			visPoints.emplace_back(sphere.sphereOrigin + point * radius, Color{col.x, col.y-exc, col.z-exc,1.0f}, sphere.pointSize);

			auto vector = flexkalman::util::quat_ln(quat);
			visAngles.emplace_back(sphere.boxOrigin + vector * sphere.boxScale, Color{col.x, col.y, col.z,1.0f}, sphere.pointSize );
		}
	}
	visualisePointsSprites(visPoints, true);
	visualisePointsSprites(visAngles, true);
	visualiseLines(visLines, 2);
}
