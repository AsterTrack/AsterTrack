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

#include "device/tracking_controller.hpp"
#include "device/tracking_camera.hpp"

#include "imgui/imgui_onDemand.hpp"

void InterfaceState::UpdateCameras()
{
	ServerState &state = GetState();
	std::shared_lock dev_lock(state.deviceAccessMutex); // cameras

	// Check for change in connecting cameras
	int connecting = 0;
	for (auto &controller : state.controllers)
		connecting += controller->newCamerasConnecting;
	if (connecting != camerasConnecting)
	{
		camerasConnecting = connecting;
		cameraGridDirty = true;
	}

	// Check for new cameras added
	for (auto &cam : state.cameras)
	{
		if (cameraViews.find(cam->id) == cameraViews.end())
		{ // Add new camera
			CameraView view = {};
			view.camera = cam; // new shared_ptr
			if (cam->pipeline)
				view.ImGuiTitle = asprintf_s("Camera %d (%d)", cam->id, cam->pipeline->index);
			else
				view.ImGuiTitle = asprintf_s("Camera %d", cam->id);
			cameraViews.insert({ cam->id, view });
			cameraGridDirty = true;
		}
	}

	// Check for existing cameras removed
	for (auto view = cameraViews.begin(); view != cameraViews.end();)
	{
		auto cam = std::find_if(state.cameras.begin(), state.cameras.end(), [&view](const auto &c) { return c->id == view->first; });
		if (cam == state.cameras.end())
		{ // Remove old camera view
			if (view->second.isDetached)
			{
				ImGuiWindow* imguiWindow = ImGui::FindWindowByName(view->second.ImGuiTitle.c_str());
				if (imguiWindow)
					ImGui::DockContextProcessUndockWindow(ImGui::GetCurrentContext(), imguiWindow, 0);
				ImGui::SetTabItemClosed(view->second.ImGuiTitle.c_str());
			}
			view = cameraViews.erase(view);
			cameraGridDirty = true;
		}
		else view++;
	}

	if (cameraGridDirty)
		RequestUpdates(2);
}

void InterfaceState::UpdateCameraViews(InterfaceWindow &window)
{
	// Grid Window Padding
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0,0));
	// Padding withing camera views is normal WindowPadding

	static bool horizontalScrolling = false;
	ImGuiWindowFlags flags = 0;
	if (horizontalScrolling)
		flags |= ImGuiWindowFlags_HorizontalScrollbar;
	if (!ImGui::Begin(window.title.c_str(), &window.open, flags))
	{
		ImGui::PopStyleVar();
		ImGui::End();
		return;
	}
	ImGui::PopStyleVar(); // Grid Window Padding

	// Spacing between camera views
	ImVec2 gridSpacing(2,2);
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, gridSpacing);

	static ImVec2 oldSize;
	ImVec2 newSize = ImGui::GetCurrentWindowRead()->InnerRect.GetSize();
	if (cameraGridDirty || newSize != oldSize)
	{
		oldSize = newSize;
		horizontalScrolling = UpdateCameraGrid();
		cameraGridDirty = false;
	}

	bool takeKeyInput = ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows) || ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
	if (takeKeyInput)
		ImGui::SetNextFrameWantCaptureKeyboard(true);
	if (takeKeyInput) // !io.WantCaptureKeyboard
	{ // Process input
		if (ImGui::IsKeyPressed(ImGuiKey_Z))
		{
			for (auto &view : cameraViews)
			{ // Automatically zoom into "interesting" targets depending on phase
				view.second.vis.view.autoZoom = true;
				view.second.vis.view.zoom = 5.0f;
			}
			RequestUpdates();
		}
		if (ImGui::IsKeyPressed(ImGuiKey_U))
		{
			for (auto &view : cameraViews)
			{ // Force unzoomed state
				view.second.vis.view.autoZoom = false;
				view.second.vis.view.zoom = 1.0f;
				view.second.vis.view.center.setZero();
			}
			RequestUpdates();
		}
	}

	// TODO: Add some kind of panel to control visualisation settings for camera view
	// View3D has it's own integrated side panel
	// CameraViews currently only have a separate visualisation window
	// And each CameraView has some controls, which are usually more device focused, not just visualisations
	// Make sure all these are placed properly and easily and obviously accessible

	for (auto &viewIt : cameraViews)
	{
		CameraView &view = viewIt.second;
		if (view.isDetached) continue;
		if (view.gridX != 0) ImGui::SameLine(0);
		if (ImGui::BeginChild(view.ImGuiTitle.c_str(), view.size, true, ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
		{
			ImGui::PopStyleVar();
			UpdateCameraUI(view);
			view.resized = false;
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, gridSpacing);
		}
		ImGui::EndChild();
	}
	for (int i = 0; i < camerasConnecting; i++)
	{
		int gridIndex = cameraViews.size() + i;
		int gridX = gridIndex % gridColumns;
		int gridY = gridIndex / gridColumns;
		if (gridX != 0) ImGui::SameLine(0);
		ImGui::PushID(gridIndex);
		if (ImGui::BeginChild("Connecting Camera", gridCellSize, true, ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
		{
			ImGui::PopStyleVar();
			ShowConnectingCameraUI(i);
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, gridSpacing);
		}
		ImGui::EndChild();
		ImGui::PopID();
	}

	ImGui::PopStyleVar(); // Spacing between camera views

	ImGui::End();
}

static void ConstrainWindowAspect(ImGuiSizeCallbackData* data)
{
	float aspect = *(float*)data->UserData;
	float base = std::max(400.0f, (data->DesiredSize.x+data->DesiredSize.y/aspect)/2);
	// TODO: Fix slow drifting while resizing detached camera view, not sure why it happens
	//if (std::abs(data->CurrentSize.x-base) > 0.5f || std::abs(data->CurrentSize.y-base*aspect) > 0.5f)
	data->DesiredSize = ImVec2(base, base * aspect);
}

void InterfaceState::UpdateDetachedCameraView(CameraView &view)
{
	assert(view.isDetached);
	if (view.detachedIndex < 0)
	{ // Assign new index, as low as possible, as it will re-use the layout
		int index = 0;
		while (index < 100)
		{
			bool free = true;
			for (auto &viewIt : cameraViews)
			{
				if (viewIt.second.detachedIndex == index)
				{
					free = false;
					index++;
					break;
				}
			}
			if (free)
			{
				view.detachedIndex = index;
				break;
			}
		}
	}

	float aspect = (float)view.camera->pipeline->mode.aspect;
	ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(FLT_MAX, FLT_MAX),
		ConstrainWindowAspect, (void*)&aspect);

	std::string label = asprintf_s("%s###Detached Camera %d", view.ImGuiTitle.c_str(), view.detachedIndex);
	if (ImGui::Begin(label.c_str(), &view.isDetached, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
	{
		auto newSize = ImVec2(ImGui::GetWindowWidth(), ImGui::GetWindowHeight());
		view.resized = view.size != newSize;
		view.size = newSize;
		UpdateCameraUI(view);
		view.resized = false;
	}
	if (!view.isDetached)
		cameraGridDirty = true;
	ImGui::End();
}

bool InterfaceState::UpdateCameraGrid()
{
	int gridCnt = 0;
	for (auto &view : cameraViews)
		if (!view.second.isDetached)
			gridCnt++;
	gridCnt += camerasConnecting;

	if (gridCnt == 0)
		return false;

	// TODO: Update camera panel to support different camera aspects
	// Would probably group by aspect and/or take biggest aspect (in a line) for the grid and ecompass others
	float viewAspect = 0.625f; // view.second.camera.mode.aspect
	

	// Try to fit without scrollbar with minimum size
	Eigen::Vector2f minView(300, 300*viewAspect);

	// TODO: Fix oscillations due to scrollbar in some rare but reproducible situations
	// Might need to make scrollbars an explicit toggle at some threshold independent from the scrollbar itself
	// TODO: Vertical scrolling doesn't work, just scales down, unlike horizontal limits where it will scroll
	Eigen::Vector2f win(
		ImGui::GetCurrentWindowRead()->InnerRect.GetWidth(),
		ImGui::GetCurrentWindowRead()->InnerRect.GetHeight()
	);

	Eigen::Vector2f spacing(
		ImGui::GetStyle().ItemSpacing.x,
		ImGui::GetStyle().ItemSpacing.y
	);

	Eigen::Vector2f view;
	Eigen::Vector2i grid;
	auto CalculateOptimalGrid = [&]()
	{
		// Get initial estimate with width fully used by one view
		view.x() = std::min((float)win.x(), win.y()/viewAspect);
		view.y() = view.x() * viewAspect;
		grid.x() = std::max(1, (int)std::floor((float)win.x()/view.x()));
		grid.y() = (int)std::ceil((float)gridCnt/grid.x());

		Eigen::Vector2f gridSpacing(
			(grid.x()-1) * spacing.x(),
			(grid.y()-1) * spacing.y()
		);

		// Check if initial settings already produce no scrollbar
		float totalHeight = grid.y() * view.y() + gridSpacing.y();
		if (totalHeight < win.y())
			return;

		while (true)
		{
			// Try new grid with increased grid.x() to reduce height
			Eigen::Vector2i newGrid;
			newGrid.x() = grid.x()+1;
			newGrid.y() = (int)std::ceil((float)gridCnt/newGrid.x());

			Eigen::Vector2f newGridSpacing(
				(newGrid.x()-1) * spacing.x(),
				(newGrid.y()-1) * spacing.y()
			);

			// Check if resulting grid is valid
			Eigen::Vector2f newGridView;
			newGridView.x() = ((float)win.x() - newGridSpacing.x()) / newGrid.x();
			newGridView.y() = newGridView.x() * viewAspect;
			bool newGridValid = newGridView.x() > minView.x();

			float newGridTotalHeight = newGrid.y() * newGridView.y() + newGridSpacing.y();
			if (newGridValid && newGridTotalHeight > win.y())
			{ // Resulting new grid height is still producing scrollbar, continue with it to further improve
				grid = newGrid;
				view = newGridView;
				gridSpacing = newGridSpacing;
				//LOG(LGUI, LDebug, "New Grid %dx%d is valid and has a height %f > window height %d! Continuing!\n", newGrid.x(), newGrid.y(), newGrid.y() * newGridView.y(), win.y());
				continue;
			}

			// Assuming old grid, optimise to produce no scrollbar
			float optViewY = (win.y()-gridSpacing.y())/grid.y();

			if (newGridValid && newGridView.y() > optViewY)
			{ // Adjusting grid is better
				//LOG(LGUI, LDebug, "New Grid %dx%d with height %f < window height %d, has higher view height %f than old grid with optimised height %f! Done!\n", newGrid.x(), newGrid.y(), newGrid.y() * newGridView.y(), win.y(), optViewY, newGridView.y());
				grid = newGrid;
				view = newGridView;
				gridSpacing = newGridSpacing;
			}
			else if (optViewY > minView.y())
			{
				view.y() = optViewY;
				view.x() = optViewY/viewAspect;
				//LOG(LGUI, LDebug, "New Grid %dx%d had a view width %f < min %f, but scaling to fit resulting in optimised height %f! Done!\n", newGrid.x(), newGrid.y(), newGridView.x(), minViewWidth, optViewY);
			}
			else
			{ // Can't fit perfectly anyway, so just keep maximum grid
				//LOG(LGUI, LDebug, "New Grid %dx%d had a view height %f < min %f, and can't scale old grid to fit without scrollbar (height %f < min %f), so keeping old grid! Done!\n", newGrid.x(), newGrid.y(), newGridView.y(), minViewHeight, optViewY, minViewHeight);
			}
			break;
		}
		//LOG(LGUI, LDebug, "Done with Grid %dx%d and view width %f > min %d, view height %f > min %d!\n", grid.x(), grid.y(), view.x(), minViewWidth, view.y(), minViewHeight);
	};

	// If window is wide, orient with horizontal scrollbar in mind instead. Just switch axis
	float winAspect = (float)win.y()/win.x();
	bool scrollbarHorizontal = winAspect < viewAspect;
	if (scrollbarHorizontal)
	{ // Horizontal scrollbar
		win.reverseInPlace();
		minView.reverseInPlace();
		spacing.reverseInPlace();
		viewAspect = 1.0f/viewAspect;
		CalculateOptimalGrid();
		grid.reverseInPlace();
		view.reverseInPlace();
	}
	else
	{ // Vertical scrollbar
		CalculateOptimalGrid();
	}

	// Update views with their new grid positions
	int g = 0;
	for (auto &viewIt : cameraViews)
	{
		if (viewIt.second.isDetached) continue;
		viewIt.second.gridX = g%grid.x();
		viewIt.second.gridY = g/grid.x();
		viewIt.second.size.x = view.x();
		viewIt.second.size.y = view.y();
		viewIt.second.resized = true;
		g++;
	}
	gridColumns = grid.x();
	gridCellSize = ImVec2(view.x(), view.y());

	return scrollbarHorizontal;
}