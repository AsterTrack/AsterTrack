
/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include "ui/system/vis.hpp"
#include "ui/gl/visualisation.hpp"

/* Functions */

int handleSelectBounds(View3D &view3D, ImGuiKey select, ImGuiKey abort)
{
	auto &io = ImGui::GetIO();

	int acceptSelectBounds = 0;
	if (view3D.mouseIn && ImGui::IsKeyPressed(select, false) && !ImGui::IsKeyDown(abort))
	{ // Start bounded selection
		view3D.selectingBounded = io.KeyCtrl? 3 : (io.KeyShift? 2 : 1);
		view3D.selectMouseStart = view3D.mousePos;
	}
	if (view3D.selectingBounded && ImGui::IsKeyReleased(select))
	{ // Apply bounded selection (even if released outside)
		if (!view3D.selectBounds.center().hasNaN())
		{ // Too small bounds will be NaN and should not be regarded as a bounded selection
			acceptSelectBounds = view3D.selectingBounded;
			cancelClickReg(view3D); // Override any other click registration
		}
		view3D.selectingBounded = 0;
	}
	if (view3D.selectingBounded && ImGui::IsKeyDown(abort))
	{ // Abort without modifying selection
		cancelClickReg(view3D); // Override any other click registration
		view3D.selectingBounded = 0;
	}
	if (view3D.selectingBounded && (view3D.mousePos - view3D.selectMouseStart).norm() > 5*PixelSize)
	{
		view3D.selectBounds = Bounds2f(view3D.selectMouseStart, Eigen::Vector2f::Zero());
		view3D.selectBounds.include(view3D.mousePos);
	}
	else view3D.selectBounds = Bounds2f(Eigen::Vector2f::Constant(NAN), Eigen::Vector2f::Constant(NAN));

	return acceptSelectBounds;
}

template<typename T>
bool applyBoundedSelection(int selectBounds, std::set<T> &selected, std::set<T> &bounded)
{
	if (selectBounds == 0 || bounded.empty())
		return false;

	if (selectBounds == 1) // Normal
		selected = std::move(bounded);
	else if (selectBounds == 2) // Shift
		for (int id : bounded)
			selected.insert(id);
	else if (selectBounds == 3) // Ctrl
		for (int id : bounded)
			selected.erase(id);

	return true;
}

template bool applyBoundedSelection(int selectBounds, std::set<uint32_t> &selected, std::set<uint32_t> &bounded);

template<typename T>
bool multiSelection(View3D &view3D, ImGuiKey key, char source, int priority, int priorityDeselect, std::set<T> &selected, T &hovered, T none)
{
	auto &io = ImGui::GetIO();

	if (hovered == none)
	{
		if (!io.KeyShift && !io.KeyCtrl && clickRegister(view3D, { key, source, none, priorityDeselect }))
		{ // Clear selection if clicked on nothing
			selected.clear();
		}
		return false;
	}

	if (!clickRegister(view3D, { key, source, hovered, priority }))
		return false;

	if (io.KeyCtrl)
	{ // Individual toggle
		if (selected.contains(hovered))
			selected.erase(hovered);
		else
			selected.insert(hovered);
	}
	else if (io.KeyShift)
	{ // Append to
		selected.insert(hovered);
	}
	else
	{ // Contextual toggle
		bool off = selected.size() == 1 && selected.contains(hovered);
		if (!selected.empty()) selected.clear();
		if (!off) selected.insert(hovered);
	}
	return true;
}

template bool multiSelection(View3D &view3D, ImGuiKey key, char source, int priority, int priorityDeselect, std::set<uint32_t> &selected, uint32_t &hovered, uint32_t none);

bool clickRegister(View3D &view3D, View3D::Clickable clickable)
{ // Chosing to do own click processing on everything in 3D View, could instead implement it more in line with Dear ImGui
	if (view3D.mouseIn && ImGui::IsKeyReleased(clickable.key) && view3D.clickReg.source == clickable.source && view3D.clickReg.id == clickable.id)
	{
		LOG(LGUI, LDebug, "Registered click in 3D View on %c %ld", clickable.source, clickable.id);
		view3D.clickReg = {};
		return true;
	}
	if (view3D.mouseIn && ImGui::IsKeyPressed(clickable.key, false))
	{
		if (view3D.clickReg.source != 0)
			LOG(LGUI, LDebug, "Click in 3D View on %c %ld is competing with %c %ld, priorities %d and %d",
				clickable.source, clickable.id, view3D.clickReg.source, view3D.clickReg.id, clickable.priority, view3D.clickReg.priority);
		if (view3D.clickReg.priority < clickable.priority)
		{
			LOG(LGUI, LDebug, "Starting click in 3D View on %c %ld!", clickable.source, clickable.id);
			view3D.clickReg = clickable;
		}
	}
	return false;
}

void cancelClickReg(View3D &view3D)
{
	if (view3D.clickReg.source != 0)
	{
		LOG(LGUI, LDebug, "Cancelling click in 3D view on %c %ld", view3D.clickReg.source, view3D.clickReg.id);
		view3D.clickReg = {};
	}
}

CircleHit sphereHitProjection(const Eigen::Isometry3f &view, float fInv, Eigen::Vector3f pos, float size)
{
	Eigen::Isometry3f vInv = view.inverse();
	Eigen::Vector3f viewUpAxis = view.matrix().col(2).head<3>().cast<float>();
	Eigen::Vector3f sideVec = (pos - view.translation().cast<float>()).cross(viewUpAxis);
	Eigen::Vector3f sidePos = pos + (size/2 * sideVec.normalized());
	Eigen::Vector2f pSide = (vInv * sidePos).hnormalized() / fInv;
	Eigen::Vector2f pCenter = (vInv * pos).hnormalized() / fInv;
	float sizeSq = (pCenter - pSide).squaredNorm() * 2*2;
	return { pCenter, sizeSq };
}

int probePointCloudPos2D(const std::vector<VisPoint> &points, Eigen::Isometry3f view, float fInv, Eigen::Vector2f pos)
{
	Eigen::Isometry3f vInv = view.inverse();
	float selHaloSq = 8*8 * PixelSize*PixelSize;
	// Enter valid points with their distance
	thread_local std::vector<std::pair<int, float>> order;
	order.clear();
	for (int i = 0; i < points.size(); i++)
	{
		auto &pt = points[i];
		if (pt.color.a == 0) continue;
		float dist = (vInv * pt.pos).z();
		order.emplace_back(i, dist - pt.size);
	}
	// Sort front-to-back
	std::sort(order.begin(), order.end(), 
		[&](auto &a, auto &b) { return a.second < b.second; });
	// Find frontmost circle hit (not proper sphere raycasting but whatever)
	for (int i = 0; i < order.size(); i++)
	{
		auto &pt = points[order[i].first];
		auto circle = sphereHitProjection(view, fInv, pt.pos, pt.size);
		float distSq = (circle.pos - pos).squaredNorm();
		if (distSq < circle.sizeSq+selHaloSq)
			return order[i].first;
	}
	return -1;
}

std::vector<int> probePointCloudBounds2D(const std::vector<VisPoint> &points, Eigen::Isometry3f view, float fInv, Bounds2f bounds)
{
    std::vector<int> selection;
	Eigen::Isometry3f vInv = view.inverse();
	float selHalo = 8 * PixelSize;
	// Enter valid points with their distance
	thread_local std::vector<std::pair<int, float>> order;
	order.clear();
	for (int i = 0; i < points.size(); i++)
	{
		auto &pt = points[i];
		if (pt.color.a == 0) continue;
		float dist = (vInv * pt.pos).z();
		order.emplace_back(i, dist - pt.size);
	}
	// Sort front-to-back
	std::sort(order.begin(), order.end(), 
		[&](auto &a, auto &b) { return a.second < b.second; });
	// Find frontmost circle hit (not proper sphere raycasting but whatever)
	for (int i = 0; i < order.size(); i++)
	{
		auto &pt = points[order[i].first];
		auto circle = sphereHitProjection(view, fInv, pt.pos, pt.size);
		if (bounds.extendedBy(std::sqrt(circle.sizeSq)+selHalo).includes(circle.pos))
			selection.emplace_back(order[i].first);
	}
	return selection;
}
