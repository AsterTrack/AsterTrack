
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

#include "ui/system/vis.hpp"
#include "ui/gl/visualisation.hpp"

/* Functions */

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
		Eigen::Vector3f viewUpAxis = view.matrix().col(2).head<3>().cast<float>();
		Eigen::Vector3f sideVec = (pt.pos - view.translation().cast<float>()).cross(viewUpAxis);
		Eigen::Vector3f sidePos = pt.pos + (pt.size/2 * sideVec.normalized());
		Eigen::Vector2f pSide = (vInv * sidePos).hnormalized() / fInv;
		Eigen::Vector2f pCenter = (vInv * pt.pos).hnormalized() / fInv;
		float sizeSq = (pCenter - pSide).squaredNorm() * 2*2;
		float distSq = (pCenter - pos).squaredNorm();
		if (distSq < sizeSq+selHaloSq)
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
		Eigen::Vector3f viewUpAxis = view.matrix().col(2).head<3>().cast<float>();
		Eigen::Vector3f sideVec = (pt.pos - view.translation().cast<float>()).cross(viewUpAxis);
		Eigen::Vector3f sidePos = pt.pos + (pt.size/2 * sideVec.normalized());
		Eigen::Vector2f pSide = (vInv * sidePos).hnormalized() / fInv;
		Eigen::Vector2f pCenter = (vInv * pt.pos).hnormalized() / fInv;
		float size = (pCenter - pSide).norm() * 2;
		if (bounds.extendedBy(size+selHalo).includes(pCenter))
			selection.emplace_back(order[i].first);
	}
	return selection;
}