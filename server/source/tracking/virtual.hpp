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

#ifndef VIRTUAL_TRACKER_H
#define VIRTUAL_TRACKER_H

#include "pipeline/parameters.hpp"

#include "util/eigendef.hpp"

enum class VirtualTrackerType
{
	STATIC
};

struct TrackerVirtualConfig
{
	VirtualTrackerType type;

	// IDs of actual trackers that define this tracker
	std::vector<int> ids;

	// These are currently only calibrated and stored per session
	// If they will ever be stored, they need to be kept in sync with ids
	std::vector<Eigen::Vector3f> offsetPos;
	std::vector<Eigen::Quaternionf> offsetRot;
};

#endif // VIRTUAL_TRACKER_H