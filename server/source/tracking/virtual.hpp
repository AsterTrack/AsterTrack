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

enum TrackerAxis
{
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,
	AXIS_MASK = 0b11,
	AXIS_SIGN = 4,
	AXIS_XP = AXIS_X,
	AXIS_YP = AXIS_Y,
	AXIS_ZP = AXIS_Z,
	AXIS_XN = AXIS_X | AXIS_SIGN,
	AXIS_YN = AXIS_Y | AXIS_SIGN,
	AXIS_ZN = AXIS_Z | AXIS_SIGN,
};

struct TrackerVirtualConfig
{
	VirtualTrackerType type;

	// IDs of actual trackers that define this tracker
	std::vector<int> ids;

	// Centering configuration
	std::vector<float> centerWeights; // Influence of each sub-tracker on the position center (default 1)
	Eigen::Vector3f centerOffset = Eigen::Vector3f::Zero(); // Offset applied ontop of that

	// Alignment configuration
	int copyRotationFromTracker = 0; // Default tracker index 0, -1 is disabled
	// TODO: Current design of following alignment options is fixed, even though later ones may override earlier ones
	// Ideally it would be reorderable and/or even fit equally to multiple options (likely too complicated)
	// Also only one axis can be picked by each option but that should be fine
	// 2 copyAxis -> use copyRotationFromTracker, 2 of alignAxis might be useful, 2 of fitPlane or calibrateUp doesn't make sense at all
	struct AxisSource
	{
		int tracker;
		TrackerAxis axis;
	};
	struct
	{ // Copy/Average axis from specific axes of selected trackers
		std::vector<AxisSource> sources;
		TrackerAxis axis;
	} copyAxis;
	struct
	{ // Align axis to point from weighted center to selected tracker
		int tracker = -1;
		TrackerAxis axis;
	} alignAxis;
	// Future Ideas:
	/* struct
	{ // Fit plane to subtrackers (not masked) and use normal as axis
		// TODO: Need deterministic direction
		bool enable = false;
		std::vector<bool> mask;
		TrackerAxis axis;
	} fitPlane; */
	/* struct
	{ // Manually calibrate up axis via button in trackers UI
		// TODO: How would it interface with the pipeline?
		// How to store persistent rotation "offset" when none of the trackers is considered fixed and can be used as reference? 
		bool enable = false;
		TrackerAxis axis;
	} calibrateUp; */

	// These are currently only calibrated and stored per session
	// If they will ever be stored, they need to be kept in sync with ids
	std::vector<Eigen::Vector3f> offsetPos;
	std::vector<Eigen::Quaternionf> offsetRot;
};

#endif // VIRTUAL_TRACKER_H