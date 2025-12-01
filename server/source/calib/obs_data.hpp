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

#ifndef OBS_DATA_H
#define OBS_DATA_H

#include "util/eigendef.hpp"

#include "util/blocked_vector.hpp"
#include <list>
#include <map>

struct ObsPointSample
{
	uint16_t camera;
	Eigen::Vector2f point;
};
struct ObsPoint
{
	uint32_t marker, frame;
	std::vector<ObsPointSample> samples;
	float error = NAN;
	bool outlier = false;
};
struct ObsPointData
{
	BlockedVector<ObsPoint, 128> points;
	int totalSamples = 0; // Temp value summing all points::observations::size()
	int outlierPoints = 0, outlierSamples = 0; // Temp value of all points marked as outlier
};

struct ObsTargetSample
{
	uint32_t marker;
	uint16_t camera;
	Eigen::Vector2f point;
};
struct ObsTargetFrame
{
	uint32_t frame;
	std::vector<ObsTargetSample> samples;
	Eigen::Isometry3f pose;
	float error = NAN;
};
struct ObsTarget
{
	int trackerID;
	std::vector<ObsTargetFrame> frames;
	std::map<int,int> markerMap;
	std::vector<Eigen::Vector3f> markers;
	int totalSamples = 0; // Temp value summing all frames::samples::size()
	int outlierSamples = 0; // Temp value of all outliers that have been removed (not included in totalSamples)
};

struct ObsData
{
	ObsPointData points = {};
	std::vector<ObsTarget> targets;

	inline std::size_t getValidSamples() const
	{ // NOT equal to the total samples used in optimisation due to differences in how point & target outliers are handled
		std::size_t samples = points.totalSamples - points.outlierSamples;
		for (auto &target : targets)
			samples += target.totalSamples;
		return samples;
	}
};

#endif // OBS_DATA_H