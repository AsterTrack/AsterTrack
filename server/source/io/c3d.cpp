/**
AsterTrack Optical Tracking System
Copyright (C)  2026 Seneral <seneral@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "c3d.hpp"

#include "util/log.hpp"

#include "ezc3d/ezc3d.h"
#include "ezc3d/Parameters.h"
#include "ezc3d/Frame.h"

#include <map>
#include <filesystem>

std::string c3d_exportFile(std::string filePath, FrameNum frameFirst, FrameNum frameLast,
	int baseRate, const TrackingRecord &record, const std::vector<TrackerConfig> &trackerConfig)
{
	TimePoint_t t0 = sclock::now();

	auto frames = record.frames.getView();
	auto frameBegin = frames.pos(std::max(frames.beginIndex(), frameFirst));
	auto frameEnd = frames.pos(std::min(frames.endIndex(), frameLast+1));

	// Collect relevant markers and trackers
	std::map<int,int> markers, trackers;
	for (auto frameIt = frameBegin; frameIt < frameEnd; frameIt++)
	{
		auto &frame = *frameIt;
		if (!frame || !frame->finishedProcessing) continue;
		for (auto &marker : frame->markers3D)
		{
			if (marker.id == 0) continue; // Not labelled
			markers[marker.id]++;
		}
		for (auto &tracker : frame->trackers)
		{
			if (!tracker.result.isTracked()) continue;
			trackers[tracker.id]++;
		}
	}

	// Trim markers to actually save
	std::vector<std::string> markerLabels;
	markerLabels.reserve(markers.size());
	for (auto markerIt = markers.begin(); markerIt != markers.end();)
	{
		if (markerIt->second > 10)
		{
			markerIt->second = markerLabels.size();
			markerLabels.push_back(asprintf_s("Marker %d", markerIt->first));
			markerIt++;
		}
		else markerIt = markers.erase(markerIt);
	}

	// Trim trackers to actually save
	std::vector<std::string> trackerLabels;
	trackerLabels.reserve(trackers.size());
	for (auto trackerIt = trackers.begin(); trackerIt != trackers.end();)
	{
		auto config = std::find_if(trackerConfig.begin(), trackerConfig.end(),
			[&](auto &cfg){ return cfg.id == trackerIt->first; });
		if (trackerIt->second > 10 && config != trackerConfig.end())
		{
			trackerIt->second = trackerLabels.size();
			trackerIt++;
			trackerLabels.push_back(config->label);
		}
		else trackerIt = trackers.erase(trackerIt);
	}

	// TODO: Maybe store IMU data?
	std::vector<std::string> analogLabels;

	// Start filling C3D
	ezc3d::c3d c3d;

	// First frame AND last frame (1 based) need to fit in uint16_t...
	if (frameEnd.index() < (1<<16) - 2)
		c3d.setFirstFrame(frameBegin.index());

	c3d.parameters().setMandatoryParametersForSpecialGroup("ROTATION");

	c3d.addPoint(markerLabels);
	c3d.addAnalog(analogLabels);
	c3d.addPose(trackerLabels);

	ezc3d::ParametersNS::GroupNS::Parameter opticalRate("RATE");
	opticalRate.set(std::vector<double>() = {(double)baseRate}, {1});
	c3d.parameter("POINT", opticalRate);
	c3d.parameter("ROTATION", opticalRate);

	/* ezc3d::ParametersNS::GroupNS::Parameter analogRate("RATE");
	analogRate.set(std::vector<double>() = {1000}, {1});
	c3d.parameter("ANALOG", analogRate); */

	c3d.frames().resize(frameEnd.index() - frameBegin.index());
	for (auto frameIt = frameBegin; frameIt < frameEnd; frameIt++)
	{
		auto &frame = *frameIt;
		if (!frame || !frame->finishedProcessing) continue;

		ezc3d::DataNS::Frame &frameData = c3d.frames()[frameIt.index() - frameBegin.index()];

		frameData.points->points.resize(markerLabels.size());
		for (auto &marker : frame->markers3D)
		{
			if (marker.id == 0) continue; // Not labelled
			auto index = markers.find(marker.id);
			if (index == markers.end()) continue; // Culled
			auto &point = frameData.points->points[index->second];
			Eigen::Vector3f pos = marker.pos * 1000.0f;
			point.set(pos.x(), pos.y(), pos.z(), marker.uncertainty3D); // or RMSE, error2D?
		}

		auto &poses = frameData.rotations->subframes.emplace_back();
		poses.rotations.resize(trackerLabels.size());
		for (auto &tracker : frame->trackers)
		{
			if (!tracker.result.isTracked()) continue;
			auto index = trackers.find(tracker.id);
			if (index == trackers.end()) continue; // Culled
			auto p = tracker.pose.filtered.matrix();
			poses.rotations[index->second] = ezc3d::DataNS::RotationNS::Rotation(
				p(0, 0), p(1, 0), p(2, 0), p(3, 0) * 1000.0f,
				p(0, 1), p(1, 1), p(2, 1), p(3, 1) * 1000.0f,
				p(0, 2), p(1, 2), p(2, 2), p(3, 2) * 1000.0f,
				p(0, 3), p(1, 3), p(2, 3), p(3, 3),
				tracker.pose.filteredCov.determinant()
			);
		}

		/* frameData.analogs->subframes.resize(c3d.header().nbAnalogByFrame());
		for (size_t i=0; i < frameData.analogs->subframes.size(); i++)
		{
			auto &subframe = frameData.analogs->subframes[i];
			subframe.channels.resize(c3d.header().nbAnalogs());
			for (size_t j = 0; j < subframe.channels.size(); j++)
			{
				subframe.channels[j].data = j+1;
			}
		} */
	}

	// Finalise Frames and validate
	c3d.finaliseFrames(true);

	// Finalize the internal structure
	c3d.updateHeader();

	TimePoint_t t1 = sclock::now();

	c3d.write(filePath);

	TimePoint_t t2 = sclock::now();

	LOG(LIO, LInfo, "Exported C3D with %d frames, %d markers and %d trackers to '%s'. Filling C3D took %.2fms, writing %.2fms!",
		(int)(frameEnd.index() - frameBegin.index()), (int)markers.size(), (int)trackers.size(), filePath.c_str(), dtMS(t0, t1), dtMS(t1, t2));

	return asprintf_s("Exported C3D with %d frames, %d markers and %d trackers to:\n'%s'",
		(int)(frameEnd.index() - frameBegin.index()), (int)markers.size(), (int)trackers.size(),
		std::filesystem::absolute(filePath).c_str());
}