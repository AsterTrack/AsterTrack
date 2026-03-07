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

#include "obs_data.inl"

#include "point/sequence_data.inl"
#include "point/sequence_data.hpp"
#include "point/triangulation.hpp"
#include "target/tracking2D.hpp"

/**
 * Adds triangulatable points in the frame range to the point database.
 */
void addTriangulatableObservations(ObsPointData &data, const std::vector<MarkerSequences> &observations, FrameNum frameStart, FrameNum frameEnd)
{
	// TODO: Ability to pass only relevant markers
	int obsAdded = 0;
	for (uint32_t m = 0; m < observations.size(); m++)
	{
		const MarkerSequences &marker = observations[m];
		std::map<FrameNum, std::size_t> frameMap;
		std::size_t frameCount = getTriangulationFrameMap(marker, frameMap, frameStart, frameEnd);
		if (frameCount == 0)
			continue;

		// Continuous storage frameMap maps to (could directly append to data.points)
		std::vector<ObsPoint> triFrames;
		triFrames.resize(frameCount);

		handleMappedSequences(marker, frameMap,
			[m, &triFrames](const PointSequence &seq, int c, int s, int seqOffset, int start, int length)
		{
			// Write frame mapped observations to point database
			for (int p = 0; p < length; p++)
			{
				triFrames[start+p].frame = (uint32_t)(seq.startFrame + seqOffset + p);
				triFrames[start+p].samples.push_back({ (uint16_t)c, seq.rawPoints[seqOffset+p] });
			}
		});

		for (int i = 0; i < frameCount; i++)
		{
			assert(triFrames[i].samples.size() > 1);
			obsAdded += triFrames[i].samples.size();
			data.points.push(std::move(triFrames[i]));
		}
	}
	data.totalSamples += obsAdded;
}

/**
 * Fill target observation database with observations from existing frames and markers (specified by target.frames and target.markerMap)
 */
void updateTargetObservations(ObsTarget &target, const std::vector<MarkerSequences> &observations)
{
	if (target.markerMap.empty() || target.frames.empty())
	{
		target.totalSamples = 0;
		target.frames.clear();
		return;
	}

	FrameNum blockBegin = target.frames.back().frame+1, blockEnd = target.frames.front().frame;
	for (auto &frame : target.frames)
	{
		if (frame.tracked) continue;
		frame.samples.clear();
		blockBegin = std::min(blockBegin, frame.frame);
		blockEnd = std::max(blockEnd, frame.frame);
	}
	if (blockBegin >= blockEnd)
		return;

	for (auto &map : target.markerMap)
	{
		int m = map.first;
		if (m < 0) continue;
		if (observations.size() <= m) break;
		const MarkerSequences &marker = observations[m];

		for (int c = 0; c < marker.cameras.size(); c++)
		{
			const CameraSequences &camera = marker.cameras[c];
			if (camera.sequences.empty()) continue;

			int i = 0;
			for (auto cur = camera.upper(blockBegin); cur != camera.upper(blockEnd); cur++)
			{
				while (target.frames[i].frame < cur.frame()) i++;
				if (target.frames[i].frame != cur.frame()) continue;
				if (target.frames[i].tracked) continue;
				ObsTargetSample sample;
				sample.marker = m;
				sample.camera = (uint16_t)c;
				sample.point = cur.raw();
				target.frames[i].samples.push_back(sample);
			}
		}
	}

	target.totalSamples = 0;
	for (auto &frame : target.frames)
		target.totalSamples += frame.samples.size();
}

/**
 * Fill target observation database with observations from given frames and markers (specified by target.markerMap)
 */
void addTargetObservations(ObsTarget &target, const std::vector<MarkerSequences> &observations, const std::vector<FrameNum> frames)
{
	if (target.markerMap.empty())
		return;

	target.frames.clear();
	target.frames.resize(frames.size());
	for (int f = 0; f < frames.size(); f++)
		target.frames[f].frame = (FrameNum)frames[f];
	updateTargetObservations(target, observations);
}

/**
 * Adds observations of a target for the current frame
 * Used for iterative addition of tracked targets
 */
void addTrackedTarget(ObsData &data, const TargetMatch2D &observations, const std::vector<std::vector<Eigen::Vector2f>*> &points2D, FrameNum frame);

/**
 * Adds triangulatable points of the current frame to the point database.
 */
void addTriangulations(ObsData &data, const std::vector<TriangulatedPoint> &triPoints, FrameNum frame);