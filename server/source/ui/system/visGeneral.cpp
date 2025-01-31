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
#include "ui/ui.hpp"
#include "ui/gl/visualisation.hpp"


/* Functions */

VisFrameLock VisualisationState::lockVisFrame(const PipelineState &pipeline, bool forceRealtime) const
{
	VisFrameLock snapshot = {};
	snapshot.target = lockVisTarget();
	snapshot.frames = pipeline.frameRecords.getView();
	if (snapshot.frames.empty())
		return snapshot;
	snapshot.frameIt = snapshot.frames.pos(std::max((long)0, std::min((long)snapshot.frames.endIndex()-1, pipeline.frameNum.load())));
	if (!snapshot.frameIt.accessible()) snapshot.frameIt = std::prev(snapshot.frames.end());
	if (snapshot.target.hasObs() && !forceRealtime)
	{ // Visualise selected past frame
		auto frame = snapshot.target.targetObs->frames[snapshot.target.frameIdx].frame;
		if (frame >= snapshot.frames.endIndex())
		{
			snapshot.hasFrame = snapshot.isRealtimeFrame = false;
			return snapshot;
		}
		snapshot.frameIt = snapshot.frames.pos(frame);
		snapshot.isRealtimeFrame = false;
	}
	else
	{ // Visualise most recent frame
		for (int i = 0; i < 50; i++)
		{ // Find latest processed frame
			if (*snapshot.frameIt && snapshot.frameIt->get()->finishedProcessing) break;
			if (snapshot.frameIt == snapshot.frames.begin()) break;
			snapshot.frameIt--;
		}
	}
	snapshot.hasFrame = snapshot.frameIt.accessible() && *snapshot.frameIt && snapshot.frameIt->get()->finishedProcessing;
	return snapshot;
}

Eigen::Vector3f VisualisationState::getPreferredTarget(const VisFrameLock &visFrame) const
{ // Try to find a 3D target to zoom into
	if (visFrame.target)
	{
		Eigen::Isometry3f tgtPose = visFrame.target.getPose();
		if (target.focusOnMarkerSelection)
		{
			Eigen::Vector3f target3D = Eigen::Vector3f::Zero();
			int highlightCnt = 0;
			for (int m = 0; m < visFrame.target.targetTemplate->markers.size() && m < target.markerSelect.size(); m++)
			{
				if (!target.markerSelect[m]) continue;
				target3D += visFrame.target.targetTemplate->markers[m].pos;
				highlightCnt++;
			}
			if (highlightCnt > 0)
				return tgtPose * (target3D/highlightCnt);
			else
				return tgtPose.translation();
		}
		else
		{
			return tgtPose.translation();
		}
	}

	if (visFrame && !visFrame.frameIt->get()->tracking.targets.empty())
	{
		auto &trackers = visFrame.frameIt->get()->tracking.targets;
		if (tracking.focusedTargetID >= 0)
		{
			auto track = std::find_if(trackers.begin(), trackers.end(),
				[&](auto &tgt){ return tgt.id == tracking.focusedTargetID; });
			if (track != trackers.end())
				return track->poseFiltered.translation();
		}
		return trackers.front().poseFiltered.translation();
	}

	return Eigen::Vector3f::Constant(NAN);
}

void visualiseBlobs2D(const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, Color color, float sizeFactor, float crossSize)
{
	if (points2D.size() != properties.size())
		return;

	// Sanitise sizes because it's technically unchecked input
	auto pointSizeRange = getPointSizeRange();

	thread_local std::vector<VisPoint> vertices;
	thread_local std::vector<std::pair<VisPoint, VisPoint>> crosses;
	vertices.resize(points2D.size());
	if (crossSize > 0.0f)
		crosses.resize(points2D.size()*2);
	float alphaBase = std::min(color.a/2, 0.4f);
	float maxValue = 600.0f; // Technically up to 1000
	float depth = 1-0.9f;
	for (int i = 0; i < points2D.size(); i++)
	{
		VisPoint &vert = vertices[i];
		vert.pos.x() = points2D[i].x();
		vert.pos.y() = points2D[i].y();
		vert.pos.z() = depth;
		float value = std::min(1.0f, 0.4f + properties[i].value/maxValue*0.6f);
		float alpha = std::min(color.a, alphaBase + properties[i].value/maxValue*(color.a-alphaBase));
		vert.color = Color { color.r*value, color.g*value, color.b*value, alpha };
		vert.size = std::max(std::min(properties[i].size*sizeFactor, pointSizeRange[1]), pointSizeRange[0]);

		if (crossSize > 0.0f)
		{ // Draw cross at centers
			Eigen::Vector3f axisX = Eigen::Vector3f::UnitX()*crossSize, axisY = Eigen::Vector3f::UnitY()*crossSize;
			Color8 crossColor = vert.color;
			crossColor.a = std::min(255, (int)((alpha+0.1f)*255));
			crosses[i*2+0].first = { vert.pos - axisX, crossColor };
			crosses[i*2+0].second = { vert.pos + axisX, crossColor };
			crosses[i*2+1].first = { vert.pos - axisY, crossColor };
			crosses[i*2+1].second = { vert.pos + axisY, crossColor };
		}
	}

	visualisePointsSprites(vertices, true);

	if (crossSize > 0.0f)
	{ // Draw cross at centers
		visualiseLines(crosses, 2.0f);
	}
}

void updateErrorPointsVBO(unsigned int &VBO, const std::vector<Eigen::Vector3f> &pointErrors, Color colorA, Color colorB, Color colorO, float size, float depth)
{
	thread_local std::vector<VisPoint> vertices;
	vertices.clear();
	vertices.reserve(pointErrors.size());
	for (int i = 0; i < pointErrors.size(); i++)
	{
		VisPoint vert;
		vert.pos.x() = pointErrors[i].x();
		vert.pos.y() = pointErrors[i].y();
		vert.pos.z() = 1-depth;
		if (pointErrors[i].z() > 1.0f || pointErrors[i].z() < 0)
			vert.color = colorO;
		else
			vert.color = lerp(colorA, colorB, pointErrors[i].z());
		vert.size = size;
		vertices.push_back(vert);
	}

	updatePointsVBO(VBO, vertices);
}

void visualiseRays(const CameraCalib &emitter, const std::vector<Eigen::Vector2f> &points2D, Color8 color)
{
	if (points2D.empty()) return;
	thread_local std::vector<std::pair<VisPoint, VisPoint>> rayLines;
	rayLines.clear();
	for (const auto &pt : points2D)
	{
		Ray3f ray = castRay<float>(pt, emitter);
		rayLines.emplace_back(
			VisPoint{ ray.pos, color },
			VisPoint{ ray.pos + ray.dir * 1000, color }
		);
	}
	visualiseLines(rayLines, 0.5f);
}

void visualiseBounds3D(Bounds3f bounds, Eigen::Isometry3f pose)
{
	std::array<Eigen::Vector3f, 8> corners = {
		pose * Eigen::Vector3f(bounds.minX, bounds.minY, bounds.minZ),
		pose * Eigen::Vector3f(bounds.maxX, bounds.minY, bounds.minZ),
		pose * Eigen::Vector3f(bounds.maxX, bounds.maxY, bounds.minZ),
		pose * Eigen::Vector3f(bounds.minX, bounds.maxY, bounds.minZ),
		// Upper verts
		pose * Eigen::Vector3f(bounds.minX, bounds.minY, bounds.maxZ),
		pose * Eigen::Vector3f(bounds.maxX, bounds.minY, bounds.maxZ),
		pose * Eigen::Vector3f(bounds.maxX, bounds.maxY, bounds.maxZ),
		pose * Eigen::Vector3f(bounds.minX, bounds.maxY, bounds.maxZ)
	};
	Color8 col = Color{ 1, 1, 0, 1 };
	std::vector<std::pair<VisPoint, VisPoint>> lines = {
		// Lower edges
		{ { corners[0], col }, { corners[1], col } },
		{ { corners[1], col }, { corners[2], col } },
		{ { corners[2], col }, { corners[3], col } },
		{ { corners[3], col }, { corners[0], col } },
		// Upper edges
		{ { corners[4], col }, { corners[5], col } },
		{ { corners[5], col }, { corners[6], col } },
		{ { corners[6], col }, { corners[7], col } },
		{ { corners[7], col }, { corners[4], col } },
		// Connecting edges
		{ { corners[0], col }, { corners[4], col } },
		{ { corners[1], col }, { corners[5], col } },
		{ { corners[2], col }, { corners[6], col } },
		{ { corners[3], col }, { corners[7], col } },
	};
	visualiseLines(lines, 1.0f);
}

void visualiseBounds2D(Bounds2f bounds)
{
	float depth = 1-0.9f;
	std::array<Eigen::Vector3f, 4> corners = {
		Eigen::Vector3f(bounds.minX, bounds.minY, depth),
		Eigen::Vector3f(bounds.maxX, bounds.minY, depth),
		Eigen::Vector3f(bounds.maxX, bounds.maxY, depth),
		Eigen::Vector3f(bounds.minX, bounds.maxY, depth)
	};
	Color8 col = Color{ 1, 1, 0, 1 };
	std::vector<std::pair<VisPoint, VisPoint>> lines = {
		{ { corners[0], col }, { corners[1], col } },
		{ { corners[1], col }, { corners[2], col } },
		{ { corners[2], col }, { corners[3], col } },
		{ { corners[3], col }, { corners[0], col } },
	};
	visualiseLines(lines, 1.0f);
}

/**
 * Visualise camera distortion using a grid of size num
 */
void visualiseDistortion(const CameraCalib &calibCB, const CameraCalib &calibGT, const CameraMode &mode, float alphaBS, float alphaCB, float alphaGT)
{
	// Camera projections
	Eigen::Projective3f PGT = createProjectionMatrixCV<float>(calibGT);
	Eigen::Projective3f PCB = createProjectionMatrixCV<float>(calibCB);

	// Grid parameters
	int num = 20;
	float size = 200, dist = 100;

	// Resulting points
	std::vector<Eigen::Vector2f> undistorted, distortedGT, distortedCB;
	undistorted.reserve(num*num);
	distortedGT.reserve(num*num);
	distortedCB.reserve(num*num);

	for (int i = 0; i < num; i++)
	{
		for (int j = 0; j < num; j++)
		{
			//Eigen::Vector3f point (i * size/(num+1) - size/2, j * size/(num+1) - size/2, -dist);
			Eigen::Vector3f point (i * size/(num-1) - size/2, j * size/(num-1) - size/2, -dist);
			Eigen::Vector2f projGT = projectPoint2D(PGT, point);
			Eigen::Vector2f projCB = projectPoint2D(PCB, point);
			undistorted.push_back(projGT);
			if (projGT.x() > -mode.sizeW*1.2 && projGT.y() > -mode.sizeH*1.2 && projGT.x() < mode.sizeW*1.2 && projGT.y() < mode.sizeH*1.2)
				distortedGT.push_back(distortPointUnstable(calibGT, projGT));
			if (projCB.x() > -mode.sizeW*1.2 && projCB.y() > -mode.sizeH*1.2 && projCB.x() < mode.sizeW*1.2 && projCB.y() < mode.sizeH*1.2)
				distortedCB.push_back(distortPointUnstable(calibCB, projCB));
		}
	}

	if (alphaBS > 0)
		visualisePoints2D(undistorted, { 0,0,1,alphaBS }, 2.0f, 0.6f);
	if (alphaGT > 0)
		visualisePoints2D(distortedGT, { 0,1,0,alphaGT }, 2.0f, 0.5f);
	if (alphaCB > 0)
		visualisePoints2D(distortedCB, { 1,1,0,alphaCB }, 2.0f, 0.4f);
}