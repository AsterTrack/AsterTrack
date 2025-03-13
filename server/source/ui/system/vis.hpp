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

#ifndef SYSTEM_VIS_H
#define SYSTEM_VIS_H

#include "ui/ui.hpp"
#include "ui/gl/visualisation.hpp"


struct VisTargetLock
{
	SynchronisedS<ObsTarget>::ConstLockedPtr target_lock;
	Synchronised<ObsData>::ConstLockedPtr db_lock;
	const ObsTarget *obs = nullptr;
	const TargetCalibration3D *calib = nullptr;
	bool hasPose = false;
	int frameIdx = -1;
	const TargetCalibration3D *targetGT = nullptr;

    operator bool() const { return calib != nullptr; }
    bool hasObs() const { return obs != nullptr; }

	Eigen::Isometry3f getPose() const
	{
		Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
		if (obs)
			pose = obs->frames[frameIdx].pose;
		else
			pose.translation() = Eigen::Vector3f(0,0,1);
		return pose;
	}
};

struct VisFrameLock
{
	BlockedQueue<std::shared_ptr<FrameRecord>>::View<true> frames;
	BlockedQueue<std::shared_ptr<FrameRecord>>::const_iterator frameIt;
	bool isRealtimeFrame = true;
	bool hasFrame = false;
	VisTargetLock target = {};

    operator bool() const { return hasFrame; }
};

/**
 * General Visualisation
 */


/**
 * Render 2D blobs with their respective sizes
 */
void visualiseBlobs2D(const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, Color color, float sizeFactor = 1.0f, float crossSize = 0.0f);

/**
 * Update VBO with a set of points with errors from 0-1 in its z coordinate lerping the color from A to B, or O if out of that range
 */
void updateErrorPointsVBO(unsigned int &VBO, const std::vector<Eigen::Vector3f> &pointErrors, Color colorA, Color colorB, Color colorO, float size = 1.0f, float depth = 0.9f);

/**
 * Write target markers into given section of VisPoints using given pose
 */
void updateTargetMarkerVis(const PipelineState &pipeline, const TargetCalibration3D &target,
	const std::vector<std::vector<int>> &visibleMarker, Eigen::Isometry3f pose, Color8 color, float scale, VisPoint *points);

/**
 * Render 3D Rays coming from the camera emitter
 */
void visualiseRays(const CameraCalib &emitter, const std::vector<Eigen::Vector2f> &points2D, Color8 color = Color{ 0, 0.5f, 0, 1 });

/*
 * Render 3D wireframe of bounds in given pose
 */
void visualiseBounds3D(Bounds3f bounds, Eigen::Isometry3f pose);

/*
 * Render 2D wireframe of bounds
 */
void visualiseBounds2D(Bounds2f bounds);

/**
 * Render visualisation of camera distortion using a grid of size num
 */
void visualiseDistortion(const CameraCalib &calibCB, const CameraCalib &calibGT, const CameraMode &mode, float alphaBS = 1.0, float alphaCB = 1.0, float alphaGT = 0.0);


/**
 * VisTarget (target calibration visualisation)
 */
std::vector<VisPoint>& visualiseVisTargetMarkers(const PipelineState &pipelineGT, const VisualisationState &visState, const VisTargetLock &visTarget);
std::pair<int,int> interactWithVisTargetMarker(Eigen::Isometry3f view, Eigen::Projective3f proj, Eigen::Vector2f mouse);
void visualiseVisTargetObservations(const std::vector<CameraCalib> &calibs, const VisualisationState &visState, const VisTargetLock &visTarget);
void visualiseVisTargetMarkerFoV(const std::vector<CameraCalib> &calibs, const VisualisationState &visState, const VisTargetLock &visTarget);
void visualiseVisTargetObsCameraRays(const std::vector<CameraCalib> &calibs, const VisualisationState &visState, const VisTargetLock &visTarget);
void visualiseMarkerSequenceRays(const std::vector<CameraCalib> &calibs, const VisTargetLock &visTarget, const MarkerSequences &seq, int sequenceMarker);

/**
 * Unternal TargetTracking2DData visualisation
 */
void visualiseTarget2DMatchingStages(VisualisationState &visState, const CameraCalib &calib, const CameraFrameRecord &frame,
	const TargetCalibration3D &target, const TargetTracking2DData::CameraMatchingStages &matchingData, float expandMarkerFoV);
void visualiseTarget2DUncertaintyAxis(UncertaintyAxisAlignment uncertaintyAxis);

#endif // SYSTEM_VIS_H