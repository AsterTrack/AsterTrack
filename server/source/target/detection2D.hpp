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

#ifndef TARGET_DETECTION_2D_H
#define TARGET_DETECTION_2D_H

#include "util/eigendef.hpp"

#include "target.hpp"
#include "tracking2D.hpp"

#include <stop_token>

/**
 * Detecting a target (set of markers) in a set of 2D points
 */


/**
 * Probe for a target in 2D observations by testing a number of rotations at the approximate given position using matchTargetPointsSlow
 */
TargetMatch2D probeTarget2D(std::stop_token stopToken, const TargetCalibration3D &target3D, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, 
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	Eigen::Vector3f pos, int cameraCount, int probeCount, const TargetDetectionParameters &params, const TargetTrackingParameters &track, 
	TargetTracking2DData &internalData);

/**
 * Search for a target in 2D observations by trying to match target point combinations to a set of 3 blobs with p3p
 */
TargetMatch2D searchTarget2D(std::stop_token stopToken, const TargetCalibration3D &target3D, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, 
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	int focusCamera, int cameraCount, const TargetDetectionParameters &params, const TargetTrackingParameters &track, 
	TargetTracking2DData &internalData);

#endif // TARGET_DETECTION_2D_H