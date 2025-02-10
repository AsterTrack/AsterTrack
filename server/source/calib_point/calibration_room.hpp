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

#ifndef CALIBRATION_ROOM_H
#define CALIBRATION_ROOM_H

#include "parameters.hpp"

#include "calib/optimisation.hpp"
#include "calib/obs_data.hpp"

#include "util/eigendef.hpp"


/**
 * Calibration of room floor and extends
 */

/**
 * A calibration of a static point with multiple samples that is invariant to room calibration
 */
template<typename Scalar>
struct StaticPointSamples
{
	std::vector<std::pair<int,Vector2<Scalar>>> samples;
	Eigen::Matrix<Scalar,3,1> pos;
	float confidence;
	int startObservation;
	int sampleCount;
	bool sampling;

    void update(const std::vector<CameraCalib> &calibs);
};

/**
 * Calculates, logs and returns overall camera reprojection errors. Also marks all outliers
 * Returns avg, stdDev, max in -1 to 1 coordinates
 */
OptErrorRes determinePointOutliers(ObsData &data, const std::vector<CameraCalib> &cameraCalibs, const PointOutlierErrors &params);

/**
 * Calibrate a transformation to the room coordinate system given 3 or more calibrated points on the floor, with the first being the new origin
 * The distance between the first two points should be passed as distance12 to determine the scale
 */
template<typename Scalar>
int estimateFloorTransform(const std::vector<StaticPointSamples<Scalar>> &floorPoints, Scalar distance12, Eigen::Matrix<Scalar,3,3> &roomOrientation, Eigen::Transform<Scalar,3,Eigen::Affine> &roomTransform);

/**
 * Normalise the calibration
 */
template<typename Scalar>
void getCalibNormalisation(const std::vector<CameraCalib_t<Scalar>> &calibs, Eigen::Matrix<Scalar,3,3> &roomOrientation, Eigen::Transform<Scalar,3,Eigen::Affine> &roomTransform, Scalar height = 2.5, Scalar pairwiseDist = 4.0);

/**
 * Attempt to transfer the room calibration between calibrations - cameras should match and mostly retain their relations
 * This is intended to be used between optimisations to ensure the room calibration isn't eroded away
 * But it can also transfer if cameras moved as long as two didn't move
 */
template<typename Scalar>
bool transferRoomCalibration(const std::vector<CameraCalib_t<Scalar>> &calibsSrc, const std::vector<CameraCalib_t<Scalar>> &calibsTgt,
	Matrix3<Scalar> &roomOrientation, Affine3<Scalar> &roomTransform);

#endif // CALIBRATION_ROOM_H