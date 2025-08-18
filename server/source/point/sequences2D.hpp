/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef POINT_SEQUENCES_2D_H
#define POINT_SEQUENCES_2D_H

#include "sequence_data.hpp"
#include "parameters.hpp"

#include "calib/camera_system.hpp" // CameraSystemCalibration

#include "pipeline/record.hpp" // BlobProperty

#define INCLUDE_MATCHING_PARAMS_ONLY
#include "util/matching.hpp"


/**
 * Recording 2D sequences of observed points and correlating them between cameras
 */


/**
 * Updates the sequences of cameraIndex only to include new points, and verifies sequences with other camera sequences if required
 */
bool updateSequenceCaptures(const SequenceAquisitionParameters &params,
	CameraSystemCalibration &calibration, SequenceData &sequences, int curFrame, int cameraIndex,
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<Eigen::Vector2f> &rawPoints2D, 
	const std::vector<BlobProperty> &properties, const std::vector<int> &pt2GTMarker);


void checkSequenceHealth(const SequenceAquisitionParameters &params, CameraSystemCalibration &calibration, SequenceData &sequences, int curFrame, bool confidenceCheck);

void updateTrustFromEpipolarStats(const SequenceAquisitionParameters &params, FundamentalMatrix &FM);

#endif // POINT_SEQUENCES_2D_H