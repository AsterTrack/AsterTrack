/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/**
 * NOTE:
 * This file is licensed under the MPL despite currently being deeply integrated into the project.
 * The intent is that if you need it badly enough, it is easy to extract it to use in your own project.
 * This would require removing reference to, among others, obs_data.hpp, but it is very much doable.
 */

#ifndef RECONSTRUCTION_POINT_H
#define RECONSTRUCTION_POINT_H

#include "calib/obs_data.hpp"
#include "calib_point/parameters.hpp"

#include "util/eigendef.hpp"
#include "util/error.hpp"

/**
 * Geometric reconstruction of observations used for calibration
 */

/**
 * Attempts to reconstruct the geometry (points, cameras calibration) of the scene given the observed points
 */
std::optional<ErrorMessage> reconstructGeometry(const ObsPointData &data, std::vector<CameraCalib> &cameraCalibs, PointReconstructionParameters params);

#endif // RECONSTRUCTION_POINT_H