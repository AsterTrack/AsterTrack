/**
AsterTrack Optical Tracking System
Copyright (C) 2026 Seneral <seneral@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, specifically version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef RECONSTRUCTION_TARGET_H
#define RECONSTRUCTION_TARGET_H

#include "calib/obs_data.hpp"

#include "util/eigendef.hpp"

/**
 * Reconstruction of Geometry and Motion used for Target Calibration
 */

/**
 * Attempts to reconstruct the motion and structure of a single rigid target given its observations
 */
bool reconstructTarget(const std::vector<CameraCalib> &cameraCalibs, ObsTarget &target, 
	int pMaxIteration, int pMinAbortIteration, int pCorrectIteration, int pDropIteration,
	ObsTarget *targetGT = NULL, std::vector<float> *frameChanges = NULL, 
    std::vector<float> *frameErrors = NULL, std::vector<float> *frameGTDiff = NULL);

#endif // RECONSTRUCTION_TARGET_H