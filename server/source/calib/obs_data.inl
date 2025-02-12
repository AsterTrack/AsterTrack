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

#ifndef OBS_DATA_INL
#define OBS_DATA_INL

#include "obs_data.hpp"

#ifdef POINT_TRIANGULATION_H

/**
 * Adds triangulated points of the current frame to the point database.
 */
void addTriangulations(ObsPointData &data, const std::vector<TriangulatedPoint> &triPoints, int frame);

#endif

#ifdef SEQUENCE_DATA_H

/**
 * Adds triangulatable points in the frame range to the point database.
 */
void addTriangulatableObservations(ObsPointData &data, const std::vector<MarkerSequences> &observations, int frameStart = 0, int frameEnd = std::numeric_limits<int>::max());

/**
 * Fill target observation database with observations from existing frames and markers (specified by target.frames and target.markerMap)
 */
void updateTargetObservations(ObsTarget &target, const std::vector<MarkerSequences> &observations);

/**
 * Fill target observation database with observations from given frames and markers (specified by target.markerMap)
 */
void addTargetObservations(ObsTarget &target, const std::vector<MarkerSequences> &observations, const std::vector<int> frames);

#endif

#ifdef TARGET_TRACKING_2D_H

/**
 * Adds observations of a target for the current frame
 * Used for iterative addition of tracked targets
 */
void addTrackedTarget(ObsData &data, const TargetMatch2D &observations, const std::vector<std::vector<Eigen::Vector2f>*> &points2D, std::size_t frame);

#endif

#endif // OBS_DATA_INL