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

#ifndef CALIBRATION_POINT_PARAMETERS_H
#define CALIBRATION_POINT_PARAMETERS_H

#include "util/eigendef.hpp"

/**
 * Parameters for point calibration
 */

struct PointOutlierErrors
{
	Eigen::Vector2i gridSize = Eigen::Vector2i(10, 10);
	struct
	{
		float trigger = 4;
		float consider = 2.5f;
		float force = 6.0f;
		float bucket = 2.0f;
	} sigma;
};

struct PointReconstructionParameters
{
	struct
	{
		int minPairwiseCorrespondences = 50;
		float minConfidence = 1;
	} FM;
	struct
	{
		float nColMaxFactor = 2.0f;
		float nColMinFactor = 0.2f;
		int nColMin = 100;
		float tupleMaxFactor = 1;
		float tupleTestMaxFactor = 100;
		float minRankFactor = 10;
	} basis;
};

struct PointCalibParameters
{
	PointReconstructionParameters reconstruction;
	PointOutlierErrors outliers;
};

#endif // CALIBRATION_POINT_PARAMETERS_H