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
		// Mimic old, non-iterative behaviour
		bool forceOneshotReconstruction = false;
		int testCondition = 4; // If using iterative algorithm

		// Experimental options for projective depth transfers
		bool allowIndirectTransfers = false;
		bool allowNewInititialisations = false;

		// Picking best next transfer
		float transferPow = 0.6f;
		float correspondencePow = 0.8f;
		float indirectionPow = 0.6f;

		// Controlling size of iteration submatrix
		// \ Judging whether to accept next best transfer or abort
		int minIterationViews = 100; // Always accept transfer if view count is lacking
		float acceptWithFillRateMin = 0.95f; // Always accept transfer if current submatrix is basically fully determined
		float minFillRate = 0.6f; // Min total sample fill rate in resulting submatrix
		float minPrimaryPointRate = 0.4f; // Min primary points in resulting submatrix
		float primaryPointCoverage = 0.75f; // Coverage to treat a point as primary point

		// Early-out for iteration if most points are already fully recovered
		float stopAtPointRecoveryRate = 0.1f; // Use for testing only, does not scale with more cameras
	} strategy;
	struct
	{
		int maxParallelism = 8;
		float nColMaxFactor = 1.0f;
		float nColMinFactor = 0.1f;
		int nColMin = 100;
		float tupleMaxFactor = 0.5;
		float tupleTestMaxFactor = 20;
		float minRankFactor = 10;
	} basis;

	float outlierSigma = 1000.0f; // Effectively disabled
};

struct PointCalibParameters
{
	PointReconstructionParameters reconstruction;
	PointOutlierErrors outliers;
};

#endif // CALIBRATION_POINT_PARAMETERS_H