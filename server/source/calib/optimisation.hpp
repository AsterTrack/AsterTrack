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

#ifndef OPTIMISATION_H
#define OPTIMISATION_H

#include "util/eigendef.hpp"

#include "obs_data.hpp"

#include <stop_token>

/**
 * Optimisation of existing camera calibrations to minimise reprojection error
 */


struct OptimisationOptions
{
	// Targets
	bool motion;
	bool structure;
	// Camera Extrinsics
	bool position;
	bool rotation;
	// Camera Intrinsics
	bool focalLen;
	bool principal;
	bool tangential;
	bool radial;
	int radialOrder;
	bool sharedRadial;
	// Normalisation
	bool normalisePos, normaliseScale;
	// Outliers
	bool ignoreOutliers;

	OptimisationOptions(bool extrinsics, bool align, bool radial, bool target, bool normalise = true)
		: motion(target), structure(target),
		position(extrinsics), rotation(extrinsics), focalLen(align), principal(align),
		tangential(align), radial(radial), radialOrder(3), 
		sharedRadial(true), 
		normalisePos(normalise), normaliseScale(normalise),
		ignoreOutliers(true)
	{}
	OptimisationOptions() : OptimisationOptions(true, true, true, false)
	{}
};

struct CameraErrorMaps
{
	Eigen::Vector2i mapSize;
	std::vector<uint8_t> errorMap;
	std::vector<uint8_t> outlierMap;
	std::vector<uint8_t> coverageMap;
	std::vector<Eigen::Vector3f> pointErrors;
};

enum OptOptions {
	OptNone = 0,
	OptAutoDiff = 1<<0,			// TODO: Use instead of #define OPT_AUTODIFF
	OptTgtMotionOpt = 1<<1,		// Optimise target motion with nested optimisations instead of adding them to main parameters
	OptSparse = 1<<2,
	OptParallel = 1<<3
};

struct OptErrorRes
{
	int num = 0;
    float mean = std::numeric_limits<float>::max();
	float rmse = std::numeric_limits<float>::max();
	float stdDev = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::max();
};


/* Functions */

const char *getStopCodeText(int status);

/**
 * Calculates, logs and returns overall reprojection errors
 * Returns avg, stdDev, max in -1 to 1 coordinates
 */
OptErrorRes updateReprojectionErrors(ObsData &data, const std::vector<CameraCalib> &cameraCalibs);

/**
 * Calculates, logs and returns overall camera reprojection errors
 * Returns avg, stdDev, max in -1 to 1 coordinates
 * Also fills the error maps
 */
OptErrorRes updateCameraErrorMaps(const ObsData &data, const std::vector<CameraCalib> &cameraCalibs, std::vector<CameraErrorMaps*> &cameraErrorMaps, std::vector<OptErrorRes> &cameraErrors);

/**
 * Optimises a set of parameters (defined by options) to conform to the dataset of observations
 * Parameters may include camera extrinsics and intrinsics, target structure and motion, etc.
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
int optimiseDataSparse(const OptimisationOptions &options, const ObsData &data, std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, std::stop_token stopToken, float toleranceFactor = 1.0f);

/**
 * Optimises a set of parameters (defined by options) to conform to the dataset of observations
 * Parameters may include camera extrinsics and intrinsics, but NOT target parameters, as they would need to be written back to data
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
int optimiseCameras(const OptimisationOptions &options, const ObsData &data, std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, std::stop_token stopToken);

/**
 * Optimises target structure and motion to conform to the dataset of observations
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
int optimiseTargets(const ObsData &data, const std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, std::stop_token stopToken, float toleranceFactor = 1.0f);

/**
 * Optimises target structure and motion to conform to the dataset of observations
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
int optimiseTargetsCompare(const ObsData &data, const std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, std::stop_token stopToken, float toleranceFactor = 1.0f);

#endif // OPTIMISATION_H