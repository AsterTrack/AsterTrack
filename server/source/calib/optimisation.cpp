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

#define EIGEN_HAS_OPENMP

#include "optimisation.hpp"
#include "opt/utilities.hpp"
#include "opt/ReprojectionError.hpp"

#include "util/log.hpp"

#include "unsupported/Eigen/NonLinearOptimization"
#include "Eigen-custom/LevenbergMarquardtSparse.hpp"

#include <float.h>
#include <algorithm>
#include <cmath>
#include <cassert>

/**
 * Optimisation of existing camera calibrations to minimise reprojection error
 */


/* Functions */

const char *getStopCodeText(int status)
{
	switch (status)
	{
	case Eigen::LevenbergMarquardtSpace::ImproperInputParameters:
		return "Improper optimisation input parameters!";
	case Eigen::LevenbergMarquardtSpace::RelativeReductionTooSmall:
		return "Reached ftol limit, relative reduction too small!";
	case Eigen::LevenbergMarquardtSpace::RelativeErrorTooSmall:
		return "Reached xtol limit, relative error too small!";
	case Eigen::LevenbergMarquardtSpace::RelativeErrorAndReductionTooSmall:
		return "Reached ftol and xtol limit, relative error and reduction too small!";
	case Eigen::LevenbergMarquardtSpace::CosinusTooSmall:
		return "Reached gtol limit, gradient too small!";
	case Eigen::LevenbergMarquardtSpace::TooManyFunctionEvaluation:
		return "Reached function evaluation limit!";
	case Eigen::LevenbergMarquardtSpace::FtolTooSmall:
		return "Reached F Tolerance limit!";
	case Eigen::LevenbergMarquardtSpace::XtolTooSmall:
		return "Reached X Tolerance limit!";
	case Eigen::LevenbergMarquardtSpace::GtolTooSmall:
		return "Reached G Tolerance limit!";
	case Eigen::LevenbergMarquardtSpace::UserAsked:
		return "User asked!";
	default:
		if (status > 0 && status < 10)
		{
			thread_local std::string text = "Ending optimisation with unknown error code 0!";
			thread_local const int len = text.length();
			text[len-2] = '0' + ((uint8_t)status)%10;
			return text.c_str();
		}
		else
			return "";
	}
}

/**
 * Calculates, logs and returns overall camera reprojection errors
 * Returns avg, stdDev, max in -1 to 1 coordinates
 */
OptErrorRes updateReprojectionErrors(ObsData &data, const std::vector<CameraCalib> &cameraCalibs)
{
	if (data.points.totalSamples == 0 && data.targets.empty())
	{
		LOGC(LError, "Got no points nor targets in optimisation data!");
		return {};
	}
	ScopedLogCategory optLogCategory(LOptimisation, false);
	
	// NOTE: Currently, target outliers are actually removed, so lm.fvec does not actually include them, so no need to compensate for them here
	int removedOutlierCount = 0;
	for (auto &tgt : data.targets)
		removedOutlierCount += tgt.outlierSamples;

	typedef ScalarInternal Scalar;
	std::vector<CameraCalib_t<Scalar>> camerasInternal(cameraCalibs.size());
	for (int c = 0; c < cameraCalibs.size(); c++)
		camerasInternal[c] = cameraCalibs[c];

	// Initialise optimisation error term, preparing the data (minus existing outliers)
	ReprojectionError<Scalar> errorTerm(camerasInternal, data);
	errorTerm.m_options.ignoreOutliers = false;

	// Create error stats
	VectorX<Scalar> errorVec(errorTerm.values());
	errorTerm.calculateError(camerasInternal, errorVec);
	if (errorVec.hasNaN())
	{
		LOGC(LWarn, "Error Vector has NaNs!\n");
		LOGC(LWarn, "Error Vector size: %d, tri point count: %d!\n", (int)errorVec.size(), (int)errorTerm.m_data->points.points.size());
	}

	// Calculate outlier limit
	OptErrorRes errors = logErrorStats("    Errors Unfiltered", errorVec, 0, removedOutlierCount);

	// Per-camera error accumulation
	VectorX<Scalar> cameraErrors = VectorX<Scalar>::Zero(cameraCalibs.size());
	std::vector<int> cameraBlobCounts(cameraCalibs.size(), 0);

	int index = 0;
	int outlierCnt = 0;
	for (ObsPoint &point : data.points.points)
	{
		point.error = 0.0f;
		for (ObsPointSample &sample : point.samples)
		{
			point.error += (float)errorVec(index);
			if (point.outlier)
			{
				outlierCnt++;
				errorVec(index) = 0;
			}
			else
			{
				cameraBlobCounts[sample.camera]++;
				cameraErrors(sample.camera) += errorVec(index)*errorVec(index);
			}
			index++;
		}
		point.error /= point.samples.size();
	}
	for (ObsTarget &target : data.targets)
	{
		for (ObsTargetFrame &frame : target.frames)
		{
			frame.error = 0;
			for (ObsTargetSample &sample : frame.samples)
			{
				frame.error += (float)errorVec(index);
				cameraBlobCounts[sample.camera]++;
				cameraErrors(sample.camera) += errorVec(index)*errorVec(index);
				index++;
			}
			frame.error /= frame.samples.size();
		}
	}

	// Get errors and log everything
	errors = logErrorStats("    Errors Filtered", errorVec, outlierCnt, removedOutlierCount);
	for (int c = 0; c < cameraCalibs.size(); c++)
		LOGCL("        Camera %d has %fpx RMSE reprojection error from %d inlier blobs!\n",
			c, std::sqrt(cameraErrors(c)/cameraBlobCounts[c])*PixelFactor, cameraBlobCounts[c]);
	if (std::isnan(errors.mean) || std::isnan(errors.stdDev) || std::isnan(errors.max))
		LOGCL("Error result has NaNs: (%f, %f, %f)\n", errors.mean, errors.stdDev, errors.max);

	return errors;
}

/**
 * Calculates, logs and returns overall camera reprojection errors
 * Returns avg, stdDev, max in -1 to 1 coordinates
 * Also fills the error maps
 */
OptErrorRes updateCameraErrorMaps(const ObsData &data, const std::vector<CameraCalib> &cameraCalibs, std::vector<CameraErrorMaps*> &cameraErrorMaps)
{
	if (data.points.totalSamples == 0 && data.targets.empty())
	{
		LOGC(LError, "Got no points nor targets in optimisation data!");
		return {};
	}
	assert(cameraCalibs.size() == cameraErrorMaps.size());
	ScopedLogCategory optLogCategory(LOptimisation, false);

	assert(data.targets.empty());
	// TODO: Consider using target errors for error maps if it will be relevant for continuous calibration

	typedef ScalarInternal Scalar;
	std::vector<CameraCalib_t<Scalar>> camerasInternal(cameraCalibs.size());
	for (int c = 0; c < cameraCalibs.size(); c++)
		camerasInternal[c] = cameraCalibs[c];

	// Initialise maps
	std::vector<std::vector<float>> mapErrorAccum(cameraCalibs.size());
	std::vector<std::vector<int>> mapCount(cameraCalibs.size());
	std::vector<std::vector<int>> mapOutliers(cameraCalibs.size());
	for (int c = 0; c < cameraCalibs.size(); c++)
	{
		if (cameraCalibs[c].invalid()) continue;
		Eigen::Vector2i mapSize = cameraErrorMaps[c]->mapSize;
		mapErrorAccum[c].resize(mapSize.x()*mapSize.y(), 0);
		mapCount[c].resize(mapSize.x()*mapSize.y(), 0);
		mapOutliers[c].resize(mapSize.x()*mapSize.y(), 0);
		cameraErrorMaps[c]->pointErrors.clear();
	}

	// Calculate error values
	ReprojectionError<Scalar> errorTerm(camerasInternal, data);
	errorTerm.m_options.ignoreOutliers = false;

	// Create error stats
	VectorX<Scalar> errorVec(errorTerm.values());
	errorTerm.calculateError(camerasInternal, errorVec);
	if (errorVec.hasNaN())
	{
		LOGC(LWarn, "Error Vector has NaNs!\n");
		LOGC(LWarn, "Error Vector size: %d, tri point count: %d!\n", (int)errorVec.size(), (int)errorTerm.m_data->points.points.size());
	}

	// Calculate outlier limit
	auto errors = logErrorStats("    Error Total", errorVec, 0);
	Scalar errorMax = errors.max;
	//Scalar outlierMax = errors.mean + outlierSigma*errors.stdDev;

	// Add errors from inliers
	int index = 0;
	for (auto &point : data.points.points)
	{
		for (auto &sample : point.samples)
		{
			int c = sample.camera;
			if (cameraCalibs[c].invalid()) continue;
			Eigen::Vector2i mapSize = cameraErrorMaps[c]->mapSize;
			Eigen::Vector2f pos = sample.point;
			int mapX = std::clamp((int)((pos.x()*mapSize.x() + mapSize.x())/2), 0, mapSize.x()-1);
			int mapY = std::clamp((int)((pos.y()*mapSize.y() + mapSize.y())/2), 0, mapSize.y()-1);
			//if (errorVec(index) < outlierMax)
			//{
			//	mapErrorAccum[c][mapY*mapSize.x()+mapX] += errorVec(index);
			//	mapCount[c][mapY*mapSize.x()+mapX]++;
			//	pointErrors[c]->push_back(Eigen::Vector3f(pos.x(), pos.y(), errorVec(index) / outlierMax));
			//}
			//else // Outliers not yet marked as outlier
			//{
			//	pointErrors[c]->push_back(Eigen::Vector3f(pos.x(), pos.y(), 2.0f));	
			//	mapOutliers[c][mapY*mapSize.x()+mapX]++;
			//}
			mapErrorAccum[c][mapY*mapSize.x()+mapX] += (float)errorVec(index);
			mapCount[c][mapY*mapSize.x()+mapX]++;
			cameraErrorMaps[c]->pointErrors.push_back(Eigen::Vector3f(pos.x(), pos.y(), (float)(errorVec(index) / errorMax)));
			index++;
			// TODO: No way to actually report already removed outliers in mapOutliers
		}
	}

	// Compile and normalise maps
	float maxReportedError = (float)(errors.mean+errors.stdDev*2);
	float errorScale = 255.0f / maxReportedError;
	float coverageScale = 255.0f / 50;
	float outlierScale = 255.0f / 5;
	float cellErrorAccum = 0, cellTotalErrorAccum = 0, cellErrorMax = 0;
	int usedCells = 0;
	for (int c = 0; c < cameraCalibs.size(); c++)
	{
		if (cameraCalibs[c].invalid()) continue;
		auto &maps = *cameraErrorMaps[c];
		Eigen::Vector2i mapSize = maps.mapSize;
		maps.errorMap.resize(mapSize.x()*mapSize.y());
		maps.outlierMap.resize(mapSize.x()*mapSize.y());
		maps.coverageMap.resize(mapSize.x()*mapSize.y());
		for (int i = 0; i < mapSize.x()*mapSize.y(); i++)
		{
			uint8_t error = mapCount[c][i] == 0? 0 : (uint8_t)std::min(mapErrorAccum[c][i] / mapCount[c][i] * errorScale, 255.0f);
			uint8_t outliers = (uint8_t)std::min(mapOutliers[c][i] * outlierScale, 255.0f);
			uint8_t coverage = (uint8_t)std::min(mapCount[c][i] * coverageScale, 255.0f);
			maps.errorMap[i] = error;
			maps.outlierMap[i] = outliers;
			maps.coverageMap[i] = coverage;
			if (mapCount[c][i] > 0)
			{
				usedCells++;
				cellErrorAccum += maps.errorMap[i];
				cellTotalErrorAccum += error;
				if (cellErrorMax < error) cellErrorMax = error;
			}
		}
	}

	LOGC(LTrace, "Error map has maximum reported error at %f, max error at %f, average cell error %f, "
		"avg capped cell error %f, among used cells, average cell error %f, avg capped cell error %f, max cell error %f\n", 
		maxReportedError, errorMax, cellTotalErrorAccum/index, cellErrorAccum/index, cellTotalErrorAccum/usedCells, cellErrorAccum/usedCells, cellErrorMax);

	return errors;
}

/**
 * Optimises a set of parameters (defined by options) to conform to the dataset of observations
 * Parameters may include camera extrinsics and intrinsics, target structure and motion, etc.
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
template<int Options>
int optimiseData(const OptimisationOptions &options, ObsData &data, std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, float toleranceFactor)
{
	if (data.points.totalSamples == 0 && data.targets.empty())
	{
		LOGC(LError, "Got no points nor targets in optimisation data!");
		return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
	}
	ScopedLogCategory optLogCategory(LOptimisation, false);

	std::vector<CameraCalib_t<ScalarInternal>> camerasInternal(cameraCalibs.size());
	for (int c = 0; c < cameraCalibs.size(); c++)
		camerasInternal[c] = cameraCalibs[c];

	// Norm the camera transforms and record for later
	auto norm = setCameraNorms<ScalarInternal>(camerasInternal, options, 1.0);

	// Initialise optimisation error term, preparing the data
	ReprojectionError<ScalarInternal, Options> errorTerm(camerasInternal, data);
	errorTerm.setOptimisationOptions(options);
	if (errorTerm.values() < errorTerm.inputs())
	{
		LOGC(LError, "Got only %d inputs to optimise %d parameters!\n", errorTerm.values(), errorTerm.inputs());
		return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
	}

	// Create initial parameter vector
	VectorX<ScalarInternal> calibParams(errorTerm.m_paramCount);
	writeCameraParameters<ScalarInternal>(camerasInternal, options, calibParams);
	int tgtIdx = 0;
	for (auto &tgt : data.targets)
	{
		int paramIdx = errorTerm.m_paramTarget[tgtIdx], paramLen = errorTerm.m_paramTarget[tgtIdx+1]-paramIdx;
		writeTargetParameters<ScalarInternal>(tgt, options, norm, calibParams.segment(paramIdx, paramLen));
		tgtIdx++;
	}

	auto readFromParameters = [&]()
	{
		// Read camera parameters
		readCameraParameters<ScalarInternal>(camerasInternal, options, calibParams);
		// Reapply previous norm
		setCameraNorms(camerasInternal, options, norm.second, norm.first);
		// Convert
		for (int c = 0; c < cameraCalibs.size(); c++)
			cameraCalibs[c] = camerasInternal[c];

		// Read out target parameters
		tgtIdx = 0;
		for (auto &tgt : data.targets)
		{
			int paramIdx = errorTerm.m_paramTarget[tgtIdx], paramLen = errorTerm.m_paramTarget[tgtIdx+1]-paramIdx;
			readTargetParameters<ScalarInternal>(tgt, options, norm, calibParams.segment(paramIdx, paramLen));
			tgtIdx++;
		}
		// Apply alternate target motion update
		if constexpr (Options & OptTgtMotionOpt)
			errorTerm.updateTargetMotions(data); // TODO: Use camera norms, currently incompatible with normaliseScale
	};

	typedef std::conditional_t<(Options & OptSparse) == OptSparse, 
		Eigen::LevenbergMarquardtSparse<ReprojectionError<ScalarInternal, Options>, ScalarInternal>,
		Eigen::LevenbergMarquardt<ReprojectionError<ScalarInternal, Options>, ScalarInternal>> LM;

	// Initialise optimisation algorithm
	LM lm(errorTerm);
	Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeInit(calibParams);
	lm.parameters.maxfev = 500; // fev+jev since we always return 1 fev for each jev
	lm.parameters.xtol = toleranceFactor * 0.001 * PixelSize;
	lm.parameters.ftol = toleranceFactor * 0.001;
	lm.parameters.gtol = toleranceFactor * 0.001;
	 // Seems to work well enough
	if (status == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
		return status;

	// Optimisation steps
	OptErrorRes lastError;
	while (true)
	{
		status = lm.minimizeOneStep(calibParams);

		int outlierCount = data.points.outlierSamples;
		// NOTE: Currently, target outliers are actually removed, so lm.fvec does not actually include them, so no need to compensate for them here
		int removedOutlierCount = 0;
		for (auto &tgt : data.targets)
			removedOutlierCount += tgt.outlierSamples;

		lastError = logErrorStats("    New Errors", lm.fvec, outlierCount, removedOutlierCount);

		readFromParameters();
		if (!iteration(lastError))
			break; // Decided to abort

		if (status != Eigen::LevenbergMarquardtSpace::Running)
			break;
	}

	return status;
}

/**
 * Optimises a set of parameters (defined by options) to conform to the dataset of observations
 * Parameters may include camera extrinsics and intrinsics, target structure and motion, etc.
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
int compareOptimiseData(const OptimisationOptions &options, ObsData &data, std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, float toleranceFactor)
{
	if (data.points.totalSamples == 0 && data.targets.empty())
	{
		LOGC(LError, "Got no points nor targets in optimisation data!");
		return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;
	}
	ScopedLogCategory optLogCategory(LOptimisation, false);

	std::vector<CameraCalib_t<ScalarInternal>> camerasInternal(cameraCalibs.size());
	for (int c = 0; c < cameraCalibs.size(); c++)
		camerasInternal[c] = cameraCalibs[c];

	// Norm the camera transforms and record for later
	auto norm = setCameraNorms<ScalarInternal>(camerasInternal, options, 1.0);

	constexpr int OPT1 = OptParallel | OptTgtMotionOpt;
	constexpr int OPT2 = OptSparse;
	constexpr int OPT3 = OptSparse;

	// Initialise optimisation error term, preparing the data
	ReprojectionError<ScalarInternal, OPT1> errorTerm1(camerasInternal, data);
	errorTerm1.setOptimisationOptions(options);

	ReprojectionError<ScalarInternal, OPT2> errorTerm2(camerasInternal, data);
	errorTerm2.setOptimisationOptions(options);

	ReprojectionError<ScalarInternal, OPT3> errorTerm3(camerasInternal, data);
	errorTerm3.setOptimisationOptions(options);

	assert(errorTerm1.m_paramCount == errorTerm2.m_paramCount);
	assert(errorTerm1.m_paramCount == errorTerm3.m_paramCount);
	auto &errorTerm = errorTerm1;
	if (errorTerm.values() < errorTerm.inputs())
		return Eigen::LevenbergMarquardtSpace::ImproperInputParameters;

	// Create initial parameter vector
	std::array<VectorX<ScalarInternal>, 3> calibParamsOpt;
	auto &calibParams = calibParamsOpt[0];
	for (int i = 0; i < 3; i++)
	{
		calibParamsOpt[i].resize(errorTerm1.m_paramCount);
		writeCameraParameters<ScalarInternal>(camerasInternal, options, calibParamsOpt[i]);
		int tgtIdx = 0;
		for (auto &tgt : data.targets)
		{
			int paramIdx = errorTerm1.m_paramTarget[tgtIdx], paramLen = errorTerm1.m_paramTarget[tgtIdx+1]-paramIdx;
			writeTargetParameters<ScalarInternal>(tgt, options, norm, calibParamsOpt[i].segment(paramIdx, paramLen));
			tgtIdx++;
		}
	}

	auto readFromParameters = [&]()
	{
		// Read camera parameters
		readCameraParameters<ScalarInternal>(camerasInternal, options, calibParams);
		// Reapply previous norm
		setCameraNorms(camerasInternal, options, norm.second, norm.first);
		// Convert
		for (int c = 0; c < cameraCalibs.size(); c++)
			cameraCalibs[c] = camerasInternal[c];

		// Read out target parameters
		int tgtIdx = 0;
		for (auto &tgt : data.targets)
		{
			int paramIdx = errorTerm.m_paramTarget[tgtIdx], paramLen = errorTerm.m_paramTarget[tgtIdx+1]-paramIdx;
			readTargetParameters<ScalarInternal>(tgt, options, norm, calibParams.segment(paramIdx, paramLen));
			tgtIdx++;
		}
		// Apply alternate target motion update
		if constexpr ((OPT1 & OptTgtMotionOpt) == OptTgtMotionOpt)
			errorTerm1.updateTargetMotions(data); // TODO: Use camera norms, currently incompatible with normaliseScale
		if constexpr ((OPT2 & OptTgtMotionOpt) == OptTgtMotionOpt)
			errorTerm2.updateTargetMotions(data); // TODO: Use camera norms, currently incompatible with normaliseScale
		if constexpr ((OPT3 & OptTgtMotionOpt) == OptTgtMotionOpt)
			errorTerm3.updateTargetMotions(data); // TODO: Use camera norms, currently incompatible with normaliseScale
	};


	typedef std::conditional_t<(OPT1 & OptSparse) == OptSparse, 
		Eigen::LevenbergMarquardtSparse<ReprojectionError<ScalarInternal, OPT1>, ScalarInternal>,
		Eigen::LevenbergMarquardt<ReprojectionError<ScalarInternal, OPT1>, ScalarInternal>> LM1;
	typedef std::conditional_t<(OPT2 & OptSparse) == OptSparse, 
		Eigen::LevenbergMarquardtSparse<ReprojectionError<ScalarInternal, OPT2>, ScalarInternal>,
		Eigen::LevenbergMarquardt<ReprojectionError<ScalarInternal, OPT2>, ScalarInternal>> LM2;
	typedef std::conditional_t<(OPT3 & OptSparse) == OptSparse, 
		Eigen::LevenbergMarquardtSparse<ReprojectionError<ScalarInternal, OPT3>, ScalarInternal>,
		Eigen::LevenbergMarquardt<ReprojectionError<ScalarInternal, OPT3>, ScalarInternal>> LM3;

	// Initialise optimisation algorithm
	LM1 lm1(errorTerm1);
	Eigen::LevenbergMarquardtSpace::Status status1 = lm1.minimizeInit(calibParamsOpt[0]);
	lm1.parameters.maxfev = 500; // fev+jev since we always return 1 fev for each jev
	lm1.parameters.xtol = toleranceFactor * 0.001 * PixelSize;
	lm1.parameters.ftol = toleranceFactor * 0.001;
	lm1.parameters.gtol = toleranceFactor * 0.001;
	 // Seems to work well enough
	if (status1 == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
		return status1;
	status1 = Eigen::LevenbergMarquardtSpace::Running;

	LM2 lm2(errorTerm2);
	Eigen::LevenbergMarquardtSpace::Status status2 = lm2.minimizeInit(calibParamsOpt[1]);
	lm2.parameters.maxfev = 500; // fev+jev since we always return 1 fev for each jev
	lm2.parameters.xtol = toleranceFactor * 0.001 * PixelSize;
	lm2.parameters.ftol = toleranceFactor * 0.001;
	lm2.parameters.gtol = toleranceFactor * 0.001;
	 // Seems to work well enough
	if (status2 == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
		return status2;
	status2 = Eigen::LevenbergMarquardtSpace::Running;

	LM3 lm3(errorTerm3);
	Eigen::LevenbergMarquardtSpace::Status status3 = lm3.minimizeInit(calibParamsOpt[2]);
	lm3.parameters.maxfev = 500; // fev+jev since we always return 1 fev for each jev
	lm3.parameters.xtol = toleranceFactor * 0.001 * PixelSize;
	lm3.parameters.ftol = toleranceFactor * 0.001;
	lm3.parameters.gtol = toleranceFactor * 0.001;
	 // Seems to work well enough
	if (status3 == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
		return status3;
	//status3 = Eigen::LevenbergMarquardtSpace::Running;
	status3 = Eigen::LevenbergMarquardtSpace::UserAsked;

	// Optimisation steps
	std::array<OptErrorRes, 3> lastError;
	std::array<float, 3> totalTime;
	std::array<int, 3> itCount;
	while (true)
	{
		ScopedLogLevel scopedLogLevel(LInfo);

		int outlierCount = data.points.outlierSamples;
		// NOTE: Currently, target outliers are actually removed, so lm.fvec does not actually include them, so no need to compensate for them here
		int removedOutlierCount = 0;
		for (auto &tgt : data.targets)
			removedOutlierCount += tgt.outlierSamples;

		if (status1 == Eigen::LevenbergMarquardtSpace::Running)
		{
			auto s = sclock::now();
			status1 = lm1.minimizeOneStep(calibParamsOpt[0]);
			totalTime[0] += dtMS(s, sclock::now());
			LOGCL("  Option 1 time %fms, status %d", dtMS(s, sclock::now()), status1);
			lastError[0] = logErrorStats("    Errors", lm1.fvec, outlierCount, removedOutlierCount);
			itCount[0]++;
		}


		if (status2 == Eigen::LevenbergMarquardtSpace::Running)
		{
			auto s = sclock::now();
			status2 = lm2.minimizeOneStep(calibParamsOpt[1]);
			totalTime[1] += dtMS(s, sclock::now());
			LOGCL("  Option 2 time %fms, status %d", dtMS(s, sclock::now()), status2);
			lastError[1] = logErrorStats("    Errors", lm2.fvec, outlierCount, removedOutlierCount);
			itCount[1]++;
		}

		if (status3 == Eigen::LevenbergMarquardtSpace::Running)
		{
			auto s = sclock::now();
			status3 = lm3.minimizeOneStep(calibParamsOpt[2]);
			totalTime[2] += dtMS(s, sclock::now());
			LOGCL("  Option 3 time %fms, status %d", dtMS(s, sclock::now()), status3);
			lastError[2] = logErrorStats("    Errors", lm3.fvec, outlierCount, removedOutlierCount);
			itCount[2]++;
		}

		readFromParameters();
		if (!iteration(lastError[0]))
			break; // Decided to abort

		if (status1 != Eigen::LevenbergMarquardtSpace::Running && status2 != Eigen::LevenbergMarquardtSpace::Running && status3 != Eigen::LevenbergMarquardtSpace::Running)
			break;
	}

	LOGC(LInfo, "Option 1 took %fms to optimise to %.2fpx RMSE in %d its",
		totalTime[0], lastError[0].rmse*PixelFactor, itCount[0]);
	LOGC(LInfo, "Option 2 took %fms to optimise to %.2fpx RMSE in %d its",
		totalTime[1], lastError[1].rmse*PixelFactor, itCount[1]);
	LOGC(LInfo, "Option 3 took %fms to optimise to %.2fpx RMSE in %d its",
		totalTime[2], lastError[2].rmse*PixelFactor, itCount[2]);

	return status1;
}

[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
int optimiseDataSparse(const OptimisationOptions &options, const ObsData &data, std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, float toleranceFactor)
{
	return optimiseData<OptSparse>(options, const_cast<ObsData&>(data), cameraCalibs, iteration, toleranceFactor);
}

/**
 * Optimises a set of parameters (defined by options) to conform to the dataset of observations
 * Parameters may include camera extrinsics and intrinsics, but NOT target parameters, as they would need to be written back to data
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
int optimiseCameras(const OptimisationOptions &options, const ObsData &data, std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration)
{
	assert(data.targets.empty() || (!options.motion && !options.structure));
	// Does not make sense to use OptSparse only for points, generally over 10x slower - only benefit is for targets
	return optimiseData<OptParallel>(options, const_cast<ObsData&>(data), cameraCalibs, iteration, 1.0f);
}

/**
 * Optimises target structure and motion to conform to the dataset of observations
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
int optimiseTargets(const ObsData &data, const std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, float toleranceFactor)
{
	assert(data.points.points.empty());
	if (data.targets.empty()) return Eigen::LevenbergMarquardtSpace::Status::ImproperInputParameters;
	return optimiseData<OptSparse>(
		OptimisationOptions(true, false, false, false, false), 
		const_cast<ObsData&>(data), const_cast<std::vector<CameraCalib>&>(cameraCalibs), iteration, toleranceFactor);
}

/**
 * Optimises target structure and motion to conform to the dataset of observations
 * iteration receives errors each iteration and can return false to abort further optimisation
 */
[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
int optimiseTargetsCompare(const ObsData &data, const std::vector<CameraCalib> &cameraCalibs, std::function<bool(OptErrorRes)> iteration, float toleranceFactor)
{
	assert(data.points.points.empty());
	if (data.targets.empty()) return Eigen::LevenbergMarquardtSpace::Status::ImproperInputParameters;
	return compareOptimiseData(
		OptimisationOptions(true, false, false, false, false),
		const_cast<ObsData&>(data), const_cast<std::vector<CameraCalib>&>(cameraCalibs), iteration, toleranceFactor);
}
