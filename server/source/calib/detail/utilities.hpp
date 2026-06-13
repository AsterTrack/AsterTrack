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

#ifndef OPTIMISATION_UTIL_H
#define OPTIMISATION_UTIL_H

#include "../optimisation.hpp"

#include "util/eigenutil.hpp"
#include "util/log.hpp"

#include <set>

typedef CVScalar ScalarInternal;

template<typename Scalar = ScalarInternal>
static inline int numCamParams(const OptimisationOptions &opt, const std::vector<CameraCalib_t<Scalar>> &cameras)
{
	int extCam = 3 * (opt.position + opt.rotation);
	int intCam = opt.focalLen + (opt.tangential? 2 : 0) + (opt.principal? 2 : 0);
	int numRad = cameras.size();
	if (opt.sharedRadial)
	{
		std::set<int> lenses;
		for (auto &camera : cameras)
		{
			if (camera.lensID >= 0 && !lenses.insert(camera.lensID).second)
				numRad--; // Remove duplicates
		}
	}
	int shared = opt.radial? (opt.radialOrder * numRad) : 0;
	return (extCam + intCam) * cameras.size() + shared;
}
static inline int numTgtStruct(const OptimisationOptions &opt, const ObsTarget& tgt) { return opt.structure? tgt.markers.size()*3 : 0; }
static inline int numTgtMotion(const OptimisationOptions &opt, const ObsTarget& tgt) { return opt.motion? tgt.frames.size()*6 : 0; }
static inline int numTgt(const OptimisationOptions &opt, const ObsTarget& tgt) { return (opt.motion? tgt.frames.size()*6 : 0) + (opt.structure? tgt.markers.size()*3 : 0); }


/**
 * Set the specified norms and return the previous norm
*/
template<typename Scalar = ScalarInternal>
static inline std::pair<Vector3<Scalar>, Scalar> setCameraNorms(std::vector<CameraCalib_t<Scalar>> &cameras, const OptimisationOptions &opt, Scalar fixScale = 0, Vector3<Scalar> fixOrigin = Vector3<ScalarInternal>::Constant(0))
{
	Vector3<Scalar> origin = Vector3<Scalar>::Zero();
	Scalar scale = 1;
	if (!opt.normalisePos)
		return { origin, scale };

	for (int c = 0; c < cameras.size(); c++)
		origin += cameras[c].transform.translation();
	origin /= cameras.size();

	if (opt.normaliseScale)
	{
		scale = 0;
		for (int c = 0; c < cameras.size(); c++)
		{
			cameras[c].transform.translation() -= origin;
			scale += (cameras[c].transform.translation()).norm();
		}
		scale /= (Scalar)cameras.size();
		if (scale < 0.000000001f)
			return { origin, 1.0f };
		Scalar factor = fixScale / scale;
		for (int c = 0; c < cameras.size(); c++)
			cameras[c].transform.translation() = cameras[c].transform.translation() * factor + fixOrigin;
	}
	else
	{
		for (int c = 0; c < cameras.size(); c++)
			cameras[c].transform.translation() += fixOrigin-origin;
	}

	for (int c = 0; c < cameras.size(); c++)
		cameras[c].UpdateDerived();

	return { origin, scale };
}

template<typename Scalar = ScalarInternal>
static inline void readCameraParameters(std::vector<CameraCalib_t<Scalar>> &cameras, const OptimisationOptions &opt, const Eigen::Ref<const VectorX<Scalar>> &calibParams)
{
	int numShared = 0;
	if (opt.sharedRadial && opt.radial)
	{ // Don't touch radial if none enabled, reset higher orders if at least one is enabled
		std::map<int,int> lenses;
		for (auto &camera : cameras)
		{
			if (camera.lensID >= 0)
				lenses.insert({ camera.lensID, 0 });
		}
		for (auto &lens : lenses)
			lens.second = numShared++;
		for (int c = 0; c < cameras.size(); c++)
		{
			int lens = cameras[c].lensID;
			if (lens < 0) continue;
			int i = lenses[lens] * opt.radialOrder;
			if (opt.radialOrder >= 1)
				cameras[c].distortion.k1 = calibParams(i++);
			else
				cameras[c].distortion.k1 = 0;
			if (opt.radialOrder >= 2)
				cameras[c].distortion.k2 = calibParams(i++);
			else
				cameras[c].distortion.k2 = 0;
			if (opt.radialOrder >= 3)
				cameras[c].distortion.k3 = calibParams(i++);
			else 
				cameras[c].distortion.k3 = 0;
		}
	}

	int index = numShared * opt.radialOrder;
	for (int c = 0; c < cameras.size(); c++)
	{
		if (opt.focalLen)
		{
			cameras[c].f = calibParams(index++);
			cameras[c].fInv = 1.0/cameras[c].f;
		}
		if (opt.principal)
		{
			cameras[c].principalPoint.x() = calibParams(index++);
			cameras[c].principalPoint.y() = calibParams(index++);
		}
		if (opt.tangential)
		{
			cameras[c].distortion.p1 = calibParams(index++);
			cameras[c].distortion.p2 = calibParams(index++);
		}
		if ((opt.sharedRadial && cameras[c].lensID >= 0) || !opt.radial)
			continue;
		if (opt.radialOrder >= 1)
			cameras[c].distortion.k1 = calibParams(index++);
		else
			cameras[c].distortion.k1 = 0;
		if (opt.radialOrder >= 2)
			cameras[c].distortion.k2 = calibParams(index++);
		else
			cameras[c].distortion.k2 = 0;
		if (opt.radialOrder >= 3)
			cameras[c].distortion.k3 = calibParams(index++);
		else 
			cameras[c].distortion.k3 = 0;
	}

	for (int c = 0; c < cameras.size(); c++)
	{
		if (opt.position)
		{
			cameras[c].transform.translation() = calibParams.template segment<3>(index);
			index += 3;
		}
		if (opt.rotation)
		{
			cameras[c].transform.linear() = DecodeAARot<Scalar>(calibParams.template segment<3>(index)); 
			index += 3;
		}
		cameras[c].UpdateDerived();
	}
}

template<typename Scalar = ScalarInternal>
static inline void writeCameraParameters(const std::vector<CameraCalib_t<Scalar>> &cameras, const OptimisationOptions &opt, Eigen::Ref<VectorX<Scalar>> calibParams)
{
	calibParams.setZero();

	int numShared = 0;
	if (opt.sharedRadial && opt.radial)
	{ // Don't touch radial if none enabled, reset higher orders if at least one is enabled
		std::map<int,std::pair<int,int>> lenses;
		for (auto &camera : cameras)
		{
			if (camera.lensID >= 0)
				lenses.insert({ camera.lensID, { 0, 0 } });
			// TODO: Add ability to mark lenses as static to not optimise them but others
			// useful when they were already optimised previously on more data, but some lenses are new
			// Would require passing in way more information in here - e.g. a list of LensCalibs to draw that info from:/
			// Current workarounds include: Optimising it anyway, later reversing manually in lens_presets.json, then optimising again but not radial distortions
		}
		for (auto &lens : lenses)
			lens.second.first = numShared++;
		for (int c = 0; c < cameras.size(); c++)
		{
			int lens = cameras[c].lensID;
			if (lens < 0) continue;
			lenses[lens].second++;
			int i = lenses[lens].first * opt.radialOrder;
			if (opt.radialOrder >= 1)
				calibParams(i++) += cameras[c].distortion.k1;
			if (opt.radialOrder >= 2)
				calibParams(i++) += cameras[c].distortion.k2;
			if (opt.radialOrder >= 3)
				calibParams(i++) += cameras[c].distortion.k3;
		}
		for (auto &lens : lenses)
			for (int j = 0; j < opt.radialOrder; j++)
				calibParams(lens.second.first*opt.radialOrder+j) /= lens.second.second;
	}

	int index = numShared * opt.radialOrder;
	for (int c = 0; c < cameras.size(); c++)
	{
		if (opt.focalLen)
			calibParams(index++) = cameras[c].f;
		if (opt.principal)
		{
			calibParams(index++) = cameras[c].principalPoint.x();
			calibParams(index++) = cameras[c].principalPoint.y();
		}
		if (opt.tangential)
		{
			calibParams(index++) = cameras[c].distortion.p1;
			calibParams(index++) = cameras[c].distortion.p2;
		}
		if ((opt.sharedRadial && cameras[c].lensID >= 0) || !opt.radial)
			continue;
		if (opt.radialOrder >= 1)
			calibParams(index++) = cameras[c].distortion.k1;
		if (opt.radialOrder >= 2)
			calibParams(index++) = cameras[c].distortion.k2;
		if (opt.radialOrder >= 3)
			calibParams(index++) = cameras[c].distortion.k3;
	}

	for (int c = 0; c < cameras.size(); c++)
	{
		if (opt.position)
		{
			calibParams.template segment<3>(index) = cameras[c].transform.translation();
			index += 3;
		}
		if (opt.rotation)
		{
			calibParams.template segment<3>(index) = EncodeAARot<Scalar>(cameras[c].transform.linear());
			index += 3;
		}
	}
}

template<typename Scalar = ScalarInternal>
static inline void readTargetParameters(ObsTarget &target, const OptimisationOptions &opt, const std::pair<Vector3<Scalar>, Scalar> norm, const Eigen::Ref<const VectorX<Scalar>> &calibParams)
{
	int base = 0;
	if (opt.structure)
	{
		for (int m = 0; m < target.markers.size(); m++)
			target.markers[m] = (norm.second*calibParams.template segment<3>(m*3)).template cast<float>();
		base = target.markers.size()*3;
	}
	if (opt.motion)
	{
		int f = 0;
		for (auto frame = target.frames.begin(); frame != target.frames.end(); frame++, f++)
		{
			frame->pose.translation() = (norm.second * calibParams.template segment<3>(base+f*6+0) + norm.first).template cast<float>();
			frame->pose.linear() = DecodeAARot<float>(calibParams.template segment<3>(base+f*6+3));
		}
	}
}
template<typename Scalar = ScalarInternal>
static inline void writeTargetParameters(const ObsTarget &target, const OptimisationOptions &opt, const std::pair<Vector3<Scalar>, Scalar> norm, Eigen::Ref<VectorX<Scalar>> calibParams)
{
	int base = 0;
	if (opt.structure)
	{
		for (int m = 0; m < target.markers.size(); m++)
			calibParams.template segment<3>(m*3) = target.markers[m].template cast<Scalar>() / norm.second;
		base = target.markers.size()*3;
	}
	if (opt.motion)
	{
		int f = 0;
		for (auto frame = target.frames.begin(); frame != target.frames.end(); frame++, f++)
		{
			calibParams.template segment<3>(base+f*6+0) = (frame->pose.translation().template cast<Scalar>() - norm.first) / norm.second;
			calibParams.template segment<3>(base+f*6+3) = EncodeAARot<Scalar>(frame->pose.rotation().template cast<Scalar>());
		}
	}
}


/**
 * Calculates the error distribution stats, accounting for the number of outliers
 */
template<typename Derived>
static inline OptErrorRes getErrorStats(Eigen::MatrixBase<Derived> &errors, int outliers = 0)
{
	if (errors.size() == 0)
		return OptErrorRes();	

	OptErrorRes error;
	error.num = errors.size()-outliers;
	error.max = errors.maxCoeff();
	error.mean = errors.sum() / error.num;
	error.rmse = std::sqrt(errors.cwiseProduct(errors).sum() / error.num);	
	error.stdDev = 0;
	if (outliers == 0)
	{
		error.stdDev = (errors.array() - error.mean).square().sum();
	}
	else
	{
		for (int i = 0; i < errors.size(); i++)
			if (errors(i) > 0) // Outlier
				error.stdDev += (errors(i) - error.mean)*(errors(i) - error.mean);
	}
	error.stdDev = std::sqrt(error.stdDev / (error.num - 1));
	return error;
}

/**
 * Returns the avg, stdDev and Max of the given errorVec, accounting for the number of outliers, and optionally debugging the results including number of 1px and 10px outliers
 */
template<typename Scalar>
static OptErrorRes logErrorStats(const char* label, VectorX<Scalar> &errorVec, int outliers = 0, int removedOutliers = 0)
{
	if (errorVec.size() == 0)
		return {};	

	if (errorVec.hasNaN())
	{
		LOGC(LWarn, "%s: Error contains NANs!\n", label);
		return { 0, NAN, NAN, NAN, NAN };
	}

	OptErrorRes error = getErrorStats(errorVec, outliers);

	if (SHOULD_LOGCL())
	{
		Scalar optError = errorVec.cwiseProduct(errorVec).stableNorm();
		int over1PxCount = (errorVec.array() > (1.0 / PixelFactor)).count();
		int over10PxCount = (errorVec.array() > (10.0 / PixelFactor)).count();
		LOGCL("%s: %lf opt -- %fpx +- %fpx, %fpx max -- RMSE %lfpx -- Of %d, %d > 10px, %d > 1px, %d < 1px, %d outliers\n",
			label, optError, error.mean*PixelFactor, error.stdDev*PixelFactor, error.max*PixelFactor, error.rmse*PixelFactor,
			(int)errorVec.size()+removedOutliers, over10PxCount, over1PxCount, (int)errorVec.size()-over1PxCount-outliers, outliers+removedOutliers);
	}

	return error;
}

#endif // OPTIMISATION_UTIL_H