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

#ifndef REPROJECTION_ERROR_H
#define REPROJECTION_ERROR_H

#include "utilities.hpp"
#include "calib/optimisation.hpp"
#include "calib/obs_data.hpp"
#include "point/triangulation.hpp"

#include "target/TargetReprojectionError.hpp"

#include "util/eigendef.hpp"

// TODO: Temporary. Remove debugging/timing
#include "util/util.hpp"
#include "util/log.hpp"

//#define OPT_AUTODIFF // Would need to be set for point/triangulation.cpp as well
#include "differentiation.hpp"

#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

#include "unsupported/Eigen/NonLinearOptimization"

#include <cassert>



template<typename Scalar>
using TargetStructure = std::vector<Vector3<Scalar>>;

template<typename Scalar>
struct TargetParams
{
	TargetStructure<Scalar> structure;
	std::vector<Isometry3<Scalar>> motion;
};

template<typename Scalar = ScalarInternal, int Options = OptNone>
struct ReprojectionError
{
	enum
	{
		InputsAtCompileTime = Eigen::Dynamic,
		ValuesAtCompileTime = Eigen::Dynamic
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	// Information about initial state
	std::vector<CameraCalib_t<Scalar>> m_cameras;
	Scalar m_initialScale;
	// Information about the dataset
	const ObsData * const m_data;
	int m_totalSamples, m_pointSamples, m_targetSamples;
	// Information about optimised parameters
	int m_paramCount, m_tgtParamIndex, m_paramCameras;
	std::vector<int> m_paramTarget, m_errorTarget;
	OptimisationOptions m_options;

	std::vector<VectorX<Scalar>> m_tgtPosesVec;

	ReprojectionError(const std::vector<CameraCalib_t<Scalar>> &cameras, const ObsData &data, bool ignoreOutliers = true) : m_data(&data)
	{
		m_cameras = cameras;
		m_pointSamples = 0;
		m_targetSamples = 0;
		m_options = {};
		m_paramCount = 0;

	#ifdef NORMALIZE_SCALE
		m_initialScale = 0;
		for (int c = 0; c < m_cameras.size(); c++)
			m_initialScale += m_cameras[c].transform.translation().norm();
		m_initialScale /= m_cameras.size();
		// TODO: Consider using a stick with 2 markers of known distance
		// Then fix to a scale where stick length is accurate, OR take stick length as average
	#endif

		m_pointSamples = m_data->points.totalSamples;
		m_errorTarget.reserve(m_data->targets.size()+1);
		for (auto &tgt : m_data->targets)
		{
			m_errorTarget.push_back(m_pointSamples+m_targetSamples);
			m_targetSamples += tgt.totalSamples;
		}
		m_errorTarget.push_back(m_pointSamples+m_targetSamples);
		m_totalSamples = m_pointSamples + m_targetSamples;

		if constexpr ((Options & OptTgtMotionOpt) == OptTgtMotionOpt)
		{ // Read pose estimates from db
			m_tgtPosesVec.resize(m_data->targets.size());
			int targetIndex = 0;
			for (auto &tgt : m_data->targets)
				readInitialPoseVec(tgt, m_tgtPosesVec[targetIndex++]);
		}
	}

	void setOptimisationOptions(OptimisationOptions options)
	{
		m_options = options;
		m_paramCount = m_paramCameras = m_tgtParamIndex = numCamParams(options, m_cameras.size());
		m_paramTarget.clear();
		m_paramTarget.reserve(m_data->targets.size());
		for (auto &target : m_data->targets)
		{
			int params = numTgt(options, target);
			m_paramTarget.push_back(m_paramCount);
			m_paramCount += params;
		}
		m_paramTarget.push_back(m_paramCount);
	}

	int inputs() const { return m_paramCount; }
	int values() const { return m_totalSamples; }

	void calculateError(const std::vector<CameraCalib_t<Scalar>> &cameras, VectorX<Scalar> &errors) const
	{
		// Calculate point errors
		calculatePointErrors<Scalar>(cameras, errors.segment(0, m_pointSamples));

		int tgtIndex = 0;
		for (auto &target : m_data->targets)
		{
			thread_local TargetParams<Scalar> targetParams;

			// Copy structure data
			targetParams.structure.resize(target.markers.size());
			for (int m = 0; m < target.markers.size(); m++)
				targetParams.structure[m] = target.markers[m].template cast<Scalar>();

			// Copy motion data
			targetParams.motion.resize(target.frames.size());
			for (int f = 0; f < target.frames.size(); f++)
				targetParams.motion[f] = target.frames[f].pose.template cast<Scalar>();

			// Calculate target errors
			int errorStart = m_errorTarget[tgtIndex], errorLen = m_errorTarget[tgtIndex+1]-errorStart;
			calculateTargetErrors<Scalar>(cameras, target, targetParams, errors.segment(errorStart, errorLen));

			tgtIndex++;
		}
	}

	void updateTargetMotions(ObsData &data)
	{
		static_assert((Options & OptTgtMotionOpt) == OptTgtMotionOpt);
		assert(data.targets.size() == m_tgtPosesVec.size());
		int targetIndex = 0;
		for (auto &target : data.targets)
			writeUpdatedPoseVec<Scalar>(target, m_tgtPosesVec[targetIndex++]);
	}

	template<typename DiffScalar = Scalar>
	int operator()(const VectorX<DiffScalar> &calibParams, VectorX<DiffScalar> &errors)
	{
		// Setup camera parameters
		std::vector<CameraCalib_t<DiffScalar>> cameras(m_cameras.size());
		for (int c = 0; c < m_cameras.size(); c++)
			cameras[c] = (CameraCalib_t<DiffScalar>)m_cameras[c];
		setCameraNorms<DiffScalar>(cameras, m_options, m_initialScale);

		readCameraParameters<DiffScalar>(cameras, m_options, calibParams);

		// Calculate point errors
		calculatePointErrors<DiffScalar>(cameras, errors.segment(0, m_pointSamples));

		// Prepare target parameters and calculate errors
		int tgtIndex = 0;
		for (auto &target : m_data->targets)
		{
			int paramStart = m_paramTarget[tgtIndex], paramLen = m_paramTarget[tgtIndex+1]-paramStart;
			int errorStart = m_errorTarget[tgtIndex], errorLen = m_errorTarget[tgtIndex+1]-errorStart;

			if constexpr ((Options & OptTgtMotionOpt) == OptTgtMotionOpt)
			{
				// Read target structure from params
				thread_local TargetStructure<DiffScalar> tgtStruct;
				readTargetStructure<DiffScalar>(tgtStruct, target, calibParams.segment(paramStart, paramLen));

				// Optimise target errors and write back poses to m_tgtPosesVec for further optimisation
				if constexpr (std::is_same_v<DiffScalar, Scalar>)
				{
					auto stats = optimiseTargetPoseErrors<Scalar>(cameras, target, tgtStruct, m_tgtPosesVec[tgtIndex], errors.segment(errorStart, errorLen), 1);
					LOGC(LDebug, "    Nested target optimisation stopped with code %d (%f%%)", stats.first, stats.second*100);
				}
				else
				{
					VectorX<DiffScalar> posesVec = m_tgtPosesVec[tgtIndex].template cast<DiffScalar>();
					auto stats = optimiseTargetPoseErrors<DiffScalar>(cameras, target, tgtStruct, posesVec, errors.segment(errorStart, errorLen), 1);
					LOGC(LDebug, "    Nested target optimisation stopped with code %d (%f%%)", stats.first, stats.second*100);
					m_tgtPosesVec[tgtIndex] = posesVec.template cast<Scalar>();
				}
			}
			else
			{

				// Read target structure and motion from params 
				thread_local TargetParams<DiffScalar> targetParams;
				readTargetParams<DiffScalar>(targetParams, target, calibParams.segment(paramStart, paramLen));

				// Calculate target errors over all frames
				calculateTargetErrors<DiffScalar>(cameras, target, targetParams, errors.segment(errorStart, errorLen));
			}

			tgtIndex++;
		}

		return 0;
	}

	int df(const VectorX<Scalar> &calibParams, JacobianType &jacobian) const
	{
		TimePoint_t s1, s2;

		/* s1 = sclock::now();tgtIndex
		NumericDiffJacobian<Scalar>([this](const VectorX<Scalar> &calibParams, VectorX<Scalar> &errors)
		{
			if constexpr (Options%OptTgtMotionOpt)
				#warning "Nested optimisation doesn't work well here, since even a jacobian update would update the best pose estimation. Not optimal, but works"
			return operator()(calibParams, errors);
		}, calibParams, jacobian, 100);
		s2 = sclock::now(); */

		//JacobianType numericFullJac = jacobian;
		//LOGC(LTrace, "NumericDiff Full: norm %f in %fms\n", jacobian.norm(), dt(s1, s2));

#ifdef OPT_AUTODIFF

/* 		s1 = sclock::now();
		AutoDiffJacobian<Scalar>([this](const VectorX<AutoDiffScalar<Scalar>> &calibParams, VectorX<AutoDiffScalar<Scalar>> &errors)
		{
			return operator()(calibParams, errors);
		}, calibParams, jacobian);
		s2 = sclock::now();

		JacobianType autoFullJac = jacobian;
		LOGC(LTrace, "AutoDiff Full: norm %f in %fms, diff to NumericDiff %f\n", jacobian.norm(), dt(s1, s2), (numericFullJac-autoFullJac).array().mean());
 */


/* 		s1 = sclock::now();

		jacobian.setZero();

		// Setup camera parameters
		typedef AutoDiffScalar<Scalar> ActiveScalar;
		VectorX<ActiveScalar> cameraParam = calibParams.head(m_paramCameras).template cast<ActiveScalar>();
	
		{ // Calculate point error derivatives w.r.t. camera parameters
			for (int i = 0; i < m_paramCameras; i++)
				cameraParam[i].derivatives() = VectorX<Scalar>::Unit(m_paramCameras, i);

			// Read camera parameters
			std::vector<CameraCalib_t<ActiveScalar>> cameras(m_cameras.size());
			for (int c = 0; c < m_cameras.size(); c++)
				cameras[c] = (CameraCalib_t<ActiveScalar>)m_cameras[c];
			readCameraParameters<ActiveScalar>(cameras, m_options, cameraParam);
			setCameraNorms<ActiveScalar>(cameras, m_options, (ActiveScalar)m_initialScale);

	
			VectorX<ActiveScalar> pointErrors(m_pointObsCount);
			calculatePointErrors<ActiveScalar>(cameras, pointErrors);

			for (int i = 0; i < m_pointObsCount; i++)
				jacobian.block(i, 0, 1, m_paramCameras) = pointErrors[i].derivatives();
		}

		// Prepare target parameters and calculate errors
		int tgtIndex = 0;
		for (auto &target : m_data->targets)
		{
			thread_local TargetParams<ActiveScalar> targetParams;
			int start = m_paramTarget[tgtIndex], length = m_paramTarget[tgtIndex+1]-start;

			// Prepare active target params
			VectorX<ActiveScalar> tgtParams = calibParams.segment(start, length).template cast<ActiveScalar>()
			for (int i = 0; i < length; i++)
				tgtParams[i].derivatives() = VectorX<Scalar>::Unit(m_paramCameras+length, i);

			// Load target structure and motion from params 
			readTargetParams<ActiveScalar>(targetParams, target, tgtParams);

			// Calculate target errors over all frames
			VectorX<ActiveScalar> targetErrors(length);
			calculateTargetErrors<DiffScalar>(cameras, target, targetParams, targetErrors);

			// Update jacobians
			for (int i = 0; i < m_pointObsCount; i++)
				jacobian.block(i, 0, 1, m_paramCameras) = pointErrors[i].derivative();



			tgtIndex++;
		}

		s2 = sclock::now();

		JacobianType autoPartJac = jacobian;
		LOGC(LTrace, "AutoDiff Part: norm %f in %fms, diff to NumericDiff %f\n", jacobian.norm(), dt(s1, s2), (numericFullJac-autoPartJac).array().mean());
 */
#endif

		s1 = sclock::now();

		jacobian.setZero();

		// Setup camera parameters
		std::vector<CameraCalib_t<Scalar>> cameras(m_cameras.size());
		for (int c = 0; c < m_cameras.size(); c++)
			cameras[c] = (CameraCalib_t<Scalar>)m_cameras[c];
		setCameraNorms<Scalar>(cameras, m_options, m_initialScale);

		// Camera parameters for point errors
		NumericDiffJacobian<Scalar, NumericDiffForward, (Options&OptParallel) == OptParallel>([&](const VectorX<Scalar> &params, VectorX<Scalar> &errors)
		{
			if constexpr ((Options & OptParallel) == OptParallel)
			{
				auto camerasCpy = cameras;
				readCameraParameters<Scalar>(camerasCpy, m_options, params);
				calculatePointErrors<Scalar>(camerasCpy, errors);
			}
			else
			{
				readCameraParameters<Scalar>(cameras, m_options, params);
				calculatePointErrors<Scalar>(cameras, errors);
			}
		}, calibParams.head(m_paramCameras), jacobian.block(0, 0, m_pointSamples, m_paramCameras));

		// Prepare target parameters and calculate errors
		std::vector<TargetParams<Scalar>> targetParams(m_data->targets.size());
		int tgtIndex = 0;
		for (auto &target : m_data->targets)
		{
			int paramStart = m_paramTarget[tgtIndex], paramLen = m_paramTarget[tgtIndex+1]-paramStart;
			readTargetParams<Scalar>(targetParams[tgtIndex], target, calibParams.segment(paramStart, paramLen));
			tgtIndex++;
		}

		// Camera parameters for target errors
		NumericDiffJacobian<Scalar, NumericDiffForward, (Options&OptParallel) == OptParallel>([&](const VectorX<Scalar> &params, VectorX<Scalar> &errors)
		{
			readCameraParameters<Scalar>(cameras, m_options, params);
			int tgtIndex = 0, errorIndex = 0;
			for (auto &target : m_data->targets)
			{
				calculateTargetErrors<Scalar>(cameras, target, targetParams[tgtIndex], errors.segment(errorIndex, target.totalSamples));
				errorIndex += target.totalSamples;
				tgtIndex++;
			}
		}, calibParams.head(m_paramCameras), jacobian.block(m_pointSamples, 0, m_targetSamples, m_paramCameras));

		if constexpr ((Options & OptTgtMotionOpt) == OptTgtMotionOpt)
		{ // Structure parameters for target errors
			int tgtIndex = 0;
			for (auto &target : m_data->targets)
			{
				VectorX<Scalar> posesVec = m_tgtPosesVec[tgtIndex];

				int paramStart = m_paramTarget[tgtIndex], paramLen = m_paramTarget[tgtIndex+1]-paramStart;
				int errorStart = m_errorTarget[tgtIndex], errorLen = m_errorTarget[tgtIndex+1]-errorStart;

				std::vector<float> statAccum(10);
				// (Options&OptParallel) == OptParallel
				NumericDiffJacobian<Scalar, NumericDiffForward, (Options&OptParallel) == OptParallel>([&](const VectorX<Scalar> &params, VectorX<Scalar> &errors)
				{
					TargetStructure<Scalar> tgtStruct = targetParams[tgtIndex].structure;
					readTargetStructure<Scalar>(tgtStruct, target, params);
					auto stats = optimiseTargetPoseErrors<Scalar>(cameras, target, tgtStruct, posesVec, errors, 1);
					//statAccum[stats.first] += stats.second;
				}, calibParams.segment(paramStart, paramLen), jacobian.block(errorStart, paramStart, errorLen, paramLen), 10);

				/* int maxCode = 0;
				float sumStat = 0;
				for (int i = 0; i < 10; i++)
				{
					sumStat += statAccum[i];
					if (statAccum[maxCode] < statAccum[i])
						maxCode = i;
				}
				LOGC(LDebug, "    Nested target diff optimisation stopped with code %d (%f%%)", maxCode, statAccum[maxCode]/sumStat*100); */

				tgtIndex++;
			}
		}
		else
		{
			if (m_options.structure)
			{ // Structure parameters for target errors
				int tgtIndex = 0;
				for (auto &target : m_data->targets)
				{
					int tgtStructureCnt = numTgtStruct(m_options, target);
					int paramStart = m_paramTarget[tgtIndex], paramLen = tgtStructureCnt;
					int errorStart = m_errorTarget[tgtIndex], errorLen = m_errorTarget[tgtIndex+1]-errorStart;

					// (Options&OptParallel) == OptParallel
					NumericDiffJacobian<Scalar, NumericDiffForward, false>([&](const VectorX<Scalar> &params, VectorX<Scalar> &errors)
					{
						TargetParams<Scalar> tgtParam = targetParams[tgtIndex];
						readTargetStructure<Scalar>(tgtParam.structure, target, params);
						calculateTargetErrors<Scalar>(cameras, target, tgtParam, errors);
					}, calibParams.segment(paramStart, paramLen), jacobian.block(errorStart, paramStart, errorLen, paramLen));

					tgtIndex++;
				}
			}

			if (m_options.motion)
			{ // Motion parameters for target errors
				int tgtIndex = 0;
				for (auto &target : m_data->targets)
				{
					int errorIndex = m_errorTarget[tgtIndex];
					int paramIndex = m_paramTarget[tgtIndex] + numTgtStruct(m_options, target);
					TargetParams<Scalar> tgtParam = targetParams[tgtIndex];
					/* if constexpr ((Options & OptParallel) == OptParallel)
					{
						std::vector<std::pair<int,int>> tgtFrameIndices;
						tgtFrameIndices.reserve(target.frames.size()+1);
						int sampleIndex = 0;
						for (int f = 0; f < target.frames.size(); f++)
						{
							tgtFrameIndices.push_back(std::make_pair(f, sampleIndex));
							sampleIndex += target.frames[f].samples.size();
						}
					#pragma omp parallel for schedule(dynamic)
						for (int f = 0; f < tgtFrameIndices.size(); f++)
						{
							int frameIndex = tgtFrameIndices[f].first;
							int errorStart = errorIndex+tgtFrameIndices[f].second, errorLen = target.frames[frameIndex].samples.size(), paramStart = paramIndex+f*6;
							NumericDiffJacobian<Scalar>([&](const VectorX<Scalar> &params, VectorX<Scalar> &errors)
							{
								calculateTargetObsErrors<Scalar>(cameras, target, tgtParam.structure, frameIndex, DecodeAAPose<Scalar>(params), errors);
							}, calibParams.segment(paramStart, 6), jacobian.block(errorStart, paramStart, errorLen, 6));
						}
						errorIndex += target.totalSamples;
						paramIndex += target.frames.size()*6;
					}
					else */
					{
						for (int f = 0; f < target.frames.size(); f++)
						{
							int errorLen = target.frames[f].samples.size();
							NumericDiffJacobian<Scalar>([&](const VectorX<Scalar> &params, VectorX<Scalar> &errors)
							{
								calculateTargetObsErrors<Scalar>(cameras, target, tgtParam.structure, f, DecodeAAPose<Scalar>(params), errors);
							}, calibParams.segment(paramIndex, 6), jacobian.block(errorIndex, paramIndex, errorLen, 6));
							errorIndex += errorLen;
							paramIndex += 6;
						}
					}
					tgtIndex++;
				}
			}
		}

		s2 = sclock::now();

		//JacobianType numericPartJac = jacobian;
		//LOGC(LTrace, "NumericDiff Part: norm %f in %fms\n", jacobian.norm(), dt(s1, s2));

		//jacobian = numericFullJac;

		return 1; // <0 : abort, =0 : success, >0 : num function evaluation
	}

	int dfs(const VectorX<Scalar> &calibParams, Eigen::SparseMatrix<Scalar> &jacobian) const
	{
		JacobianType jacDense(m_totalSamples, m_paramCount);
		df(calibParams, jacDense);
		jacobian.setZero();
		for (int i = 0; i < m_totalSamples; i++)
		{
			for (int j = 0; j < m_paramCount; j++)
			{
				if (jacDense(i,j) != 0.0)
				{
					jacobian.insert(i,j) = jacDense(i,j);
				}
			}
		}
		jacobian.makeCompressed();
		return 1;
	}

	/**
	 * Calculate reprojection errors for each 2D observaction given the current new parameters for camera and target motion
	 */
	template<typename ErrorScalar = Scalar>
	void calculatePointErrors(const std::vector<CameraCalib_t<ErrorScalar>> &cameras, Eigen::Ref<VectorX<ErrorScalar>> errors) const
	{
		// Setup interfacing structures for triangulating points
		std::vector<std::vector<Vector2<ErrorScalar>>> blobContainer(cameras.size());
		std::vector<std::vector<Vector2<ErrorScalar>> const *> points2D(cameras.size());
		for (int c = 0; c < cameras.size(); c++)
		{
			blobContainer[c].resize(1);		 // Used to store a single blob for each camera involved with a point
			points2D[c] = &blobContainer[c]; // Just interfacing
		}
		TriangulatedPoint triPoint(Eigen::Vector3f::Zero(), 0, 10, cameras.size());

		int errorIndex = 0;
		for (auto &point : m_data->points.points)
		{
			if (m_options.ignoreOutliers && point.outlier)
			{
				for (auto &sample : point.samples)
					errors(errorIndex++) = 0;
				continue;
			}
			// Reset point
			for (int c = 0; c < cameras.size(); c++)
				triPoint.blobs[c] = InvalidBlob;
			// Fill point with involved blobs
			for (auto &sample : point.samples)
			{
				int c = sample.camera;
				Vector2<ErrorScalar> measurement = sample.point.template cast<ErrorScalar>();
				// Undistort point
				blobContainer[c][0] = undistortPoint(cameras[c], measurement);
				// Register as involved camera and blob
				triPoint.blobs[c] = 0;
			}
			// Calculate optimal triangulation
			//Vector3<ErrorScalar> tri = refineTriangulation<ErrorScalar, ErrorScalar, ErrorScalar>(points2D, cameras, triPoint);
			Vector3<ErrorScalar> tri = refineTriangulationIterative<ErrorScalar, ErrorScalar, ErrorScalar>(points2D, cameras, triPoint);
			
			// Reproject triangulation to calculate reprojection error
			for (auto &sample : point.samples)
			{
				int c = sample.camera;
				Vector2<ErrorScalar> point = projectPoint2D(cameras[c].camera, tri);
				errors(errorIndex++) = (blobContainer[c][0] - point).norm();
			}
		}

		assert(errorIndex == errors.size());
	}

	template<typename LoadScalar = Scalar>
	inline int readTargetStructure(TargetStructure<LoadScalar> &tgtStruct, const ObsTarget &tgtData, const Eigen::Ref<const VectorX<LoadScalar>> paramSegment) const
	{
		int paramIndex = 0;
		// Read or copy structure data
		tgtStruct.resize(tgtData.markers.size());
		if (m_options.structure)
		{
			for (int m = 0; m < tgtData.markers.size(); m++)
				tgtStruct[m] = paramSegment.template segment<3>(paramIndex+m*3);
			paramIndex += tgtData.markers.size()*3;
		}
		else
		{
			for (int m = 0; m < tgtData.markers.size(); m++)
				tgtStruct[m] = tgtData.markers[m].template cast<LoadScalar>();
		}
		return paramIndex;
	}

	template<typename LoadScalar = Scalar>
	inline void readTargetParams(TargetParams<LoadScalar> &tgtParams, const ObsTarget &tgtData, const Eigen::Ref<const VectorX<LoadScalar>> paramSegment) const
	{
		int paramIndex = readTargetStructure(tgtParams.structure, tgtData, paramSegment);
		// Read or copy motion data
		tgtParams.motion.resize(tgtData.frames.size());
		if (m_options.motion)
		{
			for (int f = 0; f < tgtData.frames.size(); f++)
				tgtParams.motion[f] = DecodeAAPose<LoadScalar>(paramSegment.template segment<6>(paramIndex+f*6));
			paramIndex += tgtData.frames.size()*6;
		}
		else
		{
			auto frame = tgtData.frames.begin();
			for (int f = 0; f < tgtData.frames.size(); f++)
				tgtParams.motion[f] = (frame++)->pose.template cast<LoadScalar>();
		}
	}

	template<typename LoadScalar = Scalar>
	inline void readInitialPoseVec(const ObsTarget &tgtData, VectorX<LoadScalar> &posesVec) const
	{
		posesVec.resize(tgtData.frames.size()*6);
		int f = 0;
		for (auto &frame : tgtData.frames)
			posesVec.template segment<6>((f++)*6) = EncodeAAPose(frame.pose).template cast<LoadScalar>();
	}

	template<typename LoadScalar = Scalar>
	inline void writeUpdatedPoseVec(ObsTarget &tgtData, const VectorX<LoadScalar> &posesVec) const
	{
		assert(posesVec.size() == tgtData.frames.size()*6);
		int f = 0;
		for (auto &frame : tgtData.frames)
			frame.pose = DecodeAAPose<float>(posesVec.template segment<6>((f++)*6).template cast<LoadScalar>());
	}

	template<typename ErrorScalar = Scalar>
	inline void calculateTargetObsErrors(const std::vector<CameraCalib_t<ErrorScalar>> &cameras, const ObsTarget &tgt, 
		const TargetStructure<ErrorScalar> &tgtStruct, int frameIndex, const Isometry3<ErrorScalar> &motion,
		Eigen::Ref<VectorX<ErrorScalar>> errors) const
	{

		int errorIndex = 0;
		const ObsTargetFrame &frame = tgt.frames[frameIndex];
		assert(errors.size() == frame.samples.size());
		for (const ObsTargetSample &sample : frame.samples)
		{ // Reproject point observation of given marker into given camera at current frame
			Vector3<ErrorScalar> marker = tgtStruct[tgt.markerMap.at(sample.marker)].template cast<ErrorScalar>();
			Vector2<ErrorScalar> measurement = sample.point.template cast<ErrorScalar>();
			Vector2<ErrorScalar> proj = projectPoint2D(cameras[sample.camera].camera, motion * marker);
			Vector2<ErrorScalar> point = undistortPoint(cameras[sample.camera], measurement);
			errors(errorIndex++) = (point - proj).norm();
		}
	}

	template<typename ErrorScalar = Scalar>
	inline void calculateTargetErrors(const std::vector<CameraCalib_t<ErrorScalar>> &cameras, const ObsTarget &target, 
		const TargetParams<ErrorScalar> &tgtParams, Eigen::Ref<VectorX<ErrorScalar>> errors) const
	{
		assert(errors.size() == target.totalSamples);
		assert(tgtParams.motion.size() == target.frames.size());
		assert(tgtParams.structure.size() == target.markers.size());

		/* int errorIndex = 0;
		for (int f = 0; f < target.frames.size(); f++)
		{ // Reproject a target at a specific frame
			calculateTargetObsErrors(cameras, target, tgtParams.structure, f, tgtParams.motion[f], errors.segment(errorIndex, target.frames[f].samples.size()));
			errorIndex += target.frames[f].samples.size();
		} */
		int errorIndex = 0;
		for (int f = 0; f < target.frames.size(); f++)
		{ // Reproject a target at a specific frame
			for (auto &sample : target.frames[f].samples)
			{ // Reproject point observation of given marker into given camera at current frame
				Vector3<ErrorScalar> marker = tgtParams.structure[target.markerMap.at(sample.marker)].template cast<ErrorScalar>();
				Vector2<ErrorScalar> measurement = sample.point.template cast<ErrorScalar>();
				Vector2<ErrorScalar> proj = projectPoint2D(cameras[sample.camera].camera, tgtParams.motion[f] * marker);
				Vector2<ErrorScalar> point = undistortPoint(cameras[sample.camera], measurement);
				errors(errorIndex++) = (point - proj).norm();
			}
		}
		assert(errorIndex == errors.size());
		assert(!errors.hasNaN());
	}

	template<typename ErrorScalar = Scalar>
	inline auto optimiseTargetPoseErrors(const std::vector<CameraCalib_t<ErrorScalar>> &cameras, const ObsTarget &tgt, 
		const TargetStructure<ErrorScalar> &tgtStruct, VectorX<ErrorScalar> &posesVec,
		Eigen::Ref<VectorX<ErrorScalar>> errors, float tolerances) const
	{
		assert(errors.size() == tgt.totalSamples);
		assert(posesVec.size() == tgt.frames.size()*6);

		// Initialise optimisation error term
		typedef TargetReprojectionError<ErrorScalar> TgtError;
		TgtError errorTerm(cameras);

		// Initialise optimisation algorithm
		Eigen::LevenbergMarquardt<TgtError, ErrorScalar> lm(errorTerm);
		lm.parameters.maxfev = 100; // Max number of evaluations of errorTerm, bounds for number of iterations
		float toleranceFactor = tolerances * 0.0001f;
		lm.parameters.xtol = toleranceFactor * 0.001 * PixelSize;
		lm.parameters.ftol = toleranceFactor * 0.0000001;
		lm.parameters.gtol = toleranceFactor * 0.0005;

		int errorIndex = 0;
		std::vector<int> statusCodes(10);
		for (int f = 0; f < tgt.frames.size(); f++)
		{ // Optimise the target pose for each frame
			// Copy measurements for errorTerm to use
			errorTerm.setData(tgtStruct, tgt, f);
			// Read pre-initialised pose estimate
			VectorX<ErrorScalar> poseVec = posesVec.template segment<6>(f*6);
			// Minimise fully (to specified tolerance)
			auto status = lm.minimize(poseVec);
			if (status >= 0) statusCodes[status]++;
			// Read back pose to use as future pose estimate
			posesVec.template segment<6>(f*6) = poseVec;
			// Write error values
			errors.segment(errorIndex, lm.fvec.size()) = lm.fvec;
			errorIndex += lm.fvec.size();
		}
		assert(errorIndex == errors.size());
		int status = std::max_element(statusCodes.begin(), statusCodes.end()) - statusCodes.begin();
		float perc = (float)statusCodes[status] / tgt.frames.size();
		return std::make_pair(status,perc);
	}
};

#endif // REPROJECTION_ERROR_H