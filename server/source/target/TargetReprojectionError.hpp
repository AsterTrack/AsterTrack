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

#ifndef TARGET_REPROJECTION_ERROR_H
#define TARGET_REPROJECTION_ERROR_H

//#define OPT_AUTODIFF
#include "calib/opt/differentiation.hpp"

#include "util/eigenutil.hpp" // MRP2Quat, DecodeAARot, etc.

#include <cassert>

enum TgtOptOptions {
	OptTgtNone = 0,
	OptOutliers = 1,
	OptCorrectivePose = 2,
	OptRotationMRP = 4,
	OptUndistorted = 8,
	OptReferencePose = 16,
};

template<typename Scalar = float, int Options = OptTgtNone>
struct TargetReprojectionError
{
	enum
	{
		InputsAtCompileTime = Eigen::Dynamic,
		ValuesAtCompileTime = Eigen::Dynamic
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	std::vector<int> m_cams;
	std::vector<CameraCalib_t<Scalar>> m_calibs;
	std::vector<Projective3<Scalar>> m_mvps;
	std::vector<std::tuple<int, Vector3<Scalar>, Vector2<Scalar>>> m_observedPoints;
	std::vector<bool> *m_outlierMap;
	Eigen::Matrix<Scalar,6,1> refPose;
	Scalar refInfluence = 1.0f;

	template<typename LoadScalar = Scalar>
	TargetReprojectionError(const std::vector<CameraCalib_t<LoadScalar>> &calibs)
	{
		m_outlierMap = nullptr;
		int maxCameraIndex = 0;
		for (const auto &calib : calibs)
		{
			assert(calib.valid());
			if (calib.index > maxCameraIndex)
				maxCameraIndex = calib.index;
		}
		m_cams.reserve(calibs.size());
		m_calibs.resize(maxCameraIndex+1);
		m_mvps.resize(maxCameraIndex+1);
		for (const auto &calib : calibs)
		{
			m_calibs[calib.index] = (CameraCalib_t<Scalar>)calib; // Converting Scalar here
			m_mvps[calib.index] = m_calibs[calib.index].camera;
			m_cams.push_back(calib.index);
		}
	}

	void setReferencePose(const Isometry3<Scalar> &pose, Scalar influence = 1.0f)
	{
		static_assert(Options&OptReferencePose);
		refPose.template head<3>() = pose.translation();
		if constexpr (Options&OptRotationMRP)
			refPose.template tail<3>() = Quat2MRP(Eigen::Quaternion<Scalar>(pose.rotation()));
		else
			refPose.template tail<3>() = EncodeAARot<Scalar>(pose.rotation());
		refInfluence = influence;
	}

	void setStartingPose(const Isometry3<Scalar> &pose)
	{
		if constexpr (Options&OptReferencePose)
			assert(!refPose.matrix().hasNaN());
		if constexpr (Options&OptCorrectivePose)
		{
			for (int c = 0; c < m_cams.size(); c++)
			{
				int index = m_cams[c];
				m_mvps[index] = m_calibs[index].camera * pose;
			}
			if constexpr (Options&OptReferencePose)
				refPose = refPose * pose.inverse();
		}
	}

#ifdef TARGET_TRACKING_2D_H
	void setData(const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const TargetMatch2D &match)
	{
		static_assert(Options&OptUndistorted);
		m_observedPoints.clear();
		m_observedPoints.reserve(match.error.samples);
		for (int c = 0; c < m_cams.size(); c++)
		{ // Only regard cameras for which we got the calibration
			int index = m_cams[c];
			// Though points2D should be empty anyways if we didn't get a calibration
			for (auto &pts : match.points2D[index])
				m_observedPoints.push_back({ index,
					match.targetTemplate->markers[pts.first].pos.template cast<Scalar>(),
					points2D[c]->at(pts.second).template cast<Scalar>() // Already undistorted
				});
		}
		setStartingPose(match.pose.template cast<Scalar>());
	}
#endif

#ifdef OBS_DATA_H
	template<typename LoadScalar = float>
	void setData(const std::vector<Vector3<LoadScalar>> &points3D, const ObsTarget &target, int f)
	{
		static_assert(!(Options&OptUndistorted));
		const ObsTargetFrame &frame = target.frames[f];
		m_observedPoints.clear();
		m_observedPoints.reserve(frame.samples.size());
		for (const ObsTargetSample &sample : frame.samples)
		{ // Assumes we previously got the calibrations for all relevant cameras
			int m = target.markerMap.at(sample.marker);
			m_observedPoints.push_back({ sample.camera,
				points3D[m].template cast<Scalar>(),
				sample.point.template cast<Scalar>()  // Not yet undistorted
			});
		}
		setStartingPose(frame.pose.template cast<Scalar>());
	}
#endif

	template<typename DiffScalar = Scalar>
	int operator()(const VectorX<DiffScalar> &poseVec, VectorX<DiffScalar> &errors) const
	{
		Isometry3<DiffScalar> pose;
		pose.translation() = poseVec.template head<3>();
		if constexpr (Options&OptRotationMRP)
			pose.linear() = MRP2Quat(poseVec.template tail<3>()).toRotationMatrix();
		else
			pose.linear() = DecodeAARot<DiffScalar>(poseVec.template tail<3>());

		for (int p = 0; p < m_observedPoints.size(); p++)
		{
			if ((Options&OptOutliers) && m_outlierMap && m_outlierMap->at(p))
			{
				errors(p) = 0;
				continue;
			}
			auto &pt = m_observedPoints[p];
			int c = std::get<0>(pt);
			Vector3<DiffScalar> tgtPt = std::get<1>(pt).template cast<DiffScalar>();
			Vector2<DiffScalar> proj = projectPoint2D(m_mvps[c], pose * tgtPt);
			Vector2<DiffScalar> measurement = std::get<2>(pt).template cast<DiffScalar>();
			if constexpr (!(Options&OptUndistorted))
				measurement = undistortPoint(m_calibs[c], measurement);
			errors(p) = (proj - measurement).norm();
		}

		if constexpr (Options&OptReferencePose)
		{
			errors.template tail<6>() = refInfluence * (poseVec-refPose).array().square();
		}

		return 0;
	}

	int df(const VectorX<Scalar> &poseVec, JacobianType &jacobian) const
	{
		NumericDiffJacobian<Scalar>([this](const VectorX<Scalar> &poseVec, VectorX<Scalar> &errors)
		{
			return operator()(poseVec, errors);
		}, poseVec, jacobian);

		return 0; // <0 : abort, =0 : success, >0 : num function evaluation
	}

	int inputs() const { return 6; }
	int values() const { return m_observedPoints.size() + ((Options&OptReferencePose)? 6 : 0); }
};

#endif // REPROJECTION_ERROR_H