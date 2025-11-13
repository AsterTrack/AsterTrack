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
#include "calib/opt/covariance.hpp"

#include "util/eigenutil.hpp" // MRP2Quat, DecodeAARot, etc.

#include "flexkalman/EigenQuatExponentialMap.h" // quat_exp, quat_ln

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
	std::vector<bool> m_outlierMap;
	Eigen::Matrix<Scalar,6,1> refPose;
	Scalar refInfluence = 1.0f;

	template<typename LoadScalar = Scalar>
	TargetReprojectionError(const std::vector<CameraCalib_t<LoadScalar>> &calibs)
	{
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

	template<typename DiffScalar = Scalar>
	VectorX<DiffScalar> encodePose(const Isometry3<DiffScalar> &pose) const
	{
		if constexpr (Options&OptRotationMRP)
			return EncodeMRPPose<DiffScalar>(pose);
		else
			return EncodeAAPose<DiffScalar>(pose);
	}

	template<typename DiffScalar = Scalar, typename Derived>
	Isometry3<DiffScalar> decodePose(const Eigen::MatrixBase<Derived> &poseVec) const
	{
		if constexpr (Options&OptRotationMRP)
			return DecodeMRPPose<DiffScalar>(poseVec);
		else
			return DecodeAAPose<DiffScalar>(poseVec);
	}

	void setReferencePose(const Isometry3<Scalar> &pose, Scalar influence = 1.0f)
	{
		static_assert(Options&OptReferencePose);
		refPose = encodePose(pose);
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
	void setData(const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, const TargetMatch2D &match, const TargetCalibration3D &calib)
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
					calib.markers[pts.first].pos.template cast<Scalar>(),
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

	/*
	 * For Optimisation
	 */

	template<typename DiffScalar = Scalar>
	DiffScalar calculateSampleError(const Isometry3<DiffScalar> &pose, int p) const
	{
		auto &pt = m_observedPoints[p];
		int c = std::get<0>(pt);
		Vector3<DiffScalar> tgtPt = std::get<1>(pt).template cast<DiffScalar>();
		Vector2<DiffScalar> proj = projectPoint2D(m_mvps[c], pose * tgtPt);
		Vector2<DiffScalar> measurement = std::get<2>(pt).template cast<DiffScalar>();
		if constexpr (!(Options&OptUndistorted))
			measurement = undistortPoint(m_calibs[c], measurement);
		return (proj - measurement).norm();
		// TODO: Switch to squaredNorm to save on many sqrts in tracking
		// Previous attempts resulted in optimisation that couldn't optimise (did not abort early, just no progress)
	}

	template<typename DiffScalar = Scalar>
	void calculateSampleErrors(const Isometry3<DiffScalar> &pose, VectorX<DiffScalar> &errors) const
	{
		for (int p = 0; p < m_observedPoints.size(); p++)
		{
			if ((Options&OptOutliers) && m_outlierMap.at(p))
				errors(p) = 0;
			else
			 	errors(p) = calculateSampleError(pose, p);
		}
	}

	template<typename DiffScalar = Scalar>
	int operator()(const VectorX<DiffScalar> &poseVec, VectorX<DiffScalar> &errors) const
	{
		Isometry3<DiffScalar> pose = decodePose(poseVec);

		calculateSampleErrors(pose, errors);

		if constexpr (Options&OptReferencePose)
		{
			errors.template tail<6>() = refInfluence * (poseVec-refPose).array().square();
		}

		return 0;
	}

	template<typename DiffScalar = Scalar>
	int df(const VectorX<DiffScalar> &poseVec, JacobianType &jacobian) const
	{
		NumericDiffJacobian<DiffScalar>([this](const VectorX<DiffScalar> &poseVec, VectorX<DiffScalar> &errors)
		{
			return operator()(poseVec, errors);
		}, poseVec, jacobian);

		return 0; // <0 : abort, =0 : success, >0 : num function evaluation
	}

	/*
	 * Filter Covariance
	 */

	template<typename DiffScalar = Scalar>
	Eigen::Matrix<DiffScalar,6,6> covarianceEXP(const Isometry3<DiffScalar> &pose, DiffScalar errorStdDev, std::vector<VectorX<DiffScalar>> &deviations) const
	{
		Eigen::Matrix<DiffScalar,6,1> poseVec;
		poseVec.template head<3>() = pose.translation();
		poseVec.template tail<3>() = flexkalman::util::quat_ln(Eigen::Quaternion<DiffScalar>(pose.rotation()));

		Eigen::Matrix<DiffScalar,6,6> covariance;
		VectorX<DiffScalar> errors(m_observedPoints.size());
		NumericCovariance<DiffScalar, false>([this, &errors](const VectorX<DiffScalar> &poseVec)
		{
			Isometry3<DiffScalar> pose;
			pose.translation() = poseVec.template head<3>();
			pose.linear() = flexkalman::util::quat_exp(poseVec.template tail<3>()).toRotationMatrix();
			calculateSampleErrors<DiffScalar>(pose, errors);
			return std::sqrt(errors.mean());
		}, poseVec, covariance, errorStdDev, 0.01f*PixelSize, 0.001f, deviations);
		return covariance;
	}

	/*
	 * Filter Measurement
	 */

	template<typename DiffScalar = Scalar>
	void calculatePointErrors(const Isometry3<DiffScalar> &pose, VectorX<DiffScalar> &errors) const
	{
		assert(errors.size() == m_observedPoints.size()*2);
		for (int p = 0; p < m_observedPoints.size(); p++)
		{
			auto &pt = m_observedPoints[p];
			int c = std::get<0>(pt);
			Vector3<DiffScalar> tgtPt = std::get<1>(pt).template cast<DiffScalar>();
			Vector2<DiffScalar> proj = projectPoint2D(m_mvps[c], pose * tgtPt);
			Vector2<DiffScalar> measurement = std::get<2>(pt).template cast<DiffScalar>();
			if constexpr (!(Options&OptUndistorted))
				measurement = undistortPoint(m_calibs[c], measurement);
			errors.template segment<2>(p*2) = proj - measurement;
		}
	}

	template<typename DiffScalar = Scalar>
	void calculatePointProjections(const Isometry3<DiffScalar> &pose, VectorX<DiffScalar> &projections) const
	{
		assert(projections.size() == m_observedPoints.size()*2);
		for (int p = 0; p < m_observedPoints.size(); p++)
		{
			auto &pt = m_observedPoints[p];
			int c = std::get<0>(pt);
			Vector3<DiffScalar> tgtPt = std::get<1>(pt).template cast<DiffScalar>();
			Vector2<DiffScalar> proj = projectPoint2D(m_mvps[c], pose * tgtPt);
			projections.template segment<2>(p*2) = proj;
		}
	}

	template<typename DiffScalar = Scalar>
	void getPointMeasurements(VectorX<DiffScalar> &measurements) const
	{
		assert(measurements.size() == m_observedPoints.size()*2);
		for (int p = 0; p < m_observedPoints.size(); p++)
		{
			auto &pt = m_observedPoints[p];
			int c = std::get<0>(pt);
			Vector2<DiffScalar> measurement = std::get<2>(pt).template cast<DiffScalar>();
			if constexpr (!(Options&OptUndistorted))
				measurement = undistortPoint(m_calibs[c], measurement);
			measurements.template segment<2>(p*2) = measurement;
		}
	}

	int inputs() const { return 6; }
	int values() const { return m_observedPoints.size() + ((Options&OptReferencePose)? 6 : 0); }
};

#endif // REPROJECTION_ERROR_H