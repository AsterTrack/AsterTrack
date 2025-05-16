/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TARGET_KALMAN_H
#define TARGET_KALMAN_H

#include "flexkalman/BaseTypes.h"
#include "flexkalman/FlexibleKalmanBase.h"
#include "flexkalman/EigenQuatExponentialMap.h"

#include "target/TargetReprojectionError.hpp"

/**
 * AbsolutePoseMeasurement for use with flexkalman UKF
 */

class AbsolutePoseMeasurement :
	public flexkalman::MeasurementBase<AbsolutePoseMeasurement>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr size_t Dimension = 6;
	using MeasurementVector = flexkalman::types::Vector<Dimension>;
	using MeasurementSquareMatrix = flexkalman::types::SquareMatrix<Dimension>;

	AbsolutePoseMeasurement(Eigen::Vector3d const &pos,
							Eigen::Quaterniond const &quat,
							Eigen::Vector3d const &posVariance,
							Eigen::Vector3d const &emVariance)
		: m_pos(pos), m_quat(quat)
	{
		m_covariance.setIdentity();
		m_covariance.diagonal().head<3>() = posVariance;
		m_covariance.diagonal().tail<3>() = emVariance;
	}

	AbsolutePoseMeasurement(Eigen::Vector3d const &pos,
							Eigen::Quaterniond const &quat,
							MeasurementSquareMatrix const &covariance)
		: m_pos(pos), m_quat(quat), m_covariance(covariance) {}

	template <typename State>
	MeasurementSquareMatrix const &getCovariance(State const &)
	{
		return m_covariance;
	}

	template <typename State>
	MeasurementVector predictMeasurement(State const &state) const
	{
		MeasurementVector prediction;
		prediction.head<3>() = state.position();
		prediction.tail<3>() = state.incrementalOrientation();
		return prediction;
	}

	template <typename State>
	MeasurementVector getResidual(MeasurementVector const &prediction,
								  State const &state) const
	{
		// The prediction we're given is effectively "the state's incremental
		// rotation", which is why we're using our measurement here as well as
		// the prediction.
		const Eigen::Quaterniond predictedQuat =
			flexkalman::util::quat_exp(prediction.tail<3>() / 2) * state.getQuaternion();
		MeasurementVector residual;
		residual.head<3>() = m_pos - prediction.head<3>();
		residual.tail<3>() = 2 * flexkalman::util::smallest_quat_ln(m_quat * predictedQuat.conjugate());
		return residual;
	}

	template <typename State>
	MeasurementVector getResidual(State const &state) const
	{
		const Eigen::Quaterniond predictedQuat = state.getCombinedQuaternion();
		// Two equivalent quaternions: but their logs are typically
		// different: one is the "short way" and the other is the "long
		// way". We'll compute both and pick the "short way".
		MeasurementVector residual;
		residual.head<3>() = m_pos - state.position();
		residual.tail<3>() = 2 * flexkalman::util::smallest_quat_ln(m_quat * predictedQuat.conjugate());
		return residual;
	}

	void setMeasurement(Eigen::Vector3d const &pos, Eigen::Quaterniond const &quat)
	{
		m_quat = quat;
		m_pos = pos;
	}

private:
	Eigen::Vector3d m_pos;
	Eigen::Quaterniond m_quat;
	MeasurementSquareMatrix m_covariance;
};

/**
 * FusedIMUMeasurement for use with flexkalman UKF
 */

class FusedIMUMeasurement :
	public flexkalman::MeasurementBase<FusedIMUMeasurement>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr size_t Dimension = 3;
	using MeasurementVector = flexkalman::types::Vector<Dimension>;
	using MeasurementSquareMatrix = flexkalman::types::SquareMatrix<Dimension>;

	FusedIMUMeasurement(Eigen::Quaterniond const &quat,
						flexkalman::types::Vector<3> const &emVariance)
		: m_quat(quat), m_covariance(emVariance.asDiagonal()) {}

	template <typename State>
	MeasurementSquareMatrix const &getCovariance(State const &)
	{
		return m_covariance;
	}

	template <typename State>
	MeasurementVector predictMeasurement(State const &state) const
	{
		return state.incrementalOrientation();
	}

	template <typename State>
	MeasurementVector getResidual(MeasurementVector const &prediction,
								  State const &state) const
	{
		// The prediction we're given is effectively "the state's incremental
		// rotation", which is why we're using our measurement here as well as
		// the prediction.
		const Eigen::Quaterniond predictedQuat =
			flexkalman::util::quat_exp(prediction / 2) * state.getQuaternion();
		return 2 * flexkalman::util::smallest_quat_ln(m_quat * predictedQuat.conjugate());
	}

	template <typename State>
	MeasurementVector getResidual(State const &state) const
	{
		const Eigen::Quaterniond predictedQuat = state.getCombinedQuaternion();
		// Two equivalent quaternions: but their logs are typically
		// different: one is the "short way" and the other is the "long
		// way". We'll compute both and pick the "short way".
		return 2 * flexkalman::util::smallest_quat_ln(m_quat * predictedQuat.conjugate());
	}

	void setMeasurement(Eigen::Quaterniond const &quat)
	{
		m_quat = quat;
	}

private:
	Eigen::Quaterniond m_quat;
	MeasurementSquareMatrix m_covariance;
};

/**
 * AbsolutePositionMeasurement for use with flexkalman UKF
 */

class AbsolutePositionMeasurement :
	public flexkalman::MeasurementBase<AbsolutePositionMeasurement>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr size_t Dimension = 3;
	using MeasurementVector = flexkalman::types::Vector<Dimension>;
	using MeasurementSquareMatrix = flexkalman::types::SquareMatrix<Dimension>;

	AbsolutePositionMeasurement(Eigen::Vector3d const &pos,
								Eigen::Vector3d const &variance)
		: m_pos(pos)
	{
		m_covariance.setIdentity();
		m_covariance.diagonal() = variance;
	}

	AbsolutePositionMeasurement(Eigen::Vector3d const &pos,
								MeasurementSquareMatrix const &covariance)
		: m_pos(pos), m_covariance(covariance) {}

	template <typename State>
	MeasurementSquareMatrix const &getCovariance(State const &)
	{
		return m_covariance;
	}

	template <typename State>
	MeasurementVector predictMeasurement(State const &state) const
	{
		return state.position();
	}

	template <typename State>
	MeasurementVector getResidual(MeasurementVector const &prediction,
								  State const &state) const
	{
		return m_pos - prediction.head<3>();
	}

	template <typename State>
	MeasurementVector getResidual(State const &state) const
	{
		return m_pos - state.position();
	}

	void setMeasurement(Eigen::Vector3d const &pos)
	{
		m_pos = pos;
	}

private:
	Eigen::Vector3d m_pos;
	MeasurementSquareMatrix m_covariance;
};

/**
 * TargetMatchMeasurement for use with flexkalman UKF/EKF
 * E.g. use point measurements directly to correct when full pose is underdetermined
 * See SCAAT method (though this is not necessarily implemented as SCAAT)
 */

template<int DIM = Eigen::Dynamic>
class TargetMatchMeasurement :
	public flexkalman::MeasurementBase<TargetMatchMeasurement<DIM>> {
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr int Dimension = DIM;
	using MeasurementVector = flexkalman::types::Vector<Dimension>;
	using MeasurementSquareMatrix = flexkalman::types::SquareMatrix<Dimension>;
	using TargetMatchError = TargetReprojectionError<double, OptUndistorted>;

	// TODO: If we end up using individual point samples (e.g. setting m_sample)
	// Then rewrite as static-sized measurement
	int fullSize() const { return DIM == Eigen::Dynamic? (m_targetError.values()*2) : DIM; }
	int size() const { return DIM == Eigen::Dynamic? (m_sample < 0? (m_targetError.values()*2) : 2) : DIM; }

	TargetMatchMeasurement(TargetMatchError &error,
							double const &pointVariance)
		: m_variance(pointVariance), m_targetError(error), m_sample(-1), m_numerical(false)
	{
	}

	MeasurementVector getMeasurement() const
	{
		MeasurementVector measurement(fullSize());
		m_targetError.getPointMeasurements(measurement);
		if (m_sample < 0) return measurement;
		return measurement.segment(m_sample*2, 2);
	}

	template <typename State>
	MeasurementSquareMatrix const &getCovariance(State const &)
	{
		m_covariance = MeasurementSquareMatrix::Identity(size(), size());
		m_covariance.diagonal().setConstant(m_variance);
		return m_covariance;
	}

	template <typename State>
	MeasurementVector predictMeasurement(State const &state) const
	{
		MeasurementVector prediction(fullSize());
		m_targetError.calculatePointProjections(state.getIsometry(), prediction);
		if (m_sample < 0) return prediction;
		return prediction.segment(m_sample*2, 2);
	}

	template <typename State>
	MeasurementVector getResidual(MeasurementVector const &prediction,
								  State const &state) const
	{
		/* MeasurementVector measurement(fullSize());
		m_targetError.getPointMeasurements(measurement);
		return measurement - prediction; */
		return getMeasurement() - prediction;
	}

	template <typename State>
	MeasurementVector getResidual(State const &state) const
	{
		/* MeasurementVector residual(fullSize());
		m_targetError.calculatePointErrors(state.getIsometry(), residual);
		return residual; */
		return getMeasurement() - predictMeasurement(state);
	}

	void setMeasurement(TargetMatchError &error)
	{
		m_targetError = error;
	}

	void setSample(int sample)
	{
		m_sample = sample;
	}

	void useNumerical(bool numerical)
	{
		m_numerical = numerical;
	}

	template <typename State>
	void calculateJacobian(State const &state, int p, Eigen::Ref<Eigen::MatrixXd> jacobian) const
	{
		auto &pt = m_targetError.m_observedPoints[p];
		int c = std::get<0>(pt);
		Vector3<double> tgtPt = std::get<1>(pt).cast<double>();

		if (m_numerical)
		{
			Eigen::Vector<double, 6> stateParams;
			stateParams.head<3>() = state.position();
			stateParams.tail<3>() = state.incrementalOrientation();
			NumericDiffJacobian<double>([&](const VectorX<double> &params, VectorX<double> &output)
			{
				State tempState = state;
				tempState.position() = params.head<3>();
				tempState.incrementalOrientation() = params.tail<3>();
				output = projectPoint2D(m_targetError.m_mvps[c], tempState.getIsometry() * tgtPt);
			}, stateParams, jacobian, 100);
			return;
		}

		// Following code is being derived:

		Vector3<double> incRot = state.incrementalOrientation()/2;
		double theta = incRot.norm();
		Eigen::Quaternion<double> incQuat;
		incQuat.vec() = flexkalman::util::ei_quat_exp_map::sinc(theta) * incRot;
		incQuat.w() = std::cos(theta);

		Eigen::Quaternion<double> baseQuat = state.getQuaternion();
		Eigen::Quaternion<double> poseQuat = incQuat * baseQuat;

		Vector3<double> scnPt = poseQuat * tgtPt + state.position();
		Vector4<double> tmpPt = m_targetError.m_mvps[c].cast<double>() * scnPt.homogeneous();
		Vector2<double> camPt = tmpPt.head<2>() / tmpPt.w();

		// Helpers:

		double aX = incRot.x(), aY = incRot.y(), aZ = incRot.z();
		double aSq = incRot.squaredNorm();
		double ct = cos(theta), st = sin(theta);

		double bQW = baseQuat.w(), bQX = baseQuat.x(), bQY = baseQuat.y(), bQZ = baseQuat.z();
		double pQW = poseQuat.w(), pQX = poseQuat.x(), pQY = poseQuat.y(), pQZ = poseQuat.z();

		double posX = state.position().x(), posY = state.position().y(), posZ = state.position().z();
		double tgtX = tgtPt.x(), tgtY = tgtPt.y(), tgtZ = tgtPt.z();
		double tX = tmpPt.x(), tY = tmpPt.y(), tZ = tmpPt.z(), tW = tmpPt.w();

		auto cm = m_targetError.m_mvps[c];

		// Derivation (generated for the most part):

		double d_theta_aX = aX/theta;
		double d_theta_aY = aY/theta;
		double d_theta_aZ = aZ/theta;

		double temp_3 = 1.0/(2*theta*theta);
		double d_iQW_aX = -st*d_theta_aX;
		double d_iQX_aX = (-aX*st*d_theta_aX + (aX*ct*d_theta_aX + st)*theta)*temp_3;
		double d_iQY_aX = aY*d_theta_aX*(theta*ct - st)*temp_3;
		double d_iQZ_aX = aZ*d_theta_aX*(theta*ct - st)*temp_3;
		double d_iQW_aY = -st*d_theta_aY;
		double d_iQX_aY = aX*d_theta_aY*(theta*ct - st)*temp_3;
		double d_iQY_aY = (-aY*st*d_theta_aY + (aY*ct*d_theta_aY + st)*theta)*temp_3;
		double d_iQZ_aY = aZ*d_theta_aY*(theta*ct - st)*temp_3;
		double d_iQW_aZ = -st*d_theta_aZ;
		double d_iQX_aZ = aX*d_theta_aZ*(theta*ct - st)*temp_3;
		double d_iQY_aZ = aY*d_theta_aZ*(theta*ct - st)*temp_3;
		double d_iQZ_aZ = (-aZ*st*d_theta_aZ + (aZ*ct*d_theta_aZ + st)*theta)*temp_3;

		double d_pQW_aX = bQW*d_iQW_aX - bQX*d_iQX_aX - bQY*d_iQY_aX - bQZ*d_iQZ_aX;
		double d_pQX_aX = bQW*d_iQX_aX + bQX*d_iQW_aX - bQY*d_iQZ_aX + bQZ*d_iQY_aX;
		double d_pQY_aX = bQW*d_iQY_aX + bQX*d_iQZ_aX + bQY*d_iQW_aX - bQZ*d_iQX_aX;
		double d_pQZ_aX = bQW*d_iQZ_aX - bQX*d_iQY_aX + bQY*d_iQX_aX + bQZ*d_iQW_aX;
		double d_pQW_aY = bQW*d_iQW_aY - bQX*d_iQX_aY - bQY*d_iQY_aY - bQZ*d_iQZ_aY;
		double d_pQX_aY = bQW*d_iQX_aY + bQX*d_iQW_aY - bQY*d_iQZ_aY + bQZ*d_iQY_aY;
		double d_pQY_aY = bQW*d_iQY_aY + bQX*d_iQZ_aY + bQY*d_iQW_aY - bQZ*d_iQX_aY;
		double d_pQZ_aY = bQW*d_iQZ_aY - bQX*d_iQY_aY + bQY*d_iQX_aY + bQZ*d_iQW_aY;
		double d_pQW_aZ = bQW*d_iQW_aZ - bQX*d_iQX_aZ - bQY*d_iQY_aZ - bQZ*d_iQZ_aZ;
		double d_pQX_aZ = bQW*d_iQX_aZ + bQX*d_iQW_aZ - bQY*d_iQZ_aZ + bQZ*d_iQY_aZ;
		double d_pQY_aZ = bQW*d_iQY_aZ + bQX*d_iQZ_aZ + bQY*d_iQW_aZ - bQZ*d_iQX_aZ;
		double d_pQZ_aZ = bQW*d_iQZ_aZ - bQX*d_iQY_aZ + bQY*d_iQX_aZ + bQZ*d_iQW_aZ;

		double temp_0 = 1.0/(poseQuat.squaredNorm()*poseQuat.squaredNorm());
		double temp_1_aX = -(pQW*d_pQW_aX + pQX*d_pQX_aX + pQY*d_pQY_aX + pQZ*d_pQZ_aX);
		double temp_1_aY = -(pQW*d_pQW_aY + pQX*d_pQX_aY + pQY*d_pQY_aY + pQZ*d_pQZ_aY);
		double temp_1_aZ = -(pQW*d_pQW_aZ + pQX*d_pQX_aZ + pQY*d_pQY_aZ + pQZ*d_pQZ_aZ);
		double temp_2_sX = posX*pQW*pQW + posX*pQX*pQX + posX*pQY*pQY + posX*pQZ*pQZ + tgtX*pQW*pQW + tgtX*pQX*pQX - tgtX*pQY*pQY - tgtX*pQZ*pQZ - 2*tgtY*pQW*pQZ + 2*tgtY*pQX*pQY + 2*tgtZ*pQW*pQY + 2*tgtZ*pQX*pQZ;
		double temp_2_sY = posY*pQW*pQW + posY*pQX*pQX + posY*pQY*pQY + posY*pQZ*pQZ + 2*tgtX*pQW*pQZ + 2*tgtX*pQX*pQY + tgtY*pQW*pQW - tgtY*pQX*pQX + tgtY*pQY*pQY - tgtY*pQZ*pQZ - 2*tgtZ*pQW*pQX + 2*tgtZ*pQY*pQZ;
		double temp_2_sZ = posZ*pQW*pQW + posZ*pQX*pQX + posZ*pQY*pQY + posZ*pQZ*pQZ - 2*tgtX*pQW*pQY + 2*tgtX*pQX*pQZ + 2*tgtY*pQW*pQX + 2*tgtY*pQY*pQZ + tgtZ*pQW*pQW - tgtZ*pQX*pQX - tgtZ*pQY*pQY + tgtZ*pQZ*pQZ;
		double d_sX_aX = 2*temp_0*(temp_1_aX*temp_2_sX + poseQuat.squaredNorm()*(posX*pQW*d_pQW_aX + posX*pQX*d_pQX_aX + posX*pQY*d_pQY_aX + posX*pQZ*d_pQZ_aX + tgtX*pQW*d_pQW_aX + tgtX*pQX*d_pQX_aX - tgtX*pQY*d_pQY_aX - tgtX*pQZ*d_pQZ_aX - tgtY*pQW*d_pQZ_aX + tgtY*pQX*d_pQY_aX + tgtY*pQY*d_pQX_aX - tgtY*pQZ*d_pQW_aX + tgtZ*pQW*d_pQY_aX + tgtZ*pQX*d_pQZ_aX + tgtZ*pQY*d_pQW_aX + tgtZ*pQZ*d_pQX_aX));
		double d_sY_aX = 2*temp_0*(temp_1_aX*temp_2_sY + poseQuat.squaredNorm()*(posY*pQW*d_pQW_aX + posY*pQX*d_pQX_aX + posY*pQY*d_pQY_aX + posY*pQZ*d_pQZ_aX + tgtX*pQW*d_pQZ_aX + tgtX*pQX*d_pQY_aX + tgtX*pQY*d_pQX_aX + tgtX*pQZ*d_pQW_aX + tgtY*pQW*d_pQW_aX - tgtY*pQX*d_pQX_aX + tgtY*pQY*d_pQY_aX - tgtY*pQZ*d_pQZ_aX - tgtZ*pQW*d_pQX_aX - tgtZ*pQX*d_pQW_aX + tgtZ*pQY*d_pQZ_aX + tgtZ*pQZ*d_pQY_aX));
		double d_sZ_aX = 2*temp_0*(temp_1_aX*temp_2_sZ + poseQuat.squaredNorm()*(posZ*pQW*d_pQW_aX + posZ*pQX*d_pQX_aX + posZ*pQY*d_pQY_aX + posZ*pQZ*d_pQZ_aX - tgtX*pQW*d_pQY_aX + tgtX*pQX*d_pQZ_aX - tgtX*pQY*d_pQW_aX + tgtX*pQZ*d_pQX_aX + tgtY*pQW*d_pQX_aX + tgtY*pQX*d_pQW_aX + tgtY*pQY*d_pQZ_aX + tgtY*pQZ*d_pQY_aX + tgtZ*pQW*d_pQW_aX - tgtZ*pQX*d_pQX_aX - tgtZ*pQY*d_pQY_aX + tgtZ*pQZ*d_pQZ_aX));
		double d_sX_aY = 2*temp_0*(temp_1_aY*temp_2_sX + poseQuat.squaredNorm()*(posX*pQW*d_pQW_aY + posX*pQX*d_pQX_aY + posX*pQY*d_pQY_aY + posX*pQZ*d_pQZ_aY + tgtX*pQW*d_pQW_aY + tgtX*pQX*d_pQX_aY - tgtX*pQY*d_pQY_aY - tgtX*pQZ*d_pQZ_aY - tgtY*pQW*d_pQZ_aY + tgtY*pQX*d_pQY_aY + tgtY*pQY*d_pQX_aY - tgtY*pQZ*d_pQW_aY + tgtZ*pQW*d_pQY_aY + tgtZ*pQX*d_pQZ_aY + tgtZ*pQY*d_pQW_aY + tgtZ*pQZ*d_pQX_aY));
		double d_sY_aY = 2*temp_0*(temp_1_aY*temp_2_sY + poseQuat.squaredNorm()*(posY*pQW*d_pQW_aY + posY*pQX*d_pQX_aY + posY*pQY*d_pQY_aY + posY*pQZ*d_pQZ_aY + tgtX*pQW*d_pQZ_aY + tgtX*pQX*d_pQY_aY + tgtX*pQY*d_pQX_aY + tgtX*pQZ*d_pQW_aY + tgtY*pQW*d_pQW_aY - tgtY*pQX*d_pQX_aY + tgtY*pQY*d_pQY_aY - tgtY*pQZ*d_pQZ_aY - tgtZ*pQW*d_pQX_aY - tgtZ*pQX*d_pQW_aY + tgtZ*pQY*d_pQZ_aY + tgtZ*pQZ*d_pQY_aY));
		double d_sZ_aY = 2*temp_0*(temp_1_aY*temp_2_sZ + poseQuat.squaredNorm()*(posZ*pQW*d_pQW_aY + posZ*pQX*d_pQX_aY + posZ*pQY*d_pQY_aY + posZ*pQZ*d_pQZ_aY - tgtX*pQW*d_pQY_aY + tgtX*pQX*d_pQZ_aY - tgtX*pQY*d_pQW_aY + tgtX*pQZ*d_pQX_aY + tgtY*pQW*d_pQX_aY + tgtY*pQX*d_pQW_aY + tgtY*pQY*d_pQZ_aY + tgtY*pQZ*d_pQY_aY + tgtZ*pQW*d_pQW_aY - tgtZ*pQX*d_pQX_aY - tgtZ*pQY*d_pQY_aY + tgtZ*pQZ*d_pQZ_aY));
		double d_sX_aZ = 2*temp_0*(temp_1_aZ*temp_2_sX + poseQuat.squaredNorm()*(posX*pQW*d_pQW_aZ + posX*pQX*d_pQX_aZ + posX*pQY*d_pQY_aZ + posX*pQZ*d_pQZ_aZ + tgtX*pQW*d_pQW_aZ + tgtX*pQX*d_pQX_aZ - tgtX*pQY*d_pQY_aZ - tgtX*pQZ*d_pQZ_aZ - tgtY*pQW*d_pQZ_aZ + tgtY*pQX*d_pQY_aZ + tgtY*pQY*d_pQX_aZ - tgtY*pQZ*d_pQW_aZ + tgtZ*pQW*d_pQY_aZ + tgtZ*pQX*d_pQZ_aZ + tgtZ*pQY*d_pQW_aZ + tgtZ*pQZ*d_pQX_aZ));
		double d_sY_aZ = 2*temp_0*(temp_1_aZ*temp_2_sY + poseQuat.squaredNorm()*(posY*pQW*d_pQW_aZ + posY*pQX*d_pQX_aZ + posY*pQY*d_pQY_aZ + posY*pQZ*d_pQZ_aZ + tgtX*pQW*d_pQZ_aZ + tgtX*pQX*d_pQY_aZ + tgtX*pQY*d_pQX_aZ + tgtX*pQZ*d_pQW_aZ + tgtY*pQW*d_pQW_aZ - tgtY*pQX*d_pQX_aZ + tgtY*pQY*d_pQY_aZ - tgtY*pQZ*d_pQZ_aZ - tgtZ*pQW*d_pQX_aZ - tgtZ*pQX*d_pQW_aZ + tgtZ*pQY*d_pQZ_aZ + tgtZ*pQZ*d_pQY_aZ));
		double d_sZ_aZ = 2*temp_0*(temp_1_aZ*temp_2_sZ + poseQuat.squaredNorm()*(posZ*pQW*d_pQW_aZ + posZ*pQX*d_pQX_aZ + posZ*pQY*d_pQY_aZ + posZ*pQZ*d_pQZ_aZ - tgtX*pQW*d_pQY_aZ + tgtX*pQX*d_pQZ_aZ - tgtX*pQY*d_pQW_aZ + tgtX*pQZ*d_pQX_aZ + tgtY*pQW*d_pQX_aZ + tgtY*pQX*d_pQW_aZ + tgtY*pQY*d_pQZ_aZ + tgtY*pQZ*d_pQY_aZ + tgtZ*pQW*d_pQW_aZ - tgtZ*pQX*d_pQX_aZ - tgtZ*pQY*d_pQY_aZ + tgtZ*pQZ*d_pQZ_aZ));

		double d_tX_aX = d_sX_aX*cm(0,0) + d_sY_aX*cm(0,1) + d_sZ_aX*cm(0,2);
		double d_tW_aX = d_sX_aX*cm(3,0) + d_sY_aX*cm(3,1) + d_sZ_aX*cm(3,2);
		double d_tY_aX = d_sX_aX*cm(1,0) + d_sY_aX*cm(1,1) + d_sZ_aX*cm(1,2);
		double d_tX_aY = d_sX_aY*cm(0,0) + d_sY_aY*cm(0,1) + d_sZ_aY*cm(0,2);
		double d_tW_aY = d_sX_aY*cm(3,0) + d_sY_aY*cm(3,1) + d_sZ_aY*cm(3,2);
		double d_tY_aY = d_sX_aY*cm(1,0) + d_sY_aY*cm(1,1) + d_sZ_aY*cm(1,2);
		double d_tX_aZ = d_sX_aZ*cm(0,0) + d_sY_aZ*cm(0,1) + d_sZ_aZ*cm(0,2);
		double d_tW_aZ = d_sX_aZ*cm(3,0) + d_sY_aZ*cm(3,1) + d_sZ_aZ*cm(3,2);
		double d_tY_aZ = d_sX_aZ*cm(1,0) + d_sY_aZ*cm(1,1) + d_sZ_aZ*cm(1,2);

		double d_cX_posX = (tW*cm(0,0) - tX*cm(3,0))/(tW*tW);
		double d_cY_posX = (tW*cm(1,0) - tY*cm(3,0))/(tW*tW);
		double d_cX_posY = (tW*cm(0,1) - tX*cm(3,1))/(tW*tW);
		double d_cY_posY = (tW*cm(1,1) - tY*cm(3,1))/(tW*tW);
		double d_cX_posZ = (tW*cm(0,2) - tX*cm(3,2))/(tW*tW);
		double d_cY_posZ = (tW*cm(1,2) - tY*cm(3,2))/(tW*tW);
		double d_cX_aX = (tW*d_tX_aX - tX*d_tW_aX)/(tW*tW);
		double d_cY_aX = (tW*d_tY_aX - tY*d_tW_aX)/(tW*tW);
		double d_cX_aY = (tW*d_tX_aY - tX*d_tW_aY)/(tW*tW);
		double d_cY_aY = (tW*d_tY_aY - tY*d_tW_aY)/(tW*tW);
		double d_cX_aZ = (tW*d_tX_aZ - tX*d_tW_aZ)/(tW*tW);
		double d_cY_aZ = (tW*d_tY_aZ - tY*d_tW_aZ)/(tW*tW);
	
		jacobian.template block<2,3>(0,0) << // w.r.t. x,y
			d_cX_posX, d_cX_posY, d_cX_posZ,
			d_cY_posX, d_cY_posY, d_cY_posZ;

		jacobian.template block<2,3>(0,3) << // w.r.t. rotation
			d_cX_aX, d_cX_aY, d_cX_aZ,
			d_cY_aX, d_cY_aY, d_cY_aZ;
	}

	template <typename State>
	flexkalman::types::Matrix<Dimension, State::Dimension> getJacobian(State const &state) const
	{
		flexkalman::types::Matrix<Dimension, State::Dimension> jacobian(size(), state.size());
		jacobian.setZero();
		if (m_sample < 0)
		{
			for (int p = 0; p < m_targetError.m_observedPoints.size(); p++)
				calculateJacobian(state, p, jacobian.block(p*2, 0, 2, 6));
		}
		else
		{
			calculateJacobian(state, m_sample, jacobian.block(0, 0, 2, 6));
		}
		return jacobian;
	}


  private:
	MeasurementSquareMatrix m_covariance;
	double m_variance;
	TargetMatchError &m_targetError;
	int m_sample;
	bool m_numerical;
};

#endif // TARGET_KALMAN_H