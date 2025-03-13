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

	/*!
	 * Gets the measurement residual, also known as innovation: predicts
	 * the measurement from the predicted state, and returns the
	 * difference.
	 */
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

	//! Convenience method to be able to store and re-use measurements.
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

	/*!
	 * Gets the measurement residual, also known as innovation: predicts
	 * the measurement from the predicted state, and returns the
	 * difference.
	 */
	template <typename State>
	MeasurementVector getResidual(State const &state) const
	{
		const Eigen::Quaterniond predictedQuat = state.getCombinedQuaternion();
		// Two equivalent quaternions: but their logs are typically
		// different: one is the "short way" and the other is the "long
		// way". We'll compute both and pick the "short way".
		return 2 * flexkalman::util::smallest_quat_ln(m_quat * predictedQuat.conjugate());
	}

	//! Convenience method to be able to store and re-use measurements.
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

	/*!
	 * Gets the measurement residual, also known as innovation: predicts
	 * the measurement from the predicted state, and returns the
	 * difference.
	 */
	template <typename State>
	MeasurementVector getResidual(State const &state) const
	{
		return m_pos - state.position();
	}

	//! Convenience method to be able to store and re-use measurements.
	void setMeasurement(Eigen::Vector3d const &pos)
	{
		m_pos = pos;
	}

private:
	Eigen::Vector3d m_pos;
	MeasurementSquareMatrix m_covariance;
};

#endif // TARGET_KALMAN_H