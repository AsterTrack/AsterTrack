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

#ifndef FLEX_UKF_H
#define FLEX_UKF_H

#include "flexkalman/EigenQuatExponentialMap.h"
#include "flexkalman/FlexibleKalmanBase.h"
#include "flexkalman/BaseTypes.h"

/**
 * AbsolutePoseMeasurement for use with flexkalman UKF
 */

using namespace flexkalman;

class AbsolutePoseMeasurement :
	public flexkalman::MeasurementBase<AbsolutePoseMeasurement> {
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static constexpr size_t Dimension = 6;
	using MeasurementVector = types::Vector<Dimension>;
	using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;

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

	template <typename State>
	MeasurementSquareMatrix const &getCovariance(State const &) {
		return m_covariance;
	}

	template <typename State>
	MeasurementVector predictMeasurement(State const &state) const {
		MeasurementVector prediction;
		prediction.head<3>() = state.position();
		prediction.tail<3>() = state.incrementalOrientation();
		return prediction;
	}

	template <typename State>
	MeasurementVector getResidual(MeasurementVector const &prediction,
								  State const &s) const {
		// The prediction we're given is effectively "the state's incremental
		// rotation", which is why we're using our measurement here as well as
		// the prediction.
		const Eigen::Quaterniond predictedQuat = util::quat_exp(
			prediction.tail<3>() / 2.) * s.getQuaternion();
		MeasurementVector residual;
		residual.head<3>() = m_pos - prediction.head<3>();
		residual.tail<3>() = 2 * util::smallest_quat_ln(m_quat * predictedQuat.conjugate());
		return residual;
	}
	/*!
	 * Gets the measurement residual, also known as innovation: predicts
	 * the measurement from the predicted state, and returns the
	 * difference.
	 *
	 * State type doesn't matter as long as we can
	 * `.getCombinedQuaternion()`
	 */
	template <typename State>
	MeasurementVector getResidual(State const &s) const {
		const Eigen::Quaterniond predictedQuat = s.getCombinedQuaternion();
		// Two equivalent quaternions: but their logs are typically
		// different: one is the "short way" and the other is the "long
		// way". We'll compute both and pick the "short way".
		MeasurementVector residual;
		residual.head<3>() = m_pos - s.position();
		residual.tail<3>() = 2 * util::smallest_quat_ln(m_quat * predictedQuat.conjugate());
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

#endif // FLEX_UKF_H