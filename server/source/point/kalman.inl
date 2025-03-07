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

#ifndef POINT_KALMAN_H
#define POINT_KALMAN_H

#include "flexkalman/BaseTypes.h"
#include "flexkalman/FlexibleKalmanBase.h"

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

#endif // POINT_KALMAN_H