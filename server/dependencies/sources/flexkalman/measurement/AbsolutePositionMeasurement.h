/** @file
    @brief Header

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Internal Includes
#include "flexkalman/BaseTypes.h"
#include "flexkalman/FlexibleKalmanBase.h"
#include "flexkalman/state/PoseState.h"

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace flexkalman {
class AbsolutePositionMeasurementBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const size_t Dimension = 3; // 3 position
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementDiagonalMatrix = types::DiagonalMatrix<Dimension>;
    using MeasurementMatrix = types::SquareMatrix<Dimension>;
    AbsolutePositionMeasurementBase(MeasurementVector const &pos,
                                    MeasurementVector const &variance)
        : m_pos(pos), m_covariance(variance.asDiagonal()) {}

    template <typename State>
    MeasurementMatrix getCovariance(State const &) const {
        return m_covariance;
    }
    template <typename State>
    MeasurementVector predictMeasurement(State const &s) const {
        return s.position();
    }
    template <typename State>
    MeasurementVector getResidual(MeasurementVector const &prediction,
                                  State const &s) const {
        MeasurementVector residual = m_pos - prediction;
        return residual;
    }
    /*!
     * Gets the measurement residual, also known as innovation: predicts
     * the measurement from the predicted state, and returns the
     * difference.
     *
     * State type doesn't matter as long as we can `.position()`
     */
    template <typename State>
    MeasurementVector getResidual(State const &s) const {
        return getResidual(predictMeasurement(s), s);
    }

    //! Convenience method to be able to store and re-use measurements.
    void setMeasurement(MeasurementVector const &pos) { m_pos = pos; }

  private:
    MeasurementVector m_pos;
    MeasurementDiagonalMatrix m_covariance;
};
/*!
 * This class is a 3D position measurement.
 *
 * It can be used with any state class that exposes a `position()`
 * method. On its own, it is only suitable for unscented filter correction,
 * since the jacobian depends on the arrangement of the state vector. See
 * AbsolutePositionEKFMeasurement's explicit specializations for use in EKF
 * correction mode.
 */
class AbsolutePositionMeasurement
    : public AbsolutePositionMeasurementBase,
      public MeasurementBase<AbsolutePositionMeasurement> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using AbsolutePositionMeasurementBase::AbsolutePositionMeasurementBase;
};

/*!
 * This is the EKF-specific relative of AbsolutePositionMeasurement: only
 * explicit specializations, and on state types.
 */
template <typename StateType> class AbsolutePositionEKFMeasurement;

//! AbsolutePositionEKFMeasurement with a pose_externalized_rotation::State
template <>
class AbsolutePositionEKFMeasurement<pose_externalized_rotation::State>
    : public AbsolutePositionMeasurementBase,
      public MeasurementBase<
          AbsolutePositionEKFMeasurement<pose_externalized_rotation::State>> {
  public:
    using State = pose_externalized_rotation::State;
    using AbsolutePositionMeasurementBase::Dimension;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t StateDimension = getDimension<State>();
    using Jacobian = types::Matrix<Dimension, StateDimension>;
    AbsolutePositionEKFMeasurement(MeasurementVector const &pos,
                                   MeasurementVector const &variance)
        : AbsolutePositionMeasurementBase(pos, variance),
          m_jacobian(Jacobian::Zero()) {
        m_jacobian.block<3, 3>(0, 0) = types::SquareMatrix<3>::Identity();
    }

    types::Matrix<Dimension, StateDimension> const &
    getJacobian(State const &) const {
        return m_jacobian;
    }

  private:
    types::Matrix<Dimension, StateDimension> m_jacobian;
};

} // namespace flexkalman
