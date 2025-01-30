/** @file
    @brief Header

    @date 2015-2019

    @author
    Rylie Pavlik
    <rylie.pavlik@collabora.com>
    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// Copyright 2019 Collabora, Ltd.
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
#include "flexkalman/ExternalQuaternion.h"
#include "flexkalman/FlexibleKalmanBase.h"
#include "flexkalman/state/OrientationState.h"
#include "flexkalman/state/PoseState.h"

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace flexkalman {

class AngularVelocityMeasurementBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    AngularVelocityMeasurementBase(MeasurementVector const &vel,
                                   MeasurementVector const &variance)
        : m_measurement(vel), m_covariance(variance.asDiagonal()) {}

    template <typename State>
    MeasurementSquareMatrix const &getCovariance(State const &) {
        return m_covariance;
    }

    template <typename State>
    types::Vector<3> predictMeasurement(State const &s) const {
        return s.angularVelocity();
    }

    template <typename State>
    MeasurementVector getResidual(MeasurementVector const &prediction,
                                  State const & /* s */) const {
        const MeasurementVector residual = m_measurement - prediction;
        return residual;
    }

    /*!
     * Gets the measurement residual, also known as innovation: predicts
     * the measurement from the predicted state, and returns the
     * difference.
     *
     * State type doesn't matter as long as we can `.angularVelocity()`
     */
    template <typename State>
    MeasurementVector getResidual(State const &s) const {
        return getResidual(predictMeasurement(s), s);
    }

    //! Convenience method to be able to store and re-use measurements.
    void setMeasurement(MeasurementVector const &vel) { m_measurement = vel; }

  private:
    MeasurementVector m_measurement;
    MeasurementSquareMatrix m_covariance;
};
/*!
 * This class is a 3D angular velocity measurement.
 *
 * It can be used with any state class that exposes a `angularVelocity()`
 * method. On its own, it is only suitable for unscented filter correction,
 * since the jacobian depends on the arrangement of the state vector. See
 * AngularVelocityEKFMeasurement's explicit specializations for use in EKF
 * correction mode.
 */
class AngularVelocityMeasurement
    : public AngularVelocityMeasurementBase,
      public MeasurementBase<AngularVelocityMeasurement> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using AngularVelocityMeasurementBase::AngularVelocityMeasurementBase;
};

/*!
 * This is the EKF-specific relative of AngularVelocityMeasurement: only
 * explicit specializations, and on state types.
 *
 * Only required for EKF-style correction (since jacobian depends closely on the
 * state).
 */
template <typename StateType> class AngularVelocityEKFMeasurement;

//! AngularVelocityEKFMeasurement with a pose_externalized_rotation::State
template <>
class AngularVelocityEKFMeasurement<pose_externalized_rotation::State>
    : public AngularVelocityMeasurementBase,
      public MeasurementBase<
          AngularVelocityEKFMeasurement<pose_externalized_rotation::State>> {
  public:
    using State = pose_externalized_rotation::State;
    static constexpr size_t StateDimension = getDimension<State>();
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AngularVelocityEKFMeasurement(MeasurementVector const &vel,
                                  MeasurementVector const &variance)
        : AngularVelocityMeasurementBase(vel, variance) {}

    types::Matrix<Dimension, StateDimension> getJacobian(State const &) const {
        using Jacobian = types::Matrix<Dimension, StateDimension>;
        Jacobian ret = Jacobian::Zero();
        ret.topRightCorner<3, 3>() = types::SquareMatrix<3>::Identity();
        return ret;
    }
};

/*!
 * AngularVelocityEKFMeasurement with a orient_externalized_rotation::State
 * The code is in fact identical except for the state types, due to a
 * coincidence of how the state vectors are arranged.
 */
template <>
class AngularVelocityEKFMeasurement<orient_externalized_rotation::State>
    : public AngularVelocityMeasurementBase,
      public MeasurementBase<
          AngularVelocityEKFMeasurement<orient_externalized_rotation::State>> {
  public:
    using State = orient_externalized_rotation::State;
    static constexpr size_t StateDimension = getDimension<State>();
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AngularVelocityEKFMeasurement(MeasurementVector const &vel,
                                  MeasurementVector const &variance)
        : AngularVelocityMeasurementBase(vel, variance) {}

    types::Matrix<Dimension, StateDimension> getJacobian(State const &) const {
        using Jacobian = types::Matrix<Dimension, StateDimension>;
        Jacobian ret = Jacobian::Zero();
        ret.topRightCorner<3, 3>() = types::SquareMatrix<3>::Identity();
        return ret;
    }
};

} // namespace flexkalman
