/** @file
    @brief Header for measurements of absolute orientation.

    @date 2015, 2020

    @author
    Rylie Pavlik
    <rylie.pavlik@collabora.com>
    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
// Copyright 2020 Collabora, Ltd.
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
#include "flexkalman/EigenQuatExponentialMap.h"
#include "flexkalman/ExternalQuaternion.h"
#include "flexkalman/FlexibleKalmanBase.h"
#include "flexkalman/state/PoseState.h"

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace flexkalman {

//! Default implementation: overload if this won't work for your state type.
//! @see AbsoluteOrientationMeasurementBase
template <typename State>
types::Vector<3> predictAbsoluteOrientationMeasurement(State const &s) {
    return s.incrementalOrientation();
}

class AbsoluteOrientationMeasurementBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    AbsoluteOrientationMeasurementBase(Eigen::Quaterniond const &quat,
                                       types::Vector<3> const &emVariance)
        : m_quat(quat), m_covariance(emVariance.asDiagonal()) {}

    template <typename State>
    MeasurementSquareMatrix const &getCovariance(State const &) {
        return m_covariance;
    }

    template <typename State>
    MeasurementVector predictMeasurement(State const &state) const {
        return predictAbsoluteOrientationMeasurement(state);
    }
    template <typename State>
    MeasurementVector getResidual(MeasurementVector const &predictedMeasurement,
                                  State const &s) const {
        // The prediction we're given is effectively "the state's incremental
        // rotation", which is why we're using our measurement here as well as
        // the prediction.
        const Eigen::Quaterniond fullPredictedOrientation =
            util::quat_exp(predictedMeasurement / 2.) * s.getQuaternion();
        return 2 * util::smallest_quat_ln(m_quat *
                                          fullPredictedOrientation.conjugate());
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
        const Eigen::Quaterniond prediction = s.getCombinedQuaternion();
        // Two equivalent quaternions: but their logs are typically
        // different: one is the "short way" and the other is the "long
        // way". We'll compute both and pick the "short way".
        return 2 * util::smallest_quat_ln(m_quat * prediction.conjugate());
    }
    //! Convenience method to be able to store and re-use measurements.
    void setMeasurement(Eigen::Quaterniond const &quat) { m_quat = quat; }

  private:
    Eigen::Quaterniond m_quat;
    MeasurementSquareMatrix m_covariance;
};
/*!
 * A measurement of absolute orientation in 3D space.
 *
 * It can be used with any state class that exposes a `getCombinedQuaternion()`
 * method (that is, an externalized quaternion state). On its own, it is only
 * suitable for unscented filter correction, since the jacobian depends on the
 * arrangement of the state vector. See AbsoluteOrientationEKFMeasurement's
 * explicit specializations for use in EKF correction mode.
 */
class AbsoluteOrientationMeasurement
    : public AbsoluteOrientationMeasurementBase,
      public MeasurementBase<AbsoluteOrientationMeasurement> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using AbsoluteOrientationMeasurementBase::
        AbsoluteOrientationMeasurementBase;
};

/*!
 * This is the EKF-specific relative of AbsoluteOrientationMeasurement: only
 * explicit specializations, and on state types.
 *
 * Only required for EKF-style correction (since jacobian depends closely on the
 * state).
 */
template <typename StateType> class AbsoluteOrientationEKFMeasurement;

//! AbsoluteOrientationEKFMeasurement with a pose_externalized_rotation::State
template <>
class AbsoluteOrientationEKFMeasurement<pose_externalized_rotation::State>
    : public AbsoluteOrientationMeasurementBase,
      public MeasurementBase<AbsoluteOrientationEKFMeasurement<
          pose_externalized_rotation::State>> {
  public:
    using State = pose_externalized_rotation::State;
    static constexpr size_t StateDimension = getDimension<State>();
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AbsoluteOrientationEKFMeasurement(Eigen::Quaterniond const &quat,
                                      types::Vector<3> const &eulerVariance)
        : AbsoluteOrientationMeasurementBase(quat, eulerVariance) {}

    types::Matrix<Dimension, StateDimension> getJacobian(State const &s) const {
        using namespace pose_externalized_rotation;
        using Jacobian = types::Matrix<Dimension, StateDimension>;
        Jacobian ret = Jacobian::Zero();
        ret.block<Dimension, 3>(0, 3) = types::SquareMatrix<3>::Identity();
        return ret;
    }
};

} // namespace flexkalman
