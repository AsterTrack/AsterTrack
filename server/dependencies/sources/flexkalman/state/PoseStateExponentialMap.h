/** @file
    @brief Header defining a state that uses the "Exponential Map" rotation
   formalism" instead of the "internal incremental rotation, externalized
   quaternion" representation.

   @todo incomplete

    @date 2015

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
#include "flexkalman/FlexibleKalmanBase.h"
#include "flexkalman/MatrixExponentialMap.h"
#include "flexkalman/PoseConstantVelocityGeneric.h"
#include "flexkalman/SO3.h"

// Library/third-party includes
// - none

// Standard includes
// - none

namespace flexkalman {

namespace pose_exp_map {

    constexpr size_t Dimension = 12;
    using StateVector = types::Vector<Dimension>;
    using StateVectorBlock3 = StateVector::FixedSegmentReturnType<3>::Type;
    using ConstStateVectorBlock3 =
        StateVector::ConstFixedSegmentReturnType<3>::Type;

    using StateVectorBlock6 = StateVector::FixedSegmentReturnType<6>::Type;
    using ConstStateVectorBlock6 =
        StateVector::ConstFixedSegmentReturnType<6>::Type;
    using StateSquareMatrix = types::SquareMatrix<Dimension>;
    inline double computeAttenuation(double damping, double dt) {
        return std::pow(damping, dt);
    }
    class State : public StateBase<State> {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        static constexpr size_t Dimension = 12;
        using StateVector = ::flexkalman::pose_exp_map::StateVector;
        using StateSquareMatrix = ::flexkalman::pose_exp_map::StateSquareMatrix;

        //! Default constructor
        State()
            : m_state(StateVector::Zero()),
              m_errorCovariance(
                  StateSquareMatrix::
                      Identity() /** @todo almost certainly wrong */) {}
        //! set xhat
        void setStateVector(StateVector const &state) { m_state = state; }
        //! xhat
        StateVector const &stateVector() const { return m_state; }
        // set P
        void setErrorCovariance(StateSquareMatrix const &errorCovariance) {
            m_errorCovariance = errorCovariance;
        }
        //! P
        StateSquareMatrix const &errorCovariance() const {
            return m_errorCovariance;
        }

        void postCorrect() {
            types::Vector<3> ori = rotationVector();
            matrix_exponential_map::avoidSingularities(ori);
            rotationVector() = ori;
        }

        StateVectorBlock3 position() { return m_state.head<3>(); }

        ConstStateVectorBlock3 position() const { return m_state.head<3>(); }

        Eigen::Quaterniond getQuaternion() const {
            return matrix_exponential_map::toQuat(rotationVector());
        }
        Eigen::Matrix3d getRotationMatrix() const {
            return matrix_exponential_map::rodrigues(rotationVector());
        }

        StateVectorBlock3 velocity() { return m_state.segment<3>(6); }

        ConstStateVectorBlock3 velocity() const {
            return m_state.segment<3>(6);
        }

        StateVectorBlock3 angularVelocity() { return m_state.segment<3>(9); }

        ConstStateVectorBlock3 angularVelocity() const {
            return m_state.segment<3>(9);
        }

        /// Linear and angular velocities
        StateVectorBlock6 velocities() { return m_state.tail<6>(); }

        /// Linear and angular velocities
        ConstStateVectorBlock6 velocities() const { return m_state.tail<6>(); }

        StateVectorBlock3 rotationVector() { return m_state.segment<3>(3); }

        ConstStateVectorBlock3 rotationVector() const {
            return m_state.segment<3>(3);
        }

        /*!
         * Get the position and quaternion combined into a single isometry
         * (transformation)
         */
        Eigen::Isometry3d getIsometry() const {
            Eigen::Isometry3d ret;
            ret.fromPositionOrientationScale(position(), getQuaternion(),
                                             Eigen::Vector3d::Constant(1));
            return ret;
        }

      private:
        /*!
         * In order: x, y, z, exponential rotation coordinates w1, w2, w3,
         * then their derivatives in the same order.
         */
        StateVector m_state;
        //! P
        StateSquareMatrix m_errorCovariance;
    };

    /*!
     * Stream insertion operator, for displaying the state of the state
     * class.
     */
    template <typename OutputStream>
    inline OutputStream &operator<<(OutputStream &os, State const &state) {
        os << "State:" << state.stateVector().transpose() << "\n";
        os << "error:\n" << state.errorCovariance() << "\n";
        return os;
    }

    /*!
     * This returns A(deltaT), though if you're just predicting xhat-, use
     * applyVelocity() instead for performance.
     *
     * State is a parameter for ADL.
     */
    inline StateSquareMatrix stateTransitionMatrix(State const &, double dt) {
        // eq. 4.5 in Welch 1996 - except we have all the velocities at the
        // end
        StateSquareMatrix A = StateSquareMatrix::Identity();
        A.topRightCorner<6, 6>() = types::SquareMatrix<6>::Identity() * dt;

        return A;
    }

    inline StateSquareMatrix
    stateTransitionMatrixWithVelocityDamping(State const &s, double dt,
                                             double damping) {

        // eq. 4.5 in Welch 1996

        auto A = stateTransitionMatrix(s, dt);
        auto attenuation = computeAttenuation(damping, dt);
        A.bottomRightCorner<6, 6>() *= attenuation;
        return A;
    }

    /*!
     * Returns the state transition matrix for a constant velocity with
     * separate damping paramters for linear and angular velocity (not for
     * direct use in computing state transition, because it is very sparse,
     * but in computing other values)
     */
    inline StateSquareMatrix stateTransitionMatrixWithSeparateVelocityDamping(
        State const &state, double dt, double posDamping, double oriDamping) {
        // eq. 4.5 in Welch 1996
        auto A = stateTransitionMatrix(state, dt);
        A.block<3, 3>(6, 6) *= computeAttenuation(posDamping, dt);
        A.bottomRightCorner<3, 3>() *= computeAttenuation(oriDamping, dt);
        return A;
    }

    //! Separately dampen the linear and angular velocities
    inline void separatelyDampenVelocities(State &state, double posDamping,
                                           double oriDamping, double dt) {
        state.velocity() *= computeAttenuation(posDamping, dt);
        state.angularVelocity() *= computeAttenuation(oriDamping, dt);
    }

    /// Computes A(deltaT)xhat(t-deltaT) (or, the more precise, non-linear thing
    /// that is intended to simulate.)
    inline void applyVelocity(State &state, double dt) {
        state.position() += state.velocity() * dt;

        // Do the full thing, not just the small-angle approximation as we have
        // in the state transition matrix.
        SO3 newOrientation = SO3{(state.angularVelocity() * dt).eval()} *
                             SO3{(state.rotationVector()).eval()};
        state.rotationVector() = newOrientation.getVector();
    }

    inline types::Vector<3> predictAbsoluteOrientationMeasurement(State const &s) {
        return s.rotationVector();
    }
    using ConstantVelocityProcessModel =
        PoseConstantVelocityGenericProcessModel<State>;
} // namespace pose_exp_map

} // namespace flexkalman
