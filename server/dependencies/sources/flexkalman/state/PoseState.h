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
// Copyright 2019-2020 Collabora, Ltd.
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

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace flexkalman {

namespace pose_externalized_rotation {
    constexpr size_t Dimension = 12;
    using StateVector = types::Vector<Dimension>;
    using StateVectorBlock3 = StateVector::FixedSegmentReturnType<3>::Type;
    using ConstStateVectorBlock3 =
        StateVector::ConstFixedSegmentReturnType<3>::Type;

    using StateVectorBlock6 = StateVector::FixedSegmentReturnType<6>::Type;
    using ConstStateVectorBlock6 =
        StateVector::ConstFixedSegmentReturnType<6>::Type;
    using StateSquareMatrix = types::SquareMatrix<Dimension>;

    /*!
     * This returns A(deltaT), though if you're just predicting xhat-, use
     * applyVelocity() instead for performance.
     */
    inline StateSquareMatrix stateTransitionMatrix(double dt) {
        // eq. 4.5 in Welch 1996 - except we have all the velocities at the
        // end
        StateSquareMatrix A = StateSquareMatrix::Identity();
        A.topRightCorner<6, 6>() = types::SquareMatrix<6>::Identity() * dt;

        return A;
    }
    /*!
     * Function used to compute the coefficient m in v_new = m * v_old.
     * The damping value is for exponential decay.
     */
    inline double computeAttenuation(double damping, double dt) {
        return std::pow(damping, dt);
    }

    class State : public StateBase<State> {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static constexpr size_t Dimension = 12;
        using StateVector = types::Vector<Dimension>;
        using StateSquareMatrix = types::SquareMatrix<Dimension>;

        //! Default constructor
        State()
            : m_state(StateVector::Zero()),
              m_errorCovariance(StateSquareMatrix::Identity() *
                                10 /** @todo almost certainly wrong */),
              m_orientation(Eigen::Quaterniond::Identity()) {}
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
        StateSquareMatrix &errorCovariance() { return m_errorCovariance; }

        //! Intended for startup use.
        void setQuaternion(Eigen::Quaterniond const &quaternion) {
            m_orientation = quaternion.normalized();
        }

        void postCorrect() { externalizeRotation(); }

        void externalizeRotation() {
            setQuaternion(getCombinedQuaternion());
            incrementalOrientation() = Eigen::Vector3d::Zero();
        }

        StateVectorBlock3 position() { return m_state.head<3>(); }

        ConstStateVectorBlock3 position() const { return m_state.head<3>(); }

        StateVectorBlock3 incrementalOrientation() {
            return m_state.segment<3>(3);
        }

        ConstStateVectorBlock3 incrementalOrientation() const {
            return m_state.segment<3>(3);
        }

        StateVectorBlock3 velocity() { return m_state.segment<3>(6); }

        ConstStateVectorBlock3 velocity() const {
            return m_state.segment<3>(6);
        }

        StateVectorBlock3 angularVelocity() { return m_state.segment<3>(9); }

        ConstStateVectorBlock3 angularVelocity() const {
            return m_state.segment<3>(9);
        }

        //! Linear and angular velocities
        StateVectorBlock6 velocities() { return m_state.tail<6>(); }

        //! Linear and angular velocities
        ConstStateVectorBlock6 velocities() const { return m_state.tail<6>(); }

        Eigen::Quaterniond const &getQuaternion() const {
            return m_orientation;
        }

        Eigen::Quaterniond getCombinedQuaternion() const {
            // divide by 2 since we're integrating it essentially.
            return util::quat_exp(incrementalOrientation() / 2.) *
                   m_orientation;
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
         * In order: x, y, z, incremental rotations phi (about x), theta
         * (about y), psy (about z), then their derivatives in the same
         * order.
         */
        StateVector m_state;
        //! P
        StateSquareMatrix m_errorCovariance;
        //! Externally-maintained orientation per Welch 1996
        Eigen::Quaterniond m_orientation;
    };

    /*!
     * Stream insertion operator, for displaying the state of the state
     * class.
     */
    template <typename OutputStream>
    inline OutputStream &operator<<(OutputStream &os, State const &state) {
        os << "State:" << state.stateVector().transpose() << "\n";
        os << "quat:" << state.getCombinedQuaternion().coeffs().transpose()
           << "\n";
        os << "error:\n" << state.errorCovariance() << "\n";
        return os;
    }

    //! Computes A(deltaT)xhat(t-deltaT)
    inline void applyVelocity(State &state, double dt) {
        // eq. 4.5 in Welch 1996

        /*!
         * @todo benchmark - assuming for now that the manual small
         * calcuations are faster than the matrix ones.
         */

        state.position() += state.velocity() * dt;
        state.incrementalOrientation() += state.angularVelocity() * dt;
    }

    //! Dampen all 6 components of velocity by a single factor.
    inline void dampenVelocities(State &state, double damping, double dt) {
        auto attenuation = computeAttenuation(damping, dt);
        state.velocities() *= attenuation;
    }

    //! Separately dampen the linear and angular velocities
    inline void separatelyDampenVelocities(State &state, double posDamping,
                                           double oriDamping, double dt) {
        state.velocity() *= computeAttenuation(posDamping, dt);
        state.angularVelocity() *= computeAttenuation(oriDamping, dt);
    }

    inline StateSquareMatrix stateTransitionMatrix(State const & /* state */,
                                                   double dt) {
        return stateTransitionMatrix(dt);
    }
    /*!
     * Returns the state transition matrix for a constant velocity with a
     * single damping parameter (not for direct use in computing state
     * transition, because it is very sparse, but in computing other
     * values)
     */
    inline StateSquareMatrix
    stateTransitionMatrixWithVelocityDamping(State const &state, double dt,
                                             double damping) {
        // eq. 4.5 in Welch 1996
        auto A = stateTransitionMatrix(state, dt);
        A.bottomRightCorner<6, 6>() *= computeAttenuation(damping, dt);
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
} // namespace pose_externalized_rotation

} // namespace flexkalman
