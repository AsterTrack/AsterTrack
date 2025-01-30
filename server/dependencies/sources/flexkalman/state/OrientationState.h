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

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace flexkalman {

namespace orient_externalized_rotation {
    constexpr size_t Dimension = 6;
    using StateVector = types::Vector<Dimension>;
    using StateVectorBlock3 = StateVector::FixedSegmentReturnType<3>::Type;
    using ConstStateVectorBlock3 =
        StateVector::ConstFixedSegmentReturnType<3>::Type;

    using StateSquareMatrix = types::SquareMatrix<Dimension>;

    /*!
     * This returns A(deltaT), though if you're just predicting xhat-, use
     * applyVelocity() instead for performance.
     */
    inline StateSquareMatrix stateTransitionMatrix(double dt) {
        StateSquareMatrix A = StateSquareMatrix::Identity();
        A.topRightCorner<3, 3>() = types::SquareMatrix<3>::Identity() * dt;
        return A;
    }
    inline StateSquareMatrix
    stateTransitionMatrixWithVelocityDamping(double dt, double damping) {

        // eq. 4.5 in Welch 1996

        auto A = stateTransitionMatrix(dt);
        auto attenuation = std::pow(damping, dt);
        A.bottomRightCorner<3, 3>() *= attenuation;
        return A;
    }
    class State : public StateBase<State> {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static constexpr size_t Dimension = 6;

        //! Default constructor
        State()
            : m_state(StateVector::Zero()),
              m_errorCovariance(
                  StateSquareMatrix::
                      Identity() /** @todo almost certainly wrong */),
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

        //! Intended for startup use.
        void setQuaternion(Eigen::Quaterniond const &quaternion) {
            m_orientation = quaternion.normalized();
        }

        void postCorrect() { externalizeRotation(); }

        void externalizeRotation() {
            m_orientation = getCombinedQuaternion();
            incrementalOrientation() = Eigen::Vector3d::Zero();
        }

        void normalizeQuaternion() { m_orientation.normalize(); }

        StateVectorBlock3 incrementalOrientation() { return m_state.head<3>(); }

        ConstStateVectorBlock3 incrementalOrientation() const {
            return m_state.head<3>();
        }

        StateVectorBlock3 angularVelocity() { return m_state.tail<3>(); }

        ConstStateVectorBlock3 angularVelocity() const {
            return m_state.tail<3>();
        }

        Eigen::Quaterniond const &getQuaternion() const {
            return m_orientation;
        }

        Eigen::Quaterniond getCombinedQuaternion() const {
            // divide by 2 since we're integrating it essentially.
            return util::quat_exp(incrementalOrientation() / 2.) *
                   m_orientation;
        }

      private:
        /*!
         * In order: x, y, z, orientation , then its derivatives in the
         * same
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

        state.incrementalOrientation() += state.angularVelocity() * dt;
    }

    inline void dampenVelocities(State &state, double damping, double dt) {
        auto attenuation = std::pow(damping, dt);
        state.angularVelocity() *= attenuation;
    }

} // namespace orient_externalized_rotation

} // namespace flexkalman
