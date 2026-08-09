/** @file
    @brief Header

    @date 2015-2019

    @author
    Seneral
    <seneral@seneral.dev>

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

#include "flexkalman/BaseTypes.h"
#include "flexkalman/FlexibleKalmanBase.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cassert>

namespace flexkalman
{

namespace pos_marker
{
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
        // eq. 4.5 in Welch 1996 - except we have all the velocities at the
        // end
        StateSquareMatrix A = StateSquareMatrix::Identity();
        A.topRightCorner<3, 3>() = types::SquareMatrix<3>::Identity() * dt;
        return A;
    }
    /*!
     * Function used to compute the coefficient m in v_new = m * v_old.
     * The damping value is for exponential decay.
     */
    inline double computeAttenuation(double damping, double dt) {
        return std::pow(damping, dt);
    }

    class State : public StateBase<State>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static constexpr size_t Dimension = 6;
        using StateVector = types::Vector<Dimension>;
        using StateSquareMatrix = types::SquareMatrix<Dimension>;

        State() : m_state(StateVector::Zero()),
            m_errorCovariance(StateSquareMatrix::Identity() * 10) {}

        void setStateVector(StateVector const &state) { m_state = state; }

        StateVector const &stateVector() const { return m_state; }

        void setErrorCovariance(StateSquareMatrix const &errorCovariance) {
            m_errorCovariance = errorCovariance;
        }
        StateSquareMatrix const &errorCovariance() const { return m_errorCovariance; }

        StateSquareMatrix &errorCovariance() { return m_errorCovariance; }

        void postCorrect() { }

        StateVectorBlock3 position() { return m_state.head<3>(); }

        ConstStateVectorBlock3 position() const { return m_state.head<3>(); }

        StateVectorBlock3 velocity() { return m_state.tail<3>(); }

        ConstStateVectorBlock3 velocity() const { return m_state.tail<3>(); }

      private:
        StateVector m_state;
        StateSquareMatrix m_errorCovariance;
    };

    //! Computes A(deltaT)xhat(t-deltaT)
    inline void applyVelocity(State &state, double dt) {
        // eq. 4.5 in Welch 1996
        state.position() += state.velocity() * dt;
    }

    //! Dampen all 3 components of velocity by a single factor.
    inline void dampenVelocity(State &state, double damping, double dt) {
        auto attenuation = computeAttenuation(damping, dt);
        state.velocity() *= attenuation;
    }

    inline StateSquareMatrix stateTransitionMatrix(State const &state, double dt) {
        return stateTransitionMatrix(dt);
    }
    /*!
     * Returns the state transition matrix for a constant velocity with a
     * single damping parameter (not for direct use in computing state
     * transition, because it is very sparse, but in computing other
     * values)
     */
    inline StateSquareMatrix
    stateTransitionMatrixWithVelocityDamping(State const &state, double dt, double damping) {
        // eq. 4.5 in Welch 1996
        auto A = stateTransitionMatrix(state, dt);
        A.bottomRightCorner<3, 3>() *= computeAttenuation(damping, dt);
        return A;
    }
} // namespace pos_marker

//! A constant-velocity model for a 3DOF position (with velocity)
class MarkerConstantVelocityProcessModel
    : public ProcessModelBase<MarkerConstantVelocityProcessModel>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = pos_marker::State;
    using StateVector = pos_marker::StateVector;
    using StateSquareMatrix = pos_marker::StateSquareMatrix;
    using NoiseAutocorrelation = types::Vector<3>;
    MarkerConstantVelocityProcessModel(double positionNoise = 0.01) {
        setNoiseAutocorrelation(positionNoise);
    }
    void setNoiseAutocorrelation(double positionNoise = 0.01) {
        m_mu = types::Vector<3>::Constant(positionNoise);
    }
    void setNoiseAutocorrelation(NoiseAutocorrelation const &noise) {
        m_mu = noise;
    }

    //! Also known as the "process model jacobian" in TAG, this is A.
    StateSquareMatrix getStateTransitionMatrix(State const &, double dt) const {
        return pos_marker::stateTransitionMatrix(dt);
    }

    //! Does not update error covariance
    void predictStateOnly(State &s, double dt) const {
        pos_marker::applyVelocity(s, dt);
    }
    //! Updates state vector and error covariance
    void predictState(State &s, double dt) const {
        predictStateOnly(s, dt);
        auto Pminus = predictErrorCovariance(s, *this, dt);
        s.setErrorCovariance(Pminus);
    }

    /*!
     * This is Q(deltaT) - the Sampled Process Noise Covariance
     * @return a matrix of dimension n x n.
     *
     * Like all covariance matrices, it is real symmetrical (self-adjoint),
     * so .selfAdjointView<Eigen::Upper>() might provide useful performance
     * enhancements in some algorithms.
     */
    StateSquareMatrix getSampledProcessNoiseCovariance(double dt) const {
        constexpr auto dim = getDimension<State>();
        StateSquareMatrix cov = StateSquareMatrix::Zero();
        auto dt3 = (dt * dt * dt) / 3;
        auto dt2 = (dt * dt) / 2;
        for (std::size_t xIndex = 0; xIndex < dim / 2; ++xIndex) {
            auto xDotIndex = xIndex + dim / 2;
            // xIndex is 'i' and xDotIndex is 'j' in eq. 4.8
            const auto mu = getMu(xIndex);
            cov(xIndex, xIndex) = mu * dt3;
            auto symmetric = mu * dt2;
            cov(xIndex, xDotIndex) = symmetric;
            cov(xDotIndex, xIndex) = symmetric;
            cov(xDotIndex, xDotIndex) = mu * dt;
        }
        return cov;
    }

  private:
    /*!
     * this is mu-arrow, the auto-correlation vector of the noise
     * sources
     */
    NoiseAutocorrelation m_mu;
    double getMu(std::size_t index) const {
        assert(index < (getDimension<State>() / 2) &&
               "Should only be passing "
               "'i' - the main state, not "
               "the derivative");
        // This may not be totally correct but it's one of the parameters
        // you can kind of fudge in kalman filters anyway.
        // Should techincally be the diagonal of the correlation kernel of
        // the noise sources. (p77, p197 in Welch 1996)
        return m_mu(index);
    }
};

/*!
 * A basically-constant-velocity model, with the addition of some
 * damping of the velocities inspired by TAG
 */
class MarkerDampedConstantVelocityProcessModel
    : public ProcessModelBase<MarkerDampedConstantVelocityProcessModel>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = pos_marker::State;
    using StateVector = pos_marker::StateVector;
    using StateSquareMatrix = pos_marker::StateSquareMatrix;
    using BaseProcess = MarkerConstantVelocityProcessModel;
    using NoiseAutocorrelation = BaseProcess::NoiseAutocorrelation;
    MarkerDampedConstantVelocityProcessModel(double damping = 0.1, double positionNoise = 0.01)
        : m_constantVelModel(positionNoise) {
        setDamping(damping);
    }

    void setNoiseAutocorrelation(double positionNoise = 0.01) {
        m_constantVelModel.setNoiseAutocorrelation(positionNoise);
    }

    void setNoiseAutocorrelation(NoiseAutocorrelation const &noise) {
        m_constantVelModel.setNoiseAutocorrelation(noise);
    }
    //! Set the damping - must be positive
    void setDamping(double damping) {
        if (damping > 0) {
            m_damp = damping;
        }
    }

    //! Also known as the "process model jacobian" in TAG, this is A.
    StateSquareMatrix getStateTransitionMatrix(State const &s, double dt) const {
        return pos_marker::
            stateTransitionMatrixWithVelocityDamping(s, dt, m_damp);
    }

    void predictStateOnly(State &s, double dt) const {
        m_constantVelModel.predictStateOnly(s, dt);
        // Dampen velocities
        pos_marker::dampenVelocity(s, m_damp, dt);
    }

    void predictState(State &s, double dt) const {
        predictStateOnly(s, dt);
        auto Pminus = predictErrorCovariance(s, *this, dt);
        s.setErrorCovariance(Pminus);
    }

    /*!
     * This is Q(deltaT) - the Sampled Process Noise Covariance
     * @return a matrix of dimension n x n. Note that it is real
     * symmetrical (self-adjoint), so .selfAdjointView<Eigen::Upper>()
     * might provide useful performance enhancements.
     */
    StateSquareMatrix getSampledProcessNoiseCovariance(double dt) const {
        return m_constantVelModel.getSampledProcessNoiseCovariance(dt);
    }

  private:
    BaseProcess m_constantVelModel;
    double m_damp = 0.1;
};

} // namespace flexkalman
