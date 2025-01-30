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

// Copyright 2015-2016 Sensics, Inc.
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
#include "FlexibleKalmanBase.h"

// Library/third-party includes
// - none

// Standard includes
// - none

namespace flexkalman {

template <typename Derived> class StateBase;
template <typename Derived> class MeasurementBase;
template <typename Derived> class ProcessModelBase;

template <typename StateType, typename MeasurementType>
struct CorrectionInProgress {
    //! Dimension of measurement
    static constexpr size_t m = getDimension<MeasurementType>();
    //! Dimension of state
    static constexpr size_t n = getDimension<StateType>();

    CorrectionInProgress(StateType &state, MeasurementType &meas,
                         types::SquareMatrix<n> const &P_,
                         types::Matrix<n, m> const &PHt_,
                         types::SquareMatrix<m> const &S)
        : P(P_), PHt(PHt_), denom(S), deltaz(meas.getResidual(state)),
          stateCorrection(PHt * denom.solve(deltaz)), state_(state),
          stateCorrectionFinite(stateCorrection.array().allFinite()) {}

    //! State error covariance
    types::SquareMatrix<n> P;

    //! The kalman gain stuff to not invert (called P12 in TAG)
    types::Matrix<n, m> PHt;

    /*!
     * Decomposition of S
     *
     * Not going to directly compute Kalman gain K = PHt (S^-1)
     * Instead, decomposed S to solve things of the form (S^-1)x
     * repeatedly later, by using the substitution
     * Kx = PHt denom.solve(x)
     * @todo Figure out if this is the best decomp to use
     */
    // TooN/TAG use this one, and others online seem to suggest it.
    Eigen::LDLT<types::SquareMatrix<m>> denom;

    //! Measurement residual/delta z/innovation
    types::Vector<m> deltaz;

    //! Corresponding state change to apply.
    types::Vector<n> stateCorrection;

    //! Is the state correction free of NaNs and +- infs?
    bool stateCorrectionFinite;

    //! That's as far as we go here before you choose to continue.

    /*!
     * Finish computing the rest and correct the state.
     *
     * @param cancelIfNotFinite If the new error covariance is detected to
     * contain non-finite values, should we cancel the correction and not
     * apply it?
     *
     * @return true if correction completed
     */
    bool finishCorrection(bool cancelIfNotFinite = true) {
        // Compute the new error covariance
        // differs from the (I-KH)P form by not factoring out the P (since
        // we already have PHt computed).
        types::SquareMatrix<n> newP = P - (PHt * denom.solve(PHt.transpose()));

#if 0
        // Test fails with this one:
        // VariedProcessModelStability/1.AbsolutePoseMeasurementXlate111,
        // where TypeParam =
        // flexkalman::PoseDampedConstantVelocityProcessModel
        FLEXKALMAN_DEBUG_OUTPUT(
            "error covariance scale",
            (types::SquareMatrix<n>::Identity() - PHt * denom.solve(H)));
        types::SquareMatrix<n> newP =
            (types::SquareMatrix<n>::Identity() - PHt * denom.solve(H)) * P;
#endif

        if (!newP.array().allFinite()) {
            return false;
        }

        // Correct the state estimate
        state_.setStateVector(state_.stateVector() + stateCorrection);

        // Correct the error covariance
        state_.setErrorCovariance(newP);

#if 0
        // Doesn't seem necessary to re-symmetrize the covariance matrix.
        state_.setErrorCovariance((newP + newP.transpose()) / 2.);
#endif

        // Let the state do any cleanup it has to (like fixing externalized
        // quaternions)
        state_.postCorrect();
        return true;
    }

  private:
    StateType &state_;
};

template <typename State, typename ProcessModel, typename Measurement>
inline CorrectionInProgress<State, Measurement>
beginExtendedCorrection(StateBase<State> &state,
                        ProcessModelBase<ProcessModel> &processModel,
                        MeasurementBase<Measurement> &meas) {

    //! Dimension of measurement
    static constexpr size_t m = getDimension<Measurement>();
    //! Dimension of state
    static constexpr size_t n = getDimension<State>();

    //! Measurement Jacobian
    types::Matrix<m, n> H = meas.derived().getJacobian(state.derived());

    //! Measurement covariance
    types::SquareMatrix<m> R = meas.derived().getCovariance(state.derived());

    //! State error covariance
    types::SquareMatrix<n> P = state.derived().errorCovariance();

    //! The kalman gain stuff to not invert (called P12 in TAG)
    types::Matrix<n, m> PHt = P * H.transpose();

    /*!
     * the stuff to invert for the kalman gain
     * also sometimes called S or the "Innovation Covariance"
     */
    types::SquareMatrix<m> S = H * PHt + R;

    //! More computation is done in initializers/constructor
    return {state.derived(), meas.derived(), P, PHt, S};
}

/*!
 * Correct a Kalman filter's state using a measurement that provides a
 * Jacobian, in the manner of an Extended Kalman Filter (EKF).
 *
 * @param cancelIfNotFinite If the state correction or new error covariance
 * is detected to contain non-finite values, should we cancel the
 * correction and not apply it?
 *
 * @return true if correction completed
 */
template <typename State, typename ProcessModel, typename Measurement>
static inline bool correctExtended(StateBase<State> &state,
                                   ProcessModelBase<ProcessModel> &processModel,
                                   MeasurementBase<Measurement> &meas,
                                   bool cancelIfNotFinite = true) {

    auto inProgress = beginExtendedCorrection(state, processModel, meas);
    if (cancelIfNotFinite && !inProgress.stateCorrectionFinite) {
        return false;
    }

    return inProgress.finishCorrection(cancelIfNotFinite);
}

//! Delegates to correctExtended, a more explicit name which is preferred.
template <typename State, typename ProcessModel, typename Measurement>
static inline bool
correct(StateBase<State> &state, ProcessModelBase<ProcessModel> &processModel,
        MeasurementBase<Measurement> &meas, bool cancelIfNotFinite = true) {
    return correctExtended(state, processModel, meas, cancelIfNotFinite);
}

} // namespace flexkalman
