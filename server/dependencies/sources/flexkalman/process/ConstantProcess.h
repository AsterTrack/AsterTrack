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

// Library/third-party includes
// - none

// Standard includes
// - none

namespace flexkalman {

/*!
 * A simple process model for a "constant" process: all prediction does at
 * most is bump up the uncertainty. Since it's widely applicable, it's
 * templated on state type.
 *
 * One potential application is for beacon autocalibration in a device
 * filter.
 */
template <typename StateType>
class ConstantProcess : public ProcessModelBase<ConstantProcess<StateType>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = StateType;
    static constexpr size_t Dimension = getDimension<State>();
    using StateVector = types::Vector<Dimension>;
    using StateSquareMatrix = types::SquareMatrix<Dimension>;
    ConstantProcess() : m_constantNoise(StateSquareMatrix::Zero()) {}
    void predictState(State &state, double dt) {

        // Predict a-priori P
        // The formula for this prediction is AP(A^T) + Q, where Q is
        // getSampledProcessNoiseCovariance, and A is
        // getStateTransitionMatrix, both optional process model methods.
        // Since the state transition matrix for this, a constant process,
        // is just the identity, this simplifies to a sum, so we just
        // directly do the computation here rather than calling the
        // predictErrorCovariance() free function.
        StateSquareMatrix Pminus =
            state.errorCovariance() + dt * m_constantNoise;
        state.setErrorCovariance(Pminus);
    }
    void setNoiseAutocorrelation(double noise) {
        m_constantNoise = StateVector::Constant(noise).asDiagonal();
    }

    void setNoiseAutocorrelation(StateVector const &noise) {
        m_constantNoise = noise.asDiagonal();
    }

  private:
    StateSquareMatrix m_constantNoise;
};

} // namespace flexkalman
