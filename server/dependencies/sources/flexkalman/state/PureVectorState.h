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
 * A very simple (3D by default) vector state with no velocity, ideal for
 * use as a position, with ConstantProcess for beacon autocalibration
 */
template <size_t Dim = 3>
class PureVectorState : public StateBase<PureVectorState<Dim>> {
  public:
    static constexpr size_t Dimension = Dim;
    using SquareMatrix = types::SquareMatrix<Dimension>;
    using StateVector = types::Vector<Dimension>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PureVectorState(double x, double y, double z)
        : m_state(x, y, z), m_errorCovariance(SquareMatrix::Zero()) {
        static_assert(Dimension == 3, "This constructor, which takes 3 "
                                      "scalars, only works with a 3D "
                                      "vector!");
    }

    PureVectorState(double x, double y, double z,
                    SquareMatrix const &covariance)
        : m_state(x, y, z), m_errorCovariance(covariance) {
        static_assert(Dimension == 3, "This constructor, which takes 3 "
                                      "scalars, only works with a 3D "
                                      "vector!");
    }

    PureVectorState(StateVector const &state, SquareMatrix const &covariance)
        : m_state(state), m_errorCovariance(covariance) {}
    //! @name Methods required of State types
    /// @{
    //! set xhat
    void setStateVector(StateVector const &state) { m_state = state; }
    //! xhat
    StateVector const &stateVector() const { return m_state; }
    // set P
    void setErrorCovariance(SquareMatrix const &errorCovariance) {
        m_errorCovariance = errorCovariance;
    }
    //! P
    SquareMatrix const &errorCovariance() const { return m_errorCovariance; }
    void postCorrect() {}
    //! @}
  private:
    //! x
    StateVector m_state;
    //! P
    SquareMatrix m_errorCovariance;
};

} // namespace flexkalman
