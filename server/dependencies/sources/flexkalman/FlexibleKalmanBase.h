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
// - none

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
#include <type_traits>

#ifndef FLEXKALMAN_DEBUG_OUTPUT
#define FLEXKALMAN_DEBUG_OUTPUT(Name, Value)
#endif

namespace flexkalman {

//! @brief Type aliases, including template type aliases.
namespace types {
    //! Common scalar type
    using Scalar = double;
} // namespace types

/*!
 * Convenience base class for things (like states and measurements) that
 * have a dimension.
 */
template <size_t DIM> struct HasDimension {
    static constexpr size_t Dimension = DIM;
};

template <typename T> static constexpr size_t getDimension() {
    return T::Dimension;
}

namespace types {
    //! Given a filter type, get the state type.
    template <typename FilterType> using StateType = typename FilterType::State;

    //! Given a filter type, get the process model type.
    template <typename FilterType>
    using ProcessModelType = typename FilterType::ProcessModel;

    //! A vector of length n
    template <size_t n> using Vector = Eigen::Matrix<Scalar, n, 1>;

    //! A square matrix, n x n
    template <size_t n> using SquareMatrix = Eigen::Matrix<Scalar, n, n>;

    //! A square diagonal matrix, n x n
    template <size_t n> using DiagonalMatrix = Eigen::DiagonalMatrix<Scalar, n>;

    //! A matrix with rows = m,  cols = n
    template <size_t m, size_t n> using Matrix = Eigen::Matrix<Scalar, m, n>;

    //! A matrix with rows = dimension of T, cols = dimension of U
    template <typename T, typename U>
    using DimMatrix = Matrix<T::Dimension, U::Dimension>;

} // namespace types

/*!
 * Computes P-
 *
 * Usage is optional, most likely called from the process model
 * `updateState()`` method.
 */
template <typename StateType, typename ProcessModelType>
inline types::SquareMatrix<getDimension<StateType>()>
predictErrorCovariance(StateType const &state, ProcessModelType &processModel,
                       double dt) {
    const auto A = processModel.getStateTransitionMatrix(state, dt);
    // FLEXKALMAN_DEBUG_OUTPUT("State transition matrix", A);
    auto &&P = state.errorCovariance();
    /*!
     * @todo Determine if the fact that Q is (at least in one case)
     * symmetrical implies anything else useful performance-wise here or
     * later in the data flow.
     */
    // auto Q = processModel.getSampledProcessNoiseCovariance(dt);
    FLEXKALMAN_DEBUG_OUTPUT("Process Noise Covariance Q",
                            processModel.getSampledProcessNoiseCovariance(dt));
    return A * P * A.transpose() +
           processModel.getSampledProcessNoiseCovariance(dt);
}

} // namespace flexkalman
