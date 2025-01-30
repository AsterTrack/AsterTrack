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
#include "FlexibleKalmanBase.h"
#include "FlexibleKalmanCorrect.h"

// Library/third-party includes
// - none

// Standard includes
// - none

namespace flexkalman {

template <typename Derived> class StateBase;
template <typename Derived> class MeasurementBase;
template <typename Derived> class ProcessModelBase;

/*!
 * Advance time in the filter, by applying the process model to the state with
 * the given dt.
 *
 * Usually followed by correct() or beginUnscentedCorrection() and its
 * continuation. If you aren't correcting immediately, make sure to run
 * `state.postCorrect()` to clean up. Otherwise, consider calling
 * getPrediction() instead.
 */
template <typename StateType, typename ProcessModelType>
static inline void predict(StateBase<StateType> &state,
                           ProcessModelBase<ProcessModelType> &processModel,
                           double dt) {
    processModel.derived().predictState(state.derived(), dt);
    FLEXKALMAN_DEBUG_OUTPUT("Predicted state",
                            state.derived().stateVector().transpose());

    FLEXKALMAN_DEBUG_OUTPUT("Predicted error covariance",
                            state.derived().errorCovariance());
}

/*!
 * Performs prediction of the state only (not the error covariance), as well as
 * post-correction. Unsuitable for continued correction for this reason, but
 * usable to get a look at a predicted value.
 *
 * Requires that your process model provide `processModel.predictStateOnly()`
 */
template <typename StateType, typename ProcessModelType>
static inline void
predictAndPostCorrectStateOnly(StateBase<StateType> &state,
                               ProcessModelBase<ProcessModelType> &processModel,
                               double dt) {
    processModel.derived().predictStateOnly(state.derived(), dt);
    state.derived().postCorrect();
    FLEXKALMAN_DEBUG_OUTPUT("Predicted state",
                            state.derived().stateVector().transpose());
}

/*!
 * Performs prediction on a copy of the state, as well as
 * post-correction. By default, also skips prediction of error covariance.
 *
 * Requires that your process model provide `processModel.predictStateOnly()`
 */
template <typename StateType, typename ProcessModelType>
static inline StateType
getPrediction(StateBase<StateType> const &state,
              ProcessModelBase<ProcessModelType> const &processModel, double dt,
              bool predictCovariance = false) {
    StateType stateCopy{state.derived()};
    if (predictCovariance) {
        processModel.derived().predictState(stateCopy, dt);
    } else {
        processModel.derived().predictStateOnly(stateCopy, dt);
    }
    stateCopy.postCorrect();
    return stateCopy;
}

} // namespace flexkalman
