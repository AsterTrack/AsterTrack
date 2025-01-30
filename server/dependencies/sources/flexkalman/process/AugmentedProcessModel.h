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
#include "flexkalman/state/AugmentedState.h"

// Library/third-party includes
// - none

// Standard includes
#include <functional>
#include <type_traits>

namespace flexkalman {

/*!
 * Process model type that consists entirely of references to two
 * sub-process models, for operating on an AugmentedState<>.
 */
template <typename ModelA, typename ModelB>
class AugmentedProcessModel
    : public ProcessModelBase<AugmentedProcessModel<ModelA, ModelB>> {
  public:
    using ModelTypeA = ModelA;
    using ModelTypeB = ModelB;
    using StateA = typename ModelA::State;
    using StateB = typename ModelB::State;
    using State = AugmentedState<StateA, StateB>;

    //! Constructor
    AugmentedProcessModel(ModelTypeA &modA, ModelTypeB &modB)
        : a_(std::ref(modA)), b_(std::ref(modB)) {}

    //! @name Method required of Process Model types
    /// @{
    void predictState(State &state, double dt) {
        modelA().predictState(state.a(), dt);
        modelB().predictState(state.b(), dt);
    }
    //! @}

    //! @name Access to the components of the process model
    /// @{
    ModelTypeA &modelA() { return a_.get(); }
    ModelTypeA const &modelA() const { return a_.get(); }

    ModelTypeB &modelB() { return b_.get(); }
    ModelTypeB const &modelB() const { return b_.get(); }
    //! @}
  private:
    std::reference_wrapper<ModelTypeA> a_;
    std::reference_wrapper<ModelTypeB> b_;
};

/*!
 * Template alias to make removing const from the deduced types less
 * verbose/painful.
 */
template <typename ModelA, typename ModelB>
using DeducedAugmentedProcessModel =
    AugmentedProcessModel<typename std::remove_const<ModelA>::type,
                          typename std::remove_const<ModelB>::type>;

/*!
 * Factory function, akin to `std::tie()`, to make an augmented process
 * model.
 */
template <typename ModelA, typename ModelB>
inline DeducedAugmentedProcessModel<ModelA, ModelB>
makeAugmentedProcessModel(ModelA &a, ModelB &b) {
    return {a, b};
}

} // namespace flexkalman
