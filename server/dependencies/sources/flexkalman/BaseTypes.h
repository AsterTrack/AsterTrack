/** @file
    @brief Header

    @date 2019

    @author
    Rylie Pavlik
    <rylie.pavlik@collabora.com>
*/

// Copyright 2019 Collabora, Ltd.
//
// SPDX-License-Identifier: Apache-2.0 OR BSL-1.0
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
// - None

// Library/third-party includes
// - None

// Standard includes
// - None

namespace flexkalman {

/*!
 * @brief CRTP base for state types.
 *
 * All your State types should derive from this template, passing themselves as
 * the Derived type.
 */
template <typename Derived> class StateBase {
  public:
    Derived &derived() noexcept { return *static_cast<Derived *>(this); }
    Derived const &derived() const noexcept {
        return *static_cast<Derived const *>(this);
    }
};

/*!
 * @brief CRTP base for measurement types.
 *
 * All your Measurement types should derive from this template, passing
 * themselves as the Derived type.
 */
template <typename Derived> class MeasurementBase {
  public:
    Derived &derived() noexcept { return *static_cast<Derived *>(this); }
    Derived const &derived() const noexcept {
        return *static_cast<Derived const *>(this);
    }
};

/*!
 * @brief CRTP base for process model types.
 *
 * All your ProcessModel types should derive from this template, passing
 * themselves as the Derived type.
 */
template <typename Derived> class ProcessModelBase {
  public:
    Derived &derived() noexcept { return *static_cast<Derived *>(this); }
    Derived const &derived() const noexcept {
        return *static_cast<Derived const *>(this);
    }
};
} // namespace flexkalman
