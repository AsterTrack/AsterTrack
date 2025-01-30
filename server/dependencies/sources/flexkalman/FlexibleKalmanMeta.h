/** @file
    @brief Header including all of the FlexKalman flexible Kalman filter
   framework headers

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

#ifndef FLEXKALMAN_DEBUG_OUTPUT
#define FLEXKALMAN_DEBUG_OUTPUT(Name, Value)
#endif

/*!
 * @brief Header-only framework for building Kalman-style filters, prediction,
 * and sensor fusion
 */
namespace flexkalman {
// NOTE: Everything in this file up through the preceding line
// will be included, in perhaps slightly-modified form,
// in the combined headers generated from this library.

} // namespace flexkalman

// Full-include list simply generated as follows. (The header combiner does the
// topological sort.) $ ls *.h | grep -v FlexibleKalmanMeta.h | sort | sed -e
// 's/^/#include "/' -e 's/$/"/'

#include "AbsoluteOrientationMeasurement.h"
#include "AbsolutePositionMeasurement.h"
#include "AngularVelocityMeasurement.h"
#include "AugmentedProcessModel.h"
#include "AugmentedState.h"
#include "BaseTypes.h"
#include "ConstantProcess.h"
#include "EigenQuatExponentialMap.h"
#include "ExternalQuaternion.h"
#include "FlexibleKalmanBase.h"
#include "FlexibleKalmanCorrect.h"
#include "FlexibleKalmanFilter.h"
#include "FlexibleUnscentedCorrect.h"
#include "MatrixExponentialMap.h"
#include "OrientationConstantVelocity.h"
#include "OrientationState.h"
#include "PoseConstantAccel.h"
#include "PoseConstantVelocity.h"
#include "PoseConstantVelocityGeneric.h"
#include "PoseDampedConstantVelocity.h"
#include "PoseSeparatelyDampedConstantVelocity.h"
#include "PoseState.h"
#include "PoseStateExponentialMap.h"
#include "PoseStateWithAccel.h"
#include "PureVectorState.h"
#include "SO3.h"
#include "SigmaPointGenerator.h"
