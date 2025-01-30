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

// Internal Includes
#include "flexkalman/BaseTypes.h"
#include "flexkalman/process/PoseConstantVelocity.h"
#include "flexkalman/state/PoseState.h"

// Library/third-party includes
// - none

// Standard includes
#include <cassert>

namespace flexkalman {

/*!
 * A basically-constant-velocity model, with the addition of some
 * damping of the velocities inspired by TAG
 */
class PoseDampedConstantVelocityProcessModel
    : public ProcessModelBase<PoseDampedConstantVelocityProcessModel> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = pose_externalized_rotation::State;
    using StateVector = pose_externalized_rotation::StateVector;
    using StateSquareMatrix = pose_externalized_rotation::StateSquareMatrix;
    using BaseProcess = PoseConstantVelocityProcessModel;
    using NoiseAutocorrelation = BaseProcess::NoiseAutocorrelation;
    PoseDampedConstantVelocityProcessModel(double damping = 0.1,
                                           double positionNoise = 0.01,
                                           double orientationNoise = 0.1)
        : m_constantVelModel(positionNoise, orientationNoise) {
        setDamping(damping);
    }

    void setNoiseAutocorrelation(double positionNoise = 0.01,
                                 double orientationNoise = 0.1) {
        m_constantVelModel.setNoiseAutocorrelation(positionNoise,
                                                   orientationNoise);
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
    StateSquareMatrix getStateTransitionMatrix(State const &s,
                                               double dt) const {
        return pose_externalized_rotation::
            stateTransitionMatrixWithVelocityDamping(s, dt, m_damp);
    }

    void predictStateOnly(State &s, double dt) const {
        m_constantVelModel.predictStateOnly(s, dt);
        // Dampen velocities
        pose_externalized_rotation::dampenVelocities(s, m_damp, dt);
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
