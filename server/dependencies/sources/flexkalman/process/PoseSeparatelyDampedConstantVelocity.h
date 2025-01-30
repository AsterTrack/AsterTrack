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
#include "flexkalman/process/PoseConstantVelocityGeneric.h"

// Library/third-party includes
// - none

// Standard includes
#include <cassert>

namespace flexkalman {

/*!
 * A basically-constant-velocity model, with the addition of some
 * damping of the velocities inspired by TAG. This model has separate
 * damping/attenuation of linear and angular velocities.
 */
template <typename StateType>
class PoseSeparatelyDampedConstantVelocityProcessModel
    : public ProcessModelBase<
          PoseSeparatelyDampedConstantVelocityProcessModel<StateType>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using State = StateType;
    using StateVector = typename StateType::StateVector;
    using StateSquareMatrix = typename StateType::StateSquareMatrix;
    using BaseProcess = PoseConstantVelocityGenericProcessModel<State>;
    using NoiseAutocorrelation = typename BaseProcess::NoiseAutocorrelation;
    PoseSeparatelyDampedConstantVelocityProcessModel(
        double positionDamping = 0.3, double orientationDamping = 0.01,
        double positionNoise = 0.01, double orientationNoise = 0.1)
        : m_constantVelModel(positionNoise, orientationNoise) {
        setDamping(positionDamping, orientationDamping);
    }

    void setNoiseAutocorrelation(double positionNoise = 0.01,
                                 double orientationNoise = 0.1) {
        m_constantVelModel.setNoiseAutocorrelation(positionNoise,
                                                   orientationNoise);
    }

    void setNoiseAutocorrelation(NoiseAutocorrelation const &noise) {
        m_constantVelModel.setNoiseAutocorrelation(noise);
    }
    //! Set the damping - must be in (0, 1)
    void setDamping(double posDamping, double oriDamping) {
        if (posDamping > 0 && posDamping < 1) {
            m_posDamp = posDamping;
        }
        if (oriDamping > 0 && oriDamping < 1) {
            m_oriDamp = oriDamping;
        }
    }

    //! Also known as the "process model jacobian" in TAG, this is A.
    StateSquareMatrix getStateTransitionMatrix(State const &s,
                                               double dt) const {
        // using argument-dependent lookup
        return stateTransitionMatrixWithSeparateVelocityDamping(
            s, dt, m_posDamp, m_oriDamp);
    }

    void predictStateOnly(State &s, double dt) const {
        m_constantVelModel.predictStateOnly(s, dt);
        // Dampen velocities - using argument-dependent lookup
        separatelyDampenVelocities(s, m_posDamp, m_oriDamp, dt);
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
    double m_posDamp = 0.2;
    double m_oriDamp = 0.01;
};

} // namespace flexkalman
