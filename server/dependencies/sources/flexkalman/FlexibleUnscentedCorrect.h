/** @file
    @brief Header for a flexible Unscented-style Kalman filter correction from a
    measurement.

    Conventions generally are based on the following publication:
    van der Merwe, R., Wan, E. A., & Julier, S. J. (2004). "Sigma-Point Kalman
    Filters for Nonlinear Estimation and Sensor-Fusion: Applications to
    Integrated Navigation." In AIAA Guidance, Navigation, and Control Conference
    and Exhibit (pp. 1--30). Reston, Virginia: American Institute of Aeronautics
    and Astronautics. http://doi.org/10.2514/6.2004-5120

    @date 2016-2019

    @author
    Rylie Pavlik
    <rylie.pavlik@collabora.com>

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
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
#include "SigmaPointGenerator.h"

// Library/third-party includes
// - none

// Standard includes
// - none

namespace flexkalman {

template <typename Derived> class StateBase;
template <typename Derived> class MeasurementBase;
template <typename Derived> class ProcessModelBase;

/*!
 * The UKF parallel to CorrectionInProgress as used in an EKF.
 *
 * Initialization is done by beginUnscentedCorrection (like
 * beginExtendedCorrection). stateCorrectionFinite is provided immediately,
 * while the finishCorrection() method takes an optional bool (true by default)
 * to optionally cancel if the new error covariance is not finite.
 */
template <typename State, typename Measurement>
class SigmaPointCorrectionApplication {
  public:
    static constexpr size_t n = getDimension<State>();
    static constexpr size_t m = getDimension<Measurement>();

    using StateVec = types::Vector<n>;
    using StateSquareMatrix = types::SquareMatrix<n>;
    using MeasurementVec = types::Vector<m>;
    using MeasurementSquareMatrix = types::SquareMatrix<m>;

    //! state augmented with measurement noise mean
    static constexpr size_t AugmentedStateDim = n + m;
    using AugmentedStateVec = types::Vector<AugmentedStateDim>;
    using AugmentedStateCovMatrix = types::SquareMatrix<AugmentedStateDim>;
    using SigmaPointsGen = AugmentedSigmaPointGenerator<AugmentedStateDim, n>;

    static constexpr size_t NumSigmaPoints = SigmaPointsGen::NumSigmaPoints;

    using Reconstruction =
        ReconstructedDistributionFromSigmaPoints<m, SigmaPointsGen>;
    using TransformedSigmaPointsMat =
        typename Reconstruction::TransformedSigmaPointsMat;

    using GainMatrix = types::Matrix<n, m>;

    SigmaPointCorrectionApplication(
        State &s, Measurement &meas,
        SigmaPointParameters const &params = SigmaPointParameters())
        : state(s), measurement(meas),
          sigmaPoints(getAugmentedStateVec(s),
                      getAugmentedStateCov(s, measurement), params),
          transformedPoints(
              transformSigmaPoints(state, measurement, sigmaPoints)),
          reconstruction(sigmaPoints, transformedPoints),
          innovationCovariance(
              computeInnovationCovariance(state, measurement, reconstruction)),
          PvvDecomp(innovationCovariance.ldlt()),
          deltaz(measurement.getResidual(reconstruction.getMean(), state)),
          stateCorrection(
              computeStateCorrection(reconstruction, deltaz, PvvDecomp)),
          stateCorrectionFinite(stateCorrection.array().allFinite()) {}

    static AugmentedStateVec getAugmentedStateVec(State const &s) {
        AugmentedStateVec ret;
        //! assuming measurement noise is zero mean
        ret << s.stateVector(), MeasurementVec::Zero();
        return ret;
    }

    static AugmentedStateCovMatrix getAugmentedStateCov(State const &s,
                                                        Measurement &meas) {
        AugmentedStateCovMatrix ret;
        ret << s.errorCovariance(), types::Matrix<n, m>::Zero(),
            types::Matrix<m, n>::Zero(), meas.getCovariance(s);
        return ret;
    }

    /*!
     * Transforms sigma points by having the measurement class compute the
     * estimated measurement for a state whose state vector we update to
     * each of the sigma points in turn.
     */
    static TransformedSigmaPointsMat
    transformSigmaPoints(State const &s, Measurement &meas,
                         SigmaPointsGen const &sigmaPoints) {
        TransformedSigmaPointsMat ret;
        State tempS = s;
        for (std::size_t i = 0; i < NumSigmaPoints; ++i) {
            tempS.setStateVector(sigmaPoints.getSigmaPoint(i));
            ret.col(i) = meas.predictMeasurement(tempS);
        }
        return ret;
    }

    static MeasurementSquareMatrix
    computeInnovationCovariance(State const &s, Measurement &meas,
                                Reconstruction const &recon) {
        return recon.getCov() + meas.getCovariance(s);
    }

#if 0
    // Solve for K in K=Pxy Pvv^-1
    // where the cross-covariance matrix from the reconstruction is
    // transpose(Pxy) and Pvv is the reconstructed covariance plus the
    // measurement covariance
    static GainMatrix computeKalmanGain(MeasurementSquareMatrix const &Pvv,
                                        Reconstruction const &recon) {
        // (Actually solves with transpose(Pvv) * transpose(K) =
        // transpose(Pxy) )
        GainMatrix ret = Pvv.transpose().ldlt().solve(recon.getCrossCov());
        return ret;
    }
#endif
    static StateVec computeStateCorrection(
        Reconstruction const &recon, MeasurementVec const &deltaz,
        Eigen::LDLT<MeasurementSquareMatrix> const &pvvDecomp) {
        StateVec ret = recon.getCrossCov() * pvvDecomp.solve(deltaz);
        return ret;
    }

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
        /*!
         * Logically state.errorCovariance() - K * Pvv * K.transpose(),
         * but considering just the second term, we can
         * replace K with its definition (Pxv Pvv^-1), distribute the
         * transpose on the right over the product, then pull out
         * Pvv^-1 * Pvv * (Pvv^-1).transpose()
         * as "B", leaving Pxv B Pxv.transpose()
         *
         * Since innovationCovariance aka Pvv is symmetric,
         * (Pvv^-1).transpose() = Pvv^-1.
         * Left multiplication gives
         * Pvv B = Pvv * Pvv^-1 * Pvv * Pvv^-1
         * whose right hand side is the Pvv-sized identity, and that is in
         * a form that allows us to use our existing LDLT decomp of Pvv to
         * solve for B then evaluate the full original expression.
         */
        StateSquareMatrix newP =
            state.errorCovariance() -
            reconstruction.getCrossCov() *
                PvvDecomp.solve(MeasurementSquareMatrix::Identity()) *
                reconstruction.getCrossCov().transpose();
        bool finite = newP.array().allFinite();
        if (cancelIfNotFinite && !finite) {
            return false;
        }

        state.setStateVector(state.stateVector() + stateCorrection);

        state.setErrorCovariance(newP);
        // Let the state do any cleanup it has to (like fixing externalized
        // quaternions)
        state.postCorrect();
        return finite;
    }

    State &state;
    Measurement &measurement;
    SigmaPointsGen sigmaPoints;
    TransformedSigmaPointsMat transformedPoints;
    Reconstruction reconstruction;
    //! aka Pvv
    MeasurementSquareMatrix innovationCovariance;
    Eigen::LDLT<MeasurementSquareMatrix> PvvDecomp;
#if 0
    GainMatrix K;
#endif
    //! reconstructed mean measurement residual/delta z/innovation
    types::Vector<m> deltaz;
    StateVec stateCorrection;
    bool stateCorrectionFinite;
};

template <typename State, typename Measurement>
inline SigmaPointCorrectionApplication<State, Measurement>
beginUnscentedCorrection(
    StateBase<State> &state, MeasurementBase<Measurement> &meas,
    SigmaPointParameters const &params = SigmaPointParameters()) {
    return SigmaPointCorrectionApplication<State, Measurement>(
        state.derived(), meas.derived(), params);
}

/*!
 * Correct a Kalman filter's state using a measurement that provides a
 * two-parameter getResidual function, in the manner of an
 * Unscented Kalman Filter (UKF).
 *
 * @param cancelIfNotFinite If the state correction or new error covariance
 * is detected to contain non-finite values, should we cancel the
 * correction and not apply it?
 *
 * @return true if correction completed
 */
template <typename State, typename Measurement>
static inline bool
correctUnscented(StateBase<State> &state, MeasurementBase<Measurement> &meas,
                 bool cancelIfNotFinite = true,
                 SigmaPointParameters const &params = SigmaPointParameters()) {

    auto inProgress = beginUnscentedCorrection(state, meas, params);
    if (cancelIfNotFinite && !inProgress.stateCorrectionFinite) {
        return false;
    }

    return inProgress.finishCorrection(cancelIfNotFinite);
}
} // namespace flexkalman
