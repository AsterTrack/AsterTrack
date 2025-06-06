/** @file
    @brief Header

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
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

// Library/third-party includes
#include <Eigen/Cholesky>

// Standard includes
#include <cstddef>
#include <cassert>

namespace flexkalman {

inline double computeNu(std::size_t L, double alpha) {
    auto lambda = (L * (alpha * alpha - 1));
    auto nu = std::sqrt(L + lambda);
    return nu;
}
/*!
 * For further details on the scaling factors, refer to:
 * Julier, S. J., & Uhlmann, J. K. (2004). Unscented filtering and
 * nonlinear estimation. Proceedings of the IEEE, 92(3), 401�422.
 * http://doi.org/10.1109/JPROC.2003.823141
 * Appendix V (for alpha), Appendix VI (for beta)
 */
struct SigmaPointParameters {
    SigmaPointParameters(double alpha_ = 0.001, double beta_ = 2.,
                         double kappa_ = 0.)
        : alpha(alpha_), beta(beta_), kappa(kappa_) {}
    /*!
     * double L;
     * Primary scaling factor, typically in the range [1e-4, 1]
     */
    double alpha;
    /*!
     * Secondary scaling to emphasize the 0th sigma point in covariance
     * weighting - 2 is optimal for gaussian distributions
     */
    double beta;
    /*!
     * Tertiary scaling factor, typically 0.
     * Some authors recommend parameter estimation to use L - 3
     */
    double kappa;
};
struct SigmaPointParameterDerivedQuantities {
    SigmaPointParameterDerivedQuantities(SigmaPointParameters const &p,
                                         std::size_t L)
        : alphaSquared(p.alpha * p.alpha),
          lambda(alphaSquared * (L + p.kappa) - L),
          gamma(std::sqrt(L + lambda)), weightMean0(lambda / (L + lambda)),
          weightCov0(weightMean0 + p.beta + 1 - alphaSquared),
          weight(1. / (2. * (L + lambda))) {}

  private:
    double alphaSquared;

  public:
    //! "Compound scaling factor"
    double lambda;
    //! Scales the matrix square root in computing sigma points
    double gamma;

    //! Element 0 of weight vector for computing means
    double weightMean0;
    //! Element 0 of weight vector for computing covariance
    double weightCov0;
    //! Other elements of weight vector
    double weight;
};

template <int Dim, int OrigDim = Dim>
class AugmentedSigmaPointGenerator {
  public:
    static_assert(OrigDim == Eigen::Dynamic || Dim == Eigen::Dynamic || OrigDim <= Dim,
                  "Original, non-augmented dimension must "
                  "be equal or less than the full dimension");
    static const int OriginalDimension = OrigDim;
    static const int NumSigmaPoints = Dim == Eigen::Dynamic? Eigen::Dynamic : (Dim * 2 + 1);
    using MeanVec = types::Vector<Dim>;
    using CovMatrix = types::SquareMatrix<Dim>;
    using SigmaPointsMat = types::Matrix<Dim, NumSigmaPoints>;
    using SigmaPointWeightVec = types::Vector<NumSigmaPoints>;

    AugmentedSigmaPointGenerator(MeanVec const &mean, CovMatrix const &cov,
        SigmaPointParameters params, std::size_t originalDim = OriginalDimension)
        : originalDim_(originalDim), p_(params, mean.size()), mean_(mean), covariance_(cov) {
        assert(originalDim <= mean.size() &&
            "Original, non-augmented dimension must "
            "be equal or less than the full dimension");
        weights_ = SigmaPointWeightVec::Constant(getNumSigmaPoints(), p_.weight);
        weightsForCov_ = weights_;
        weights_[0] = p_.weightMean0;
        weightsForCov_[0] = p_.weightCov0;
        scaledMatrixSqrt_ = cov.llt().matrixL();
        //! scaledMatrixSqrt_ *= p_.gamma;
        sigmaPoints_.resize(mean.size(), getNumSigmaPoints());
        sigmaPoints_ << mean, (p_.gamma * scaledMatrixSqrt_).colwise() + mean,
            (-p_.gamma * scaledMatrixSqrt_).colwise() + mean;
    }

    std::size_t getOriginalDimension() const { return originalDim_; }

    SigmaPointsMat const &getSigmaPoints() const { return sigmaPoints_; }
    std::size_t getNumSigmaPoints() const { return mean_.size() * 2 + 1; }

    using SigmaPointBlock = Eigen::Block<SigmaPointsMat, OrigDim, 1>;
    using ConstSigmaPointBlock = Eigen::Block<const SigmaPointsMat, OrigDim, 1>;

    ConstSigmaPointBlock getSigmaPoint(std::size_t i) const {
        return sigmaPoints_.template block<OrigDim, 1>(0, i, originalDim_, 1);
    }

    SigmaPointWeightVec const &getWeightsForMean() const { return weights_; }
    SigmaPointWeightVec const &getWeightsForCov() const {
        return weightsForCov_;
    }

    MeanVec const &getMean() const { return mean_; }

    using ConstOrigMeanVec = Eigen::VectorBlock<const MeanVec, OrigDim>;

    //! Get the "un-augmented" mean
    ConstOrigMeanVec getOrigMean() const {
        return mean_.template head<OrigDim>(originalDim_);
    }

  private:
    std::size_t originalDim_;
    SigmaPointParameterDerivedQuantities p_;
    MeanVec mean_;
    CovMatrix covariance_;
    CovMatrix scaledMatrixSqrt_;
    SigmaPointsMat sigmaPoints_;
    SigmaPointWeightVec weights_;
    SigmaPointWeightVec weightsForCov_;
};

template <int Dim>
using SigmaPointGenerator = AugmentedSigmaPointGenerator<Dim, Dim>;

template <int XformedDim, typename SigmaPointsGenType>
class ReconstructedDistributionFromSigmaPoints {
  public:
    static const int Dimension = XformedDim;
    using SigmaPointsGen = SigmaPointsGenType;
    static const int NumSigmaPoints = SigmaPointsGen::NumSigmaPoints;

    static const int OriginalDimension = SigmaPointsGen::OriginalDimension;
    using TransformedSigmaPointsMat = types::Matrix<XformedDim, NumSigmaPoints>;

    using CrossCovMatrix = types::Matrix<OriginalDimension, Dimension>;

    using MeanVec = types::Vector<XformedDim>;
    using CovMat = types::SquareMatrix<XformedDim>;
    ReconstructedDistributionFromSigmaPoints(
        SigmaPointsGen const &sigmaPoints,
        TransformedSigmaPointsMat const &xformedPointsMat)
        : xformedCov_(CovMat::Zero(xformedPointsMat.rows(), xformedPointsMat.rows())),
        crossCov_(CrossCovMatrix::Zero(sigmaPoints.getOriginalDimension(), xformedPointsMat.rows())) {
//! weighted average
#if 1
        xformedMean_ = MeanVec::Zero(xformedPointsMat.rows());
        for (std::size_t i = 0; i < sigmaPoints.getNumSigmaPoints(); ++i) {
            auto weight = sigmaPoints.getWeightsForMean()[i];
            xformedMean_ += weight * xformedPointsMat.col(i);
        }
#else
        xformedMean_ =
            xformedPointsMat.rowwise() * sigmaPoints.getWeightsForMean();
#endif
        TransformedSigmaPointsMat zeroMeanPoints =
            xformedPointsMat.colwise() - xformedMean_;

        for (std::size_t i = 0; i < sigmaPoints.getNumSigmaPoints(); ++i) {
            auto weight = sigmaPoints.getWeightsForCov()[i];
            xformedCov_ += weight * zeroMeanPoints.col(i) *
                           zeroMeanPoints.col(i).transpose();
            crossCov_ +=
                weight *
                (sigmaPoints.getSigmaPoint(i) - sigmaPoints.getOrigMean()) *
                zeroMeanPoints.col(i).transpose();
        }
    }

    MeanVec const &getMean() const { return xformedMean_; }
    CovMat const &getCov() const { return xformedCov_; }
    // matrix of cross-covariance between original and transformed (such as
    // state and measurement residuals)
    CrossCovMatrix const &getCrossCov() const { return crossCov_; }

  private:
    MeanVec xformedMean_;
    CovMat xformedCov_;
    CrossCovMatrix crossCov_;
};

} // namespace flexkalman
