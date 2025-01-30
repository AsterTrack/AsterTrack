/** @file
    @brief Header for measurements of absolute position with an offset.

    @date 2019-2020

    @author
    Rylie Pavlik
    <rylie.pavlik@collabora.com>
*/
// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0 OR Apache-2.0

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "flexkalman/BaseTypes.h"

namespace flexkalman {
/*!
 * For PS Move-like things, where there's a directly-computed absolute position
 * that is not at the tracked body's origin.
 */
class AbsolutePositionLeverArmMeasurement
    : public MeasurementBase<AbsolutePositionLeverArmMeasurement> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t Dimension = 3;
    using MeasurementVector = types::Vector<Dimension>;
    using MeasurementSquareMatrix = types::SquareMatrix<Dimension>;

    /*!
     * @todo the point we get from the camera isn't the center of the ball,
     * but the center of the visible surface of the ball - a closer
     * approximation would be translation along the vector to the center of
     * projection....
     */
    AbsolutePositionLeverArmMeasurement(
        MeasurementVector const &measurement,
        MeasurementVector const &knownLocationInBodySpace,
        MeasurementVector const &variance)
        : measurement_(measurement),
          knownLocationInBodySpace_(knownLocationInBodySpace),
          covariance_(variance.asDiagonal()) {}

    template <typename State>
    MeasurementSquareMatrix const &getCovariance(State const & /*s*/) {
        return covariance_;
    }

    template <typename State>
    types::Vector<3> predictMeasurement(State const &s) const {
        return s.getIsometry() * knownLocationInBodySpace_;
    }

    template <typename State>
    MeasurementVector getResidual(MeasurementVector const &predictedMeasurement,
                                  State const & /*s*/) const {
        return measurement_ - predictedMeasurement;
    }

    template <typename State>
    MeasurementVector getResidual(State const &s) const {
        return getResidual(predictMeasurement(s), s);
    }

  private:
    MeasurementVector measurement_;
    MeasurementVector knownLocationInBodySpace_;
    MeasurementSquareMatrix covariance_;
};
} // namespace flexkalman
