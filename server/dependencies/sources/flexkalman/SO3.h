/** @file
    @brief Header for SO3 pose representation

    @date 2019-2020

    @author
    Rylie Pavlik
    <rylie.pavlik@collabora.com>
*/

// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0 OR Apache-2.0

#pragma once

#include "MatrixExponentialMap.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace flexkalman {

template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
rot_matrix_ln(Eigen::MatrixBase<Derived> const &mat) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    using Scalar = typename Derived::Scalar;
    Eigen::AngleAxis<Scalar> angleAxis =
        Eigen::AngleAxis<Scalar>{mat.derived()};
    return angleAxis.angle() * angleAxis.axis();
}

template <typename Derived>
static inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
rot_matrix_ln(Eigen::QuaternionBase<Derived> const &q) {
    using Scalar = typename Derived::Scalar;
    Eigen::AngleAxis<Scalar> angleAxis{q.derived()};
    return angleAxis.angle() * angleAxis.axis();
}

/*!
 * Represents an orientation as a member of the "special orthogonal group in 3D"
 * SO3.
 *
 * This means we're logically using a 3D vector that can be converted to a
 * rotation matrix using the matrix exponential map aka "Rodrigues' formula".
 * We're actually storing both the rotation matrix and the vector for simplicity
 * right now.
 */
class SO3 {
  public:
    SO3() = default;
    explicit SO3(Eigen::Vector3d const &v)
        : matrix_(matrix_exponential_map::rodrigues(
              matrix_exponential_map::singularitiesAvoided(v))) {}
    explicit SO3(Eigen::Matrix3d const &mat) : matrix_(mat) {}

    static SO3 fromQuat(Eigen::Quaterniond const &q) {
        Eigen::AngleAxisd angleAxis{q};
        Eigen::Vector3d omega = angleAxis.angle() * angleAxis.axis();
        return SO3{omega};
    }
    Eigen::Vector3d getVector() const {
        double angle = getAngle();
        while (angle < -EIGEN_PI) {
            angle += 2 * EIGEN_PI;
        }
        while (angle > EIGEN_PI) {
            angle -= 2 * EIGEN_PI;
        }
        return angle * getAxis();
    }

    Eigen::Matrix3d const &getRotationMatrix() const noexcept {
        return matrix_;
    }

    Eigen::Quaterniond getQuat() const {
        return matrix_exponential_map::toQuat(getVector());
    }

    SO3 getInverse() const { return SO3{getRotationMatrix(), InverseTag{}}; }

    double getAngle() const {
        return Eigen::AngleAxisd{getRotationMatrix()}.angle();
    }

    Eigen::Vector3d getAxis() const {
        return Eigen::AngleAxisd{getRotationMatrix()}.axis();
    }

  private:
    //! tag type to choose the inversion constructor.
    struct InverseTag {};

    //! Inversion constructor - fast
    SO3(Eigen::Matrix3d const &mat, InverseTag const & /*tag*/)
        : //  omega_(Eigen::Vector3d::Zero()),
          matrix_(mat.transpose()) {
        // omega_ = rot_matrix_ln(matrix_);
    }

    // Eigen::Vector3d omega_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d matrix_{Eigen::Matrix3d::Identity()};
};

static inline SO3 operator*(SO3 const &lhs, SO3 const &rhs) {
    Eigen::Matrix3d product = lhs.getRotationMatrix() * rhs.getRotationMatrix();
    return SO3{product};
}
} // namespace flexkalman
