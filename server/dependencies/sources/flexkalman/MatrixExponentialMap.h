/** @file
    @brief Header

    @date 2015, 2019

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
// - none

// Library/third-party includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard includes
// - none

namespace flexkalman {

/*!
 * Produces the "hat matrix" that produces the same result as
 * performing a cross-product with v. This is the same as the "capital
 * omega" skew-symmetrix matrix used by a matrix-exponential-map
 * rotation vector.
 * @param v a 3D vector
 * @return a matrix M such that for some 3D vector u, Mu = v x u.
 */
template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
makeSkewSymmetrixCrossProductMatrix(Eigen::MatrixBase<Derived> const &v) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ret;
    ret << 0, -v.z(), v.y(), //
        v.z(), 0, -v.x(),    //
        -v.y(), v.x(), 0;
    return ret;
}

/*!
 * Utilities for interacting with a "matrix exponential map vector"
 * rotation parameterization/formalism, where rotation is represented as a
 * 3D vector that is turned into a rotation matrix by applying Rodrigues'
 * formula that resembles a matrix exponential.
 *
 * Based on discussion in section 2.2.3 of:
 *
 * Lepetit, V., & Fua, P. (2005). Monocular Model-Based 3D Tracking of
 * Rigid Objects. Foundations and Trends® in Computer Graphics and Vision,
 * 1(1), 1–89. http://doi.org/10.1561/0600000001
 *
 * Not to be confused with the quaternion-related exponential map espoused
 * in:
 *
 * Grassia, F. S. (1998). Practical Parameterization of Rotations Using the
 * Exponential Map. Journal of Graphics Tools, 3(3), 29–48.
 * http://doi.org/10.1080/10867651.1998.10487493
 */
namespace matrix_exponential_map {
    /*!
     * Adjust a matrix exponential map rotation vector, if required, to
     * avoid  singularities.
     * @param omega a 3D "matrix exponential map" rotation vector, which
     * will be modified if required.
     */
    template <typename Derived>
    inline void avoidSingularities(Eigen::MatrixBase<Derived> &omega) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        // if magnitude gets too close to 2pi, in this case, pi...
        if (omega.squaredNorm() > EIGEN_PI * EIGEN_PI) {
            // replace omega with an equivalent one.
            omega.derived() =
                ((1 - (2 * EIGEN_PI) / omega.norm()) * omega).eval();
        }
    }
    /*!
     * Return a matrix exponential map rotation vector, modified if required
     * to avoid singularities.
     *
     * @param omega a 3D "matrix exponential map" rotation vector.
     *
     * This call returns the result instead of modifying in place.
     *
     * @see avoidSingularities()
     */
    template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
    singularitiesAvoided(Eigen::MatrixBase<Derived> const &omega) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        // if magnitude gets too close to 2pi, in this case, pi...
        if (omega.squaredNorm() > EIGEN_PI * EIGEN_PI) {
            // replace omega with an equivalent one.
            return (1 - (2 * EIGEN_PI) / omega.norm()) * omega;
        }
        return omega;
    }

    /*!
     * Gets the rotation angle of a rotation vector.
     * @param omega a 3D "exponential map" rotation vector
     */
    template <typename Derived>
    inline typename Derived::Scalar
    getAngle(Eigen::MatrixBase<Derived> const &omega) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return omega.norm();
    }

    /*!
     * Gets the unit quaternion corresponding to the exponential rotation
     * vector.
     * @param omega a 3D "exponential map" rotation vector
     */
    template <typename Derived>
    inline Eigen::Quaterniond getQuat(Eigen::MatrixBase<Derived> const &omega) {
        auto theta = getAngle(omega);
        auto xyz = omega * std::sin(theta / 2.);
        return Eigen::Quaterniond(std::cos(theta / 2.), xyz.x(), xyz.y(),
                                  xyz.z());
    }

    //! Contained cached computed values
    class ExponentialMapData {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*!
         * Construct from a matrixy-thing: should be a 3d vector containing
         * a matrix-exponential-map rotation formalism.
         */
        template <typename Derived>
        explicit ExponentialMapData(Eigen::MatrixBase<Derived> const &omega)
            : m_omega(omega) {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        }

        ExponentialMapData() : m_omega(Eigen::Vector3d::Zero()) {}

        //! assignment operator - its presence is an optimization only.
        ExponentialMapData &operator=(ExponentialMapData const &other) {
            if (&other != this) {
                m_omega = other.m_omega;
                m_gotTheta = other.m_gotTheta;
                if (m_gotTheta) {
                    m_theta = other.m_theta;
                }
                m_gotBigOmega = other.m_gotBigOmega;
                if (m_gotBigOmega) {
                    m_bigOmega = other.m_bigOmega;
                }
                m_gotR = other.m_gotR;
                if (m_gotR) {
                    m_R = other.m_R;
                }
                m_gotQuat = other.m_gotQuat;
                if (m_gotQuat) {
                    m_quat = other.m_quat;
                }
            }
            return *this;
        }

        //! move-assignment operator - its presence is an optimization only.
        ExponentialMapData &operator=(ExponentialMapData &&other) {
            if (&other != this) {
                m_omega = std::move(other.m_omega);
                m_gotTheta = std::move(other.m_gotTheta);
                if (m_gotTheta) {
                    m_theta = std::move(other.m_theta);
                }
                m_gotBigOmega = std::move(other.m_gotBigOmega);
                if (m_gotBigOmega) {
                    m_bigOmega = std::move(other.m_bigOmega);
                }
                m_gotR = std::move(other.m_gotR);
                if (m_gotR) {
                    m_R = std::move(other.m_R);
                }
                m_gotQuat = std::move(other.m_gotQuat);
                if (m_gotQuat) {
                    m_quat = std::move(other.m_quat);
                }
            }
            return *this;
        }

        template <typename Derived>
        void reset(Eigen::MatrixBase<Derived> const &omega) {
            //! Using assignment operator to be sure I didn't miss a flag.
            *this = ExponentialMapData(omega);
        }

        /*!
         * Gets the "capital omega" skew-symmetrix matrix.
         *
         * (computation is cached)
         */
        Eigen::Matrix3d const &getBigOmega() {
            if (!m_gotBigOmega) {
                m_gotBigOmega = true;
                m_bigOmega = makeSkewSymmetrixCrossProductMatrix(m_omega);
            }
            return m_bigOmega;
        }

        /*!
         * Gets the rotation angle of a rotation vector.
         *
         * (computation is cached)
         */
        double getTheta() {
            if (!m_gotTheta) {
                m_gotTheta = true;
                m_theta = getAngle(m_omega);
            }
            return m_theta;
        }

        /*!
         * Converts a rotation vector to a rotation matrix:
         * Uses Rodrigues' formula, and the first two terms of the Taylor
         * expansions of the trig functions (so as to be nonsingular as the
         * angle goes to zero).
         *
         * (computation is cached)
         */
        Eigen::Matrix3d const &getRotationMatrix() {
            if (!m_gotR) {
                m_gotR = true;
                auto theta = getTheta();
                auto &Omega = getBigOmega();
                //! two-term taylor approx of sin(theta)/theta
                double k1 = 1. - theta * theta / 6.;

                //! two-term taylor approx of (1-cos(theta))/theta
                double k2 = theta / 2. - theta * theta * theta / 24.;

                m_R = Eigen::Matrix3d::Identity() + k1 * Omega +
                      k2 * Omega * Omega;
            }
            return m_R;
        }

        Eigen::Quaterniond const &getQuaternion() {
            if (!m_gotQuat) {
                m_gotQuat = true;
                auto theta = getTheta();
                auto xyz = m_omega * std::sin(theta / 2.);
                m_quat = Eigen::Quaterniond(std::cos(theta / 2.), xyz.x(),
                                            xyz.y(), xyz.z());
            }
            return m_quat;
        }

      private:
        Eigen::Vector3d m_omega;
        bool m_gotTheta = false;
        double m_theta;
        bool m_gotBigOmega = false;
        Eigen::Matrix3d m_bigOmega;
        bool m_gotR = false;
        Eigen::Matrix3d m_R;
        bool m_gotQuat = false;
        Eigen::Quaterniond m_quat;
    };

    /*!
     * Converts a rotation vector to a rotation matrix:
     * Uses Rodrigues' formula, and the first two terms of the Taylor
     * expansions of the trig functions (so as to be nonsingular as the
     * angle goes to zero).
     */
    template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3>
    rodrigues(Eigen::MatrixBase<Derived> const &v) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        using Scalar = typename Derived::Scalar;
        Scalar theta = v.norm();
        Eigen::Matrix<Scalar, 3, 3> Omega = makeSkewSymmetrixCrossProductMatrix(
            v); //! two-term taylor approx of sin(theta)/theta
        Scalar k1 = Scalar(1) - theta * theta / Scalar(6);

        //! two-term taylor approx of (1-cos(theta))/theta
        Scalar k2 = theta / Scalar(2) - theta * theta * theta / Scalar(24);

        return Eigen::Matrix<Scalar, 3, 3>::Identity() + k1 * Omega +
               k2 * Omega * Omega;
    }

    /*!
     * Convert a matrix exponential map to a quat.
     */
    template <typename Derived>
    inline Eigen::Quaternion<typename Derived::Scalar>
    toQuat(Eigen::MatrixBase<Derived> const &v) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        using Scalar = typename Derived::Scalar;
        double theta = v.norm();
        Eigen::Vector3d xyz = v * std::sin(theta / 2.);
        return Eigen::Quaterniond(std::cos(theta / 2.), xyz.x(), xyz.y(),
                                  xyz.z());
    }
} // namespace matrix_exponential_map

} // namespace flexkalman
