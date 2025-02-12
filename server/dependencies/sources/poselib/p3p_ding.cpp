// Copyright (c) 2020, Viktor Larsson
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of the copyright holder nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Author: Mark Shachkov (mark.shachkov@gmail.com)

// Source: https://github.com/PoseLib/PoseLib
// Modifications by Seneral:
// - merged p3p_common.h, p3p.h into files
// - switched from CameraPose to Eigen::Isometry
// - made templated and switched to arrays

#include "p3p_ding.hpp"
#include "Eigen/src/Geometry/Transform.h"

#include <cmath>

namespace poselib {

// Performs a few newton steps on the equations
template<typename Scalar>
static inline void refine_lambda(
    Scalar &lambda1, Scalar &lambda2, Scalar &lambda3,
    const Scalar a12, const Scalar a13, const Scalar a23,
    const Scalar b12, const Scalar b13, const Scalar b23)
{

    for (int iter = 0; iter < 5; ++iter) {
        Scalar r1 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda2 * b12 + lambda2 * lambda2 - a12);
        Scalar r2 = (lambda1 * lambda1 - 2.0 * lambda1 * lambda3 * b13 + lambda3 * lambda3 - a13);
        Scalar r3 = (lambda2 * lambda2 - 2.0 * lambda2 * lambda3 * b23 + lambda3 * lambda3 - a23);
        if (std::abs(r1) + std::abs(r2) + std::abs(r3) < 1e-10)
            return;
        Scalar x11 = lambda1 - lambda2 * b12;
        Scalar x12 = lambda2 - lambda1 * b12;
        Scalar x21 = lambda1 - lambda3 * b13;
        Scalar x23 = lambda3 - lambda1 * b13;
        Scalar x32 = lambda2 - lambda3 * b23;
        Scalar x33 = lambda3 - lambda2 * b23;
        Scalar detJ = 0.5 / (x11 * x23 * x32 + x12 * x21 * x33); // half minus inverse determinant
        // This uses the closed form of the inverse for the jacobean.
        // Due to the zero elements this actually becomes quite nice.
        lambda1 += (-x23 * x32 * r1 - x12 * x33 * r2 + x12 * x23 * r3) * detJ;
        lambda2 += (-x21 * x33 * r1 + x11 * x33 * r2 - x11 * x23 * r3) * detJ;
        lambda3 += (x21 * x32 * r1 - x11 * x32 * r2 - x12 * x21 * r3) * detJ;
    }
}

template<typename Scalar>
Scalar cubic_trigonometric_solution(const Scalar alpha, const Scalar beta, const Scalar k2)
{
    const Scalar H = std::sqrt(-alpha * alpha * alpha / 27.0);
    const Scalar I = std::sqrt(-alpha / 3.0);
    const Scalar J = std::acos(-beta / (2.0 * H));
    const Scalar K = std::cos(J / 3.0);
    return 2.0 * I * K - k2 / 3.0;
}

template<typename Scalar>
Scalar cubic_cardano_solution(const Scalar beta, const Scalar G, const Scalar k2)
{
    const Scalar M = std::cbrt(-0.5 * beta + std::sqrt(G));
    const Scalar N = -std::cbrt(0.5 * beta + std::sqrt(G));
    return M + N - k2 / 3.0;
}

template<typename Scalar>
std::array<Eigen::Matrix<Scalar,3,1>, 2> compute_pq(
    const Scalar s, const Scalar a, const Scalar b,
    const Scalar m12, const Scalar m13, const Scalar m23)
{
    std::array<Eigen::Matrix<Scalar,3,1>, 2> pq;
    Eigen::Matrix<Scalar,3,3> C, C_adj;

    C(0, 0) = -a + s * (1 - b);
    C(0, 1) = -m13 * s;
    C(0, 2) = a * m23 + b * m23 * s;
    C(1, 0) = -m13 * s;
    C(1, 1) = s + 1;
    C(1, 2) = -m12;
    C(2, 0) = a * m23 + b * m23 * s;
    C(2, 1) = -m12;
    C(2, 2) = -a - b * s + 1;

    C_adj(0, 0) = C(1, 2) * C(2, 1) - C(1, 1) * C(2, 2);
    C_adj(1, 1) = C(0, 2) * C(2, 0) - C(0, 0) * C(2, 2);
    C_adj(2, 2) = C(0, 1) * C(1, 0) - C(0, 0) * C(1, 1);
    C_adj(0, 1) = C(0, 1) * C(2, 2) - C(0, 2) * C(2, 1);
    C_adj(0, 2) = C(0, 2) * C(1, 1) - C(0, 1) * C(1, 2);
    C_adj(1, 0) = C(1, 0) * C(2, 2) - C(1, 2) * C(2, 0);
    C_adj(1, 2) = C(0, 0) * C(1, 2) - C(0, 2) * C(1, 0);
    C_adj(2, 0) = C(1, 1) * C(2, 0) - C(1, 0) * C(2, 1);
    C_adj(2, 1) = C(0, 0) * C(2, 1) - C(0, 1) * C(2, 0);

    Eigen::Matrix<Scalar,3,1> v;
    if (C_adj(0, 0) > C_adj(1, 1)) {
        if (C_adj(0, 0) > C_adj(2, 2)) {
            v = C_adj.col(0) / std::sqrt(C_adj(0, 0));
        } else {
            v = C_adj.col(2) / std::sqrt(C_adj(2, 2));
        }
    } else if (C_adj(1, 1) > C_adj(2, 2)) {
        v = C_adj.col(1) / std::sqrt(C_adj(1, 1));
    } else {
        v = C_adj.col(2) / std::sqrt(C_adj(2, 2));
    }

    Eigen::Matrix<Scalar,3,3> D = C;
    D(0, 1) -= v(2);
    D(0, 2) += v(1);
    D(1, 2) -= v(0);
    D(1, 0) += v(2);
    D(2, 0) -= v(1);
    D(2, 1) += v(0);

    pq[0] = D.col(0);
    pq[1] = D.row(0);

    return pq;
}

template<typename Scalar>
std::pair<int, std::array<Scalar, 2>> compute_line_conic_intersection(
    Eigen::Matrix<Scalar,3,1> &l, const Scalar b, const Scalar m13, const Scalar m23)
{
    std::pair<int, std::array<Scalar, 2>> result;
    const Scalar cxa = -b * l(1) * l(1) + l(2) * l(2);
    const Scalar cxb = -2 * b * m23 * l(1) * l(2) - 2 * b * l(0) * l(1) - 2 * m13 * l(2) * l(2);
    const Scalar cxc = -2 * b * m23 * l(0) * l(2) - b * l(0) * l(0) - b * l(2) * l(2) + l(2) * l(2);
    const Scalar d = cxb * cxb - 4 * cxa * cxc;

    if (d < 0) {
        result.first = 0;
        return result;
    }

    result.second[0] = (-cxb + std::sqrt(d)) / (2.0 * cxa);
    result.second[1] = (-cxb - std::sqrt(d)) / (2.0 * cxa);
    result.first = d > 0 ? 2 : 1;
    return result;
}

template<typename Scalar>
Eigen::Transform<Scalar,3,Eigen::Isometry> compute_pose(
    const std::array<Eigen::Matrix<Scalar,3,1>,3> &x, const std::array<Eigen::Matrix<Scalar,3,1>,3> &X, 
    const Scalar a12, const Scalar a13, const Scalar a23,
    const Scalar m12, const Scalar m13, const Scalar m23,
    const Scalar x_root, const Scalar y_root)
{
    Scalar d3 = std::sqrt(a13) / std::sqrt(x_root * x_root - 2 * m13 * x_root + 1);
    Scalar d2 = y_root * d3;
    Scalar d1 = x_root * d3;

    refine_lambda(d1, d2, d3, a12, a13, a23, m12, m13, m23);

    Eigen::Matrix<Scalar,3,3> A, B;
    A.col(0) = X[0] - X[1];
    A.col(1) = X[2] - X[0];
    A.col(2) = (X[0] - X[1]).cross(X[2] - X[0]);
    B.col(0) = d1 * x[0] - d2 * x[1];
    B.col(1) = d3 * x[2] - d1 * x[0];
    B.col(2) = B.col(0).cross(B.col(1));

    Eigen::Transform<Scalar,3,Eigen::Isometry> pose;
    pose.linear() = B * A.inverse();
    pose.translation() = d1 * x[0] - pose.rotation() * X[0];
    return pose;
}

template<typename Scalar>
int p3p_ding(const std::array<Eigen::Matrix<Scalar,3,1>,3> &x,
             const std::array<Eigen::Matrix<Scalar,3,1>,3> &X,
             std::array<Eigen::Transform<Scalar,3,Eigen::Isometry>,4> &output)
{
    const Scalar a12 = (X[0] - X[1]).squaredNorm();
    const Scalar a13 = (X[0] - X[2]).squaredNorm();
    const Scalar a23 = (X[1] - X[2]).squaredNorm();
    const Scalar a = a12 / a23;
    const Scalar b = a13 / a23;

    const Scalar m12 = x[0].dot(x[1]);
    const Scalar m13 = x[0].dot(x[2]);
    const Scalar m23 = x[1].dot(x[2]);

    const Scalar k3_inv = 1.0 / (b * (-b * m23 * m23 + b + m13 * m13 - 1));
    const Scalar k2 = k3_inv * (-2 * a * b * m23 * m23 + 2 * a * b + a * m13 * m13 - a - b * b * m23 * m23 + b * b +
                                2 * b * m12 * m13 * m23 - 2 * b - m13 * m13 + 1);
    const Scalar k1 = k3_inv * (-a * a * m23 * m23 + a * a - 2 * a * b * m23 * m23 + 2 * a * b +
                                2 * a * m12 * m13 * m23 - 2 * a + b * m12 * m12 - b - m12 * m12 + 1);
    const Scalar k0 = k3_inv * (a * (-a * m23 * m23 + a + m12 * m12 - 1));
    const Scalar alpha = k1 - 1.0 / 3.0 * k2 * k2;
    const Scalar beta = k0 - 1.0 / 3.0 * k1 * k2 + (2.0 / 27.0) * k2 * k2 * k2;
    const Scalar G = beta * beta / 4.0 + alpha * alpha * alpha / 27.0;

    Scalar s;
    if (G != 0) {
        if (G < 0) {
            s = cubic_trigonometric_solution(alpha, beta, k2);
        } else {
            s = cubic_cardano_solution(beta, G, k2);
        }
    } else {
        s = -k2 / 3.0 + (alpha != 0 ? (3.0 * beta / alpha) : 0);
    }

    std::array<Eigen::Matrix<Scalar,3,1>, 2> pq = compute_pq(s, a, b, m12, m13, m23);

    int sol = 0;
    for (int i = 0; i < 2; i++) {
        auto [n_roots, x_roots] = compute_line_conic_intersection(pq[i], b, m13, m23);
        for (int j = 0; j < n_roots; j++) {
            const Scalar x_root = x_roots[j];
            const Scalar y_root = (-pq[i](0) - pq[i](1) * x_root) / pq[i](2);
            if (x_root <= 0 || y_root <= 0) {
                continue;
            }
            output[sol++] = compute_pose(x, X, a12, a13, a23, m12, m13, m23, x_root, y_root);
        }

        if ((G == 0) | (G < 0 ? n_roots : !n_roots)) {
            continue;
        }

        break;
    }

    return sol;
}

// Specialisation
template int p3p_ding(const std::array<Eigen::Vector3f,3> &x,
             const std::array<Eigen::Vector3f,3> &X,
             std::array<Eigen::Isometry3f,4> &output);
template int p3p_ding(const std::array<Eigen::Vector3d,3> &x,
             const std::array<Eigen::Vector3d,3> &X,
             std::array<Eigen::Isometry3d,4> &output);

} // namespace poselib
