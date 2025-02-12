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

// Source: https://github.com/PoseLib/PoseLib
// Modifications by Seneral:
// - merged p3p_common.h, p3p.h into files
// - switched from CameraPose to Eigen::Isometry
// - made templated and switched to arrays

#ifndef POSELIB_P3P_DING_H_
#define POSELIB_P3P_DING_H_

#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "Eigen/src/Core/util/Constants.h"

#include <vector>

namespace poselib {

// Solves for camera pose such that: lambda*x = R*X+t  with positive lambda.
// Re-implementation of the P3P solver from
//    Y. Ding, J. Yang, V. Larsson, C. Olsson, K. Åström, Revisiting the P3P Problem, CVPR 2023
// Note: this impl. assumes that x has been normalized.
template<typename Scalar = float>
int p3p_ding(const std::array<Eigen::Matrix<Scalar,3,1>,3> &x,
             const std::array<Eigen::Matrix<Scalar,3,1>,3> &X,
             std::array<Eigen::Transform<Scalar,3,Eigen::Isometry>,4> &output);
// Specialised for float and double only

}

#endif
