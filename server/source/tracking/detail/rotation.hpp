/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef TARGET_KALMAN_H
#define TARGET_KALMAN_H

#include "util/eigendef.hpp"

// TODO: Fix jacobians for rotation transformations (1/2)
// None of them are verified to work yet
// Useful for visualising covariances in euler angles
// Useful to specify measurement variances by axis, e.g. higher variance for Z axis

static inline Eigen::Matrix<float,3,4> jacobianQuat2Euler(Eigen::Quaternionf quat)
{
	// From https://www.ucalgary.ca/engo_webdocs/GL/96.20096.JSchleppe.pdf
	// formula 4.80 - 4.84, pp. 69-71
	float x = quat.x(), y = quat.y(), z = quat.z(), w = quat.w();
	float yzP = z + y, yzM = z - y;
	float xwP = w + x, xwM = w - x;
	float sqP = yzP*yzP + xwP*xwP, sqM = yzM*yzM + xwM*xwM;
	yzP /= sqP;
	xwP /= sqP;
	yzM /= sqM;
	xwM /= sqM;
	float sq = y*z + x*w;
	float factor = 2 / std::sqrt(1 - 4*sq);
	Eigen::Matrix<float,3,4> jac;
	jac(0, 0) = -yzP +yzM;
	jac(0, 1) = +xwP -xwM;
	jac(0, 2) = +xwP +xwM;
	jac(0, 3) = -yzP -yzM;
	jac(1, 0) = w * factor;
	jac(1, 1) = z * factor;
	jac(1, 2) = y * factor;
	jac(1, 3) = x * factor;
	jac(2, 0) = -yzP -yzM;
	jac(2, 1) = +xwP +xwM;
	jac(2, 2) = +xwP -xwM;
	jac(2, 3) = -yzP +yzM;
	return jac;
}

static inline Eigen::Matrix<float,3,4> jacobianLogMap(Eigen::Quaternionf quat)
{
	Eigen::Matrix<float,3,4> jac = Eigen::Matrix<float,3,4>::Identity();
	float vecnorm = quat.vec().norm();
	if (vecnorm > 1e-4)
	{
		float phi = std::atan2(vecnorm, quat.w());
		jac.diagonal() = Eigen::Vector3f::Constant(1/std::sin(phi)) - quat.vec()*2/std::sin(2*phi);
	}
	return jac;
}

static inline Eigen::Matrix<float,3,4> jacobianLogMap2(Eigen::Quaternionf quat)
{
	Eigen::Matrix<float,3,4> jac = Eigen::Matrix<float,3,4>::Identity();
	float vecnorm = quat.vec().norm();
	if (vecnorm > 1e-4)
	{
		float n = quat.w() / vecnorm;
		float f1 = std::sqrt(1 + n*n);
		float f2 = std::sqrt(4 + n*n);
		jac.diagonal() = Eigen::Vector3f::Constant(f1) - quat.vec()*f2;
	}
	return jac;
}

static Eigen::Matrix<float,3,3> jacobianEuler2EXP(Eigen::Quaternionf quat)
{
	return jacobianLogMap(quat) * jacobianQuat2Euler(quat).transpose();
}

static Eigen::Matrix<float,3,3> jacobianEXP2Euler(Eigen::Quaternionf quat)
{
	return jacobianQuat2Euler(quat) * jacobianLogMap(quat).transpose();
}

#endif