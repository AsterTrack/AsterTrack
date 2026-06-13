/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef ROTATION_GEN_PARAMS_H
#define ROTATION_GEN_PARAMS_H

#include "util/eigendef.hpp"

struct RotationGenerationParameters
{
	int shellPoints = 100;
	int rollAxisShells = 10;

	// Right parameter is increasingly sensitive with point counts > 1000, so precision here is required
	std::vector<Eigen::Vector2f> shells = {
		Eigen::Vector2f(0.0f, 0.0f),
		Eigen::Vector2f(2.5f, 0.0728f),
		Eigen::Vector2f(0.5f, 0.5f),
		Eigen::Vector2f(1.25f, 0.191f)
	};

	float spreadFloor = 0.5f, spreadCeil = 0.1f;
	float spreadVariance = 1.0f;
};

#endif

#ifdef INCLUDE_ROTATION_GEN_PARAMS_ONLY
#undef INCLUDE_ROTATION_GEN_PARAMS_ONLY
#else

#ifndef ROTATION_GEN_H
#define ROTATION_GEN_H

#include <cassert>

/**
 * Parameters for tracking algorithms
 */

static const float PHI = PI * (std::sqrt(5.0f) - 1.0f); // golden angle in radians

static inline float generateZ(const RotationGenerationParameters &gen, int shell, int point)
{
    float low = std::pow(gen.spreadFloor, gen.spreadVariance);
    float high = std::pow(gen.spreadCeil, gen.spreadVariance);
    float z = (point+gen.shells[shell].x()) / (gen.shellPoints);
    z = std::fmod(z, 1.0f);
    z = std::pow(z * (gen.spreadCeil-gen.spreadFloor) + gen.spreadFloor, gen.spreadVariance);
    z = (z - low) / (high-low);
    return std::clamp(1 - z * 2, -1.0f, 1.0f);
}

static inline Eigen::Vector3f generateSpherePoint(const RotationGenerationParameters &gen, int shell, int point)
{
    float z = generateZ(gen, shell, point); // z coords - derivative of heading angle
    float plane = std::sqrt(1 - z * z); // radius on xy-plane
    float theta = PHI * (point + gen.shells[shell].y()); // golden angle increment
    float x = std::cos(theta) * plane;
    float y = std::sin(theta) * plane;
    return Eigen::Vector3f(x, y, z);
};

static inline Eigen::Quaternionf generateRotation(const RotationGenerationParameters &gen, int shell, int point, float rollAngle)
{
    Eigen::Vector3f spherePoint = generateSpherePoint(gen, shell, point);
    Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0,0,1), spherePoint);
    rotation = rotation * Eigen::Quaternionf(Eigen::AngleAxisf(rollAngle, Eigen::Vector3f::UnitZ()));
    assert(!rotation.coeffs().hasNaN());
    return rotation;
};

#endif // ROTATION_GEN_H
#endif // INCLUDE_ROTATION_GEN_PARAMS_ONLY