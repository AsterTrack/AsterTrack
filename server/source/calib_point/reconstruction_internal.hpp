/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

/**
 * NOTE:
 * This file is licensed under the MPL despite currently being deeply integrated into the project.
 * The intent is that if you need it badly enough, it is easy to extract it to use in your own project.
 * This would require removing reference to, among others, obs_data.hpp, but it is very much doable.
 */

#ifndef RECONSTRUCTION_POINT_INTERNAL_H
#define RECONSTRUCTION_POINT_INTERNAL_H

#include "calib_point/parameters.hpp"

#include "util/eigendef.hpp"
#include "util/error.hpp"

typedef uint_fast8_t BOOL;


/**
 * Recovery of measurement matrix by estimating projective depths and filling missing data
 */

MatrixX<BOOL> estimateProjectiveDepths(
	Eigen::MatrixXd &projectiveDepthMatrix,
	const Eigen::MatrixXd &originalMeasurementMatrix,
	const Eigen::MatrixXd &measurementMatrix,
	const PointReconstructionParameters &params);

float determineRank4Basis(
	Eigen::MatrixXd &basis,
	const Eigen::MatrixXd &projectiveMatrix,
	const MatrixX<BOOL> &projectiveDepthMissing,
	const MatrixX<BOOL> &observationDataMissing,
	const PointReconstructionParameters &params,
	std::stop_token stopToken);

std::pair<int, int> recoverPointData(
	Eigen::MatrixXd &projectiveMatrix,
	MatrixX<BOOL> &projectiveDepthMissing,
	const Eigen::MatrixXd &basis, Eigen::MatrixXd &P_approx);


/**
 * Stratification of decomposed perspective view and point matrices
 */

Eigen::Matrix4d getStratificationMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P);

Eigen::Matrix4d getCorrectionMatrix(const Eigen::Ref<const Eigen::MatrixXd> &V, const Eigen::Ref<const Eigen::MatrixXd> &P);

std::optional<ErrorMessage> SeparatePerspectiveProjection(Eigen::Matrix<double,3,4> V_v, int v, const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix, CameraCalib &cameraCalib);


/**
 * Tools used by main reconstruction routines
 */

double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::Matrix<double,3,4>> &V, int v,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix);

double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::MatrixXd> V,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix);

/**
 * Normalises the given 3xN row of points by centering and scaling to a distance of sqrt(2), and returns the inverse of that normalisation transform
 */
Eigen::Matrix3d normalisePointRow(const Eigen::Ref<const Eigen::MatrixXd> &pointRow, Eigen::Ref<Eigen::MatrixXd> normRow);

/**
 * Balances the given (3*M)xN matrix of points so that on average each point (3x1) has the norm 1
 */
void balanceMatrix3Triplet(Eigen::Ref<Eigen::MatrixXd> matrix);

/**
 * Calculates the fundamental matrix from two corresponding 3xN rows of points (may have NaNs)
 */
std::pair<int,double> calculateFundamentalMatrix(const Eigen::Ref<const Eigen::MatrixXd> &pointsA, const Eigen::Ref<const Eigen::MatrixXd> &pointsB, Eigen::Matrix3d &fundamentalMatrix, int minPointCount = 8);

#endif // RECONSTRUCTION_POINT_INTERNAL_H