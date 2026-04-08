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

#include "reconstruction_internal.hpp"
#include "util/eigenalg.hpp"

//#define LOG_MAX_LEVEL LTrace
#include "util/log.hpp"
#include "util/util.hpp"

#include <algorithm>
#include <cassert>

/**
 * Tools used by main reconstruction routines
 */

double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::Matrix<double,3,4>> &V, int v,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix)
{
	assert(P.rows() == 4);
	assert(P.cols() == measurementMatrix.cols());

	int ptCnt = 0;
	double reprojectionErrorSq = 0.0;
	for (int p = 0; p < measurementMatrix.cols(); p++)
	{
		Eigen::Vector3d measure = measurementMatrix.block<3,1>(v*3, p);
		if (measure.hasNaN()) continue;
		Eigen::Vector3d reprojection = V * P.block<4,1>(0, p);
		reprojectionErrorSq += (measure.hnormalized()-reprojection.hnormalized()).squaredNorm();
		ptCnt++;
	}
	assert(ptCnt > 0);
	return reprojectionErrorSq / ptCnt;
}


double calculateReprojectionErrorSq(const Eigen::Ref<const Eigen::MatrixXd> V,
	const Eigen::Ref<const Eigen::MatrixXd> &P, const Eigen::Ref<const Eigen::MatrixXd> &measurementMatrix)
{
	assert(V.cols() == 4);
	assert(P.rows() == 4);
	assert(V.rows() == measurementMatrix.rows());
	assert(P.cols() == measurementMatrix.cols());

	// Test reprojection error on original (non-estimated) measurements
	int ptCnt = 0;
	double reprojectionErrorSq = 0;
	for (int v = 0; v < measurementMatrix.rows(); v+=3)
	{
		for (int p = 0; p < measurementMatrix.cols(); p++)
		{
			Eigen::Vector3d measure = measurementMatrix.block<3,1>(v,p);
			if (measure.hasNaN()) continue;
			Eigen::Vector3d reprojection = V.block<3,4>(v,0) * P.block<4,1>(0,p);
			reprojectionErrorSq += (measure.hnormalized()-reprojection.hnormalized()).squaredNorm();
			ptCnt++;
		}
	}
	return reprojectionErrorSq / ptCnt;
}

/**
 * Normalises the given 3xN row of points by centering and scaling to a distance of sqrt(2) and writes it into normRow, and returns the inverse of that normalisation transform
 */
Eigen::Matrix3d normalisePointRow(const Eigen::Ref<const Eigen::MatrixXd> &pointRow, Eigen::Ref<Eigen::MatrixXd> normRow)
{
	// Data normalisation, center and scale raw points
	Eigen::Vector2d imageCentroid = Eigen::Vector2d::Zero();
	double imageScale = 0;
	int imagePointCount = 0;

	// Get centroids
	for (int p = 0; p < pointRow.cols(); p++)
	{
		if (pointRow.col(p).hasNaN()) continue;
		imageCentroid += pointRow.block<2,1>(0,p) / pointRow(2,p);
		imagePointCount++;
	}
	imageCentroid /= imagePointCount;

	// Apply centroids and get scale
	for (int p = 0; p < pointRow.cols(); p++)
	{
		if (pointRow.col(p).hasNaN()) continue;
		normRow.block<2,1>(0,p) = pointRow.block<2,1>(0,p)/pointRow(2,p) - imageCentroid;
		normRow(2,p) = 1;
		imageScale += normRow.block<2,1>(0,p).norm();
	}
	imageScale /= imagePointCount*sqrt(2);
	double imageRescale = 1.0 / imageScale;

	// Apply scale
	for (int p = 0; p < pointRow.cols(); p++)
	{
		if (pointRow.col(p).hasNaN())
			normRow.col(p).setConstant(NAN);
		else
			normRow.block<2,1>(0,p) *= imageRescale;
	}

	// Build normalisation matrices to invert normalisations later
	Eigen::Matrix3d normInv = Eigen::Vector3d(imageScale,imageScale,1).asDiagonal();
	normInv.block<2,1>(0,2) = imageCentroid;
	return normInv;
}

/**
 * Balances the given (3*M)xN matrix of points so that on average each point (3x1) has the norm 1
 */
void balanceMatrix3Triplet(Eigen::Ref<Eigen::MatrixXd> matrix)
{
	int rows = matrix.rows()/3, cols = matrix.cols();

	// Perform second balancing on just the projective depths and point scales (proxy matrix)
	int maxIt = 10;
	double avgRowChange, avgColChange;
	do
	{
		maxIt--;
		avgRowChange = 0;
		for (int r = 0; r < rows; r++)
		{
			int ptCnt = 0;
			double ptSum = 0;
			for (int v = 0; v < cols; v++)
			{
				if (!matrix.block<3,1>(r*3,v).hasNaN())
				{
					ptCnt++;
					ptSum += matrix.block<3,1>(r*3,v).squaredNorm();
				}
			}
			double rowScaleSq = std::sqrt(ptCnt/ptSum);
			matrix.middleRows<3>(r*3) *= rowScaleSq;
			avgRowChange += std::abs(rowScaleSq-1);
		}
		avgRowChange /= rows;
		avgColChange = 0;
		for (int v = 0; v < cols; v++)
		{
			int ptCnt = 0;
			double ptSum = 0;
			for (int r = 0; r < rows; r++)
			{
				if (!matrix.block<3,1>(r*3,v).hasNaN())
				{
					ptCnt++;
					ptSum += matrix.block<3,1>(r*3,v).squaredNorm();
				}
			}
			double colScaleSq = std::sqrt(ptCnt/ptSum);
			matrix.col(v) *= colScaleSq;
			avgColChange += std::abs(colScaleSq-1);
		}
		avgColChange /= cols;
		LOGC(LTrace, "Balancing iteration change: row %f, col %f", avgRowChange, avgColChange);
	} while ((avgRowChange > 0.01 || avgColChange > 0.01) && maxIt > 0);
}

/**
 * Calculates the fundamental matrix from two corresponding 3xN rows of points (may have NaNs)
 */
[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
std::pair<int,double> calculateFundamentalMatrix(const Eigen::Ref<const Eigen::MatrixXd> &pointsA, const Eigen::Ref< const Eigen::MatrixXd> &pointsB, Eigen::Matrix3d &fundamentalMatrix, int minPointCount)
{
	int colCount = std::min(pointsA.cols(), pointsB.cols());
	if (colCount < minPointCount)
	{
		LOGC(LWarn, "    Cannot calculate fundamental matrix from %d points!", colCount);
		return { colCount, 0 };
	}
	int pointCount = 0;
	for (int p = 0; p < colCount; p++)
		if (!pointsA.col(p).hasNaN() && !pointsB.col(p).hasNaN())
			pointCount++;
	LOGC(LDebug, "Calculating fundamental matrix with %d/%d overlapping points!", pointCount, colCount);
	if (pointCount < minPointCount)
	{
		LOGC(LWarn, "    Cannot calculate fundamental matrix from %d/%d non-nan points!", pointCount, colCount);
		return { pointCount, 0 };
	}

	Eigen::MatrixXd normA = Eigen::MatrixXd(3, pointsA.cols());
	Eigen::MatrixXd normB = Eigen::MatrixXd(3, pointsB.cols());
	Eigen::Matrix3d TA = normalisePointRow(pointsA, normA);
	Eigen::Matrix3d TB = normalisePointRow(pointsB, normB);

	// Build data matrix
	Eigen::MatrixXd dataMatrix(pointCount, 9);
	int r = 0;
	for (int p = 0; p < colCount; p++)
	{
		if (pointsA.col(p).hasNaN() || pointsB.col(p).hasNaN()) continue;
		Eigen::Vector2d a = normA.block<2,1>(0,p);
		Eigen::Vector2d b = normB.block<2,1>(0,p);
		Eigen::Matrix<double, 1, 9> fcoeff;
		fcoeff << a.x()*b.x(), a.y()*b.x(), b.x(), a.x()*b.y(), a.y()*b.y(), b.y(), a.x(), a.y(), 1.0;
		dataMatrix.row(r++) = fcoeff;
	} 

	double confidence = solveFundamentalMatrix(dataMatrix, fundamentalMatrix);

	// Revert normalisations
	fundamentalMatrix = TB.transpose() * fundamentalMatrix * TA;

	// Frobenius Norm
	//fundamentalMatrix->normalise();
	// Alternative Norm
	/*Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd_rank(fundamentalMatrix);
	Eigen::DiagonalMatrix<double, 3> truncatedSV(1, svd_rank.singularValues().y()/svd_rank.singularValues().x(), 0);
	fundamentalMatrix = svd_rank.matrixU() * truncatedSV * svd_rank.matrixV().transpose();*/

	// Verify error
	/*double error = 0;
	for (int p = 0; p < colCount; p++)
	{
		if (!pointsA.col(p).hasNaN() && !pointsB.col(p).hasNaN())
			error += std::abs((pointsB.template block<3,1>(0,p)/pointsB(2,p)).transpose() * fundamentalMatrix * (pointsA.template block<3,1>(0,p)/pointsA(2,p)));
	}
	error /= pointCount;
	LOGC(LDebug, "    Fundamental matrix leaves %f average point error and has %f confidence", error, confidence);*/

	return { pointCount, confidence };
}