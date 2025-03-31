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

#ifndef TARGET_PARAMETERS_H
#define TARGET_PARAMETERS_H

#define INCLUDE_MATCHING_PARAMS_ONLY
#include "util/matching.hpp"

#include "util/eigendef.hpp"

/**
 * Parameters for target algorithms
 */

struct TargetMatchingParametersFast
{
	float matchRadius = 10.0f*PixelSize;
	MatchingParameters match = { 0.1f*PixelSize, 2, 2, 2, 1 };
};

struct TargetMatchingParametersSlow
{
	// Factors for final match filter value
	float differenceFactor = 0.1f;
	float mismatchFactor = 2.0f;
	float maxValue = 2.0f;

	// Matches to consider
	int maxCandidates = 10;
	float matchRadius = 20.0f*PixelSize;

	// Calculation of difference in position
	float differenceRadius = 15.0f*PixelSize;
	int differenceFalloff = 2;

	// Influence of neighbouring markers on mismatch
	float influenceRadius = 70.0f*PixelSize;
	int influenceFalloff = 2;
	// Evaluating similarity of neighbouring matches for mismatch
	float similarityRadius = 7.0f*PixelSize;
	int similarityFalloff = 2;
	// Mismatch is roughly 1 / (similarity*influence)
	float mismatchPower = 2.5f;

	MatchingParameters match = { 0.05f, 2, 2, 2, 1 };
};

struct TargetMatchingParametersUncertain
{
	// Accounting for excessive uncertainty after initial match
	float maxDominantFactor = 2.0f;
	float perpDeviation = 1.2f;
	float stepLength = 3.0f*PixelSize;
	int maxSteps = 20;
	TargetMatchingParametersFast subMatch = {};
};
struct TargetOptimisationParameters
{
	int maxIterations = 10;
	float tolerances = 1.0f;
	float outlierSigma = 2.3f;
	float outlierVarMin = 0.5f*PixelSize;
	float predictionInfluence = 0.000005f;
};

struct TargetFilteringParameters
{
	float stdDevPos = 0.00001f, stdDevEXP = 0.0005f;
	float stdDevAccel = 0.00001f, stdDevIMU = 0.0005f;
	float sigmaInitState = 5, sigmaInitChange = 1000;
	float sigmaAlpha = 0.001f, sigmaBeta = 2.0f, sigmaKappa = 0.0f;
	float dampeningPos = 0.95f, dampeningRot = 0.9f;

	template<typename Scalar>
	Eigen::Matrix<Scalar,6,6> getCovariance() const
	{
		Eigen::Matrix<Scalar,6,6> covariance = Eigen::Matrix<Scalar,6,6>::Identity();
		covariance.diagonal().template head<3>().setConstant(stdDevPos*stdDevPos);
		covariance.diagonal().template tail<3>().setConstant(stdDevEXP*stdDevEXP);
		return covariance;
	}
};

struct TargetDetectionParameters
{
	// Detection algorithms (2D brute force, or quick based on 3D triangulations)
	bool enable2DSync = false, enable2DAsync = true, enable3D = true;
	struct
	{
		float errorMax = 5.0f*PixelSize;
		float errorSigma = 3;
		int maxCandidates = 10;
	} search;
	struct
	{
		bool quickAssignTargetMatches = false;
		float sigmaError = 2.0f;
		float poseSigmaError = 4.0f;
		int minPointCount = 5;
		float maxErrorRMSE = 5;
	} tri;
	// Initial marker matching (before trackTarget2D)
	float expandMarkerFoV = 0.0f;
	float normaliseDistance = 5.0f;
	TargetMatchingParametersSlow match = {};
	TargetOptimisationParameters opt = {};
	// Quality
	struct {
		int focus = 6;
		int sec = 3;
		int total = 10;
	} minObservations;
	// For trackTarget2D
	float initialStdDev = 0.01f;
};

struct TargetTrackingParameters
{
	// Prediction
	float minUncertainty3D = 0.01f;
	float addUncertaintyPx = 30.0f*PixelSize;
	// TODO: Rethink how tracking uncertainty maps to increased "search" range in both 2D and 3D
	float uncertaintySigma = 3;
	// Marker Matching
	float expandMarkerFoV = 0.0f;
	float normaliseDistance = 5.0f;
	TargetMatchingParametersFast matchFast = {};
	TargetMatchingParametersSlow matchSlow = {};
	TargetMatchingParametersUncertain matchUncertain = {};
	TargetMatchingParametersSlow matchSlowSecond = {
		1.0f, 0.0f, 1.0f,
		3, 5.0f*PixelSize, 5.0f*PixelSize
	};
	// Quality
	int cameraGoodObs = 5;
	float cameraGoodRatio = 0.5f;
	int minTotalObs = 6;
	int minCameraObs = 1;
	float maxTotalError = 5.0f*PixelSize;
	// Optimisation
	TargetOptimisationParameters opt = {};
	// Filtering
	TargetFilteringParameters filter = {};
	// Tracking Loss
	float lostTargetCoastMS = 100.0f;
};

// Two more used statically, no easy access to TargetDetectionParameters::triDetection, but recorded here nonetheless
extern int triTargetMinRelations;
extern float triTargetMinDistance;

#endif // TARGET_PARAMETERS_H