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
	float matchRadius = 2.0f*PixelSize;
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

	MatchingParameters match = { 0.02f, 2, 2, 2, 1 };
};

struct TargetMatchingParametersUncertain
{
	// Accounting for excessive uncertainty after initial match
	float perpDeviation = 1.2f;
	float stepLength = 3.0f*PixelSize;
	int maxSteps = 20;
	TargetMatchingParametersFast subMatch = {};
};
struct TargetOptimisationParameters
{
	int maxOutlierIterations = 20;
	int maxRefineIterations = 5;
	float refineTolerances = 10.0f;
	float outlierTolerances = 200.0f;
	float outlierErrorLimit = 2.0f*PixelSize;
	float outlierGrouping = 0.9f;
	float outlierSigma = 2.5f;
	float outlierVarMin = 0.5f*PixelSize;
	float predictionInfluence = 0.000005f;
};

struct TargetFilteringParameters
{
	// General behaviour for all filters
	float sigmaInitState = 50000, sigmaInitChange = 10000000;
	float detectSigma = 1000000, trackSigma = 1;
	float dampeningPos = 0.95f, dampeningRot = 0.9f;

	// UKR settings for all filters
	float sigmaAlpha = 0.001f, sigmaBeta = 2.0f, sigmaKappa = 0.0f;

	// Full Target Pose Filter Update
	struct {
		bool useUnscented = true;
		bool useNumericCov = false;
		bool useNumericCovPos = false;
		float stdDevPos = 0.0001f, stdDevEXP = 0.002f;
	} pose;

	// Partial Target Points Filter Update
	struct {
		float stdDev = 3.0f * PixelSize;
		int obsLimit = 8;
		bool useUnscented = false;
		bool useNumericJac = true;
		bool separateCorrections = false;
	} point;

	// IMU Filter Update
	struct {
		float stdDevAccel = 0.00001f, stdDevIMU = 0.004f;
		bool useForPrediction = true;
	} imu;

	template<typename Scalar>
	Eigen::Matrix<Scalar,6,6> getSyntheticCovariance() const
	{
		Eigen::Matrix<Scalar,6,6> covariance = Eigen::Matrix<Scalar,6,6>::Identity();
		covariance.diagonal().template head<3>().setConstant(pose.stdDevPos*pose.stdDevPos);
		covariance.diagonal().template tail<3>().setConstant(pose.stdDevEXP*pose.stdDevEXP);
		return covariance;
	}
};

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

struct TargetDetectionConfig
{
	bool match3D = true; // Quick, but may not work on all targets
	// Computationally intensive 2D detection methods (limited number active at the same time):
	bool search2D = true; // Faster, may not work on targets with few markers visible at once
	bool probe2D = true; // Slower, should work on all targets

	// Critical parameters that may be tuned per target:
	int probeCount = 1000;
};

struct TargetDetectionParameters
{
	bool useAsyncDetection = true;
	struct
	{
		float errorMax = 5.0f*PixelSize;
		float errorSigma = 3;
		int maxCandidates = 10;
		bool allowSingleCamera = true;
	} search;
	RotationGenerationParameters rotGen;
	struct
	{
		int minObs = 8;
		float errorInitialMax = 8.0f*PixelSize;
		float errorMax = 2.0f*PixelSize;
		int maxCandidates = 20;
	} probe;
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
		int total = 10;
	} minObservations;
};

struct TargetTrackingParameters
{
	// Prediction
	float minUncertainty3D = 0.02f;
	float uncertaintySigma = 3;
	// Marker Matching
	float expandMarkerFoV = -0.3f;
	float normaliseDistance = 5.0f;
	TargetMatchingParametersFast matchFast = {};
	struct { // Selecting whether to go down recover path
		int minCamerasGood = 2;
		int maxRecoverStages = 2;
	} selectRecover;
	TargetMatchingParametersSlow matchRecover = {};
	struct { // Selecting whether to go down single camera uncertainty path
		// Uncertain matching is too unstable, may destroy a fine matching
		// A/B Testing didn't result in a clear benefit either way, so rework it in the future
		int maxCamerasGood = 0; // Disabled via this. Would be 2
		float maxDominantFactor = 2.0f;
	} selectUncertain = {};
	TargetMatchingParametersUncertain matchUncertain = {};
	TargetMatchingParametersFast matchFastFinal = {};
	// Quality
	struct {
		int cameraGoodObs = 5;
		float cameraGoodRatio = 0.5f;
		int minImprovePoints = 2;
		float minImproveFactor = 1.3f;
		int minTotalObs = 3;
		int minCameraObs = 1;
		float maxTotalError = 2.0f*PixelSize;
	} quality = {};
	// Mistrust
	struct {
		float noTrackingAccum = 0.2f;
		float highErrorFactor = 100.0f;
		float highErrorThreshold = 0.8f*PixelSize;
		float matchedMarkerFactor = 0.99f;
		float conflictedMarkerAccum = 0.05f;
		float unmatchedCertainAccum = 0.015f;
		float unmatchedObservationsAccum = 0.005f;
		float unmatchedProjectionsAccum = 0.000f;
		float closebyObservationsRange = 1.0f;
		float maxMistrust = 1.0f;
	} mistrust = {};
	// Optimisation
	TargetOptimisationParameters opt = {};
	// Filtering
	TargetFilteringParameters filter = {};
	// Tracking Loss
	float lostTargetCoastMS = 100.0f;
	float coastMinTrackTime = 50.0f;
};

// Two more used statically, no easy access to TargetDetectionParameters::triDetection, but recorded here nonetheless
extern int triTargetMinRelations;
extern float triTargetMinDistance;

#endif // TARGET_PARAMETERS_H