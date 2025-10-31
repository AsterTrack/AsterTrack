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

#include "detection2D.hpp"

#include "target/parameters.hpp"
#include "target/rotationGen.hpp"

#include "util/matching.hpp"
#include "util/eigenalg.hpp"
#include "util/stats.hpp"
#include "util/log.hpp"

#include "poselib/p3p_ding.hpp"

#include <algorithm>
#include <random>
#include <numeric>
#include <cassert>


/**
 * Detecting a target (set of markers) in a triangulated 3D point cloud
 */

static std::mt19937 gen = std::mt19937(std::random_device{}());

/* Functions */

static std::vector<std::array<int,3>> generateTriplets(int n);

static std::pair<int, float> getMatchErrorApprox(const std::vector<Eigen::Vector2f> &points2D, const std::vector<Eigen::Vector2f> &reprojected2D, float maxError);

[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
TargetMatch2D probeTarget2D(std::stop_token stopToken, const TargetCalibration3D &target3D, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, 
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	Eigen::Vector3f pos, int cameraCount, int probeCount, const TargetDetectionParameters &params, const TargetTrackingParameters &track, 
	TargetTracking2DData &internalData)
{
	ScopedLogCategory scopedLogCategory(LDetection2D);

	// Brute-force a few candidate poses based on 3 points only
	assert(probeCount > 0);
	auto gen = params.rotGen;
	float axisCount = std::pow(probeCount, 0.3334f);
	gen.rollAxisShells = std::max<int>(1, std::floor(axisCount));
	gen.shellPoints = probeCount/gen.rollAxisShells;
	LOG(LDetection2D, LDebug, "Probing %d rotations, %d on the sphere with %d rotation shells (%f equal split)",
		probeCount, gen.shellPoints, gen.rollAxisShells, axisCount);

	std::vector<Eigen::Quaternionf> rotations(gen.rollAxisShells*gen.shellPoints);
	for (int r = 0; r < gen.rollAxisShells; r++)
	{
		float rollAngle = 2*PI * ((float)r)/gen.rollAxisShells;
		for (int i = 0; i < gen.shellPoints; i++)
			rotations[r*gen.shellPoints + i] = generateRotation(gen, r % gen.shells.size(), i, rollAngle);
	}

	// Normalise pixel parameters to a fixed distance
	std::vector<float> paramScale(calibs.size());
	for (int c = 0; c < calibs.size(); c++)
		paramScale[c] = params.normaliseDistance / (pos - calibs[c].transform.translation().cast<float>()).norm();

	if (stopToken.stop_requested())
		return {};

	// Evaluate all possible combinations
	std::vector<std::pair<int,float>> itResults(rotations.size(), { 0, std::numeric_limits<float>::max() });
	std::vector<TargetMatch2D> itMatches(rotations.size());
#pragma omp parallel for schedule(static, 100)
	for (int i = 0; i < rotations.size(); i++)
	{
		const auto &quat = rotations[i];
		if (stopToken.stop_requested())
			continue;

		Eigen::Isometry3f estimate;
		estimate.linear() = quat.toRotationMatrix();
		estimate.translation() = pos;

		// Find candidate for 2D point matches
		TargetMatch2D targetMatch2D = {};
		targetMatch2D.calib = &target3D;
		targetMatch2D.pose = estimate;
		targetMatch2D.points2D.resize(cameraCount); // Indexed with calib.index, not index into calibs

		// Reused allocation of target point reprojections
		thread_local std::vector<std::vector<Eigen::Vector2f>> projected2D;
		thread_local std::vector<std::vector<int>> relevantProjected2D;
		if (projected2D.size() < points2D.size())
			projected2D.resize(points2D.size());
		if (relevantProjected2D.size() < points2D.size())
			relevantProjected2D.resize(points2D.size());

		for (int c = 0; c < calibs.size(); c++)
		{
			if (!relevantPoints2D[c] || relevantPoints2D[c]->empty()) continue;
			projectTarget(projected2D[c], relevantProjected2D[c],
				target3D, calibs[c], targetMatch2D.pose, params.expandMarkerFoV);
			if (relevantProjected2D[c].empty()) continue;

			// Reused allocation of target matching data
			thread_local TargetMatchingData matchingStage;
			matchingStage.clear();

			// Match relevant points (observation and projected target)
			// Note: We use "Slow" here because it can deal with our position estimate being wildly off
			auto &cameraMatches = targetMatch2D.points2D[calibs[c].index];
			matchTargetPointsSlow(*points2D[c], *properties[c], *relevantPoints2D[c],
				projected2D[c], relevantProjected2D[c], cameraMatches,
				matchingStage, track.matchSlow, paramScale[c]);
			if (cameraMatches.size() < track.quality.minCameraObs)
				cameraMatches.clear();
			targetMatch2D.error.samples += cameraMatches.size();
		}

		if (targetMatch2D.error.samples >= params.probe.minObs)
		{ // Initial optimisation
			optimiseTargetPose<false>(calibs, points2D, targetMatch2D,
				Eigen::Isometry3f::Identity(), params.opt);
		}

		itResults[i] = { targetMatch2D.error.samples, targetMatch2D.error.mean };

		bool accept = targetMatch2D.error.samples >= params.probe.minObs
					&& targetMatch2D.error.mean < params.probe.errorInitialMax;
		if (accept)
		{
			itMatches[i] = targetMatch2D;
		}
	}
	if (stopToken.stop_requested())
		return {};

	auto &results = itResults;
	float maxError = params.probe.errorInitialMax;
	auto cullTheForgotten = [&](const int &a, const int &b) {
		return (results[a].first < params.probe.minObs && results[b].first < params.probe.minObs)
			|| (results[a].second > maxError && results[b].second > maxError);
	};
	auto errorRules = [&](const int &a, const int &b) {
		return results[a].second > maxError || results[b].second > maxError? 
			results[a].second < results[b].second :
			(results[a].first > results[b].first || (results[a].first == results[b].first && results[a].second < results[b].second));
	};
	auto pointSupremacy = [&](const int &a, const int &b) {
		return results[a].first > results[b].first;
	};
	auto conservativeOrder = [&](const int &a, const int &b) {
		if (cullTheForgotten(a, b)) return false;
		return results[a].first < params.probe.minObs || results[b].first < params.probe.minObs? 
			pointSupremacy(a, b) : errorRules(a, b);
	};

	// Sort solutions by lowest error
	std::vector<int> bestSolutions(rotations.size());
	std::iota(bestSolutions.begin(), bestSolutions.end(), 0);
	std::stable_sort(bestSolutions.begin(), bestSolutions.end(), conservativeOrder);

	std::vector<std::pair<int,float>> candResults(params.probe.maxCandidates, { 0, std::numeric_limits<float>::max() });
	std::vector<TargetMatch2D> candMatches(params.probe.maxCandidates);
	int index = 0;
	for (int it : bestSolutions)
	{
		auto &res = itResults[it];
		if (res.first < params.probe.minObs) break;
		if (res.second >= params.probe.errorInitialMax)
		{
			LOG(LDetection2D, LTrace, "    Discarded: Rotation %d resulted in %d matches, error %fpx!", it, res.first, res.second*PixelFactor);
			continue;
		}
		LOG(LDetection2D, LDebug, "    Best: Rotation %d resulted in %d matches, error %fpx!", it, res.first, res.second*PixelFactor);

		TargetMatch2D targetMatch2D = std::move(itMatches[it]);
		TargetMatchError prevErrors = evaluateTargetPose(calibs, points2D, targetMatch2D);
		TargetMatchError newErrors = optimiseTargetPose<false>(calibs, points2D, targetMatch2D,
			Eigen::Isometry3f::Identity(), params.opt);
		if (newErrors.samples > 0)
		{
			LOGC(LDebug, "          Reduced average pixel error of %d points from %.4fpx to %d inliers with %.4fpx error\n",
				prevErrors.samples, prevErrors.mean * PixelFactor, newErrors.samples, newErrors.mean * PixelFactor);
		}
		else
			LOGC(LDebug, "          Failed to optimise pose of %d points!\n", prevErrors.samples);

		candResults[index] = { targetMatch2D.error.samples, targetMatch2D.error.mean };
		candMatches[index] = std::move(targetMatch2D);
		index++;
		if (index >= params.probe.maxCandidates)
			break;
	}
	if (index == 0)
		return {};

	// Sort solutions by lowest error
	results = candResults;
	maxError = params.probe.errorMax;
	std::vector<int> candSolutions(index);
	std::iota(candSolutions.begin(), candSolutions.end(), 0);
	std::stable_sort(candSolutions.begin(), candSolutions.end(), conservativeOrder);

	auto bestResult = candResults[candSolutions[0]];
	if (bestResult.first < params.minObservations.total || bestResult.second > params.probe.errorMax)
		return candMatches[candSolutions[0]];

	return candMatches[candSolutions[0]];
}

static std::vector<Eigen::Isometry3f> bruteForcePoseCandidates(std::stop_token stopToken,
	const TargetCalibration3D &target3D, const CameraCalib &calib,
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, const std::vector<int> &relevantPoints2D,
	const TargetDetectionParameters &params)
{
	if (relevantPoints2D.size() < 3)
		return {};

	// Choose 3 points from the image
	std::vector<int> triplet;
	// Random selection
	//std::sample(relevantPoints2D.begin(), relevantPoints2D.end(), std::back_inserter(triplet), 3, gen);
	// Deterministic random selection
	//triplet = { relevantPoints2D[0], relevantPoints2D[1], relevantPoints2D[2] };
	// Deterministically sample based on quality of blobs
	triplet = relevantPoints2D;
	std::partial_sort(triplet.begin(), triplet.begin()+3, triplet.end(),
		[&](const int &a, const int &b) { return properties[a].value > properties[b].value; });
	triplet.resize(3);	
	assert(triplet.size() == 3);

	// Convert picked triplets to (normalised) camera coordinates for p3p use
	std::array<Eigen::Vector3f, 3> camPoints_triplet;
	for (int i = 0; i < 3; ++i)
		camPoints_triplet[i] = (applyReprojection2D(calib, points2D[triplet[i]]).homogeneous().normalized());

	// Get all length 3 subsets of the target points to match the three image points to
	auto permIndices = generateTriplets(target3D.markers.size());
	// TODO: Precalculate triples likely to be seen together by analyzing projections from all angles

	// Evaluate all possible combinations
	std::vector<std::pair<int,float>> itResults(permIndices.size(), { 0, std::numeric_limits<float>::max() });
	std::vector<Eigen::Isometry3f> itPoses(permIndices.size());
#pragma omp parallel for schedule(static, 100)
	for (int i = 0; i < permIndices.size(); i++)
	{
		const auto &perm = permIndices[i];
		if (stopToken.stop_requested())
			continue;

		std::array<Eigen::Isometry3f,4> poses;
		int poseCnt = poselib::p3p_ding(camPoints_triplet,
							{ target3D.markers[perm[0]].pos, target3D.markers[perm[1]].pos, target3D.markers[perm[2]].pos },
							poses);

		for (int j = 0; j < poseCnt; ++j)
		{
			if (!poses[j].matrix().allFinite())
				continue; // Does happen in the order of about 1/1000

			// Reproject target points and estimate 2D error
			thread_local std::vector<Eigen::Vector2f> projected2D;
			projectTarget(projected2D, target3D, calib, calib.transform.cast<float>()*poses[j], params.expandMarkerFoV);
			auto res = getMatchErrorApprox(points2D, projected2D, params.search.errorMax);
			if (res.first >= itResults[i].first && res.second < itResults[i].second)
			{
				itResults[i] = res;
				itPoses[i] = poses[j];
			}
		}
	}
	if (stopToken.stop_requested())
		return {};

	// Sort solutions by lowest error
	std::vector<int> bestSolutions(permIndices.size());
	std::iota(bestSolutions.begin(), bestSolutions.end(), 0);
	std::stable_sort(bestSolutions.begin(), bestSolutions.end(),
		[&itResults](const int &a, const int &b) { return itResults[a].first > itResults[b].first || (itResults[a].first == itResults[b].first && itResults[a].second < itResults[b].second); });

	auto bestResult = itResults[bestSolutions[0]];
	if (bestResult.first < params.minObservations.focus || bestResult.second > params.search.errorMax)
		return {};

	std::vector<Eigen::Isometry3f> poseCandidates;
	poseCandidates.reserve(params.search.maxCandidates);
	for (int it : bestSolutions)
	{
		if (itResults[it].first < params.minObservations.focus || itResults[it].second > bestResult.second*params.search.errorSigma || itResults[it].second > params.search.errorMax)
			break;
		LOGC(LTrace, "    Considering pose candidate %d from iteration %d with %d matches and %fpx RMSE!",
			(int)poseCandidates.size(), it, itResults[it].first, itResults[it].second*PixelFactor);
		// Record pose transformed from camera-space into world-space
		poseCandidates.push_back(calib.transform.cast<float>() * itPoses[it]);
		if (poseCandidates.size() >= params.search.maxCandidates)
			break;
	}
	return poseCandidates;
}

[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
TargetMatch2D searchTarget2D(std::stop_token stopToken, const TargetCalibration3D &target3D, const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D, 
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	int focusCamera, int cameraCount, const TargetDetectionParameters &params, const TargetTrackingParameters &track, 
	TargetTracking2DData &internalData)
{
	ScopedLogCategory scopedLogCategory(LDetection2D);

	int focus = calibs[focusCamera].index;
	LOGC(LDebug, "Brute forcing camera %d's %d points!",
		focus, (int)relevantPoints2D[focusCamera]->size());

	// Brute-force a few candidate poses based on 3 points only
	auto candidates = bruteForcePoseCandidates(stopToken, target3D, calibs[focusCamera],
		*points2D[focusCamera], *properties[focusCamera], *relevantPoints2D[focusCamera], params);
	if (stopToken.stop_requested())
		return {};

	LOGC(LDebug, "Found %d candidates by brute forcing camera %d's %d points!",
		(int)candidates.size(), focus, (int)relevantPoints2D[focusCamera]->size());

	thread_local std::vector<Eigen::Vector2f> projected2D;
	thread_local std::vector<int> relevantProjected2D;

	// Verify pose candidates with other points of focus camera, then with other cameras
	TargetMatch2D bestMatch = {};
	bestMatch.error.mean = std::numeric_limits<float>::max();
	TargetMatchingData matchData = {};
	TargetTracking2DData trackData = {};
	trackData.init(cameraCount);
	int i = 0;
	for (const Eigen::Isometry3f &pose : candidates)
	{
		if (stopToken.stop_requested())
			return {};

		TargetMatch2D targetMatch2D = {};
		targetMatch2D.calib = &target3D;
		targetMatch2D.pose = pose;
		targetMatch2D.points2D.resize(cameraCount);

		// Project using initial pose estimate using only focus camera
		projectTarget(projected2D, relevantProjected2D, target3D, calibs[focusCamera], pose, params.expandMarkerFoV);
		if (relevantProjected2D.size() < params.minObservations.focus)
			continue;

		// Normalise pixel parameters to 2m distance
		float distFactor = params.normaliseDistance / (pose.translation() - calibs[focusCamera].transform.translation().cast<float>()).norm();

		// Add focus camera point matches using initial pose
		auto &cameraMatches = targetMatch2D.points2D[focus];
		matchTargetPointsSlow(*points2D[focusCamera], *properties[focusCamera], *relevantPoints2D[focusCamera],
			projected2D, relevantProjected2D, cameraMatches, matchData,
			params.match, distFactor);
	
		{ // For debug only
			TargetMatchError errors = evaluateTargetPose(calibs, points2D, targetMatch2D);
			LOGC(LTrace, "    Candidate %d matched %d points in focus camera %d with error %fpx!",
				i++, errors.samples, focus, errors.mean*PixelFactor);
		}
		if (cameraMatches.size() < params.minObservations.focus)
			continue;

		// Optimise and remove outliers
		TargetMatchError errors = optimiseTargetPose<false>(calibs, points2D, targetMatch2D, pose, params.opt, track.filter.point.stdDev, true);
		LOGC(LDebug, "        Optimised to %d points with error %fpx!",
			errors.samples, errors.mean*PixelFactor);

		// Potentially overwrite numeric covariance with default initial covariance
		targetMatch2D.covariance = track.filter.getSyntheticCovariance<float>() * track.filter.detectSigma;

		{
			auto stdDev = targetMatch2D.covariance.diagonal().cwiseSqrt();
			Eigen::Vector3f devPos = stdDev.head<3>()*1000, devRot = stdDev.tail<3>()*1000;
			LOGC(LDebug, "            Has stdDev of (%.2f,%.2f,%.2f)mm (%.2f,%.2f,%.2f) rotation!",
				devPos.x(), devPos.y(), devPos.z(), devRot.x(), devRot.y(), devRot.z());
		}

		// Properly track to expand to other cameras to acquire more certainty
		targetMatch2D = trackTarget2D(target3D, targetMatch2D.pose, targetMatch2D.covariance,
			calibs, cameraCount, points2D, properties, relevantPoints2D, track, trackData);

		if (bestMatch.error.mean/bestMatch.error.samples > targetMatch2D.error.mean/targetMatch2D.error.samples)
		{
			LOGC(LDebug, "      Candidate ursurped current best candidate with %d points and error %fpx (ratio %f better than %f)!",
				targetMatch2D.error.samples, errors.mean*PixelFactor, targetMatch2D.error.mean/targetMatch2D.error.samples, 
				bestMatch.error.mean/bestMatch.error.samples);
			bestMatch = std::move(targetMatch2D);
			std::swap(internalData, trackData);
		}
	}

	return bestMatch;
}

static std::vector<std::array<int,3>> generateTriplets(int n)
{
	std::vector<std::array<int,3>> triplets;
	triplets.reserve(n*n*n);
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < n; ++j)
		{
			if (i == j) continue;
			for (int k = 0; k < n; ++k)
			{
				if (i == k || j == k) continue;
				triplets.push_back({ i, j, k });
			}
		}
	}
	return triplets;
}

static std::pair<int, float> getMatchErrorApprox(const std::vector<Eigen::Vector2f> &points2D, const std::vector<Eigen::Vector2f> &reprojected2D, float maxError)
{
	float error = 0;
	int maxMatches = std::min(points2D.size(), reprojected2D.size());
	MultipleExtremum<float, -1> bestErrors(std::numeric_limits<float>::max());
	for (int i = 0; i < points2D.size(); ++i)
	{
		float minErrorSq = std::numeric_limits<float>::max();
		for (int j = 0; j < reprojected2D.size(); ++j)
		{
			float errorSq = (points2D[i] - reprojected2D[j]).squaredNorm();
			if (errorSq < minErrorSq)
				minErrorSq = errorSq;
		}
		if (minErrorSq < maxError*maxError)
			bestErrors.min(minErrorSq, maxMatches);
	}
	return std::make_pair((int)bestErrors.rank.size(), std::sqrt(bestErrors.average()));
}