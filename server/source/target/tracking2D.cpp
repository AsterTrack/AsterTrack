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

#include "target/tracking2D.hpp"

#include "target/parameters.hpp"
#include "target/target.hpp"
#include "target/TargetReprojectionError.hpp"

#include "util/matching.hpp"
#include "util/stats.hpp"
#include "util/eigenalg.hpp"
#include "util/log.hpp"

#include "util/debugging.hpp"
#include "util/stats.hpp"

#include "unsupported/Eigen/NonLinearOptimization"

/**
 * Fitting a target (set of markers) to observed points in 2D space
 */


/* Functions */

const char *getStopCodeText(int status); // calib/optimisation.hpp

UncertaintyAxisAlignment::UncertaintyAxisAlignment()
{
	projMin = std::numeric_limits<float>::max();
	projMax = 0;
	obsMin = std::numeric_limits<float>::max();
	obsMax = 0;
}

/**
 * Analyze the points and updates uncertainty with a set of potential shifts along the axis to align them
 */
int shiftPointsAlongAxis(
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<int> &relevantPoints2D,
	const std::vector<Eigen::Vector2f> &projected2D, const std::vector<int> &relevantProjected2D,
	const CameraCalib &calib, Ray3f uncertainAxis, Eigen::Vector3f pos, float axisPerpShift, 
	UncertaintyAxisAlignment &uncertainty)
{
	// Project uncertainAxis into camera to get direction to shift points in (fine approximation for small shifts)
	uncertainty.rayCam = projectPoint2D(calib.camera, uncertainAxis.pos);
	uncertainty.rayTgt = projectPoint2D(calib.camera, pos);
	uncertainty.rayDir = (uncertainty.rayTgt - uncertainty.rayCam).normalized();

	// Initial idea: Find match allowing shifts along ray:
	// Project relevantProjected2D onto ray
	// Find maximum distance of any such projection onto ray)
	// Consider observed points within that distance of the ray + margin
	// Project those onto ray as well
	// Match primary/secondary local peak density along rays, with fallbacks (multiple candidates)
	// Try each candidate with matchTargetPoints

	// TODO: Make more robust to other points of other targets in sweeping area, maybe initial idea above
	// Also consider working on clusters of points, only consider closest cluster / start with closest

	// TODO: Ignore distance along this axis when doing matchTargetPoints
	// Most likely do a custom version of matchTargetPoints for specific use in this scenario
	// Since using this method means a single camera was dominant and pose was optimised on it already
	// So rotation and position along this axis is probably constrained quite well
	// So the robust version of matchTargetPoints might not be needed, a quick specialised version sounds better

	// Right now, just find min/max potential shift and try each candidate in that range
	float widthSq = 0;
	for (int p : relevantProjected2D)
	{
		Eigen::Vector2f pt = projected2D[p];
		float section = uncertainty.rayDir.dot(pt - uncertainty.rayCam)
			/ uncertainty.rayDir.dot(uncertainty.rayDir);
		Eigen::Vector2f proj = uncertainty.rayCam + uncertainty.rayDir * section;
		widthSq = std::max(widthSq, (pt - proj).squaredNorm());
		uncertainty.projMin = std::min(uncertainty.projMin, section);
		uncertainty.projMax = std::max(uncertainty.projMax, section);
	}
	widthSq = widthSq * axisPerpShift * axisPerpShift;
	int obsConsidered = 0;
	for (int p : relevantPoints2D)
	{
		Eigen::Vector2f pt = points2D[p];
		float section = uncertainty.rayDir.dot(pt - uncertainty.rayCam)
			/ uncertainty.rayDir.dot(uncertainty.rayDir);
		Eigen::Vector2f proj = uncertainty.rayCam + uncertainty.rayDir * section;
		if ((pt - proj).squaredNorm() > widthSq) continue;
		uncertainty.obsMin = std::min(uncertainty.obsMin, section);
		uncertainty.obsMax = std::max(uncertainty.obsMax, section);
		obsConsidered++;
	}
	return obsConsidered;
}

/**
 * Raise x to the power of integer n.
 * Very quick for small n.
 */
template<typename T>
static constexpr inline T pown(T x, unsigned n)
{
	T result = 1;
	while (n)
	{
		if (n&1) result *= x;
		x *= x;
		n >>= 1;
	}
	return result;
}

/**
 * Matches a relevant set of projected target points to a relevant set of point observations
 */
bool matchTargetPointsFast(
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, const std::vector<int> &relevantPoints2D,
	const std::vector<Eigen::Vector2f> &projected2D, const std::vector<int> &relevantProjected2D,
	std::vector<std::pair<int, int>> &matches, TargetMatchingData &matchData,
	const TargetMatchingParametersFast params, float distFactor, Eigen::Vector2f offset)
{
	ScopedLogCategory scopedLogCategory(LTrackingMatch);

	// Store internal data for visualisation purposes (low overhead)
	if (matchData.markers.size() < relevantProjected2D.size())
	{ // Add space, right up to full marker count for this target because why not
		matchData.markers.resize(projected2D.size());
		for (int m = 0; m < projected2D.size(); m++)
			matchData.markers[m].matches.reserve(10);
	}
	matchData.markerCount = relevantProjected2D.size();
	matchData.identifier = 0;
	matchData.lower = params.matchRadius*distFactor;
	matchData.upper = matchData.lower*params.match.primAdvantage;
	matches.clear();

	typedef MatchCandidate<TargetMatchingData::Point2DMatchCandidate*> PointMatchCand;
	thread_local std::vector<MatchCandidates<int,PointMatchCand,2>> matchCandidates;
	matchCandidates.clear();
	matchCandidates.resize(relevantProjected2D.size());

	float matchRadius = params.matchRadius*distFactor;
	float includeRadiusSq = matchRadius*matchRadius*params.match.primAdvantage*params.match.primAdvantage;
	LOGC(LDebug, "           Attempting to find matches between %d and %d points\n", (int)relevantProjected2D.size(), (int)relevantPoints2D.size());

	int matchCnt = 0;
	TargetMatchingData::Point2DMatchCandidate match = {};
	for (int m = 0; m < relevantProjected2D.size(); m++)
	{
		auto &mkMatch = matchData.markers[m];
		mkMatch.matches.clear();
		mkMatch.marker = relevantProjected2D[m];
		mkMatch.projected = projected2D[relevantProjected2D[m]] + offset;
		Eigen::Vector2f projPoint2D = projected2D[relevantProjected2D[m]] + offset;
		matchCandidates[m].context = m;
		PointMatchCand matchCand = {};
		for (int p = 0; p < relevantPoints2D.size(); p++)
		{
			match.offset = points2D[relevantPoints2D[p]] - projPoint2D;
			match.distSq = match.offset.squaredNorm();
			if (match.distSq > includeRadiusSq) continue;
			match.index = relevantPoints2D[p];
			float sizeSq = properties[match.index].size*properties[match.index].size;
			match.value = std::sqrt(std::max(0.0f, match.distSq-sizeSq));
			matchData.markers[m].matches.push_back(match);

			// Add match
			matchCand.index = match.index;
			matchCand.value = match.value;
			matchCand.context = &match;
			recordMatchCandidate(matchCandidates[m], matchCand);
		}
		matchCnt += matchData.markers[m].matches.size();
	}
	if (matchCnt == 0)
	{ // Just in case
		LOGC(LDebug, "               Got 0 match candidates in quick match, aborting!\n");
		return false;
	}

	// Resolve match candidates tentatively
	int numMatches = resolveMatchCandidates(matchCandidates, points2D.size(), params.match);
	if (numMatches <= 0)
	{ // No hope if even relaxed constraints resulted in 0
		// 1 match for fast algorithm means something, do not discard like for slow algorithm
		LOGC(LTrace, "               Got %d candidates but only %d valid matches!\n",
			(int)matchCandidates.size(), numMatches);
		return false;
	}
	LOGC(LTrace, "               Got %d final sorted candidates, of which %d are valid!\n", (int)matchCandidates.size(), numMatches);

	// Gather resolved matching points
	matches.reserve(numMatches);
	for (auto &cand : matchCandidates)
	{
		if (cand.matches.empty()) continue;
		auto &match = cand.matches.front();
		if (match.valid() && match.value < matchRadius)
		{ // TODO: Technically match.value also accounts for size, so not entirely accurate, but fine
			LOGC(LTrace, "                   Accepted match mk%d / pt%d with distance %.4fpx (max %fpx)\n",
				relevantProjected2D[cand.context], match.index, match.value*PixelFactor, matchRadius*PixelFactor);
			matches.push_back({ relevantProjected2D[cand.context], match.index });

			// Update internal data for visualisation purposes (low overhead)
			auto &mkMatch = matchData.markers[cand.context];
			for (auto &ptMatch : mkMatch.matches)
			{
				if (ptMatch.index == match.index)
				{
					ptMatch.accepted = true;
					break;
				}
			}
		}
		else if (match.index >= 0)
		{
			LOGC(LTrace, "                   Discarded match mk%d / pt%d with distance %.4fpx (max %fpx)\n",
				relevantProjected2D[cand.context], match.index, match.value*PixelFactor, matchRadius*PixelFactor);
		}
	}
	LOGC(LDebug, "               Matched %d points!\n", (int)matches.size());
	return true;
}

/**
 * Matches a relevant set of projected target points to a relevant set of point observations
 * This can recover from translational errors in image space, but may benefit from a fast match afterwards
 */
void matchTargetPointsRecover(
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, const std::vector<int> &relevantPoints2D,
	const std::vector<Eigen::Vector2f> &projected2D, const std::vector<int> &relevantProjected2D,
	std::vector<std::pair<int, int>> &matches, TargetMatchingData &matchData,
	const TargetMatchingParametersSlow params, float distFactor, Eigen::Vector2f offset)
{
	ScopedLogCategory scopedLogCategory(LTrackingMatch);

	// Store internal data for visualisation purposes (low overhead)
	if (matchData.markers.size() < relevantProjected2D.size())
	{ // Add space, right up to full marker count for this target because why not
		matchData.markers.resize(projected2D.size());
		for (int m = 0; m < projected2D.size(); m++)
			matchData.markers[m].matches.reserve(params.maxCandidates*2);
	}
	matchData.markerCount = relevantProjected2D.size();
	matchData.identifier = 1;
	matchData.lower = params.matchRadius*distFactor;
	matchData.upper = matchData.lower*params.match.primAdvantage;
	matches.clear();

	// Use dynamic MatchCandidates to adjust to params.candidateCount, try to keep matches allocations
	typedef MatchCandidate<TargetMatchingData::Point2DMatchCandidate*> PointMatchCand;
	thread_local std::vector<MatchCandidates<int,PointMatchCand,-1>> matchCandidates;
	for (int m = 0; m < matchCandidates.size(); m++)
		matchCandidates[m].matches.clear();
	int candidateCount = 0;


	float filterMatchValue = params.maxValue*params.match.primAdvantage;
	float matchRadiusSq = params.matchRadius*params.matchRadius*distFactor*distFactor;
	const float cutoff = 0.05f; // Max being 1
	float influenceFalloffEnd = std::pow(1/cutoff-1, 1.0f/params.influenceFalloff);
	float influenceFactorSq = influenceFalloffEnd/(params.influenceRadius*params.influenceRadius*distFactor*distFactor);
	float similarityFalloffEnd = std::pow(1/cutoff-1, 1.0f/params.similarityFalloff);
	float similarityFactorSq = similarityFalloffEnd/(params.similarityRadius*params.similarityRadius*distFactor*distFactor);
	float differenceFalloffEnd = std::pow(1/cutoff-1, 1.0f/params.differenceFalloff);
	float differenceFactorSq = differenceFalloffEnd/(params.differenceRadius*params.differenceRadius*distFactor*distFactor);

	// Sole purpose: If there's NO match within maxIncludeDistance, this will increase it to include at least a few points
	MultipleExtremum<float, 3> minimumDistanceSq(std::numeric_limits<float>::max());

	LOGC(LDebug, "           Attempting to find matches between %d and %d points\n", (int)relevantProjected2D.size(), (int)relevantPoints2D.size());
	
	auto gatherPointMatches = [&]()
	{
		int matches = 0;
		TargetMatchingData::Point2DMatchCandidate match = {};
		for (int m = 0; m < relevantProjected2D.size(); m++)
		{
			matchData.markers[m].matches.clear();
			Eigen::Vector2f projPoint2D = projected2D[relevantProjected2D[m]] + offset;
			for (int p = 0; p < relevantPoints2D.size(); p++)
			{
				match.offset = points2D[relevantPoints2D[p]] - projPoint2D;
				match.distSq = match.offset.squaredNorm();
				match.index = relevantPoints2D[p];
				if (match.distSq < matchRadiusSq)
					matchData.markers[m].matches.push_back(match);
				else
				 	minimumDistanceSq.min(match.distSq);
			}
			matches += matchData.markers[m].matches.size();
		}
		return matches;
	};

	if (gatherPointMatches() == 0)
	{ // Just in case
		LOGC(LDarn, "               Got 0 initial match candidates, trying closest points!\n");
		assert(minimumDistanceSq.weakest() >= 0); // Else relevantPoints2D.empty()
		// Get offset to closest point
		// TODO: Is a significant 2D offset beyond matchRadius actually happening in real testing?
		// If so, update to be more robust, which could allow us to lower matchRadius even further
		// e.g. X number of offset candidates based on all minimumDistanceSq.ranks
		// and offset beyond the closest match, e.g. so that the farthest projected point along the axis matches up
		offset.setConstant(NAN);
		for (int m = 0; m < relevantProjected2D.size(); m++)
		{
			Eigen::Vector2f projPoint2D = projected2D[relevantProjected2D[m]];
			for (int p = 0; p < relevantPoints2D.size(); p++)
			{
				Eigen::Vector2f dev = points2D[relevantPoints2D[p]] - projPoint2D;
				if (dev.squaredNorm() <= minimumDistanceSq.rank[0])
				{
					offset = dev;
					break;
				}
			}
			if (!offset.hasNaN())
				break;
		}
		if (offset.hasNaN())
			return;
		gatherPointMatches();
	}

	// Properly calculate values of matches
	for (int m = 0; m < relevantProjected2D.size(); m++)
	{
		auto &mkMatch = matchData.markers[m];
		mkMatch.marker = relevantProjected2D[m];
		mkMatch.projected = projected2D[relevantProjected2D[m]];
		LOGC(LTrace, "                 Projected point %d has %d match candidates:\n",
			m, (int)mkMatch.matches.size());
		if (mkMatch.matches.empty()) continue;

		if (params.mismatchFactor > 0 && params.influenceRadius > 0)
		{ // Calculate agreement of shift with neighbouring points
			for (int n = m+1; n < relevantProjected2D.size(); n++)
			{
				float distSq = (projected2D[relevantProjected2D[n]] - projected2D[relevantProjected2D[m]]).squaredNorm();
				if (distSq > params.influenceRadius*params.influenceRadius) continue;
				float influence = 1.0f / (1.0f + pown(distSq*influenceFactorSq, params.influenceFalloff));

				for (int q = 0; q < matchData.markers[m].matches.size(); q++)
				{
					auto &match1 = matchData.markers[m].matches[q];
					for (int r = 0; r < matchData.markers[n].matches.size(); r++)
					{
						auto &match2 = matchData.markers[n].matches[r];
						if (match1.index == match2.index) continue;
						float diffSq = (match1.offset - match2.offset).squaredNorm();
						float sizeSq = properties[match1.index].size*properties[match2.index].size;
						float errSq = std::max(0.0f, diffSq-sizeSq);
						float similarity = 1.0f / (1.0f + pown(errSq*similarityFactorSq, params.similarityFalloff));
						match1.similarity += similarity*influence;
						match2.similarity += similarity*influence;
						match1.influence += influence;
						match2.influence += influence;
						match1.influenceCount++;
						match2.influenceCount++;
					}
				}
			}
		}

		// Calculate final match values
		bool hasMatch = false;
		for (auto &match : mkMatch.matches)
		{
			// Similarity is near 0 if there's little evidence in the neighbourhood that it's a good match
			// If there is, it should have significant impact on the value
			// TODO: Rethink mismatch formula, is works ok but is not properly normalised
			// Really there is no thought-out reason for its current details, needs more brain matter too perfect
			match.mismatch = params.mismatchFactor == 0? 0 : std::pow(match.similarity, -params.mismatchPower);
			// Closeness should model distance influence on value if pose is already assumed to be decently accurate
			float diffSq = match.distSq; // match.offset.squaredNorm()
			// TODO: Include uncertainty axis (offset) or better, create separate matchTargetPoints for the uncertaintyAxis case (detailed above)
			float sizeSq = properties[match.index].size*properties[match.index].size;
			float errSq = std::max(0.0f, diffSq-sizeSq);
			match.difference = 1.0f - 1.0f / (1.0f + pown(errSq*differenceFactorSq, params.differenceFalloff));
			match.value = match.mismatch*params.mismatchFactor + match.difference*params.differenceFactor;

			if (match.value < filterMatchValue)
				hasMatch = true;
			LOGC(LTrace, "                   Candidate %d has value of %f, abnormality of %f from %f similarity grading among %d influences summing %f!\n",
				match.index, match.value, match.mismatch, match.similarity, match.influenceCount, match.influence);
		}
		if (!hasMatch) continue;

		// Record significant matches for evaluation
		if (matchCandidates.size() <= candidateCount)
			matchCandidates.push_back({});
		auto &cand = matchCandidates[candidateCount++];
		cand.context = m;
		PointMatchCand matchCand = {};
		for (auto &match : mkMatch.matches)
		{
			if (match.value < filterMatchValue)
			{
				matchCand.index = match.index;
				matchCand.value = match.value;
				matchCand.context = &match;
				recordMatchCandidate(cand, matchCand, params.maxCandidates);
			}
		}
		assert(!cand.matches.empty());
		LOGC(LTrace, "                 Projected point %d has %d match candidates recorded (%d total), best with value of %f!\n",
			m, (int)cand.matches.size(), (int)mkMatch.matches.size(), cand.matches.front().value);
	}

	// TODO: Sadly have to discarding excess matches allocations. matching.hpp can't deal with size limit on matchCandidates right now
	matchCandidates.resize(candidateCount);

	// Resolve match candidates tentatively
	int numMatches = resolveMatchCandidates(matchCandidates, points2D.size(), params.match);
	if (numMatches <= 1)
	{ // No hope if even relaxed constraints resulted in only 1
		// 1 match for slow algorithm means nothing, discard
		LOGC(LTrace, "               Got %d candidates but only %d valid matches!\n",
			(int)matchCandidates.size(), numMatches);
		return;
	}
	/* { // Boost other matches whose diff
		for (int i = 0; i < matchCandidates.size(); i++)
		{
			auto &match = matchCandidates[i];
			bool resolved = !match.matches.front().valid();
			for (auto &m : match.matches)
				m.invalid = false;
			if (!resolved) continue;
			for (int j = 0; j < matchCandidates.size(); j++)
			{
				if (i == j) continue;
				for (auto &m : matchCandidates[j].matches)
				{

				}
			}
		}
	} */
	// TODO: Consider using hungarian algorithm instead, the custom algorithm performs great but has many parameters

	LOGC(LTrace, "               Got %d final sorted candidates, of which %d are valid!\n", (int)matchCandidates.size(), numMatches);

	// Gather resolved matching points
	matches.reserve(numMatches);
	for (auto &cand : matchCandidates)
	{
		if (cand.matches.empty()) continue;
		auto &match = cand.matches.front();
		if (match.valid() && match.value < params.maxValue)
		{
			LOGC(LTrace, "                   Accepted match mk%d / pt%d with shift disagreement %.4f\n",
				relevantProjected2D[cand.context], match.index, match.value);
			matches.push_back({ relevantProjected2D[cand.context], match.index });

			// Update internal data for visualisation purposes (low overhead)
			auto &mkMatch = matchData.markers[cand.context];
			for (auto &ptMatch : mkMatch.matches)
			{
				if (ptMatch.index == match.index)
				{
					ptMatch.accepted = true;
					break;
				}
			}
		}
		else
		{
			LOGC(LTrace, "                   Discarded match mk%d with shift disagreement %.4f\n",
				relevantProjected2D[cand.context], match.value);
		}
	}
	LOGC(LDebug, "               Matched %d points!\n",
		(int)matches.size());
}


float matchTargetPointsAlongAxis(
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<BlobProperty> &properties, const std::vector<int> &relevantPoints2D,
	const std::vector<Eigen::Vector2f> &projected2D, const std::vector<int> &relevantProjected2D,
	std::vector<std::pair<int, int>> &matches, TargetMatchingData &matchData,
	const TargetMatchingParametersUncertain &params, float distFactor,
	UncertaintyAxisAlignment axis)
{
	// TODO: Reimplement to be much smarter

	float shiftMin = std::min(axis.obsMin-axis.projMin, axis.obsMax-axis.projMax);
	float shiftMax = std::max(axis.obsMin-axis.projMin, axis.obsMax-axis.projMax);
	LOG(LTrackingMatch, LDebug, "                Offset along uncertainty axis from %.1fpx to %.1fpx!",
		shiftMin*PixelFactor, shiftMax*PixelFactor);

	// Try multiple shift candidates along uncertainty axis
	float bestShift = NAN;
	std::vector<std::pair<int,int>> tempMatches;
	TargetMatchingData tempMatchData;
	int steps = std::max(1, std::min(params.maxSteps, (int)std::floor((shiftMax-shiftMin) / params.stepLength)));
	float shift = shiftMin, shiftStep = (shiftMax-shiftMin) / (steps-1);
	for (int i = 0; i < steps; i++)
	{
		// Using first match parameters for now since we expect an offset still
		matchTargetPointsFast(points2D, properties, relevantPoints2D, projected2D, relevantProjected2D,
			tempMatches, tempMatchData, params.subMatch, distFactor, axis.rayDir * shift);
		if (tempMatches.size() > matches.size() || (tempMatches.size() > 0 && tempMatches.size() == matches.size()))
		{
			matches.swap(tempMatches);
			std::swap(matchData.markers, tempMatchData.markers);
			matchData.markerCount = tempMatchData.markerCount;
			bestShift = shift;
		}
		shift += shiftStep;
	}
	matchData.identifier = 2;
	matchData.lower = params.subMatch.matchRadius*distFactor;
	matchData.upper = matchData.lower*params.subMatch.match.primAdvantage;
	return bestShift;
}

/**
 * Optimise the given target match and update its pose, pose error and variance
 */
template<bool REFERENCE, bool OUTLIER>
TargetMatchError optimiseTargetPose(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, const TargetCalibration3D &calib,
	Eigen::Isometry3f prediction, TargetOptimisationParameters params, float errorStdDev, bool updateCovariance)
{
	ScopedLogCategory scopedLogCategory(LTrackingOpt);
	assert(match.error.samples >= 0);

	// Initialise optimisation error term, preparing the data
	constexpr int OPTIONS = OptUndistorted | (OUTLIER? OptOutliers : 0) | (REFERENCE? OptReferencePose : 0);
	typedef TargetReprojectionError<double, OPTIONS> TgtError;
	TgtError errorTerm(calibs);
	if constexpr (REFERENCE)
		errorTerm.setReferencePose(prediction.cast<double>(), params.predictionInfluence);
	errorTerm.setData(points2D, match, calib);
	if (errorTerm.values() < errorTerm.inputs() || errorTerm.m_observedPoints.empty())
	{
		LOGC(LError, "Got only %d inputs to optimise %d parameters!\n", errorTerm.values(), errorTerm.inputs());
		return { 10000.0f, 0.0f, 10000.0f, 0 };
	}
	LOGC(LTrace, "        Optimising target pose using %d point matches!\n", (int)errorTerm.m_observedPoints.size());

	// On-the-fly outlier detection
	if constexpr (OUTLIER)
		errorTerm.m_outlierMap.resize(errorTerm.m_observedPoints.size());

	// Create initial parameter vector for optimisation
	Eigen::VectorXd poseVec;
	if constexpr (OPTIONS & OptCorrectivePose)
		poseVec = errorTerm.encodePose(Eigen::Isometry3d::Identity());
	else
		poseVec = errorTerm.encodePose(match.pose.cast<double>());

	Eigen::VectorXd errors(errorTerm.m_observedPoints.size());
	errorTerm.calculateSampleErrors(errorTerm.decodePose(poseVec), errors);
	float prevError = errors.mean();

	// Initialise optimisation algorithm
	Eigen::LevenbergMarquardt<TgtError, double> lm(errorTerm);
	// Max number of evaluations of errorTerm, bounds for number of iterations
	lm.parameters.maxfev = (params.maxOutlierIterations+params.maxRefineIterations)*10;
	auto updateTolerances = [&](bool outlierPasses)
	{
		float tolerances = outlierPasses? params.outlierTolerances : params.refineTolerances;
		lm.parameters.xtol = 0.001f * PixelSize * tolerances;
		lm.parameters.ftol = 0.0001f * tolerances;
		lm.parameters.gtol = 0.0001f * tolerances;
	};
	Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeInit(poseVec);
	if (status == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
	{
		LOGC(LError, "Improper optimisation input parameters!\n");
		return { 10000.0f, 0.0f, 10000.0f, 0 };
	}

	// Optimisation steps
	updateTolerances(OUTLIER);
	bool outlierPasses = OUTLIER;
	int outlierCount = 0;
	match.error.samples = errorTerm.m_observedPoints.size();
	int outIt = 0, refIt = 0;
	while (true)
	{
		status = lm.minimizeOneStep(poseVec);
		auto errors = lm.fvec.head(errorTerm.m_observedPoints.size()); // Ignore reference pose
		match.error.mean = errors.sum() / match.error.samples;
		match.error.stdDev = std::sqrt((errors.array() - match.error.mean).square().sum() / match.error.samples);
		match.error.max = errors.maxCoeff();
		LOGC(LTrace, "            Current Error: %.4fpx (max %f, inliers %d)\n", match.error.mean*PixelFactor, match.error.max*PixelFactor, match.error.samples);
		if (!outlierPasses)
		{
			if (++refIt >= params.maxRefineIterations)
				break;
			if (status != Eigen::LevenbergMarquardtSpace::Running)
			{
				LOGC(LDebug, "          Stopped optimisation: %s\n", getStopCodeText(status));
				break;
			}
			continue;
		}
		if constexpr (!OUTLIER) continue;
		if (++outIt >= params.maxOutlierIterations)
		{ // Leave outlier passes and refine
			LOGC(LDebug, "            Reached maximum outlier iterations of %d! Remaining Errors %.4fpx (max %f, inliers %d)\n",
				outIt, match.error.mean*PixelFactor, match.error.max*PixelFactor, match.error.samples);
			outlierPasses = false;
			updateTolerances(false);
			continue;
		}
		if (status == Eigen::LevenbergMarquardtSpace::Running)
		{ // Outlier tolerances not yet reached
			continue;
		}
		// Find outliers - remove one at a time in case of severe errors
		float maxAllowed = match.error.max > params.outlierErrorLimit?
			std::max(match.error.max * params.outlierGrouping * 0.9999f, params.outlierErrorLimit) : 
			match.error.mean + std::max(params.outlierSigma*match.error.stdDev, params.outlierVarMin);
		if (match.error.max < maxAllowed)
		{ // Leave outlier passes and refine
			LOGC(LDebug, "            No more outliers to discern at iteration %d! Remaining Errors %.4fpx (max %f, inliers %d)\n",
				outIt, match.error.mean*PixelFactor, match.error.max*PixelFactor, match.error.samples);
			outlierPasses = false;
			updateTolerances(false);
			continue;
		}
		LOGC(LDebug, "            Checking for outliers in iteration %d with max error allowed of %fpx! Current Error: %.4fpx (max %f, inliers %d)\n",
			outIt, maxAllowed*PixelFactor, match.error.mean*PixelFactor, match.error.max*PixelFactor, match.error.samples);
		int newOutliers = 0;
		for (int i = 0; i < errorTerm.m_observedPoints.size(); i++)
		{
			int cam = errorTerm.m_observedPoints[i].camera;
			// TODO: Per camera outlier limit, definitely need to treat dominant cameras differently than fringe cameras
			if (errors(i) < maxAllowed) continue;
			float outlierError = errors(i);
			// TODO: Determine point outliers using jacobian
			// A point that goes against the general direction of the jacobian is more likely to be an outlier
			match.error.samples--;
			outlierCount++;
			errorTerm.m_outlierMap[i] = true;
			errors(i) = 0;
			newOutliers++;

			// Rest is just for debug
			if (SHOULD_LOGC(LTrace))
			{
				Eigen::Vector2f point = errorTerm.m_observedPoints[i].obs.template cast<float>();
				int c = 0, marker = -1, pt = -1;
				for (c = 0; c < calibs.size(); c++)
					if (calibs[c].index == cam)
						break;
				const auto &pointsMatch = match.points2D[cam];
				for (int p = 0; p < pointsMatch.size(); p++)
				{
					if (points2D[c]->at(pointsMatch[p].second) != point) continue;
					marker = pointsMatch[p].first;
					pt = pointsMatch[p].second;
					break;
				}
				LOGC(LDebug, "            Found outlier (cam %d, marker %d, pt %d) with error %.4fpx! %d inliers left, with %.4f max\n",
					cam, marker, pt,
					outlierError*PixelFactor, match.error.samples, maxAllowed*PixelFactor);
			}
			else if (SHOULD_LOGC(LDebug))
			{
				LOGC(LDebug, "            Found outlier in cam %d with error %.4fpx! %d inliers left, with %.4f max\n",
					cam, outlierError*PixelFactor, match.error.samples, maxAllowed*PixelFactor);
			}
		}
		if (match.error.samples == 0 || errorTerm.values()-outlierCount < errorTerm.inputs())
			break;
		if (newOutliers == 0)
		{ // Leave outlier passes and refine
			LOGC(LDebug, "            Could not find any more outliers at iteration %d! Remaining Errors %.4fpx (max %f, inliers %d)\n",
				outIt, match.error.mean*PixelFactor, match.error.max*PixelFactor, match.error.samples);
			outlierPasses = false;
			updateTolerances(false);
			continue;
		}
	}
	LOGC(LDebug, "        Optimised target pose with %d points from %.4fpx to %.4fpx error and %d outliers in %d+%d / %d iterations!\n",
		(int)errorTerm.m_observedPoints.size(), prevError*PixelFactor, match.error.mean*PixelFactor,
		(int)errorTerm.m_observedPoints.size()-match.error.samples, outIt, refIt, params.maxOutlierIterations+params.maxRefineIterations);

	// Apply optimised pose
	if constexpr (OPTIONS & OptCorrectivePose)
		match.pose = match.pose * errorTerm.decodePose(poseVec).template cast<float>();
	else
		match.pose = errorTerm.decodePose(poseVec).template cast<float>();

	if (updateCovariance)
		match.covariance = errorTerm.covarianceEXP(match.pose, errorStdDev, match.deviations);

	if constexpr (OUTLIER) if (outlierCount > 0)
	{ // Remove detected outliers from target match
		float maxAllowed = std::min(params.outlierErrorLimit,
			match.error.mean + std::max(params.outlierSigma*match.error.stdDev, params.outlierVarMin));

		int errorIndex = 0;
		for (int c = 0; c < calibs.size(); c++)
		{
			const CameraCalib &calib = calibs[c];
			auto &pointsMatch = match.points2D[calib.index];
			int writeIndex = 0;
			int outliersCam = 0;
			for (int i = 0; i < pointsMatch.size(); i++)
			{
				if (errorTerm.m_outlierMap[errorIndex])
				{
					float outError = errorTerm.calculateSampleError(match.pose, errorIndex);
					if (outError > maxAllowed)
					{
						LOGC(LTrace, "            Camera %d Marker %d - Point %d, outlier with error %.4fpx!", calib.id, pointsMatch[i].first, pointsMatch[i].second, outError*PixelFactor);
						outliersCam++;
					}
					else
					{
						LOGC(LDebug, "            Camera %d Marker %d - Point %d, reinstated as inlier with error %.4fpx",
							calib.id, pointsMatch[i].first, pointsMatch[i].second, outError*PixelFactor);
						pointsMatch[writeIndex++] = pointsMatch[i];
						match.error.samples++;
						outlierCount--;
					}
				}
				else
				{
					LOGC(LTrace, "            Camera %d Marker %d - Point %d, remaining pixel error %.4fpx",
						calib.id, pointsMatch[i].first, pointsMatch[i].second, lm.fvec(errorIndex)*PixelFactor);
					pointsMatch[writeIndex++] = pointsMatch[i];
				}
				errorIndex++;
			}
			pointsMatch.resize(pointsMatch.size()-outliersCam);
		}
	}
	assert(match.error.samples >= 0);

	return match.error;
}

template TargetMatchError optimiseTargetPose<true, true>(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, const TargetCalibration3D &calib,
	Eigen::Isometry3f prediction, TargetOptimisationParameters params, float errorStdDev, bool updateCovariance);
template TargetMatchError optimiseTargetPose<false, true>(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, const TargetCalibration3D &calib,
	Eigen::Isometry3f prediction, TargetOptimisationParameters params, float errorStdDev, bool updateCovariance);

/**
 * Evaluate the given target match and update its pose error
 */
TargetMatchError evaluateTargetPose(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, const TargetCalibration3D &calib)
{
	TargetReprojectionError<float, OptUndistorted> errorTerm(calibs);
	errorTerm.setData(points2D, match, calib);
	if (errorTerm.m_observedPoints.size() == 0)
		return match.error = { 0, 0, 0, 0 };
	Eigen::VectorXf errors(errorTerm.m_observedPoints.size());
	errorTerm.calculateSampleErrors(match.pose, errors);
	match.error.samples = errorTerm.m_observedPoints.size();
	match.error.mean = errors.sum() / match.error.samples;
	match.error.stdDev = std::sqrt((errors.array() - match.error.mean).square().sum() / match.error.samples);
	match.error.max = errors.maxCoeff();
	return match.error;
}

/**
 * Evaluate the given target match and update its pose error, and numerically calculates its covariance
 */
TargetMatchError evaluateTargetPoseCovariance(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, const TargetCalibration3D &calib, float errorStdDev)
{
	TargetReprojectionError<float, OptUndistorted> errorTerm(calibs);
	errorTerm.setData(points2D, match, calib);
	if (errorTerm.m_observedPoints.size() == 0)
		return { 0, 0, 0, 0 };
	Eigen::VectorXf errors(errorTerm.m_observedPoints.size());
	errorTerm.calculateSampleErrors(match.pose, errors);
	match.error.samples = errorTerm.m_observedPoints.size();
	match.error.mean = errors.sum() / match.error.samples;
	match.error.stdDev = std::sqrt((errors.array() - match.error.mean).square().sum() / match.error.samples);
	match.error.max = errors.maxCoeff();
	match.covariance = errorTerm.covarianceEXP(match.pose, errorStdDev, match.deviations);
	return match.error;
}

/**
 * Updates visibleMarkers of target to those matched
 */
void updateVisibleMarkers(std::vector<std::vector<int>> &visibleMarkers, const TargetMatch2D &match)
{
	// Adopt camera count
	visibleMarkers.resize(match.points2D.size());
	// Register currently visible target points for next frame
	for (int c = 0; c < match.points2D.size(); c++)
	{
		auto &targetPoints2D = match.points2D[c];
		visibleMarkers[c].resize(targetPoints2D.size());
		for (int i = 0; i < targetPoints2D.size(); i++)
			visibleMarkers[c][i] = targetPoints2D[i].first;
	}
}

/**
 * Redetect the target in the observed 2D points using a predicted pose
 * Iteratively matches fast, then slow if needed, optimises, matches more, and optimises
 */
[[gnu::flatten, gnu::target_clones("arch=x86-64-v4", "default")]]
TargetMatch2D trackTarget2D(const TargetCalibration3D &target, Eigen::Isometry3f prediction, const CovarianceMatrix &covariance,
	const std::vector<CameraCalib> &calibs, int cameraCount,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	const TargetTrackingParameters &params, TargetTracking2DData &internalData)
{
	ScopedLogCategory scopedLogCategory(LTracking);
	assert(calibs.size() == points2D.size());
	assert(calibs.size() == properties.size());
	assert(calibs.size() == relevantPoints2D.size());

	// Find candidate for 2D point matches
	TargetMatch2D targetMatch2D = {};
	targetMatch2D.pose = prediction;
	targetMatch2D.points2D.resize(cameraCount); // Indexed with calib.index, not index into calibs

	// Add positional uncertainty in target-space (rotated by prediction) to target-local bounds
	Eigen::Vector3f uncertainty = sampleCovarianceUncertainty<float,3>(covariance.topLeftCorner<3,3>(),
		params.uncertaintySigma, prediction.rotation());
	uncertainty += Eigen::Vector3f::Constant(params.minUncertainty3D);
	Bounds3f targetBounds = target.bounds.extendedBy(uncertainty);

	// Reused allocation of per-camera data
	thread_local std::vector<std::vector<Eigen::Vector2f>> projected2D;
	thread_local std::vector<std::vector<int>> relevantProjected2D;
	thread_local std::vector<std::vector<int>> closePoints2D;
	if (projected2D.size() < calibs.size())
		projected2D.resize(calibs.size());
	if (relevantProjected2D.size() < calibs.size())
		relevantProjected2D.resize(calibs.size());
	if (closePoints2D.size() < calibs.size())
		closePoints2D.resize(calibs.size());

	// Clear internal data for visualisation purposes (low overhead)
	internalData.init(cameraCount);
	for (int c = 0; c < calibs.size(); c++)
		internalData.matching.at(calibs[c].index).clear();

	// Normalise pixel parameters to a fixed distance
	std::vector<float> paramScale(calibs.size());
	for (int c = 0; c < calibs.size(); c++)
		paramScale[c] = params.normaliseDistance / (prediction.translation() - calibs[c].transform.translation().cast<float>()).norm();

	bool newMatches = false;
	std::vector<std::pair<int, int>> matches;
	int matchedSamples = 0;

	auto reevaluateClosebyPoints = [&](Eigen::Vector3f uncertaintyRay = Eigen::Vector3f::Zero())
	{ // Find set of closeby points to consider
		// TODO: Use nearby cluster(s) from all cameras passed into this function
		for (int c = 0; c < calibs.size(); c++)
		{
			closePoints2D[c].clear();
			if (!relevantPoints2D[c] || relevantPoints2D[c]->empty()) continue;

			// Project target bounds and relevant target points into camera view
			Eigen::Projective3f mvp = calibs[c].camera.cast<float>() * targetMatch2D.pose;
			Bounds2f projectedBounds = projectBounds(mvp, targetBounds);

			Eigen::Vector2f source = projectPoint2D(calibs[c].camera, targetMatch2D.pose.translation());
			Eigen::Vector2f end1 = projectPoint2D(calibs[c].camera, targetMatch2D.pose.translation()+uncertaintyRay);
			projectedBounds.extendBy(end1-source); // Extending in both directions

			// Filter observed points by target bounds
			for (int p : *relevantPoints2D[c])
			{
				if (projectedBounds.includes(points2D[c]->at(p)))
					closePoints2D[c].push_back(p);
			}
			LOGC(LTrace, "        Camera %d: Found %d/%d closeby in %.1fx%.1f bounds!\n",
				c, (int)closePoints2D[c].size(), (int)points2D[c]->size(),
				projectedBounds.extends().x()*PixelFactor, projectedBounds.extends().y()*PixelFactor);
		}
	};

	auto reprojectTargetMarkers = [&]()
	{
		matchedSamples = 0;
		for (int c = 0; c < calibs.size(); c++)
		{
			relevantProjected2D[c].clear();
			if (closePoints2D[c].empty()) continue;

			// TODO: Improve selection/projection of markers at a distance
			// Instead of limiting which markers to reproject at a distance
			// May want to project more (especially if pose is uncertain)
			// And instead weight them somehow based on their suspected brightness
			float angleLimit = params.expandMarkerFoV / paramScale[c];

			// Project markers and mark those not seen with NAN
			Eigen::Isometry3f mv = calibs[c].view.cast<float>() * targetMatch2D.pose;
			projected2D[c].resize(target.markers.size());
			relevantProjected2D[c].reserve(target.markers.size());
			for (int i = 0; i < target.markers.size(); i++)
			{
				if (projectMarker(projected2D[c][i], target.markers[i], calibs[c], mv, angleLimit))
					relevantProjected2D[c].push_back(i);
				else
				 	projected2D[c][i].setConstant(NAN);
			}
			LOGC(LDebug, "        Camera %d: Projected %d target markers, matching them to %d closeby observations!\n",
				c, (int)relevantProjected2D[c].size(), (int)closePoints2D[c].size());

			// Remove existing matches that are not visible anymore - happens rarely
			int camera = calibs[c].index;
			auto &cameraMatches = targetMatch2D.points2D[camera];
			for (int p = 0; p < cameraMatches.size();)
			{
				if (projected2D[c][cameraMatches[p].first].hasNaN())
					cameraMatches.erase(cameraMatches.begin()+p);
				else p++;
			}
		}
	};

	auto improveTargetMatch =  [&]() -> TargetMatchError
	{ // Optimise pose to observations
		if (targetMatch2D.count() == 0)
		{
			LOGC(LDebug, "        Cannot optimise pose without any points!\n");
			return {};
		}
		auto errors = optimiseTargetPose<true>(calibs, points2D, targetMatch2D, target, prediction, params.opt, params.filter.point.stdDev, false);
		if (errors.samples > 0)
		{
			reprojectTargetMarkers();
			newMatches = false;
		}

		// Recalculate because of outliers and reprojection changing cameraMatches
		matchedSamples = 0;
		for (int c = 0; c < calibs.size(); c++)
		{
			int camera = calibs[c].index;
			auto &cameraMatches = targetMatch2D.points2D[camera];
			matchedSamples += cameraMatches.size();
		}
		return errors;
	};

	auto canUpdateCameraMatches = [&](int c)
	{
		auto &cameraMatches = targetMatch2D.points2D[calibs[c].index];
		int maxPossible = std::min(closePoints2D[c].size(), relevantProjected2D[c].size());
		if (cameraMatches.size() == maxPossible)
			return false; // If no improvements are possible, skip
		if (maxPossible < params.quality.minCameraObs)
			return false; // If no valid match is possible, skip
		matches.clear();
		return true;
	};

	auto checkDiscardCameraMatches = [&](int c)
	{
		auto &cameraMatches = targetMatch2D.points2D[calibs[c].index];
		if (cameraMatches.size() < params.quality.minCameraObs)
		{ // Even if we did match, it's not sufficient with the current pose anymore
			matchedSamples -= cameraMatches.size();
			cameraMatches.clear();
		}
		return true;
	};

	auto updateCameraMatches = [&](int c, std::vector<std::pair<int, int>> matches)
	{
		int camera = calibs[c].index;
		auto &cameraMatches = targetMatch2D.points2D[camera];
		if (SHOULD_LOGC(LDebug))
		{
			if (cameraMatches.empty())
			{
				if (matches.size() < params.quality.minCameraObs)
					LOGC(LDebug, "            Camera %d: Only found %d matches, discarding!", c, (int)matches.size());
				else
					LOGC(LDebug, "            Camera %d: Found %d matches!", c, (int)matches.size());
			}
			else
			{
				if (matches.size() < params.quality.minCameraObs)
					LOGC(LDebug, "            Camera %d: Only found %d matches, %d before, discarding!", c, (int)matches.size(), (int)cameraMatches.size());
				else if (matches.size() <= cameraMatches.size())
					LOGC(LDebug, "            Camera %d: Still only found %d matches, with %d before!", c, (int)matches.size(), (int)cameraMatches.size());
				else
					LOGC(LDebug, "            Camera %d: Found %d matches, had %d before!", c, (int)matches.size(), (int)cameraMatches.size());
			}
		}
		bool accepted = matches.size() > cameraMatches.size() && matches.size() >= params.quality.minCameraObs;
		if (accepted)
		{
			newMatches = true;
			matchedSamples += matches.size() - cameraMatches.size();
			std::swap(matches, cameraMatches);
		}
		else checkDiscardCameraMatches(c);
		matches.clear();
		return accepted;
	};

	LOGC(LDebug, "    trackTarget2D:");

	reevaluateClosebyPoints();
	reprojectTargetMarkers();

	// Try first with simple matching algorithm
	for (int c = 0; c < calibs.size(); c++)
	{
		if (!canUpdateCameraMatches(c) && checkDiscardCameraMatches(c)) continue;

		// Match relevant points (observation and projected target)
		matchTargetPointsFast(*points2D[c], *properties[c], closePoints2D[c],
			projected2D[c], relevantProjected2D[c], matches,
			internalData.nextMatchingStage(calibs[c].index, targetMatch2D.pose, "Initial fast match"),
			params.matchFast, paramScale[c]);

		updateCameraMatches(c, matches);
	}
	LOGC(LDebug, "        Matched a total of %d samples in initial fast-path!", matchedSamples);

	auto shouldRecoverCameraMatches = [&]()
	{
		int camerasGood = 0, improvableCameras = 0;
		for (int c = 0; c < calibs.size(); c++)
		{
			int camera = calibs[c].index;
			auto &cameraMatches = targetMatch2D.points2D[camera];
			if (cameraMatches.size() >= params.quality.cameraGoodObs && 
				cameraMatches.size()/(float)closePoints2D[c].size() > params.quality.cameraGoodRatio)
				camerasGood++;
			int maxPossible = std::min(closePoints2D[c].size(), relevantProjected2D[c].size());
			int minImprov = std::max<int>(params.quality.minImprovePoints, std::ceil(cameraMatches.size() * params.quality.minImproveFactor));
			if (cameraMatches.size() < params.quality.cameraGoodObs && minImprov <= maxPossible)
				improvableCameras++;
		}
		bool recover = improvableCameras > 0 && camerasGood < params.selectRecover.minCamerasGood;
		if (recover)
			LOGC(LDebug, "        Only %d cameras had a good amount of samples with %d total, entering slow-path!", camerasGood, matchedSamples);
		return recover;
	};
	auto shouldRecoverCamera = [&](int c)
	{
		int camera = calibs[c].index;
		auto &cameraMatches = targetMatch2D.points2D[camera];
		int maxPossible = std::min(closePoints2D[c].size(), relevantProjected2D[c].size());
		int minImprov = std::max<int>(params.quality.minImprovePoints, std::ceil(cameraMatches.size() * params.quality.minImproveFactor));
		return cameraMatches.size() < params.quality.cameraGoodObs && minImprov <= maxPossible;
	};
	auto continueRecoveringCameras = [&]()
	{
		int camerasGood = 0;
		for (int c = 0; c < calibs.size(); c++)
		{
			int camera = calibs[c].index;
			auto &cameraMatches = targetMatch2D.points2D[camera];
			if (cameraMatches.size() >= params.quality.cameraGoodObs && 
				cameraMatches.size()/(float)closePoints2D[c].size() > params.quality.cameraGoodRatio)
				camerasGood++;
		}
		return camerasGood < params.selectRecover.minCamerasGood;
	};
	if (shouldRecoverCameraMatches())
	{ // Try again with recover matching algorithm
		for (int i = 0; i < params.selectRecover.maxRecoverStages; i++)
		{
			for (int c = 0; c < calibs.size(); c++)
			{
				if (!shouldRecoverCamera(c) && checkDiscardCameraMatches(c)) continue;

				// Match relevant points (observation and projected target)
				matchTargetPointsRecover(*points2D[c], *properties[c], closePoints2D[c],
					projected2D[c], relevantProjected2D[c], matches,
					internalData.nextMatchingStage(calibs[c].index, targetMatch2D.pose, "Recover match"),
					params.matchRecover, paramScale[c]);

				updateCameraMatches(c, matches);
			}
			LOGC(LDebug, "        Matched a total of %d samples in matching slow-path %d!", matchedSamples, i);

			if (!continueRecoveringCameras()) break;
			if (!newMatches) break;
			if (improveTargetMatch().samples == 0)
				break; // If we cannot imrpove, continue
		}

		LOGC(LDebug, "        Continuing normally after slow-path finished!");
	}

	DualExtremum<int> camPoints(0);
	for (int c = 0; c < calibs.size(); c++)
		camPoints.max(targetMatch2D.points2D[calibs[c].index].size());
	auto shouldHandleSingleCameraMatches = [&](const DualExtremum<int> &camPoints)
	{
		int camerasGood = 0;
		for (int c = 0; c < calibs.size(); c++)
		{
			int camera = calibs[c].index;
			auto &cameraMatches = targetMatch2D.points2D[camera];
			if (cameraMatches.size() >= params.quality.cameraGoodObs && 
				cameraMatches.size()/(float)closePoints2D[c].size() > params.quality.cameraGoodRatio)
				camerasGood++;
		}
		// If one camera is either the only matched one or dominating over others
		// as long as the others aren't good themselves, at which point there's no worry over uncertainty
		return camPoints.rank[0] > 0 && camerasGood < params.selectUncertain.maxCamerasGood &&
			camPoints.rank[0] > camPoints.rank[1]*params.selectUncertain.maxDominantFactor;
	};
	if (shouldHandleSingleCameraMatches(camPoints))
	{ // If a single camera is dominant, allow other cameras to shift along its uncertainty axis
		// The existing pose is very uncertain along the axis through the dominant camera

		// Find dominant camera
		int dominantCamera = -1;
		for (int c = 0; c < calibs.size(); c++)
		{
			if (targetMatch2D.points2D[calibs[c].index].size() == camPoints.rank[0])
				dominantCamera = c;
			//else // No a good idea in general
			// 	targetMatch2D.points2D[c].clear();
		}
		assert(dominantCamera >= 0);

		LOGC(LDebug, "        Camera %d is dominant with %d out of %d total samples (next best %d)! Entering uncertainty compensation path!",
			dominantCamera, camPoints.rank[0], matchedSamples, camPoints.rank[1]);
		Breakpoint(2);

		improveTargetMatch();

		// Find axis of most uncertainty
		Ray3f uncertainAxis;
		uncertainAxis.pos = calibs[dominantCamera].transform.translation().cast<float>();
		uncertainAxis.dir = (targetMatch2D.pose.translation() - uncertainAxis.pos).normalized();

		float uncertaintyDistance = 0.5f; // How uncertain along either direction of the axis
		reevaluateClosebyPoints(uncertainAxis.dir * uncertaintyDistance);

		// Find more visible target points (which would be newly visible in this frame)
		for (int c = 0; c < calibs.size(); c++)
		{
			if (dominantCamera == c) continue;
			if (!canUpdateCameraMatches(c) && checkDiscardCameraMatches(c)) continue;
			int camera = calibs[c].index;

			// Try to shift along uncertain axis to align the prediction with the observations of this camera
			int obsConsidered = shiftPointsAlongAxis(
				*points2D[c], closePoints2D[c], projected2D[c], relevantProjected2D[c],
				calibs[c], uncertainAxis, targetMatch2D.pose.translation(), params.matchUncertain.perpDeviation,
				internalData.uncertaintyAxis[camera]);
			LOGC(LDebug, "            Considering %d/%d points of camera %d to match with %d projected points along uncertainty axis!",
				obsConsidered, (int)closePoints2D[c].size(), c, (int)relevantProjected2D[c].size());
			if (obsConsidered == 0 && checkDiscardCameraMatches(c))
				continue;

			// Try to match points quickly along the uncertainty axis
			float bestShift = matchTargetPointsAlongAxis(*points2D[c], *properties[c], closePoints2D[c],
				projected2D[c], relevantProjected2D[c], matches,
				internalData.nextMatchingStage(camera, targetMatch2D.pose, "Corrective align match"),
				params.matchUncertain, paramScale[c],
				internalData.uncertaintyAxis[camera]);

			updateCameraMatches(c, matches);
		}

		// Probably not necessary
		//reevaluateClosebyPoints();

		if (!newMatches)
			LOGC(LDebug, "        Failed to find any more samples from other cameras to break dominance! Aborting!");
	}

	improveTargetMatch();

	LOGC(LDebug, "        Searching for final additions:");

	// Find some more points if possible
	for (int c = 0; c < calibs.size(); c++)
	{
		if (!canUpdateCameraMatches(c) && checkDiscardCameraMatches(c)) continue;

		matchTargetPointsFast(*points2D[c], *properties[c], closePoints2D[c],
			projected2D[c], relevantProjected2D[c], matches,
			internalData.nextMatchingStage(calibs[c].index, targetMatch2D.pose, "Final fast match"),
			params.matchFastFinal, paramScale[c]);

		updateCameraMatches(c, matches);
	}

	//if (!nothingNew)
	{ // Final optimisation
		auto errors = optimiseTargetPose<true>(calibs, points2D, targetMatch2D, target, prediction, params.opt, params.filter.point.stdDev, true);
	}
	// Update covariance (done in optimiseTargetMatch with !quick)
	//evaluateTargetPoseCovariance(calibs, points2D, targetMatch2D, target, params.filter.point.stdDev);
	{
		auto stdDev = targetMatch2D.covariance.diagonal().cwiseSqrt();
		Eigen::Vector3f devPos = stdDev.head<3>()*1000, devRot = stdDev.tail<3>()*1000;
		LOGC(LDebug, "            Has stdDev of (%.2f,%.2f,%.2f)mm (%.2f,%.2f,%.2f) rotation!",
			devPos.x(), devPos.y(), devPos.z(), devRot.x(), devRot.y(), devRot.z());
	}

	// Record how many points where involved for judging end result
	for (int c = 0; c < calibs.size(); c++)
	{
		int matched = targetMatch2D.points2D[calibs[c].index].size();
		internalData.freeProjections += std::max<int>(0, relevantProjected2D[c].size()-matched);

		if (closePoints2D[c].empty()) continue;

		// Project target bounds and relevant target points into camera view
		Eigen::Projective3f mvp = calibs[c].camera.cast<float>() * targetMatch2D.pose;
		Bounds2f bounds = projectBounds(mvp, target.bounds);
		auto center = bounds.center();
		bounds.min = center + (bounds.min-center) * params.mistrust.closebyObservationsRange;
		bounds.max = center + (bounds.max-center) * params.mistrust.closebyObservationsRange;

		// Filter observed points by target bounds
		int closeObservations = 0;
		for (int p : closePoints2D[c])
		{
			if (bounds.includes(points2D[c]->at(p)))
				closeObservations++;
		}
		internalData.freeObservations += std::max<int>(0, closeObservations-matched);
	}

	return targetMatch2D;
}