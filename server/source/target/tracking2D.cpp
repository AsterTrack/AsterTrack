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
 * Analyze the points and returns a set of potential shifts along the axis to align them
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

template<typename T>
static constexpr inline T pown(T x, unsigned p)
{
	T result = 1;
	while (p)
	{
		if (p&1) result *= x;
		x *= x;
		p >>= 1;
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

	float matchRadiusSq = params.matchRadius*params.matchRadius*distFactor*distFactor;
	float includeRadiusSq = matchRadiusSq*params.match.primAdvantage*params.match.primAdvantage;
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
	if (numMatches <= 1)
	{ // No hope if even relaxed constraints resulted in 0
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
		if (match.valid() && match.value < matchRadiusSq)
		{ // TODO: Technically match.value also accounts for size, so not entirely accurate, but fine
			LOGC(LTrace, "                   Accepted match mk%d / pt%d with distance %.4f\n",
				relevantProjected2D[cand.context], match.index, match.value*PixelFactor);
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
			LOGC(LTrace, "                   Discarded match mk%d with distance %.4f\n",
				relevantProjected2D[cand.context], match.value*PixelFactor);
		}
	}
	LOGC(LDebug, "               Matched %d points!\n", (int)matches.size());
	return true;
}

/**
 * Matches a relevant set of projected target points to a relevant set of point observations
 */
void matchTargetPointsSlow(
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
	{ // No hope if even relaxed constraints resulted in 0
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
	LOG(LTrackingMatch, LDebug, "                Offset along ncertainty axis from %.1fpx to %.1fpx!",
		shiftMin*PixelFactor, shiftMax*PixelFactor);

	// Try multiple shift candidates along uncertainty axis
	float bestShift = NAN;
	std::vector<std::pair<int,int>> tempMatches;
	TargetMatchingData tempMatchData;
	float shiftStep = std::min(params.stepLength, (shiftMax-shiftMin) / params.maxSteps);
	for (float shift = shiftMin; shift < shiftMax; shift += shiftStep)
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
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, Eigen::Isometry3f prediction,
	TargetOptimisationParameters params)
{
	ScopedLogCategory scopedLogCategory(LTrackingOpt);

	// Initialise optimisation error term, preparing the data
	constexpr int OPTIONS = OptUndistorted | (OUTLIER? OptOutliers : 0) | (REFERENCE? OptReferencePose : 0);
	typedef TargetReprojectionError<double, OPTIONS> TgtError;
	TgtError errorTerm(calibs);
	if constexpr (REFERENCE)
		errorTerm.setReferencePose(prediction.cast<double>(), params.predictionInfluence);
	errorTerm.setData(points2D, match);
	if (errorTerm.values() < errorTerm.inputs() || errorTerm.m_observedPoints.empty())
	{
		LOGC(LError, "Got only %d points to optimise %d parameters!\n", errorTerm.values(), errorTerm.inputs());
		return { 10000.0f, 0.0f, 10000.0f, 0 };
	}
	LOGC(LDebug, "        Optimising target pose using %d point matches!\n", (int)errorTerm.m_observedPoints.size());

	// On-the-fly outlier detection
	if constexpr (OUTLIER)
		errorTerm.m_outlierMap.resize(errorTerm.m_observedPoints.size());

	// Create initial parameter vector for optimisation
	Eigen::VectorXd poseVec;
	if constexpr (OPTIONS & OptCorrectivePose)
		poseVec = errorTerm.encodePose(Eigen::Isometry3d::Identity());
	else
		poseVec = errorTerm.encodePose(match.pose.cast<double>());

	// Initialise optimisation algorithm
	Eigen::LevenbergMarquardt<TgtError, double> lm(errorTerm);
	lm.parameters.maxfev = params.maxIterations*10; // Max number of evaluations of errorTerm, bounds for number of iterations
	lm.parameters.xtol = 0.0000001f * PixelSize * params.tolerances;
	lm.parameters.ftol = 0.00001f * params.tolerances;
	lm.parameters.gtol = 0.00001f * params.tolerances;
	Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeInit(poseVec);
	if (status == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
	{
		LOGC(LError, "Improper optimisation input parameters!\n");
		return { 10000.0f, 0.0f, 10000.0f, 0 };
	}

	// Optimisation steps
	int it = 0;
	int outlierCount = 0;
	match.error.samples = errorTerm.m_observedPoints.size();
	while (++it <= params.maxIterations)
	{
		status = lm.minimizeOneStep(poseVec);
		auto errors = lm.fvec.head(errorTerm.m_observedPoints.size());
		match.error.mean = errors.sum() / match.error.samples;
		match.error.stdDev = std::sqrt((errors.array() - match.error.mean).square().sum() / match.error.samples);
		match.error.max = errors.maxCoeff();
		LOGC(LTrace, "            Current RMSE: %.4f (sum %f, inliers %d)\n", match.error.mean*PixelFactor, errors.sum(), match.error.samples);
		if (status != Eigen::LevenbergMarquardtSpace::Running)
		{
			LOGC(LDebug, "          Stopped optimisation: %s\n", getStopCodeText(status));
			break;
		}
		if constexpr (!OUTLIER) continue;
		// Find outliers
		float maxAllowed = match.error.mean + std::max(params.outlierSigma*match.error.stdDev, params.outlierVarMin);
		for (int i = 0; i < errorTerm.m_observedPoints.size(); i++)
		{
			int cam = std::get<0>(errorTerm.m_observedPoints[i]);
			// TODO: Per camera outlier limit, definitely need to treat dominant cameras differently than fringe cameras
			if (lm.fvec(i) <= maxAllowed) continue;
			match.error.samples--;
			outlierCount++;
			errorTerm.m_outlierMap[i] = true;

			// Rest is just for debug
			int marker = -1, pt = -1;
			const auto &pointsMatch = match.points2D[calibs[cam].index];
			for (int p = 0; p < pointsMatch.size(); p++)
			{
				if (points2D[cam]->at(pointsMatch[p].second) == std::get<2>(errorTerm.m_observedPoints[i]).template cast<float>())
				{
					marker = pointsMatch[p].first;
					pt = pointsMatch[p].second;
					break;
				}
			}
			LOGC(LDebug, "            Found outlier (cam %d, marker %d, pt %d) with error %.4f! %d inliers left, with %.4f max\n",
				cam, marker, pt,
				lm.fvec(i)*PixelFactor, match.error.samples, maxAllowed*PixelFactor);
		}
		if (errorTerm.values()-outlierCount < errorTerm.inputs())
			break;
	}
	LOGC(LTrace, "          Finished with it %d / %d\n", it, params.maxIterations);

	// Apply optimised pose
	if constexpr (OPTIONS & OptCorrectivePose)
		match.pose = match.pose * errorTerm.decodePose(poseVec).template cast<float>();
	else
		match.pose = errorTerm.decodePose(poseVec).template cast<float>();

	if constexpr (OUTLIER)
	{ // Remove detected outliers from target match
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
					LOGC(LTrace, "            Camera %d Marker %d - Point %d, outlier!\n", calib.id, pointsMatch[i].first, pointsMatch[i].second);
					outliersCam++;
				}
				else
				{
					LOGC(LTrace, "            Camera %d Marker %d - Point %d, remaining pixel error %.4f\n",
						calib.id, pointsMatch[i].first, pointsMatch[i].second, lm.fvec(errorIndex)*PixelFactor);
					pointsMatch[writeIndex++] = pointsMatch[i];
				}
				errorIndex++;
			}
			pointsMatch.resize(pointsMatch.size()-outliersCam);
		}
	}

	return match.error;
}

template TargetMatchError optimiseTargetPose<true, true>(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, Eigen::Isometry3f prediction,
	TargetOptimisationParameters params);
template TargetMatchError optimiseTargetPose<false, true>(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match, Eigen::Isometry3f prediction,
	TargetOptimisationParameters params);

/**
 * Evaluate the given target match and update its pose error
 */
TargetMatchError evaluateTargetPose(const std::vector<CameraCalib> &calibs,
	const std::vector<std::vector<Eigen::Vector2f> const *> points2D, TargetMatch2D &match)
{
	TargetReprojectionError<float, OptUndistorted> errorTerm(calibs);
	errorTerm.setData(points2D, match);
	if (errorTerm.values() == 0)
		return { 0, 0, 0, 0 };
	Eigen::VectorXf errors(errorTerm.values());
	errorTerm.calculateSampleErrors(match.pose, errors);
	match.error.samples = errorTerm.values();
	match.error.mean = errors.sum() / errorTerm.values();
	match.error.stdDev = std::sqrt((errors.array() - match.error.mean).square().sum() / errorTerm.values());
	match.error.max = errors.maxCoeff();
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
 * First re-acquires the previously visible markers, finds a better pose with them,
 * then finds newly appearing markers and optimises the pose to fit the observations
 */
TargetMatch2D trackTarget2D(const TargetCalibration3D &target, Eigen::Isometry3f prediction, Eigen::Vector3f stdDev,
	const std::vector<CameraCalib> &calibs, int cameraCount,
	const std::vector<std::vector<Eigen::Vector2f> const *> &points2D,
	const std::vector<std::vector<BlobProperty> const *> &properties,
	const std::vector<std::vector<int> const *> &relevantPoints2D,
	const TargetTrackingParameters &params, TargetTracking2DData &internalData)
{
	ScopedLogCategory scopedLogCategory(LTracking);

	// Find candidate for 2D point matches
	TargetMatch2D targetMatch2D = {};
	targetMatch2D.calib = &target;
	targetMatch2D.pose = prediction;
	targetMatch2D.points2D.resize(cameraCount); // Indexed with calib.index, not index into calibs
	int relevantCams = points2D.size();

	// Determine 3D bounds to look in
	Bounds3f targetBounds = target.bounds;
	Eigen::Vector3f uncertainty = Eigen::Vector3f::Constant(params.minUncertainty3D).cwiseMax(stdDev*params.uncertaintySigma);
	targetBounds.extendBy(targetMatch2D.pose.rotation().transpose().cwiseAbs() * uncertainty);

	// Reused allocation of target point reprojections
	thread_local std::vector<std::vector<Eigen::Vector2f>> projected2D;
	thread_local std::vector<std::vector<int>> relevantProjected2D;
	if (projected2D.size() < relevantCams)
		projected2D.resize(relevantCams);
	if (relevantProjected2D.size() < relevantCams)
		relevantProjected2D.resize(relevantCams);

	int maxComplexStages = 2;
	int maxTotalStages = 1+maxComplexStages+1+1;
	int matchingStage = 0;

	// Clear internal data for visualisation purposes (low overhead)
	internalData.init(cameraCount);
	for (int c = 0; c < calibs.size(); c++)
	{
		auto &matchingData = internalData.matching.at(calibs[c].index);
		for (auto &matchData : matchingData)
			matchData.clear();
		if (matchingData.size() < maxTotalStages) matchingData.resize(maxTotalStages);
	}

	// Find initial set of closeby points to consider
	// TODO: Use nearby cluster(s) from all cameras passed into this function
	std::vector<std::vector<int>> closePoints2D(calibs.size());
	for (int c = 0; c < calibs.size(); c++)
	{
		const CameraCalib &calib = calibs[c];

		// Project target bounds and relevant target points into camera view
		Eigen::Projective3f mvp = calib.camera.cast<float>() * targetMatch2D.pose;
		Bounds2f projectedBounds = projectBounds(mvp, targetBounds);
		projectedBounds.extendBy({ params.addUncertaintyPx, params.addUncertaintyPx });

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


	LOGC(LDebug, "    trackTarget2D:");

	auto reprojectTargetMarkers = [&]()
	{
		for (int c = 0; c < calibs.size(); c++)
		{
			projected2D[c].clear();
			relevantProjected2D[c].clear();
			if (closePoints2D[c].empty()) continue;
			projectTarget(projected2D[c], relevantProjected2D[c],
				target, calibs[c], targetMatch2D.pose, params.expandMarkerFoV);
			LOGC(LDebug, "        Camera %d: Projected %d target markers, matching them to %d closeby observations!\n",
				c, (int)relevantProjected2D[c].size(), (int)closePoints2D[c].size());
		}
	};
	auto optimiseTargetMatch =  [&](bool quick)
	{ // Optimise pose to observations
		// TODO: Provide a quick option
		TargetMatchError prevErrors = evaluateTargetPose(calibs, points2D, targetMatch2D);
		TargetMatchError newErrors = optimiseTargetPose<true>(calibs, points2D, targetMatch2D, prediction, params.opt);
		if (newErrors.samples == 0)
		{
			LOGC(LDarn, "        Failed to optimise pose of %d points!\n", prevErrors.samples);
			return;
		}
		LOGC(LDebug, "        Reduced average pixel error of %d points from %.4fpx to %d inliers with %.4fpx error\n",
			prevErrors.samples, prevErrors.mean * PixelFactor, newErrors.samples, newErrors.mean * PixelFactor);
	};

	reprojectTargetMarkers();

	int camerasGood = 0;
	int matchedSamples = 0;
	const int minCamerasGood = 2;

	// Try first with simple matching algorithm
	for (int c = 0; c < calibs.size(); c++)
	{
		if (relevantProjected2D[c].empty() || closePoints2D[c].empty()) continue;
		const CameraCalib &calib = calibs[c];
		auto &matchingData = internalData.matching.at(calib.index);
		auto &cameraMatches = targetMatch2D.points2D[calib.index];

		// Normalise pixel parameters to a fixed distance
		float distFactor = params.normaliseDistance / (prediction.translation() - calib.transform.translation().cast<float>()).norm();

		// Internal data for stage
		auto &matchData = matchingData[matchingStage];
		matchData.pose = targetMatch2D.pose;

		// Match relevant points (observation and projected target)
		matchTargetPointsFast(*points2D[c], *properties[c], closePoints2D[c],
			projected2D[c], relevantProjected2D[c], cameraMatches, matchData,
			params.matchFast, distFactor);

		if (cameraMatches.size() < params.minCameraObs)
		{
			LOGC(LDebug, "            Camera %d: Found %d matches, discarded all!\n", c, (int)cameraMatches.size());
			cameraMatches.clear();
		}
		else
		{
			LOGC(LDebug, "            Camera %d: Found %d matches!\n", c, (int)cameraMatches.size());
			matchedSamples += cameraMatches.size();
			if (cameraMatches.size() >= params.cameraGoodObs && (float)cameraMatches.size()/closePoints2D[c].size() > params.cameraGoodRatio)
				camerasGood++;
		}
	}
	matchingStage++;
	LOGC(LDebug, "        Matched a total of %d samples in initial fast-path!", matchedSamples);

	if (camerasGood < minCamerasGood || matchedSamples < params.minTotalObs)
	{
		LOGC(LDebug, "        Only %d cameras had a good amount of samples with %d total, entering slow-path!", camerasGood, matchedSamples);
		for (int i = 0; i < maxComplexStages; i++)
		{ // Try again with complex matching algorithm

			camerasGood = 0;
			matchedSamples = 0;
			bool nothingNew = true;

			for (int c = 0; c < calibs.size(); c++)
			{
				const CameraCalib &calib = calibs[c];
				auto &cameraMatches = targetMatch2D.points2D[calib.index];
				if (cameraMatches.size() == closePoints2D[c].size())
				{ // If no observations left in bounds, skip
					matchedSamples += cameraMatches.size();
					continue;
				}
				int prevPointCount = cameraMatches.size();
				cameraMatches.clear();
				if (relevantProjected2D[c].empty() || closePoints2D[c].empty())
					continue; // Even if we did match, it's not visible with the current pose anymore
				auto &matchingData = internalData.matching.at(calib.index);

				// Normalise pixel parameters to a fixed distance
				float distFactor = params.normaliseDistance / (prediction.translation() - calib.transform.translation().cast<float>()).norm();

				// Internal data for stage
				auto &matchData = matchingData[matchingStage];
				matchData.pose = targetMatch2D.pose;

				// Match relevant points (observation and projected target)
				matchTargetPointsSlow(*points2D[c], *properties[c], closePoints2D[c],
					projected2D[c], relevantProjected2D[c], cameraMatches, matchData,
					params.matchSlow, distFactor);

				if (cameraMatches.size() < params.minCameraObs)
				{
					LOGC(LDebug, "            Camera %d: Found %d matches, discarded all!\n", c, (int)cameraMatches.size());
					cameraMatches.clear();
				}
				else
				{
					LOGC(LDebug, "            Camera %d: Found %d matches!\n", c, (int)cameraMatches.size());
					matchedSamples += cameraMatches.size();
					if (cameraMatches.size() >= params.cameraGoodObs && (float)cameraMatches.size()/closePoints2D[c].size() > params.cameraGoodRatio)
						camerasGood++;
					if (prevPointCount < cameraMatches.size())
						nothingNew = false;
				}
			}
			matchingStage++;

			LOGC(LDebug, "        Matched a total of %d samples in matching slow-path %d!",
				matchedSamples, i);

			if (camerasGood >= minCamerasGood) break; // Continue normally
			if (nothingNew) return targetMatch2D; // No hope improving
			if (i+1 >= maxComplexStages)
			{ // Last iterations, don't optimise again
				if (matchedSamples >= params.minTotalObs)
					break; // Some new data after last try, continue normally
				return targetMatch2D; // Else, not enough data still
			}
			// Else, with some new data, try again after quick optimisation

			optimiseTargetMatch(true);
			reprojectTargetMarkers();
		}

		LOGC(LDebug, "        Continuing normally after slow-path finished!");
	}
	assert(camerasGood >= minCamerasGood || matchedSamples >= params.minTotalObs);

	if (camerasGood >= minCamerasGood)
	{ // If a single camera is dominant, allow other cameras to shift along its uncertainty axis
		DualExtremum<int> camPoints(0);
		for (int c = 0; c < calibs.size(); c++)
			camPoints.max(targetMatch2D.points2D[c].size());
		if (camPoints.rank[0] > camPoints.rank[1]*params.matchUncertain.maxDominantFactor)
		{ // The existing pose is very uncertain along the axis through the dominant camera

			// Find dominant camera
			int dominantCamera = -1;
			for (int c = 0; c < calibs.size(); c++)
				if (targetMatch2D.points2D[c].size() == camPoints.rank[0])
					dominantCamera = c;
			assert(dominantCamera >= 0);

			LOGC(LDebug, "        Camera %d is dominant with %d out of %d total samples (next best %d)! Entering uncertainty compensation path!",
				dominantCamera, camPoints.rank[0], matchedSamples, camPoints.rank[1]);
			Breakpoint(2);

			optimiseTargetMatch(true);
			reprojectTargetMarkers();

			// Find axis of most uncertainty
			Ray3f uncertainAxis;
			uncertainAxis.pos = calibs[dominantCamera].transform.translation().cast<float>();
			uncertainAxis.dir = (targetMatch2D.pose.translation() - uncertainAxis.pos).normalized();

			// Find more visible target points (which would be newly visible in this frame)
			bool nothingNew = true;
			matchedSamples = targetMatch2D.points2D[calibs[dominantCamera].index].size();
			for (int c = 0; c < calibs.size(); c++)
			{
				if (dominantCamera == c) continue;
				const CameraCalib &calib = calibs[c];
				auto &cameraMatches = targetMatch2D.points2D[calib.index];
				if (cameraMatches.size() == closePoints2D[c].size())
				{ // If no observations left in bounds, skip
					matchedSamples += cameraMatches.size();
					continue;
				}
				int prevPointCount = cameraMatches.size();
				cameraMatches.clear();
				if (relevantProjected2D[c].empty() || closePoints2D[c].empty())
					continue; // Even if we did match, it's not visible with the current pose anymore
				auto &matchingData = internalData.matching.at(calib.index);

				// Try to shift along uncertain axis to align the prediction with the observations of this camera
				int obsConsidered = shiftPointsAlongAxis(
					*points2D[c], closePoints2D[c], projected2D[c], relevantProjected2D[c],
					calibs[c], uncertainAxis, targetMatch2D.pose.translation(), params.matchUncertain.perpDeviation,
					internalData.uncertaintyAxis[calib.index]);
				LOGC(LDebug, "            Considering %d/%d points of camera %d to match with %d projected points along uncertainty axis!",
					obsConsidered, (int)closePoints2D[c].size(), c, (int)relevantProjected2D[c].size());
				if (obsConsidered == 0)
					continue;

				// Normalise pixel parameters to a fixed distance
				float distFactor = params.normaliseDistance / (prediction.translation() - calib.transform.translation().cast<float>()).norm();

				// Internal data for stage
				auto &matchData = matchingData[matchingStage];
				matchData.pose = targetMatch2D.pose;

				// Try to match points quickly along the uncertainty axis
				float bestShift = matchTargetPointsAlongAxis(*points2D[c], *properties[c], closePoints2D[c],
					projected2D[c], relevantProjected2D[c], cameraMatches, matchData,
					params.matchUncertain, distFactor,
					internalData.uncertaintyAxis[calib.index]);

				if (cameraMatches.size() < params.minCameraObs)
				{
					LOGC(LDebug, "            Camera %d: Found %d matches along uncertainty axis, discarded all!\n", c, (int)cameraMatches.size());
					cameraMatches.clear();
				}
				else
				{
					LOGC(LDebug, "            Camera %d: Found %d matches in %d points shifted %.1fpx along uncertainty axis!",
						c, (int)cameraMatches.size(), obsConsidered, bestShift*PixelFactor);
					matchedSamples += cameraMatches.size();
					if (prevPointCount < cameraMatches.size())
						nothingNew = false;
				}
			}
			matchingStage++;

			if (nothingNew)
			{
				LOGC(LDebug, "        Failed to find any more samples from other cameras to break dominance! Aborting!");
				return targetMatch2D; // No hope improving
			}
			// Else, with some new data, continue normally
		}
	}

	optimiseTargetMatch(true);
	reprojectTargetMarkers();

	LOGC(LDebug, "        Searching for final additions:");

	// Find some more points if possible
	matchedSamples = 0;
	bool nothingNew = true;
	for (int c = 0; c < calibs.size(); c++)
	{
		const CameraCalib &calib = calibs[c];
		auto &cameraMatches = targetMatch2D.points2D[calib.index];
		if (cameraMatches.size() == closePoints2D[c].size())
		{ // If no observations left in bounds, skip
			matchedSamples += cameraMatches.size();
			continue;
		}
		int prevPointCount = cameraMatches.size();
		cameraMatches.clear();
		if (relevantProjected2D[c].empty() || closePoints2D[c].empty())
			continue; // Even if we did match, it's not visible with the current pose anymore
		auto &matchingData = internalData.matching.at(calib.index);

		// Normalise pixel parameters to a fixed distance
		float distFactor = params.normaliseDistance / (prediction.translation() - calib.transform.translation().cast<float>()).norm();

		// Internal data for stage
		auto &matchData = matchingData[matchingStage];
		matchData.pose = targetMatch2D.pose;

		// Match relevant points (observation and projected target)
		if (prevPointCount < params.cameraGoodObs)
		{
			matchTargetPointsSlow(*points2D[c], *properties[c], closePoints2D[c],
				projected2D[c], relevantProjected2D[c], cameraMatches, matchData,
				params.matchSlowSecond, distFactor);
		}
		else
		{ // TODO: Only match points not already matched
			matchTargetPointsFast(*points2D[c], *properties[c], closePoints2D[c],
				projected2D[c], relevantProjected2D[c], cameraMatches, matchData,
				params.matchFast, distFactor);
		}

		if (cameraMatches.size() < params.minCameraObs)
		{
			LOGC(LDebug, "            Camera %d: Found %d matches, discarded all!\n", c, (int)cameraMatches.size());
			cameraMatches.clear();
		}
		else
		{
			LOGC(LDebug, "            Camera %d: Found %d matches!\n", c, (int)cameraMatches.size());
			matchedSamples += cameraMatches.size();
			if (prevPointCount < cameraMatches.size())
				nothingNew = false;
		}
	}

	//if (!nothingNew)
	{ // Final optimisation
		optimiseTargetMatch(false);
	}

	return targetMatch2D;
}