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

#include "assembly.hpp"

#include "target/tracking2D.hpp"
#include "point/sequence_data.inl"

#include "unsupported/Eigen/NonLinearOptimization"

#include "util/matching.hpp"
#include "util/eigenalg.hpp"
#include "util/log.hpp"

#include <cassert>
#include <numeric>
#include <random>


/**
 * Calibration of target structure
 */

// Merge pairwise close markers
int mergeCloseMarkers(std::vector<Eigen::Vector3f> &markers, std::map<int,int> &markerMap, std::vector<int> *markerSamples, float mergeMM, int maxMerges)
{
	LOGL(LTargetCalib, "Analyzing close markers for equality among %d markers!\n", (int)markers.size());
	std::multimap<float,std::pair<int,int>> closeMatches;
	for (int i = 0; i < markers.size(); i++)
	{
		for (int j = i+1; j < markers.size(); j++)
		{
			float dist = (markers[i] - markers[j]).squaredNorm();
			if (dist*1000*1000 < mergeMM*2*mergeMM*2)
			{
				LOGL(LTargetCalib, "    Close Match between marker points %d and %d: %fmm\n", i, j, std::sqrt(dist*1000*1000));
				closeMatches.insert({ dist, std::make_pair(i,j) });
			}
		}
	}

	// Get recursive map to new merged markers
	std::vector<int> mergers(markers.size());
	for (int i = 0; i < markers.size(); i++)
		mergers[i] = i;
	int merges = 0;
	for (auto &close : closeMatches)
	{
		if (close.first*1000*1000 >= mergeMM*mergeMM)
			continue;
		mergers[close.second.second] = close.second.first;
		merges++;
		if (maxMerges > 0 && merges >= maxMerges)
			break;
	}
	// Map remaining markers to new indices
	int markerCnt = 0;
	std::vector<int> indexMap(markers.size(), -1);
	for (int m = 0; m < markers.size(); m++)
		if (mergers[m] == m)
			indexMap[m] = markerCnt++;

	// Merge markers
	std::vector<int> weightBackend(markers.size(), 1);
	if (!markerSamples)
		markerSamples = &weightBackend;
	for (int m = 0; m < markers.size(); m++)
	{
		// Flatten mergers
		while (mergers[m] != mergers[mergers[m]])
			mergers[m] = mergers[mergers[m]];
		assert(m >= mergers[m]);
		// Merge marker with new index
		int i = indexMap[mergers[m]];
		assert(i >= 0);
		assert(i <= m);
		if (m == mergers[m])
		{
			markers[i] = markers[m];
			markerSamples->at(i) = markerSamples->at(m);
			continue;
		}
		LOG(LTargetCalib, LTrace, "    Markers %d and %d merged with samples %d and %d", i, m, markerSamples->at(i), markerSamples->at(m));
		auto prevI = markers[i], prevM = markers[m];
		markers[i] = (markers[i]*markerSamples->at(i) + markers[m]*markerSamples->at(m)) * (1.0f/(markerSamples->at(i)+markerSamples->at(m)));
		markerSamples->at(i) += markerSamples->at(m);
		LOG(LTargetCalib, LTrace, "    -> Distances %fmm and %fmm", (markers[i]-prevI).norm()*1000.0f, (markers[i]-prevM).norm()*1000.0f);
	}

	// Apply map to markerMap
	for (auto &map : markerMap)
		map.second = indexMap[mergers[map.second]];
	int merged = markers.size() - markerCnt;
	markers.resize(markerCnt);
	return merged;
};

// Check per-frame and per-marker errors of current Target View Calibration
bool determineTargetOutliers(const std::vector<CameraCalib> &calibs, ObsTarget &target, TargetOutlierErrors maxErrors, const TargetAquisitionParameters &params)
{
	if (target.frames.empty())
		return false;

	int oldSampleCnt = target.totalSamples, oldOutlierCnt = target.outlierSamples,
		oldFrameCnt = target.frames.size(), oldMarkerCnt = target.markers.size(), oldSequenceCnt = target.markerMap.size();
	auto oldErrors = getTargetErrorDist(calibs, target);

	int sampleOutliers = 0, frameOutliers = 0, markerOutliers = 0;
	std::vector<float> markerError(target.markers.size());
	std::vector<int> markerSamples(target.markers.size());

	std::vector<float> obsErrors;
	for (auto frameIt = target.frames.begin(); frameIt != target.frames.end();)
	{
		if (frameIt->samples.size() < params.minFrameObsCount)
		{
			frameOutliers++;
			frameIt = target.frames.erase(frameIt);
			continue;
		}

		// 2D reprojection error
		obsErrors.clear();
		obsErrors.reserve(frameIt->samples.size());
		float diff2D = 0;
		for (auto sampleIt = frameIt->samples.begin(); sampleIt != frameIt->samples.end();)
		{
			int m = target.markerMap.at(sampleIt->marker);
			Eigen::Vector3f pt3D = frameIt->pose * target.markers[m];
			Eigen::Vector2f proj = (calibs[sampleIt->camera].camera.cast<float>() * pt3D.homogeneous()).hnormalized().head<2>();
			Eigen::Vector2f pt2D = undistortPoint(calibs[sampleIt->camera], sampleIt->point);
			float diff = (proj-pt2D).norm();
			if (diff > maxErrors.sample)
			{
				sampleOutliers++;
				sampleIt = frameIt->samples.erase(sampleIt);
			}
			else
			{
				diff2D += diff;
				obsErrors.push_back(diff);
				sampleIt++;
			}
		}
		diff2D = diff2D/frameIt->samples.size(); // may be NAN
		if (frameIt->samples.size() < params.minFrameObsCount || diff2D > maxErrors.frame)
		{
			frameOutliers++;
			frameIt = target.frames.erase(frameIt);
		}
		else
		{
			for (int i = 0; i < frameIt->samples.size(); i++)
			{
				int m = target.markerMap.at(frameIt->samples[i].marker);
				markerError[m] += obsErrors[i];
				markerSamples[m]++;
			}
			frameIt++;
		}
	}

	for (int m = 0; m < target.markers.size(); m++)
	{
		markerError[m] /= markerSamples[m]; // may be NAN
		if (markerSamples[m] < params.minMarkerObsCount || markerError[m] > maxErrors.marker)
		{
			target.markers[m].setConstant(NAN);
			markerOutliers++;
		}
	}

	int frameRemoved = 0;
	target.totalSamples = 0;
	for (auto frameIt = target.frames.begin(); frameIt != target.frames.end();)
	{
		for (auto sampleIt = frameIt->samples.begin(); sampleIt != frameIt->samples.end();)
		{
			int m = target.markerMap.at(sampleIt->marker);
			if (target.markers[m].hasNaN())
				sampleIt = frameIt->samples.erase(sampleIt);
			else
				sampleIt++;
		}
		if (frameIt->samples.size() < params.minFrameObsCount)
		{
			frameRemoved++;
			frameIt = target.frames.erase(frameIt);
		}
		else
		{
			target.totalSamples += frameIt->samples.size();
			frameIt++;
		}
	}
	target.outlierSamples += oldSampleCnt-target.totalSamples;

	for (auto markerIt = target.markerMap.begin(); markerIt != target.markerMap.end();)
	{
		if (target.markers[markerIt->second].hasNaN())
			markerIt = target.markerMap.erase(markerIt);
		else
		{ // Adjust target index to new index after markers will be deleted
			int indexSearch = markerIt->second;
			for (int m = 0; m < indexSearch; m++)
				if (target.markers[m].hasNaN())
					markerIt->second--;
			markerIt++;
		}
	}

	for (auto markerIt = target.markers.begin(); markerIt != target.markers.end();)
	{
		if (markerIt->hasNaN())
			markerIt = target.markers.erase(markerIt);
		else
			markerIt++;
	}

	auto newErrors = getTargetErrorDist(calibs, target);

	LOG(LTargetCalib, LDebug, "Identified Outliers: %d samples, %d markers, %d frames, and further %d frames removed",
		sampleOutliers, markerOutliers, frameOutliers, frameRemoved);
	LOG(LTargetCalib, LDebug, "    Reduced counts: samples %d -> %d, outliers %d -> %d, frames %d -> %d, markers %d -> %d, sequences %d -> %d!",
		oldSampleCnt, target.totalSamples, oldOutlierCnt, target.outlierSamples, oldFrameCnt, (int)target.frames.size(),
		oldMarkerCnt, (int)target.markers.size(), oldSequenceCnt, (int)target.markerMap.size());
	LOG(LTargetCalib, LDebug, "   Reduced RMSE from %.2fpx (max %.2fpx) with %.2f%% outliers to %.2fpx (max %.2fpx) with %.2f%% outliers\n",
		oldErrors.rmse*PixelFactor, oldErrors.max*PixelFactor, (float)oldOutlierCnt/(oldSampleCnt+oldOutlierCnt)*100,
		newErrors.rmse*PixelFactor, newErrors.max*PixelFactor, (float)target.outlierSamples/(target.totalSamples+target.outlierSamples)*100);
	return sampleOutliers || markerOutliers || frameOutliers;
}

template<bool APPLY>
void reevaluateMarkerSequences(const std::vector<CameraCalib> &calibs, const std::vector<MarkerSequences> &observations, ObsTarget &target,
	ReevaluateSequenceParameters params)
{
	if (target.frames.size() < params.minOverlap) return;
	params.maxSeqRMSE *= PixelSize;
	params.uncertainty *= PixelSize;
	params.match.uncertainty *= PixelSize;

	ScopedLogCategory scopedLogCategory(LTargetCalib);

	// NOTE: In logging of this function:
	// sequence means MarkerSequences and all it's sequences,
	// Marker means a marker in target represented by multiple MarkerSequences as recorded by target.markerMap

	// For each sequence, try to assign to a marker
		// Check against each marker, even if previously assigned, and debug close seconds
		// New sequence assignments can happen if error is sufficiently low
		// Sequence assignments shouldn't really be upturned unless error really high
	// For each marker, try to merge with other markers:
		// Check assignments plus candidates for significant overlap indicating they might be the same
		// Check assignments for conflicts, e.g. two sequences of one camera with temporal overlap
	
	// Use for loaded TargetViews to try to re-acquire
		// NO - can just to an update to frames based on markerMap
	// Use for merging markers
		// 
	// Use for merging views
		// Merge frames, reevaluateMarkerSequences, observe mapping from marker to marker
		// updateTargetObservations, optimise on subset of frames, realign

	LOGC(LDebug, "Reevaluating marker sequences:")

	// Precalculate marker positions for each frame in linear access vector
	std::pair<int, int> frameRange = { target.frames.front().frame, target.frames.back().frame+1 };
	std::vector<std::vector<Eigen::Vector3f>> frameMarkers(frameRange.second-frameRange.first);
	for (auto &frame : target.frames)
	{
		auto &markers = frameMarkers[frame.frame - frameRange.first];
		assert(markers.empty());
		markers.reserve(target.markers.size());
		for (auto &mkTgt : target.markers)
			markers.push_back(frame.pose * mkTgt);
	}

	// Get all potential matches of observed sequences to the 3D markers
	const int MAX_MERGE_COUNT = 4; // Won't be able to handle more than this many markers that are close / should be merged
	typedef MatchCandidates<int, WeightedMatch<>, MAX_MERGE_COUNT> SeqMatchCandidates;
	std::vector<SeqMatchCandidates> seqMarkerMatches;
	std::vector<WeightedMatch<>> markerMatch;
	for (int o = 0; o < observations.size(); o++)
	{
		auto &mkObs = observations[o];
		auto range = mkObs.getFrameRange();
		if (range.second < target.frames.front().frame+params.minOverlap) continue;
		if (range.first > target.frames.back().frame-params.minOverlap) continue;
		// Got overlap, check if markers observation sequences match a markers 3D path
		markerMatch.clear();
		markerMatch.resize(target.markers.size());
		int totalObs = 0;

		// Check all observation samples for frames we have a pose of (all frames)
		std::map<int, int> frameMap;
		getObservationFrameMap(mkObs, frameMap, frameRange.first, frameRange.second);
		handleMappedSequences(mkObs, frameMap, [&]
			(const PointSequence &seq, int c, int s, int seqOffset, int start, int length)
		{
			for (int i = 0; i < length; i++)
			{
				auto &markers = frameMarkers[seq.startFrame+seqOffset+i - frameRange.first];
				if (markers.empty()) continue;
				totalObs++;
				for (int m = 0; m < markers.size(); m++)
				{
					Eigen::Vector2f proj = (calibs[c].camera.cast<float>() * markers[m].homogeneous()).hnormalized().head<2>();
					markerMatch[m].value += (proj-seq.getPoint(seqOffset+i)).squaredNorm();
				}
			}
		});

		// Record promising candidates (this markers sequences matching to a 3D marker)
		SeqMatchCandidates candidates = {};
		float maxSESum = params.maxSeqRMSE*params.maxSeqRMSE*totalObs; // Max square error sum
		for (int m = 0; m < target.markers.size(); m++)
		{
			auto &match = markerMatch[m];
			if (totalObs >= params.minOverlap && match.value < maxSESum)
			{
				match.value = std::sqrt(match.value/totalObs); // RMSE
				match.weight = totalObs;
				recordMatchCandidate(candidates, match).index = m;
			}
		}
		if (candidates.matches[0].index >= 0)
		{
			candidates.context = o;
			seqMarkerMatches.push_back(candidates);
		}
	}

	resolveMatchCandidates(seqMarkerMatches, target.markers.size(), params.match);

	// Double-check existing sequence-marker associations
	std::vector<std::pair<int, float>> markerSEsum(target.markers.size());
	for (auto mapIt = target.markerMap.begin(); mapIt != target.markerMap.end();)
	{
		auto &map = *mapIt;
		auto match = std::find_if(seqMarkerMatches.begin(), seqMarkerMatches.end(), [&](auto &m){ return m.context == map.first; });
		if (match == seqMarkerMatches.end())
		{ // Bad
			LOGC(LDarn, "    Lost sequence %d (for marker %d) completely!",
				map.first, map.second);
			if constexpr (APPLY)
				mapIt = target.markerMap.erase(mapIt);
			else mapIt++;
			continue;
		}
		auto &pri = match->matches[0];
		auto &seq = observations[match->context];
		auto range = seq.getFrameRange();
		
		if (pri.index == map.second)
		{ // Good
			LOGC(LDebug, "    Retained association of sequence %d (frames %d to %d) to marker %d with reprojection RMSE of %fpx across %d observations! (next %fpx with %d)",
				map.first, range.first, range.second, map.second, pri.value*PixelFactor, pri.weight, match->matches[1].value*PixelFactor, match->matches[1].weight);
			markerSEsum[map.second].first += pri.weight;
			markerSEsum[map.second].second += (pri.value*pri.value) * pri.weight; // Undo RM of RMSE
			mapIt++;
			continue;
		}

		// Potentially bad, find position of marker in match
		int foundIdx = -1;
		for (int i = 1; i < match->matches.size(); i++)
		{
			if (match->matches[i].index == map.second)
			{
				foundIdx = i;
				break;
			}
		}
		if (foundIdx > 0 && !pri.valid())
		{ // Found, and still reasonably a valid match, so keep association
			auto &found = match->matches[foundIdx];
			LOGC(LDarn, "    Loosened association of sequence %d (frames %d to %d) to marker %d, "
				"with reprojection RMSE of %fpx across %d observations in place %d, best marker %d has RMSE of %fpx across %d observations!",
				map.first, range.first, range.second, map.second, found.value*PixelFactor, found.weight, foundIdx,
				pri.index, pri.value*PixelFactor, pri.weight);
			markerSEsum[map.second].first += found.weight;
			markerSEsum[map.second].second += (found.value*found.value) * found.weight; // Undo RM of RMSE
			mapIt++;
			continue;
		}
		// Very bad
		LOGC(LDarn, "    Lost sequence %d (frames %d to %d), initially for marker %d, completely, "
			"found better marker %d with reprojection RMSE of %fpx across %d observations!",
			map.first, range.first, range.second, map.second, pri.index, pri.value*PixelFactor, pri.weight);
		if constexpr (APPLY)
			mapIt = target.markerMap.erase(mapIt);
		else mapIt++;
	}

	// Calculate full RMSE as copy (SEsum will keep getting updated)
	std::vector<std::pair<int, float>> markerRMSE = markerSEsum; // SE-sum
	for (auto &rmse : markerRMSE)
	{
		if (rmse.first > 0)
		{ // Compute total RMSE from individual SE-sums
			rmse.second = std::sqrt(rmse.second/rmse.first);
		}
	}

	// Consider adding new sequences to markers
	for (auto &match : seqMarkerMatches)
	{
		if (target.markerMap.find(match.context) != target.markerMap.end())
			continue; // Already assigned sequence to a marker
		auto &pri = match.matches[0];
		// Good
		LOGC(LDebug, "    Potential new association of sequence %d "
			"to marker %d with reprojection RMSE of %fpx across %d observations! (next %d with %fpx across %d)",
			match.context, pri.index, pri.value*PixelFactor, pri.weight, match.matches[1].index, match.matches[1].value*PixelFactor, match.matches[1].weight);
		if (!pri.valid())
			continue;
		int m = pri.index;
		if (markerRMSE[m].first == 0)
		{ // Shouldn't happen often, last sequence association of marker was deleted, but why unless that sequence was completely wack?
			markerRMSE[m].second = params.uncertainty; // Add more easily
		}
		if (pri.value < (markerRMSE[m].second + params.uncertainty) * params.sigmaAddSeq)
		{
			LOGC(LDebug, "        Accepted as new association!");
			markerSEsum[m].first += pri.weight;
			markerSEsum[m].second += pri.value*pri.value * pri.weight; // Undo RM of RMSE
			if constexpr (APPLY)
				target.markerMap[match.context] = m;
		}
	}

	// Log RMSE of updated markers
	LOGC(LDebug, "  Initial modifications to sequence assignments:")
	for (int m = 0; m < target.markers.size(); m++)
	{
		if (markerRMSE[m].first != markerSEsum[m].first)
		{ // Compute total RMSE from individual SE-sums
			float newRMSE = std::sqrt(markerSEsum[m].second/markerSEsum[m].first);
			LOGC(LDebug, "    Marker %d was updated from %fpx RMSE (%d samples) to %fpx RMSE (%d samples)!",
				m, markerRMSE[m].second*PixelFactor, markerRMSE[m].first, newRMSE*PixelFactor, markerSEsum[m].first);
		}
	}

	LOGC(LDebug, "  Evaluating marker merging based on sequence support:")

	// Consider merging markers if their sequences are long enough, non-conflicting, and they all have respectively low RMSE
	// Start by accumulating sequences that COULD support another marker
	std::vector<std::map<int,std::pair<int, float>>> mergeStats(target.markers.size());
	for (auto &cand : seqMarkerMatches)
	{
		if (target.markerMap.find(cand.context) == target.markerMap.end())
			continue; // Sequence doesn't map to any marker
		int m = target.markerMap[cand.context];
		for (int i = 0; i < cand.matches.size(); i++)
		{
			auto &match = cand.matches[i];
			if (match.index < 0 || match.index == m) continue;
			LOGC(LDebug, "    Sequence %d supports merging marker %d with marker %d: RMSE %.3fpx over %d frames!",
				cand.context, m, match.index, match.value*PixelFactor, match.weight);
			auto &merge = mergeStats[m][match.index];
			merge.first += match.weight;
			merge.second += match.value*match.value * match.weight; // Undo RM of RMSE
		}
	}
	// For each marker pair, check if all sequences of each supports the other
	std::vector<int> markerMap(target.markers.size());
	std::iota(markerMap.begin(), markerMap.end(), 0);
	std::vector<Eigen::Vector3f> mergedMarkers = target.markers;
	std::vector<bool> ogMarkersMerged(target.markers.size());
	for (int m1 = 0; m1 < mergeStats.size(); m1++)
	{
		for (auto &map : mergeStats[m1])
		{
			int m2 = map.first;
			auto support1 = map.second;
			auto support2 = mergeStats[m2][m1];
			mergeStats[m2].erase(m1);
			// Compute total RMSE from individual SE-sums
			float RMSE1 = std::sqrt(support1.second/support1.first);
			float RMSE2 = std::sqrt(support2.second/support2.first);
			// Check if all sequences (including newly added ones) agree
			// TODO: Detect joined markers if a lot of sequences agree, and a lot disagree
			// If two joined markers only add sequences that fully agree with both, it will discard a lot
			// So at some point, the sequences that agree with a significant portion, but not with the rest, will increase
			// If we make use of that we can gather evidence a marker is really two joined markers and split them here
			bool merge = true;
			if (support1.first < markerSEsum[m1].first) merge = false;
			if (support2.first < markerSEsum[m2].first) merge = false;
			if (!merge)
			{ // Don't merge
				LOGC(LTrace, "    Won't merge markers %d and %d, support %d/%d /w %fpx RMSE and %d/%d /w %fpx RMSE",
					m1, m2, support1.first, markerSEsum[m1].first, RMSE1*PixelFactor, support2.first, markerSEsum[m2].first, RMSE2*PixelFactor);
				continue;
			}
			if (RMSE1 > (markerRMSE[m1].second+params.uncertainty)*params.sigmaMerge) merge = false;
			if (RMSE2 > (markerRMSE[m2].second+params.uncertainty)*params.sigmaMerge) merge = false;
			if (!merge)
			{ // Don't merge, errors too high
				LOGC(LDebug, "    Could merge markers %d and %d, support %d/%d /w %fpx RMSE and %d/%d /w %fpx RMSE, but errors are too high",
					m1, m2, support1.first, markerSEsum[m1].first, RMSE1*PixelFactor, support2.first, markerSEsum[m2].first, RMSE2*PixelFactor);
				continue;
			}
			// TODO: Consider direction both markers are viewed from and only merge if consistent with assumed FoV
			// Assuming flat markers, that would prevent false merging of two markers on different sides of a thin structure
			for (auto &map1 : target.markerMap)
			{
				if (markerMap[map1.second] != m1) continue;
				for (auto &map2 : target.markerMap)
				{
					if (markerMap[map2.second] != m2) continue;
					if (map1.first > map2.first) continue; // Only check one direction
					int overlapFrame = canProveMarkerDistinct(observations[map1.first], observations[map2.first]);
					if (overlapFrame >= 0)
					{ // Found overlap
						int tgtFrame = 0;
						for (; tgtFrame < target.frames.size() && target.frames[tgtFrame].frame < overlapFrame; tgtFrame++) {}
						LOGC(LDebug, "    Sequences %d and %d overlapped starting frame %d (index %d+)", map1.first, map2.first, overlapFrame, tgtFrame);
						merge = false;
						break;
					}
				}
				if (!merge) break;
			}
			if (!merge)
			{ // Don't merge, sequence overlap proves they are distinct
				LOGC(LDebug, "    Planned to merge markers %d and %d, support %d/%d /w %fpx RMSE and %d/%d /w %fpx RMSE, but sequence overlap proved them to be distinct!",
					m1, m2, support1.first, markerSEsum[m1].first, RMSE1*PixelFactor, support2.first, markerSEsum[m2].first, RMSE2*PixelFactor);
				continue;
			}
			int mergeM = std::min(m1, m2);
			int deleteM = std::max(m1, m2);
			int mergeMk = markerMap[mergeM];
			int deleteMk = markerMap[deleteM];
			if (ogMarkersMerged[m1] || ogMarkersMerged[m2])
			{
				LOGC(LInfo, "    Was going to merge markers %d(%d) and %d(%d), support %d/%d /w %fpx RMSE and %d/%d /w %fpx RMSE, but either one was already merged!",
					mergeM, mergeMk, deleteM, deleteMk, support1.first, markerSEsum[m1].first, RMSE1*PixelFactor, support2.first, markerSEsum[m2].first, RMSE2*PixelFactor);
				continue;
			}
			ogMarkersMerged[m1] = ogMarkersMerged[m2] = true;
			LOGC(LInfo, "    Merging markers %d(%d) and %d(%d), support %d/%d /w %fpx RMSE and %d/%d /w %fpx RMSE",
				mergeM, mergeMk, deleteM, deleteMk, support1.first, markerSEsum[m1].first, RMSE1*PixelFactor, support2.first, markerSEsum[m2].first, RMSE2*PixelFactor);
			for (int &m : markerMap)
			{
				if (m == deleteMk) m = mergeMk;
				else if (m > deleteMk) m--;
			}
			LOGC(LTrace, "        3D difference is %fmm, current indices %d and %d",
				(mergedMarkers[mergeMk] - mergedMarkers[deleteMk]).norm()*1000, mergeMk, deleteMk);
			mergedMarkers[mergeMk] = (mergedMarkers[mergeMk] + mergedMarkers[deleteMk]) / 2;
			mergedMarkers.erase(mergedMarkers.begin()+deleteMk);
		}
		mergeStats[m1].clear(); // All handled
	}
	if constexpr (APPLY)
	{
		target.markers.swap(mergedMarkers);
		for (auto &map : target.markerMap)
		{
			map.second = markerMap[map.second];
		}


		LOGC(LDebug, "New mapping of sequences to markers:");
		for (auto &map : target.markerMap)
		{
			LOGC(LDebug, "    Sequence %d maps to marker %d", map.first, map.second);
		}
	}
}

template void reevaluateMarkerSequences<true>(const std::vector<CameraCalib> &calibs, const std::vector<MarkerSequences> &observations, ObsTarget &target,
	ReevaluateSequenceParameters params);
template void reevaluateMarkerSequences<false>(const std::vector<CameraCalib> &calibs, const std::vector<MarkerSequences> &observations, ObsTarget &target,
	ReevaluateSequenceParameters params);

ObsTarget subsampleTargetObservations(const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, const std::vector<MarkerSequences> &observations, const ObsTarget &target,
	SubsampleTargetParameters params)
{
	ObsTarget subsampled = {};
	subsampled.targetID = target.targetID;
	subsampled.markerMap = target.markerMap;
	subsampled.markers = target.markers;
	
	// Now fill frames with randomised most important selection of frames from original target

	/* Address dataset size with sparse selection of optimisation data
    - As sample size increases, optimisation becomes exponentially (?) slower
    - Start to subsample dataset to speed up
    - Pick frames via descending priority:
        - Markers observed in the fewest frames take priority
        - Samples matching their sequences weight take priority over weak ones (higher potential for outliers)
        - Frames with high-value samples (and more of them) take priority 
    - Calculate frame value dynamically:
        - value(frame) = sum_over_samples(exp(value(sample) * value(marker)))
        - value(marker) = 1 / num_samples_of_marker_selected / log(num_samples_of_marker_total)
        - value(sample) = weight(sample) / weight_75th_percentile(sequence)
            and maybe   * weight(sample) / weight_75th_percentile(marker)
        Percentile calculated once from a fixed array of buckets, only value(marker) has to be updated (every time or in intervals) */

	std::pair<int, int> frameRange = { target.frames.front().frame, target.frames.back().frame+1 };

	const int MAX_VALUE = 255*4; 
	typedef std::array<unsigned int, 100> ValueBuckets;
	auto getPercentile = [](const ValueBuckets &buckets, float percentile)
	{ // May not be entirely accurate, but is sufficient
		assert(percentile > 0);
		unsigned int sum = 0;
		for (unsigned int bucket : buckets)
			sum += bucket;
		unsigned int proc = 0;
		for (int i = 0; i < buckets.size(); i++)
		{
			float prevPerc = (float)proc/sum;
			proc += buckets[i];
			float nextPerc = (float)proc/sum;
			if (nextPerc >= percentile)
			{ // Doesn't work with 0 percentile and prevPerc == nextPerc == 0, but no matter
				float lerp = (percentile-prevPerc) / (nextPerc-prevPerc);
				return (i+lerp)/buckets.size();
			}
		}
		return 1.0f;
	};

	auto getBlobValue = [&](int frame, int cam, Eigen::Vector2f raw)
	{
		// Get frame record
		auto frameRec = frameRecords.getView().pos(frame);
		if (!frameRec.accessible() || !*frameRec) return -1;
		auto &camFrameRec = frameRec->get()->cameras[cam];
		// Find blob in source FrameRecord - no direct link exists
		int closest = -1;
		float bestDist = 1000000.0f;
		for (int i = 0; i < camFrameRec.rawPoints2D.size(); i++)
		{
			float dist = (raw - camFrameRec.rawPoints2D[i]).squaredNorm();
			if (dist < bestDist)
			{
				bestDist = dist;
				closest = i;
			}
		}
		if (bestDist > 0.1f*PixelSize) return -1;
		return camFrameRec.properties[closest].value;
	};

    // value(sample) = weight(sample) / weight_75th_percentile(sequence)
	//				 * weight(sample) / weight_75th_percentile(marker)

	// Precalculate fixed marker and sequence percentiles
	std::vector<ValueBuckets> markerValueBuckets(target.markers.size());
	std::map<const PointSequence*, float> sequencePercentile;
	for (auto &map : target.markerMap)
	{
		auto &mkObs = observations[map.first];
		auto range = mkObs.getFrameRange();		

		// Calculate third quartile value of marker and its sequences
		ValueBuckets &markerBuckets = markerValueBuckets[map.second];
		std::map<const PointSequence*, ValueBuckets> sequenceBuckets;

		// Iterate over all sequences relevant to the target
		std::map<int, int> frameMap;
		getObservationFrameMap(mkObs, frameMap, frameRange.first, frameRange.second);
		handleMappedSequences(mkObs, frameMap, [&]
			(const PointSequence &seq, int c, int s, int seqOffset, int start, int length)
		{
			// Instead of looping only over mapped range, handle full sequence
			ValueBuckets &seqBuckets = sequenceBuckets[&seq];
			for (int p = 0; p < seq.length(); p++)
			{
				// Find blob value by searching in frame records
				int value = getBlobValue(seq.startFrame + p, c, seq.rawPoints[p]);
				if (value < 0) continue;
				// Enter value into bucket of both marker and sequence
				int bucket = (int)std::floor((float)value/(MAX_VALUE+1) * markerBuckets.size());
				markerBuckets[bucket]++;
				seqBuckets[bucket]++;
			}
		});

		// Calculate desired percentile of sequence values
		for (auto &seqBuckets : sequenceBuckets)
			sequencePercentile[seqBuckets.first] = getPercentile(seqBuckets.second, params.percentile) * MAX_VALUE;
	}
	std::vector<float> markerPercentile(target.markers.size());
	for (int m = 0; m < target.markers.size(); m++)
		markerPercentile[m] = getPercentile(markerValueBuckets[m], params.percentile) * MAX_VALUE;

	// Precalculate fixed inividual sample factors
	std::vector<std::vector<float>> frameSampleFactors(target.frames.size());
	std::vector<int> markerSamples(target.markers.size());
	for (int f = 0; f < target.frames.size(); f++)
	{
		auto &frame = target.frames[f];
		frameSampleFactors[f].reserve(frame.samples.size());
		for (auto &sample : frame.samples)
		{
			// Get sample value
			int value = getBlobValue(frame.frame, sample.camera, sample.point);
			if (value < 0)
			{ // Outlier, have no observed blob
				frameSampleFactors[f].push_back(NAN);
				continue;
			}
			// Get sequence and it's value percentile
			auto &camObs = observations[sample.marker].cameras[sample.camera];
			auto ptIt = camObs.frame(frame.frame);
			auto &seq = camObs.sequences[ptIt.seq];
			float seqPerc = sequencePercentile[&seq];
			// Get marker percentile
			int marker = target.markerMap.at(sample.marker);
			float markerPerc = markerPercentile[marker];
			// Calculate final sample value
			frameSampleFactors[f].push_back(value/seqPerc * value/markerPerc);
			// value(sample) = weight(sample) / weight_75th_percentile(sequence)
			//				 * weight(sample) / weight_75th_percentile(marker)
			markerSamples[marker]++;
		}
	}

	// value(frame) = sum_over_samples(exp(value(sample) * value(marker)))
	// value(marker) = 1 / num_samples_of_marker_selected / log(num_samples_of_marker_total)

	// Precalculate fixed marker factors
	std::vector<float> fixedMarkerFactor(target.markers.size());
	for (int m = 0; m < target.markers.size(); m++)
		fixedMarkerFactor[m] = 1.0f / std::log(markerSamples[m]);

	// Collectors for samples selected
	markerSamples.clear();
	markerSamples.resize(target.markers.size());
	int totalSamples = 0, totalFrames = 0;

	// Iteratively calculate frame factors and add best in batches
	const int MAX_FRAME_BATCH = 1000;
	std::vector<float> frameFactors;
	std::vector<bool> frameSelected(target.frames.size());


	static std::mt19937 gen = std::mt19937(std::random_device{}());
	std::normal_distribution<float> noise(0, params.randomStdDev);
	while (true)
	{
		// Recalculate frame factors
		frameFactors.clear();
		frameFactors.resize(target.frames.size());
		MultipleExtremum<float, MAX_FRAME_BATCH> bestFrameFactors(std::numeric_limits<float>::min());
		for (int f = 0; f < target.frames.size(); f++)
		{
			if (frameSelected[f]) continue;
			float &frameFactor = frameFactors[f];
			frameFactor = 0.0f;
			// value(frame) = sum_over_samples(exp(value(sample) * value(marker)))
			// value(marker) = 1 / num_samples_of_marker_selected / log(num_samples_of_marker_total)
			auto &frame = target.frames[f];
			for (int s = 0; s < frame.samples.size(); s++)
			{
				float sampleFac = frameSampleFactors[f][s];
				if (std::isnan(sampleFac)) continue; // Outlier
				int marker = target.markerMap.at(frame.samples[s].marker);
				float markerFac = fixedMarkerFactor[marker] / (markerSamples[marker]+1);
				frameFactor += std::pow(sampleFac * markerFac, params.sampleFactorPower);
			}
			frameFactor += noise(gen);
			bestFrameFactors.max(frameFactor);
		}

		// Get threshold as weakest link of batch
		int weakest = std::min(params.maxSelectBatch, bestFrameFactors.weakest());
		if (weakest < 0) break; // No more frames to add
		float factorLimit = bestFrameFactors.rank[weakest];

		// Select frames below threshold (should be MAX_FRAME_BATCH, or less at the end)
		int framesAdded = 0;
		for (int f = 0; f < target.frames.size(); f++)
		{
			float frameFactor = frameFactors[f];
			if (frameSelected[f] || frameFactor < factorLimit) continue;
			frameSelected[f] = true;
			framesAdded++;
			auto &frame = target.frames[f];
			totalSamples += frame.samples.size();
			for (int s = 0; s < frame.samples.size(); s++)
			{
				float sampleFac = frameSampleFactors[f][s];
				if (std::isnan(sampleFac)) continue; // Outlier
				int marker = target.markerMap.at(frame.samples[s].marker);
				markerSamples[marker]++;
			}
		}
		totalFrames += framesAdded;
		LOG(LTargetCalib, LTrace, "Added batch of %d frames with values %f-%f, total %d samples!",
			framesAdded, bestFrameFactors.rank[0], factorLimit, totalSamples);

		if (weakest < std::min(params.maxSelectBatch, MAX_FRAME_BATCH)) break; // No more frames to add
		if (totalSamples > params.targetSampleCount) break;
	}

	// Compile final frame selection in correct order
	subsampled.totalSamples = 0;
	subsampled.outlierSamples = 0;
	subsampled.frames.reserve(totalFrames);
	for (int f = 0; f < target.frames.size(); f++)
	{
		if (!frameSelected[f]) continue;
		subsampled.frames.push_back(target.frames[f]);
		subsampled.totalSamples += target.frames[f].samples.size();
		// TODO: Filter samples based on NAN in frameSampleFactors to discard outliers?
		// Sequence2D algorithm may add intermediate fake blobs not actually observed
		// These outliers have no value recorded and are thus set to NAN in frameSampleFactors
		// Use that to filter them out here before they are used for optimisation
	}
	assert(subsampled.totalSamples == totalSamples);
	assert(subsampled.frames.size() == totalFrames);

	return subsampled;
}

static TargetMatch2D tryTrackFrame(const std::vector<CameraCalib> &calibs, const std::shared_ptr<FrameRecord> &frameRecord,
	const TargetCalibration3D &trkTarget, const ObsTarget &target, const ObsTargetFrame &reference, const TargetTrackingParameters &params)
{
	std::vector<std::vector<Eigen::Vector2f> const *> points2D(frameRecord->cameras.size());
	std::vector<std::vector<BlobProperty> const *> properties(frameRecord->cameras.size());
	for (int c = 0; c < frameRecord->cameras.size(); c++)
	{
		points2D[c] = &frameRecord->cameras[c].points2D;
		properties[c] = &frameRecord->cameras[c].properties;
	}

	// TODO: Get remainingPoints2D from FrameRecord
	std::vector<std::vector<int>> remainingPoints2D(frameRecord->cameras.size());
	std::vector<std::vector<int> const *> relevantPoints2D(frameRecord->cameras.size());
	for (int c = 0; c < frameRecord->cameras.size(); c++)
	{
		remainingPoints2D[c].resize(points2D[c]->size());
		std::iota(remainingPoints2D[c].begin(), remainingPoints2D[c].end(), 0);
		relevantPoints2D[c] = &remainingPoints2D[c];
	}

	TargetTracking2DData internalData(frameRecord->cameras.size());
	return trackTarget2D(trkTarget, reference.pose, Eigen::Vector3f::Zero(),
		calibs, frameRecord->cameras.size(),
		points2D, properties, relevantPoints2D, params, internalData);
}

void expandFrameObservations(const std::vector<CameraCalib> &calibs, const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords,
	ObsTarget &target, const TargetCalibration3D &trkTarget, TrackFrameParameters params)
{
	LOG(LTargetCalib, LDebug, "Expand frame observations for target:");
	// Try filling in previously discarded frames
	uint32_t startFrame = target.frames.front().frame, endFrame = target.frames.back().frame;
	uint32_t prevFrames = target.frames.size();
	uint32_t missingFrameCount = endFrame-startFrame-prevFrames, filledFrameCount = 0;
	uint32_t refFrame;
	auto frames = frameRecords.getView();
	auto frameIt = frames.begin();
	for (uint32_t i = 0; i < prevFrames-1; i++)
	{
		refFrame = i; // Use last/closest frame (even filled in) for reference (pose, observed points)
		uint32_t first = target.frames[i].frame+1;
		if (frameIt.index() < first) frameIt += first-frameIt.index();
		for (; frameIt.index() < target.frames[i+1].frame; frameIt++)
		{ // Try to fill any missing frames
			if (!*frameIt) continue;
			TargetMatch2D targetMatch2D = tryTrackFrame(calibs, *frameIt, trkTarget, target, target.frames[refFrame], params.track);
			LOG(LTargetCalib, LDebug, "    Trying to fill in frame %ld yielded %d matches with %fpx error!",
				frameIt.index(), targetMatch2D.error.samples, targetMatch2D.error.mean*PixelFactor);
			if (targetMatch2D.error.samples < params.minTrackPts || targetMatch2D.error.mean > (target.frames[i].error+params.uncertainty)*params.sigmaFill)
			{
				if (frameIt.index()-target.frames[refFrame].frame >= 2) break;
				else continue;
			}
			ObsTargetFrame newFrame;
			newFrame.frame = frameIt.index();
			newFrame.pose = targetMatch2D.pose;
			newFrame.error = targetMatch2D.error.mean;
			// TODO: SERIOUS BLUNDER! Need to properly add samples from targetMatch2D.points2D
			// But need to not disturb the sequence2D markers recorded in markerMap
			// So perhaps add virtual markers (negative?) to markerMap to signal they are not a sequence but random points
			// But then reevaluateMarkerSequences needs to account for that
			// If it finds and adds a sequence that matches these points, it needs to change these markers to that sequences marker
			newFrame.samples = target.frames[i].samples; // Copy to allow new frame to be used as refIt
			refFrame = target.frames.size();
			target.frames.push_back(std::move(newFrame)); // But will have to reevaluateMarkerSequences
			filledFrameCount++;
		}
	}
	// Try to expand to new frames outward
	int prependFrameCount = 0, appendFrameCount = 0;	
	refFrame = 0;
	frameIt = frames.pos(startFrame);
	while (frameIt-- > frames.begin())
	{
		TargetMatch2D targetMatch2D = tryTrackFrame(calibs, *frameIt, trkTarget, target, target.frames[refFrame], params.track);
		LOG(LTargetCalib, LDebug, "    Trying to prepend frame %ld yielded %d matches with %fpx error!",
			frameIt.index(), targetMatch2D.error.samples, targetMatch2D.error.mean*PixelFactor);
		if (targetMatch2D.error.samples < params.minTrackPts || targetMatch2D.error.mean > (target.frames[0].error+params.uncertainty)*params.sigmaAdd)
		{
			if (target.frames[refFrame].frame-frameIt.index() >= 2) break;
			else continue;
		}
		ObsTargetFrame newFrame;
		newFrame.frame = frameIt.index();
		newFrame.pose = targetMatch2D.pose;
		newFrame.error = targetMatch2D.error.mean;
		// TODO: SERIOUS BLUNDER! See above
		newFrame.samples = target.frames[0].samples; // Copy to allow new frame to be used as refIt
		refFrame = target.frames.size();
		target.frames.push_back(std::move(newFrame)); // But will have to reevaluateMarkerSequences
		prependFrameCount++;
	}
	refFrame = prevFrames-1;
	frameIt = frames.pos(endFrame);
	while (++frameIt < frames.end())
	{
		TargetMatch2D targetMatch2D = tryTrackFrame(calibs, *frameIt, trkTarget, target, target.frames[refFrame], params.track);
		LOG(LTargetCalib, LDebug, "    Trying to append frame %ld yielded %d matches with %fpx error!",
			frameIt.index(), targetMatch2D.error.samples, targetMatch2D.error.mean*PixelFactor);
		if (targetMatch2D.error.samples < params.minTrackPts || targetMatch2D.error.mean > (target.frames[prevFrames-1].error+params.uncertainty)*params.sigmaAdd)
		{
			if (frameIt.index()-target.frames[refFrame].frame >= 2) break;
			else continue;
		}
		ObsTargetFrame newFrame;
		newFrame.frame = frameIt.index();
		newFrame.pose = targetMatch2D.pose;
		newFrame.error = targetMatch2D.error.mean;
		// TODO: SERIOUS BLUNDER! See above
		newFrame.samples = target.frames[prevFrames-1].samples; // Copy to allow new frame to be used as refIt
		refFrame = target.frames.size();
		target.frames.push_back(std::move(newFrame)); // But will have to reevaluateMarkerSequences
		appendFrameCount++;
	}

	// Bring both frame vectors back into correct order
	std::sort(target.frames.begin(), target.frames.end(), [&](auto &a, auto &b){ return a.frame < b.frame; });

	LOG(LTargetCalib, LInfo, "Managed to fill in %d/%d missing frames, added %d to the beginning and %d to the end, for a total of %d frames",
		filledFrameCount, missingFrameCount, prependFrameCount, appendFrameCount, (int)target.frames.size());
}

template<bool APPLY>
OptErrorRes reevaluateFrameObservations(const std::vector<CameraCalib> &calibs, const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, ObsTarget &target, const TargetCalibration3D &trkTarget, TrackFrameParameters params)
{
	// target.frames contains all frames with a best-estimate of the pose
	// target.markers contains all current 3D markers
	// Now reproject in each frame, and match to observations from scratch
	// But instead of sequence markers, which filtered out potentially unreliable, short views of markers,
	// Use the raw records of points seen in that frame similar to how it's actually tracked
	// This will discard the sequence marker association we use (see markerMap), but that should be alright

	// TODO: Currently deprecated, was meant as a outlier detection method of sorts
	// But it IS useful to add new observations that didn't make it as a sequence
	// So rework to only ADD new markers that aren't part of a sequence?
	// Then do it similarly to the SERIOUS BLUNDER note above in expandFrameObservations with negative markers

	int prevSampleCnt = 0, newSampleCnt = 0;
	float prevError = 0;
	int invalidFrames = 0, invalidSamples = 0;
	OptErrorRes error = {};

	if constexpr (APPLY)
	{
		target.totalSamples = 0;
		target.outlierSamples = 0;
		// Will have "outliers", but no way to count them
	}
	
	auto frames = frameRecords.getView();
	for (auto &frame : target.frames)
	{
		prevError += frame.error;
		prevSampleCnt += frame.samples.size();

		auto &frameRecord = frames[frame.frame];
		TargetMatch2D targetMatch2D = tryTrackFrame(calibs, frameRecord, trkTarget, target, frame, params.track);	
		if (targetMatch2D.error.samples == 0)
		{
			invalidFrames++;
			invalidSamples += frame.samples.size();
			error.mean += frame.error;
			error.max = std::max(error.max, frame.error);
			newSampleCnt += frame.samples.size();
			for (auto &sample : frame.samples)
				sample.marker = target.markerMap[sample.marker];
			continue;
		}

		error.mean += targetMatch2D.error.mean;
		error.max = std::max(error.max, targetMatch2D.error.max);
		newSampleCnt += targetMatch2D.error.samples;

		if constexpr (!APPLY) continue;

		frame.samples.clear();
		for (int c = 0; c < frameRecord->cameras.size(); c++)
		{
			for (auto &match : targetMatch2D.points2D[c])
				frame.samples.push_back({ (uint32_t)match.first, (uint16_t)c, frameRecord->cameras[c].rawPoints2D.at(match.second) });
		}
		frame.pose = targetMatch2D.pose;
		frame.error = targetMatch2D.error.mean;
		target.totalSamples += targetMatch2D.error.samples;
	}

	if constexpr (APPLY)
	{
		// Sequence2D marker designation makes no sense now that we include samples outside of point sequence observations
		target.markerMap.clear();
		for (int m = 0; m < target.markers.size(); m++)
			target.markerMap[m] = m;
		// TODO: May conflict with actual sequence2D markers when integrating further TargetViews!
		// Maybe look at existing markerMap and pick a random sequence marker for each target marker to use
		// Addendum: Look at above TODOs for further info on how to proceed
	}
	error.mean /= target.frames.size();

	LOG(LTargetCalib, LInfo, "    Reevaluating observations of %d frames of %d previous samples with avg error of %fpx, "
		"yielded %d total samples with avg error of %fpx, including %d untrackable frames with %d samples",
		(int)target.frames.size(), prevSampleCnt, (prevError/target.frames.size())*PixelFactor,
		newSampleCnt, error.mean*PixelFactor, invalidFrames, invalidSamples);

	return error;
}

template OptErrorRes reevaluateFrameObservations<true>(const std::vector<CameraCalib> &calibs, const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, ObsTarget &target, const TargetCalibration3D &trkTarget, TrackFrameParameters params);

void verifyTargetObservations(const std::vector<CameraCalib> &calibs, const BlockedQueue<std::shared_ptr<FrameRecord>> &frameRecords, const ObsTarget &target, const TargetCalibration3D &trkTarget)
{
	int sampleCnt = 0;
	float prevError = 0, newError = 0;

	auto frames = frameRecords.getView();
	for (auto &frame : target.frames)
	{
		auto &frameRecord = frames[frame.frame];

		std::vector<std::vector<Eigen::Vector2f>> points2DRecStore(frameRecord->cameras.size());
		TargetMatch2D targetMatch2D = {};
		targetMatch2D.calib = &trkTarget;
		targetMatch2D.pose = frame.pose;
		targetMatch2D.points2D.resize(frameRecord->cameras.size());
		for (auto &sample : frame.samples)
		{
			targetMatch2D.points2D[sample.camera].push_back({ target.markerMap.at(sample.marker), (int)points2DRecStore[sample.camera].size() } );
			points2DRecStore[sample.camera].push_back(undistortPoint(calibs[sample.camera], sample.point));
		}

		std::vector<std::vector<Eigen::Vector2f> const *> points2D(frameRecord->cameras.size());
		for (int c = 0; c < frameRecord->cameras.size(); c++)
			points2D[c] = &points2DRecStore[c];

		TargetMatchError errors = evaluateTargetPose(calibs, points2D, targetMatch2D);
		prevError += frame.error;
		newError += errors.mean;
		sampleCnt += frame.samples.size();
	}

	LOG(LTargetCalib, LInfo, "    Verifying observations of %d frames of %d samples with avg error of %fpx, yielded new avg error of %fpx",
		(int)target.frames.size(), sampleCnt, (prevError/target.frames.size())*PixelFactor, (newError/target.frames.size())*PixelFactor);
}

Eigen::Vector3f analyzeMarkerOrientation(std::vector<Eigen::Vector3f> &markerRays, const TargetPostProcessingParameters &params)
{
	struct ErrorTerm
	{
		enum
		{
			InputsAtCompileTime = Eigen::Dynamic,
			ValuesAtCompileTime = Eigen::Dynamic
		};
		std::vector<Eigen::Vector3f> dirs;
		float extremityCenteringPower;
		typedef float Scalar;
		typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
		typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
		typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

		int operator()(const Eigen::VectorXf &coeffs, Eigen::VectorXf &errors) const
		{
			Eigen::Vector3f centralDir;
			centralDir[0] = coeffs(0);
			centralDir[1] = coeffs(1);
			centralDir[2] = coeffs(2);
			centralDir.normalize();
			for (int d = 0; d < dirs.size(); d++)
			{
				float align = centralDir.dot(dirs[d]);
				float angle = std::acos(align); // Linearise
				errors(d) = std::pow(angle, extremityCenteringPower); // High enough order to make just the most extreme angles matter
			}
			return 0;
		}

		int inputs() const { return 3; }
		int values() const { return dirs.size(); }
	};
	ErrorTerm errorTerm = {};
	errorTerm.dirs = std::move(markerRays);
	errorTerm.extremityCenteringPower = params.viewCone.extremityCenteringPower;

	Eigen::NumericalDiff<ErrorTerm> errorGradient(errorTerm);
	Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ErrorTerm>, float> lm(errorGradient);

	Eigen::Vector3f centralDir = Eigen::Vector3f::Zero();
	for (auto &dir : errorTerm.dirs)
		centralDir += dir;
	centralDir.normalize();

	Eigen::VectorXf dirCoeff(centralDir);
	auto status = lm.minimize(dirCoeff);
	centralDir = Eigen::Vector3f(dirCoeff);

	markerRays = std::move(errorTerm.dirs);

	return centralDir.normalized();
}

template<std::size_t NUM, typename CONTAINER>
static inline float mixLimits(const CONTAINER limits, const std::array<float,NUM> &factors)
{
	float limit = 0, sum = 0;
	for (int i = 0; i < NUM; i++)
	{
		limit += limits[i] * factors[i];
		sum += factors[i];
	}
	return limit/sum;
};

void updateMarkerOrientations(std::vector<TargetMarker> &markers, const std::vector<CameraCalib> &calibs, const ObsTarget &target, const TargetPostProcessingParameters &params)
{
	assert(markers.size() == target.markers.size());
	if (markers.size() < 3) return;

	// Gather all samples determining a marker as rays
	std::vector<std::vector<Eigen::Vector3f>> markerRays(target.markers.size());
	for (auto &frame : target.frames)
	{
		for (auto &sample : frame.samples)
		{
			Eigen::Vector2f point = undistortPoint(calibs[sample.camera], sample.point);
			Ray3f ray = castRay<float>(point, calibs[sample.camera]);
			Eigen::Vector3f dir = frame.pose.inverse().rotation() * (-ray.dir);
			int m = target.markerMap.at(sample.marker);
			markerRays[m].push_back(dir);
		}
	}

	std::vector<Eigen::Vector3f> normals(markers.size());
	std::vector<float> limits(markers.size());
	for (int m = 0; m < markers.size(); m++)
	{
		// Find most likely marker orientation
		normals[m] = analyzeMarkerOrientation(markerRays[m], params);
		// Find field of view with some resistance to outliers
		MultipleExtremum<float, 3> worstAligns(1.0f);
		for (auto &dir : markerRays[m])
			worstAligns.min(normals[m].dot(dir));
		limits[m] = mixLimits(worstAligns.rank, params.viewCone.ownSampleExtremes);
	}
	// Find widest markers and estimate field of view to use for all markers
	std::partial_sort(limits.begin(), limits.begin()+3, limits.end());
	float limit = mixLimits(limits, params.viewCone.topMarkerExtremes);
	LOG(LTargetCalib, LDebug, "Determined primary FoV for %d markers as %.4fdg. Top 3 are %.4fdg, %.4fdg, and %.4fdg",
		(int)markers.size(), std::acos(limit)*360/PI,
		std::acos(limits[0])*360/PI, std::acos(limits[1])*360/PI, std::acos(limits[2])*360/PI);

	for (int m = 0; m < markers.size(); m++)
	{
		markers[m].nrm = normals[m];
		// This is accounting for a small range of observations by overestimating FoV greatly
		float conservativeLimit = std::cos(std::acos(limit)*2 - std::acos(limits[m]));
		// In the end, mix individual observed limit, shared limit from widest markers, and conservative overestimated limit
		markers[m].angleLimit = mixLimits(std::array{ limits[m], limit, conservativeLimit }, params.viewCone.markerLimitMix);
		LOG(LTargetCalib, LTrace, "    Marker %d with %d observations had own FoV of %.4fdg, so conservatively %.4fdg, updated to %.4fdg",
			m, (int)markerRays[m].size(), std::acos(limits[m])*360/PI, std::acos(conservativeLimit)*360/PI, std::acos(markers[m].angleLimit)*360/PI);
	}
}

std::vector<TargetMarker> finaliseTargetMarkers(const std::vector<CameraCalib> &calibs, const ObsTarget &target, const TargetPostProcessingParameters &params)
{
	// Init markers
	float assumedLimit = std::cos(params.assumedAngle/360*PI); // NAN for no assumption
	std::vector<TargetMarker> markers;
	markers.reserve(target.markers.size());
	for (int m = 0; m < target.markers.size(); m++)
		markers.emplace_back(target.markers[m], Eigen::Vector3f::Zero(), assumedLimit, params.assumedSize);

	updateMarkerOrientations(markers, calibs, target, params);
	return markers;
}