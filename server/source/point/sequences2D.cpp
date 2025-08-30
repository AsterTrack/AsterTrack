/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

//#define LOG_MAX_LEVEL LTrace
#include "point/sequence_data.hpp"
#include "util/log.hpp"

#include "point/sequences2D.hpp"
#include "point/sequence_data.inl"

#include "calib/camera_system.inl"

#include "util/matching.hpp"
#include "util/eigenalg.hpp"

#include "util/util.hpp" // TimePoint_t

#include <iterator>

/**
 * Recording 2D sequences of observed points and correlating them between cameras
 */


/* Structures */

typedef std::vector<Eigen::Vector2f>::const_iterator VecIt;

template<typename Iterator>
struct Range
{
	Iterator start;
	Iterator end;
	int length;
};

struct PtSeqMatch
{
	PointSequence *seq;
	int marker;
};
typedef MatchCandidates<PtSeqMatch, MatchCandidate<bool>> PtMatchCandidates;


/* Function Prototypes */

/**
 * Match both markers against each other to check if they can be merged
 * Calculates stats of pairwise checks of overlapping observations
 */
static void calculateMarkerMatchStats(const SequenceAquisitionParameters &params, const CameraSystemCalibration &calibration,
		const MarkerSequences &markerA, const MarkerSequences &markerB,
		StatDistf &stats, float &fitError, float &supportingWeight, float &discreditingWeight);

/**
 * Returns the overlap between a sequence seqT and relevant sequences of a marker observed by a camera
 */
static OverlapRange getOverlappingObservations(const CameraSequences &camera, const PointSequence &seqT, std::vector<Eigen::Vector2f> &pointsC, std::vector<Eigen::Vector2f> &pointsT);

/**
 * Returns a normalisation matrix to normalise the given point sequence by centering and scaling to a distance of sqrt(2)
 */
template<typename Scalar, typename PointIterator>
static Eigen::Matrix<Scalar,3,3> getPointNormalisation(const Range<PointIterator> &sequence);

/**
 * Calculates the fundamental matrix from a two corresponding sequences of points (may be concatenated)
 */
template<typename Scalar, typename PointIterator>
static float calculateFundamentalMatrix(FundamentalMatrix &FM, const Range<PointIterator> &sequenceA, const Range<PointIterator> &sequenceB);

/* Functions */

static inline float getStatsUncertainty(const SequenceAquisitionParameters &params, const FundamentalMatrix &FM)
{ // Increase by an uncertainty margin
	float uncertainty = 1.0f - std::min(1.0f, (float)FM.stats.num/params.FM.ConfidentMinSamples);
	return uncertainty*params.FM.UpperError;
}

static inline float getFitError(const SequenceAquisitionParameters &params, const FundamentalMatrix &FM, float error)
{ // Fit of a single sample to an FM for ongoing correspondence verification
	float limit = FM.stats.avg + FM.stats.stdDev()*params.FM.FitPointSigma + getStatsUncertainty(params, FM);
	return error / limit;
}

static inline float getFitError(const SequenceAquisitionParameters &params, const FundamentalMatrix &FM, StatDistf errors)
{ // Fit of a range of samples to an FM for correspondence matching
	float limit = FM.stats.avg + FM.stats.stdDev()*params.FM.FitSeqSigmaMax + getStatsUncertainty(params, FM);
	return (errors.avg + errors.stdDev()*params.FM.FitSeqSigmaSelect) / limit;
}

static inline float getFitError(const SequenceAquisitionParameters &params, StatDistf newFMErrors)
{ // Fit of a new FM calculated on a small range of samples
	return (newFMErrors.avg + newFMErrors.stdDev()*params.FM.FitNewSigma) / params.FM.UpperError;
}

static inline float getTrustFactor(float trust, float base)
{ // Fit of a new FM calculated on a small range of samples
	return std::max( 0.1f, 1 + std::log(trust / base));
}

static inline StatDistf calculateCorrespondenceStats(const Range<VecIt> &rangesA, const Range<VecIt> &rangesB, const Eigen::Matrix3f &FM)
{ // Verify correspondence with fundamental matrix
	// TODO: Could do this more efficiently than the iterative Welford algorithm, whether that's relevant to runtime, idk
	StatDistf errorStats = {};
	assert(!FM.hasNaN());
	for (auto itA = rangesA.start, itB = rangesB.start; itA != rangesA.end && itB != rangesB.end; itA++, itB++)
	{
		float error = itB->homogeneous().transpose() * FM * itA->homogeneous();
		if (std::isnan(error) || std::isinf(error))
		{
			LOG(LSequence, LWarn, "Had NAN/INF error during correspondence check!");
			return {};
		}
		errorStats.update(std::abs(error));
	}
	if (errorStats.num > 0 && errorStats.avg == 0)
	{
		LOG(LSequence, LWarn, "Had 0 total error during correspondence check!");
		return {};
	}
	return errorStats;
}

static int resolveCorrespondences(const SequenceAquisitionParameters &params, CameraSystemCalibration &calibration, SequenceData &sequences, const PointSequence &tmpSeq, int cameraIndex, bool checkGT = false)
{
	if (tmpSeq.isInactive())
	{
		LOGC(LError, "    Cannot reliably solve correspondence of inactive sequence!\n");
		return -1;
	}

	LOGC(LDebug, "Searching for correspondences for temporary sequence (GT %d) in camera %d!", tmpSeq.lastGTMarker, cameraIndex);

	struct CorrespondenceSupport
	{
		StatDistf stats;
		float fitError;
		float trust; // Trust of primary FM
		int existingFM; // Index of existing FM (either primary or marked for merging)
		bool testedNewFM; // Is primary FM existing or new?
		Eigen::Matrix3f newFM; // Potentially newly calculated FM to support correspondence

		CorrespondenceSupport()
			: stats{}, fitError(0.0f), existingFM(-1), testedNewFM(false) {}
		CorrespondenceSupport(StatDistf &Stats, float Error, int FMex, float FMtrust)
			: stats(Stats), fitError(Error), trust(FMtrust), existingFM(FMex), testedNewFM(false) {}
		CorrespondenceSupport(StatDistf &Stats, float Error, Eigen::Matrix3f FMmat, int FMmerge)
			: stats(Stats), fitError(Error), trust(100), existingFM(FMmerge), testedNewFM(true), newFM(FMmat) {}
	};

	int GTOverlap = 0, GTOverlapVerifiable = 0;
	
	MatchCandidates<NoCtx, WeightedMatch<std::vector<CorrespondenceSupport>>> correspondenceCandidates;
	for (int m = 0; m < sequences.markers.size(); m++)
	{ // Find ongoing sequence of another camera to find correspondence
		const MarkerSequences &marker = sequences.markers[m];
		if (marker.lastFrame < tmpSeq.startFrame)
			continue; // No significant overlap to verify correspondence with
		if (marker.cameras.size() > cameraIndex && !marker.cameras[cameraIndex].sequences.empty()
			&& marker.cameras[cameraIndex].sequences.back().endFrame() >= tmpSeq.startFrame)
		{ // Overlap with sequence corresponding to this marker by this camera, can't be this marker if existing correspondence was correct
			continue;
		}

		bool GTshouldMatch = false;
		int GTMarkerOverlap = 0, GTMarkerOverlapVerifiable = 0;
		std::pair<int, float> seqGTMarker = { -1, 0.0f };
		if (checkGT)
		{
			seqGTMarker = marker.resolveGTMarker();
			GTshouldMatch = seqGTMarker.first == tmpSeq.lastGTMarker && seqGTMarker.second > 0.9f;
		}

		std::vector<CorrespondenceSupport> correspondence(marker.cameras.size());

		for (int c = 0; c < marker.cameras.size(); c++)
		{
			if (c == cameraIndex) continue;
			const CameraSequences &camObs = marker.cameras[c];
			if (camObs.sequences.empty()) continue;
			int maxOverlapLength = camObs.sequences.back().endFrame() - tmpSeq.startFrame;
			if (maxOverlapLength <= 0) continue;
			if (GTshouldMatch) GTMarkerOverlap += maxOverlapLength;
			// Take any amount of samples, because only a few might be enough to disprove correspondence

			// Get existing fundamental matrix
			bool flipFM = cameraIndex < c;
			int camA = flipFM? c : cameraIndex;
			int camB = flipFM? cameraIndex : c;
			auto &fmEntry = calibration.relations.getFMEntry(camA, camB);
			if (fmEntry.candidates.empty() && maxOverlapLength < params.correspondences.minOverlap)
				continue; // Need minimum to calculate new fundamental matrix
			// Assume trust has been managed externally (recent update, best candidate in front)
			bool exConfident = !fmEntry.candidates.empty() && fmEntry.candidates.front().floatingTrust > params.FM.ConfidentMinTrust;

			// Get overlap between tmpSeq and all sequences from this camera
			std::vector<Eigen::Vector2f> pointsC, pointsT;
			OverlapRange overlap = getOverlappingObservations(camObs, tmpSeq, pointsC, pointsT);
			if (fmEntry.candidates.empty() && overlap.length < params.correspondences.minOverlap)
				continue; // Need minimum to calculate new fundamental matrix
			if (GTshouldMatch) GTMarkerOverlapVerifiable += overlap.length;
			Range<VecIt> rangesA = { pointsT.begin(), pointsT.end(), (int)pointsT.size() };
			Range<VecIt> rangesB = { pointsC.begin(), pointsC.end(), (int)pointsC.size() };

			LOGC(LTrace, "    Overlap of %d frames in cameras (%d-%d) for marker %d (GT %d, %.0f%%)", 
				overlap.length, cameraIndex, c, m, seqGTMarker.first, seqGTMarker.second*100);
			if (exConfident)
			{ // Calculate correspondence error from existing fundamental matrix
				auto &FMex = fmEntry.candidates.front();
				Eigen::Matrix3f matrixEx = flipFM? FMex.matrix.transpose() : FMex.matrix;
				auto errorEx = calculateCorrespondenceStats(rangesA, rangesB, matrixEx);
				float fitError = getFitError(params, FMex, errorEx);
				correspondence[c] = CorrespondenceSupport(errorEx, fitError, 0, fmEntry.candidates.front().floatingTrust);
				LOGC(LTrace, "      Trusted FM candidate had %f fit error for correspondence with error %f+-%f and weight %d",
					fitError, errorEx.avg, errorEx.stdDev(), overlap.length);
				// TODO: Early discard if a trusted FM discredits correspondence
				continue;
			}

			// Find best FM candidate that supports correspondence
			struct CorrContext
			{
				float fitError;
				StatDistf dist;
			};
			MatchCandidates<NoCtx, MatchCandidate<CorrContext>> correspondenceCandidates;
			MatchCandidate<CorrContext> candidate;
			for (int i = 0; i < fmEntry.candidates.size(); i++)
			{ // Calculate correspondence error from existing fundamental matrix
				auto &FMex = fmEntry.candidates[i];
				Eigen::Matrix3f matrixEx = flipFM? FMex.matrix.transpose() : FMex.matrix;
				auto errorEx = calculateCorrespondenceStats(rangesA, rangesB, matrixEx);
				candidate.value = errorEx.avg;
				candidate.context = { getFitError(params, FMex, errorEx), errorEx };
				LOGC(LTrace, "        Existing FM Candidate %d (%d samples) had %f fit error for correspondence with error %f+-%f and weight %d",
					i, FMex.stats.num, candidate.context.fitError, errorEx.avg, errorEx.stdDev(), overlap.length);
				recordMatchCandidate(correspondenceCandidates, candidate).index = i;
			}
			// TODO: Give preference to first (most confident) one to not encourage divergent, but similar FMs?

			auto &bestCorr = correspondenceCandidates.matches[0];
			if (bestCorr.index >= 0)
			{ // Found an FM candidate supporting this correspondence
				correspondence[c] = CorrespondenceSupport(bestCorr.context.dist, bestCorr.context.fitError, bestCorr.index, fmEntry.candidates[bestCorr.index].floatingTrust);
				if (bestCorr.context.fitError < 1.0f)
				{ // Found an FM candidate supporting this correspondence
					LOGC(!checkGT || GTshouldMatch? LTrace : LDarn,
						"      Found FM candidate %d to be supporting correspondence with %f fit error (error %f+-%f) - leaving it at that",
						bestCorr.index, bestCorr.context.fitError, bestCorr.context.dist.avg, bestCorr.context.dist.stdDev());
					continue;
				}
				LOGC(LTrace, "      Best FM candidate %d does not support correspondence, will attempt to create new FM candidate",
					bestCorr.index);
			}
			if (GTshouldMatch && !fmEntry.candidates.empty())
			{
				LOGC(LDarn, "      Found no supporting FM among %d for GT correspondence! Best had %f fit error for correspondence with error %f+-%f",
					(int)fmEntry.candidates.size(), bestCorr.context.fitError, bestCorr.context.dist.avg, bestCorr.context.dist.stdDev());
			}

			if (overlap.length < params.correspondences.minOverlap)
			{
				if (GTshouldMatch)
				{
					LOGC(LDarn, "      No supporting correspondence and not enough overlap to support GT correspondence!");
				}
				else
				{
					LOGC(LTrace, "      No supporting correspondence and not enough overlap to determine new correspondence!");
				}
				continue;
			}

			// Calculate new fundamental matrix from sequence overlap
			FundamentalMatrix FMseq = {};
			float confidence = calculateFundamentalMatrix<float>(FMseq, rangesA, rangesB);
			if (FMseq.stats.num < params.correspondences.minOverlap)
				continue;

			// Check for basic correspondence
			if (confidence < params.correspondences.minConfidence)
			{ // Discard degenerate cases
				if (GTshouldMatch)
				{
					LOGC(LDarn, "      New FM (%d-%d) is not confident enough with error %f over "
						"%d points overlap and confidence %f (< %f) for GT correspondence", 
						cameraIndex, c, FMseq.stats.avg, FMseq.stats.num, confidence, params.correspondences.minConfidence);
				}
				else
				{
					LOGC(LTrace, "      New FM (%d-%d) is not confident enough with error %f over "
						"%d points overlap and confidence %f (< %f)", 
						cameraIndex, c, FMseq.stats.avg, FMseq.stats.num, confidence, params.correspondences.minConfidence);
				}
				continue;
			}
			if (FMseq.stats.avg > params.correspondences.maxError)
			{ // Not corresponding
				if (GTshouldMatch)
				{
					LOGC(LDarn, "      New FM (%d-%d) failed to support correspondence with error %f (> %f) over "
						"%d points overlap and confidence %f", 
						cameraIndex, c, FMseq.stats.avg, params.correspondences.maxError, FMseq.stats.num, confidence);
				}
				else
					LOGC(LTrace, "      New FM (%d-%d) does not support correspondence with error %f (> %f) over "
						"%d points overlap and confidence %f", 
						cameraIndex, c, FMseq.stats.avg, params.correspondences.maxError, FMseq.stats.num, confidence);
				continue;
			}
			float fitError = getFitError(params, FMseq.stats);
			LOGC(checkGT && !GTshouldMatch? LDarn : LDebug,
				"      New FM (%d-%d) confirms correspondence with %f error (fit error %f) over %d points and confidence %f!", 
				cameraIndex, c, FMseq.stats.avg, fitError, FMseq.stats.num, confidence);

			// If existing best FM is close enough, it may just be slighly off
			// In that case, if the correspondence is accepted, merge FMs instead of creating a new candidate
			int mergeFM = -1;
			if (bestCorr.index >= 0 && bestCorr.context.fitError < 5)
				mergeFM = bestCorr.index;

			// Add new FM supporting this correspondence, if it is accepted, the FM will be taken on as a candidate
			correspondence[c] = CorrespondenceSupport(FMseq.stats, fitError, FMseq.matrix, mergeFM);
		}

		GTOverlapVerifiable += GTMarkerOverlapVerifiable;
		GTOverlap += GTMarkerOverlap;

		LOGC(LTrace, "    Summary of best correspondence clues to marker %d from relevant cameras:", m);
		float supportingWeight = 0.0f, discreditingWeight = 0.0f;
		float errorAvg = 0.0f;
		for (int c = 0; c < marker.cameras.size(); c++)
		{
			if (c == cameraIndex) continue;
			auto &corr = correspondence[c];
			if (corr.stats.num == 0) continue;
			// Camera had clues supporting or discrediting correspondence
			if (corr.fitError > 1.0f)
			{ // Existing FM discrediting this
				assert(!corr.testedNewFM);
				float trustFactor = getTrustFactor(corr.trust, params.correspondences.TrustBaseDiscrediting);
				float weight = corr.stats.num * (corr.fitError - 1) * trustFactor;
				LOGC(LTrace, "      Camera %d correspondence to marker %d has fit error %f along %d frames with %f trust factor - discrediting with %f!", c, m, corr.fitError, corr.stats.num, trustFactor, weight);
				discreditingWeight += weight;
			}
			else
			{
				float trustFactor = getTrustFactor(corr.trust, params.correspondences.TrustBaseSupporting);
				float weight = corr.stats.num * (1 / corr.fitError - 1) * trustFactor;
				LOGC(LTrace, "      Camera %d correspondence to marker %d has fit error %f along %d frames with %f trust factor - supporting with %f!", c, m, corr.fitError, corr.stats.num, trustFactor, weight);
				supportingWeight += weight;
			}
			errorAvg += corr.stats.avg;
			assert(!std::isnan(errorAvg));
			assert(!std::isnan(supportingWeight) && !std::isnan(discreditingWeight));
			assert(!std::isinf(supportingWeight) && !std::isinf(discreditingWeight));
		}
		errorAvg /= marker.cameras.size();

		if (supportingWeight < params.correspondences.minWeight || supportingWeight*params.correspondences.maxDiscrediting <= discreditingWeight)
		{ // Not corresponding
			if (GTshouldMatch && GTMarkerOverlap > params.correspondences.minOverlap)
				LOGC(LDarn, "    Failed to add correct correspondence candidate for marker %d with error %f and %f discrediting, %f supporting it! Had %d total overlaps, %d verifiable",
					m, errorAvg, discreditingWeight, supportingWeight, GTMarkerOverlap, GTMarkerOverlapVerifiable);
			else
				LOGC(LTrace, "      Deemed NOT corresponding to marker %d with error %f and %f discrediting, %f supporting it!",
					m, errorAvg, discreditingWeight, supportingWeight);
			continue;
		}

		if (checkGT && !GTshouldMatch)
		{
			LOGC(LDarn, "    Incorrectly added wrong correspondence candidate for marker %d with error %f and %f discrediting, %f supporting it!", 
				m, errorAvg, discreditingWeight, supportingWeight);
		}
		else
		{
			LOGC(LDebug, "    Added correspondence candidate for marker %d with error %f and %f discrediting, %f supporting it!", 
				m, errorAvg, discreditingWeight, supportingWeight);
		}

		WeightedMatch<std::vector<CorrespondenceSupport>> match = 
			{ false, m, errorAvg, (int)(supportingWeight-discreditingWeight), std::move(correspondence) };
		recordMatchCandidate(correspondenceCandidates, match);
	}

	auto &pri = correspondenceCandidates.matches[0];
	auto &sec = correspondenceCandidates.matches[1];
	bool mergeMarkers = false;

	if (sec.index >= 0)
	{ // Potentially discard primary if it's not advantaged over secondary
		bool advantaged = sec.getValue() > (pri.getValue() + params.correspondences.errorUncertainty) * params.correspondences.minPrimAdvantage;

		LOGC(LDebug,
		"Had competing potential correspondences for temporary sequence (GT %d) with primary value %f (error %f, support %d) and secondary value %f, (error %f, support %d)%s", 
			tmpSeq.lastGTMarker, pri.getValue()*1000, pri.value, pri.weight, sec.getValue()*1000, sec.value, sec.weight, advantaged? " - advantaged" : " - not advantaged");

		if (checkGT)
		{ // Debug correspondence competition
			auto priGT = sequences.markers[pri.index].resolveGTMarker();
			auto secGT = sequences.markers[sec.index].resolveGTMarker();
			if (!advantaged)
			{ // Whether correct to drop or not, indicates the correspondence check didn't work as well as intended
				LOGC(priGT.first != tmpSeq.lastGTMarker? LDebug : LDarn,
				"--- Dropped correspondence of temporary sequence (GT %d) with primary error %f (support %d) (GT %d, %.0f%%) due to secondary error %f (support %d) (GT %d, %.0f%%)", 
					tmpSeq.lastGTMarker,
					pri.value, pri.weight, priGT.first, priGT.second*100,
					sec.value, sec.weight, secGT.first, secGT.second*100);
			}
			else if (priGT.first != tmpSeq.lastGTMarker)
			{ // Should have better discrimination abilities since it didn't discard wrong correspondence, even WITH competition
				LOGC(LDarn, 
				"--- Should've dropped correspondence of temporary sequence (GT %d) with primary error %f (support %d) (GT %d, %.0f%%) and secondary error %f[%d] (GT %d, %.0f%%)", 
					tmpSeq.lastGTMarker,
					pri.value, pri.weight, priGT.first, priGT.second*100,
					sec.value, sec.weight, secGT.first, secGT.second*100);
			}
		}

		if (!advantaged && canProveMarkerDistinct(sequences.markers[pri.index], sequences.markers[sec.index]) >= 0)
		{
			LOGC(LDebug, "    Competing markers are definitively distinct, cannot assume correspondence!");
			return -1;
		}
		if (!advantaged)
		{ // Try merging
			StatDistf stats = {};
			float fitError, supportingWeight, discreditingWeight;
			calculateMarkerMatchStats(params, calibration, sequences.markers[pri.index], sequences.markers[sec.index],
				stats, fitError, supportingWeight, discreditingWeight);
			LOGC(LDebug, "    Competing markers are matching with fit error %f, supporting %f and discrediting %f, with error %f+-%f across %d overlapping frames",
				fitError, supportingWeight, discreditingWeight, stats.avg, stats.stdDev(), stats.num);
			if (supportingWeight < params.correspondences.minWeight || supportingWeight*params.correspondences.maxDiscrediting <= discreditingWeight)
				return -1;
			mergeMarkers = true;
		}
	}

	if (checkGT && pri.index >= 0)
	{ // Debug whether matched correspondence is correct or not
		auto priGT = sequences.markers[pri.index].resolveGTMarker();
		if (tmpSeq.lastGTMarker != priGT.first)
		{
			LOGC(LDarn, "--- Incorrectly corresponded temporary sequence (GT %d) to marker %d (GT %d, %.0f%%) with error %f (support %d)",
				tmpSeq.lastGTMarker, pri.index, priGT.first, priGT.second*100, pri.value, pri.weight);
			bool matchingSec = sec.index >= 0 && sequences.markers[sec.index].resolveGTMarker().first == tmpSeq.lastGTMarker;
			if (!matchingSec && GTOverlap > 0)
			{ // Debug existing correct overlap when it wasn't used for correspondence matching
				LOGC(LDarn, "    -> Didn't use correct correspondences (%d total, %d useable) with correct marker(s)!",
					GTOverlap, GTOverlapVerifiable);
			}
		}
		else
		{
			LOGC(LDebug, 
			"    Successfully found correspondence for temporary sequence (GT %d) with primary error %f (support %d) and secondary error %f (support %d)", 
				tmpSeq.lastGTMarker, pri.value, pri.weight, sec.weight == 0? 0 : sec.value, sec.weight);
			if (priGT.second < 0.9f)
			{
				LOGC(LDarn,
				"--- Correctly corresponded temporary sequence (GT %d) to marker %d (GT %d, %.0f%%) with error %f (support %d) even though sequence was heavily conflicted!",
					tmpSeq.lastGTMarker, pri.index, priGT.first, priGT.second*100, pri.value, pri.weight);
			}
		}
	}

	if (pri.index >= 0)
	{
		// TODO: Check how established each supporting correspondence is, and disregard those of new FMs
		// So new FMs will have to accumulate more trust before they can actually do any harm (in matching wrong correspondences)


		// Check what FMs supported correspondence and add to their trust
		auto &correspondence = pri.context;
		for (int c = 0; c < correspondence.size(); c++)
		{
			auto &corr = correspondence[c];
			if (corr.stats.num <= 0) continue;

			bool flipFM = cameraIndex < c;
			int camA = flipFM? c : cameraIndex;
			int camB = flipFM? cameraIndex : c;
			auto &fmEntry = calibration.relations.getFMEntry(camA, camB);
			if (corr.testedNewFM && corr.existingFM < 0)
			{ // Add new FM candidate
				FundamentalMatrix FMnew = {};
				FMnew.matrix = flipFM? corr.newFM.transpose() : corr.newFM;
				FMnew.stats = corr.stats;
				FMnew.firstFrame = tmpSeq.startFrame;
				FMnew.lastFrame = tmpSeq.lastFrame();
				FMnew.floatingTrust = corr.stats.num / corr.fitError;
				fmEntry.candidates.push_back(FMnew);

				LOGC(LInfo, "  Added new FM candidate for (%d-%d) %d/%d with error %.4f +- %f across %d observations",
					cameraIndex, c, (int)fmEntry.candidates.size(), (int)fmEntry.candidates.size(), FMnew.stats.avg, FMnew.stats.stdDev(), FMnew.stats.num);
			}
			else if (corr.testedNewFM && corr.existingFM >= 0)
			{
				auto &FM = fmEntry.candidates[corr.existingFM];

				// Gather all data for which FM claims to be valid for (may end before curFrame - then don't recalculate)
				std::vector<Eigen::Vector2f> pointsA, pointsB;
				int pointCount = getSharedObservations(sequences.markers, FM.firstFrame, FM.lastFrame+1, camA, camB, pointsA, pointsB);
				LOGC(LTrace, "Found a total of %d point correspondences!\n", pointCount);

				// Calculate fundamental matrix using shared observations
				FundamentalMatrix FMnew = {};
				Range<VecIt> rangesA = { pointsA.begin(), pointsA.end(), pointCount};
				Range<VecIt> rangesB = { pointsB.begin(), pointsB.end(), pointCount};
				float confidence = calculateFundamentalMatrix<float>(FMnew, rangesA, rangesB);
				FMnew.floatingTrust = pointCount / getFitError(params, FMnew.stats);

				if (confidence < params.correspondences.minConfidence)
				{ // Confidence way too low
					LOGC(LDarn, "    Tried to update relation (%d, %d) fundamental matrix with %d points (prev %d), but only has confidence %f (< %f) with error %f+-%f", 
						camA, camB, FMnew.stats.num, FM.stats.num, confidence, params.correspondences.minConfidence, FMnew.stats.avg, FMnew.stats.stdDev());
					continue;
				}
				else if (FMnew.stats.avg+FMnew.stats.stdDev() > params.correspondences.maxError)
				{ // Error way too high
					LOGC(LDarn, "    Tried to update relation (%d, %d) fundamental matrix with %d points (prev %d), but only has confidence %f with error %f+-%f (> %f)", 
						camA, camB, FMnew.stats.num, FM.stats.num, confidence, FMnew.stats.avg, FMnew.stats.stdDev(), params.correspondences.maxError);
					continue;
				}

				LOGC(LDebug, "    Updated relation (%d, %d) fundamental matrix with %d points (prev %d), has confidence %f with error %f+-%f - estimated trust %f, floating %f", 
					camA, camB, FMnew.stats.num, FM.stats.num, confidence, FMnew.stats.avg, FMnew.stats.stdDev(), FMnew.floatingTrust, FM.floatingTrust);

				FM.matrix = FMnew.matrix;
				FM.stats = FMnew.stats;
				FM.floatingTrust = FM.floatingTrust; // TODO: Update?
				FM.lastCalculation = tmpSeq.lastFrame();
			}
			else
			{ // Merge stats into FM candidate
				FundamentalMatrix &FMex = fmEntry.candidates[corr.existingFM];
				auto oldStats = FMex.stats;
				FMex.stats.merge(corr.stats);
				FMex.lastFrame = tmpSeq.lastFrame();
				FMex.floatingTrust += corr.stats.num / corr.fitError;
				LOGC(LDebug, "  Updated FM (%d-%d) candidate %d/%d from error %.4f +- %.4f with error %.4f +- %.4f to error %.4f +- %.4f across %d observations",
					cameraIndex, c, corr.existingFM+1, (int)fmEntry.candidates.size(),
						oldStats.avg, oldStats.stdDev(),
						corr.stats.avg, corr.stats.stdDev(),
						FMex.stats.avg, FMex.stats.stdDev(), FMex.stats.num);
			}
		}

		/* if (pri.weight < opt.minNewCorrespondenceWeight)
		{ // May decide to evaluate differently if the FMs are new and not add the correspondence, but that makes it quite difficult for calibration
			pri.index = -1;
		} */
	}

	if (mergeMarkers)
	{
		int tgt = std::min(pri.index, sec.index), src = std::max(pri.index, sec.index);
		 LOGC(LDarn, "    Merging marker %d into %d!", src, tgt);
		MarkerSequences &markerTgt = sequences.markers[tgt];
		MarkerSequences &markerSrc = sequences.markers[src];
		for (int c = 0; c < markerTgt.cameras.size(); c++)
		{
			for (auto &seq : markerSrc.cameras[c].sequences)
			{
				seq.marker = tgt;
				markerTgt.cameras[c].sequences.push_back(std::move(seq));
			}
			std::sort(markerTgt.cameras[c].sequences.begin(), markerTgt.cameras[c].sequences.end(), 
				[](PointSequence &s1, PointSequence &s2) { return s1.startFrame < s2.startFrame; });
		}
		sequences.markers.erase(sequences.markers.begin() + src);
		for (int m = src; m < sequences.markers.size(); m++)
		{
			MarkerSequences &marker = sequences.markers[m];
			for (int c = 0; c < marker.cameras.size(); c++)
				for (auto &seq : marker.cameras[c].sequences)
					seq.marker = m;
		}
		return tgt;
	}

	return pri.index;
}

static void verifySequenceMatches(const SequenceAquisitionParameters &params,
	const SequenceData &sequences, int curFrame, int cameraIndex,
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<int> &pt2GTMarker,
	const std::vector<PtMatchCandidates> &candidates)
{
	for (int c = 0; c < candidates.size(); c++)
	{
		const PtMatchCandidates &match = candidates[c];
		const PointSequence &seq = *match.context.seq;
		if (!match.matches[0].valid()) continue;
		int matchPri = match.matches[0].index;
		int matchSec = match.matches[1].index;

		int seqNum = seq.marker >= 0? seq.marker : reinterpret_cast<intptr_t>(&seq)%9876543;

		bool validPri = pt2GTMarker[matchPri] >= 0 && seq.lastGTMarker == pt2GTMarker[matchPri];
		bool validSec = matchSec >= 0 && pt2GTMarker[matchSec] >= 0 && seq.lastGTMarker == pt2GTMarker[matchSec];
		bool invalidPri = pt2GTMarker[matchPri] >= 0 && seq.lastGTMarker != pt2GTMarker[matchPri];
		//bool invalidSec = pt2GTMarker[matchSec] >= 0 && seq.lastGTMarker != pt2GTMarker[matchSec];
		bool acceptedPri = !match.matches[0].invalid;
		std::pair<int, float> markerGT = { -1, 0.0f };
		if (seq.marker >= 0)
			markerGT = sequences.markers[seq.marker].resolveGTMarker();
		
		float errorPri = std::sqrt(match.matches[0].value);
		float errorSec = matchSec < 0? std::numeric_limits<float>::infinity() : 
			std::sqrt(match.matches[1].value);
		float distPri = (points2D[matchPri] - seq.points.back()).norm();
		float distSec = matchSec < 0? std::numeric_limits<float>::infinity() : 
			(points2D[matchSec] - seq.points.back()).norm();

		if (validPri && !acceptedPri)
		{ // Should've accepted match
			LOGC(LDebug, 
			"--- Frame %d: Should've matched sequence %d with last GT marker %d (total %d, %.0f%%) to point %d (GT %d) (e/d %f / %f)",
				curFrame, seqNum, seq.lastGTMarker, markerGT.first, markerGT.second*100,
				matchPri, pt2GTMarker[matchPri], errorPri*PixelFactor, distPri*PixelFactor);
			bool disruptedSec = false;
			if (matchSec >= 0)
			{ // Had competing match
				disruptedSec = errorSec < (errorPri+params.match.uncertainty)*params.match.primAdvantage;
				if (disruptedSec)
				{
					LOGC(LDebug, 
					"     -> Disrupted by secondary match %d (GT %d) (e/d %f / %f)",
						matchSec, pt2GTMarker[matchSec], errorSec*PixelFactor, distSec*PixelFactor);
				}
				else
				{
					LOGC(LDebug, 
					"        Not disrupted by secondary match %d (GT %d) (e/d %f / %f)...",
						matchSec, pt2GTMarker[matchSec], errorSec*PixelFactor, distSec*PixelFactor);
				}
				// Find secondary match in other candidates, to check if potential disruption was justified or not
				bool secAltCand = false;
				for (int cc = 0; cc < candidates.size(); cc++)
				{
					if (cc == c) continue;
					auto &cand = candidates[cc];
					if (cand.matches[0].index == matchSec)
					{
						secAltCand = true;
						LOGC(LDebug, 
						"        -> Found in other match for GT %d as primary match (secondary was %d), match was %s!", 
							cand.context.seq->lastGTMarker, cand.matches[1].index, cand.matches[0].valid()? "accepted" : "discarded");
					}
					else if (cand.matches[1].index == matchSec)
					{
						secAltCand = true;
						LOGC(LDebug, 
						"        -> Found in other match for GT %d as secondary match (primary was %d), match was %s!", 
							cand.context.seq->lastGTMarker, cand.matches[0].index, cand.matches[0].valid()? "accepted" : "discarded");
					}
				}
				if (!secAltCand)
				{
					LOGC(LDebug,
					"        -> Not found in any other match!");
				}
			}

			bool conflictingSeq = false;
			for (int cc = 0; cc < candidates.size(); cc++)
			{
				if (cc == c) continue;
				auto &cand = candidates[cc];
				float otherDist = (points2D[matchPri] - cand.context.seq->points.back()).norm() * PixelFactor;
				int otherSeqNum = cand.context.seq->marker >= 0? cand.context.seq->marker 
					: reinterpret_cast<intptr_t>(cand.context.seq)%9876543;
				if (cand.matches[0].index == matchPri)
				{
					conflictingSeq = true;
					float otherErrPri = std::sqrt(cand.matches[0].value) * PixelFactor;
					LOGC(LDebug, 
					"     -> Other sequence %d (GT %d) had same primary match %d (e/d %f / %f)!", 
						otherSeqNum, cand.context.seq->lastGTMarker, matchPri, otherErrPri, otherDist);
				}
				else if (cand.matches[1].index == matchPri)
				{
					conflictingSeq = true;
					float otherErrPri = std::sqrt(cand.matches[0].value) * PixelFactor;
					float otherErrSec = std::sqrt(cand.matches[1].value) * PixelFactor;
					LOGC(LDebug,
					"     -> Other sequence %d (GT %d) had secondary match %d (e/d %f / %f)! Primary match %d (GT %d) (e %f) was %s",
						otherSeqNum, cand.context.seq->lastGTMarker, matchPri, otherErrSec, otherDist,
						cand.matches[0].index, pt2GTMarker[cand.matches[0].index], otherErrPri,
						cand.matches[0].invalid? "discarded" : "accepted");
				}
			}
			if (!conflictingSeq && !disruptedSec)
			{
				LOGC(LWarn,
				"     -> Weird, no other sequence nor secondary match to conflict with!");
			}
		}
		else if (invalidPri && acceptedPri)
		{ // Should've discarded match
			LOGC(LDarn,
			"--- Frame %d: Should've discarded match of sequence %d with last GT marker %d (total %d, %.0f%%) to point %d (GT %d) (e/d %f / %f)",
				curFrame, seqNum, seq.lastGTMarker, markerGT.first, markerGT.second*100,
				matchPri, pt2GTMarker[matchPri], errorPri, distPri);
			if (matchSec >= 0 && validSec)
			{
				LOGC(LDarn, 
				"     -> Should've matched with secondary match %d (GT %d) (e/d %f / %f)",
					matchSec, pt2GTMarker[matchSec], errorSec, distSec);
			}
			else if (seq.lastGTMarker >= 0)
			{ // Search for what it should have matched with
				if (matchSec >= 0)
				{
					LOGC(LDarn, 
					"        Had irrelevant secondary match %d (GT %d) (e/d %f / %f)",
						matchSec, pt2GTMarker[matchSec], errorSec, distSec);
				}

				bool seqHasPtMatch = false;
				for (int p = 0; p < pt2GTMarker.size(); p++)
				{
					if (pt2GTMarker[p] == seq.lastGTMarker)
					{
						seqHasPtMatch = true;
						float dist = (points2D[p]-seq.points.back()).norm();
						LOGC(LDarn, 
						"     -> Sequence should have matched point %d (GT %d) (d %f)!", 
							p, pt2GTMarker[p], dist*PixelFactor);
						break;
					}
				}
				if (!seqHasPtMatch)
				{
					LOGC(LDarn,
					"     -> Sequence should've probably ended, GT %d has no clear/unique point this frame!\n", 
						seq.lastGTMarker);
				}

				bool ptHasSeqMatch = false;
				for (int mm = sequences.markers.size()-1; mm >= 0; mm--)
				{
					const MarkerSequences &marker = sequences.markers[mm];
					if (marker.cameras.size() <= cameraIndex) continue;
					const CameraSequences &cam = marker.cameras[cameraIndex];
					if (cam.sequences.empty()) continue;
					auto &seq = cam.sequences.back();

					if (seq.lastGTMarker == pt2GTMarker[matchPri])
					{
						ptHasSeqMatch = true;
						bool oldSeq = curFrame - seq.endFrame() > params.allowedDrops || seq.endFrame() > curFrame;
						float distAlt = (points2D[matchPri]-seq.points.back()).norm();
						LOGC(LDarn,
						"     -> Marker %d has %s sequence that should've matched point instead (d %f), last seen %d frames ago!", 
							mm, oldSeq? "dropped" : "active", distAlt*PixelFactor, curFrame - seq.endFrame());
						//break; // Only log most recent
					}
				}

				auto &temporaries = sequences.temporary[cameraIndex];
				for (int tt = 0; tt < temporaries.size(); tt++)
				{
					const PointSequence &tmpSeq = temporaries[tt];

					if (tmpSeq.lastGTMarker == pt2GTMarker[matchPri])
					{
						ptHasSeqMatch = true;
						float distAlt = (points2D[matchPri]-tmpSeq.points.back()).norm();
						LOGC(LDarn,
						"     -> Temporary sequence %ld should've matched point instead (d %f), last seen %d frames ago!", 
							reinterpret_cast<intptr_t>(&tmpSeq)%9876543, distAlt*PixelFactor, curFrame - tmpSeq.endFrame() + 1);
					}
				}

				if (!ptHasSeqMatch)
				{
					LOGC(LDarn,
					"     -> No sequence for GT %d found, should've started a new temporary!", 
						seq.lastGTMarker);
				}
			}
		}
		else if (validPri && acceptedPri)
		{
			// Uninteresting case, got it all correct
		}
		else if (invalidPri && !acceptedPri)
		{
			// Uninteresting case, got it all correct
		}
		else
		{
			LOGC(LDarn,
			"--- Unaccounted for case found, maybe merged markers (-2), seq GT %d, pri GT %d, sec GT %d!", 
				seq.lastGTMarker, pt2GTMarker[matchPri], matchSec < 0? -1 : pt2GTMarker[matchSec]);
		}
	}

	for (int c = 0; c < candidates.size(); c++)
	{
		const PtMatchCandidates &match = candidates[c];
		if (match.matches[0].valid())
		{
			int gt = pt2GTMarker[match.matches[0].index];
			if (gt >= 0)
				match.context.seq->lastGTMarker = gt;
		}
	}
}

/**
 * Updates the observations of cameraIndex only to include new points, and verifies sequences with other camera observations if required
 */
bool updateSequenceCaptures(const SequenceAquisitionParameters &params,
	CameraSystemCalibration &calibration, SequenceData &sequences, int curFrame, int cameraIndex,
	const std::vector<Eigen::Vector2f> &points2D, const std::vector<Eigen::Vector2f> &rawPoints2D,
	const std::vector<BlobProperty> &properties, const std::vector<int> &pt2GTMarker)
{
	ScopedLogCategory scopedLogCategory(LSequence);

	sequences.lastRecordedFrame = curFrame;

	thread_local std::vector<PtMatchCandidates> matchCandidates;
	matchCandidates.clear();

	auto findContinuationCandidates = [&](PointSequence &seq, int m)
	{
		// Project sequence out to where we expect the current point to be
		int passedFrames = curFrame - seq.startFrame - seq.length() + 1;
		float maxDistSq;
		Eigen::Vector2f projPos;
		if (seq.isInactive())
		{
			maxDistSq = params.maxInactivityMovement*params.maxInactivityMovement;
			projPos = seq.avgPos;
		}
		else
		{
			// Can only allow max acceleration if were actually sure this is an active marker
			// Else noise just jumps around and never becomes inactive
			if (m < 0 && seq.length() < params.minInactivityLength)
				maxDistSq = params.maxInactivityMovement * params.maxInactivityMovement;
			else
			 	maxDistSq = params.maxAcceleration*params.maxAcceleration;
			if (seq.points.size() > 1)
				projPos = seq.points.back() + (seq.points.end()[-1] - seq.points.end()[-2]) * passedFrames;
			else
				projPos = seq.points.back();
		}
		float maxIncludeSq = maxDistSq * params.match.primAdvantage * params.match.primAdvantage; // Need to include further limits

		// Find points in range of projection to add as candidate matches
		PtMatchCandidates candidates = { { &seq, m } };
		MatchCandidate<bool> candidate;
		for (int j = 0; j < points2D.size(); j++)
		{
			float distSq = (points2D[j] - projPos).squaredNorm();
			if (distSq < maxIncludeSq)
			{
				float valueDiff = std::abs(seq.value - properties[j].value);
				candidate.value = distSq + valueDiff * params.valueDiffFactor;
				candidate.context = distSq < maxDistSq;
				recordMatchCandidate(candidates, candidate).index = j;
			}
		}
		if (candidates.matches[0].index < 0)
			return false;
		matchCandidates.push_back(candidates);
		return true;
	};
	auto checkInactivity = [&](PointSequence &seq)
	{
		const float maxInactivityMovementSq = params.maxInactivityMovement*params.maxInactivityMovement;
		Eigen::Vector2f avgDiff = seq.points.back()-seq.avgPos;
		seq.avgPos += avgDiff/params.minInactivityLength/2; // Make it slower to adapt
		if (seq.inactiveLength > 0) return (int)seq.points.size(); // Already marked inactive
		if (seq.points.size() < params.minInactivityLength) return 0; // Not enough data yet
		if (avgDiff.squaredNorm() > maxInactivityMovementSq) return 0;

		//float var = 0.0f, maxSq = 0.0;
		int limit = 0; //seq.points.size()-opt.minInactivityLength; // or 0
		int p;
		for (p = seq.points.size()-1; p >= limit; p--)
		{
			float diffSq = (seq.points[p] - seq.avgPos).squaredNorm();
			if (diffSq > maxInactivityMovementSq) break;
			//maxSq = maxSq < diffSq? diffSq : maxSq;
			//var += diffSq;
		}
		int length = seq.points.size()-p-1;
		if (length < params.minInactivityLength)
		{
			//LOGC(LTrace, "Dropping inactivity test after %d tests, below minimum\n", length);
			return 0;
		}
		//var = var/length;
		//if (maxSq > var*opt.maxSigmaInactivity*opt.maxSigmaInactivity) return -1;
		return length;
	};

	TimePoint_t t0 = sclock::now();

	for (int m = sequences.markers.size()-1; m >= 0; m--)
	{
		MarkerSequences &marker = sequences.markers[m];

		// TODO: Allowed drops may ignore a sequence in ongoing conflict with another sequence
		// In that case, if one drops out before the other, the other gets matched to the conflicted point without proper assurance
		// Need to either block the point for longer (but not match)
		// Or need to orderly drop both sequences when a conflict holds out for more than a few frames
		// Or allow no drops at all - conflict -> drop, and properly match conflicted point as a new temporary with correspondences

		if (curFrame - marker.lastFrame > params.allowedDrops+1) continue;
		if (marker.cameras.size() <= cameraIndex) continue;
		CameraSequences &cam = marker.cameras[cameraIndex];

		// Filter by currently ongoing sequences
		if (cam.sequences.empty()) continue;
		auto &seq = cam.sequences.back();
		if (curFrame - seq.endFrame() > params.allowedDrops || seq.endFrame() > curFrame)
			continue;
		findContinuationCandidates(seq, m);
	}

	TimePoint_t t1 = sclock::now();

	auto &temporaries = sequences.temporary[cameraIndex];
	for (int t = 0; t < temporaries.size(); t++)
	{
		PointSequence &tmpSeq = temporaries[t];
		int maxDrops = tmpSeq.isInactive()? params.allowedDropsTempInactive : params.allowedDropsTemp;
		if (curFrame - tmpSeq.lastFrame() > maxDrops+1)
			continue; // May still be kept around for correspondence later
		findContinuationCandidates(tmpSeq, -t-1);
	}

	TimePoint_t t2 = sclock::now();

	// Resolve matches in a conservative fashion (false positives are worse than false negatives)
	resolveMatchCandidates(matchCandidates, points2D.size(), params.match.squared());

	// Verify matches with GT marker mapping
	if (!pt2GTMarker.empty() && SHOULD_LOGC(LDebug))
	{
		verifySequenceMatches(params, sequences, curFrame, cameraIndex,
			points2D, pt2GTMarker, matchCandidates);
	}

	TimePoint_t t3 = sclock::now();

	// Apply matching points to sequences
	for (int c = 0; c < matchCandidates.size(); c++)
	{
		auto &match = matchCandidates[c];
		if (!match.matches[0].valid()) continue; // No valid match
		if (!match.matches[0].context) continue; // only included to compete
		int p = match.matches[0].index;

		PointSequence &seq = *match.context.seq;

		float valueDiff = std::abs(seq.value - properties[p].value);
		bool outlier = valueDiff > params.maxValueDiff; // Probably temporarily excluded
		if (outlier) continue;

		int dropped = curFrame - seq.endFrame();
		if (dropped > 0)
		{
			for (int i = 0; i < dropped; i++)
			{ // TODO: Fill dropped frames better, or mark them (had outlier list before, too complicated so removed it again)
				seq.points.push_back(seq.points.back() + (points2D[p]-seq.points.back()) * (i+1.0f)/(dropped+1.0f));
				seq.rawPoints.push_back(seq.rawPoints.back() + (rawPoints2D[p]-seq.rawPoints.back()) * (i+1.0f)/(dropped+1.0f));
			}
		}

		/* if (match.context.marker < 0)
		{
			LOGC(LTrace, "TEMP %d:%d:%.4d:%.4d ACTIVE\n", cameraIndex, -match.context.marker-1, seq.startFrame, (int)seq.length());
		}
		else
			LOGC(LTrace, "SEQN %d:%d:%.4d:%.4d ACTIVE\n", cameraIndex, match.context.marker, seq.startFrame, (int)seq.length()); */

		seq.points.push_back(points2D[p]);
		seq.rawPoints.push_back(rawPoints2D[p]);
		if (seq.inactiveLength > 0)
			seq.inactiveLength = seq.points.size();
		if (seq.marker >= 0)
			sequences.markers[seq.marker].lastFrame = curFrame;
		if (!outlier)
		{
			int floatingLength = std::min(params.valueFlowingAvgerage, seq.length());
			seq.value = ((floatingLength-1) * seq.value + properties[p].value) / floatingLength;
		}
	}

	TimePoint_t t4 = sclock::now();

	// Check sequences for inactivity (non-moving points)

	for (int m = sequences.markers.size()-1; m >= 0; m--)
	{
		MarkerSequences &marker = sequences.markers[m];
		if (curFrame - marker.lastFrame > params.allowedDrops+1) continue;
		if (marker.cameras.size() <= cameraIndex) continue;
		CameraSequences &cam = marker.cameras[cameraIndex];

		// Filter by currently ongoing sequences
		if (cam.sequences.empty()) continue;
		auto &seq = cam.sequences.back();
		int droppedFrames = curFrame - seq.lastFrame();
		if (droppedFrames == params.allowedDrops+1)
		{
			// TODO: Keep list of "active" sequences or better, index of oldest still active sequence - either way, need updating here
			LOGC(LTrace, "SEQN %d:%d:%.4d:%.4d INTERRUPTED\n", cameraIndex, m, seq.startFrame, (int)seq.length());
		}
		if (droppedFrames > params.allowedDrops || seq.lastFrame() > curFrame)
			continue;

		if (seq.isInactive())
			continue; // Already inactive
			
		// Check for inactivity
		PointSequence inactiveSequence;
		inactiveSequence.inactiveLength = checkInactivity(seq);
		if (inactiveSequence.inactiveLength == 0)
			continue; // No inactivity

		LOGC(LTrace, "SEQN %d:%d:%.4d:%.4d INACTIVE for %d frames\n", 
			cameraIndex, m, seq.startFrame, (int)seq.points.size(), inactiveSequence.inactiveLength);
		int lengthLeft = seq.points.size()-inactiveSequence.inactiveLength;
		if (lengthLeft < params.minSequenceLength)
		{ // Erase small active start of sequence
			seq.startFrame = seq.startFrame+lengthLeft;
			seq.avgPos = seq.avgPos;
			seq.points.erase(seq.points.begin(), seq.points.begin()+lengthLeft);
			seq.rawPoints.erase(seq.rawPoints.begin(), seq.rawPoints.begin()+lengthLeft);
			seq.inactiveLength = inactiveSequence.inactiveLength;
		}
		else
		{ // Create new inactive sequence
			inactiveSequence.marker = seq.marker;
			inactiveSequence.startFrame = seq.startFrame+lengthLeft;
			inactiveSequence.avgPos = seq.avgPos;
			inactiveSequence.points.insert(inactiveSequence.points.begin(), 
				seq.points.end()-inactiveSequence.inactiveLength, seq.points.end());
			inactiveSequence.rawPoints.insert(inactiveSequence.rawPoints.begin(), 
				seq.rawPoints.end()-inactiveSequence.inactiveLength, seq.rawPoints.end());
			// Truncate remaining sequence
			seq.points.resize(lengthLeft);
			seq.rawPoints.resize(lengthLeft);
			// Add new inactive sequence
			cam.sequences.push_back(inactiveSequence);
		}
	}

	TimePoint_t t5 = sclock::now();

	bool triggerConfidenceCheck = false;
	for (int t = 0; t < temporaries.size(); t++)
	{
		PointSequence &tmpSeq = temporaries[t];
		bool forceCheck = false;

		// Check drops
		int maxDrops = tmpSeq.isInactive()? params.allowedDropsTempInactive : params.allowedDropsTemp;
		int droppedFrames = curFrame - tmpSeq.lastFrame();
		if (droppedFrames > 0)
		{
			LOGC(LTrace, "TEMP %d:%d:%.4d:%.4d Dropout lasting %d frames!",
				cameraIndex, t, tmpSeq.startFrame, (int)tmpSeq.length(), droppedFrames);
		}
		if (droppedFrames > maxDrops)
		{ // Temporary sequence was interrupted, and don't need to wait for resolveCorrespondence
			if (tmpSeq.isInactive() || tmpSeq.length() < params.minSequenceLength)
			{
				if (tmpSeq.length() > 10)
					LOGC(LTrace, "TEMP %d:%d:%.4d:%.4d INTERRUPTED (%d limit, inactive? %c)",
						cameraIndex, t, tmpSeq.startFrame, (int)tmpSeq.length(), params.minSequenceLength, tmpSeq.isInactive()? 'y' : 'n');
				temporaries.erase(temporaries.begin()+t--);
				continue;
			}
			else
			{ // Drop sequence, but do correspondence check first
				forceCheck = true;
				LOGC(LTrace, "TEMP %d:%d:%.4d:%.4d INTERRUPTED - correspondence matching!",
					cameraIndex, t, tmpSeq.startFrame, (int)tmpSeq.length());
			}
		}
		
		// Check for inactivity (once inactive, keep around until dropped)
		bool prevInactive = tmpSeq.isInactive();
		tmpSeq.inactiveLength = checkInactivity(tmpSeq);
		if (tmpSeq.isInactive() && !prevInactive)
		{
			LOGC(LTrace, "TEMP %d:%d:%.4d:%.4d INACTIVITY started!",
				cameraIndex, t, tmpSeq.startFrame, (int)tmpSeq.length());
		}

		forceCheck |= tmpSeq.length() >= params.sequenceCorrespondenceLength;
		if (!tmpSeq.isInactive() && forceCheck)
		{ // Check at stable boundary to get maximum length of temporary
			// Need to call resolveCorresponce on sequences consistently by first frame
			// If no reference found, or uncertain about it, assign new correspondence
			LOGC(LTrace, "TEMP %d:%d:%.4d:%.4d CORRESPONDENCE ATTEMPT\n", 
				cameraIndex, t, tmpSeq.startFrame, (int)tmpSeq.length());

			tmpSeq.marker = resolveCorrespondences(params, calibration, sequences, tmpSeq, cameraIndex, !pt2GTMarker.empty());
			if (tmpSeq.marker >= 0)
			{ // Move to list dedicated to corresponding marker
				LOGC(LTrace, "SEQN %d:%d:%.4d:%.4d ACCEPTED\n", 
					cameraIndex, tmpSeq.marker, tmpSeq.startFrame, (int)tmpSeq.points.size());
				MarkerSequences &marker = sequences.markers[tmpSeq.marker];
				marker.cameras[cameraIndex].sequences.push_back(std::move(tmpSeq));
				marker.lastFrame = curFrame;
				temporaries.erase(temporaries.begin()+t--);
				// Whenever a new sequence of a marker is detected, check if it can contribute to a more confident fundamental matrix
				triggerConfidenceCheck = true;
			}
			else
			{ // Add new marker
				tmpSeq.marker = sequences.markers.size();
				LOGC(LTrace, "SEQN %d:%d:%.4d:%.4d ACCEPTED AS SEQN WITH NEW MARKER\n", 
					cameraIndex, tmpSeq.marker, tmpSeq.startFrame, (int)tmpSeq.points.size());
				MarkerSequences marker;
				marker.lastFrame = curFrame;
				marker.cameras.resize(sequences.temporary.size());
				marker.cameras[cameraIndex].sequences.push_back(std::move(tmpSeq));
				sequences.markers.push_back(std::move(marker));
				temporaries.erase(temporaries.begin()+t--);
			}
		}
	}

	// Add unclaimed points as new temporary sequences
	std::vector<bool> encumberedPoint(points2D.size(), false);
	for (int c = 0; c < matchCandidates.size(); c++)
	{
		auto &cand = matchCandidates[c];
		if (cand.matches[0].index >= 0)
		{ // Point has been assigned or was in conflict, encumbered either way
			encumberedPoint[cand.matches[0].index] = true;
			if (cand.matches[0].invalid && cand.matches[1].index >= 0)
			{ // But was not accepted - also claim secondary as encumbered
				encumberedPoint[cand.matches[1].index] = true;
			}
		}
	}
	for (int i = 0; i < points2D.size(); i++)
	{
		if (encumberedPoint[i]) continue;
		// Start a new temporary point sequence
		LOGC(LTrace, "TEMP %d:%.4d:%d BEGIN\n", cameraIndex, curFrame, (int)temporaries.size());
		temporaries.emplace_back(curFrame, points2D[i], rawPoints2D[i], properties[i].value);
		if (!pt2GTMarker.empty())
			temporaries.back().lastGTMarker = pt2GTMarker[i];
	}

	TimePoint_t t6 = sclock::now();

	//LOGC(LTrace, "Found %d matches - took %.1fms, %.1fms, %.1fms, %.1fms, %.1fms, %.1fms\n", 
	//	matchedPoints, dtMS(t0, t1), dtMS(t1, t2), dtMS(t2, t3), dtMS(t3, t4), dtMS(t4, t5), dtMS(t5, t6));

	return triggerConfidenceCheck;
}

void updateTrustFromEpipolarStats(const SequenceAquisitionParameters &params, FundamentalMatrix &FM)
{
	if (FM.stats.num == 0)
		FM.floatingTrust = 0;
	else
		FM.floatingTrust = FM.stats.num / getFitError(params, FM.stats);
}

void checkSequenceHealth(const SequenceAquisitionParameters &params, CameraSystemCalibration &calibration, SequenceData &sequences, int curFrame, bool confidenceCheck)
{
	if ((curFrame % 100) == 0)
	{
		for (int i = 1; i < calibration.relations.FMStore.size(); i++)
		{
			auto &relations = calibration.relations.FMStore[i];
			assert(i == relations.size());
			for (int j = 0; j < relations.size(); j++)
			{
				auto &fmEntry = relations[j];
				int index = 0;
				for (FundamentalMatrix &FM : fmEntry.candidates)
				{
					index++;
					// TODO: Filter out FM candidates that haven't proven themselves
					// Recalculate only recent ones that still claim to be current
					// And trigger continuous calibration when a long-standing primary FM was ursurped
					// Finally, lower the trust of the primary FM if it gets no new correspondences despite there being plenty sequences
					// Only that way will resolveCorrespondences consider alternative FMs when the primary gets invalidated (e.g. a bumped camera)
					
					// UPDATE: Recent plans changed to not have sequence2D be responsible for monitoring calibration health, only in establishing it
					// Continuous calibration together with tracking is in a much better position to do that
					// The place to do this would be in calib/camera_system
					// TODO: Carefully rework FM trust so it focuses on establishing new FMs, less on managing trust of established FMs

					// Update decaying trust
					float prevTrust = FM.floatingTrust;
					float trust = FM.updateTrust(curFrame, params.FM.TrustDecayRate);
					LOGC(LTrace, "Let FM(%d-%d) candidate %d/%d trust decay from %f to %f", i, j, index, (int)fmEntry.candidates.size(), prevTrust, trust);

					// Decide when to calculate
					if (FM.precalculated)
						continue; // Don't modify externally calculated FM
					if (FM.stats.num > params.FM.ConfidentMinSamples)
						continue; // Already confident
					if (FM.lastFrame <= FM.lastCalculation)
						continue; // Didn't get any new 

					// Gather all data for which FM claims to be valid for (may end before curFrame - then don't recalculate)
					std::vector<Eigen::Vector2f> pointsA, pointsB;
					int pointCount = getSharedObservations(sequences.markers, FM.firstFrame, FM.lastFrame+1, i, j, pointsA, pointsB);
					LOGC(LTrace, "Found a total of %d point correspondences!\n", pointCount);
					/* if (pointCount < FM.stats.num*1.2)
						continue; // No need to recalculate yet */

					// Calculate fundamental matrix using shared observations
					FundamentalMatrix FMnew = {};
					Range<VecIt> rangesA = { pointsA.begin(), pointsA.end(), pointCount};
					Range<VecIt> rangesB = { pointsB.begin(), pointsB.end(), pointCount};
					float confidence = calculateFundamentalMatrix<float>(FMnew, rangesA, rangesB);
					FMnew.floatingTrust = pointCount / getFitError(params, FMnew.stats);

					if (confidence < params.correspondences.minConfidence)
					{ // Confidence way too low
						LOGC(LDarn, "    Tried to update relation (%d, %d) fundamental matrix with %d points (prev %d), but only has confidence %f (< %f) with error %f+-%f", 
							i, j, FMnew.stats.num, FM.stats.num, confidence, params.correspondences.minConfidence, FMnew.stats.avg, FMnew.stats.stdDev());
						continue;
					}
					else if (FMnew.stats.avg+FMnew.stats.stdDev() > params.correspondences.maxError)
					{ // Error way too high
						LOGC(LDarn, "    Tried to update relation (%d, %d) fundamental matrix with %d points (prev %d), but only has confidence %f with error %f+-%f (> %f)", 
							i, j, FMnew.stats.num, FM.stats.num, confidence, FMnew.stats.avg, FMnew.stats.stdDev(), params.correspondences.maxError);
						continue;
					}

					LOGC(LTrace, "    Updated relation (%d, %d) fundamental matrix with %d points (prev %d), has confidence %f with error %f+-%f - estimated trust %f, existing %f", 
						i, j, FMnew.stats.num, FM.stats.num, confidence, FMnew.stats.avg, FMnew.stats.stdDev(), FMnew.floatingTrust, FM.floatingTrust);

					FM.matrix = FMnew.matrix;
					FM.stats = FMnew.stats;
					FM.floatingTrust = FM.floatingTrust; // TODO: Update?
					FM.lastCalculation = curFrame;
				}

				if (fmEntry.candidates.size() > 1)
				{
					// Sort by trust
					std::sort(fmEntry.candidates.begin(), fmEntry.candidates.end(), 
						[](const auto &a, const auto &b){ return a.floatingTrust > b.floatingTrust; });

					// If first one is both trustworthy and confident, clear all others
					// Then, no new FMs will be considered until first FM becomes untrustworthy
					auto &mainFM = fmEntry.candidates.front();
					if (mainFM.floatingTrust > params.FM.ConfidentMinTrust && mainFM.stats.num > params.FM.ConfidentMinSamples)
					{
						LOGC(LInfo, "Accepting main FM of relation (%d, %d) with trust %f, discarding %d other candidates!",
							i, j, mainFM.floatingTrust, (int)fmEntry.candidates.size()-1);
						fmEntry.candidates.resize(1);
					}

					for (auto it = std::next(fmEntry.candidates.begin()); it != fmEntry.candidates.end();)
					{
						if (it->floatingTrust < 1.0f && it->stats.num < params.FM.ConfidentMinSamples
							&& (curFrame - it->lastFrame) > 500)
						{
							LOGC(LInfo, "    Deleting lacking FM of relation (%d, %d) with trust %f%s, have %lu remaining candidates!",
								i, j, it->floatingTrust, it->precalculated? " that was precalculated" : "", fmEntry.candidates.size()-1);
							it = fmEntry.candidates.erase(it);
						}
						else
							++it;
					}
				}
			}
		}
	}

	std::vector<float> errors, relativeErrors;
	for (int m = sequences.markers.size()-1; m >= 0; m--)
	{
		MarkerSequences &marker = sequences.markers[m];
		if (marker.lastFrame < curFrame) continue;
		int pairChecks = 0;
		errors.clear();
		errors.resize(marker.cameras.size(), 0);
		relativeErrors.clear();
		relativeErrors.resize(marker.cameras.size(), 1);
		for (int i = 1; i < marker.cameras.size(); i++)
		{
			const auto &seqI = marker.cameras[i].sequences;
			if (seqI.empty() || seqI.back().lastFrame() != curFrame)
				continue;
			for (int j = 0; j < i; j++)
			{
				const auto &seqJ = marker.cameras[j].sequences;
				if (seqJ.empty() || seqJ.back().lastFrame() != curFrame)
					continue;

				auto &fmEntry = calibration.relations.getFMEntry(i, j);
				if (fmEntry.candidates.empty())
					continue; // No existing correspondence at all

				// NOTE: Assume trust has been managed externally (recent update, best candidate in front)
				// TODO: Rework - update all FMs that claim to still be active (recent lastFrame)

				FundamentalMatrix &FM = fmEntry.candidates.front();
				float error = std::abs(seqJ.back().points.back().homogeneous().transpose() * FM.matrix * seqI.back().points.back().homogeneous());
				float fitError = getFitError(params, FM, error);
				FM.stats.update(error);
				FM.floatingTrust += 1 / std::max(fitError, 0.1f);
				FM.lastFrame = curFrame;

				// Determine if correspondence is broken
				relativeErrors[i] *= fitError;
				relativeErrors[j] *= fitError;
				errors[i] += error;
				errors[j] += error;
				pairChecks++;
			}
		}
		if (pairChecks == 0)
			continue; // No way to cross-check correspondences

		for (int c = 0; c < marker.cameras.size(); c++)
		{
			if (relativeErrors[c] > 1)
			{
				// TODO: Check last points rigorously, and split sequence off into temporary sequence
				// Or better yet, assume whole sequence was wrongfully curresoponded, and separate all completely
				PointSequence &seq = marker.cameras[c].sequences.back();
				LOGC(LDarn, "--- Marker %d: camera %d's last point failed correspondence check with error %f, factor %f > 1",
					m, c, errors[c], relativeErrors[c]);

				// Truncate remaining sequence
				int removeLength = 10;
				int newLength = seq.length()-removeLength;
				seq.points.resize(newLength);
				seq.rawPoints.resize(newLength);

				LOGC(LDarn, "    -> Split off last %d points, %d points remaining\n",
					removeLength, (int)seq.length());
			}
			else if (relativeErrors[c] > 0.8 && relativeErrors[c] < 1)
			{
				LOGC(LDebug, "    Marker %d: camera %d's last point barely passed correspondence check with error %f, factor %f < 1\n", 
					m, c, errors[c], relativeErrors[c]);
			}
			else if (relativeErrors[c] < 1)
			{
				LOGC(LTrace, "    Marker %d: camera %d's last point passed correspondence check with error %f, factor %f < 1\n", 
					m, c, errors[c], relativeErrors[c]);
			}
		}
	}
}

/**
 * Match both markers against each other to check if they can be merged
 * Calculates stats of pairwise checks of overlapping observations
 */
static void calculateMarkerMatchStats(const SequenceAquisitionParameters &params, const CameraSystemCalibration &calibration,
		const MarkerSequences &markerA, const MarkerSequences &markerB,
		StatDistf &stats, float &fitError, float &supportingWeight, float &discreditingWeight)
{
	fitError = 1.0f;
	supportingWeight = discreditingWeight = 0.0f;
	for (int a = 0; a < markerA.cameras.size(); a++)
	{
		for (int b = 0; b < markerB.cameras.size(); b++)
		{
			if (a == b) continue;
			FundamentalMatrix FM;
			if (!calibration.relations.getFundamentalMatrix(a, b, FM))
				continue;
			StatDistf relStats = {};
			handleSharedObservations(markerA, markerB, 0, INT_MAX, a, b,
				[&](auto itA, auto itB, int frame, int length) {
				for (int i = 0; i < length; ++i, ++itA, ++itB)
				{
					float error = itB->homogeneous().transpose() * FM.matrix * itA->homogeneous();
					relStats.update(std::abs(error));
				}
			});

			if (relStats.num == 0) continue;
			stats.merge(relStats);
			float fE = getFitError(params, FM, relStats);
			fitError += fE;
			if (fE >= 1.0)
			{
				float trustFactor = getTrustFactor(FM.floatingTrust, params.correspondences.TrustBaseDiscrediting);
				discreditingWeight += relStats.num * (fE - 1) * trustFactor;
			}
			else
			{
				float trustFactor = getTrustFactor(FM.floatingTrust, params.correspondences.TrustBaseSupporting);
				supportingWeight += relStats.num * (1 / fE - 1) * trustFactor;
			}
		}
	}
}

/**
 * Returns the overlap between a sequence seqT and relevant sequences of a marker observed by a camera
 * 
 */
static OverlapRange getOverlappingObservations(const CameraSequences &camera, const PointSequence &seqT, std::vector<Eigen::Vector2f> &pointsC, std::vector<Eigen::Vector2f> &pointsT)
{
	OverlapRange totalOverlap = {};
	if (camera.sequences.empty()) return totalOverlap;
	auto seqAIt = std::prev(camera.sequences.end());
	while (seqAIt->endFrame() > seqT.startFrame)
	{
		const PointSequence &seqA = *seqAIt;
		OverlapRange overlap = determineOverlap(seqA, seqT);
		if (overlap.length > 0)
		{ // Add overlapping observation of the same point to the point correspondence
			totalOverlap.startA = std::min(totalOverlap.startA, overlap.startB);
			totalOverlap.startB = std::max(totalOverlap.startB, overlap.startB+overlap.length);
			totalOverlap.length += overlap.length;
			pointsC.insert(pointsC.end(), seqA.points.begin()+overlap.startA, seqA.points.begin()+overlap.startA+overlap.length);
			pointsT.insert(pointsT.end(), seqT.points.begin()+overlap.startB, seqT.points.begin()+overlap.startB+overlap.length);
		}
		if (seqAIt == camera.sequences.begin())
			return totalOverlap;
		seqAIt--;
	}

	return totalOverlap;
}

/**
 * Returns a normalisation matrix to normalise the given point sequence by centering and scaling to a distance of sqrt(2)
 */
template<typename Scalar, typename PointIterator>
static Eigen::Matrix<Scalar,3,3> getPointNormalisation(const Range<PointIterator> &sequence)
{
	static_assert(std::is_same<
		typename std::iterator_traits<PointIterator>::value_type, 
		typename Eigen::Matrix<Scalar,2,1>>::value, "Must be a 2D point iterator!");

	// Find Center
	Eigen::Matrix<Scalar,2,1> center = Eigen::Matrix<Scalar,2,1>::Zero();
	for (auto it = sequence.start; it != sequence.end; it++)
		center += it->template cast<Scalar>();
	center /= sequence.length;

	// Find average distance
	Scalar scale = 0;
	for (auto it = sequence.start; it != sequence.end; it++)
		scale += (it->template cast<Scalar>() - center).norm();
	scale /= sequence.length;

	// Build isotropic scale factor
	scale = std::sqrt((Scalar)2.0)/scale;

	// Build normalisation transformations
	Eigen::Matrix<Scalar,3,3> T = Eigen::DiagonalMatrix<Scalar,3>(scale,scale,(Scalar)1.0);
	T.template block<2,1>(0, 2) = -center * scale;
	return T;
}

/**
 * Calculates the fundamental matrix from a two corresponding sequences of points (may be concatenated)
 */
template<typename Scalar, typename PointIterator>
static float calculateFundamentalMatrix(FundamentalMatrix &FM, const Range<PointIterator> &sequenceA, const Range<PointIterator> &sequenceB)
{ // Normalised 8-Point Algorithm
	static_assert(std::is_same<typename std::iterator_traits<PointIterator>::value_type, typename Eigen::Matrix<Scalar,2,1>>::value, "Must be a 2D point iterator!");
	int pointCount = std::min(sequenceA.length, sequenceB.length);
	if (pointCount < 8)
		return 0.0f;

	// Build normalisation transformations
	Eigen::Matrix<Scalar,3,3> TA = getPointNormalisation<Scalar>(sequenceA);
	Eigen::Matrix<Scalar,3,3> TB = getPointNormalisation<Scalar>(sequenceB);

	// Build data matrix
	Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> dataMatrix(pointCount, 9);
	int r = 0;
	for (auto itA = sequenceA.start, itB = sequenceB.start; 
		itA != sequenceA.end && itB != sequenceB.end; itA++, itB++)
	{
		Eigen::Matrix<Scalar,2,1> a = TA.template topLeftCorner<2,2>() * itA->template cast<Scalar>() + TA.template block<2,1>(0,2);
		Eigen::Matrix<Scalar,2,1> b = TB.template topLeftCorner<2,2>() * itB->template cast<Scalar>() + TB.template block<2,1>(0,2);
		Eigen::Matrix<Scalar,1,9> fcoeff;
		fcoeff << a.x()*b.x(), a.y()*b.x(), b.x(), a.x()*b.y(), a.y()*b.y(), b.y(), a.x(), a.y(), 1.0;
		dataMatrix.row(r++) = fcoeff;
	}

	// Solve for fundamental matrix
	float confidence = solveFundamentalMatrix<Scalar>(dataMatrix, FM.matrix);

	// Revert normalisations
	FM.matrix = TB.transpose() * FM.matrix * TA;

	// Frobenius Norm
	//FM.matrix.normalise();
	// Alternative Norm
	/*Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::ComputeFullU | Eigen::ComputeFullV> svd_rank(FM.matrix);
	Eigen::DiagonalMatrix<double, 3> truncatedSV(1, svd_rank.singularValues().y()/svd_rank.singularValues().x(), 0);
	FM.matrix = svd_rank.matrixU() * truncatedSV * svd_rank.matrixV().transpose();*/

	// Verify error
	FM.stats = calculateCorrespondenceStats(sequenceA, sequenceB, FM.matrix);

	return confidence;
}