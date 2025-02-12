/**
AsterTrack Optical Tracking System
Copyright (C)  2025 Seneral <contact@seneral.dev> and contributors

MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef MATCHING_PARAMS_H
#define MATCHING_PARAMS_H

struct MatchingParameters
{
	float uncertainty;
	float primAdvantage;
	float compAdvantage;
	int conservativeLevel;
	int competeRange = 1;

	MatchingParameters squared() const
	{
		MatchingParameters params = *this;
		params.uncertainty *= params.uncertainty;
		params.primAdvantage *= params.primAdvantage;
		params.compAdvantage *= params.compAdvantage;
		return params;
	}
};

#endif

#ifdef INCLUDE_MATCHING_PARAMS_ONLY
#undef INCLUDE_MATCHING_PARAMS_ONLY
#else

#ifndef MATCHING_H
#define MATCHING_H

#include <limits>
#include <vector>
#include <utility>
#include <cassert>

// Matching between two sets based on best candidates rated by a value (lower is better fit)

struct NoCtx { };
template<typename Context = NoCtx>
struct MatchCandidate {
	bool invalid = false;
	int index = -1;
	float value = std::numeric_limits<float>::max();
	Context context;

	bool valid() const { return !invalid && index >= 0; }
	float getValue() const { return value; }
};

struct TargetMatch
{
	int source = -1;
	int priority = -1;
};

template<typename Context = NoCtx>
struct WeightedMatch
{
	bool invalid = false;
	int index = -1;
	float value = 0;
	int weight = 0;
	Context context;

	bool valid() const { return !invalid && index >= 0; }
	float getValue() const { return weight > 0? value/weight : std::numeric_limits<float>::max(); }

	bool merge(const WeightedMatch &other)
	{
		if (other.index != index) return false;
		value = (value*weight + other.value*other.weight)/(weight+other.weight);
		weight += other.weight;
		// Context? Can't merge
		return true;
	}
};
template<typename Context = NoCtx, typename CandidateT = MatchCandidate<>, int N = 2>
struct MatchCandidates {
	using Candidate = CandidateT;
	using Dynamic = std::conditional_t<N <= 0, std::true_type, std::false_type>;
	using Container = std::conditional_t<Dynamic::value, std::vector<Candidate>, std::array<Candidate, (std::size_t)N>>;
	Context context;
	Container matches;
};

/**
 * Record candidates if it is better than existing matches and returns the candidate for further modifications
 */
template<typename Candidates>
static typename Candidates::Candidate& recordMatchCandidate(Candidates &matches, typename Candidates::Candidate &candidate, int maxMatches = -1)
{
	int i = 0;
	for (; i < matches.matches.size(); i++)
	{
		if (candidate.getValue() < matches.matches[i].getValue())
		{
			std::swap(matches.matches[i], candidate);
			break;
		}
	}
	for (int j = i+1; j < matches.matches.size(); j++)
		std::swap(matches.matches[j], candidate);
	if constexpr (Candidates::Dynamic::value)
	{
		if (matches.matches.size() < maxMatches)
		{
			i = matches.matches.size();
			matches.matches.push_back(candidate);
		}
	}
	if (i < matches.matches.size())
		return matches.matches[i];
	return candidate;
}

template<typename Candidates>
static typename Candidates::Candidate& integrateWeightedMatch(Candidates &matches, typename Candidates::Candidate &candidate, int maxMatches = -1)
{
	for (int i = 0; i < matches.matches.size(); i++)
	{
		if (!matches.matches[i].merge(candidate)) continue;
		// Was able to merge, now reposition
		int j = 0;
		for (; j < i; j++)
		{
			if (matches.matches[i].getValue() < matches.matches[j].getValue())
			{
				std::swap(matches.matches[i], matches.matches[j]);
				break;
			}
		}
		for (int k = j+1; k < i; k++)
			std::swap(matches.matches[i], matches.matches[k]);
		return matches.matches[j];
	}
	// New candidate, treat normally
	return recordMatchCandidate(matches, candidate, maxMatches);
}

/**
 * Resolve conflicts among match candidates and only leave the best matches
 * This makes some optimistic assumptions and may lead to false positives otherwise
 * E.g. For most candidates, the real match is always available, though it might not be the primary match
 */
template<typename Source>
static int resolveMatchCandidatesOptimistic(std::vector<Source> &sourceMatches, int targetCount)
{
	int matches = sourceMatches.size();
	thread_local std::vector<int> targetMatches;
	targetMatches.clear();
	targetMatches.resize(targetCount, -1);
	for (int i = 0; i < sourceMatches.size(); i++)
	{
		int index = i;
		while (true)
		{ // Try to register current primary match of candidate at index
			Source &source = sourceMatches[index];
			if (source.matches.empty())
				break; // Incase a dynamic container is used
			auto &match = source.matches.front();
			if (!match.valid()) break;
			int target = match.index;
			match.invalid = false;

			int contestingSource = targetMatches[target];
			if (contestingSource < 0)
			{ // Match is not (yet) contested
				targetMatches[target] = index; // Register for match
				break;
			}
			// Else, a match was already registered, resolve which one is better

			Source &contSource = sourceMatches[contestingSource];
			int problematicMatchInd = index;
			if (match.value < contSource.matches[0].value)
			{ // Replace contesting match
				problematicMatchInd = contestingSource;
				targetMatches[target] = index;
			}

			// Primary match wasn't accepted for one of them, handle next
			Source &problematicMatch = sourceMatches[problematicMatchInd];
			if (problematicMatch.matches[1].index >= 0)
			{ // Try registering it's secondary match next
				problematicMatch.matches[0] = problematicMatch.matches[1];
				problematicMatch.matches[1].invalid = true;
				index = problematicMatchInd;
			}
			else
			{ // No secondary match for it, invalidate
				problematicMatch.matches[0].invalid = true;
				matches--;
				break;
			}
		}
	}
	return matches;
}

/**
 * Resolve conflicts among match candidates and only leave the best matches
 * A match is advantaged over another if it's value (plus a minimum uncertainty) is significantly smaller than the others
 * Being competitively-advantaged (over a competing candidate) is a hard requirement
 * Being primary-advantaged (over the secondary match) is a variable requirement
 * Based on conservativeLevel, different requirements apply to a potential secondary match:
 * 0: the primary match must be primary-advantaged over the secondary match
 * 1: same as 0, or any secondary match must be a competitively- and primary-advantaged match of some other candidate
 * 2: same as 0, or any secondary match must be a competitively- and (towards this primary's match) primary-advantaged match of some other candidate
 * 	TODO: Currently, 2 is the same as 3. Needs values that can't be calculated in this method.
 * 3: same as 0, or any secondary match must be a competitively-advantaged primary match of some other candidate
 *
 * Additionally, competeRange will require each primary match to compete with non-primary matches of other sources
 */
template<typename Source>
static int resolveMatchCandidates(std::vector<Source> &sourceMatches, int targetCount, const MatchingParameters &params)
{
	thread_local std::vector<TargetMatch> targetMatches;
	targetMatches.clear();
	targetMatches.resize(targetCount);

	for (int s = 0; s < sourceMatches.size(); s++)
	{
		Source &source = sourceMatches[s];
		if (source.matches.empty())
			continue; // Incase a dynamic container is used
		auto &priMatch = source.matches.front();
		if (priMatch.index < 0)
			continue; // No match found
		priMatch.invalid = false;

		// Enter non-primary matches as competitors to other primary matches as well (simplified, only need best)
		for (int m = 1; m < params.competeRange && m < source.matches.size(); m++)
		{
			auto &match = source.matches[m];
			if (!match.valid()) continue;
			auto &cont = targetMatches[match.index];
			if (cont.source < 0)
			{ // Match is not (yet) contested, claim as best, even if it's not our best match
				cont = { s, m };
				continue;
			}
			auto &contMatch = sourceMatches[cont.source].matches[cont.priority];
			if (match.value < contMatch.value)
			{
				targetMatches[match.index] = { s, m };
				contMatch.invalid = true;
			}
		}

		int target = priMatch.index;
		int contestingSource = targetMatches[target].source;
		if (contestingSource < 0)
		{ // Match was already claimed as best
			targetMatches[priMatch.index] = { s, 0 };
			continue;
		}
		// Else, a match was previously registered, which means it's *a* match for both candidates - need to compete with it

		Source &contSource = sourceMatches[contestingSource];
		int contPrio = targetMatches[target].priority;
		auto &contMatch = contSource.matches[contPrio];
		if (contMatch.value > (priMatch.value + params.uncertainty) * params.compAdvantage)
		{ // New match is advantaged over existing match, replace
			targetMatches[target] = { s, 0 };
			contMatch.invalid = true;
		}
		else if (priMatch.value > (contMatch.value + params.uncertainty) * params.compAdvantage)
		{ // Existing match prevails as advantaged over new match
			priMatch.invalid = true;
		}
		else
		{ // Neither of them prevails
			contMatch.invalid = true;
			priMatch.invalid = true;
			// Make sure match has best candidate registered, even if it's not a valid match
			// So a future competitor would have to compete favourably against best candidate
			if (priMatch.value < contMatch.value)
				targetMatches[target] = { s, 0 };
		}
	}
	int matchCnt = 0;
	for (int i = 0; i < sourceMatches.size(); i++)
	{
		Source &source = sourceMatches[i];
		if (source.matches.empty())
			continue; // Incase a dynamic container is used
		auto &match = source.matches.front();
		if (!match.valid())
			continue; // No match found, or already discarded
		assert(targetMatches[match.index].source == i);
		bool primAdvantaged = (source.matches.size() < 2) || (source.matches[1].index < 0) || source.matches[1].value > (match.value + params.uncertainty) * params.primAdvantage;
		if (primAdvantaged)
		{ // It is both competitively and primarily advantaged and thus a valid match
			matchCnt++;
			continue;
		}
		if (params.conservativeLevel == 0)
		{ // Very conversative, so no other options, discard match
			match.invalid = true;
			continue;
		}
		// Require secondary match to have been picked by another source as it's primary match
		assert(source.matches[1].index >= 0);
		auto &secTarget = targetMatches[source.matches[1].index];
		if (secTarget.source < 0 || secTarget.priority != 0)
		{ // Not primarily-advantaged over secondary match, and no other (primary) claims on it to make deciding easier. Discard match
			match.invalid = true;
			continue;
		}
		Source &secSource = sourceMatches[secTarget.source];
		bool secCompAdv = secSource.matches[0].index == source.matches[1].index;
		if (!secCompAdv)
		{ // Alternate match was not able to claim secondary match for itself as primary. Discard match
			match.invalid = true;
			continue;
		}
		if (params.conservativeLevel == 3)
		{ // Conditions for params.conservativeLevel == 3 have been met
			matchCnt++;
			continue;
		}
		if (params.conservativeLevel == 1)
		{ // Make sure that match is primarily advantaged itself
			bool secPrimAdv = (secSource.matches.size() < 2) || (secSource.matches[1].index < 0) || secSource.matches[1].value > (secSource.matches[0].value + params.uncertainty) * params.primAdvantage;
			if (!secPrimAdv)
			{ // Wasn't able to confirm
				match.invalid = true;
				continue;
			}
		}
		/* else if (params.conservativeLevel == 2)
		{ // Check if that match is primarily advantaged over this candidates' primary match specifically
			bool secPrimAdv = [value of secMatch.matches[0].index to match.index]
				> (match.value + params.uncertainty) * params.primAdvantage;
			// TODO: Can't calculate value here, could only confirm
			if (!secPrimAdv)
			{ // Wasn't able to confirm

			}
		} */

		// Conditions for all have been met
		matchCnt++;
	}
	int matchesVerify = 0;
	for (int i = 0; i < sourceMatches.size(); i++)
	{
		Source &source = sourceMatches[i];
		if (source.matches.empty())
			continue; // Incase a dynamic container is used
		if (source.matches.front().valid())
			matchesVerify++;
	}
	assert(matchCnt == matchesVerify);
	return matchCnt;
}


#endif // MATCHING_H
#endif // ndef INCLUDE_MATCHING_PARAMS_ONLY