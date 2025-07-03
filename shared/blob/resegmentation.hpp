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

#ifndef RESEGMENTATION_H
#define RESEGMENTATION_H

#include "parameters.hpp"
#include "blob.hpp"

#include "../util/util.hpp"
#include "../util/eigendef.hpp"

#include <vector>
#include <deque>
#include <string>
#include <cstdint>
#include <cassert>

struct ClusterPeakHint
{
	Eigen::Vector2i peak;
	int scale;
	int certainty;
	int area;
};

// Debug output
struct ScaleCenter
{
	Eigen::Vector2f center;
	int scale;
};
struct BlobEmulImage
{
	std::vector<uint8_t> image;
	std::string label;
};

static std::vector<float> discreteGaussianKernel(float sigma, int radius)
{
	float t = sigma * sigma;
	std::vector<float> kernel(2 * radius + 1);
	for (int i = -radius; i <= radius; i++)
	{
#if defined(_LIBCPP_VERSION)
		#warning Camera Blob Detection Emulation not supported when compiling for libcpp
		kernel[radius + i] = i == 0? 1 : 0; // No blurring
#else
		kernel[radius + i] = std::exp(-t) * std::cyl_bessel_i(std::abs(i), t);
#endif
	}
	return kernel;
}

static std::vector<float> discreteGaussianKernelNormalised(float sigma, int radius)
{
	float t = sigma * sigma;
	float correction = 0.0f;
	std::vector<float> kernel(2 * radius + 1);
	for (int i = -radius; i <= radius; i++)
	{
#if defined(_LIBCPP_VERSION)
		#warning Camera Blob Detection Emulation not supported when compiling for libcpp
		float value = i == 0? 1 : 0;
#else
		float value = std::exp(-t) * std::cyl_bessel_i(std::abs(i), t);
#endif
		kernel[radius + i] = value;
		correction += value;
	}
	correction = 1.0f / correction;
	for (int i = -radius; i <= radius; i++)
		kernel[radius + i] *= correction;
	return kernel;
}

static inline float calcKernel(const uint8_t *image, int stride, Eigen::Vector2i pos, const std::vector<float> &kernel)
{
	int radius = (kernel.size()-1)/2;
	float value = 0;
	for (int yy = -radius; yy <= +radius; yy++)
	for (int xx = -radius; xx <= +radius; xx++)
		value += image[(pos.y()+yy) * stride + (pos.x()+xx)] * kernel[xx+radius] * kernel[yy+radius];
	return value;
}

static inline float calcKernelBorder(const uint8_t *image, int stride, int width, int height, Eigen::Vector2i pos, const std::vector<float> &kernel)
{
	int radius = (kernel.size()-1)/2;
	if (pos.x() >= radius && pos.y() >= radius && pos.x()+radius < width && pos.y()+radius < height)
		return calcKernel(image, stride, pos, kernel);
	float value = 0;
	for (int yy = -radius; yy <= +radius; yy++)
	for (int xx = -radius; xx <= +radius; xx++)
	{
		int x = std::max(0, std::min(width-1, pos.x()+xx));
		int y = std::max(0, std::min(height-1, pos.y()+yy));
		value += image[y * stride + x] * kernel[xx+radius] * kernel[yy+radius];
	}
	return value;
}

static void applyKernel(const uint8_t *image, uint8_t *output, int stride, Bounds2i bounds, const std::vector<float> &kernel)
{
	int radius = (kernel.size()-1)/2;
	for (int y = bounds.minY; y < bounds.maxY; y++)
	for (int x = bounds.minX; x < bounds.maxX; x++)
	{ // Calling calcKernel here makes debug modes unbearably slow since they won't inline, so keep direct
		float value = 0;
		for (int yy = -radius; yy <= +radius; yy++)
		for (int xx = -radius; xx <= +radius; xx++)
			value += image[(y+yy) * stride + (x+xx)] * kernel[xx+radius] * kernel[yy+radius];
		output[y * stride + x] = (uint8_t)value;
	}
}

static void applySeparableKernel(const uint8_t *image, uint8_t *output, uint8_t *temp, int stride, Bounds2i bounds, const std::vector<float> &kernel)
{
	int radius = (kernel.size()-1)/2;
	auto applyKernelHorizontal = [&](const uint8_t *img, uint8_t *out)
	{
		for (int y = bounds.minY; y < bounds.maxY; y++)
		{
			int index = y*stride + bounds.minX;
			for (int i = 0; i < bounds.extends().x(); i++)
			{
				float value = 0;
				for (int j = -radius; j <= +radius; j++)
					value += img[index+j] * kernel[radius+j];
				out[index++] = value;
			}
		}
	};
	auto applyKernelVertical = [&](const uint8_t *img, uint8_t *out)
	{
		for (int x = bounds.minX; x < bounds.maxX; x++)
		{
			int index = bounds.minY*stride + x;
			for (int i = 0; i < bounds.extends().y(); i++)
			{
				float value = 0;
				for (int j = -radius; j <= +radius; j++)
					value += img[index+j*stride] * kernel[radius+j];
				out[index] = value;
				index += stride;
			}
		}
	};
	applyKernelHorizontal(image, temp);
	applyKernelVertical(temp, output);
}

static std::pair<float, int> generateGaussianBlurBoxes(float sigma, std::vector<int> &boxes)
{
	int n = boxes.size();
	float wIdeal = std::sqrt((12*sigma*sigma/n) + 1); // Ideal averaging filter width 
	int wl = (int)std::floor(wIdeal);
	if (!(wl&1)) wl--;
	int wu = wl+2;

	float mIdeal = (12*sigma*sigma - n*wl*wl - 4*n*wl - 3*n) / (-4*wl - 4);
	float m = std::round(mIdeal);

	// Turn from box size to "radius"
	wl = (wl-1)/2;
	wu = (wu-1)/2;

	// Setup boxes
	boxes.clear();
	boxes.resize(m, wl);
	boxes.resize(n, wu);

	float actualSigma = std::sqrt((m*wl*wl + (n-m)*wu*wu - n) / 12);
	int actualSize = m*wl + n*wu;
	return std::make_pair(actualSigma, actualSize);
}

static void applyBoxFilter(const uint8_t *image, uint8_t *output, uint8_t *temp, int stride, Bounds2i bounds, int radius)
{
	float weight = 1.0f / (radius*2 + 1);
	auto applyBoxBlurHorizontal = [&](const uint8_t *img, uint8_t *out)
	{
		for (int y = bounds.minY; y < bounds.maxY; y++)
		{
			int index = y*stride + bounds.minX;
			int indexL = index-radius, indexR = index+radius;
			float value = 0;
			for (int i = indexL; i <= indexR; i++)
				value += img[i];
			out[index++] = value * weight;
			for (int i = 1; i < bounds.extends().x(); i++)
			{
				value = value - img[indexL++] + img[++indexR]; 
				out[index++] = value * weight;
			}
		}
	};
	auto applyBoxBlurVertical = [&](const uint8_t *img, uint8_t *out)
	{
		for (int x = bounds.minX; x < bounds.maxX; x++)
		{
			int index = bounds.minY*stride + x;
			int indexT = index-radius*stride, indexB = index+radius*stride;
			float value = 0;
			for (int i = indexT; i <= indexB; i += stride)
				value += img[i];
			out[index] = value * weight;
			index += stride;
			for (int i = 1; i < bounds.extends().y(); i++)
			{
				indexB += stride;
				value = value - img[indexT] + img[indexB]; 
				indexT += stride;
				out[index] = value * weight;
				index += stride;
			}
		}
	};
	applyBoxBlurHorizontal(image, temp);
	applyBoxBlurVertical(temp, output);
}

static void applyBoxFilters(const uint8_t *image, uint8_t *output, uint8_t *temp, int stride, Bounds2i &bounds, const std::vector<int> &iterations)
{
	for (int i = 0; i < iterations.size(); i++)
	{
		bounds.extendBy(-iterations[i]);
		applyBoxFilter(i == 0? image : output, output, temp, stride, bounds, iterations[i]);
	}
}

static void generateSSR(const SSRParameters params, std::vector<float> &sigmas, std::vector<std::vector<float>> &kernels)
{
	int ssrScales = params.sigmaSteps;
	float ssrCurve = params.sigmaCurve;
	float sigmaMinCurved = std::pow(params.sigmaMin, ssrCurve);
	float sigmaStepCurved = (std::pow(params.sigmaMax, ssrCurve) - sigmaMinCurved)/std::max(1, ssrScales-1);
	float ssrGaussianTimes = 0;
	sigmas.resize(params.sigmaSteps);
	kernels.resize(params.sigmaSteps);
	for (int s = 0; s < ssrScales; s++)
	{
		float sigma = std::pow(sigmaMinCurved + s*sigmaStepCurved, 1.0f/ssrCurve);
		int radius = std::ceil(params.sigmaTrunc * sigma);
		sigmas[s] = sigma;
		kernels[s] = discreteGaussianKernel(sigma, radius);
	}
};

struct MaxArea
{
	uint8_t value;
	int dots;
	Eigen::Vector2f center;
};

/**
 * Check if pos is part of a local maximum area (connected area of the same value with no neighbours within 1 of higher value)
 * Returns the maximum area if it exists
 * Returns value of 0 if the pos has been checked before (whether it was part of a local maximum or not)
 * Returns pixel value with 0 dots if pos is not a local maximum
 *   e.g. if pos or any connected dot with the same value has a higher value within 1
 * Will only consider bounds, but will pass positions 1 past bounds to queryImg
 * BUT: Assumes pos to be in bounds, and handled.size() == bounds.size()
 */
template<typename QF>
static inline MaxArea checkMaximumArea(const QF &queryImg, Bounds2i bounds, Eigen::Vector2i pos, std::vector<bool> &handled, int plateauThreshold, int plateauFill)
{
	assert(bounds.includes(pos));
	assert(handled.size() == bounds.size());
	int stride = bounds.extends().x();
	int index = (pos.y()-bounds.minY) * stride + (pos.x()-bounds.minX);
	if (handled[index])
	{ // Might be part of a blob area already!
		return { 0, 0 };
	}
	handled[index] = true;
	uint8_t value = queryImg(pos);
	uint8_t max = 0;
	int target;
	if (value >= plateauThreshold)
	{ // In a blob plateau, fill whole area unconditionally
		max = 255;
		target = plateauFill;
	}
	else
	{ // Dealing with a smaller blob, make sure we only take the proper local maximum
		for (int yy = -1; yy <= +1; yy++)
		for (int xx = -1; xx <= +1; xx++)
		{
			if (xx == 0 && yy == 0) continue;
			max = std::max(max, queryImg(Eigen::Vector2i(pos.x()+xx, pos.y()+yy)));
		}
		if (value < max)
		{ // Not a maximum
			return { value, 0 };
		}
		if (value > max)
		{ // Single local maximum
			return { value, 1, pos.cast<float>() };
		}
		// Floodfill local maxima area
		target = value;
	}
	// Floodfill full maximum area
	bool foundHigher = false; // Abort when encountering a higher maximum, discard this one, when not in a plateau
	std::deque<Eigen::Vector2i> todo = { pos };
	MaxArea area = { max, 1, pos.cast<float>() };
	while (!todo.empty())
	{
		Eigen::Vector2i c = todo.front();
		todo.pop_front();
		Eigen::Vector2i neighbors[] = {
			Eigen::Vector2i(c.x()-1, c.y()),
			Eigen::Vector2i(c.x(), c.y()-1),
			Eigen::Vector2i(c.x()+1, c.y()),
			Eigen::Vector2i(c.x(), c.y()+1),
		};
		for (auto &p : neighbors)
		{
			if (!bounds.includes(p)) continue;
			index = (p.y()-bounds.minY) * stride + (p.x()-bounds.minX);
			if (handled[index]) continue;
			uint8_t val = queryImg(p);
			if (val > max)
			{ // Tough case, current floodfill is not a valid maximum area, and need to make sure handled for higher value stays false
				// But continue this floodfill first to mark all of the same or lower as handled, else it might be started for this max again
				foundHigher = true;
				continue;
			}
			// Marking even lower values has some annoying implications, but prevents queryImg from being spammed
			// TODO: False positives in local max areas possible due to this!
			// If a higher value maximum area is within reach of a lower value "maximum area"
			// floodfilling the higher first will result in the lower not being able to detect the higher
			// resulting in the lower being falsely accepted as a local max area!
			handled[index] = true;
			if (val >= target)
			{
				area.dots++;
				area.center += p.cast<float>();
				todo.push_back(p);
			}
		}
	}
	if (foundHigher)
		return { value };
	area.center /= area.dots;
	return area;
};

/**
 * Find meaningful peaks that are local peaks across multiple scales
 */
template<bool DEBUG, typename QF>
static std::vector<ClusterPeakHint> FindMaximaHints(
	const uint8_t *image, int stride, int width, int height,
	int scales, const QF &QueryGaussian, Bounds2i bounds,
	const MaximaHintParameters params,
	const Cluster &cluster, std::vector<std::vector<ScaleCenter>> &maximaStages,
	int &localMaxTimeUS, int &maxIterTimeUS, int &finalCheckTimeUS)
{
	if constexpr (DEBUG)
		maximaStages.resize(1+scales);

	TimePoint_t dt0, dt1;

	struct Maxima
	{
		Eigen::Vector2f pos;
		int scale;
		int stable;
		bool sealed;
		int area = 1; // For large blobs that plateau
	};
	std::vector<Maxima> localMaxima;
	// Track maxima blobs as they evolve through the scales

	dt0 = sclock::now();

	// Bounds will be searched for maximas
	std::vector<bool> handled(bounds.size());

	// Find local maxima in base scale
	auto queryBase = [&](Eigen::Vector2i pos)
	{
		return image[pos.y()*stride + pos.x()];
	};
	for (auto &dot : cluster.dots)
	{
		if (!bounds.includes(dot.cast<int>())) continue;
		MaxArea area = checkMaximumArea(queryBase, bounds, dot.cast<int>(), handled,
			params.plateauThreshold, params.plateauThreshold-params.plateauFillOffset);
		if (area.dots > 0)
		{ // Found new local maximum area
			if (area.value >= params.plateauThreshold)
			{ // Big blob with plateau, immediately accept as maximum
				localMaxima.push_back({ area.center, scales-1, scales-1, true, area.dots });
				if constexpr (DEBUG)
					maximaStages[0].emplace_back(area.center, scales-1);
			}
			else
			{ // Small blob, need to verify maximum over multiple scales
				localMaxima.push_back({ area.center, -1 });
				if constexpr (DEBUG)
					maximaStages[0].emplace_back(area.center, 0);
			}
		}
	}
	dt1 = sclock::now();
	localMaxTimeUS += dtUS(dt0, dt1);

	dt0 = sclock::now();
	for (int s = 0; s < scales; s++)
	{ // Try to match local maxima in new blur scale to prior maximas by proximity

		auto queryScale = [&](Eigen::Vector2i pos)
		{
			return QueryGaussian(s, pos);
		};

		// Clear bool array, should be quick
		handled.clear();
		handled.resize(bounds.size());

		// Find local maxima in current blur scale
		int blobsLeft = 0, minStable = std::numeric_limits<int>::max();
		for (auto it = localMaxima.begin(); it != localMaxima.end();)
		{
			auto &blob = *it;
			if (blob.sealed)
			{ // Already considered done
				it++;
				continue;
			}
			Eigen::Vector2i pos = blob.pos.template cast<int>();
			MaxArea area = checkMaximumArea(queryScale, bounds, pos, handled, 255, 255);
			if (area.value == 0)
			{ // Probably already merged unstably with another blob, but not properly (?)
				blob.sealed = true;
				it++;
				continue;
			}
			if (area.value > 0 && area.dots == 0)
			{ // Maximum disappeared or moved, check immediate neighbours
				uint8_t nMaxVal = area.value;
				Eigen::Vector2i nMaxPos;
				for (int yy = std::max(bounds.minY, pos.y()-1); yy < std::min(bounds.maxY, pos.y()+2); yy++)
				for (int xx = std::max(bounds.minX, pos.x()-1); xx < std::min(bounds.maxX, pos.x()+2); xx++)
				{
					Eigen::Vector2i nPos(xx, yy);
					uint8_t nValue = QueryGaussian(s, nPos);
					if (nValue > nMaxVal)
					{
						nMaxVal = nValue;
						nMaxPos = nPos;
					}
				}
				if (nMaxVal > area.value)
				{ // Maximum moved
					area = checkMaximumArea(queryScale, bounds, nMaxPos, handled, 255, 255);
				}
			}
			if (area.value == 0)
			{ // Probably already merged unstably with another blob, but not properly (?)
				blob.sealed = true;
				it++;
				continue;
			}
			if (area.dots == 0)
			{ // Maximum disappeared
				blob.sealed = true;
				it++;
				continue;
			}
			// Still part of local maximum area

			// Update blob
			blob.scale = s;
			blob.stable++;
			blob.pos = (blob.pos * blob.stable + area.center) / (blob.stable+1);

			// Check remaining maximas if they were merged into this
			for (auto itSub = it+1; itSub != localMaxima.end();)
			{
				if (!itSub->sealed)
				{ // Already considered done
					int index = ((int)itSub->pos.y()-bounds.minY) * bounds.extends().x() + ((int)itSub->pos.x()-bounds.minX);
					if (handled[index])
					{ // These blobs have merged, not stable anymore
						blob.stable = 0;
						blob.pos = area.center;
						itSub = localMaxima.erase(itSub);
						continue;
					}
				}
				itSub++;
			}


			blobsLeft++;
			minStable = std::min(minStable, blob.stable);

			it++;
		}

		if constexpr (DEBUG)
		{
			for (auto &blob : localMaxima)
				maximaStages[s+1].emplace_back(blob.pos, blob.scale+1);
		}

		if (minStable >= params.minStable)
		{ // Terminate peak reduction iterations
			if constexpr (DEBUG)
			{
				for (int ss = s+1; ss < scales; ss++)
				{ // Copy to later stages for debugging
					for (auto &blob : localMaxima)
						maximaStages[ss+1].emplace_back(blob.pos, blob.scale+1);
				}
			}
			break;
		}
	}
	dt1 = sclock::now();
	maxIterTimeUS += dtUS(dt0, dt1);

	dt0 = sclock::now();
	std::vector<ClusterPeakHint> hints;
	for (const auto &maxima : localMaxima)
	{
		if (maxima.scale < params.minScale)
			continue; // Insignificant local peak that was sealed quickly
		int certainty = maxima.scale * QueryGaussian(maxima.scale, maxima.pos.template cast<int>());
		hints.push_back({ maxima.pos.template cast<int>(), maxima.scale, certainty, maxima.area });
	}
	dt1 = sclock::now();
	finalCheckTimeUS += dtUS(dt0, dt1);

	return hints;
}

/**
 * Resegment cluster based on hints by iterative competitive floodfilling from those hints
 * Each hint will have its own targets and limits for floodfilling
 * The goal is to resegment the area in bounds to the new clusters, potentially even outside the original clusters bounds
 * Then, based on the floodfilling results, clusters will be filtered and returned
 */
template<bool DEBUG>
static std::vector<Cluster> ResegmentCluster(
	const uint8_t *image, int stride, int width, int height,
	Bounds2i bounds, const Cluster &cluster, std::vector<ClusterPeakHint> &hints,
	const BlobProcessingParameters &params,
	std::vector<BlobEmulImage> &floodfillStages, std::vector<BlobEmulImage> &resegmentationMasks, std::vector<ScaleCenter> &iterativeCenters)
{
	if (hints.empty())
	{ // Keep cluster as-is
		return {};
	}

	// Sort peaks by certainty (only necessary for newTarget to use second largest)
	std::sort(hints.begin(), hints.end(),
		[](const auto &a, const auto &b){
		return a.certainty > b.certainty;
	});

	int bstride = bounds.extends().x();
	auto getIndex = [&bounds, &bstride](Eigen::Vector2i pos) -> int
	{
		return (pos.y()-bounds.minY)*bstride + (pos.x()-bounds.minX);
	};
	auto getImgIndex = [stride](Eigen::Vector2i pos)
	{
		return pos.y()*stride + pos.x();
	};

	// Prepare temporary data for each sub-cluster
	struct ClusterState
	{
		// Temporary state for each iteration
		std::vector<bool> handled; // True: New border or new claim
		std::vector<Eigen::Vector2i> border, oldBorder;
		int conflicts, claims;
		bool hitDisabled, hitAreaLimit;

		// Classification of cluster
		bool needsSubsteps; // Temporary state requesting resolution using substeps before being frozen or even disabled
		bool frozen;	// Stage at which further dots don't affect the accuracy of the blob center
		bool disabled;	// Stage at which cluster doesn't take part in competitive floodfilling anymore

		// Range of pixel values added to the cluster in total
		int maxValue, minValue;

		// Number of pixels allowed to claim
		int remainingBudget;

		// Min target before it's disabled
		int minTarget;

		// Calculating center accurately
		int totalWeight; // Weight of pixels contributing to center
		int centroidWeight; // Weight of pixels contributing to center
		int centroidPixels; // Number of pixels contributing to center

		struct ScaleCircumference
		{
			int count;
			Eigen::Vector2f center = Eigen::Vector2f::Zero();
			int weight;
		};
		std::vector<ScaleCircumference> scaleCircumferences;
	};

	// Prepare list of temporary state and final struct for new sub-clusters
	std::vector<ClusterState> clusterState(hints.size());
	std::vector<Cluster> subClusters(hints.size());

	// Prepare temporary data for floodfilling
	typedef uint8_t ClusterIndex;
	const ClusterIndex InvalidCluster = 255;
	std::vector<ClusterIndex> assigned(bounds.size(), InvalidCluster);
	std::vector<ClusterIndex> claimed(bounds.size(), InvalidCluster);
	std::vector<Eigen::Vector2i> claims;

	// Init debug output
	std::vector<std::vector<uint8_t>> ffStages;
#ifdef BLOB_EMULATION
	if constexpr (DEBUG)
	{
		// As a sort of header
		floodfillStages.emplace_back(std::vector<uint8_t>(image, image+(stride*height)), asprintf_s("Cluster with %d points", cluster.ptCnt));
		ffStages.resize(clusterState.size());

		// One for unconflicted claims, and one for each clusters conflicted claims
		if (resegmentationMasks.size() <= 1+hints.size())
			resegmentationMasks.resize(1+hints.size(), BlobEmulImage{ std::vector<uint8_t>(stride*height) });
	}
#endif

	auto iterativeFloodfill = [&](int i, int target, int upperTarget)
	{ // Modifies claimed, claims
		const auto &hint = hints[i];
		auto &state = clusterState[i];

		// Reset temporary status for this iteration
		state.handled.clear();
		state.handled.resize(bounds.size(), false);
		state.conflicts = 0;
		state.claims = 0;
		state.hitAreaLimit = false;
		state.hitDisabled = false;
		// It's fine to reset it only here, since any OTHER clusterStates accessed here has been reset in their iterativeFloodfill before

		// Keep old border around incase there will be a conflict, take it as seed for floodfilling
		state.oldBorder.clear();
		state.oldBorder.swap(state.border);
		std::deque<Eigen::Vector2i> todo(state.oldBorder.begin(), state.oldBorder.end());
		target = std::max(target, state.minTarget);
		while (!todo.empty())
		{
			Eigen::Vector2i pos = todo.front();
			todo.pop_front();

			int index = getIndex(pos);
			if (state.handled[index]) continue; // Handled pixel for this cluster already
			if (assigned[index] == i) continue; // Claimed for this cluster in a prior iteration
			state.handled[index] = true;

			// Check at the scale of the peak, so competing peaks may use different base values
			int value = image[getImgIndex(pos)];
			if (value >= upperTarget)
			{ // If it wasn't assigned already, it's part of a blob that's not competing, prevent interaction
				// This happens when bounds are overlapping but the initial clusters don't
				// Can happen normally, or due to bounds being artificially extended to let small blobs expand

				// TODO: Doesn't behave well in the first iterations in the following case:
				// - when blob has multiple peaks seperated by valleys with contrast higher than subStep
				// - it encounters a conflict in the first iteration, immediately entering subSteps
				// -> doesn't get the other peaks within the first iteration, in the following, it thinks it's another blob
				/* { // Find workaround?
					state.conflicts++;
					continue;
				} */
			}
			if (value >= target)
			{ // Claim for this peak
				if (++state.claims >= state.remainingBudget)
				{ // Abort - the current step won't be accepted
					state.hitAreaLimit = true;
					return;
				}
				if (assigned[index] != InvalidCluster)
				{ // Already assigned to another cluster before
					state.conflicts++;
					clusterState[assigned[index]].conflicts++;
					if (clusterState[assigned[index]].disabled)
						state.hitDisabled = true;
					// If this is a subStep and the other is not participating in it (=> already frozen)
					//  it will just silently ignore the conflict - this is intended, this cluster would still be frozen after this
					continue;
				}
				int comp = claimed[index];
				if (comp != InvalidCluster)
				{ // Compete with claim of other cluster
					state.conflicts++;
					clusterState[comp].conflicts++;
					float dist = (hint.peak - pos).squaredNorm(), distComp = (hints[comp].peak - pos).squaredNorm();
					if (dist / hints[i].certainty < distComp / hints[comp].certainty)
					{ // Override claim
						claimed[index] = i;
					}
				}
				else
				{ // New claim
					claimed[index] = i;
					claims.push_back(pos);
				}
				// Add neighbours to todo
				Eigen::Vector2i neighbors[] = {
					Eigen::Vector2i(pos.x()-1, pos.y()),
					Eigen::Vector2i(pos.x(), pos.y()-1),
					Eigen::Vector2i(pos.x()+1, pos.y()),
					Eigen::Vector2i(pos.x(), pos.y()+1),
				};
				for (auto &n : neighbors)
				{
					if (bounds.includes(n))
						todo.push_back(n);
				}
			}
			else if (value >= state.minTarget)
			{ // Mark as new border
				state.border.push_back(pos);
			}
		}
	};

	auto consumeClaims = [&](int scale, int target)
	{
		for (auto &claim : claims)
		{
			int index = getIndex(claim);
			int i = claimed[index];
			claimed[index] = InvalidCluster;
			auto &state = clusterState[i];
			if (state.disabled || state.needsSubsteps) continue;
			// Update assignment
			assigned[index] = i;
			// Update new sub-cluster stats
			auto &subCluster = subClusters[i];
			subCluster.dots.push_back(claim.cast<uint16_t>());
			subCluster.bounds.include(Bounds2<uint16_t>(claim.x(), claim.y(), claim.x()+1, claim.y()+1));
			int px = getImgIndex(claim);
			int value = image[px];
			state.minValue = std::min(state.minValue, value);
			value = std::max(0, value-params.floodfilling.threshold.min);
			if constexpr (DEBUG)
			{
				ffStages[i][px] = state.frozen? 150 : 200;
				if (state.frozen) resegmentationMasks[i+1].image[px] = 255;
				else resegmentationMasks[0].image[px] = 255;
			}
			state.totalWeight += value;
			if (state.frozen)
			{ // Don't affect centroid when frozen
				continue;
			}
			subCluster.centroid += (claim * value).cast<float>();
			state.centroidWeight += value;
			state.centroidPixels++;
			if constexpr (DEBUG)
			{
				state.scaleCircumferences.resize(scale+1);
				auto &scaleCirc = state.scaleCircumferences[scale];
				scaleCirc.count++;
				scaleCirc.center += (claim * value).cast<float>();
				scaleCirc.weight += value;
			}
		}
		claims.clear();
	};

	int prevTarget = image[getImgIndex(hints[0].peak)];
	int minTarget = 255;
	if (hints.size() > 1)
		prevTarget = std::min(prevTarget, image[getImgIndex(hints[1].peak)] + params.floodfilling.threshold.step);

	for (int i = 0; i < hints.size(); i++)
	{ // Init cluster state
		auto &state = clusterState[i];
		state.border = { hints[i].peak };
		state.maxValue = image[getImgIndex(hints[i].peak)];
		// TODO: Slightly wrong - peak position is at a higher scale in the pyramid, so this might not be the actual max value
		// Maybe store max value as peaks get moved/merged on different levels
		state.minValue = state.maxValue;
		// Adaptive minimum threshold for each cluster
		const int lerpArea = 10;
		if (hints[i].area >= lerpArea)
		{ // Plateau reached, don't need much more, aim for edge value used for refinement anyway
			state.minTarget = params.refinement.targetEdgeVal;
		}
		else
		{ // Need to grab as much as needed, but don't expand into surrounding background
			auto &ff = params.floodfilling;
			state.minTarget = ff.threshold.min + std::max(0, (int)((state.maxValue-ff.threshold.min)/ff.blob.peakMinimumRatio));
			state.minTarget = (hints[i].area*params.refinement.targetEdgeVal + (lerpArea-hints[i].area)*state.minTarget) / lerpArea;
		}
		minTarget = std::min(minTarget, state.minTarget);
		// Limit area to expand into
		state.remainingBudget = hints[i].area*params.floodfilling.blob.limitExpansionFactor + state.maxValue*params.floodfilling.blob.limitExpansionBase;
	}

	for (int it = 0;; it++)
	{
		int curTarget = std::max(minTarget, prevTarget-params.floodfilling.threshold.step);
		if (curTarget+params.floodfilling.threshold.acceptableLoss > prevTarget)
			break; // Not worth it, only going to get a few more pixels

		// Iteratively floodfill all blobs down to curTarget
		for (int i = 0; i < clusterState.size(); i++)
		{
			if (!clusterState[i].disabled)
				iterativeFloodfill(i, curTarget, subClusters[i].ptCnt == 0? 255 : prevTarget);
		}

		// Update debug images
		if constexpr (DEBUG)
		{
			for (auto &ffStage : ffStages)
			{
				ffStage.clear();
				ffStage.resize(stride*height);
			}
			for (int y = bounds.minY; y < bounds.maxY; y++)
			for (int x = bounds.minX; x < bounds.maxX; x++)
			{
				Eigen::Vector2i pos(x, y);
				int i = assigned[getIndex(pos)];
				if (i != InvalidCluster)
					ffStages[i][getImgIndex(pos)] = 255; // Assigned in previous step
			}
		}

		// Pick clusters for conflict resolution by floodfilling in finer sub-steps
		for (auto &state : clusterState)
		{
			if (state.disabled) continue;
			if (state.conflicts > 0 && !state.frozen)
			{ // Re-do in finer substeps
				state.needsSubsteps = true;
				continue;
			}
			if (state.hitAreaLimit)
			{ // Re-do in finer substeps
				state.needsSubsteps = true; // Do substeps for increased accuracy
				//state.disabled = true; // Alternatively, just ignore
				continue;
			}
			if (state.hitDisabled)
			{ // Re-do in finer substeps
				state.needsSubsteps = true; // Do substeps for increased accuracy
				//state.disabled = true; // Alternatively, just ignore
				continue;
			}
			state.remainingBudget -= state.claims;
		}

		// Accept claims of clusters !disabled && !needsSubsteps
		consumeClaims(it, curTarget);

		// Gather clusters with for conflict resolution
		std::vector<int> conflictedClusters; // All of them will be frozen or disabled by the end of this loop
		for (int i = 0; i < clusterState.size(); i++)
		{
			auto &state = clusterState[i];
			if (state.needsSubsteps)
			{
				state.needsSubsteps = false;
				conflictedClusters.push_back(i);
				// Reset border to prior state (seed for floodfilling)
				state.border.swap(state.oldBorder);
				continue;
			}
			if (curTarget < state.minTarget)
				state.disabled = true;
		}

		// Apply floodfilling substep if there are conflicted clusters
		int subStep = std::max(params.floodfilling.threshold.minSubStep, (int)std::log(prevTarget)*2);
		int subTarget = prevTarget-subStep;
		while (!conflictedClusters.empty() && subTarget >= curTarget)
		{
			// Floodfill all clusters in lockstep
			for (int i : conflictedClusters)
			{
				if (!clusterState[i].disabled)
					iterativeFloodfill(i, subTarget, subClusters[i].ptCnt == 0? 255 : prevTarget);
			}
			// Note: Cannot only floodfill non-frozen ones, as a cluster could be conflicting with two other clusters
			// As such, a newly frozen cluster is still relevant in making sure other clusters are frozen reliably
			// However, disabled clusters cannot proceed with floodfilling, as their size might expand uncontrollably
			// Thus, when a disabled cluster is touched (conflicted with), it will forcefully disable the conflicted cluster, too
			// This is fine as a disabled cluster indicates a high base value and the expansion is likely undesired anyway

			for (int i : conflictedClusters)
			{
				auto &state = clusterState[i];
				if (state.conflicts)
					state.frozen = true;
				if (state.hitAreaLimit)
					state.disabled = true;
				if (state.hitDisabled)
					state.disabled = true;
			}

			// Update all clusters of substep, and clear claims
			consumeClaims(it, subTarget); // Same iteration, only used for non-conflicted data anyway

			bool continueSub = false, finishSub = false;
			for (int i : conflictedClusters)
			{
				auto &state = clusterState[i];
				if (subTarget < state.minTarget)
					state.disabled = true;
				// Determine next substep
				if (!state.disabled && !state.frozen)
					continueSub = true;
				if (!state.disabled)
					finishSub = true;
			}

			prevTarget = subTarget;
			if (continueSub) // Next substep
				subTarget = subTarget-subStep;
			else if (prevTarget != curTarget && finishSub)
				subTarget = curTarget; // Finish frozen ones to latest step
			else // Only disabled clusters left or already at latest step
			 	break;

#ifdef BLOB_EMULATION
			if (DEBUG && subTarget >= curTarget)
			{
				for (int i : conflictedClusters)
				{
					if (subClusters[i].dots.empty()) continue;
					floodfillStages.emplace_back(ffStages[i], asprintf_s("Target %d (x) Cluster %d (%s)##%d",
						subTarget, i, clusterState[i].disabled? "Disabled" : (clusterState[i].frozen? "Frozen" : "Active"), clusterState[i].centroidWeight));
				}
			}
#endif
		}

#ifdef BLOB_EMULATION
		if constexpr (DEBUG)
		{
			for (int i = 0; i < clusterState.size(); i++)
			{
				if (subClusters[i].dots.empty()) continue;
				floodfillStages.emplace_back(ffStages[i], asprintf_s("Target %d Cluster %d (%s)##%d",
					curTarget, i, clusterState[i].disabled? "Disabled" : (clusterState[i].frozen? "Frozen" : "Active"), clusterState[i].centroidWeight));
			}
		}
#endif

		prevTarget = curTarget;

		int clustersLeft = 0;
		for (int i = 0; i < clusterState.size(); i++)
		{
			if (!clusterState[i].disabled)
				clustersLeft++;
		}
		if (clustersLeft == 0) break;
	}

	if constexpr (DEBUG)
	{
		for (int i = 0; i < clusterState.size(); i++)
		{
			auto &state = clusterState[i];
			for (int s = 0; s < state.scaleCircumferences.size(); s++)
			{
				auto &scaleCirc = state.scaleCircumferences[s];
				if (scaleCirc.weight > 0)
					iterativeCenters.emplace_back(scaleCirc.center / scaleCirc.weight, s);
			}
		}
	}

	// Finalise sub-clusters
	std::vector<Cluster> resegmentedClusters;
	resegmentedClusters.reserve(hints.size());
	// For potential edge-fetching
	Bounds2i fetchArea;
	std::vector<int> indices(clusterState.size(), -1);
	for (int i = 0; i < clusterState.size(); i++)
	{
		auto &state = clusterState[i];
		auto &blob = subClusters[i];
		if (state.centroidPixels < params.filtering.minContributingPixels)
			continue; // Not enough unconflicted pixels to support this blob accurately
		float contrast = state.maxValue-state.minValue - (state.minValue-state.minTarget) + state.minTarget-params.floodfilling.threshold.min;
		if (contrast < params.filtering.minContrastValue)
			continue; // Probably just insignificant peak in bright area - ignore
		blob.ptCnt = blob.dots.size();
		blob.centroid /= state.centroidWeight;
		blob.size = std::sqrt((float)blob.ptCnt / PI);
		blob.contrast = contrast;
		blob.certainty = hints[i].certainty;
		blob.reliability = state.centroidPixels;
		blob.circularity = 0; // TODO
		blob.weight = state.totalWeight;
		blob.value = blob.contrast + blob.weight/20;
		// NOTE: More or less mirrored with blob.cpp;;handleClusterSingle
		if (blob.ptCnt >= params.classification.blobRefinementThreshold)
		{ // Want to fetch edges
			fetchArea.include(blob.bounds.cast<int>());
			indices[i] = resegmentedClusters.size();
		}
		resegmentedClusters.push_back(std::move(blob));
	}
	if (fetchArea.size() > 0)
	{
		fetchArea.overlapWith(bounds.extendedBy(-1));
		for (int y = fetchArea.minY; y < fetchArea.maxY; y++)
		for (int x = fetchArea.minX; x < fetchArea.maxX; x++)
		{
			int index = getIndex(Eigen::Vector2i(x, y));
			int id = assigned[index];
			if (id == InvalidCluster || indices[id] < 0)
				continue;
			// Assigned to a relevant cluster, check if it's the edge
			// only InvalidCluster counts, not conflicted edges with other blobs
			if (assigned[index-1] == InvalidCluster || assigned[index+1] == InvalidCluster
			|| assigned[index-bstride] == InvalidCluster || assigned[index+bstride] == InvalidCluster)
			{
				resegmentedClusters[indices[id]].edge.emplace_back(x, y);
			}
		}
	}

	return resegmentedClusters;
}

#endif // RESEGMENTATION_H