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

#include "blob.hpp"
#include "resegmentation.hpp"

#include "../util/util.hpp"
#include "util/log.hpp"

#include <cmath>


std::vector<Vector2<float>> edgeRefined;
std::vector<bool> edgeOutliers;

PrecomputedKernels precomputeKernels(const BlobProcessingParameters &params)
{
	PrecomputedKernels kernels;
	kernels.base = discreteGaussianKernelNormalised(params.base.sigma, params.base.radius);
	std::vector<float> ssrSigmas;
	generateSSR(params.ssr, ssrSigmas, kernels.ssr);
	return kernels;
}

std::vector<Cluster> handleCluster(Cluster &&cluster, const uint8_t *frame, uint32_t stride, uint32_t width, uint32_t height,
	const BlobProcessingParameters &params, const PrecomputedKernels &kernels,
	int &blurTimeUS, int &maximaTimeUS, int &localMaxTimeUS, int &maxIterTimeUS,
	int &finalCheckTimeUS, int &resegTimeUS, int &refineTimeUS)
{
	std::vector<Cluster> subClusters;

	if (cluster.dots.size() <= params.classification.blobTinyThreshold)
	{ // Almost certainly a single, tiny blob
		return subClusters;
	}

	TimePoint_t dt0, dt1;

	if (cluster.dots.size() <= params.classification.resegmentationThreshold)
	{
		const uint8_t *baseImage = frame;
		Bounds2i baseBounds(0, 0, width, height);
		const int range = 1; // Range beyond bounds that checkMaximumArea accesses baseImage in

		Bounds2i bounds = cluster.bounds.cast<int>();
		if (bounds.size() < params.filtering.minContributingPixels*2)
			bounds.extendBy(params.floodfilling.blob.allowBoundsExpansion);
		// TODO: This may connect clusters for resegmentation even though they aren't competing, meaning one blob may consume area of another - bad!

		dt0 = sclock::now();

		thread_local std::vector<uint8_t> image;
		thread_local std::vector<uint8_t> temp;
		if (params.base.blur)
		{ // Blur base image in bounds of cluster
			// Make sure to allocate image buffers (once per thread)	
			image.resize(stride*height);
			temp.resize(stride*height);
			// Update bounds to be blurred
			bounds.extendBy(range); // Add range since it needs to be blurred
			bounds.overlapWith(baseBounds.extendedBy(-params.base.radius));
			applySeparableKernel(frame, image.data(), temp.data(), stride, bounds, kernels.base);
			baseImage = image.data();
			bounds.extendBy(-range); // Remove range to make it implicit
		}
		else
		{ // Simply limit bounds by baseBounds, accounting for range
			bounds.overlapWith(baseBounds.extendedBy(-range));
		}

		dt1 = sclock::now();
		blurTimeUS += dtUS(dt0, dt1);

		/**
		* Find peaks in clusters to detect merged blobs
		* Follow local maximas up the scale tree to determine if they are real blobs or noise
		* Applies per previously segmented cluster, expecting one or more real blobs in that cluster
		*/

		dt0 = sclock::now();

		auto calcGaussian = [&](int scale, Eigen::Vector2i dot) -> uint8_t
		{
			return (uint8_t)calcKernelBorder(frame, stride, width, height, dot, kernels.ssr[scale]);
		};

		// Find maxima hints
		std::vector<std::vector<ScaleCenter>> maximaStages; // Unused
		std::vector<ClusterPeakHint> hints = FindMaximaHints<false>(
			baseImage, stride, width, height,
			kernels.ssr.size(), calcGaussian, bounds,
			params.maximaHints,
			cluster, maximaStages,
			localMaxTimeUS, maxIterTimeUS, finalCheckTimeUS);
		

		dt1 = sclock::now();
		maximaTimeUS += dtUS(dt0, dt1);

		if (hints.size() > 1 || (!hints.empty() && params.classification.resegmentSingleClusters))
		{
			/**
			* Re-segment clusters based on hints for local peaks
			* Use competitive floodfilling from those peaks to find maximum area without conflicts with nearby merged blobs
			*/

			dt0 = sclock::now();

			std::vector<BlobEmulImage> floodfillStages; // Unused
			std::vector<BlobEmulImage> resegmentationMasks; // Unused
			std::vector<ScaleCenter> iterativeCenters; // Unused
			subClusters = ResegmentCluster<false>(
				baseImage, stride, width, height,
				bounds, cluster, hints, params,
				floodfillStages, resegmentationMasks, iterativeCenters);

			dt1 = sclock::now();
			resegTimeUS += dtUS(dt0, dt1);
		}
		else
		{ // Not resegmenting, handle normally
			if (handleClusterSingle(cluster, frame, stride, params))
			{ // Refined, and passed filter
				subClusters.push_back(std::move(cluster));
			} // else filtered out, return 0 clusters
			return subClusters;
		}
	}
	else if (cluster.dots.size() < params.classification.blobRefinementThreshold)
	{ // Too small - refine simply and apply filter
		if (handleClusterSingle(cluster, frame, stride, params))
		{ // Refined, and passed filter
			subClusters.push_back(std::move(cluster));
		} // else filtered out, return 0 clusters
		return subClusters;
	}
	else
	{ // Will refine properly
		subClusters.push_back(std::move(cluster));
	}

	/**
	 * Refinement methods for large clusters assuming them to be one blob
	 */

	TimePoint_t t_refinement = sclock::now();

	for (auto &subCluster : subClusters)
	{
		if (subCluster.dots.size() < params.classification.blobRefinementThreshold)
		{ // Don't refine small cluster, already resegmented and refined using grayscale averaging
			continue;
		}

		dt0 = sclock::now();

		if (!refineCluster(subCluster, frame, stride, params.refinement, estimateHoughParameters1(subCluster), 10))
		{ // Failed to refine
			handleClusterSingle(subCluster, frame, stride, params);
		}

		dt1 = sclock::now();
		refineTimeUS += dtUS(dt0, dt1);
	}

	return subClusters;
}

bool handleClusterSingle(Cluster &blob, const uint8_t *frame, uint32_t stride, const BlobProcessingParameters &params)
{
	blob.centroid.setZero();
	unsigned int totalWeight = 0;
	int minValue = 255, maxValue = 0;
	for (const auto &dot : blob.dots)
	{
		int value = frame[dot.y()*stride + dot.x()];
		minValue = std::min(minValue, value);
		maxValue = std::max(maxValue, value);
		float weight = std::max(0, value-params.floodfilling.threshold.min);
		blob.centroid += dot.cast<float>()*weight;
		totalWeight += weight;
	}
	if (blob.dots.size() < params.filtering.minContributingPixels)
		return false; // Not enough unconflicted pixels to support this subCluster accurately
	float contrast = maxValue-minValue;
	if (contrast < params.filtering.minContrastValue)
		return false; // Probably just insignificant peak in bright area - ignore
	blob.ptCnt = blob.dots.size();
	blob.centroid /= totalWeight;
	blob.size = std::sqrt((float)blob.ptCnt / PI);
	blob.contrast = contrast;
	blob.certainty = NAN;
	blob.reliability = blob.dots.size();
	blob.circularity = 0; // TODO
	blob.weight = totalWeight;
	blob.value = blob.contrast + blob.weight/20;
	// NOTE: More or less mirrored with resegmentation.hpp;;ResegmentCluster
	return true;
}

bool refineCluster(Cluster &blob, const uint8_t *frame, uint32_t stride, const RefinementParameters params, const HoughParameters &hough, int refineIt)
{
	auto t1 = sclock::now();

	// Refine the blob edge using the grayscale gradient
	refineBlobEdge(blob.edge, frame, stride, edgeRefined, params.targetEdgeVal, params.maxEdgeOffsetPX);

	auto t2 = sclock::now();
	
    // Vote on best parameters using a hough transform
	int radiusStep = -1;
	Vector2<float> center;
	if (!getRefinementEstimate(edgeRefined, hough, radiusStep, center))
	{ // No better estimation of blob
		LOG(LCameraEmulation, LDebug, "Failed to refine blob of %d edge points!", (int)blob.edge.size());
		return false;
	}

	auto t3 = sclock::now();

	// Mark already unlikely edge points as outliers
	edgeOutliers.resize(edgeRefined.size());
	float radius = hough.radiusMin + radiusStep * hough.radiusStep;
	float radiusSq = radius * radius;
	float circleWidth2Sq = (radius+hough.circleWidth)*(radius+hough.circleWidth) - radiusSq;
	int perimeterCnt = 0;
	for (int i = 0; i < edgeRefined.size(); i++)
	{
		edgeOutliers[i] = std::abs((edgeRefined[i]-center).squaredNorm() - radiusSq) > circleWidth2Sq;
		if (!edgeOutliers[i]) perimeterCnt++;
	}
	if (perimeterCnt == 0)
	{ // No better estimation of blob
		LOG(LCameraEmulation, LDebug, "Failed to refine blob of %d edge points after best estimate had no inlier edge!", (int)blob.edge.size());
		return false;
	}

	auto t4 = sclock::now();

	// Iteratively refine parameters to fit a set of inlier edge points
	Vector2<float> houghCenter = center;
	perimeterCnt = iterativeRefinement(center, radius, edgeRefined, edgeOutliers, refineIt);
	if (perimeterCnt == 0)
	{ // No better estimation of blob
		LOG(LCameraEmulation, LDebug, "Failed to refine blob of %d edge points after iterative refinement left no inlier edge!", (int)blob.edge.size());
		return false;
	}

	auto t5 = sclock::now();

	// Re-assign blob parameters (may have already been set in ResegmentCluster)
	blob.ptCnt = blob.dots.size();
	blob.centroid = center;
	blob.size = radius;
	blob.reliability = blob.dots.size() * ((float)perimeterCnt/blob.edge.size());
	blob.circularity = (float)perimeterCnt/blob.edge.size(); // TODO
	if (blob.weight == 0)
	{ // Overwrite only if not already set in ResegmentCluster
		blob.contrast = 230; // 255-params.floodfilling.threshold.min;
		blob.certainty = NAN;
		blob.weight = NAN;
		blob.value = blob.contrast + blob.ptCnt * 230/20.0f;
	}
	// NOTE: More or less mirrored with resegmentation.hpp;;ResegmentCluster

	LOG(LCameraEmulation, LDebug, "Refined point from (%fpx, %fpx) to (%fpx, %fpx) (%fpx), reliability %.2f\n",
		houghCenter.x(), houghCenter.y(), blob.centroid.x(), blob.centroid.y(), 
		(blob.centroid - houghCenter).norm(), blob.reliability);

	LOG(LCameraEmulation, LDebug, "Took a total of %fms! Refining: %fms - Hough: %fms - Optimising: %fm\n",
		dtMS(t1, sclock::now()), dtMS(t1, t2), dtMS(t2, t3), dtMS(t4, t5));

	return true;
}

HoughParameters estimateHoughParameters1(Cluster &blob)
{
	HoughParameters hough;
	hough.boundsMid = blob.bounds.center<float>();
	hough.boundsPX = blob.bounds.extends().cast<int>();
	//int boundsMin = std::min(hough.boundsPX.x(), hough.boundsPX.y());
	int boundsMax = std::max(hough.boundsPX.x(), hough.boundsPX.y());

	// Define initial expected parameter range for radius and position if possible
	float radiusMax = boundsMax*0.9;
	hough.radiusMin = boundsMax*0.4f;
	hough.positionStep = boundsMax/12.0f;
	hough.circleWidth = 0.6 * std::sqrt(hough.positionStep*hough.positionStep + hough.positionStep*hough.positionStep);
	hough.radiusRange = (int)std::ceil((radiusMax-hough.radiusMin)/(hough.circleWidth*1.5f));
	hough.radiusStep = (radiusMax-hough.radiusMin)/(hough.radiusRange-1);

	LOG(LCameraEmulation, LDebug, "Parameters: %f width, %f pos step (%f its), %f-%f radius, %f step (%d its)",
		hough.circleWidth, hough.positionStep, boundsMax/hough.positionStep,
		hough.radiusMin, radiusMax, hough.radiusStep, hough.radiusRange);

	return hough;
}

HoughParameters estimateHoughParameters2(Cluster &blob)
{
	HoughParameters hough;
	hough.boundsMid = blob.bounds.center<float>();
	hough.boundsPX = blob.bounds.extends().cast<int>();
	//int boundsMin = std::min(hough.boundsPX.x(), hough.boundsPX.y());
	int boundsMax = std::max(hough.boundsPX.x(), hough.boundsPX.y());

	// Define initial expected parameter range for radius and position if possible
	float radiusMax = boundsMax*0.6 + 0.005*boundsMax;
	hough.radiusMin = boundsMax*0.45f;
	hough.radiusStep = (radiusMax-hough.radiusMin)/(hough.radiusRange-1);
	hough.radiusRange = (int)(6 + boundsMax/15);
	float expectedRadius = boundsMax*0.5f;
	//hough.positionStep = 0.5f + 0.1f * expectedRadius;
	hough.positionStep = hough.radiusStep;
	//hough.circleWidth = 0.6f * std::sqrt(hough.positionStep*hough.positionStep + hough.positionStep*hough.positionStep);
	hough.circleWidth = hough.radiusStep*2.0f;

	LOG(LCameraEmulation, LDebug, "Parameters: %f width, %f pos step (%f its), %f-%f radius, %f step (%d its)",
		hough.circleWidth, hough.positionStep, boundsMax/hough.positionStep,
		hough.radiusMin, radiusMax, hough.radiusStep, hough.radiusRange);

	return hough;
}