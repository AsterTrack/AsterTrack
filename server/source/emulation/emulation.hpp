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

#ifndef EMULATION_H
#define EMULATION_H


#include "emulation/detection.hpp" // Simplified replacement for the blob detection subsystem
#include "blob/parameters.hpp" // Parameters for all blob detection algorithms
#include "blob/blob.hpp" // Exact copy of the blob refinement subsystem
#include "blob/qpu_blob_tiled.hpp" // To re-evaluate blob detection program layout and coverage
#include "blob/resegmentation.hpp" // Resegmenting clusters to make sure only peaks are included

#include "server.hpp"
#include "pipeline/record.hpp"
#include "device/tracking_camera.hpp"
#include "ui/gl/visualisation.hpp"

#include "util/eigendef.hpp"
#include "util/eigenutil.hpp"

#include <vector>
#include <cstdint>

// Immediate results and debug data of emulated blob detection 
struct BlobEmulationResults
{
	// Associated Info
	FrameID frameID;
	int width, height;
	Bounds2f boundsRel;

	// Detection/Segmentation
	std::vector<Cluster> clusters;
	std::vector<VisPoint> maskVerts, maskCenterVerts, maskEdgeVerts;
	// Resegmented clusters
	std::vector<Cluster> resegmentedClusters;
	std::vector<VisPoint> resegmentedBlobs;
	// Additional refined blobs
	std::vector<VisPoint> refinedBlobs;
	std::vector<VisPoint> edgeRefinedVerts;
	// Final blobs
	std::vector<VisPoint> emulatedBlobs;
	// Labels
	std::vector<SceneLabel> labels;

	// Image debug data
	std::vector<BlobEmulImage> tempImages;
	std::vector<BlobEmulImage> ssrImages;
	std::vector<BlobEmulImage> floodfillStages;
	std::vector<BlobEmulImage> resegmentationMasks;

	// Point debug data
	std::vector<Eigen::Vector3f> maximaHints;
	std::vector<std::vector<Eigen::Vector3f>> maximaHintStages;
	std::vector<Eigen::Vector3f> peripheralCenters;
};

// Cached GL resources for visualisation of emulated blob detection
struct BlobEmulationVis
{
	unsigned int maskVBO, maskSize;
	unsigned int maskCenterVBO, maskCenterSize;
	unsigned int maskEdgeVBO, maskEdgeSize;
	unsigned int edgeRefinedVBO, edgeRefinedCount;
	struct BlobEmulTexture
	{
		unsigned int texID;
		std::string label;
		bool show;
	};
	std::vector<BlobEmulTexture> baseImages;
	std::vector<BlobEmulTexture> ssrImages;
	std::vector<BlobEmulTexture> floodfillStages;
	std::vector<BlobEmulTexture> resegmentationMasks;
};

static void updateEmulationVis(std::shared_ptr<BlobEmulationVis> &vis, const std::shared_ptr<BlobEmulationResults> &result)
{
	// Update mask and point VBOs
	updatePointsVBO(vis->maskVBO, result->maskVerts);
	vis->maskSize = result->maskVerts.size();
	updatePointsVBO(vis->maskCenterVBO, result->maskCenterVerts);
	vis->maskCenterSize = result->maskCenterVerts.size();
	updatePointsVBO(vis->maskEdgeVBO, result->maskEdgeVerts);
	vis->maskEdgeSize = result->maskEdgeVerts.size();
	updatePointsVBO(vis->edgeRefinedVBO, result->edgeRefinedVerts);
	vis->edgeRefinedCount = result->edgeRefinedVerts.size();

	// Update image textures
	vis->baseImages.resize(result->tempImages.size());
	for (int i = 0; i < result->tempImages.size(); i++)
	{
		loadGrayscaleFrame(vis->baseImages[i].texID, result->tempImages[i].image.data(), result->width, result->height);
		vis->baseImages[i].label = result->tempImages[i].label;
	}
	vis->ssrImages.resize(result->ssrImages.size());
	for (int i = 0; i < result->ssrImages.size(); i++)
	{
		loadGrayscaleFrame(vis->ssrImages[i].texID, result->ssrImages[i].image.data(), result->width, result->height);
		vis->ssrImages[i].label = result->ssrImages[i].label;
	}
	vis->floodfillStages.resize(result->floodfillStages.size());
	for (int i = 0; i < result->floodfillStages.size(); i++)
	{
		loadGrayscaleFrame(vis->floodfillStages[i].texID, result->floodfillStages[i].image.data(), result->width, result->height);
		vis->floodfillStages[i].label = result->floodfillStages[i].label;
	}
	vis->resegmentationMasks.resize(result->resegmentationMasks.size());
	for (int i = 0; i < result->resegmentationMasks.size(); i++)
	{
		loadGrayscaleFrame(vis->resegmentationMasks[i].texID, result->resegmentationMasks[i].image.data(), result->width, result->height);
	}
}

// Base emulation-only parameters
static float baseSigma = 0.3f;
static int boxBlurIterations = 3;

// Refinement
static bool forceHoughParams = false;
static HoughParameters forcedHoughParams = {};


static std::shared_ptr<BlobEmulationResults> performCameraEmulation(const TrackingCameraState &camera, std::shared_ptr<const CameraImage> &frame)
{
	std::shared_ptr<BlobEmulationResults> result = std::make_shared<BlobEmulationResults>();
	result->frameID = frame->frameID;
	result->width = frame->width;
	result->height = frame->height;
	result->boundsRel = frame->boundsRel;

	/**
	 * Preparation - layout, spaces, transformations
	 */

	TimePoint_t t_prep = sclock::now();

	// Fetch camera & blob detection configuration
	CameraConfig config = GetState().cameraConfig.getCameraConfig(camera.id);

	// Emulate program layout that determines where blob detection is applied on
	ProgramLayout layout = SetupProgramLayout(config.width, config.height, 8);

	// Overlap QPU program layout with bounds of image (may be subsection of frame)
	Bounds2i proc = frame->boundsPx;
	//proc.extendBy(Eigen::Vector2i(-2, -2)); // Not necessary, the edge might have less blobs than camera, but not more
	// Keeping a 2px margin would just keep that edge empty and thus prevent potentially misleading differences between emulation and camera
	proc.overlapWith(layout.validMaskRect);

	// Transform from frame space into image pixel space (may be a subsection of frame)
	proc.min -= frame->boundsPx.min;
	proc.max -= frame->boundsPx.min;
	auto size = Eigen::Vector2i(frame->width, frame->height), bounds = frame->boundsPx.extends();
	proc.min = proc.min.cwiseProduct(size).cwiseQuotient(bounds);
	proc.max = proc.max.cwiseProduct(size).cwiseQuotient(bounds);

	// Transform all results from image pixel space back to frame space
	Eigen::Vector2f camPixFactor((float)frame->boundsPx.extends().x()/frame->width, (float)frame->boundsPx.extends().y()/frame->height);
	Eigen::Vector2f camPixOffset = frame->boundsPx.min.cast<float>();
	auto emulPix2Cam = [=](Eigen::Vector2f px)
	{
		px = px + Eigen::Vector2f(0.5f,0.5f);
		px = px.cwiseProduct(camPixFactor) + camPixOffset;
		return pix2cam<float>(camera.pipeline->mode, px);
	};

	// Set pixel values for visualisation only
	float PixelSize = 1.0f / config.width * camPixFactor.x();
	float PixelStride = 2.0f * PixelSize; // 2 because of -1 to 1 space

	// CV image manipulation
	Bounds2i imageRect(0, 0, frame->width, frame->height);
	int stride = frame->width;
	int width = frame->width, height = frame->height;
	auto &params = config.blobProcessing;

	/**
	 * Preparing image with blurring and creating a SSR (Scale Space Representation)
	 */

	TimePoint_t t_base = sclock::now();

	// Base image with optional blurring
	std::vector<uint8_t> fullBaseImage(frame->image.size());
	std::vector<uint8_t> partialBaseImage(frame->image.size());
	std::vector<uint8_t> tempImage(frame->image.size());
	std::vector<float> baseKernel;
	Bounds2i baseBounds = imageRect;
	float baseGaussianTime = 0, baseBoxBlurTime = 0;
	if (params.base.blur)
	{ // Blur base image, has added jpeg noise anyway so never exactly the same as from the camera

		// Real gaussian blur
		TimePoint_t dt0 = sclock::now();
		baseKernel = discreteGaussianKernel(params.base.sigma, params.base.radius);
		auto bounds = imageRect.extendedBy(-params.base.radius);
		applySeparableKernel(frame->image.data(), fullBaseImage.data(), tempImage.data(), stride, bounds, baseKernel);
		TimePoint_t dt1 = sclock::now();
		baseGaussianTime = dtMS(dt0, dt1);
		result->tempImages.emplace_back(fullBaseImage, 
			asprintf_s("Base Gaussian Blur - Sigma %f, Radius %d - %fms###BaseGaussian",
				params.base.sigma, params.base.radius, baseGaussianTime));
		proc.overlapWith(bounds.extendedBy(-2));
		baseBounds = bounds;

		// Box blur alternative (really only good for large sigmas > ~10)
		std::vector<uint8_t> boxGaussian = frame->image;
		dt0 = sclock::now();
		auto boxBounds = imageRect;
		std::vector<int> boxes(boxBlurIterations);
		auto actualParams = generateGaussianBlurBoxes(params.base.sigma, boxes);
		applyBoxFilters(frame->image.data(), boxGaussian.data(), tempImage.data(), stride, boxBounds, boxes);
		dt1 = sclock::now();
		baseBoxBlurTime = dtMS(dt0, dt1);
		result->tempImages.emplace_back(boxGaussian, 
			asprintf_s("Base Box Blur - Sigma %f, Radius %d, Boxes %d - %fms###BaseBox", 
				actualParams.first, actualParams.second, (int)boxes.size(), baseBoxBlurTime));
		//proc.overlapWith(boxBounds.extendedBy(-2));
		//baseBounds = boxBounds;
	}
	else
	{
		fullBaseImage = frame->image;
		partialBaseImage = frame->image;
		result->tempImages.emplace_back(fullBaseImage, "Base Gaussian Blur###BaseGaussian");
		result->tempImages.emplace_back(fullBaseImage, "Base Box Blur###BaseBox");
		result->tempImages.emplace_back(fullBaseImage, "Base Gaussian Blur (Partial)###BasePart");
	}

	TimePoint_t t_ssr = sclock::now();

	// Precalculate SSR gaussian parameters
	std::vector<float> ssrSigmas;
	std::vector<std::vector<float>> ssrKernels;
	generateSSR(params.ssr, ssrSigmas, ssrKernels);

	// Precalculate full SSR (Scale Space Representation) of image
	std::vector<std::vector<uint8_t>> ssrGaussians(ssrKernels.size(), frame->image);
	float ssrGaussianTimes = 0;
	for (int s = 0; s < ssrKernels.size(); s++)
	{
		int radius = (ssrKernels[s].size()-1)/2;
		TimePoint_t dt0 = sclock::now();
		applySeparableKernel(fullBaseImage.data(), ssrGaussians[s].data(), tempImage.data(), stride, baseBounds.extendedBy(-radius), ssrKernels[s]);
		TimePoint_t dt1 = sclock::now();
		ssrGaussianTimes += dtMS(dt0, dt1);
		result->ssrImages.emplace_back(ssrGaussians[s], asprintf_s("Gaussian %d - Sigma %f, Radius %d - %fms", s, ssrSigmas[s], radius, dtMS(dt0, dt1)));
	}

	LOG(LCameraBlob, LInfo, "Calculating base image took %fms (%fms for box blur), full gaussian SSR took %fms",
		baseGaussianTime, baseBoxBlurTime, ssrGaussianTimes);

	/**
	 * Emulate masking and CCL (Connected Component Labelling)
	 * Complete re-implementation of camera code in a simple, non-performance-sensitive way
	 */

	TimePoint_t t_masking = sclock::now();

	// Perform emulated blob detection on parts of the image
	performBlobDetection(config, proc,
		fullBaseImage.data(), frame->width, frame->height, frame->width,
		result->clusters);
	// NOTE: Applying on fullBaseImage, not frame->image, since jpeg noise distorts it anyway

	TimePoint_t t_masking_end = sclock::now();

	LOG(LCameraBlob, LInfo, "Emulated masking and CCL in %fms, found %d clusters",
		dtMS(t_masking, t_masking_end), (int)result->clusters.size());

	{ // Update mask visualisation
		for (const auto &clusters : result->clusters)
		{
			for (const auto &dot : clusters.dots)
			{
				VisPoint vert;
				vert.pos.head<2>() = emulPix2Cam(dot.cast<float>());
				vert.pos.z() = 1-0.9f;
				vert.size = PixelSize;
				if (centerMask.test(dot.y()*frame->width+dot.x()))
				{
					vert.color = Color{ 0.2f, 0.8f, 0.2f, 0.5f };
					result->maskCenterVerts.push_back(vert);
				}
				if (edgeMask.test(dot.y()*frame->width+dot.x()))
				{
					vert.color = Color{ 0.2f, 0.2f, 0.8f, 0.5f };
					result->maskEdgeVerts.push_back(vert);
				}
				{
					vert.color = Color{ 0.2f, 0.6f, 0.6f, 0.6f };
					result->maskVerts.push_back(vert);
				}
			}
		}
	}

	// Temporary output for debug
	std::vector<std::vector<ScaleCenter>> maximaStages;
	std::vector<ScaleCenter> iterativeCenters;

	// Stats for debug
	int blurTimeUS = 0, maximaHintTimeUS = 0;
	int localMaxTimeUS = 0, maxIterTimeUS = 0, finalCheckTimeUS = 0;
	int localMaxTimeUSDbg = 0, maxIterTimeUSDbg = 0, finalCheckTimeUSDbg = 0;
	int resegmentationTimeUS = 0;
	int resegmentedClusterCount = 0;

	// Precalculated SSR gaussian images
	auto queryGaussian = [&ssrGaussians, stride](int scale, Eigen::Vector2i dot) -> uint8_t
	{
		return ssrGaussians[scale][dot.y()*stride + dot.x()];
	};

	// Calculating gaussians on-demand
	auto calcGaussian = [&](int scale, Eigen::Vector2i dot) -> uint8_t
	{
		return (uint8_t)calcKernelBorder(frame->image.data(), stride, width, height, dot, ssrKernels[scale]);
	};
	for (int b = 0; b < result->clusters.size(); b++)
	{
		const auto &cluster = result->clusters[b];
		if (cluster.dots.size() <= params.classification.blobTinyThreshold)
		{ // Almost certainly a single, tiny blob
			continue;
		}

		TimePoint_t dt0, dt1;

		if (cluster.dots.size() > params.classification.resegmentationThreshold)
		{ // Don't even try to resegment, too large
			result->resegmentedClusters.push_back(cluster);
			continue;
		}

		Bounds2i bounds = result->clusters[b].bounds.cast<int>();
		if (bounds.size() < params.filtering.minContributingPixels*2)
			bounds.extendBy(params.floodfilling.blob.allowBoundsExpansion);
		bounds.overlapWith(baseBounds);
		// TODO: may connect clusters for resegmentation even though they aren't competing, meaning one blob may consume area of another - bad!
		// made some attempts to mitigate, but didn't work so well

		/**
		* Find peaks in clusters to detect merged blobs
		* Follow local maximas up the scale tree to determine if they are real blobs or noise
		* Applies per previously segmented cluster, expecting one or more real blobs in that cluster
		*/

		// Debug run
		FindMaximaHints<true>(
			fullBaseImage.data(), stride, width, height,
			ssrKernels.size(), queryGaussian, bounds,
			params.maximaHints,
			cluster, maximaStages,
			localMaxTimeUSDbg, maxIterTimeUSDbg, finalCheckTimeUSDbg);

		// Optimised run
		dt0 = sclock::now();

		if (params.base.blur)
		{ // Calculate partial blur (and store for resegmentation)
			applySeparableKernel(frame->image.data(), partialBaseImage.data(), tempImage.data(), stride, bounds, baseKernel);
		}

		dt1 = sclock::now();
		blurTimeUS += dtUS(dt0, dt1);

		// Find maxima hints
		dt0 = sclock::now();
		std::vector<ClusterPeakHint> hints = FindMaximaHints<false>(
			partialBaseImage.data(), stride, width, height,
			ssrKernels.size(), calcGaussian, bounds,
			params.maximaHints,
			cluster, maximaStages,
			localMaxTimeUS, maxIterTimeUS, finalCheckTimeUS);

		dt1 = sclock::now();
		maximaHintTimeUS += dtUS(dt0, dt1);

		// For debug only
		for (auto &hint : hints)
		{
			Eigen::Vector2f blob2D = emulPix2Cam(hint.peak.cast<float>());
			float size = (hint.scale+1) * PixelStride/3;
			result->maximaHints.push_back(Eigen::Vector3f(blob2D.x(), blob2D.y(), size));
		}

		if (hints.size() <= 1 && (hints.empty() || !params.classification.resegmentSingleClusters))
		{
			if (cluster.ptCnt >= params.filtering.minContributingPixels)
				result->resegmentedClusters.push_back(cluster);
			continue;
		}

		/**
		* Re-segment clusters based on hints for local peaks
		* Use competitive floodfilling from those peaks to find maximum area without conflicts with nearby merged blobs
		*/

		// Debug run
		ResegmentCluster<true>(
			partialBaseImage.data(), stride, width, height,
			bounds, cluster, hints, params,
			result->floodfillStages, result->resegmentationMasks, iterativeCenters);

		// Optimised run
		dt0 = sclock::now();
		std::vector<Cluster> resegmentedClusters = ResegmentCluster<false>(
			partialBaseImage.data(), stride, width, height,
			bounds, cluster, hints, params,
			result->floodfillStages, result->resegmentationMasks, iterativeCenters);
		dt1 = sclock::now();
		resegmentationTimeUS += dtUS(dt0, dt1);
		resegmentedClusterCount++;

		for (auto &cluster : resegmentedClusters)
		{
			VisPoint vert;
			vert.pos.head<2>() = emulPix2Cam(cluster.centroid);
			vert.size = cluster.size * PixelStride;
			vert.color = Color{ 1, 0, 0, 1 };
			result->resegmentedBlobs.push_back(vert);
		}

		for (auto &cluster : resegmentedClusters)
		{
			SceneLabel label;
			label.position.head<2>() = emulPix2Cam(cluster.centroid);
			label.radius = cluster.size * PixelStride;
			label.text = asprintf_s("Value %.2f\nSize %.2fpx\nWeight %.1f\nContrast %.1f\nReliability %.1f",
				cluster.value, cluster.size*2, cluster.weight/20, cluster.contrast, cluster.reliability);
			label.color = Color{ 1, 0, 0, 1 };
			result->labels.push_back(label);
		}

		std::move(std::begin(resegmentedClusters), std::end(resegmentedClusters), std::back_inserter(result->resegmentedClusters));
	}

	result->tempImages.emplace_back(partialBaseImage, asprintf_s("Base Gaussian Blur (Partial) - Sigma %f, Radius %d###BasePart", ssrSigmas[0], params.base.radius));
	LOG(LCameraBlob, LInfo, "Base blurring took %dus, finding maximas took %dus (%d+%d+%d), resegmenting %d/%d clusters into %d took %dus", 
		blurTimeUS, maximaHintTimeUS, localMaxTimeUS, maxIterTimeUS, finalCheckTimeUS, resegmentedClusterCount, (int)result->clusters.size(), (int)result->resegmentedClusters.size(), resegmentationTimeUS);

	// Update debug visualisation
	result->maximaHintStages.resize(maximaStages.size());
	for (int s = 0; s < maximaStages.size(); s++)
	{
		auto &stage = maximaStages[s];
		for (auto &peaks : stage)
		{
			Eigen::Vector2f blob2D = emulPix2Cam(peaks.center.cast<float>());
			float size = (peaks.scale+1) * PixelStride/3;
			result->maximaHintStages[s].push_back(Eigen::Vector3f(blob2D.x(), blob2D.y(), size));
		}
	}

	for (auto &itCenter : iterativeCenters)
	{
		Eigen::Vector2f blob2D = emulPix2Cam(itCenter.center);
		float size = (itCenter.scale+1) * PixelStride/3;
		result->peripheralCenters.push_back(Eigen::Vector3f(blob2D.x(), blob2D.y(), size));
	}


	/**
	 * Refinement methods for large clusters assuming them to be one blob
	 */

	TimePoint_t t_refinement = sclock::now();

	for (auto it = result->resegmentedClusters.begin(); it != result->resegmentedClusters.end();)
	{
		auto &cluster = *it;
		if (cluster.dots.size() < params.classification.blobRefinementThreshold || cluster.edge.empty())
		{ // Don't refine small cluster, already resegmented and refined using grayscale averaging
			if (cluster.value == 0)
			{ // Has not been resegmented
				if (handleClusterSingle(cluster, frame->image.data(), stride, params))
					it++;
				else // Filtered
					it = result->resegmentedClusters.erase(it);
			}
			else it++;
			continue;
		}

		edgeRefined.clear();
		bool refined = refineCluster(cluster, frame->image.data(), stride, params.refinement, estimateHoughParameters1(cluster), 10);

		// Update visualisation using temporary edgeRefined
		for (int e = 0; e < edgeRefined.size(); e++)
		{
			VisPoint vert;
			vert.pos.head<2>() = emulPix2Cam(edgeRefined[e]);
			vert.pos.z() = 1-0.9f;
			vert.size = PixelSize/6;
			if (edgeOutliers[e])
				vert.color = Color{ 0.2f, 0.8f, 0.8f, 1.0f };
			else
				vert.color = Color{ 0.8f, 0.8f, 0.2f, 1.0f };
			result->edgeRefinedVerts.push_back(vert);
		}

		if (!refined)
		{ // Failed to refine
			handleClusterSingle(cluster, frame->image.data(), stride, params);
			it++;
			continue;
		}

		{ // Update point visualisation
			VisPoint vert;
			vert.pos.head<2>() = emulPix2Cam(cluster.centroid);
			vert.size = cluster.size * PixelStride;
			vert.color = Color{ 1, 0, 0, 1 };
			result->refinedBlobs.push_back(vert);
		}

		it++;
	}

	// Update point visualisation
	for (auto &cluster : result->resegmentedClusters)
	{
		VisPoint vert;
		vert.pos.head<2>() = emulPix2Cam(cluster.centroid);
		vert.pos.z() = 0.1f;
		vert.size = cluster.size*2;
		vert.color = Color{ 
			std::min(1.0f, 0.4f + cluster.value/800*0.6f), 
			0.0f, 
			0, 
			std::min(0.6f, 0.2f + cluster.value/800*0.4f) };
		result->emulatedBlobs.push_back(vert);
	}

	TimePoint_t t_end = sclock::now();

	LOG(LCameraBlob, LInfo, "Took %fms to refine %d large blobs!",
		dtMS(t_refinement, t_end), (int)result->refinedBlobs.size());

	LOG(LCameraBlob, LInfo, "Took %fms to emulate camera frame in total, with %fms of preparation, and %fms of blurring/SSR!",
		dtMS(t_prep, t_end), dtMS(t_prep, t_base), dtMS(t_base, t_masking));

	return result;
}

#endif // EMULATION_H