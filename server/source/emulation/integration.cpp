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

#include "integration.hpp"

#include "ui/system/vis.hpp"
#include "emulation/emulation.inl"

// Solely for ImPlotColormap
#include "implot/implot.h"
#include "implot/implot_internal.h"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;


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

void updateEmulationVis(std::shared_ptr<BlobEmulationVis> &vis, const std::shared_ptr<BlobEmulationResults> &result, std::vector<SceneLabel> &labels)
{
	labels = result->labels;

	if (!vis)
		vis = std::make_shared<BlobEmulationVis>();

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

void updateEmulationVisUI(CameraVisState &visCamera)
{
	auto &emul = visCamera.emulation;
	auto &opt = emul.options;
	bool update = false;
	if (!emul.vis)
		emul.vis = std::make_shared<BlobEmulationVis>();

	ImGui::Checkbox("Show Mask All", &opt.maskAll);
	ImGui::Checkbox("Show Mask Center", &opt.maskCenter);
	ImGui::Checkbox("Show Mask Edge", &opt.maskEdge);

	ImGui::Separator();

	ImGui::Checkbox("Show blobs from Camera", &opt.blobCam);
	ImGui::Checkbox("Show emulated blobs", &opt.blobEmulated);
	ImGui::Checkbox("Show labels", &opt.labels);

	ImGui::SeparatorText("Resegmentation");

	if (ImGui::TreeNodeEx("Base Images", ImGuiTreeNodeFlags_Framed))
	{
		int id = 0;
		for (auto &tmp : emul.vis->baseImages)
			ImGui::Checkbox(asprintf_s("Show %s###%d", tmp.label.c_str(), id++).c_str(), &tmp.show);
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx("SSR Images", ImGuiTreeNodeFlags_Framed))
	{
		int id = 0;
		for (auto &tmp : emul.vis->ssrImages)
			ImGui::Checkbox(asprintf_s("Show %s###%d", tmp.label.c_str(), id++).c_str(), &tmp.show);
		ImGui::TreePop();
	}

	if (ImGui::TreeNodeEx("Maxima Hint Stages", ImGuiTreeNodeFlags_Framed))
	{
		for (int i = 0; i < opt.showBlobsMaximaStages.size(); i++)
		{
			bool show = opt.showBlobsMaximaStages[i];
			if (ImGui::Checkbox(i == 0? "Base" : asprintf_s("Scale %d", i).c_str(), &show))
				opt.showBlobsMaximaStages[i] = show;
		}
		ImGui::TreePop();
	}

	ImGui::Checkbox("Show maxima hints", &opt.maximaHints);

	if (ImGui::TreeNodeEx("Floodfilling Stages", ImGuiTreeNodeFlags_Framed))
	{
		int id = 0;
		for (auto &tmp : emul.vis->floodfillStages)
			ImGui::Checkbox(asprintf_s("%s###%d", tmp.label.c_str(), id++).c_str(), &tmp.show);
		ImGui::TreePop();
	}

	ImGui::Checkbox("Show peripheral centers", &opt.peripheralCenters);
	ImGui::Checkbox("Show resegmentation mask", &opt.resegmentationMask);
	ImGui::Checkbox("Show resegmented blobs", &opt.blobResegmented);

	ImGui::SeparatorText("Refinement");

	ImGui::Checkbox("Show refined edge", &opt.edgeRefined);
	ImGui::Checkbox("Show refined blobs", &opt.blobRefined);

	if (ImGui::TreeNodeEx("Hough Parameters", ImGuiTreeNodeFlags_Framed))
	{
		update |= ImGui::Checkbox("Force", &forceHoughParams);
		ImGui::BeginDisabled(!forceHoughParams);
		update |= ImGui::InputFloat("Radius Min", &forcedHoughParams.radiusMin);
		update |= ImGui::InputFloat("Radius Step", &forcedHoughParams.radiusStep);
		update |= ImGui::InputInt("Radius Range", &forcedHoughParams.radiusRange);
		update |= ImGui::InputFloat("Circle Width", &forcedHoughParams.circleWidth);
		update |= ImGui::InputFloat("Position Step", &forcedHoughParams.positionStep);
		ImGui::EndDisabled();
		ImGui::TreePop();
	}

	if (update && emul.enabled)
	{
		emul.update = true;
	}
}

void updateEmulationVisualisation(const TrackingCameraState &camera, CameraVisState &visCamera, const CameraFrameRecord &frame, Eigen::Vector2i viewSize)
{
	CameraMode mode = camera.pipeline->mode;
	float PixelSize = (float)viewSize.x() / mode.widthPx;
	float PixelStride = 2.0f / mode.widthPx; // 2 because of -1 to 1 space
	float blobAlpha = visCamera.image && visCamera.imageVis.show? 0.6f : 1.0f, blobCross = 2.0f / mode.widthPx;
	auto &emul = visCamera.emulation;

	if (emul.update && visCamera.image && threadPool.n_idle() > 1)
	{ // Need to use n_idle, an updating boolean flag wasn't enough, would sometimes get stuck with no threads running - weird
		emul.update = false;

		threadPool.push([&camera, &emul](int, std::shared_ptr<const CameraImage> frame)
		{
			// TODO: Verify camera still exists, technically it might be disconnected since we don't hold a shared_ptr
			emul.newResults = performCameraEmulation(camera, frame);
			GetUI().RequestUpdates();
		}, visCamera.image); // new shared_ptr 
	}

	if (emul.result && emul.vis)
	{
		// Display image in appropriate part of the frame
		Eigen::Affine3f bgProjMat = Eigen::Affine3f(
			// 3. Correct for aspect ratio already applied in projection matrix (from 1,1 to 1,aspect)
			Eigen::AlignedScaling3f((float)mode.sizeW, (float)mode.sizeH, 1.0f)
			// 2. Move image
			* Eigen::Translation3f(
				+(emul.result->boundsRel.center().x()-0.5f)*2.0f,
				-(emul.result->boundsRel.center().y()-0.5f)*2.0f,
				0.0f)
			// 1. Scale image (flip y axis from pixel coordinates to image coordinates)
			* Eigen::AlignedScaling3f(
				+emul.result->boundsRel.extends().x(),
				-emul.result->boundsRel.extends().y(),
				1.0f)
		);
		for (auto &tmp : emul.vis->baseImages)
		{
			if (!tmp.show) continue;
			showGrayscaleFrame(tmp.texID, bgProjMat);
		}
		for (auto &tmp : emul.vis->ssrImages)
		{
			if (!tmp.show) continue;
			showGrayscaleFrame(tmp.texID, bgProjMat);
		}
		for (auto &tmp : emul.vis->floodfillStages)
		{
			if (!tmp.show) continue;
			showGrayscaleFrame(tmp.texID, bgProjMat, {1,0,1,1}, 0.0f);
		}
		if (emul.options.resegmentationMask)
		{
			int i = 0;
			for (auto &msk : emul.vis->resegmentationMasks)
			{
				ImU32 col = ImPlot::GetColormapColorU32(i++, ImPlotColormap_Dark);
				Color tint = Color8{
					(uint8_t)((col >> IM_COL32_R_SHIFT) & 0xFF),
					(uint8_t)((col >> IM_COL32_G_SHIFT) & 0xFF),
					(uint8_t)((col >> IM_COL32_B_SHIFT) & 0xFF),
					128
				};
				showGrayscaleFrame(msk.texID, bgProjMat, tint, 0.0f);
			}
		}

		if (emul.options.maskAll)
			visualisePointsVBOSprites(emul.vis->maskVBO, emul.vis->maskSize, false, viewSize.x());
		if (emul.options.maskCenter)
			visualisePointsVBOSprites(emul.vis->maskCenterVBO, emul.vis->maskCenterSize, false, viewSize.x());
		if (emul.options.maskEdge)
			visualisePointsVBOSprites(emul.vis->maskEdgeVBO, emul.vis->maskEdgeSize, false, viewSize.x());
		if (emul.options.edgeRefined)
			visualisePointsVBOSprites(emul.vis->edgeRefinedVBO, emul.vis->edgeRefinedCount, true, viewSize.x());

		if (emul.options.blobCam)
		{ // Show blobs from camera normally
			visualiseBlobs2D(frame.rawPoints2D, frame.properties, Color{ 1, 1, 0.2, blobAlpha }, viewSize.x(), blobCross);
		}

		if (emul.options.blobEmulated)
		{ // Show emulated blobs using blob circle
			for (const auto &blob : emul.result->emulatedBlobs)
				visualiseBlobCircle(blob.pos.head<2>(), blob.size, Color8 { blob.color.r, blob.color.g, blob.color.b, 200 }, PixelStride);
			visualisePointsSprites(emul.result->emulatedBlobs, true, viewSize.x());
		}

		if (emul.options.blobRefined)
		{ // Show refined blobs using blob circle
			for (const auto &blob : emul.result->refinedBlobs)
				visualiseBlobCircle(blob.pos.head<2>(), blob.size, blob.color, PixelStride);
		}

		if (emul.options.blobResegmented)
		{ // Show resegmented blobs using blob circle
			for (const auto &blob : emul.result->resegmentedBlobs)
				visualiseBlobCircle(blob.pos.head<2>(), blob.size, blob.color, PixelStride);
		}

		if (emul.options.maximaHints)
		{ // Show emulated blobs using blob circle
			for (const auto &blob : emul.result->maximaHints)
				visualiseBlobCircle(blob.head<2>(), blob.z(), Color{ 1, 0, 0, 1 }, PixelStride);
		}

		if (emul.options.peripheralCenters)
		{ // Show debug peripheral centers blobs using blob circle
			for (const auto &blob : emul.result->peripheralCenters)
				visualiseBlobCircle(blob.head<2>(), blob.z(), Color{ 1, 0, 0, 1 }, 0);
		}

		if (emul.options.showBlobsMaximaStages.size() < emul.result->maximaHintStages.size())
			emul.options.showBlobsMaximaStages.resize(emul.result->maximaHintStages.size());
		for (int i = 0; i < emul.result->maximaHintStages.size(); i++)
		{ // Show debug maximal hint stages using blob circle
			if (!emul.options.showBlobsMaximaStages[i]) continue;
			for (const auto &blob : emul.result->maximaHintStages[i])
				visualiseBlobCircle(blob.head<2>(), blob.z(), Color{ 1, 0, 0, 1 }, 0);
		}
	}
}

Bounds2i getValidMaskRect(uint32_t width, uint32_t height)
{
	ProgramLayout layout = SetupProgramLayout(width, height, 8, false);
	return layout.validMaskRect;
}