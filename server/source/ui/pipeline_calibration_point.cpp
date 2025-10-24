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

#include "ui.hpp"

#include "pipeline/pipeline.hpp"
#include "calib/obs_data.inl"

#include "ctpl/ctpl.hpp"
extern ctpl::thread_pool threadPool;

#include <set>

void InterfaceState::UpdatePipelineCalibSection()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	ImVec2 ButtonSize = SizeWidthDiv3();

	BeginSection("Camera Calibration");
	{
		const auto &ptCalib = pipeline.pointCalib;
		auto errors = ptCalib.state.errors;
		float PixelFactor = (pipeline.cameras.empty()? 1280 : pipeline.cameras.front()->mode.widthPx)/2.0f; // For external logging, already dynamic
		ImGui::AlignTextToFramePadding();
		if (ptCalib.control.thread)
		{
			if (ptCalib.settings.typeFlags & 0b001)
			{
				ImGui::TextUnformatted("...");
			}
			else if (ptCalib.settings.typeFlags & 0b010)
			{
				ImGui::Text("%.4fpx += %.4fpx error, %.4fpx max (%d samples)",
					errors.mean*PixelFactor,
					errors.stdDev*PixelFactor,
					errors.max*PixelFactor,
					errors.num);
			}
		}
		else if (calibState.numUncalibrated == 0 && errors.num > 0)
		{
			if (std::isnan(errors.mean) || std::isinf(errors.mean) || errors.mean > 10.0f)
			{
				ImGui::TextUnformatted("Unable to check calibration errors!");
				ImGui::SetItemTooltip("%.4fpx += %.4fpx error, %.4fpx max (%d samples)",
					errors.mean*PixelFactor,
					errors.stdDev*PixelFactor,
					errors.max*PixelFactor,
					errors.num);
			}
			else
			{
				ImGui::Text("%.4fpx += %.4fpx error, %.4fpx max (%d samples)",
					errors.mean*PixelFactor,
					errors.stdDev*PixelFactor,
					errors.max*PixelFactor,
					errors.num);
			}
		}
		else if (calibState.numUncalibrated == 0)
		{ // Calibration exists
			if (visState.incObsUpdate.markerCount > 0 && calibError.calculating)
				ImGui::TextUnformatted("Calculating error...");
			else
				ImGui::TextUnformatted("No data to check calibration against!");
		}
		else if (visState.incObsUpdate.markerCount > 0)
		{ // Data exists
			ImGui::TextUnformatted("Incomplete calibration!");
			ImGui::SetItemTooltip("%d cameras are calibrated, %d are not, %d relations are certain, %d are not.",
				calibState.numCalibrated, calibState.numUncalibrated, calibState.relCertain, calibState.relUncertain);
		}
		else
			ImGui::TextUnformatted("No calibration nor samples available!");

		SameLineTrailing(ImGui::GetFrameHeight());
		if (RetryButton("Recalc"))
		{
			UpdateCalibrationError(false, true);
		}
	}
	{
		if (ImGui::Button("Reset##Calibration", ButtonSize))
		{
			{
				std::unique_lock pipeline_lock(pipeline.pipelineLock);
				for (auto &cam : pipeline.cameras)
				{
					if (state.defaultLens > 0)
						cam->calib = CameraCalib(state.lensPresets[state.defaultLens]);
					else cam->calib = CameraCalib();
					cam->calib.index = cam->index;
				}
			}
			pipeline.pointCalib.roomState.lastTransferSuccess = -1;
			pipeline.pointCalib.roomState.lastTransferError = std::nullopt;
			pipeline.pointCalib.roomState.unchangedCameras.clear();

			pipeline.calibration.contextualLock()->init(pipeline.cameras.size());
			UpdateCalibrations();
			UpdateCalibrationError(true);
			// TODO: This implies resetting can delete an existing calibration on saving, which it does not
			// Saved calibrations are only updated once a new calibration for this camera ID is created
			//state.cameraCalibsDirty = true;
		}
		ImGui::SameLine();
		if (SaveButton("Save##Calibration", ButtonSize, state.cameraCalibsDirty))
		{
			if (pipeline.pointCalib.roomState.lastTransferSuccess == 0)
			{ // Currently no valid room calibration - either cameras changed too much or there was no prior calibration at all
				// TODO: There is NO fool-proof way to verify the room calibration is good
				// At most the tracking can tell the scale is off if cameras never agree on where a known-good target is
				// This system fails if you save and load a calibration without room calib
				// Currently, we assume anything loaded has room calib, and this is only updated when room calibration is done
				// Then, it will continuously try to align itself to any prior calibration - and pop this message if transfer failed
				ImGui::OpenPopup("SaveNoRoomCalib");
			}
			auto error = storeCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
			if (error) GetState().errors.push(error.value());
			else state.cameraCalibsDirty = false;
		}
		if (ImGui::BeginPopup("SaveNoRoomCalib"))
		{
			ImGui::TextUnformatted("It appears the room calibration is not valid. This may happen if:\n"
				"- This is your first time setting up the camera system in this room.\n"
				"- The existing room calibration couldn't be transferred to this \n"
				"   re-calibrated system because too many cameras changed.\n"
				"You may have to do the Room Calibration below before you can track.");
			if (ImGui::Button("OK", SizeWidthFull()))
				ImGui::CloseCurrentPopup();
			ImGui::EndPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button("Load##Calibration", ButtonSize))
		{
			auto error = parseCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
			if (error) GetState().errors.push(error.value());
			else
			{
				state.cameraCalibsDirty = false;
				AdoptNewCalibrations(state.pipeline, state.cameraCalibrations, true);
				{
					auto lock = folly::detail::lock(folly::detail::wlock(pipeline.calibration), folly::detail::rlock(pipeline.seqDatabase));
					UpdateCalibrationRelations(pipeline, *std::get<0>(lock), *std::get<1>(lock));
				}

				UpdateCalibrations();
				UpdateCalibrationError(true);

				LOG(LGUI, LDebug, "== Loaded %d camera calibrations:\n", (int)state.cameraCalibrations.size());
				DebugCameraParameters(state.cameraCalibrations);
			}
		}
	}
	if (ImGui::TreeNode("Cameras"))
	{
		bool openLensPresets = false;
		if (ImGui::BeginTable("CamCalibs", 3))
		{
			ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthStretch, 3);
			ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 5);
			ImGui::TableSetupColumn("Edit", ImGuiTableColumnFlags_WidthStretch, 5);

			for (auto &camera : pipeline.cameras)
			{
				auto &calib = camera->calib;
				ImGui::PushID(camera->id);
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::AlignTextToFramePadding();
				ImGui::Text("#%d", camera->id);

				ImGui::TableNextColumn();
				bool noCalib = calib.invalid();
				bool noExtrinsics = calib.transform.matrix().isIdentity();
				bool noRadialDist = calib.distortion.k1 == 0 && calib.distortion.k2 == 0 && calib.distortion.k3 == 0;
				bool noAlignCorr = calib.principalPoint.squaredNorm() == 0 && calib.distortion.p1 == 0 && calib.distortion.p2 == 0;
				bool fullCalib = !(noCalib || noExtrinsics || noRadialDist || noAlignCorr);
				if (noCalib || noExtrinsics)
					ImGui::Text("No Calibration");
				else if (noRadialDist)
					ImGui::Text("No Distortion Profile");
				else if (noAlignCorr)
					ImGui::Text("No Alignment Correction");
				else if (!fullCalib)
					ImGui::Text("Invalid");
				else
				{
					if (camera->errorStats.num > 0)
						ImGui::Text("%.2f°h Lens, %.2fpx", getEffectiveFoVH(calib, camera->mode), camera->errorStats.rmse*PixelFactor);
					else
					 	ImGui::Text("%.2f°h Lens", getEffectiveFoVH(calib, camera->mode));
				}

				if (ImGui::BeginItemTooltip())
				{
					if (camera->errorStats.num > 0)
					{
						ImGui::Text("%.4fpx += %.4fpx error, %.4fpx max (%d samples)",
							camera->errorStats.mean*PixelFactor,
							camera->errorStats.stdDev*PixelFactor,
							camera->errorStats.max*PixelFactor,
							camera->errorStats.num);
					}
					if (fullCalib)
					{
						auto &mode = camera->mode;
						float fovH = getFoVH(calib, mode), fovV = getFoVV(calib, mode), fovD = getFoVD(calib, mode);
						//ImGui::Text("Raw FoV /wo Distortion: %.2f° H, %.2f° V, %.2f° D", fovH, fovV, fovD);
						float effH = getEffectiveFoVH(calib, mode), effV = getEffectiveFoVV(calib, mode), effD = getEffectiveFoVD(calib, mode);
						ImGui::Text("Field of View: %.2f° H, %.2f° V, %.2f° D", effH, effV, effD);
						float distH = (effH/fovH - 1) * 100, distV = (effV/fovV - 1) * 100, distD = (effD/fovD - 1) * 100;
						ImGui::Text("Relative Distortion: %.2f%% H, %.2f%% V, %.2f%% D", distH, distV, distD);
					}
					/* ImGui::Text("Raw: f = %f; principal = (%f, %f); radial = %f, %f, %f; tangential = %f, %f",
						calib.f,
						calib.principalPoint.x(), calib.principalPoint.y(),
						calib.distortion.k1, calib.distortion.k2, calib.distortion.k3,
						calib.distortion.p1, calib.distortion.p2); */
					ImGui::EndTooltip();
				}

				ImGui::TableNextColumn();
				ImGui::SetNextItemWidth(ImGui::GetColumnWidth() - ImGui::GetFrameHeight() - ImGui::GetStyle().ItemSpacing.x);
				if (ImGui::BeginCombo("##Preset", calib.lensID < 0? "Lens" : state.lensPresets[calib.lensID].label.c_str()))
				{
					if (ImGui::Selectable("None"))
					{
						if (calib.lensID >= 0)
							state.cameraCalibsDirty = true;
						calib.lensID = -1;
						SignalCameraCalibUpdate({ calib });
					}
					if (ImGui::Selectable("Make Preset"))
					{
						int id = state.lensPresets.empty()? 1 : (state.lensPresets.rbegin()->first + 1);
						assert(!state.lensPresets.contains(id));
						state.lensPresets[id] = LensCalib(id, calib);
						state.lensPresetsDirty = true;
						calib.lensID = id;
						SignalCameraCalibUpdate({ calib });
						openLensPresets = true;
					}
					for (auto &lensIt : state.lensPresets)
					{
						auto &lens = lensIt.second;
						ImGui::PushID(lensIt.first);
						if (ImGui::Selectable(lens.label.c_str(), calib.lensID == lensIt.first))
						{
							if (calib.lensID != lensIt.first)
								state.cameraCalibsDirty = true;
							if (calib.invalid())
							{ // Create new calib from lens
								calib = CameraCalib(lens);
								calib.id = camera->id;
								calib.index = camera->index;
							}
							else
							{ // Update existing calib with lens
								calib.distortion.k1 = lens.k1;
								calib.distortion.k2 = lens.k2;
								calib.distortion.k3 = lens.k3;
								calib.lensID = lens.id;
							}
							SignalCameraCalibUpdate({ calib });
							UpdateCalibrationError(true);
						}
						ImGui::PopID();
					}
					ImGui::EndCombo();
				}
				ImGui::SameLine();
				if (CrossButton("ResetCamCalib"))
				{
					if (state.defaultLens > 0)
						calib = CameraCalib(state.lensPresets[state.defaultLens]);
					else calib = CameraCalib();
				}
				
				ImGui::PopID();
			}

			ImGui::EndTable();
		}

		if (openLensPresets)
			ImGui::SetNextItemOpen(true);
		if (ImGui::TreeNode(state.lensPresetsDirty? "Lens Presets (unsaved)###LensPreset" : "Lens Presets###LensPreset"))
		{
			if (ImGui::BeginTable("LensPresets", 2))
			{
				ImGui::TableSetupColumn("Text", ImGuiTableColumnFlags_WidthStretch, 1);
				ImGui::TableSetupColumn("Edit", ImGuiTableColumnFlags_WidthFixed, GetBarWidth(ImGui::GetFrameHeight(), 3));

				for (auto lensIt = state.lensPresets.begin(); lensIt != state.lensPresets.end();)
				{
					LensCalib &lens = lensIt->second;
					CameraCalib calib(lens);
					CameraMode mode(1280, 800);
					ImGui::PushID(lens.id);
					ImGui::TableNextRow();
					ImGui::TableNextColumn();
					bool editing = editingLensPreset == lens.id;
					bool deleteLens = lensIt->first != lens.id;
					if (editing)
					{
						ImGui::SetNextItemWidth(ImGui::GetColumnWidth());
						if (ImGui::InputText("##LensName", &lens.label))
							state.lensPresetsDirty = true;
					}
					else
					{
						ImGui::AlignTextToFramePadding();
						ImGui::Text("Lens %s (%d) - %.2f° H",
							lens.label.c_str(), lens.id, getEffectiveFoVH(calib, mode));
						if (ImGui::BeginItemTooltip())
						{
							float fovH = getFoVH(calib, mode), fovV = getFoVV(calib, mode), fovD = getFoVD(calib, mode);
							float effH = getEffectiveFoVH(calib, mode), effV = getEffectiveFoVV(calib, mode), effD = getEffectiveFoVD(calib, mode);
							ImGui::Text("Field of View: %.2f° H, %.2f° V, %.2f° D", effH, effV, effD);
							float distH = (effH/fovH - 1) * 100, distV = (effV/fovV - 1) * 100, distD = (effD/fovD - 1) * 100;
							ImGui::Text("Relative Distortion: %.2f%% H, %.2f%% V, %.2f%% D", distH, distV, distD);
							ImGui::Text("Raw: ID = %d, f = %f, radial = (%f, %f, %f)",
								lens.id, lens.f, lens.k1, lens.k2, lens.k3);
							ImGui::EndTooltip();
						}
					}

					ImGui::TableNextColumn();

					if (ImGui::Button(editing? "D###Edit" : "E###Edit", SizeFrame()))
						editingLensPreset = editing? -1 : lens.id;
					ImGui::SetItemTooltip("Toggle Edit Mode to change the label and delete the lens preset.");
					ImGui::SameLine();

					if (editing)
					{
						std::string deletePopup = asprintf_s("deleteLens%d", lens.id);
						if (CrossButton("Delete"))
							ImGui::OpenPopup(deletePopup.c_str());
						if (ImGui::BeginPopup(deletePopup.c_str()))
						{
							ImGui::Text("Delete Lens Preset %d?", lens.id);
							if (ImGui::Button("Delete", SizeWidthFull()))
								deleteLens = true;
							ImGui::EndPopup();
						}
					}
					else
					{
						if (RetryButton("Refetch"))
						{
							int num = 0;
							for (auto &camera : pipeline.cameras)
								if (camera->calib.lensID == lens.id)
									num++;
							if (num > 0)
							{
								lens.f = 0;
								lens.k1 = 0;
								lens.k2 = 0;
								lens.k3 = 0;
								for (auto &camera : pipeline.cameras)
								{
									if (camera->calib.lensID != lens.id) continue;
									lens.f += camera->calib.f;
									lens.k1 += camera->calib.distortion.k1;
									lens.k2 += camera->calib.distortion.k2;
									lens.k3 += camera->calib.distortion.k3;
								}
								lens.f /= num;
								lens.k1 /= num;
								lens.k2 /= num;
								lens.k3 /= num;
								state.lensPresetsDirty = true;
							}
						}
						ImGui::SetItemTooltip("Update Lens Preset from calibration of connected cameras.");
					}

					ImGui::SameLine();
					bool defaultLens = state.defaultLens == lens.id;
					ImGui::BeginDisabled(defaultLens);
					if (ImGui::Checkbox("##Default", &defaultLens))
					{
						state.defaultLens = lens.id;
						state.lensPresetsDirty = true;
					}
					ImGui::EndDisabled();

					ImGui::PopID();

					if (deleteLens)
					{
						for (auto &camera : state.cameraCalibrations)
						{
							if (camera.lensID == lens.id)
								deleteLens = false;
						}
					}
					if (!deleteLens) lensIt++;
					else lensIt = state.lensPresets.erase(lensIt);
				}
				ImGui::EndTable();
			}

			if (SaveButton("Save Lenses", SizeWidthDiv2(), state.lensPresetsDirty))
			{
				auto error = storeLensPresets("store/lens_presets.json", state.lensPresets, state.defaultLens);
				if (error) GetState().errors.push(error.value());
				else state.lensPresetsDirty = false;
			}
			ImGui::SameLine();
			if (ImGui::Button("Reload Lenses", SizeWidthDiv2()))
			{
				auto error = parseLensPresets("store/lens_presets.json", state.lensPresets, state.defaultLens);
				if (error) GetState().errors.push(error.value());
				else state.lensPresetsDirty = false;
			}
			ImGui::TreePop();
		}

		ImGui::TreePop();
	}
	EndSection();
}

void InterfaceState::UpdatePipelineObservationSection()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	ImVec2 ButtonSize = SizeWidthDiv3();

	BeginSection("Marker Observations");
	ImGui::Checkbox("Record", &pipeline.recordSequences);
	ImGui::SetItemTooltip("Record visible markers into observations as continous sequences.");
	ImGui::SameLine(0, ImGui::GetStyle().ItemSpacing.x*2);
	ImGui::Text("%s", calibSamples.contextualRLock()->c_str());
	SameLineTrailing(ImGui::GetFrameHeight());
	if (RetryButton("Recalc"))
	{ // TODO: Fix incremental observation update (3/3)
		visState.incObsUpdate.resetFirstFrame = 0;
	}

	if (ImGui::Button("Clear", ButtonSize))
	{
		pipeline.seqDatabase.contextualLock()->clear();
		UpdateSequences(true);
	}
	ImGui::SameLine();
	if (ImGui::Button("Save", ButtonSize))
	{
		// Find path
		const char* obsPathFmt = "dump/marker_observations_%d.json";
		std::string obsPath = asprintf_s(obsPathFmt, findLastFileEnumeration(obsPathFmt)+1);
		// Copy camera ids
		std::vector<int> cameraIDs;
		for (auto &cam : pipeline.cameras)
			cameraIDs.push_back(cam->id);
		// Write to path
		auto error = dumpSequenceDatabase(obsPath, *pipeline.seqDatabase.contextualRLock(), cameraIDs);
		if (error) GetState().errors.push(error.value());
	}
	ImGui::SameLine();
	if (ImGui::Button("Load", ButtonSize))
	{
		// Find path
		const char* obsPathFmt = "dump/marker_observations_%d.json";
		int i = findLastFileEnumeration(obsPathFmt);
		if (i > 0)
		{
			std::string obsPath = asprintf_s(obsPathFmt, i);
			std::vector<int> cameraIDs;
			SequenceData observations;
			auto error = parseSequenceDatabase(obsPath, cameraIDs, observations);
			if (error)
				GetState().errors.push(error.value());
			else
			{
				bool valid = cameraIDs.size() == pipeline.cameras.size();
				if (valid)
				{
					std::vector<int> camMap(cameraIDs.size(), -1);
					for (auto &cam : pipeline.cameras)
					{
						bool found = false;
						for (int c = 0; c < cameraIDs.size(); c++)
						{
							if (cameraIDs[c] == cam->id)
							{
								camMap[c] = cam->index;
								found = true;
								break;
							}
						}
						if (!found) valid = false;
					}

					for (int c = 0; c < cameraIDs.size(); c++)
					{
						LOGC(LWarn, "Camera %d in saved observations is moved from saved slot %d to current index %d!", cameraIDs[c], c, camMap[c]);
					}

					if (valid)
					{ // Re-order cameras for all observations
						for (auto &marker : observations.markers)
						{
							std::vector<CameraSequences> camObs(pipeline.cameras.size());
							for (int c = 0; c < marker.cameras.size(); c++)
								camObs[camMap[c]] = std::move(marker.cameras[c]);
							marker.cameras = std::move(camObs);
						}
					}
				}
				if (valid)
				{
					LOG(LGUI, LDebug, "== Loaded calibration samples from '%s'!", obsPath.c_str());
					if (observations.markers.empty())
						GetState().errors.push("Failed to load any samples from file!");
					UpdateCalibrationRelations(pipeline, *pipeline.calibration.contextualLock(), observations);
					*pipeline.seqDatabase.contextualLock() = std::move(observations);
					UpdateSequences(true);
				}
				else
				{
					LOG(LGUI, LWarn, "== Calibration samples in '%s' did not match the currently available cameras!", obsPath.c_str());
					GetState().errors.push("Last saved observations had mismatching cameras!");
				}
			}
		}
		else GetState().errors.push("No observations file to load!");
	}

	if (state.mode == MODE_Replay && ImGui::TreeNode("Compare observations"))
	{
		if (ImGui::Button("Store Observations for Compare", SizeWidthFull()))
		{
			visState.observations.savedObs.push_back({});
			auto &obsCmp = visState.observations.savedObs.back();
			obsCmp.visPoints.resize(pipeline.cameras.size());
			for (auto &cam : pipeline.cameras)
				obsCmp.visPoints[cam->index] = cameraViews.at(cam->id).vis.observations.ptsStable;
			auto obs_lock = pipeline.seqDatabase.contextualRLock();
			obsCmp.markerCount = obs_lock->markers.size();
			obsCmp.obsCount = 0;
			for (auto &mk : obs_lock->markers)
				for (auto &cam : mk.cameras)
					for (auto &seq : cam.sequences)
						if (!seq.isInactive())
							obsCmp.obsCount += seq.length();
			ObsPointData pointData;
			addTriangulatableObservations(pointData, obs_lock->markers);
			obsCmp.triCount = pointData.totalSamples;
		}
		if (ImGui::BeginTable("ObsTable", 4,
			ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_PadOuterX))
		{
			ImGui::TableSetupScrollFreeze(0, 1); // Fix top row even with scrolling
			ImGui::TableSetupColumn("Markers");
			ImGui::TableSetupColumn("Samples");
			ImGui::TableSetupColumn("Tris");
			ImGui::TableSetupColumn("Delete", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
			ImGui::TableHeadersRow();

			for (int i = 0; i < visState.observations.savedObs.size(); i++)
			{
				auto &obsCmp = visState.observations.savedObs[i];
				ImGui::PushID(i);
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::AlignTextToFramePadding();
				ImGui::Text("%d", obsCmp.markerCount);
				ImGui::TableNextColumn();
				ImGui::Text("%d", obsCmp.obsCount);
				ImGui::TableNextColumn();
				ImGui::Text("%d", obsCmp.triCount);
				ImGui::TableNextColumn();
				bool select = visState.observations.visSavedObs == i;
				if (ImGui::Selectable("##Sel", &select, ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap))
				{
					visState.observations.visSavedObs = select? i : -1;
				}
				ImGui::SameLine(0, 0);
				if (CrossButton("##Del"))
				{
					if (visState.observations.visSavedObs == i)
						visState.observations.visSavedObs = -1;
					else if (visState.observations.visSavedObs > i)
						visState.observations.visSavedObs--;
					visState.observations.savedObs.erase(visState.observations.savedObs.begin()+i);
					i--;
				}
				ImGui::PopID();
			}
			ImGui::EndTable();
		}

		ImGui::TreePop();
	}

	EndSection();
}

void InterfaceState::UpdatePipelinePointCalib()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	ImVec2 ButtonSize = ImVec2(std::min(100.0f, SizeWidthDiv3().x), ImGui::GetFrameHeight());

	BeginSection("Calibration Procedure (?)");
	if (BeginInteractiveItemTooltip("procedure"))
	{
		ImGui::TextUnformatted("To calibrate the cameras (minimum 3), collect samples by waving around a small spherical marker.\n"
			"Then, depending on why you want to calibrate, hit Reconstruct, and then Optimise.");
		ImGui::TextLinkOpenURL("See Full Calibration Documentation", "https://docs.astertrack.dev/calib_cameras/");
		EndInteractiveItemTooltip();
	}
	auto &ptCalib = pipeline.pointCalib;
	auto startCalibration = [&](int typeFlags)
	{
		ptCalib.settings.typeFlags = typeFlags;
		if (typeFlags & 0b010)
			ptCalib.settings.maxSteps = ptCalib.state.numSteps + 10;
		ptCalib.planned = true;
		// Forgetting to do this is annoying as existing recorded data might get poisoned
		// In replay, a replays end means no more data, but a Restart might require this to be kept on
		if (state.mode != MODE_Replay)
			pipeline.recordSequences = false;
	};

	ImGui::BeginDisabled(ptCalib.control.stopping());
	if (ptCalib.control.running())
	{
		if (ImGui::Button(ptCalib.control.stopping()? "Stopping..." : "Stop", ButtonSize))
		{
			ptCalib.control.stop_source.request_stop();
		}
		ImGui::SameLine();
		if (ptCalib.settings.typeFlags & 0b001)
		{
			ImGui::TextUnformatted("Reconstructing Geometry...");
		}
		else if (ptCalib.settings.typeFlags & 0b010)
		{
			ImGui::Text("Optimising %d/%d...", ptCalib.state.numSteps, ptCalib.settings.maxSteps);
		}
	}
	else
	{
		auto getUniqueLenses = [&]()
		{
			int camUniqueLenses = pipeline.cameras.size();
			std::set<int> lenses;
			for (auto &camera : pipeline.cameras)
			{
				if (camera->calib.lensID >= 0 && !lenses.insert(camera->calib.lensID).second)
					camUniqueLenses--; // Remove duplicates
			}
			return camUniqueLenses;
		};
		if (ImGui::Button("Reconstruct", SizeWidthDiv2()))
		{
			if (pipeline.cameras.size() < 3)
				ImGui::OpenPopup("RecConfirm");
			else
			 	startCalibration(0b01);
		}
		ImGui::SetItemTooltip("Calculates an initial estimate of the camera setup (camera location, lens field of view).\n"
			"It is an important first step in calibration, but it cannot determine non-linear parameters like lens-distortion, and so errors may be relatively high.");
		if (ImGui::BeginPopup("RecConfirm"))
		{
			ImGui::Text("You have only %d cameras connected, with %d unique lenses.", (int)pipeline.cameras.size(), getUniqueLenses());
			ImGui::TextUnformatted("For a fully constrained system, you need at least 3 cameras.\n"
				"If all cameras use the same lens, you may be able to use just 2 cameras.\n"
				"In that case, select the same Lens for both cameras in the Camera List above.");
			if (ImGui::Button("Cancel", SizeWidthDiv2()))
				ImGui::CloseCurrentPopup();
			ImGui::SameLine();
			if (ImGui::Button("Reconstruct", SizeWidthDiv2()))
			{
				ImGui::CloseCurrentPopup();
				startCalibration(0b01);
			}
			ImGui::EndPopup();
		}
	
		ImGui::SameLine();

		auto getUnknownLenses = [&]()
		{
			int camUnknownLenses = 0;
			if (ptCalib.settings.options.sharedRadial)
			{
				for (auto &camera : pipeline.cameras)
					if (camera->calib.lensID < 0)
						camUnknownLenses++;
			}
			return camUnknownLenses;
		};
		if (ImGui::Button("Optimise", SizeWidthDiv2()))
		{
			if (getUnknownLenses() > 1)
				ImGui::OpenPopup("OptConfirm");
			else
			 	startCalibration(0b10);
		}
		ImGui::SetItemTooltip("Optimise the selected parameters of the cameras and their lenses.\n"
			"Required to determine non-linear parameters like lens-distortion.");
		if (ImGui::BeginPopup("OptConfirm"))
		{
			ImGui::Text("%d/%d cameras don't have their lenses configured.", getUnknownLenses(), (int)pipeline.cameras.size());
			ImGui::TextUnformatted(
				"The option to share distortion parameters among cameras using the same lens requires this.\n"
				"You may proceed and cameras without a lens assigned will be assumed to have a unique lens.");
			if (ImGui::Button("Cancel", SizeWidthDiv2()))
				ImGui::CloseCurrentPopup();
			ImGui::SameLine();
			if (ImGui::Button("Optimise", SizeWidthDiv2()))
			{
				ImGui::CloseCurrentPopup();
			 	startCalibration(0b10);
			}
			ImGui::EndPopup();
		}
	}
	ImGui::EndDisabled();

	ImGui::BeginDisabled(ptCalib.control.running());
	if (ImGui::TreeNode("Optimisation Options"))
	{
		auto &opt = ptCalib.settings.options;
		ImGui::Checkbox("Transform", &opt.position);
		opt.rotation = opt.position;
		ImGui::SetItemTooltip("Estimate each cameras position and rotation in the room.");
		ImGui::SameLine();
		ImGui::Checkbox("Lens", &opt.focalLen);
		opt.principal = opt.focalLen;
		ImGui::SetItemTooltip("Estimate basic lens parameters like focal length and principal point.");
		ImGui::SameLine();
		ImGui::Checkbox("Align", &opt.tangential);
		ImGui::SetItemTooltip("Estimate tangential distortion parameters.\n"
			"These compensate for misalignment of sensor and lens plane during lens installation.\n"
			"If disabled, will keep existing parameters.");

		ImGui::Checkbox("Distortions", &opt.radial);
		ImGui::SetItemTooltip("Enable estimation of radial distortion parameters.\n"
			"If disabled, will keep existing parameters.");
		ImGui::BeginDisabled(!opt.radial);
		ImGui::SameLine();
		ImGui::Checkbox("Share", &opt.sharedRadial);
		ImGui::SetItemTooltip("Share radial distortion parameters of cameras using the same Lens.\n"
			"This helps constrain them, yielding better results using less computing resources.\n"
			"This does not share parameters affected by lens installation\n"
			"    (like focal length, principal point and tangential distortion)\n"
			"NOTE: Lenses need to be setup for relevant cameras for this to have any effect at all!");
		ImGui::SameLine();
		ImGui::BeginDisabled(opt.radialOrder == 0);
		if (ImGui::Button("X", SizeFrame()))
			opt.radialOrder = 0;
		ImGui::EndDisabled();
		ImGui::SetItemTooltip("Disable radial distortion entirely.\n"
			"This will delete all existing parameters on next optimisation.");
		ImGui::SameLine();
		ImGui::BeginDisabled(opt.radialOrder == 2);
		if (ImGui::Button("M", SizeFrame()))
			opt.radialOrder = 2;
		ImGui::EndDisabled();
		ImGui::SetItemTooltip("Set radial distortion order to 2 (medium).\n"
			"This will delete the parameter of order 3 on next optimisation.");
		ImGui::SameLine();
		ImGui::BeginDisabled(opt.radialOrder == 3);
		if (ImGui::Button("H", SizeFrame()))
			opt.radialOrder = 3;
		ImGui::EndDisabled();
		ImGui::SetItemTooltip("Set radial distortion order to 3 (highest).\n"
			"Only use this if you have sufficient coverage and samples.\n"
			"You may also use 'Share' if all lenses are the same to further constrain the distortions.");
		ImGui::EndDisabled();

		ImGui::TreePop();
	}
	ImGui::EndDisabled();
	EndSection();

	BeginSection("Room Calibration (?)");
	if (BeginInteractiveItemTooltip("roomcalib"))
	{
		ImGui::TextUnformatted("After above calibration, the room rotation and scale is still undetermined.\n"
		"You need to specify at least 3 points on the floor plane with known distances to fix these.\n"
		"Currently the interface is rough, it expects exactly one marker in the tracking volume at a time. \n"
		"You will be able to select markers through the 3D View in the future.");
		ImGui::TextLinkOpenURL("See Full Calibration Documentation", "https://docs.astertrack.dev/calib_cameras/");
		EndInteractiveItemTooltip();
	}

	if (ptCalib.roomState.lastTransferSuccess == 2)
	{
		ImGui::Text("Existing Room Calibration from %d cameras", (int)ptCalib.roomState.unchangedCameras.size());
		if (ImGui::BeginItemTooltip())
		{
			ImGui::TextUnformatted("Transferred prior room calibration successfully by finding cameras that did not move:");
			for (auto &cam : ptCalib.roomState.unchangedCameras)
				ImGui::Text("Camera #%d unchanged with relational error %.2fmm", cam.first, cam.second);
			ImGui::TextUnformatted("NOTE: Whether there even was a prior room calibration is unknown!");
			ImGui::EndTooltip();
		}
	}
	else if (ptCalib.roomState.lastTransferSuccess == 1)
	{
		ImGui::Text("Uncertain Room Calibration from %d cameras", (int)ptCalib.roomState.unchangedCameras.size());
		if (ImGui::BeginItemTooltip())
		{
			ImGui::TextUnformatted("Transferred prior room calibration by relying on cameras who might have been re-mounted:");
			for (auto &cam : ptCalib.roomState.unchangedCameras)
				ImGui::Text("Camera #%d used within relational error %.2fmm", cam.first, cam.second);
			ImGui::TextUnformatted("NOTE: Whether there even was a prior room calibration is unknown!");
			ImGui::EndTooltip();
		}
	}
	else if (ptCalib.roomState.lastTransferSuccess == 0)
	{
		ImGui::TextUnformatted("Failed to transfer Room Calibration!");
		if (ImGui::BeginItemTooltip())
		{
			ImGui::TextUnformatted("Could not transfer prior room calibration as no 2 cameras appeared unchanged!");
			if (pipeline.pointCalib.roomState.lastTransferError)
				ImGui::Text("Error: %s", pipeline.pointCalib.roomState.lastTransferError->c_str());
			ImGui::TextUnformatted("NOTE: Whether there even was a prior room calibration is unknown!");
			ImGui::EndTooltip();
		}
	}
	else
	{
		ImGui::Text("Room calibration status unknown.");
	}

	auto roomCalib = ptCalib.room.contextualLock();
	ImGui::BeginDisabled(ptCalib.control.running() || calibState.numUncalibrated || calibState.relUncertain);

	ImGui::AlignTextToFramePadding();
	ImGui::Text("%d floor points", (int)roomCalib->floorPoints.size());
	SameLineTrailing(GetBarWidth(ImGui::GetFrameHeight(), 2));
	if (ImGui::Button("-", ImVec2(ImGui::GetFrameHeight(), 0)))
	{
		if (!roomCalib->floorPoints.empty())
			roomCalib->floorPoints.pop_back();
	}
	ImGui::SetItemTooltip("Remove the last point added.");
	ImGui::SameLine();
	ImGui::BeginDisabled(!roomCalib->floorPoints.empty() && roomCalib->floorPoints.back().sampling);
	if (ImGui::Button("+", ImVec2(ImGui::GetFrameHeight(), 0)))
	{
		roomCalib->floorPoints.push_back({});
		roomCalib->floorPoints.back().sampling = true;
	}
	ImGui::SetItemTooltip("Put a Marker on the ground, and push this button to record it over the period of one second.");
	ImGui::EndDisabled();

	ScalarInput<float>("Distance 1-2", "mm", &roomCalib->distance12, 1.0f, 5000.0f, 1, 1000, "%.1f");
	if (ImGui::BeginItemTooltip())
	{
		ImGui::TextUnformatted("Distance between the first two points, used to calibrate scale.");
		if (roomCalib->floorPoints.size() >= 2)
			ImGui::Text("With the current scale, they are %.2fmm apart.",
				(roomCalib->floorPoints[0].pos-roomCalib->floorPoints[1].pos).norm()*1000);
		ImGui::EndTooltip();
	}

	ImGui::BeginDisabled(roomCalib->floorPoints.size() < 3);
	if (ImGui::Button("Calibrate Floor", SizeWidthFull()))
	{
		startCalibration(0b100);
	}
	ImGui::SetItemTooltip("Use at least 3 points (not in a line) to calibrated the floor of the room.");
	ImGui::EndDisabled();

	static float floorHeight = -1.0f;
	ScalarInput<float>("Floor Height", "mm", &floorHeight, -5000.0f, 5000.0f, 1, 1000, "%.1f");
	if (ImGui::Button("Add to Floor", SizeWidthFull()))
	{
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(100));
		if (pipeline_lock.owns_lock())
		{
			for (auto &camera : pipeline.cameras)
			{
				camera->calibBackup = camera->calib;
				camera->calib.transform.translation().z() += floorHeight;
			}

			// Re-evaluate positions incase calibration changed since observation
			auto calibs = pipeline.getCalibs();
			for (auto &point : roomCalib->floorPoints)
				point.update(calibs);

			SignalCameraCalibUpdate(calibs);
		}
	}
	ImGui::SetItemTooltip("Adjust the height of the floor-plane.");

	if (ImGui::Button("Flip Vertically", SizeWidthFull()))
	{
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(100));
		if (pipeline_lock.owns_lock())
		{
			Eigen::Matrix3d orientation = Eigen::Quaterniond::FromTwoVectors(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
			for (auto &camera : pipeline.cameras)
			{
				camera->calibBackup = camera->calib;
				camera->calib.transform.linear() = orientation * camera->calib.transform.linear();
				camera->calib.transform.translation() = orientation * camera->calib.transform.translation();
				camera->calib.UpdateDerived();
			}

			// Re-evaluate positions incase calibration changed since observation
			auto calibs = pipeline.getCalibs();
			for (auto &point : roomCalib->floorPoints)
				point.update(calibs);

			SignalCameraCalibUpdate(calibs);
		}
	}
	ImGui::SetItemTooltip("Flip the cameras vertically if your setup is inverted or the automatic orientation was wrong.");

	if (ImGui::Button("Undo Once", SizeWidthFull()))
	{ // TODO: Disable this button if floor is not calibrated yet (or a new calibration exists)
		// Or maybe simple store backups as transforms, and apply transforms (default being Isometry), so no chance of "restoring" a stale calibration exists
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(100));
		if (pipeline_lock.owns_lock())
		{
			for (auto &camera : pipeline.cameras)
			{
				if (camera->calibBackup.valid())
					camera->calib = camera->calibBackup;
			}
			SignalCameraCalibUpdate(pipeline.getCalibs());
		}
	}
	ImGui::SetItemTooltip("Restore calibration to backup created before last floor calibration.");

	ImGui::EndDisabled();

	EndSection();
}