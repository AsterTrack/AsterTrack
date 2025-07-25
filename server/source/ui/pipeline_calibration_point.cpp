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

void InterfaceState::UpdatePipelineCalibSection()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	ImVec2 ButtonSize = ImVec2(std::min(100.0f, SizeWidthDiv3().x), ImGui::GetFrameHeight());

	BeginSection("Camera Calibration");
	{
		const auto &ptCalib = pipeline.pointCalib;
		auto errors = ptCalib.state.errors;
		float PixelFactor = (pipeline.cameras.empty()? 1280 : pipeline.cameras.front()->mode.widthPx)/2.0f; // For external logging, already dynamic
		if (ptCalib.control.thread)
		{
			if (ptCalib.settings.typeFlags & 0b01)
			{
				ImGui::TextUnformatted("");
			}
			else if (ptCalib.settings.typeFlags & 0b10)
			{
				ImGui::Text("%.4fpx += %.4fpx error, %.4fpx max",
					errors.mean*PixelFactor,
					errors.stdDev*PixelFactor,
					errors.max*PixelFactor);
			}
		}
		else if (calibState.numUncalibrated == 0 && visState.incObsUpdate.markerCount > 0)
		{
			ImGui::Text("%.4fpx += %.4fpx error, %.4fpx max",
				errors.mean*PixelFactor,
				errors.stdDev*PixelFactor,
				errors.max*PixelFactor);
		}
		else if (calibState.numUncalibrated == 0)
			ImGui::TextUnformatted("No data to check calibration against!");
		else
			ImGui::TextUnformatted("No calibration nor samples available!");
	}
	{
		if (ImGui::Button("Reset##Calibration", ButtonSize))
		{
			{
				std::unique_lock pipeline_lock(pipeline.pipelineLock);
				for (auto &cam : pipeline.cameras)
				{
					cam->calib = {};
					cam->calib.index = cam->index;
				}
			}

			pipeline.calibration.contextualLock()->init(pipeline.cameras.size());
			UpdateErrorFromObservations(pipeline);
			UpdateCalibrations();
			state.cameraCalibsDirty = true;
		}
		ImGui::SameLine();
		if (SaveButton("Save##Calibration", ButtonSize, state.cameraCalibsDirty))
		{
			storeCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
			state.cameraCalibsDirty = false;
		}
		ImGui::SameLine();
		if (ImGui::Button("Load##Calibration", ButtonSize))
		{
			parseCameraCalibrations("store/camera_calib.json", state.cameraCalibrations);
			state.cameraCalibsDirty = false;
			{
				std::unique_lock pipeline_lock(pipeline.pipelineLock);
				AdoptNewCalibrations(state.pipeline, state.cameraCalibrations, false);
			}
			{
				auto lock = folly::detail::lock(folly::detail::wlock(pipeline.calibration), folly::detail::rlock(pipeline.seqDatabase));
				UpdateCalibrationRelations(pipeline, *std::get<0>(lock), *std::get<1>(lock));
			}
			UpdateErrorFromObservations(pipeline);
			UpdateCalibrations();

			LOG(LGUI, LDebug, "== Loaded %d camera calibrations:\n", (int)state.cameraCalibrations.size());
			DebugCameraParameters(state.cameraCalibrations);
		}
	}
	EndSection();
}

void InterfaceState::UpdatePipelineObservationSection()
{
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	ImVec2 ButtonSize = ImVec2(std::min(100.0f, SizeWidthDiv3().x), ImGui::GetFrameHeight());

	BeginSection("Marker Observations");
	ImGui::Checkbox("Record Observations", &pipeline.recordSequences);
	ImGui::SetItemTooltip("Record visible markers into observations as continous sequences.");
	ImGui::TextUnformatted(calibSamples.contextualRLock()->c_str());

	if (ImGui::Button("Clear", ButtonSize))
	{
		pipeline.seqDatabase.contextualLock()->clear();
		UpdateErrorFromObservations(pipeline);
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
		dumpSequenceDatabase(obsPath, *pipeline.seqDatabase.contextualRLock(), cameraIDs);
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
			SequenceData observations = parseSequenceDatabase(obsPath, cameraIDs);
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
				UpdateCalibrationRelations(pipeline, *pipeline.calibration.contextualLock(), observations);
				*pipeline.seqDatabase.contextualLock() = std::move(observations);
				UpdateErrorFromObservations(pipeline);
				UpdateSequences(true);
			}
			else
			{
				LOG(LGUI, LWarn, "== Calibration samples in '%s' did not match the currently available cameras!", obsPath.c_str());
			}
		}
	}

	if (state.mode == MODE_Replay && ImGui::TreeNode("Compare observations"))
	{
		if (ImGui::Button("Store Observations for Compare", SizeWidthFull()))
		{
			visState.observations.savedObs.push_back({});
			auto &obsCmp = visState.observations.savedObs.back();
			obsCmp.visPoints.resize(pipeline.cameras.size());
			for (auto &cam : pipeline.cameras)
				obsCmp.visPoints[cam->index] = cameraViews.at(cam->id).vis.observations.contextualRLock()->ptsStable;
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

	BeginSection("Algorithm");
	auto &ptCalib = pipeline.pointCalib;
	ImGui::BeginDisabled(ptCalib.control.stopping());
	if (ptCalib.control.running())
	{
		if (ImGui::Button(ptCalib.control.stopping()? "Stopping..." : "Stop", ButtonSize))
		{
			ptCalib.control.stop_source.request_stop();
		}
		ImGui::SameLine();
		if (ptCalib.settings.typeFlags & 0b01)
		{
			ImGui::TextUnformatted("Reconstructing Geometry...");
		}
		else if (ptCalib.settings.typeFlags & 0b10)
		{
			ImGui::Text("Optimising %d/%d...", ptCalib.state.numSteps, ptCalib.settings.maxSteps);
		}
	}
	else
	{
		if (ImGui::Button("Reconstruct", SizeWidthDiv2()))
		{
			ptCalib.settings.typeFlags = 0b01;
			ptCalib.planned = true;
			// Forgetting to do this is annoying as existing recorded data might get poisoned
			pipeline.recordSequences = false;
		}
		ImGui::SetItemTooltip("Calculates an initial estimate of the camera setup (camera location, lens field of view).\n"
			"It is an important first step in calibration, but it cannot determine non-linear parameters like lens-distortion, and so errors may be relatively high.");
		ImGui::SameLine();
		if (ImGui::Button("Optimise", SizeWidthDiv2()))
		{
			ptCalib.settings.typeFlags = 0b10;
			ptCalib.settings.maxSteps = ptCalib.state.numSteps + 10;
			ptCalib.planned = true;
			// Forgetting to do this is annoying as existing recorded data might get poisoned
			pipeline.recordSequences = false;
		}
		ImGui::SetItemTooltip("Optimise the selected parameters of the cameras and their lenses.\n"
			"Required to determine non-linear parameters like lens-distortion.");
	}
	ImGui::EndDisabled();

	ImGui::BeginDisabled(ptCalib.control.running());
	ImGui::Text("Optimisation Options:");
	ImGui::Indent();
	{
		auto &opt = ptCalib.settings.options;
		opt.groupedIntrinsics = false;
		ImGui::Checkbox("Transform", &opt.position);
		opt.rotation = opt.position;
		ImGui::SameLine();
		ImGui::Checkbox("Lens", &opt.focalLen);
		opt.principal = opt.focalLen;
		ImGui::Text("Distortion Order:");
		ImGui::SameLine();
		ImGui::Checkbox("2nd", &opt.distk1);
		opt.distk2 = opt.distk1;
		ImGui::SameLine();
		ImGui::Checkbox("5th", &opt.distk3);
		opt.distp1 = opt.distk3;
		opt.distp2 = opt.distk3;
	}
	ImGui::Unindent();
	ImGui::EndDisabled();
	EndSection();

	BeginSection("Room Calibration (?)");
	ImGui::SetItemTooltip("After above calibration, the room rotation and scale is still undetermined.\n"
		"You need to specify some points on the floor plane with known distances to fix these.\n"
		"Currently the interface is rough, it expects exactly one marker in the tracking volume at a time. \n"
		"You will be able to select markers through the 3D View in the future.");

	auto &roomCalib = ptCalib.room;
	ImGui::BeginDisabled(ptCalib.control.running() || calibState.numUncalibrated || calibState.relUncertain);

	std::string label;
	if (roomCalib.floorPoints.size() < 3)
		label = asprintf_s("%d floor points, need 3 or more", (int)roomCalib.floorPoints.size());
	else
	 	label = asprintf_s("%d floor points - distance of 1-2 is %.2fmm", (int)roomCalib.floorPoints.size(),
			(roomCalib.floorPoints[0].pos-roomCalib.floorPoints[1].pos).norm()*1000);
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(label.c_str());
	SameLineTrailing(GetBarWidth(ImGui::GetFrameHeight(), 2));
	if (ImGui::Button("-", ImVec2(ImGui::GetFrameHeight(), 0)))
	{
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(50));
		if (pipeline_lock.owns_lock() && !roomCalib.floorPoints.empty())
			roomCalib.floorPoints.pop_back();
	}
	ImGui::SetItemTooltip("Remove the last point added.");
	ImGui::SameLine();
	ImGui::BeginDisabled(!roomCalib.floorPoints.empty() && roomCalib.floorPoints.back().sampling);
	if (ImGui::Button("+", ImVec2(ImGui::GetFrameHeight(), 0)))
	{
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(50));
		if (pipeline_lock.owns_lock())
		{
			roomCalib.floorPoints.push_back({});
			roomCalib.floorPoints.back().sampling = true;
		}
	}
	ImGui::SetItemTooltip("Put a Marker on the ground, and push this button to record it over the period of one second.");
	ImGui::EndDisabled();

	ScalarInput<float>("Distance 1-2", "mm", &roomCalib.distance12, 1.0f, 5000.0f, 1, 1000);
	ImGui::SetItemTooltip("Distance between the first two points, used to calibrate scale. In millimeter.");

	ImGui::SetCursorPosX(GetRightAlignedCursorPos(SizeWidthDiv3_2().x));
	if (ImGui::Button("Calibrate Floor", SizeWidthDiv3_2()))
	{
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(200));
		if (pipeline_lock.owns_lock())
			CalibrateRoom(pipeline);
	}
	ImGui::SetItemTooltip("Use at least 3 points to calibrated the floor of the room.");

	static float floorHeight = -1.0f;
	ScalarInput<float>("Floor Height", "mm", &floorHeight, -5000.0f, 5000.0f, 1, 1000);
	ImGui::SetCursorPosX(GetRightAlignedCursorPos(SizeWidthDiv3_2().x));
	if (ImGui::Button("Add to Floor", SizeWidthDiv3_2()))
	{
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(50));
		if (pipeline_lock.owns_lock())
		{
			for (auto &camera : pipeline.cameras)
			{
				camera->calibBackup = camera->calib;
				camera->calib.transform.translation().z() += floorHeight;
			}

			// Re-evaluate positions incase calibration changed since observation
			auto calibs = pipeline.getCalibs();
			for (auto &point : ptCalib.room.floorPoints)
				point.update(calibs);
		}
	}
	ImGui::SetItemTooltip("Adjust the height of the floor-plane.");

	if (ImGui::Button("Undo Once", SizeWidthFull()))
	{ // TODO: Disable this button if floor is not calibrated yet (or a new calibration exists)
		// Or maybe simple store backups as transforms, and apply transforms (default being Isometry), so no chance of "restoring" a stale calibration exists
		std::unique_lock pipeline_lock(pipeline.pipelineLock, std::chrono::milliseconds(50));
		if (pipeline_lock.owns_lock())
		{
			for (auto &camera : pipeline.cameras)
			{
				if (camera->calibBackup.valid())
					camera->calib = camera->calibBackup;
			}
		}
	}
	ImGui::SetItemTooltip("Restore calibration to backup created before last floor calibration.");

	ImGui::EndDisabled();

	EndSection();
}