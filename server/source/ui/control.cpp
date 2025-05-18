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
#include "config.hpp"

#include "imgui/imgui_custom.hpp"
#include "imgui/imgui_onDemand.hpp"

#include "util/debugging.hpp" // Provide controls here, even if it's not technically restricted to simulation mode

void InterfaceState::UpdateControl(InterfaceWindow &window)
{
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ServerState &state = GetState();
	PipelineState &pipeline = state.pipeline;

	if (state.mode == MODE_Device)
	{
		AddOnDemandText("Frame 00000000", [](const ImDrawList* dl, const ImDrawCmd* dc)
		{
			RenderOnDemandText(*static_cast<OnDemandItem*>(dc->UserCallbackData), "Frame %d", GetState().pipeline.frameNum.load());
		});
	}
	else if (state.mode == MODE_Simulation || state.mode == MODE_Replay)
	{
		bool debug = dbg_debugging.load(), breaking = dbg_isBreaking.load();
		BeginSection(state.mode == MODE_Simulation? "Simulation" : "Replay");
		ImGui::BeginDisabled(breaking);
		{ // Show controls for simulation frame advancing
			bool advancing = state.simAdvance.load() != 0;
			ImGui::AlignTextToFramePadding();
			AddOnDemandText("Frame 00000000", [](const ImDrawList* dl, const ImDrawCmd* dc)
			{
				RenderOnDemandText(*static_cast<OnDemandItem*>(dc->UserCallbackData), "Frame %d", GetState().pipeline.frameNum.load());
			});

			SameLinePos(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);
			if (ImGui::Button(advancing? "Halt##Halt" : "Continue##Halt", SizeWidthDiv3()))
			{
				advancing = !advancing;
				if (advancing)
				{ // Continue freely (-1) or limited steps (positive integers)
					state.simAdvance = -1;
					state.simAdvance.notify_all();
				}
				else
				{ // Halt
					state.simAdvance = 0;
				}
			}
			ImGui::SameLine();
			ImGui::BeginDisabled(advancing);
			if (ImGui::Button("Advance", SizeWidthDiv3()))
			{
				state.simAdvance = 1;
				state.simAdvance.notify_all();
			}
			ImGui::EndDisabled();
		}

		ImGui::Checkbox("Quickly", &state.simAdvanceQuickly);

		if (state.mode == MODE_Replay)
		{ // Show additional advancing control line in replay mode
			SameLinePos(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);
			if (ImGui::Button("Next Image", SizeWidthDiv3()))
			{
				state.simAdvance = -2;
				state.simAdvance.notify_all();
			}
		}

		SameLineTrailing(SizeWidthDiv3().x);
		if (ImGui::Button("Advance 10", SizeWidthDiv3()))
		{
			state.simAdvance = 10;
			state.simAdvance.notify_all();
		}

		ImGui::EndDisabled();

		{ // Show controls for interactive debugging
			ImGui::AlignTextToFramePadding();
			if (breaking)
			{
				ImGui::PushStyleColor(ImGuiCol_Text, isDarkMode? IM_COL32(220, 150, 150, 255) : IM_COL32(150, 50, 50, 255));
				ImGui::TextUnformatted("Debug Break!");
				ImGui::PopStyleColor();
			}
			else ImGui::TextUnformatted("Debugging:");
			SameLinePos(SizeWidthDiv3().x + ImGui::GetStyle().ItemSpacing.x);

			ImGui::SetNextItemWidth(SizeWidthDiv3().x);
			int level = dbg_debugging.load();
			if (ImGui::InputInt("", &level))
				dbg_debugging = std::clamp(level, 0, 5);
			ImGui::SetItemTooltip("Sets the debug level.");
			ImGui::SameLine();

			if (ImGui::Button(debug? "Disable##Debug" : "Enable##Debug", SizeWidthDiv3_Div2()))
			{
				dbg_debugging = !dbg_debugging;
				debug = !debug;
				if (!debug && breaking)
				{
					breaking = false;
					dbg_isBreaking = false;
					dbg_isBreaking.notify_all();
				}
			}
			ImGui::SameLine();

			ImGui::BeginDisabled(!debug || !breaking);
			if (ImGui::Button("Step", SizeWidthDiv3_Div2()))
			{
				dbg_isBreaking = false;
				dbg_isBreaking.notify_all();
			}
			ImGui::EndDisabled();
		}

		if (state.mode == MODE_Simulation)
		{
			auto sim_lock = pipeline.simulation.contextualLock();

			ImGui::TextUnformatted("Simulated Objects");
			for (auto &object : sim_lock->objects)
			{
				ImGui::PushID(object.target->id);
				ImGui::Checkbox(object.target->label.c_str(), &object.enabled);
				SameLineTrailing(SizeWidthDiv2().x);
				ImGui::SetNextItemWidth(SizeWidthDiv2().x);
				const char* motionPresetLabels[] = { "Room", "Wide", "Center", "Custom" };
				if (ImGui::Combo("##Motion", &object.motionPreset, motionPresetLabels, MotionCustom+1))
				{
					if (object.motionPreset != MotionCustom)
						object.motion = motionPresets[object.motionPreset];
				}
				ImGui::PopID();
			}
		}
		else if (state.mode == MODE_Replay)
		{
			ImGui::BeginDisabled(!state.isStreaming);
			if (state.recordFrameCount > 0)
			{
				ImGui::AlignTextToFramePadding();
				AddOnDemandText("Replaying 00000000 / 00000000 frames", [](const ImDrawList* dl, const ImDrawCmd* dc)
				{
					RenderOnDemandText(*static_cast<OnDemandItem*>(dc->UserCallbackData), "Replaying %ld / %ld frames", GetState().recordReplayFrame, GetState().recordFrameCount);
				});
				SameLineTrailing(SizeWidthDiv3().x);
				if (ImGui::Button("Restart", SizeWidthDiv3()))
				{
					// Stop advancing replay
					int prevState = state.simAdvance;
					state.simAdvance = 0;
					state.simWaiting.wait(false);
					// Reset pipeline after the last frame has processed
					ResetPipelineStreaming(state.pipeline);
					ResetPipelineData(state.pipeline);
					UpdateSequences(true);
					InitPipelineStreaming(state.pipeline);
					state.recordReplayFrame = 0;
					// Continue advancing
					state.simAdvance = prevState;
					state.simAdvance.notify_all();
				}
			}

			ImGui::Text("Jump To Frame");
			SameLineTrailing(SizeWidthDiv3_2().x);
			ImGui::SetNextItemWidth(SizeWidthDiv3().x);
			ImGui::InputScalar("##Frame", ImGuiDataType_U32, &frameJumpTarget);
			ImGui::SameLine();
			if (ImGui::Button("Jump", SizeWidthDiv3()))
			{
				// Stop advancing replay
				int prevState = state.simAdvance;
				state.simAdvance = 0;
				state.simWaiting.wait(false);
				// Jump to frame after last frame has been processed
				std::shared_ptr<FrameRecord> frame = nullptr;
				auto framesRecord = pipeline.record.frames.getView();
				if (frameJumpTarget < framesRecord.size() && framesRecord[frameJumpTarget])
					frame = framesRecord[frameJumpTarget];
				else
				{ // Try initialising with stored record
					auto framesStored = state.stored.frames.getView();
					if (frameJumpTarget < framesStored.size())
						frame = framesStored[frameJumpTarget];
				}
				if (frame)
				{
					for (int f = frameJumpTarget; f < framesRecord.endIndex(); f++)
						if (framesRecord[f]) framesRecord[f]->finishedProcessing = false;
					AdoptFrameRecordState(pipeline, *frame);
					state.recordReplayFrame = frameJumpTarget+1;
				}
				// Continue advancing
				state.simAdvance = prevState;
				state.simAdvance.notify_all();
			}
			ImGui::EndDisabled();
		}

		EndSection();
	}

	if (state.mode != MODE_Replay)
	{ // Allow recording and storing of frame sections
		BeginSection("Recording");

		if (ImGui::Button(recordSectionStart < 0? "Start Section##Section" : "Stop Section##Section", SizeWidthDiv2()))
		{
			if (recordSectionStart < 0)
			{ // TODO: Hold reference to view to prevent frame range from being deleted once garbage collect is implemented? 
				recordSectionStart = pipeline.record.frames.getView().endIndex();
				pipeline.keepFrameRecords = true;
			}
			else
			{
				recordSections.emplace_back(recordSectionStart, pipeline.record.frames.getView().endIndex());
				recordSectionStart = -1;
				pipeline.keepFrameRecords = pipeline.keepFrameRecordsDefault;
			}
		}
		ImGui::SameLine();
		ImGui::BeginDisabled(pipeline.record.frames.getView().empty());
		if (ImGui::Button("Save All Frames", SizeWidthDiv2()))
		{
			auto framesRecord = pipeline.record.frames.getView();
			recordSections.emplace_back(framesRecord.beginIndex(), framesRecord.endIndex(), true);
		}
		ImGui::EndDisabled();
		ImGui::BeginDisabled(true);
		ImGui::Checkbox("Record by default", &pipeline.keepFrameRecordsDefault);
		ImGui::EndDisabled();
		SameLineTrailing(SizeWidthDiv2().x);
		ImGui::Checkbox("Save Images", &pipeline.keepFrameImages);
		ImGui::Checkbox("Save Tracking Results", &saveTrackingResults);

		if (ImGui::BeginTable("Sections", 4, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
		{
			ImGui::TableSetupColumn("Start");
			ImGui::TableSetupColumn("Frames");
			ImGui::TableSetupColumn("Save##Header", ImGuiTableColumnFlags_WidthStretch, 3);
			ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
			ImGui::TableHeadersRow();

			for (int i = 0; i < recordSections.size(); i++)
			{
				auto &section = recordSections[i];
				ImGui::PushID(i);
				ImGui::AlignTextToFramePadding();
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::Text("%d", section.begin);
				ImGui::TableNextColumn();
				ImGui::Text("%d", section.end-section.begin);
				ImGui::TableNextColumn();
				if (section.path.empty())
				{
					if (section.forceSave || ImGui::Button("Save", ImVec2(ImGui::GetColumnWidth(2), 0)))
					{
						// Find path
						const char* capturePathFmt = "dump/frame_capture_%d.json";
						int saveIndex = findLastFileEnumeration(capturePathFmt)+1;
						section.path = asprintf_s(capturePathFmt, saveIndex);
						// Perform write in a separate thread to prevent blocking UI
						// TODO: Memory Leak. Thread never joined
						new std::thread([](auto section)
						{
							PipelineState &pipeline = GetState().pipeline;
							// Copy camera ids
							std::vector<CameraConfigRecord> cameras;
							for (auto &cam : pipeline.cameras)
								cameras.emplace_back(cam->id, cam->mode.widthPx, cam->mode.heightPx);
							// Write to path
							{
								auto framesRecord = pipeline.record.frames.getView();
								if (framesRecord.endIndex() < section.end)
								{ // Already deleted
									LOG(LGUI, LWarn, "The section to be saved has already been deleted internally!");
									return;
								}
							}
							dumpRecording(section.path, cameras, pipeline.record, section.begin, section.end);
							dumpTrackingResults(section.path, pipeline.record, section.begin, section.end, 0);
							for (auto &s : GetUI().recordSections)
								if (s.begin == section.begin && s.end == section.end)
									s.saved = true;
							GetUI().RequestUpdates();
						}, section);
					}
				}
				else if (!section.saved)
				{
					ImGui::TextUnformatted("Saving...");
				}
				else
				{
					ImGui::Text("%s", section.path.c_str());
				}
				ImGui::TableNextColumn();
				if (CrossButton("Del"))
				{
					recordSections.erase(std::next(recordSections.begin(), i));
					i--;
				}
				ImGui::PopID();
			}
			ImGui::EndTable();
		}

		EndSection();
	}

	struct FrameRange
	{
		struct Results
		{
			StatDistf frames, samples;
			int losses, detections;
		};
		unsigned int begin;
		unsigned int end;
		bool dirty, updated;
		Results baseline, results;
	};
	static std::vector<FrameRange> frameRanges;
	static std::unique_ptr<std::jthread> controlThread;

	if (state.mode == MODE_Replay && windows[WIN_TRACKING_PARAMS].open
		&& ImGui::CollapsingHeader("Optimising Tracking Parameters"))
	{
		ImGui::PushID("OptParam");

		static std::mutex mutex;

		static auto controlFunction = [](std::stop_token stop_token)
		{
			ServerState &state = GetState();
			PipelineState &pipeline = state.pipeline;

			// Assume control over pipeline processing
			state.simAdvance = 0;
			state.simWaiting.wait(false);

			while (!stop_token.stop_requested())
			{
				if (state.simAdvance.load() < 0 || !state.isStreaming)
				{ // Halt condition
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					continue;
				}
				FrameRange range;
				bool gotRange = false;
				{
					std::unique_lock lock(mutex);
					for (auto &r : frameRanges)
					{
						if (r.dirty)
						{
							r.dirty = false;
							r.updated = false;
							range = r;
							gotRange = true;
							break;
						}
					}
				}
				if (!gotRange)
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					continue;
				}
				// Ensure we have decent parameters
				pipeline.params.detect.enable2DAsync = false;
				// Instruct pipeline to process given frame range
				int frameJumpTarget = range.begin-1;
				// Jump to frame after last frame has been processed
				std::shared_ptr<FrameRecord> frame = nullptr;
				auto framesRecord = pipeline.record.frames.getView();
				if (frameJumpTarget < framesRecord.size() && framesRecord[frameJumpTarget])
					frame = framesRecord[frameJumpTarget];
				else
				{ // Try initialising with stored record
					auto framesStored = state.stored.frames.getView();
					if (frameJumpTarget < framesStored.size())
						frame = framesStored[frameJumpTarget];
				}
				if (!frame) continue;
				for (int f = frameJumpTarget; f < range.end && f < framesRecord.endIndex(); f++)
					if (framesRecord[f]) framesRecord[f]->finishedProcessing = false;
				AdoptFrameRecordState(pipeline, *frame);
				state.recordReplayFrame = frameJumpTarget+1;
				// Quickly advance through frame range
				state.simAdvanceQuickly = true;
				state.simAdvance = range.end-range.begin;
				state.simAdvance.notify_all();
				int count;
				while ((count = state.simAdvance.load()) > 0 && !stop_token.stop_requested())
					state.simAdvance.wait(count);
				if (stop_token.stop_requested()) break;
				if (state.simAdvance.load() < 0) continue;
				// Tracking completed and pipeline is stalled, read results before handling next frame range
				framesRecord = pipeline.record.frames.getView(); // New view
				auto begin = framesRecord.pos(range.begin), end = framesRecord.pos(range.end);
				FrameRange::Results results = {};
				LOG(LGUI, LDebug, "Updating range from frame %d - %d", range.begin, range.end);
				for (auto frame = begin; frame != end; frame++)
				{
					for (auto &trackRecord : frame->get()->trackers)
					{
						results.frames.update(trackRecord.error.mean);
						for (int i = 0; i < trackRecord.error.samples; i++)
							results.samples.update(trackRecord.error.mean);
						LOG(LGUI, LDebug, "    Frame %d, error %.2fpx", frame->get()->num-range.begin, trackRecord.error.mean*PixelFactor);
						if (trackRecord.result.isFailure())
							results.losses++;
						else if (trackRecord.result.isDetected())
							results.detections++;

					}
				}
				LOG(LGUI, LDebug, "Final error %.2fpx, %d samples, %d frames, %d losses", results.samples.avg*PixelFactor, results.samples.num, results.frames.num, results.losses);
				// Update range
				std::unique_lock lock(mutex);
				for (auto &r : frameRanges)
				{
					if (r.begin == range.begin && r.end == range.end)
					{
						r.updated = true;
						r.results = results;
						break;
					}
				}
			}
		};
		if (!controlThread)
			controlThread = std::make_unique<std::jthread>(controlFunction);

		std::unique_lock lock (mutex);

		if (frameRelevantParametersDirty)
		{
			for (auto &range : frameRanges)
				range.dirty = true;
			frameRelevantParametersDirty = false;
		}

		BeginSection("Frame Ranges");

		ImGui::SetNextItemWidth(SizeWidthDiv3().x);
		static unsigned int addFrameRange[2] = { 10, 20 };
		//ImGui::InputScalarN("##Frame", ImGuiDataType_U32, &addFrameRange, 2);
		static std::string frameRangeInput = "10-20";
		ImGui::InputText("##Frame", &frameRangeInput);
		ImGui::SameLine();
		if (ImGui::Button("Add", SizeWidthDiv3()))
		{
			std::stringstream ss(frameRangeInput);
			std::string range;
			while (std::getline(ss, range, ';'))
			{
				if (sscanf(range.c_str(), "%u-%u", addFrameRange+0, addFrameRange+1) != 2) continue;
				if (addFrameRange[0] >= 0 && addFrameRange[0] < addFrameRange[1])
					frameRanges.push_back({ addFrameRange[0], addFrameRange[1], true, false });
			}
			frameRangeInput.clear();
		}
		ImGui::SameLine();
		if (ImGui::Button("Copy", SizeWidthDiv3()))
		{
			std::stringstream ss;
			for (auto &range : frameRanges)
				ss << range.begin << "-" << range.end << ";";
			ImGui::SetClipboardText(ss.str().c_str());
		}
		
		if (ImGui::Button("Add Critical Frame Ranges", SizeWidthFull()))
		{
			const int reach = 10;
			bool inRange = false;
			addFrameRange[1] = 0;
			auto handleFrame = [&](const FrameRecord &frame)
			{
				bool hasFailure = std::any_of(frame.trackers.begin(), frame.trackers.end(),
					[](const auto &tracker){ return tracker.result.isFailure(); });
				if (!hasFailure) return;
				if (inRange && frame.num - addFrameRange[1] > reach*2)
				{ // Separate from last range
					frameRanges.push_back({ (unsigned int)std::max<long>(0, (long)addFrameRange[0]-reach), addFrameRange[1]+reach, true, false });
					inRange = false;
				}
				if (!inRange) // New range
					addFrameRange[0] = frame.num;
				addFrameRange[1] = frame.num;
				inRange = true;
			};
			auto framesRecord = pipeline.record.frames.getView();
			for (auto &frame : framesRecord)
				if (frame) handleFrame(*frame);
			auto framesStored = state.stored.frames.getView();
			for (int f = addFrameRange[1]+1; f < framesStored.endIndex(); f++)
				if (framesStored[f]) handleFrame(*framesStored[f]);
			if (inRange)
				frameRanges.push_back({ (unsigned int)std::max<long>(0, (long)addFrameRange[0]-reach), addFrameRange[1]+reach, true, false });
		}

		FrameRange::Results baseline = {}, results = {};
		ImVec4 normal = ImGui::GetStyleColorVec4(ImGuiCol_Text), better = ImVec4(0.33f, 0.66f, 0.4f, 1.0f), worse = ImVec4(0.66f, 0.33f, 0.4f, 1.0f);
		auto more = [&](bool n, double r, double b, double t) { return n? normal : (r+t < b? worse : (r-t > b? better : normal)); };
		auto less = [&](bool n, double r, double b, double t) { return n? normal : (r-t > b? worse : (r+t < b? better : normal)); };
		auto compare = [&](const FrameRange::Results &base, const FrameRange::Results &res)
		{
			bool n = base.samples.num == 0; // No baseline
			auto compare = [&](const char* label, const StatDistf &dist, const StatDistf &cmpDist)
			{
				ImGui::BeginGroup();
				ImGui::TextUnformatted("On average");
				ImGui::SameLine();
				ImGui::TextColored(less(n, dist.avg, cmpDist.avg, 0.01*PixelSize), "%.4fpx +- %.4fpx", dist.avg*PixelFactor, dist.stdDev()*PixelFactor);
				ImGui::SameLine();
				ImGui::TextColored(less(n, dist.max, cmpDist.max, 0.01*PixelSize), "%.4fpx max", dist.max*PixelFactor);
				ImGui::SameLine();
				ImGui::TextUnformatted("across");
				ImGui::SameLine();
				ImGui::TextColored(more(n, dist.num, cmpDist.num, 0), "%d %s", dist.num, label);
				ImGui::EndGroup();
			};
			compare("samples", res.samples, base.samples);
			compare("frames", res.frames, base.frames);
			ImGui::TextColored(less(n, res.losses, base.losses, 0), "%d losses, %d detections", res.losses, res.detections);
			ImGui::TextUnformatted("Baseline:");
			compare("samples", base.samples, res.samples);
			compare("frames", base.frames, res.frames);
			ImGui::TextColored(less(n, base.losses, res.losses, 0), "%d losses, %d detections", base.losses, base.detections);
		};
		if (ImGui::BeginTable("Sections", 4, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoClip | ImGuiTableFlags_PadOuterX))
		{
			ImGui::TableSetupColumn("Start");
			ImGui::TableSetupColumn("End");
			ImGui::TableSetupColumn("Results", ImGuiTableColumnFlags_WidthStretch, 4);
			ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFrameHeight());
			ImGui::TableHeadersRow();

			for (int i = 0; i < frameRanges.size(); i++)
			{
				auto &range = frameRanges[i];
				ImGui::PushID(i);
				ImGui::AlignTextToFramePadding();
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::Text("%d", range.begin);
				ImGui::TableNextColumn();
				ImGui::Text("%d", range.end);
				ImGui::TableNextColumn();
				if (range.dirty)
					ImGui::TextUnformatted("Waiting...");
				else if (!range.updated)
					ImGui::TextUnformatted("Updating...");
				else
				{
					// Update total errors across all ranges
					results.frames.merge(range.results.frames);
					results.samples.merge(range.results.samples);
					baseline.frames.merge(range.baseline.frames);
					baseline.samples.merge(range.baseline.samples);
					results.losses += range.results.losses;
					results.detections += range.results.detections;
					baseline.losses += range.baseline.losses;
					baseline.detections += range.baseline.detections;
					// Evaluate result for this range
					bool n = range.baseline.samples.num == 0; // No baseline
					ImGui::BeginGroup();
					ImGui::TextColored(less(n, range.results.samples.avg, range.baseline.samples.avg, 0.01*PixelSize), "%.2fpx", range.results.samples.avg*PixelFactor);
					ImGui::SameLine();
					ImGui::TextColored(more(n, range.results.samples.num, range.baseline.samples.num, 0.01*PixelSize), "%d samples", range.results.samples.num);
					ImGui::SameLine();
					ImGui::TextColored(more(n, range.results.frames.num, range.baseline.frames.num, 0), "%d frames", range.results.frames.num);
					ImGui::SameLine();
					ImGui::TextColored(less(n, range.results.losses, range.baseline.losses, 0), "%d losses", range.results.losses);
					ImGui::EndGroup();
					if (ImGui::BeginItemTooltip())
					{
						compare(range.baseline, range.results);
						ImGui::EndTooltip();
					}
				}
				ImGui::TableNextColumn();
				if (CrossButton("Del"))
				{
					frameRanges.erase(std::next(frameRanges.begin(), i));
					i--;
				}
				ImGui::PopID();
			}
			ImGui::EndTable();
		}
		EndSection();

		BeginSection("Tracking Results");

		if (ImGui::Button("Update Baseline", SizeWidthFull()))
		{
			for (auto &range : frameRanges)
				range.baseline = range.results;
		}

		ImGui::PushTextWrapPos();
		compare(baseline, results);
		ImGui::PopTextWrapPos();

		EndSection();

		ImGui::PopID();
	}
	else
	{
		controlThread = nullptr;
		frameRanges.clear();
	}

	ImGui::End();
}