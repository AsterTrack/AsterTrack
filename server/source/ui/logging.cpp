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
#include "app.hpp"
#include "imgui_internal.h" // ImLerp

#include <regex>

void InterfaceState::MaintainFilteredLogs()
{
	{ // Keep older filtered logs
		auto logs = GetApp().logEntries.getView();
		if (lastLogView.beginIndex() < logs.beginIndex() && !logsFiltered.empty())
		{ // Culled old logs - keep filtered as backlog
			auto firstUnculled = logsFiltered.end();
			for (auto it = logsFiltered.begin(); it != logsFiltered.end(); it++)
			{
				std::size_t filtered = *it;
				if (filtered >= logs.beginIndex())
				{ // Update existing filtered list
					firstUnculled = it;
					break;
				}
				// Apply additional filter as long as we can't re-filter backlog to delete lower levels
				//if (lastLogView[filtered].level <= LDebug) continue;
				// std::move here would be cool but quite risky
				// in the end, not worth it, since filtered logs are usually a small fraction only
				int newIdx = logsFilteredBacklog.push_back(lastLogView[filtered]);
				// Update selected/focused index to new backlog index
				if (filtered == logSelected) logSelected = -newIdx;
				if (filtered == logFocused) logFocused = -newIdx;
			}
			logsFiltered.eraseBefore(firstUnculled);
		}

		// Adopt new view, unlocking any culled blocks, so they will be deleted on next delete_culled
		lastLogView = logs;

		if (!logKeepBacklog)
		{ // Cull old blocks from backlog, keep 1000 (~16 million entries, should be less than 1200MB raw string + 800MB overhead)
			logsFilteredBacklog.delete_culled();
			logsFilteredBacklog.cull_front(-1000);
		}
	}

	int findItem = -1;
	{ // Add new filtered logs
		auto pos = lastLogView.begin();

		bool newFilter = logsFilterPos <= pos.index();
		if (newFilter)
		{ // Have not sorted yet or sorting got reset (e.g. logs culled, or manually reset)
			logsFiltered.clear();
		}
		else
		{ // Continue sorting new items from filterPos
			pos = lastLogView.pos(std::max(lastLogView.beginIndex(), logsFilterPos));
			assert(pos.valid());
		}

		while (pos < lastLogView.end())
		{
			if (pos->level >= LogFilterTable[pos->category])
			{
				if (logSelected == pos.index())
					logJumpTo = logsFilteredBacklog.getView().size() + logsFiltered.size();
				logsFiltered.push_back(pos.index());
			}
			pos++;
		}
		logsFilterPos = pos.index();

		// If not visible anymore, clear selection to prevent unintended behaviour
		if (newFilter && logJumpTo < 0)
			logSelected = -1;

		// If we have to jump to a selected log, stop sticking to new logs
		if (logJumpTo >= 0)
			logsStickToNew = false;
	}
}

void InterfaceState::UpdateLogging(InterfaceWindow &window)
{
	// Keep filtered logs up-to-date at all times
	// Otherwise stuttering ensues, and old filtered logs will be missing
	MaintainFilteredLogs();

	if (!window.open)
		return;
	if (!ImGui::Begin(window.title.c_str(), &window.open))
	{
		ImGui::End();
		return;
	}
	ImVec2 winSize = ImGui::GetContentRegionAvail();
	ImVec2 curPos = ImGui::GetCursorPos();

	ImVec2 consoleSize(winSize.x-160, winSize.y);
	ImVec2 sideWinSize(160, winSize.y);

	auto logs = lastLogView;
	auto backlog = logsFilteredBacklog.getView();

	if (ImGui::BeginChild("scrolling", consoleSize, false, ImGuiWindowFlags_HorizontalScrollbar))
	{
		ImGui::PushFont(fonts.imgui, fonts.imgui->LegacySize);

		float catWidth = ImGui::CalcTextSize("WWWW").x + ImGui::GetStyle().ItemSpacing.x;
		float levelWidth = ImGui::CalcTextSize("WWWWW").x + ImGui::GetStyle().ItemSpacing.x;
		float startLevel = ImGui::GetCursorPosX() + catWidth;
		float startLog = startLevel + levelWidth;
		bool focusVisible = false;

		{ // Base derived text colors on base color - not perfect, but better than nothing
			ImVec4 baseCol = ImGui::GetStyleColorVec4(ImGuiCol_Text);
			ImVec4 fadeColor(0.6f, 0.6f, 0.6f, 1.0f);
			ImVec4 errorColor(1.0f, 0.3f, 0.3f, 1.0f);
			ImVec4 warnColor(0.85f, 0.5f, 0.25f, 1.0f);
			ImVec4 outputCol(1.0f, 1.0f, 0.35f, 1.0f);
			LogLevelHexColors[LTrace]  = ImColor(ImLerp(baseCol, fadeColor, 0.8f));
			LogLevelHexColors[LDebug]  = ImColor(ImLerp(baseCol, fadeColor, 0.4f));
			LogLevelHexColors[LDarn]   = ImColor(ImLerp(baseCol, warnColor, 0.6f));
			LogLevelHexColors[LInfo]   = ImColor(ImLerp(baseCol, fadeColor, 0.0f));
			LogLevelHexColors[LWarn]   = ImColor(ImLerp(baseCol, warnColor, 0.8f));
			LogLevelHexColors[LError]  = ImColor(ImLerp(baseCol, errorColor, 0.9f));
			LogLevelHexColors[LOutput] = ImColor(ImLerp(baseCol, outputCol, 0.75f));
		}

		{ // Has some glitching sometimes, scrollbar shows you can go down further, but it refuses, resulting in visual glitching
			ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0, ImGui::GetStyle().ItemSpacing.y));
			// Has no StyleVar ID
			ImVec2 extraPad = ImGui::GetStyle().TouchExtraPadding;
			ImGui::GetStyle().TouchExtraPadding = ImVec2(0,0);
			
			ImGuiListClipper clipper;
			clipper.Begin(backlog.size() + logsFiltered.size());
			if (logJumpTo >= 0)
			{ // Makes sure it's not clipped, so SetScrollHereY works
				// This should be set to the selected log index after filtering changed
				clipper.IncludeItemByIndex(logJumpTo);
			}
			while (clipper.Step())
			{
				assert(clipper.DisplayStart < logsFiltered.size()+backlog.size());
				assert(clipper.DisplayEnd <= logsFiltered.size()+backlog.size());
				auto filteredIt = clipper.DisplayStart > backlog.size()? logsFiltered.pos(clipper.DisplayStart-backlog.size()) : logsFiltered.begin();
				for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++)
				{
					int64_t logRefIndex;
					const LogEntry *entryPtr;
					if (i < backlog.size())
					{
						logRefIndex = -i;
						entryPtr = &backlog[backlog.beginIndex()+i];
					}
					else
					{ // NOTE: logsFiltered is a BlockedVector, and after eraseBefore indices may not be continuous, hence the need to use an iterator
						assert(filteredIt != logsFiltered.end());
						logRefIndex = *(filteredIt++);
						entryPtr = &logs[logRefIndex];
					}
					const LogEntry &entry = *entryPtr;
					ImGui::PushID(logRefIndex);

					bool selected = logRefIndex == logSelected;
					ImGuiSelectableFlags flags = ImGuiSelectableFlags_SpanAvailWidth | ImGuiSelectableFlags_NoPadWithHalfSpacing;
					if (ImGui::Selectable("##SelectLog", selected, flags, ImVec2(0, ImGui::GetTextLineHeight())))
					{
						selected = !selected;
						logSelected = selected? logRefIndex : -1;
						if (selected && seqEvents)
						{
							std::smatch match;
							std::regex regex("event::([0-9]+)us");
							if (std::regex_search(entry.log, match, regex))
							{
								int64_t timeUS = std::stoll(match[1].str());
								seqJumpToPos = timeUS;
							}
						}
					}

					bool focused = logRefIndex == logFocused;
					if (ImGui::IsItemFocused())
					{
						focusVisible = true;
						if (!focused)
						{ // New keyboard focus, jump to it once
							focused = true;
							logFocused = logRefIndex;
							logsStickToNew = i == logsFiltered.size()-1;
							if (logsStickToNew)
							{ // TODO: Somehow cancel focus on log entry to allow keepAtBottom to function
							}
						}
					}

					if (logJumpTo == i)
					{ // Jump to previously selected item after re-sorting
						ImGui::SetScrollHereY();
						logsStickToNew = false;
						logJumpTo = -1;
						//assert(selectedLog == filteredLogs[i]);
					}

					ImGui::SameLine(0);
					ImGui::TextUnformatted(LogCategoryIdentifiers[entry.category], LogCategoryIdentifiers[entry.category]+4);
					ImGui::SetItemTooltip("%s", LogCategoryDescriptions[entry.category]);

					ImGui::PushStyleColor(ImGuiCol_Text, LogLevelHexColors[entry.level]);

					ImGui::SameLine(startLevel);
					ImGui::TextUnformatted(LogLevelIdentifiers[entry.level], LogLevelIdentifiers[entry.level]+5);

					ImGui::SameLine(startLog);
					ImGui::TextUnformatted(entry.log.c_str(), entry.log.c_str()+entry.log.size());

					ImGui::PopStyleColor();

					ImGui::PopID();
				}
			}
			clipper.End();

			/* if (focusedLog >= 0 && !focusVisible)
			{ // Lost focus, e.g. by leaving this level
				// TODO: Minor glitch in keyboard interaction with log, keepAtBottom would always be true here
				logsStickToNew = true;
			} */

			ImGui::GetStyle().TouchExtraPadding = extraPad;
			ImGui::PopStyleVar();
		}

		if (ImGui::GetIO().MouseWheel > 0.0f && ImGui::GetScrollY() < ImGui::GetScrollMaxY())
			logsStickToNew = false;
		else if (ImGui::GetIO().MouseWheel < 0.0f && ImGui::GetScrollY() == ImGui::GetScrollMaxY())
			logsStickToNew = true;
		else if (logsStickToNew)
			ImGui::SetScrollHereY(1.0f);

		ImGui::PopFont();
	}
	ImGui::EndChild();

	ImGui::SetCursorPos(ImVec2(curPos.x+consoleSize.x, curPos.y));
	if (ImGui::BeginChild("side", sideWinSize, true))
	{
		ImGui::Checkbox("Keep Backlog", &logKeepBacklog);
		ImGui::SetItemTooltip("Backlog keep just the filtered logs in memory for longer.\nThis options prevents those from being culled entirely.");
		for (int i = 0; i < LMaxCategory; i++)
		{
			ImGui::PushID(i);
			ImGui::TextUnformatted(LogCategoryDescriptions[i]);
			ImGui::SetItemTooltip("%s", LogCategoryDescriptions[i]);
			int filterLevel = LogFilterTable[i];
			if (ImGui::Combo("", &filterLevel, LogLevelIdentifiers, LMaxLevel))
			{ // This cannot be used to set it lower than LOG_MAX_LEVEL_DEFAULT or the LOG_MAX_LEVEL of a file
				// But we can't use that fact that because we don't know LOG_MAX_LEVEL
				// This also does not reset LogMaxLevelTable to its default as set in app.cpp
				// So once set lower, it stays at max(LOG_MAX_LEVEL_DEFAULT, lowestSet)
				// Which can cause performance problems due to excessive logs burdening filtering
				// TODO: Update log filtering to be more performant (2/2)
				LogFilterTable[i] = (LogLevel)filterLevel;
				LogMaxLevelTable[i] = std::min(LogMaxLevelTable[i], LogFilterTable[i]);
				logsFilterPos = 0;
				// TODO: If this increases log level, may want to filter backlog to delete lower log levels permanently
			}
			ImGui::PopID();
		}
	}
	ImGui::EndChild();


	if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
		ImGui::OpenPopup("Context");
	if (ImGui::BeginPopup("Context"))
	{
		std::string logCpy;
		std::size_t index = 0;
		auto appendLog = [&](const LogEntry &entry)
		{
			memcpy(logCpy.data()+index, entry.log.data(), entry.log.length());
			index += entry.log.length();
			if (entry.log.back() != '\n')
				logCpy.data()[index++] = '\n';
		};
		if (ImGui::Selectable("Copy Filtered"))
		{
			std::size_t totalSize = 0;
			for (const auto &entry : backlog)
				totalSize += entry.log.length() + 1;
			for (std::size_t filtered : logsFiltered)
				totalSize += logs[filtered].log.length() + 1;
			logCpy.resize(totalSize);
			for (const auto &entry : backlog)
				appendLog(entry);
			for (std::size_t filtered : logsFiltered)
				appendLog(logs[filtered]);
			logCpy.resize(index);
			ImGui::SetClipboardText(logCpy.c_str());
		}
		if (ImGui::Selectable("Copy All"))
		{
			std::size_t totalSize = 0;
			for (const auto &entry : backlog)
				totalSize += entry.log.length() + 1;
			for (const auto &entry : logs)
				totalSize += entry.log.length() + 1;
			logCpy.resize(totalSize);
			for (const auto &entry : backlog)
				appendLog(entry);
			for (const auto &entry : logs)
				appendLog(entry);
			logCpy.resize(index);
			ImGui::SetClipboardText(logCpy.c_str());
		}
		if (ImGui::Selectable("Clear"))
		{
			GetApp().FlushLog();
			GetApp().logEntries.cull_all();
			logsFilteredBacklog.cull_all();
			logsFiltered.clear();
			logsFilterPos = 0;
			lastLogView = GetApp().logEntries.getView();
		}
		if (ImGui::MenuItem("Jump To Bottom", nullptr, &logsStickToNew))
		{
			if (logsStickToNew)
			{
				logSelected = -1;
				ImGui::SetScrollY(ImGui::GetScrollMaxY());
				RequestUpdates();
			}
		}
		ImGui::EndPopup();
	}

	ImGui::End();
}
