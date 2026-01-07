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

void InterfaceState::UpdateLogging(InterfaceWindow &window)
{
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

	std::shared_lock lock(GetApp().logContentAccess);
	static std::size_t selectedLog = -1, focusedLog = -1;
	GetApp().logEntries.delete_culled();
	auto logs = GetApp().logEntries.getView();

	if (ImGui::BeginChild("scrolling", consoleSize, false, ImGuiWindowFlags_HorizontalScrollbar))
	{
		bool logDirty = false;
		int findItem = -1;

		{ // Update filtered logs
			auto pos = logs.begin();
			bool newFilter = logsFilterPos <= pos.index();
			if (newFilter)
			{ // Have not sorted yet or sorting got reset (e.g. logs culled, or manually reset)
				logsFiltered.clear();
			}
			else
			{ // Continue sorting new items from filterPos
				pos = logs.pos(std::max(logs.beginIndex(), logsFilterPos));
				assert(pos.valid());
			}

			while (pos < logs.end())
			{
				if (pos->level >= LogFilterTable[pos->category])
				{
					if (selectedLog == pos.index())
						findItem = logsFiltered.size();
					logsFiltered.push_back(pos.index());
					logDirty = true;
				}
				pos++;
			}
			logsFilterPos = pos.index();

			// If not visible anymore, clear selection to prevent unintended behaviour
			if (newFilter && findItem < 0)
				selectedLog = -1;

			// If we have to jump to a selected log, stop sticking to bottom
			if (findItem >= 0)
				logsStickToNew = false;
		}

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
			clipper.Begin(logsFiltered.size());
			if (findItem >= 0) // Makes sure it's not clipped, so SetScrollHereY works
				clipper.IncludeItemByIndex(findItem);
			while (clipper.Step())
			{
				//for (auto entry = log.pos(startIndex+clipper.DisplayStart); entry.index() < clipper.DisplayEnd; entry++)
				for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++)
				{
					const auto &entry = logs[logsFiltered[i]];
					ImGui::PushID(logsFiltered[i]);

					bool selected = logsFiltered[i] == selectedLog;
					ImGuiSelectableFlags flags = ImGuiSelectableFlags_SpanAvailWidth | ImGuiSelectableFlags_NoPadWithHalfSpacing;
					if (ImGui::Selectable("##SelectLog", selected, flags, ImVec2(0, ImGui::GetTextLineHeight())))
					{
						selected = !selected;
						selectedLog = selected? logsFiltered[i] : -1;
						if (selected && seqEvents)
						{
							std::smatch match;
							std::regex regex("event::([0-9]+)us");
							if (std::regex_search(entry.log, match, regex))
							{
								long timeUS = std::stoi(match[1].str());
								seqJumpToFrame = timeUS;
							}
						}
					}
					bool focused = logsFiltered[i] == focusedLog;
					if (ImGui::IsItemFocused())
					{
						focusVisible = true;
						if (!focused)
						{ // New keyboard focus, jump to it once
							focused = true;
							focusedLog = logsFiltered[i];
							logsStickToNew = i == logsFiltered.size()-1;
							if (logsStickToNew)
							{ // TODO: Somehow cancel focus on log entry to allow keepAtBottom to function
							}
						}
					}

					if (findItem == i)
					{ // Jump to previously selected item after re-sorting
						ImGui::SetScrollHereY();
						logsStickToNew = false;
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
				keepAtBottom = true;
			} */

			ImGui::GetStyle().TouchExtraPadding = extraPad;
			ImGui::PopStyleVar();
		}

		if ((ImGui::GetIO().MouseWheel > 0.0f || findItem >= 0) && ImGui::GetScrollY() < ImGui::GetScrollMaxY())
			logsStickToNew = false;
		else if (ImGui::GetIO().MouseWheel < 0.0f && ImGui::GetScrollY() == ImGui::GetScrollMaxY())
			logsStickToNew = true;
		else if (logsStickToNew && logDirty)
			ImGui::SetScrollHereY(1.0f);
		
		ImGui::PopFont();
	}
	ImGui::EndChild();

	ImGui::SetCursorPos(ImVec2(curPos.x+consoleSize.x, curPos.y));
	if (ImGui::BeginChild("side", sideWinSize, true))
	{
		for (int i = 0; i < LMaxCategory; i++)
		{
			ImGui::PushID(i);
			ImGui::TextUnformatted(LogCategoryDescriptions[i]);
			ImGui::SetItemTooltip("%s", LogCategoryDescriptions[i]);
			int filterLevel = LogFilterTable[i];
			if (ImGui::Combo("", &filterLevel, LogLevelIdentifiers, LMaxLevel))
			{
				LogFilterTable[i] = (LogLevel)filterLevel;
				LogMaxLevelTable[i] = std::min(LDebug, LogFilterTable[i]);
				logsFilterPos = 0;
			}
			ImGui::PopID();
		}
	}
	ImGui::EndChild();


	if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
		ImGui::OpenPopup("Context");
	if (ImGui::BeginPopup("Context"))
	{
		if (ImGui::Selectable("Copy Filtered"))
		{
			int totalSize = 0;
			for (int i = 0; i < logsFiltered.size(); i++)
				totalSize += logs[logsFiltered[i]].log.length() + 1;
			std::string logCpy;
			logCpy.resize(totalSize);
			int index = 0;
			for (int filter : logsFiltered)
			{
				const auto &entry = logs[filter];
				memcpy(logCpy.data()+index, entry.log.data(), entry.log.length());
				index += entry.log.length();
				if (entry.log.back() != '\n')
					logCpy.data()[index++] = '\n';
			}
			logCpy.resize(index);
			ImGui::SetClipboardText(logCpy.c_str());
		}
		if (ImGui::Selectable("Copy All"))
		{
			int totalSize = 0;
			for (const auto entry : logs)
				totalSize += entry.log.length() + 1;
			std::string logCpy;
			logCpy.resize(totalSize);
			int index = 0;
			for (const auto entry : logs)
			{
				memcpy(logCpy.data()+index, entry.log.data(), entry.log.length());
				index += entry.log.length();
				if (entry.log.back() != '\n')
					logCpy.data()[index++] = '\n';
			}
			logCpy.resize(index);
			ImGui::SetClipboardText(logCpy.c_str());
		}
		if (ImGui::Selectable("Clear"))
		{
			GetApp().FlushLog();
			GetApp().logEntries.cull_all();
			// Could reset index as well with cull_clear, but not necessary
			logsFilterPos = 0; // New filtering
		}
		if (ImGui::MenuItem("Jump To Bottom", nullptr, &logsStickToNew))
		{
			if (logsStickToNew)
			{
				selectedLog = -1;
				ImGui::SetScrollY(ImGui::GetScrollMaxY());
				RequestUpdates();
			}
		}
		ImGui::EndPopup();
	}

	ImGui::End();
}

