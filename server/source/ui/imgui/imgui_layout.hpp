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

#ifndef IMGUI_LAYOUT_H
#define IMGUI_LAYOUT_H

#include "imgui.h"
#include "imgui/imgui_internal.h"

#include <algorithm>


static inline float GetIconWidth(ImVec2 iconSize)
{
	return ImGui::GetStyle().FramePadding.x*2 + iconSize.x;
}

static inline float GetBarWidth(float width, int num)
{
	return width * num + ImGui::GetStyle().ItemSpacing.x * (num-1);
}

/**
 * ImGui has a weird line space
 * CursorPos is not offset at all, meaning it includes WindowPadding, Groups and Indents
 * StartPos is only offset within a group or column, at which point it will include WindowPadding and Indent of the group
 * Layout calculations are easiest in Line Space which should span all useable area
 * So Line Pos should be offset by WindowPadding, Groups, and Indents
 * And Line Width should account for WindowPadding, Scrollbars (and TODO: Column Widths)
 * So these functions are all in line space, with conversion at the top to CursorPos and StartPos
 * They can also directly be used to pass sizes and widths to ImGui
 * These functions adapt LineWidth if a scrollbar is visible so that it doesn't overlap anything
 */

/* Returns indent in line space */
static inline float LineIndent()
{ // Just Indent + GroupOffset, so usually 0
	auto window = ImGui::GetCurrentWindowRead();
	// Initial window->DC.Indent.x from ImGui::Begin
	float initialCursorStart = window->DecoOuterSizeX1 + ImGui::GetStyle().WindowPadding.x - window->Scroll.x;
	return window->DC.Indent.x - initialCursorStart;
}

/* Returns cursor pos for the specified position in line space */
static inline float CursorPosFromLinePos(float pos)
{
	return pos + LineIndent() + ImGui::GetStyle().WindowPadding.x;
}

/* Returns cursor pos that is right-aligned with desired space available, leaving WindowPadding to the right */
static inline float GetRightAlignedCursorPos(float width)
{
	auto window = ImGui::GetCurrentWindowRead();
	return ImGui::GetStyle().WindowPadding.x + window->WorkRect.GetWidth() - width;
}

/* Returns cursor pos used by SameLine as the base - only for use with SameLine! */
static inline float GetStartX()
{ // Get the cursor pos that SameLine uses as a base, from groups and table columns.
	auto window = ImGui::GetCurrentWindowRead();
	return window->DC.GroupOffset.x + window->DC.ColumnsOffset.x;
}

/* Continues current line at specified position in line space */
static inline void SameLinePos(float pos)
{
	ImGui::SameLine(CursorPosFromLinePos(pos) - GetStartX());
}

/* Continues current line with a trailing space of specified width */
static inline void SameLineTrailing(float width)
{
	ImGui::SameLine(GetRightAlignedCursorPos(width) - GetStartX());
}

/* Returns current cursor pos in line space */
static inline float GetLinePosX()
{
	return ImGui::GetCursorPosX() - ImGui::GetStyle().WindowPadding.x - LineIndent();
}

/* Returns useable width of line space */
static inline float LineWidth()
{
	auto window = ImGui::GetCurrentWindowRead();
	return window->WorkRect.GetWidth() - LineIndent();
}

/* Returns remaining width of line space */
static inline float LineWidthRemaining()
{
	auto window = ImGui::GetCurrentWindowRead();
	return window->WorkRect.GetWidth() - (ImGui::GetCursorPosX() - ImGui::GetStyle().WindowPadding.x);
}
template<typename... STRING>
static inline float MinSharedLabelWidth(STRING... labels)
{
	float max = 0.0f;
	for (const auto &label : {labels...})
		max = std::max(max, ImGui::CalcTextSize(label).x);
	return max;
}

static inline float GetWindowActualContentHeight()
{
	auto window = ImGui::GetCurrentWindowRead();
	return window->ContentSize.y;
}

static inline float GetWindowContentRegionHeight()
{
	auto window = ImGui::GetCurrentWindowRead();
	return window->ContentRegionRect.GetHeight();
}

static inline float CalcTableHeight(int rows, float rowHeight = ImGui::GetFrameHeight())
{
	return (rowHeight + ImGui::GetStyle().CellPadding.y*2) * rows;
}

/*
 * Subdivided sizes in line space
 */

static inline ImVec2 SizeFrame()
{
	return ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight());
}
static inline ImVec2 SizeWidthFull()
{
	return ImVec2(LineWidth(), ImGui::GetFrameHeight());
}
static inline ImVec2 SizeWidthDiv2(float minWidth = 0.0f)
{
	return ImVec2(std::max(minWidth, (LineWidth() - ImGui::GetStyle().ItemSpacing.x*1)/2), ImGui::GetFrameHeight());
}
static inline ImVec2 SizeWidthDiv3(float minWidth = 0.0f)
{
	return ImVec2(std::max(minWidth, (LineWidth() - ImGui::GetStyle().ItemSpacing.x*2)/3), ImGui::GetFrameHeight());
}
/**
 * Two parts of a 3-wise split line.
 * It is larger than SizeWidthDiv3 * 2 to allow mixing multiple Div3 sizes
 */
static inline ImVec2 SizeWidthDiv3_2()
{
	ImVec2 size = SizeWidthDiv3();
	size.x = size.x*2 + ImGui::GetStyle().ItemSpacing.x;
	return size;
}
/**
 * A 3-wise split line, halved again to fit two controls in one third
 */
static inline ImVec2 SizeWidthDiv3_Div2()
{
	ImVec2 size = SizeWidthDiv3();
	size.x = (size.x - ImGui::GetStyle().ItemSpacing.x)/2;
	return size;
}
static inline ImVec2 SizeWidthDiv4(float minWidth = 0.0f)
{
	return ImVec2(std::max(minWidth, (LineWidth() - ImGui::GetStyle().ItemSpacing.x*3)/4), ImGui::GetFrameHeight());
}
/**
 * Two parts of a 4-wise split line.
 * It is larger than SizeWidthDiv4 * 2 to allow mixing multiple Div4 sizes
 */
static inline ImVec2 SizeWidthDiv4_2()
{
	ImVec2 size = SizeWidthDiv4();
	size.x = size.x*2 + ImGui::GetStyle().ItemSpacing.x;
	return size;
}
/**
 * Three parts of a 4-wise split line.
 * It is larger than SizeWidthDiv4 * 3 to allow mixing multiple Div4 sizes
 */
static inline ImVec2 SizeWidthDiv4_3()
{
	ImVec2 size = SizeWidthDiv4();
	size.x = size.x*3 + ImGui::GetStyle().ItemSpacing.x*2;
	return size;
}

#endif // IMGUI_LAYOUT_H