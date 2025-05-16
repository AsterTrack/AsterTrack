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

#include "imgui.h"
#include "imgui/imgui_internal.h"

#ifndef IMGUI_CUSTOM_H
#define IMGUI_CUSTOM_H

ImU32 tintColor(ImU32 base, ImU32 tint);

bool CheckButton(const char* str_id);
bool CrossButton(const char* str_id);
bool RetryButton(const char* str_id);
bool CircleButton(const char* str_id, bool filled = true);

bool CircularButton(const char* str_id, float size, ImVec4 color = ImVec4(1,1,1,1), ImGuiButtonFlags flags = ImGuiButtonFlags_None);

bool CheckboxInput(const char *label, bool *value);

bool BooleanProperty(const char *label, bool *value, const bool *compare);

template<typename Scalar>
bool ScalarInputN(const char *label, const char *unit, Scalar *value, Scalar *value2, const Scalar *compare, Scalar min, Scalar max, Scalar step = 1, Scalar editFactor = 1, const char *fmt = nullptr);

template<typename Scalar>
bool ScalarInput2(const char *label, const char *unit, Scalar *value, Scalar *value2, Scalar min, Scalar max, Scalar step = 1, Scalar editFactor = 1, const char *fmt = nullptr)
{ return ScalarInputN(label, unit, value, value2, (const Scalar*)nullptr, min, max, step, editFactor, fmt); }

template<typename Scalar>
bool ScalarInput(const char *label, const char *unit, Scalar *value, Scalar min, Scalar max, Scalar step = 1, Scalar editFactor = 1, const char *fmt = nullptr)
{ return ScalarInputN(label, unit, value, (Scalar*)nullptr, (const Scalar*)nullptr, min, max, step, editFactor, fmt); }

template<typename Scalar>
bool ScalarProperty(const char *label, const char *unit, Scalar *value, const Scalar *compare, Scalar min, Scalar max, Scalar step = 1, Scalar editFactor = 1, const char *fmt = nullptr)
{ return ScalarInputN(label, unit, value, (Scalar*)nullptr, compare, min, max, step, editFactor, fmt); }

template<typename Scalar>
bool SliderInputN(const char *label, Scalar *value, Scalar *value2, Scalar min, Scalar max, Scalar editFactor = 1, const char *fmt = nullptr);

template<typename Scalar>
inline bool SliderInput(const char *label, Scalar *value, Scalar min, Scalar max, Scalar editFactor = 1, const char *fmt = nullptr)
{
	return SliderInputN<Scalar>(label, value, (Scalar*)nullptr, min, max, editFactor, fmt);
}

bool BeginIconDropdown(const char *id, ImTextureID iconTex, ImVec2 iconSize, ImGuiComboFlags flags);

/**
 * Adds an interactable background item that other items can overlap with
 * Returns whether the interaction surface itself is pressed
 * Also outputs hovered and hold states
 */
static inline bool InteractionSurface(const char *idLabel, ImRect rect, bool &hovered, bool &held, ImGuiButtonFlags flags = ImGuiButtonFlags_PressedOnClick)
{
	ImGui::SetNextItemAllowOverlap();
	ImGuiID id = ImGui::GetID(idLabel);
	ImGui::ItemAdd(rect, id);
    return ImGui::ButtonBehavior(rect, id, &hovered, &held, flags);
}

static inline void BeginSection(const char *label)
{
	/* ImGui::BeginGroup();
	ImGui::Text(label);
	ImGui::Separator();
	ImGui::EndGroup(); */

	ImGui::SeparatorText(label);
	ImGui::PushID(label);

}

static inline void EndSection()
{
	ImGui::Dummy(ImVec2(0, 4));
	ImGui::PopID();
}

static inline void BeginViewToolbar()
{
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 4.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(3,3));
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImGui::GetStyle().WindowPadding);
	ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.8f);
	ImGui::AlignTextToFramePadding();
}

static inline void EndViewToolbar()
{
	ImGui::PopStyleVar(4);
}

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

/*
 * Subdivided sizes in line space
 */

static inline ImVec2 SizeWidthFull()
{
	return ImVec2(LineWidth(), ImGui::GetFrameHeight());
}
static inline ImVec2 SizeWidthDiv2()
{
	return ImVec2((LineWidth() - ImGui::GetStyle().ItemSpacing.x*1)/2, ImGui::GetFrameHeight());
}
static inline ImVec2 SizeWidthDiv3()
{
	return ImVec2((LineWidth() - ImGui::GetStyle().ItemSpacing.x*2)/3, ImGui::GetFrameHeight());
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
static inline ImVec2 SizeWidthDiv4()
{
	return ImVec2((LineWidth() - ImGui::GetStyle().ItemSpacing.x*3)/4, ImGui::GetFrameHeight());
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

#endif // IMGUI_CUSTOM_H