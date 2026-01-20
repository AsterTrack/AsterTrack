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


#ifndef IMGUI_CUSTOM_H
#define IMGUI_CUSTOM_H

#include "imgui.h"
#include "imgui/imgui_internal.h"

#include "imgui_layout.hpp"


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

bool BeginInteractiveItemTooltip(const char* text_id = "");
void EndInteractiveItemTooltip();

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

static bool SaveButton(const char* label, const ImVec2 &size, bool marked)
{
	if (marked)
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.25f, 0.25f, 1.00f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.50f, 0.38f, 0.38f, 1.00f));
	}
	bool press = ImGui::Button(label, size);
	if (marked)
	{
		ImGui::PopStyleColor(2);
	}
	return press;
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

static void BeginLabelledGroup(const char* fmt, ...)
{
	ImGui::BeginGroup();

    va_list args;
    va_start(args, fmt);
    ImGui::TextV(fmt, args);
    va_end(args);

	SameLineTrailing(SizeWidthDiv3_2().x);
	ImGui::SetNextItemWidth(SizeWidthDiv3_2().x);
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

static inline bool InlineIconButton(const char *iconLabel)
{
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
	ImGui::PushStyleColor(ImGuiCol_Button, ImGuiCol_FrameBg);
	bool pressed = ImGui::Button(iconLabel);
	ImGui::PopStyleColor();
	ImGui::PopStyleVar();
	return pressed;
}

#endif // IMGUI_CUSTOM_H