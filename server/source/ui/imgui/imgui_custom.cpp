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

#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif

#include "imgui_custom.hpp"

#include <string>
#include <cstdio>
#include <cmath>
#include <algorithm>

ImU32 tintColor(ImU32 base, ImU32 tint)
{
	ImVec4 col1 = ImGui::ColorConvertU32ToFloat4(base);
	ImVec4 col2 = ImGui::ColorConvertU32ToFloat4(tint);
	col1 = col1 * col2;
	return ImGui::ColorConvertFloat4ToU32(col1);
}
ImU32 tintColor(ImVec4 base, ImVec4 tint)
{
	base = base * tint;
	return ImGui::ColorConvertFloat4ToU32(base);
}
ImU32 tintStyle(ImGuiCol base, ImVec4 tint)
{
	return ImGui::ColorConvertFloat4ToU32(ImGui::GetStyleColorVec4(base) * tint);
}

bool IconButtonEx(const char* str_id, ImVec2 size, ImGuiButtonFlags flags, ImRect &bb)
{
	ImGuiContext& g = *GImGui;
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	if (window->SkipItems)
		return false;

	const ImGuiID id = window->GetID(str_id);
	bb = ImRect(window->DC.CursorPos, window->DC.CursorPos + size);
	const float default_size = ImGui::GetFrameHeight();
	ImGui::ItemSize(size, (size.y >= default_size) ? g.Style.FramePadding.y : -1.0f);
	if (!ImGui::ItemAdd(bb, id))
		return false;

	bool hovered, held;
	bool pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held, flags);

	// Render
	const ImU32 bg_col = ImGui::GetColorU32((held && hovered) ? ImGuiCol_ButtonActive : hovered ? ImGuiCol_ButtonHovered : ImGuiCol_Button);
	ImGui::RenderNavCursor(bb, id);
	ImGui::RenderFrame(bb.Min, bb.Max, bg_col, true, g.Style.FrameRounding);

	IMGUI_TEST_ENGINE_ITEM_INFO(id, str_id, g.LastItemData.StatusFlags);
	return pressed;
}

bool CheckButton(const char* str_id)
{
	float sz = ImGui::GetFrameHeight();
	ImRect bb;
	bool pressed = IconButtonEx(str_id, ImVec2(sz, sz), ImGuiButtonFlags_None, bb);

	const ImU32 text_col = ImGui::GetColorU32(ImGuiCol_Text);
	float cmSize = ImGui::GetFontSize()*0.8f;
	ImVec2 margin = ImVec2(ImMax(0.0f, (bb.GetSize().x - cmSize) * 0.5f), ImMax(0.0f, (bb.GetSize().y - cmSize) * 0.5f));
	ImGui::RenderCheckMark(ImGui::GetWindowDrawList(), bb.Min + margin, text_col, cmSize);

	return pressed;
}

bool CrossButton(const char* str_id)
{
	float sz = ImGui::GetFrameHeight();
	ImRect bb;
	bool pressed = IconButtonEx(str_id, ImVec2(sz, sz), ImGuiButtonFlags_None, bb);

	const ImU32 text_col = ImGui::GetColorU32(ImGuiCol_Text);
	float cExt = ImGui::GetFontSize() * 0.5f * 0.7071f - 1.0f;
	ImVec2 center = bb.GetCenter();
	ImGui::GetWindowDrawList()->AddLine(center + ImVec2(+cExt, +cExt), center + ImVec2(-cExt, -cExt), text_col, 2.0f);
	ImGui::GetWindowDrawList()->AddLine(center + ImVec2(+cExt, -cExt), center + ImVec2(-cExt, +cExt), text_col, 2.0f);

	return pressed;
}

bool RetryButton(const char* str_id)
{
	float sz = ImGui::GetFrameHeight();
	ImRect bb;
	bool pressed = IconButtonEx(str_id, ImVec2(sz, sz), ImGuiButtonFlags_None, bb);

	const ImU32 text_col = ImGui::GetColorU32(ImGuiCol_Text);
	float cExt = ImGui::GetFontSize() * 0.5f * 0.7071f - 1.0f;
	ImVec2 center = bb.GetCenter();
	ImGui::GetWindowDrawList()->AddBezierQuadratic(center + ImVec2(0, -cExt), center + ImVec2(+cExt, -cExt), center + ImVec2(+cExt, 0), text_col, 2.0f, 5);
	ImGui::GetWindowDrawList()->AddBezierQuadratic(center + ImVec2(+cExt, 0), center + ImVec2(+cExt, +cExt), center + ImVec2(0, +cExt), text_col, 2.0f, 5);
	ImGui::GetWindowDrawList()->AddBezierQuadratic(center + ImVec2(0, +cExt), center + ImVec2(-cExt, +cExt), center + ImVec2(-cExt, 0), text_col, 2.0f, 5);
	ImGui::GetWindowDrawList()->AddTriangleFilled(center + ImVec2(-cExt-4, 0), center + ImVec2(-cExt, -4), center + ImVec2(-cExt+4, 0), text_col);

	return pressed;
}

bool CircleButton(const char* str_id, bool filled)
{
	float sz = ImGui::GetFrameHeight();
	float radius = ImGui::GetFontSize() * 0.5f * 0.7071f;

	ImRect bb;
	bool pressed = IconButtonEx(str_id, ImVec2(sz, sz), ImGuiButtonFlags_None, bb);

	const ImU32 text_col = ImGui::GetColorU32(ImGuiCol_Text);
	if (filled)
		ImGui::GetWindowDrawList()->AddCircleFilled(bb.GetCenter(), radius, text_col, 20);
	else
		ImGui::GetWindowDrawList()->AddCircle(bb.GetCenter(), radius, text_col, 20, 2.0f);

	return pressed;
}

bool CircularButton(const char* str_id, float size, ImVec4 color, ImGuiButtonFlags flags)
{
	ImGuiContext& g = *GImGui;
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	if (window->SkipItems)
		return false;

	const ImGuiID id = window->GetID(str_id);
	ImVec2 sz(size, size);
	ImRect bb(window->DC.CursorPos, window->DC.CursorPos + sz);
	ImGui::ItemSize(sz);
	if (!ImGui::ItemAdd(bb, id))
		return false;

	bool hovered, held;
	bool pressed = ImGui::ButtonBehavior(bb, id, &hovered, &held, flags);
	ImGui::RenderNavCursor(bb, id);

	// Render
	ImU32 col = tintStyle((held && hovered) ? ImGuiCol_ButtonActive : hovered ? ImGuiCol_ButtonHovered : ImGuiCol_Button, color);
	window->DrawList->AddCircleFilled(bb.GetCenter(), size/2, col, 20);

	IMGUI_TEST_ENGINE_ITEM_INFO(id, str_id, g.LastItemData.StatusFlags);
	return pressed;
}

bool BooleanProperty(const char *label, bool *value, const bool *compare)
{
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	if (window->SkipItems)
		return false;
	ImGui::BeginGroup();
	ImGui::PushID(label);
	ImGui::AlignTextToFramePadding(); // Align text vertically in line with controls
	ImGui::TextUnformatted(label);
	SameLineTrailing(ImGui::GetFrameHeight());
	bool modified = compare && *value != *compare;
	if (modified) ImGui::PushStyleColor(ImGuiCol_FrameBg, tintStyle(ImGuiCol_FrameBg, ImVec4(1,0,0,1)));
	bool updated = ImGui::Checkbox("##Check", value);
	if (modified) ImGui::PopStyleColor();
	if (modified && ImGui::BeginPopupContextItem("C"))
	{
		if (ImGui::Selectable("Reset to Default"))
		{
			*value = *compare;
			updated = true;
		}
		ImGui::EndPopup();
	}
	ImGui::PopID();
	ImGui::EndGroup();
	return updated;
}

bool CheckboxInput(const char *label, bool *value)
{
	return BooleanProperty(label, value, nullptr);
}

template<typename Scalar>
static constexpr ImGuiDataType getDataType()
{
	if constexpr (std::is_same_v<Scalar, float>)
		return ImGuiDataType_Float;
	else if constexpr (std::is_same_v<Scalar, double>)
		return ImGuiDataType_Double;
	else if constexpr (std::is_same_v<Scalar, int>)
		return ImGuiDataType_S32;
	else if constexpr (std::is_same_v<Scalar, unsigned int>)
		return ImGuiDataType_U32;
	else if constexpr (std::is_same_v<Scalar, long>)
		return ImGuiDataType_S64;
	else if constexpr (std::is_same_v<Scalar, unsigned long>)
		return ImGuiDataType_U64;
	else
	 	static_assert(std::is_same_v<Scalar, void>);
}

template<typename Scalar>
static bool isSame(const Scalar *a, const Scalar *b)
{
	if constexpr (std::numeric_limits<Scalar>::is_integer)
		return *a == *b;
	else
		return std::abs(*a - *b) <= std::numeric_limits<Scalar>::epsilon() || (std::isnan(*a) && std::isnan(*b));
}

template<typename Scalar>
bool ScalarInputN(const char *label, const char *unit, Scalar *value, Scalar *value2, const Scalar *compare, Scalar min, Scalar max, Scalar step, Scalar editFactor, const char *fmt)
{
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	if (window->SkipItems)
		return false;
	auto &style = ImGui::GetStyle();
	ImGui::PushID(label);
	ImGui::BeginGroup();
	ImGui::AlignTextToFramePadding();
	ImGui::TextEx(label, ImGui::FindRenderedTextEnd(label));
	SameLinePos(SizeWidthDiv3().x+style.ItemSpacing.x);
	ImGui::SetNextItemWidth(SizeWidthDiv3_2().x);
	Scalar temp[2] = { *value * editFactor, value2? (*value2 * editFactor) : 0 };
	bool modified = compare && (!isSame(value, compare) || (value2 && !isSame(value2, compare)));
	if (modified) ImGui::PushStyleColor(ImGuiCol_FrameBg, tintStyle(ImGuiCol_FrameBg, ImVec4(1,0,0,1)));
	if (value2)
		ImGui::InputScalarN("I", getDataType<Scalar>(), temp, 2, NULL, NULL, fmt);
	else
		ImGui::InputScalarN("I", getDataType<Scalar>(), temp, 1, &step, NULL, fmt);
	if (modified) ImGui::PopStyleColor();
	if (unit && *unit != 0)
	{ // Insert unit as an overlay
		float unitSz = ImGui::CalcTextSize(unit).x;
		if (value2 == nullptr && step > 0)
		{ // Only for single values WITH +/- buttons
			float posRight = ImGui::GetFrameHeight()*2 + style.ItemInnerSpacing.x*3;
			SameLineTrailing(posRight + unitSz), 0;
			ImGui::Text("%s", unit);
		}
		else
		{ // Without +/- buttons (single or double values)
			SameLineTrailing(style.ItemInnerSpacing.x + unitSz), 0;
			ImGui::Text("%s", unit);
			if (value2)
			{
				SameLineTrailing(style.ItemInnerSpacing.x*2 + SizeWidthDiv3().x + unitSz), 0;
				ImGui::Text("%s", unit);
			}
		}
	}
	ImGui::EndGroup();
	bool update = false;
	if (ImGui::IsItemDeactivatedAfterEdit() || ImGui::IsItemEdited())
	{
		Scalar val = std::clamp(temp[0], min, max) / editFactor;
		update |= val != *value;
		*value = val;
		if (value2)
		{
			val = std::clamp(temp[1], min, max) / editFactor;
			update |= val != *value2;
			*value2 = val;
		}
	}
	if (modified && ImGui::BeginPopupContextItem("C"))
	{
		if (ImGui::Selectable("Reset to Default"))
		{
			*value = *compare;
			if (value2) *value2 = *compare;
			update = true;
		}
		ImGui::EndPopup();
	}
	ImGui::PopID();
	return update;
}

template<typename Scalar>
bool SliderInputN(const char *label, Scalar *value, Scalar *value2, Scalar min, Scalar max, Scalar editFactor, const char *fmt)
{
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	if (window->SkipItems)
		return false;
	auto &style = ImGui::GetStyle();
	ImGui::PushID(label);
	ImGui::BeginGroup();
	ImGui::AlignTextToFramePadding();
	ImGui::TextEx(label, ImGui::FindRenderedTextEnd(label));
	SameLinePos(SizeWidthDiv3().x+style.ItemSpacing.x);
	ImGui::SetNextItemWidth(SizeWidthDiv3_2().x);
	Scalar temp[2] = { *value * editFactor, value2? (*value2 * editFactor) : 0 };
	ImGui::SliderScalarN("S", getDataType<Scalar>(), temp, value2? 2 : 1, &min, &max, NULL);
	ImGui::EndGroup();
	bool update = false;
	if (ImGui::IsItemDeactivatedAfterEdit() || ImGui::IsItemEdited())
	{
		Scalar val = temp[0] / editFactor;
		update |= val != *value;
		*value = val;
		if (value2)
		{
			val = temp[1] / editFactor;
			update |= val != *value2;
			*value2 = val;
		}
	}
	ImGui::PopID();
	return update;
}

template bool ScalarInputN(const char *label, const char *unit, int *value, int *value2, const int *compare, int min, int max, int step, int editFactor, const char *fmt);
template bool ScalarInputN(const char *label, const char *unit, float *value, float *value2, const float *compare, float min, float max, float step, float editFactor, const char *fmt);
template bool SliderInputN(const char *label, int *value, int *value2, int min, int max, int editFactor, const char *fmt);
template bool SliderInputN(const char *label, float *value, float *value2, float min, float max, float editFactor, const char *fmt);

bool BeginIconDropdown(const char *id, ImTextureID iconTex, ImVec2 iconSize, ImGuiComboFlags flags)
{
	ImGui::PushID(id);
	const ImGuiID popupID = ImGui::GetCurrentWindowRead()->GetID("ctx");
	if (ImGui::ImageButton("btn", iconTex, iconSize))
		ImGui::OpenPopup(popupID);
	ImGui::PopID();
	ImRect contextBB(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
	bool open = ImGui::BeginComboPopup(popupID, contextBB, flags);
	return open;
}