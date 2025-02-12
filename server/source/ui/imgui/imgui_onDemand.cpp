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
#include "imgui.h"
#endif

#include "imgui_onDemand.hpp"

#include "backends/imgui_impl_opengl3.h"
#include "GL/glew.h"

std::list<OnDemandItem> onDemandStack;

void OnDemandNewFrame()
{
	onDemandStack.clear();
}

OnDemandItem &MarkOnDemandArea(const ImRect &bb)
{
	OnDemandItem state = {};
	auto viewWin = ImGui::GetCurrentWindowRead();
	state.renderOwn = true; // cause a partial render to render this area
	state.bb = bb;
	state.clip = bb;
	state.viewport = ImGui::GetWindowViewport();
	state.renderScale = ImGui::GetIO().DisplayFramebufferScale;
	state.renderSize = state.viewport->Size * state.renderScale;
	onDemandStack.push_back(state);
	return onDemandStack.back();
}

ImVec2 SetOnDemandRenderArea(const OnDemandItem &state, const ImVec4 &ClipRect)
{
	const auto scale = [](ImVec4 rect, const ImVec2 &scale)
	{
		rect.x *= scale.x;
		rect.y *= scale.y;
		rect.z *= scale.x;
		rect.w *= scale.y;
		return rect;
	};
	auto view = scale(state.bb.ToVec4(), state.renderScale);
	auto clip = scale(ClipRect, state.renderScale);
	glViewport(view.x, state.renderSize.y-view.w, view.z-view.x, view.w-view.y);
	glScissor(clip.x, state.renderSize.y-clip.w, clip.z-clip.x, clip.w-clip.y);
	return ImVec2(view.z-view.x, view.w-view.y);
}

OnDemandItem *AddOnDemandRender(ImRect bb, ImDrawCallback RenderCallback, void *userData, bool resetRenderState)
{
	if (ImGui::GetCurrentWindowRead()->SkipItems) return nullptr;
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	window->DrawList->PushClipRect(bb.Min, bb.Max, true);
	ImRect clipped = ImRect(window->DrawList->GetClipRectMin(), window->DrawList->GetClipRectMax());
	if (clipped.GetArea() == 0)
	{
		window->DrawList->PopClipRect();
		return nullptr;
	}
	OnDemandItem &state = MarkOnDemandArea(bb);
	state.clip = clipped;
	state.userData = userData;
	window->DrawList->AddCallback(RenderCallback, &state);
	if (resetRenderState)
		window->DrawList->AddCallback(ImDrawCallback_ResetRenderState, nullptr);
	window->DrawList->PopClipRect();
	return &state;
}

OnDemandItem *AddOnDemandIcon(const char *idLabel, const ImVec2 &size, const ImVec2 &icon, ImDrawCallback RenderCallback, void *userData)
{
	if (ImGui::GetCurrentWindowRead()->SkipItems) return nullptr;
	auto c = ImGui::GetCurrentWindowRead()->DC.CursorPos;
	const ImRect bb(c.x, c.y, c.x+size.x, c.y+size.y);
	ImGui::ItemSize(size);
	ImGui::ItemAdd(bb, ImGui::GetID(idLabel));
	ImRect bbR(bb.GetCenter()-icon/2, bb.GetCenter()+icon/2);
	return AddOnDemandRender(bbR, RenderCallback, userData);
}

OnDemandItem *AddOnDemandText(const char* maxText, ImDrawCallback RenderCallback)
{
	if (ImGui::GetCurrentWindowRead()->SkipItems) return nullptr;
	ImGuiWindow* window = ImGui::GetCurrentWindow();
	const ImVec2 text_pos(window->DC.CursorPos.x, window->DC.CursorPos.y + window->DC.CurrLineTextBaseOffset);
	const ImVec2 text_size = ImGui::CalcTextSize(maxText, NULL, true, 0.0f);
	ImRect bb(text_pos, text_pos + text_size);
	ImGui::ItemSize(text_size);
	ImGui::ItemAdd(bb, ImGui::GetID(maxText));
	OnDemandItem *state = AddOnDemandRender(bb, RenderCallback, nullptr, false);
	if (!state) return nullptr;
	state->font = ImGui::GetFont();
	state->fontSize = ImGui::GetFontSize();
	return state;
}

void RenderOnDemandText(const OnDemandItem &state, const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	RenderOnDemandTextV(state, fmt, args);
	va_end(args);
}
void RenderOnDemandTextV(const OnDemandItem &state, const char* fmt, va_list args)
{
	const char* text, *text_end;
	ImFormatStringToTempBufferV(&text, &text_end, fmt, args);

	static ImDrawList onDemandDrawList(ImGui::GetDrawListSharedData());
	onDemandDrawList._ResetForNewFrame();

	onDemandDrawList.PushClipRect(state.bb.Min, state.bb.Max, false);
	onDemandDrawList.PushTextureID(state.font->ContainerAtlas->TexID);
	onDemandDrawList.AddText(state.font, state.fontSize, state.bb.Min, ImGui::GetColorU32(ImGuiCol_Text), text, text_end);
	onDemandDrawList.PopTextureID();
	onDemandDrawList.PopClipRect();

	ImGui_ImplOpenGL3_RenderDrawList(state.viewport, &onDemandDrawList);
}