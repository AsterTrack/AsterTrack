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
#include "imgui_internal.h"

#include <list>

#ifndef IMGUI_ONDEMAND_H
#define IMGUI_ONDEMAND_H

/**
 * Allows for integrating OpenGL elements into the UI with ImDrawCallback
 * Allows for rendering parts of the screen faster than the UI is updated
 *
 * Usage for just integrating OpenGL elements:
 * - Call OnDemandNewFrame(); after ImGui::NewFrame();
 * - Use AddOnDemand**** functions to register callbacks for custom rendering
 * - Call ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); normally
 * 
 * Usage to render OnDemandItems partially and independent from UI update:
 * - Modify backend (e.g. OpenGL3) to allow specifying render rect
 * - Loop over onDemandStack and render OnDemandItem::clip only, e.g.:
 *		for (OnDemandState &onDemandState : onDemandStack)
 *		{
 *			if (onDemandState.renderOwn)
 *				ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData(), onDemandState.clip.Min, onDemandState.clip.Max);
 *		}
 */

struct OnDemandItem
{
	bool renderOwn;
	ImGuiViewport *viewport;
	ImRect bb, clip;
	ImVec2 renderSize, renderScale;
	ImFont *font = NULL;
	float fontSize;
	void *userData = NULL;
};

extern std::list<OnDemandItem> onDemandStack;

void OnDemandNewFrame();

/**
 * Add a bounding box to the OnDemand stack
 */
OnDemandItem &MarkOnDemandArea(const ImRect &bb);

/**
 * Setup OpenGL render area for rendering specified OnDemandItem
 * Overwrites glViewport and glScissors
 */
ImVec2 SetOnDemandRenderArea(const OnDemandItem &state, const ImVec4 &ClipRect);

/**
 * Register a bounding box for custom rendering in callback
 * Use SetOnDemandRenderArea in callback to set up for OpenGL rendering
 */
OnDemandItem *AddOnDemandRender(ImRect bb, ImDrawCallback RenderCallback, void *userData = nullptr, bool resetRenderState = true);

/**
 * Register an icon of size 'icon' for custom rendering in callback 
 * Icon is centered in area of size 'size' plus FramePadding
 * Use SetOnDemandRenderArea in callback to set up for OpenGL rendering
 */
OnDemandItem *AddOnDemandIcon(const char *idLabel, const ImVec2 &size, const ImVec2 &icon, ImDrawCallback RenderCallback, void *userData = nullptr);

/**
 * Register text to be rendered on-demand with maxText specifying the size to reserve
 * Use RenderOnDemandText in callback to render the text
 */
OnDemandItem *AddOnDemandText(const char* maxText, ImDrawCallback RenderCallback);

/**
 * Uses OpenGL3 to immediately render the specifying text.
 * Assumes OpenGL state to be already set up by the OpenGL backend, and it is expected to be called from a ImDrawCallback only.
 */
void RenderOnDemandText(const OnDemandItem &state, const char* fmt, ...);

/**
 * Uses OpenGL3 to immediately render the specifying text.
 * Assumes OpenGL state to be already set up by the OpenGL backend, and it is expected to be called from a ImDrawCallback only.
 */
void RenderOnDemandTextV(const OnDemandItem &state, const char* fmt, va_list args);

#endif // IMGUI_ONDEMAND_H