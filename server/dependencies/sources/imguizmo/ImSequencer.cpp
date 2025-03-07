// https://github.com/CedricGuillemet/ImGuizmo
// v1.91.3 WIP
//
// The MIT License(MIT)
//
// Copyright(c) 2021 Cedric Guillemet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#include "ImSequencer.hpp"
#include "imgui.h"
#include "imgui_internal.h"

#include <cstdlib>
#include <limits>

namespace ImSequencer
{
#ifndef IMGUI_DEFINE_MATH_OPERATORS
   static ImVec2 operator+(const ImVec2& a, const ImVec2& b) {
      return ImVec2(a.x + b.x, a.y + b.y);
   }
#endif
   static bool SequencerAddDelButton(ImDrawList* draw_list, ImVec2 pos, bool add = true)
   {
      ImGuiIO& io = ImGui::GetIO();
      ImRect btnRect(pos, ImVec2(pos.x + 16, pos.y + 16));
      bool overBtn = btnRect.Contains(io.MousePos);
      bool containedClick = overBtn && btnRect.Contains(io.MouseClickedPos[0]);
      bool clickedBtn = containedClick && io.MouseReleased[0];
      int btnColor = overBtn ? 0xAAEAFFAA : 0x77A3B2AA;
      if (containedClick && io.MouseDownDuration[0] > 0)
         btnRect.Expand(2.0f);

      float midy = pos.y + 16 / 2 - 0.5f;
      float midx = pos.x + 16 / 2 - 0.5f;
      draw_list->AddRect(btnRect.Min, btnRect.Max, btnColor, 4);
      draw_list->AddLine(ImVec2(btnRect.Min.x + 3, midy), ImVec2(btnRect.Max.x - 3, midy), btnColor, 2);
      if (add)
         draw_list->AddLine(ImVec2(midx, btnRect.Min.y + 3), ImVec2(midx, btnRect.Max.y - 3), btnColor, 2);
      return clickedBtn;
   }

#if defined(__clang__) || defined(__GNUC__)
   __attribute__((no_sanitize("float-cast-overflow")))
#endif
   bool Sequencer(SequenceInterface* sequence, int* currentFrame, bool* expanded, int* selectedEntry, int* firstFrame, int sequenceOptions)
   {
      bool ret = false;
      ImGuiIO& io = ImGui::GetIO();
      ImGuiStyle& style = ImGui::GetStyle();
      int cx = (int)(io.MousePos.x);
      int cy = (int)(io.MousePos.y);
      int legendWidth = 200;

      static int movingEntry = -1;
      static int movingPos = -1;
      static int movingPart = -1;
      int delEntry = -1;
      int dupEntry = -1;
      int ItemHeight = 20;

      bool popupOpened = false;
      int sequenceCount = sequence->GetItemCount();
      if (!sequenceCount)
         return false;

      ImGui::BeginGroup();

      ImDrawList* draw_list = ImGui::GetWindowDrawList();
      ImVec2 canvas_pos = ImGui::GetCursorScreenPos();            // ImDrawList API uses screen coordinates!
      ImVec2 canvas_size = ImGui::GetContentRegionAvail();        // Resize canvas to what's available

      int controlHeight = sequenceCount * ItemHeight;
      for (int i = 0; i < sequenceCount; i++)
         controlHeight += int(sequence->GetCustomHeight(i));
      int frameCount = sequence->GetFrameMax() - sequence->GetFrameMin();

      static bool MovingScrollBar = false;
      static bool MovingCurrentFrame = false;

      struct CustomDraw
      {
         int index;
         ImRect customRect;
         ImRect legendRect;
         ImRect clippingRect;
         ImRect legendClippingRect;
      };
      ImVector<CustomDraw> customDraws;
      ImVector<CustomDraw> compactCustomDraws;

      // zoom in/out
      sequence->framePixelWidthTarget = ImMin(sequence->framePixelWidthTarget, 50.0f);
      sequence->framePixelWidth = ImLerp(sequence->framePixelWidth, sequence->framePixelWidthTarget, 0.33f);

      if (expanded && !*expanded)
      {
         ImGui::InvisibleButton("canvas", ImVec2(canvas_size.x, (float)ItemHeight));
         draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + ItemHeight), 0xFF3D3837, 0);
         char tmps[512];
         ImFormatString(tmps, IM_ARRAYSIZE(tmps), sequence->GetCollapseFmt(), frameCount, sequenceCount);
         draw_list->AddText(ImVec2(canvas_pos.x + 26, canvas_pos.y + 2), 0xFFFFFFFF, tmps);

         if (SequencerAddDelButton(draw_list, ImVec2(canvas_pos.x + 2, canvas_pos.y + 2), !*expanded))
            *expanded = !*expanded;
         
         ImGui::EndGroup();
         return ret;
      }
      
      bool hasScrollBar = true;
      /*int framesPixelWidth = int(frameCount * sequence->framePixelWidth);
      if ((framesPixelWidth + legendWidth) >= canvas_size.x)
      {
            hasScrollBar = true;
      }*/

      // Distribute vertical space
      ImVec2 headerSize(canvas_size.x, (float)ItemHeight);
      ImVec2 scrollBarSize(canvas_size.x, style.ScrollbarSize);
      ImVec2 childFrameSize(canvas_size.x, canvas_size.y - headerSize.y - style.ItemSpacing.y - (hasScrollBar ? scrollBarSize.y+style.ItemSpacing.y : 0));

      // Occupy header space
      ImGui::Dummy(headerSize);

      // Occupy child canvas for sequences
      ImVec2 childFramePos = ImGui::GetCursorScreenPos();
      ImGui::PushStyleColor(ImGuiCol_FrameBg, 0);
      ImGui::BeginChild(889, childFrameSize, ImGuiChildFlags_FrameStyle, ImGuiWindowFlags_AlwaysVerticalScrollbar);
      ImVec2 child_canvas_pos = ImGui::GetCursorScreenPos();
      ImVec2 child_canvas_size = ImGui::GetContentRegionAvail();
      ImRect sequenceRect(child_canvas_pos.x + float(legendWidth), childFramePos.y,
         child_canvas_pos.x + child_canvas_size.x, childFramePos.y + childFrameSize.y);

      // Get focus on child canvas
      sequence->focused = ImGui::IsWindowFocused();
      bool canvasHovered = ImGui::IsWindowHovered();

      // Occupy space for all sequences in child canvas scroll area
      ImGui::Dummy(ImVec2(child_canvas_size.x, float(controlHeight)));
      const ImVec2 contentMin = ImGui::GetItemRectMin();
      const ImVec2 contentMax = ImGui::GetItemRectMax();
      const float contentHeight = contentMax.y - contentMin.y;

      // End child canvas
      ImGui::EndChild();
      ImGui::PopStyleColor();

      // Occupy scroll bar
      ImVec2 scrollBarMin, scrollBarMax;
      if (hasScrollBar)
      {
         ImGui::Dummy(scrollBarSize);
         scrollBarMin = ImGui::GetItemRectMin();
         scrollBarMax = ImGui::GetItemRectMax();
      }

      sequence->visibleFrameCount = (int)floorf(sequenceRect.GetWidth() / sequence->framePixelWidth);

      /* Interaction */

      // Panning when zoomed in
      static bool panningView = false;
      static ImVec2 panningViewSource;
      static int panningViewFrame;
      if (ImGui::IsWindowFocused() && io.KeyAlt && io.MouseDown[2] && firstFrame)
      {
         if (!panningView)
         {
            panningViewSource = io.MousePos;
            panningView = true;
            panningViewFrame = *firstFrame;
         }
         *firstFrame = panningViewFrame - int((io.MousePos.x - panningViewSource.x) / sequence->framePixelWidth);
         *firstFrame = ImClamp(*firstFrame, sequence->GetFrameMin(), sequence->GetFrameMax() - sequence->visibleFrameCount);
      }
      if (panningView && !io.MouseDown[2])
      {
         panningView = false;
      }
      if (sequence->visibleFrameCount >= frameCount && firstFrame)
         *firstFrame = sequence->GetFrameMin();

      // May modify *firstFrame further
      sequence->PrepareRendering(sequenceRect.GetWidth());

      int firstFrameUsed = firstFrame ? *firstFrame : 0;

      // Moving current frame
      ImRect topRect(ImVec2(child_canvas_pos.x + legendWidth, canvas_pos.y), ImVec2(child_canvas_pos.x + child_canvas_size.x, canvas_pos.y + headerSize.y));
      if (!MovingCurrentFrame && !MovingScrollBar && movingEntry == -1 && sequenceOptions & SEQUENCER_CHANGE_FRAME && currentFrame && *currentFrame >= 0 && topRect.Contains(io.MousePos) && io.MouseDown[0])
      {
         MovingCurrentFrame = true;
      }
      if (MovingCurrentFrame)
      {
         if (frameCount)
            *currentFrame = (int)((io.MousePos.x - topRect.Min.x) / sequence->framePixelWidth) + firstFrameUsed;
         if (!io.MouseDown[0])
            MovingCurrentFrame = false;
      }
      if (*currentFrame < sequence->GetFrameMin())
         *currentFrame = sequence->GetFrameMin();
      if (*currentFrame > sequence->GetFrameMax())
         *currentFrame = sequence->GetFrameMax();

      /* Visuals */

      // Header
      draw_list->AddRectFilled(canvas_pos, canvas_pos + headerSize, 0xFF3D3837, 0);
      if (expanded)
      {
         if (SequencerAddDelButton(draw_list, ImVec2(canvas_pos.x + 2, canvas_pos.y + 2), !*expanded))
            *expanded = !*expanded;
      }
      if (sequenceOptions & SEQUENCER_ADD)
      {
         if (SequencerAddDelButton(draw_list, ImVec2(canvas_pos.x + legendWidth - ItemHeight, canvas_pos.y + 2), true))
            ImGui::OpenPopup("addEntry");

         if (ImGui::BeginPopup("addEntry"))
         {
            for (int i = 0; i < sequence->GetItemTypeCount(); i++)
               if (ImGui::Selectable(sequence->GetItemTypeName(i)))
               {
                  sequence->Add(i);
                  *selectedEntry = sequence->GetItemCount() - 1;
               }

            ImGui::EndPopup();
            popupOpened = true;
         }
      }

      // header frame number and lines
      int modFrameCount = 10;
      int frameStep = 1;
      while (modFrameCount < std::numeric_limits<int>::max()/2 && (modFrameCount * sequence->framePixelWidth) < 150)
      {
         modFrameCount *= 2;
         frameStep *= 2;
      };
      int halfModFrameCount = modFrameCount / 2;

      auto drawLine = [&](int i, int regionHeight) {
         bool baseIndex = ((i % modFrameCount) == 0) || (i == sequence->GetFrameMax() || i == sequence->GetFrameMin());
         bool halfIndex = (i % halfModFrameCount) == 0;
         int px = (int)child_canvas_pos.x + int(i * sequence->framePixelWidth) + legendWidth - int(firstFrameUsed * sequence->framePixelWidth);
         int tiretStart = baseIndex ? 4 : (halfIndex ? 10 : 14);
         int tiretEnd = baseIndex ? regionHeight : ItemHeight;

         if (px <= (child_canvas_size.x + child_canvas_pos.x) && px >= (child_canvas_pos.x + legendWidth))
         {
            draw_list->AddLine(ImVec2((float)px, canvas_pos.y + (float)tiretStart), ImVec2((float)px, canvas_pos.y + (float)tiretEnd - 1), 0xFF606060, 1);

            draw_list->AddLine(ImVec2((float)px, canvas_pos.y + (float)ItemHeight), ImVec2((float)px, canvas_pos.y + (float)regionHeight - 1), 0x30606060, 1);
         }

         if (baseIndex && px > (child_canvas_pos.x + legendWidth))
         {
            char tmps[512];
            ImFormatString(tmps, IM_ARRAYSIZE(tmps), "%d", i);
            draw_list->AddText(ImVec2((float)px + 3.f, canvas_pos.y), 0xFFBBBBBB, tmps);
         }

      };

      for (int i = firstFrameUsed-firstFrameUsed%modFrameCount + frameStep; i <= firstFrameUsed+sequence->visibleFrameCount; i += frameStep)
      {
         drawLine(i, headerSize.y);
      }
      drawLine(sequence->GetFrameMin(), headerSize.y);
      drawLine(sequence->GetFrameMax(), headerSize.y);

      // full background
      draw_list->AddRectFilled(childFramePos, childFramePos + childFrameSize, 0xFF242424, 0);

      // clip for legend
      draw_list->PushClipRect(childFramePos, ImVec2(child_canvas_pos.x + float(legendWidth), childFramePos.y + childFrameSize.y), true);

      // draw item names in the legend rect on the left
      size_t customHeight = 0;
      for (int i = 0; i < sequenceCount; i++)
      {
         ImVec2 tpos(contentMin.x + 3, contentMin.y + i * ItemHeight + 2 + customHeight);
         draw_list->AddText(tpos, 0xFFFFFFFF, sequence->GetItemLabel(i));

         if (sequenceOptions & SEQUENCER_DEL)
         {
            if (SequencerAddDelButton(draw_list, ImVec2(contentMin.x + legendWidth - ItemHeight + 2 - 10, tpos.y + 2), false))
               delEntry = i;

            if (SequencerAddDelButton(draw_list, ImVec2(contentMin.x + legendWidth - ItemHeight - ItemHeight + 2 - 10, tpos.y + 2), true))
               dupEntry = i;
         }
         customHeight += sequence->GetCustomHeight(i);
      }

      draw_list->PopClipRect();

      // clip for sequences
      draw_list->PushClipRect(sequenceRect.Min, sequenceRect.Max, true);

      // slots background
      customHeight = 0;
      if (selectedEntry && movingEntry < 0)
         *selectedEntry = -1;
      for (int i = 0; i < sequenceCount; i++)
      {
         unsigned int col = (i & 1) ? 0xFF3A3636 : 0xFF413D3D;

         size_t localCustomHeight = sequence->GetCustomHeight(i);
         ImVec2 pos = ImVec2(contentMin.x + legendWidth, contentMin.y + ItemHeight * i + 1 + customHeight);
         ImVec2 sz = ImVec2(child_canvas_size.x + child_canvas_pos.x, pos.y + ItemHeight - 1 + localCustomHeight);
         if (!popupOpened && cy >= pos.y && cy < pos.y + (ItemHeight + localCustomHeight) && movingEntry == -1 && cx>contentMin.x && cx < contentMin.x + child_canvas_size.x)
         {
            col += 0x80201008;
            pos.x -= legendWidth;
         }
         if (canvasHovered && ImRect(pos, sz).Contains(io.MousePos))
         {
            if (selectedEntry && movingEntry < 0)
               *selectedEntry = i;
            if (io.MouseDoubleClicked[0])
               sequence->DoubleClick(i);
         }
         draw_list->AddRectFilled(pos, sz, col, 0);
         customHeight += localCustomHeight;
      }

      // vertical frame lines in content area
      auto drawLineContent = [&](int i, int /*regionHeight*/) {
         int px = (int)child_canvas_pos.x + int(i * sequence->framePixelWidth) + legendWidth - int(firstFrameUsed * sequence->framePixelWidth);
         int tiretStart = int(contentMin.y);
         int tiretEnd = int(contentMax.y);

         if (px <= (child_canvas_size.x + child_canvas_pos.x) && px >= (child_canvas_pos.x + legendWidth))
         {
            //draw_list->AddLine(ImVec2((float)px, child_canvas_pos.y + (float)tiretStart), ImVec2((float)px, child_canvas_pos.y + (float)tiretEnd - 1), 0xFF606060, 1);

            draw_list->AddLine(ImVec2(float(px), float(tiretStart)), ImVec2(float(px), float(tiretEnd)), 0x30606060, 1);
         }
      };
      for (int i = firstFrameUsed; i <= firstFrameUsed+sequence->visibleFrameCount; i += frameStep)
      {
         drawLineContent(i, int(contentHeight));
      }
      drawLineContent(sequence->GetFrameMin(), int(contentHeight));
      drawLineContent(sequence->GetFrameMax(), int(contentHeight));

      // selection
      bool selected = selectedEntry && (*selectedEntry >= 0);
      if (selected)
      {
         customHeight = 0;
         for (int i = 0; i < *selectedEntry; i++)
            customHeight += sequence->GetCustomHeight(i);
         draw_list->AddRectFilled(ImVec2(contentMin.x, contentMin.y + ItemHeight * *selectedEntry + customHeight), 
            ImVec2(contentMin.x + child_canvas_size.x, contentMin.y + ItemHeight * (*selectedEntry + 1) + customHeight), 0x801080FF, 1.f);
      }

      // slots
      customHeight = 0;
      for (int i = 0; i < sequenceCount; i++)
      {
         int* start, * end;
         unsigned int color;
         sequence->Get(i, &start, &end, NULL, &color);
         size_t localCustomHeight = sequence->GetCustomHeight(i);

         ImVec2 pos = ImVec2(contentMin.x + legendWidth - firstFrameUsed * sequence->framePixelWidth, contentMin.y + ItemHeight * i + 1 + customHeight);
         ImVec2 slotP1(pos.x + *start * sequence->framePixelWidth, pos.y + 2);
         ImVec2 slotP2(pos.x + *end * sequence->framePixelWidth + sequence->framePixelWidth, pos.y + ItemHeight - 2);
         ImVec2 slotP3(pos.x + *end * sequence->framePixelWidth + sequence->framePixelWidth, pos.y + ItemHeight - 2 + localCustomHeight);
         unsigned int slotColor = color | 0xFF000000;
         unsigned int slotColorHalf = (color & 0xFFFFFF) | 0x40000000;

         if (slotP1.x <= (child_canvas_size.x + contentMin.x) && slotP2.x >= (contentMin.x + legendWidth) && sequence->drawSequenceBars)
         {
            draw_list->AddRectFilled(slotP1, slotP3, slotColorHalf, 2);
            draw_list->AddRectFilled(slotP1, slotP2, slotColor, 2);
         }
         // Ensure grabbable handles
         const float max_handle_width = slotP2.x - slotP1.x / 3.0f;
         const float min_handle_width = ImMin(10.0f, max_handle_width);
         const float handle_width = ImClamp(sequence->framePixelWidth / 2.0f, min_handle_width, max_handle_width);
         ImRect rects[3] = { ImRect(slotP1, ImVec2(slotP1.x + handle_width, slotP2.y))
               , ImRect(ImVec2(slotP2.x - handle_width, slotP1.y), slotP2)
               , ImRect(slotP1, slotP2) };

         const unsigned int quadColor[] = { 0xFFFFFFFF, 0xFFFFFFFF, slotColor + (selected ? 0 : 0x202020) };
         if (canvasHovered && movingEntry == -1 && (sequenceOptions & SEQUENCER_EDIT_STARTEND))// TODOFOCUS && backgroundRect.Contains(io.MousePos))
         {
            for (int j = 2; j >= 0; j--)
            {
               ImRect& rc = rects[j];
               if (!rc.Contains(io.MousePos))
                  continue;
               draw_list->AddRectFilled(rc.Min, rc.Max, quadColor[j], 2);
            }

            for (int j = 0; j < 3; j++)
            {
               ImRect& rc = rects[j];
               if (!rc.Contains(io.MousePos))
                  continue;
               if (!ImRect(childFramePos, childFramePos + childFrameSize).Contains(io.MousePos))
                  continue;
               if (ImGui::IsMouseClicked(0) && !MovingScrollBar && !MovingCurrentFrame)
               {
                  movingEntry = i;
                  movingPos = cx;
                  movingPart = j + 1;
                  sequence->BeginEdit(movingEntry);
                  break;
               }
            }
         }

         // custom draw
         if (localCustomHeight > 0)
         {
            ImVec2 rp(child_canvas_pos.x, contentMin.y + ItemHeight * i + 1 + customHeight);
            ImRect customRect(rp + ImVec2(legendWidth - (firstFrameUsed - sequence->GetFrameMin() - 0.5f) * sequence->framePixelWidth, float(ItemHeight)),
               rp + ImVec2(legendWidth + (sequence->GetFrameMax() - firstFrameUsed - 0.5f + 2.f) * sequence->framePixelWidth, float(localCustomHeight + ItemHeight)));
            ImRect clippingRect(rp + ImVec2(float(legendWidth), float(ItemHeight)), rp + ImVec2(child_canvas_size.x, float(localCustomHeight + ItemHeight)));

            ImRect legendRect(rp + ImVec2(0.f, float(ItemHeight)), rp + ImVec2(float(legendWidth), float(localCustomHeight)));
            ImRect legendClippingRect(child_canvas_pos + ImVec2(0.f, float(ItemHeight)), child_canvas_pos + ImVec2(float(legendWidth), float(localCustomHeight + ItemHeight)));
            customDraws.push_back({ i, customRect, legendRect, clippingRect, legendClippingRect });
         }
         else
         {
            ImVec2 rp(child_canvas_pos.x, contentMin.y + ItemHeight * i + customHeight);
            ImRect customRect(rp + ImVec2(legendWidth - (firstFrameUsed - sequence->GetFrameMin() - 0.5f) * sequence->framePixelWidth, float(0.f)),
               rp + ImVec2(legendWidth + (sequence->GetFrameMax() - firstFrameUsed - 0.5f + 2.f) * sequence->framePixelWidth, float(ItemHeight)));
            ImRect clippingRect(rp + ImVec2(float(legendWidth), float(0.f)), rp + ImVec2(child_canvas_size.x, float(ItemHeight)));

            compactCustomDraws.push_back({ i, customRect, ImRect(), clippingRect, ImRect() });
         }
         customHeight += localCustomHeight;
      }


      // moving
      if (/*backgroundRect.Contains(io.MousePos) && */movingEntry >= 0)
      {
#if IMGUI_VERSION_NUM >= 18723
         ImGui::SetNextFrameWantCaptureMouse(true);
#else
         ImGui::CaptureMouseFromApp();
#endif
         int diffFrame = int((cx - movingPos) / sequence->framePixelWidth);
         if (std::abs(diffFrame) > 0)
         {
            int* start, * end;
            sequence->Get(movingEntry, &start, &end, NULL, NULL);
            if (selectedEntry)
               *selectedEntry = movingEntry;
            int& l = *start;
            int& r = *end;
            if (movingPart & 1)
               l += diffFrame;
            if (movingPart & 2)
               r += diffFrame;
            if (l < 0)
            {
               if (movingPart & 2)
                  r -= l;
               l = 0;
            }
            if (movingPart & 1 && l > r)
               l = r;
            if (movingPart & 2 && r < l)
               r = l;
            movingPos += int(diffFrame * sequence->framePixelWidth);
         }
         if (!io.MouseDown[0])
         {
            // single select
            if (!diffFrame && movingPart && selectedEntry)
            {
               *selectedEntry = movingEntry;
               ret = true;
            }

            movingEntry = -1;
            sequence->EndEdit();
         }
      }

      for (auto& customDraw : customDraws)
         sequence->CustomDraw(customDraw.index, draw_list, customDraw.customRect, customDraw.legendRect, customDraw.clippingRect, customDraw.legendClippingRect);
      for (auto& customDraw : compactCustomDraws)
         sequence->CustomDrawCompact(customDraw.index, draw_list, customDraw.customRect, customDraw.clippingRect);

      // End drawing content
      draw_list->PopClipRect();

      // cursor
      if (currentFrame && firstFrame && *currentFrame >= *firstFrame && *currentFrame <= sequence->GetFrameMax())
      {
         static const float cursorWidth = 8.f;
         float cursorOffset = contentMin.x + legendWidth + (*currentFrame - firstFrameUsed) * sequence->framePixelWidth + sequence->framePixelWidth / 2 - cursorWidth * 0.5f;
         draw_list->AddLine(ImVec2(cursorOffset, canvas_pos.y), ImVec2(cursorOffset, contentMax.y), 0xA02A2AFF, cursorWidth);
         char tmps[512];
         ImFormatString(tmps, IM_ARRAYSIZE(tmps), "%d", *currentFrame);
         draw_list->AddText(ImVec2(cursorOffset + 10, canvas_pos.y + 2), 0xFF2A2AFF, tmps);
      }

      // copy paste
      if (sequenceOptions & SEQUENCER_COPYPASTE)
      {
         ImRect rectCopy(ImVec2(contentMin.x + 100, child_canvas_pos.y + 2)
            , ImVec2(contentMin.x + 100 + 30, child_canvas_pos.y + ItemHeight - 2));
         bool inRectCopy = rectCopy.Contains(io.MousePos);
         unsigned int copyColor = inRectCopy ? 0xFF1080FF : 0xFF000000;
         draw_list->AddText(rectCopy.Min, copyColor, "Copy");

         ImRect rectPaste(ImVec2(contentMin.x + 140, child_canvas_pos.y + 2)
            , ImVec2(contentMin.x + 140 + 30, child_canvas_pos.y + ItemHeight - 2));
         bool inRectPaste = rectPaste.Contains(io.MousePos);
         unsigned int pasteColor = inRectPaste ? 0xFF1080FF : 0xFF000000;
         draw_list->AddText(rectPaste.Min, pasteColor, "Paste");

         if (inRectCopy && io.MouseReleased[0])
         {
            sequence->Copy();
         }
         if (inRectPaste && io.MouseReleased[0])
         {
            sequence->Paste();
         }
      }

      if (hasScrollBar)
      { // Already occupied space previously

         const float MinBarWidth = scrollBarSize.y*2.5f;
         const float barWidthRatio = ImMin(sequence->visibleFrameCount / (float)ImMax(frameCount, 1), 1.f);
         const float barWidthInPixels = ImMax(barWidthRatio * (child_canvas_size.x - legendWidth), MinBarWidth);

         float startFrameOffset = ((float)(firstFrameUsed - sequence->GetFrameMin()) / (float)frameCount) * (child_canvas_size.x - legendWidth);
         ImVec2 scrollBarA(child_canvas_pos.x + legendWidth, scrollBarMin.y);
         ImVec2 scrollBarB(child_canvas_pos.x + child_canvas_size.x, scrollBarMax.y);
         draw_list->AddRectFilled(scrollBarA, scrollBarB, 0xFF222222, 0);
         draw_list->AddRectFilled(scrollBarA, scrollBarB, 0xFF101010, style.ScrollbarRounding);

         bool inScrollBar = ImRect(scrollBarA, scrollBarB).Contains(io.MousePos);

         ImVec2 scrollBarD(ImMin(scrollBarB.x, scrollBarA.x + startFrameOffset + barWidthInPixels), scrollBarMax.y);
         ImVec2 scrollBarC(ImMin(scrollBarD.x-barWidthInPixels, scrollBarA.x + startFrameOffset), scrollBarMin.y);
         draw_list->AddRectFilled(scrollBarC, scrollBarD, (inScrollBar || MovingScrollBar) ? 0xFF606060 : 0xFF505050, style.ScrollbarRounding);

         ImRect barHandleLeft(scrollBarC, ImVec2(scrollBarC.x + scrollBarSize.y, scrollBarD.y));
         ImRect barHandleRight(ImVec2(scrollBarD.x - scrollBarSize.y, scrollBarC.y), scrollBarD);

         bool onLeft = barHandleLeft.Contains(io.MousePos);
         bool onRight = barHandleRight.Contains(io.MousePos);

         static bool sizingRBar = false;
         static bool sizingLBar = false;

         draw_list->AddRectFilled(barHandleLeft.Min, barHandleLeft.Max, (onLeft || sizingLBar) ? 0xFFAAAAAA : 0xFF666666, style.ScrollbarRounding);
         draw_list->AddRectFilled(barHandleRight.Min, barHandleRight.Max, (onRight || sizingRBar) ? 0xFFAAAAAA : 0xFF666666, style.ScrollbarRounding);

         ImRect scrollBarThumb(scrollBarC, scrollBarD);
         if (sizingRBar && !io.MouseDown[0])
         {
            sizingRBar = false;
         }
         else if (sizingRBar)
         {
            float barNewWidth = ImMax(barWidthInPixels + io.MouseDelta.x, MinBarWidth*0.1f);
            float barRatio = barNewWidth / barWidthInPixels;
            sequence->framePixelWidthTarget = sequence->framePixelWidth = sequence->framePixelWidth / barRatio;
            int newVisibleFrameCount = int((child_canvas_size.x - legendWidth) / sequence->framePixelWidthTarget);
            int lastFrame = *firstFrame + newVisibleFrameCount;
            if (lastFrame > sequence->GetFrameMax())
            {
               sequence->framePixelWidthTarget = sequence->framePixelWidth = (child_canvas_size.x - legendWidth) / float(sequence->GetFrameMax() - *firstFrame);
            }
         }
         else if (sizingLBar && !io.MouseDown[0])
         {
            sizingLBar = false;
         }
         else if (sizingLBar)
         {
            float barNewWidth = ImMax(barWidthInPixels - io.MouseDelta.x, MinBarWidth*0.1f);
            float barRatio = barNewWidth / barWidthInPixels;
            float previousFramePixelWidthTarget = sequence->framePixelWidthTarget;
            sequence->framePixelWidthTarget = sequence->framePixelWidth = sequence->framePixelWidth / barRatio;
            int newVisibleFrameCount = int((child_canvas_size.x - legendWidth) / sequence->framePixelWidthTarget);
            int lastFrame = *firstFrame + sequence->visibleFrameCount;
            int newFirstFrame = lastFrame - newVisibleFrameCount;
            newFirstFrame = ImClamp(newFirstFrame, sequence->GetFrameMin(), ImMax(sequence->GetFrameMax() - newVisibleFrameCount, sequence->GetFrameMin()));
            if (newFirstFrame != lastFrame - newVisibleFrameCount)
               sequence->framePixelWidthTarget = sequence->framePixelWidth = (child_canvas_size.x - legendWidth) / float(lastFrame - newFirstFrame);
            *firstFrame = newFirstFrame;
         }
         else if (MovingScrollBar && !io.MouseDown[0])
         {
            MovingScrollBar = false;
         }
         else if (MovingScrollBar)
         {
            float framesPerPixelInBar = barWidthInPixels / (float)sequence->visibleFrameCount;
            *firstFrame = int((io.MousePos.x - panningViewSource.x) / framesPerPixelInBar) - panningViewFrame;
            *firstFrame = ImClamp(*firstFrame, sequence->GetFrameMin(), ImMax(sequence->GetFrameMax() - sequence->visibleFrameCount, sequence->GetFrameMin()));
         }
         else
         {
            if (scrollBarThumb.Contains(io.MousePos) && ImGui::IsMouseClicked(0) && firstFrame && !MovingCurrentFrame && movingEntry == -1)
            {
               MovingScrollBar = true;
               panningViewSource = io.MousePos;
               panningViewFrame = -*firstFrame;
            }
            if (!sizingRBar && onRight && ImGui::IsMouseClicked(0))
               sizingRBar = true;
            if (!sizingLBar && onLeft && ImGui::IsMouseClicked(0))
               sizingLBar = true;

         }
      }

      ImGui::EndGroup();

      if (delEntry != -1)
      {
         sequence->Del(delEntry);
         if (selectedEntry && (*selectedEntry == delEntry || *selectedEntry >= sequence->GetItemCount()))
            *selectedEntry = -1;
      }

      if (dupEntry != -1)
      {
         sequence->Duplicate(dupEntry);
      }
      return ret;
   }
}
