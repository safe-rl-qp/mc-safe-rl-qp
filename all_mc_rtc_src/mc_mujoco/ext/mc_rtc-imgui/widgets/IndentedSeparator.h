#pragma once

#include "imgui.h"
#include "imgui_internal.h"

namespace mc_rtc::imgui
{

inline void IndentedSeparator()
{
  auto * window = ImGui::GetCurrentWindow();
  float x1 = window->Pos.x + window->DC.Indent.x;
  float x2 = window->Pos.x + window->Size.x;
  const ImRect bb(ImVec2(x1, window->DC.CursorPos.y), ImVec2(x2, window->DC.CursorPos.y + 1.0f));
  ImGui::ItemSize(ImVec2(0.0f, 0.0f));
  const bool item_visible = ImGui::ItemAdd(bb, 0);
  if(item_visible)
  {
    window->DrawList->AddRectFilled(bb.Min, bb.Max, ImGui::GetColorU32(ImGuiCol_Separator));
  }
}

} // namespace mc_rtc::imgui
