#pragma once

#include "Widget.h"

namespace mc_rtc::imgui
{

struct Label : public Widget
{
  inline Label(Client & client, const ElementId & id) : Widget(client, id) {}

  ~Label() override = default;

  inline void data(const std::string & txt)
  {
    txt_ = txt;
  }

  inline void draw2D() override
  {
    if(txt_.size())
    {
      ImGui::Text("%s %s", id.name.c_str(), txt_.c_str());
    }
    else
    {
      ImGui::Text("%s", id.name.c_str());
    }
  }

private:
  std::string txt_;
};

} // namespace mc_rtc::imgui
