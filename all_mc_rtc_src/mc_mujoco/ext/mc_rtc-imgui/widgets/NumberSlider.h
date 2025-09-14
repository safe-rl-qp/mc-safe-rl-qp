#pragma once

#include "Widget.h"

namespace mc_rtc::imgui
{

struct NumberSlider : public Widget
{
  inline NumberSlider(Client & client, const ElementId & id) : Widget(client, id) {}

  ~NumberSlider() override = default;

  inline void data(double data, double min, double max)
  {
    data_ = data;
    min_ = min;
    max_ = max;
  }

  inline void draw2D() override
  {
    ImGui::BeginTable(label("", "Table").c_str(), 2, ImGuiTableFlags_SizingStretchProp);
    ImGui::TableNextColumn();
    ImGui::Text("%s", id.name.c_str());
    ImGui::TableNextColumn();
    if(ImGui::SliderFloat(label("").c_str(), &data_, min_, max_))
    {
      client.send_request(id, data_);
    }
    ImGui::EndTable();
  }

private:
  float data_;
  float min_;
  float max_;
};

} // namespace mc_rtc::imgui
