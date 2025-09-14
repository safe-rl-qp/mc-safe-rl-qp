#pragma once

#include "Widget.h"

namespace mc_rtc::imgui
{

struct Table : public Widget
{
  Table(Client & client, const ElementId & id) : Widget(client, id) {}

  void start(const std::vector<std::string> & header)
  {
    header_ = header;
    data_.clear();
  }

  void row(const std::vector<std::string> & row)
  {
    data_.push_back(row);
  }

  void end() {}

  void draw2D() override
  {
    auto drawVec = [](const std::vector<std::string> & vec)
    {
      for(const auto & v : vec)
      {
        ImGui::TableNextColumn();
        ImGui::Text("%s", v.c_str());
      }
    };
    ImGui::Text("%s", id.name.c_str());
    ImGui::BeginTable(label("", "_table_data").c_str(), header_.size(), ImGuiTableFlags_SizingStretchProp);
    drawVec(header_);
    for(const auto & d : data_)
    {
      drawVec(d);
    }
    ImGui::EndTable();
  }

private:
  std::vector<std::string> header_;
  std::vector<std::vector<std::string>> data_;
};

} // namespace mc_rtc::imgui
