#pragma once

#include "Widget.h"

namespace mc_rtc::imgui
{

struct ArrayLabel : public Widget
{
  inline ArrayLabel(Client & client, const ElementId & id) : Widget(client, id) {}

  ~ArrayLabel() override = default;

  inline void data(const std::vector<std::string> & labels, const Eigen::VectorXd & data)
  {
    labels_ = labels;
    data_ = data;
  }

  void draw2D() override
  {
    ImGui::Text("%s", id.name.c_str());
    if(data_.size() > 6 && labels_.size() == 0)
    {
      bool text_hovered = ImGui::IsItemHovered();
      ImGui::SameLine();
      ImGui::Text("%s", fmt::format("{:0.4f}", data_.norm()).c_str());
      if(text_hovered || ImGui::IsItemHovered())
      {
        ImGui::BeginTooltip();
        ImGui::Text("%s", fmt::format("{}", data_).c_str());
        ImGui::EndTooltip();
      }
      return;
    }
    ImVec2 min;
    ImGui::BeginTable(label("", "_table_data").c_str(), data_.size(), ImGuiTableFlags_SizingStretchProp);
    for(size_t i = 0; i < std::min<size_t>(labels_.size(), data_.size()); ++i)
    {
      ImGui::TableNextColumn();
      ImGui::Text("%s", labels_[i].c_str());
      if(i == 0)
      {
        min = ImGui::GetItemRectMin();
      }
    }
    ImGui::TableNextRow();
    ImVec2 max;
    for(int i = 0; i < data_.size(); ++i)
    {
      ImGui::TableNextColumn();
      ImGui::Text("%.4f", data_(i));
      if(i == 0 && labels_.size() == 0)
      {
        min = ImGui::GetItemRectMin();
      }
      if(i == data_.size() - 1)
      {
        max = ImGui::GetItemRectMax();
      }
    }
    ImGui::EndTable();
    if(ImGui::IsMouseHoveringRect(min, max))
    {
      ImGui::BeginTooltip();
      ImGui::Text("%s", fmt::format("{:0.4f}", data_.norm()).c_str());
      ImGui::EndTooltip();
    }
  }

private:
  std::vector<std::string> labels_;
  Eigen::VectorXd data_;
};

} // namespace mc_rtc::imgui
