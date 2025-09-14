#pragma once

#include "../Widget.h"

namespace mc_rtc::imgui
{

template<typename DataT>
struct SingleInput : public Widget
{
  inline SingleInput(Client & client, const ElementId & id) : Widget(client, id) {}

  ~SingleInput() override = default;

  inline void data(const DataT & data)
  {
    if(!busy_)
    {
      data_ = data;
    }
  }

  virtual void setupBuffer() {}

  virtual DataT dataFromBuffer() = 0;

  template<typename ImGuiFn, typename... Args>
  void draw2D(ImGuiFn fn, Args &&... args)
  {
    ImGui::BeginTable(label("", "Table").c_str(), 3, ImGuiTableFlags_SizingStretchProp);
    ImGui::TableNextColumn();
    ImGui::Text("%s", id.name.c_str());
    ImGui::TableNextColumn();
    if(!busy_)
    {
      if(ImGui::Button(label("Edit").c_str()))
      {
        busy_ = true;
        setupBuffer();
      }
      ImGui::TableNextColumn();
      fn("", std::forward<Args>(args)..., ImGuiInputTextFlags_ReadOnly);
    }
    else
    {
      bool clicked = ImGui::Button(label("Done").c_str());
      ImGui::TableNextColumn();
      fn(label("", "Input").c_str(), std::forward<Args>(args)..., ImGuiInputTextFlags_None);
      if(clicked
         || (ImGui::IsItemDeactivatedAfterEdit()
             && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeyPadEnter))))
      {
        auto nData = dataFromBuffer();
        if(nData != data_)
        {
          data_ = nData;
          client.send_request(id, data_);
          data_ = nData;
        }
        busy_ = false;
      }
    }
    ImGui::EndTable();
  }

protected:
  bool busy_ = false;
  DataT data_;
};

} // namespace mc_rtc::imgui
