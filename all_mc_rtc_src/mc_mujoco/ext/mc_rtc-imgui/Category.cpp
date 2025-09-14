#include "Category.h"

#include "widgets/IndentedSeparator.h"

namespace mc_rtc::imgui
{

void Category::draw2D()
{
  for(size_t i = 0; i < widgets.size();)
  {
    auto & w = widgets[i];
    if(w->id.sid == -1)
    {
      w->draw2D();
      ++i;
      if(i != widgets.size())
      {
        IndentedSeparator();
      }
      continue;
    }
    size_t j = i + 1;
    while(j < widgets.size() && widgets[j]->id.sid == w->id.sid)
    {
      ++j;
    }
    ImGui::BeginTable(fmt::format("{}_table_{}", w->id.category, i).c_str(), j - i, ImGuiTableFlags_SizingStretchProp);
    for(; i < j; ++i)
    {
      ImGui::TableNextColumn();
      widgets[i]->draw2D();
    }
    ImGui::EndTable();
    if(i != widgets.size())
    {
      IndentedSeparator();
    }
  }
  if(categories.size())
  {
    ImGui::Indent();
    std::sort(categories.begin(), categories.end(),
              [](const auto & lhs, const auto & rhs) { return lhs->name < rhs->name; });
    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_Reorderable;
    if(ImGui::BeginTabBar(name.c_str(), tab_bar_flags))
    {
      for(auto & cat : categories)
      {
        if(ImGui::BeginTabItem(cat->name.c_str()))
        {
          cat->draw2D();
          ImGui::EndTabItem();
        }
      }
      ImGui::EndTabBar();
    }
    ImGui::Unindent();
  }
}

void Category::draw3D()
{
  for(auto & w : widgets)
  {
    w->draw3D();
  }
  for(auto & cat : categories)
  {
    cat->draw3D();
  }
}

void Category::started()
{
  for(auto & w : widgets)
  {
    w->seen = false;
  }
  for(auto & cat : categories)
  {
    cat->started();
  }
}

void Category::stopped()
{
  /** Clean up categories first */
  for(auto & cat : categories)
  {
    cat->stopped();
  }
  /** Remove empty categories */
  {
    auto it = std::remove_if(categories.begin(), categories.end(),
                             [](const auto & c) { return c->widgets.size() == 0 && c->categories.size() == 0; });
    categories.erase(it, categories.end());
  }
  /** Remove widgets that have not been seen */
  {
    auto it = std::remove_if(widgets.begin(), widgets.end(), [](const auto & w) { return !w->seen; });
    widgets.erase(it, widgets.end());
  }
}

} // namespace mc_rtc::imgui
