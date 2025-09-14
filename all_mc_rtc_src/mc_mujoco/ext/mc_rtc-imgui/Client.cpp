#include "Client.h"

#include "widgets/ArrayInput.h"
#include "widgets/ArrayLabel.h"
#include "widgets/Button.h"
#include "widgets/Checkbox.h"
#include "widgets/ComboInput.h"
#include "widgets/DataComboInput.h"
#include "widgets/Form.h"
#include "widgets/IntegerInput.h"
#include "widgets/Label.h"
#include "widgets/NumberInput.h"
#include "widgets/NumberSlider.h"
#include "widgets/Schema.h"
#include "widgets/StringInput.h"
#include "widgets/Table.h"

#include <boost/filesystem.hpp>
// #include <fmt/core.h>

namespace bfs = boost::filesystem;

namespace mc_rtc::imgui
{

Client::Client() : mc_control::ControllerClient()
{
  std::string socket = fmt::format("ipc://{}", (bfs::temp_directory_path() / "mc_rtc_").string());
  connect(socket + "pub.ipc", socket + "rep.ipc");
  timeout(3.0);
}

void Client::update()
{
  run(buffer_, t_last_);
}

void Client::draw2D(ImVec2 windowSize)
{
  if(!bold_font_)
  {
    ImGuiIO & io = ImGui::GetIO();
    bold_font_ = io.FontDefault;
  }
  auto left_margin = 15;
  auto top_margin = 50;
  auto bottom_margin = 50;
  auto width = windowSize.x - left_margin;
  auto height = windowSize.y - top_margin - bottom_margin;
  if(!root_.empty())
  {
    ImGui::SetNextWindowPos(ImVec2(left_margin, top_margin), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(0.4f * width, 0.7f * height), ImGuiCond_FirstUseEver);
    ImGui::Begin("mc_rtc");
    root_.draw2D();
    ImGui::End();
  }
  if(active_plots_.size() || inactive_plots_.size())
  {
    bool open_plots = true;
    ImGui::Begin("Plots", active_plots_.size() != 0 ? nullptr : &open_plots);
    ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_Reorderable;
    if(ImGui::BeginTabBar("Plots", tab_bar_flags))
    {
      size_t id = 0;
      for(auto & p : active_plots_)
      {
        auto tab_id = fmt::format("{}##{}", p.second->title(), id++);
        ImGui::PushFont(bold_font_);
        if(ImGui::BeginTabItem(tab_id.c_str()))
        {
          ImGui::PopFont();
          p.second->do_plot();
          ImGui::EndTabItem();
        }
        else
        {
          ImGui::PopFont();
        }
      }
      for(auto it = inactive_plots_.begin(); it != inactive_plots_.end();)
      {
        auto & p = *it;
        auto tab_id = fmt::format("{}##{}", p->title(), id++);
        bool open_ = true;
        if(ImGui::BeginTabItem(tab_id.c_str(), &open_))
        {
          p->do_plot();
          ImGui::EndTabItem();
        }
        it = open_ ? std::next(it) : inactive_plots_.erase(it);
      }
      ImGui::EndTabBar();
    }
    ImGui::End();
    if(!open_plots)
    {
      inactive_plots_.clear();
    }
  }
}

void Client::draw3D()
{
  root_.draw3D();
}

void Client::started()
{
  root_.started();
}

void Client::stopped()
{
  root_.stopped();
  for(auto it = active_plots_.begin(); it != active_plots_.end();)
  {
    if(!it->second->seen())
    {
      inactive_plots_.push_back(it->second);
      it = active_plots_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void Client::clear()
{
  root_.categories.clear();
  root_.widgets.clear();
}

/** We rely on widgets to create categories */
void Client::category(const std::vector<std::string> &, const std::string &) {}

void Client::label(const ElementId & id, const std::string & txt)
{
  widget<Label>(id).data(txt);
}

void Client::array_label(const ElementId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data)
{
  widget<ArrayLabel>(id).data(labels, data);
}

void Client::button(const ElementId & id)
{
  widget<Button>(id);
}

void Client::checkbox(const ElementId & id, bool state)
{
  widget<Checkbox>(id).data(state);
}

void Client::string_input(const ElementId & id, const std::string & data)
{
  widget<StringInput>(id).data(data);
}

void Client::integer_input(const ElementId & id, int data)
{
  widget<IntegerInput>(id).data(data);
}

void Client::number_input(const ElementId & id, double data)
{
  widget<NumberInput>(id).data(data);
}

void Client::number_slider(const ElementId & id, double data, double min, double max)
{
  widget<NumberSlider>(id).data(data, min, max);
}

void Client::array_input(const ElementId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data)
{
  widget<ArrayInput>(id).data(labels, data);
}

void Client::combo_input(const ElementId & id, const std::vector<std::string> & values, const std::string & data)
{
  widget<ComboInput>(id).data(values, data);
}

void Client::data_combo_input(const ElementId & id, const std::vector<std::string> & values, const std::string & data)
{
  widget<DataComboInput>(id).data(values, data);
}

void Client::table_start(const ElementId & id, const std::vector<std::string> & header)
{
  widget<Table>(id).start(header);
}

void Client::table_row(const ElementId & id, const std::vector<std::string> & data)
{
  widget<Table>(id).row(data);
}

void Client::table_end(const ElementId & id)
{
  widget<Table>(id).end();
}

void Client::form(const ElementId & id)
{
  widget<Form>(id);
}

template<typename T>
std::optional<T> null_or_default(const T & value, bool user_default)
{
  if(user_default)
  {
    return value;
  }
  return std::nullopt;
}

void Client::form_checkbox(const ElementId & id,
                           const std::string & name,
                           bool required,
                           bool default_,
                           bool user_default)
{
  widget<Form>(id).widget<form::Checkbox>(name, required, null_or_default(default_, user_default));
}

void Client::form_integer_input(const ElementId & id,
                                const std::string & name,
                                bool required,
                                int default_,
                                bool user_default)
{
  widget<Form>(id).widget<form::IntegerInput>(name, required, null_or_default(default_, user_default));
}

void Client::form_number_input(const ElementId & id,
                               const std::string & name,
                               bool required,
                               double default_,
                               bool user_default)
{
  widget<Form>(id).widget<form::NumberInput>(name, required, null_or_default(default_, user_default));
}

void Client::form_string_input(const ElementId & id,
                               const std::string & name,
                               bool required,
                               const std::string & default_,
                               bool user_default)
{
  widget<Form>(id).widget<form::StringInput>(name, required, null_or_default(default_, user_default));
}

void Client::form_array_input(const ElementId & id,
                              const std::string & name,
                              bool required,
                              const Eigen::VectorXd & default_,
                              bool fixed_size,
                              bool user_default)
{
  widget<Form>(id).widget<form::ArrayInput>(name, required, null_or_default(default_, user_default), fixed_size);
}

void Client::form_combo_input(const ElementId & id,
                              const std::string & name,
                              bool required,
                              const std::vector<std::string> & values,
                              bool send_index,
                              int user_default)
{
  widget<Form>(id).widget<form::ComboInput>(name, required, values, send_index, user_default);
}

void Client::form_data_combo_input(const ElementId & id,
                                   const std::string & name,
                                   bool required,
                                   const std::vector<std::string> & ref,
                                   bool send_index)
{
  widget<Form>(id).widget<form::DataComboInput>(name, required, ref, send_index);
}

void Client::schema(const ElementId & id, const std::string & schema)
{
  widget<Schema>(id).data(schema);
}

auto Client::getCategory(const std::vector<std::string> & category) -> Category &
{
  std::reference_wrapper<Category> out(root_);
  for(size_t i = 0; i < category.size(); ++i)
  {
    auto & cat = out.get();
    auto & next = category[i];
    auto it = std::find_if(cat.categories.begin(), cat.categories.end(), [&](auto & c) { return c->name == next; });
    if(it != cat.categories.end())
    {
      out = std::ref(*it->get());
    }
    else
    {
      out = *cat.categories.emplace_back(std::make_unique<Category>(next, cat.depth + 1));
    }
  }
  return out.get();
}

void Client::start_plot(uint64_t id, const std::string & title)
{
  if(!active_plots_.count(id))
  {
    active_plots_[id] = std::make_shared<Plot>(title);
  }
  if(active_plots_[id]->title() != title)
  {
    inactive_plots_.push_back(active_plots_[id]);
    active_plots_.erase(id);
    active_plots_[id] = std::make_shared<Plot>(title);
  }
  active_plots_[id]->start_plot();
}

void Client::plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  active_plots_[id]->setup_xaxis(legend, range);
}

void Client::plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  active_plots_[id]->setup_yaxis_left(legend, range);
}

void Client::plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  active_plots_[id]->setup_yaxis_right(legend, range);
}

void Client::plot_point(uint64_t id,
                        uint64_t did,
                        const std::string & legend,
                        double x,
                        double y,
                        mc_rtc::gui::Color color,
                        mc_rtc::gui::plot::Style style,
                        mc_rtc::gui::plot::Side side)
{
  active_plots_[id]->plot_point(did, legend, x, y, color, style, side);
}

void Client::plot_polygon(uint64_t id,
                          uint64_t did,
                          const std::string & legend,
                          const mc_rtc::gui::plot::PolygonDescription & polygon,
                          mc_rtc::gui::plot::Side side)
{
  active_plots_[id]->plot_polygon(did, legend, polygon, side);
}

void Client::plot_polygons(uint64_t id,
                           uint64_t did,
                           const std::string & legend,
                           const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                           mc_rtc::gui::plot::Side side)
{
  active_plots_[id]->plot_polygons(did, legend, polygons, side);
}

void Client::end_plot(uint64_t) {}

} // namespace mc_rtc::imgui
