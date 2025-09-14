#pragma once

#include "Category.h"
#include "Plot.h"

namespace mc_rtc::imgui
{

struct Client : public mc_control::ControllerClient
{
  /** Default constructor
   *
   * Contrary to its parent class default constructor, this constructs a connected client to a reasonable default
   * address
   */
  Client();

  /** Update the client data from the latest server message */
  void update();

  /** Draw ImGui elements */
  void draw2D(ImVec2 windowSize);

  /** Draw 3D elements */
  void draw3D();

  /** Remove all elements */
  void clear();

  inline const mc_rtc::Configuration & data() const noexcept
  {
    return data_;
  }

  inline void set_bold_font(ImFont * font)
  {
    bold_font_ = font;
  }

protected:
  std::vector<char> buffer_ = std::vector<char>(65535);
  std::chrono::system_clock::time_point t_last_ = std::chrono::system_clock::now();

  /** No message for unsupported types */
  void default_impl(const std::string &, const ElementId &) final {}

  void started() override;

  void category(const std::vector<std::string> &, const std::string &) override;

  void label(const ElementId & id, const std::string & txt) override;

  void array_label(const ElementId & id,
                   const std::vector<std::string> & labels,
                   const Eigen::VectorXd & data) override;

  void button(const ElementId & id) override;

  void checkbox(const ElementId & id, bool state) override;

  void string_input(const ElementId & id, const std::string & data) override;

  void integer_input(const ElementId & id, int data) override;

  void number_input(const ElementId & id, double data) override;

  void number_slider(const ElementId & id, double data, double min, double max) override;

  void array_input(const ElementId & id,
                   const std::vector<std::string> & labels,
                   const Eigen::VectorXd & data) override;

  void combo_input(const ElementId & id, const std::vector<std::string> & values, const std::string & data) override;

  void data_combo_input(const ElementId & id, const std::vector<std::string> & ref, const std::string & data) override;

  void table_start(const ElementId & id, const std::vector<std::string> & header) override;

  void table_row(const ElementId & id, const std::vector<std::string> & data) override;

  void table_end(const ElementId & id) override;

  void schema(const ElementId & id, const std::string & schema) override;

  void form(const ElementId & id) override;

  void form_checkbox(const ElementId & formId,
                     const std::string & name,
                     bool required,
                     bool default_,
                     bool user_default) override;

  void form_integer_input(const ElementId & formId,
                          const std::string & name,
                          bool required,
                          int default_,
                          bool user_default) override;

  void form_number_input(const ElementId & formId,
                         const std::string & name,
                         bool required,
                         double default_,
                         bool user_default) override;

  void form_string_input(const ElementId & formId,
                         const std::string & name,
                         bool required,
                         const std::string & default_,
                         bool user_default) override;

  void form_array_input(const ElementId & formId,
                        const std::string & name,
                        bool required,
                        const Eigen::VectorXd & default_,
                        bool fixed_size,
                        bool user_default) override;

  void form_combo_input(const ElementId & formId,
                        const std::string & name,
                        bool required,
                        const std::vector<std::string> & values,
                        bool send_index,
                        int user_default) override;

  void form_data_combo_input(const ElementId & formId,
                             const std::string & name,
                             bool required,
                             const std::vector<std::string> & ref,
                             bool send_index) override;

  void start_plot(uint64_t id, const std::string & title) override;

  void plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;

  void plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;

  void plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;

  void plot_point(uint64_t id,
                  uint64_t did,
                  const std::string & legend,
                  double x,
                  double y,
                  mc_rtc::gui::Color color,
                  mc_rtc::gui::plot::Style style,
                  mc_rtc::gui::plot::Side side) override;

  void plot_polygon(uint64_t id,
                    uint64_t did,
                    const std::string & legend,
                    const mc_rtc::gui::plot::PolygonDescription & polygon,
                    mc_rtc::gui::plot::Side side) override;

  void plot_polygons(uint64_t id,
                     uint64_t did,
                     const std::string & legend,
                     const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                     mc_rtc::gui::plot::Side side) override;

  void end_plot(uint64_t id) override;

  void stopped() override;

  Category root_;

  /** Returns a category (creates it if it does not exist */
  Category & getCategory(const std::vector<std::string> & category);

  /** Currently active plots */
  std::unordered_map<uint64_t, std::shared_ptr<Plot>> active_plots_;

  /** Currently inactive plots */
  std::vector<std::shared_ptr<Plot>> inactive_plots_;

  /** Bold font, default font if unset */
  ImFont * bold_font_ = nullptr;

  /** Get a widget with the right type and id */
  template<typename T, typename... Args>
  T & widget(const ElementId & id, Args &&... args)
  {
    auto & category = getCategory(id.category);
    auto it =
        std::find_if(category.widgets.begin(), category.widgets.end(), [&](auto & w) { return w->id.name == id.name; });
    if(it == category.widgets.end())
    {
      auto & w = category.widgets.emplace_back(std::make_unique<T>(*this, id, std::forward<Args>(args)...));
      w->seen = true;
      return *dynamic_cast<T *>(w.get());
    }
    else
    {
      auto w_ptr = dynamic_cast<T *>(it->get());
      if(w_ptr)
      {
        w_ptr->seen = true;
        return *w_ptr;
      }
      /** Different type, remove and add the widget again */
      category.widgets.erase(it);
      return widget<T>(id, std::forward<Args>(args)...);
    }
  }
};

} // namespace mc_rtc::imgui
