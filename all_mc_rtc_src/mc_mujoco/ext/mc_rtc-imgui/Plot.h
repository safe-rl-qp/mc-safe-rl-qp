#pragma once

#include "implot.h"

#include <mc_rtc/gui/plot/types.h>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mc_rtc::imgui
{

struct Plot
{
  using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;

  using Color = mc_rtc::gui::Color;
  using Side = mc_rtc::gui::plot::Side;
  using Style = mc_rtc::gui::plot::Style;

  Plot(const std::string & title);

  inline const std::string & title() const noexcept
  {
    return title_;
  }

  inline void start_plot() noexcept
  {
    seen_ = true;
    y_plots_ = 0;
    y2_plots_ = 0;
  }

  void setup_xaxis(const std::string & label, const mc_rtc::gui::plot::Range & range);

  void setup_yaxis_left(const std::string & label, const mc_rtc::gui::plot::Range & range);

  void setup_yaxis_right(const std::string & label, const mc_rtc::gui::plot::Range & range);

  void plot_point(uint64_t did,
                  const std::string & label,
                  double x,
                  double y,
                  mc_rtc::gui::Color color,
                  mc_rtc::gui::plot::Style style,
                  mc_rtc::gui::plot::Side side);

  void plot_polygon(uint64_t did,
                    const std::string & legend,
                    const mc_rtc::gui::plot::PolygonDescription & polygon,
                    mc_rtc::gui::plot::Side side);

  void plot_polygons(uint64_t did,
                     const std::string & legend,
                     const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                     mc_rtc::gui::plot::Side side);

  void do_plot();

  inline bool seen() noexcept
  {
    auto out = seen_;
    seen_ = false;
    return out;
  }

  using AxisLimits = std::optional<std::pair<double, double>>;

private:
  uint64_t uid_;
  std::string title_;
  std::string x_label_;
  std::string y_label_ = "";
  std::string y2_label_ = "";
  AxisLimits x_limits_ = std::nullopt;
  AxisLimits y_limits_ = std::nullopt;
  AxisLimits y2_limits_ = std::nullopt;
  ImPlotRange x_range_;
  ImPlotRange y_range_;
  ImPlotRange y2_range_;
  bool seen_ = false;
  uint64_t y_plots_ = 0;
  uint64_t y2_plots_ = 0;
  struct Point
  {
    double x;
    double y;
  };
  struct PlotLine
  {
    std::vector<Point> points;
    std::string label;
    Color color;
    Side side;
    Style style;
  };
  struct Polygon
  {
    PolygonDescription polygon;
    std::string label;
    Side side;
  };
  struct PolygonGroup
  {
    std::vector<PolygonDescription> polygons;
    std::string label;
    Side side;
  };
  std::unordered_map<uint64_t, PlotLine> plots_;
  std::unordered_map<uint64_t, Polygon> polygons_;
  std::unordered_map<uint64_t, PolygonGroup> polygonGroups_;
  std::vector<ImVec2> points_;

  static uint64_t UID;
};

} // namespace mc_rtc::imgui
