#include "Plot.h"

#include "implot_internal.h"

// #include <fmt/core.h>

namespace mc_rtc::imgui
{

namespace
{

inline void range_to_limits(const mc_rtc::gui::plot::Range & range,
                            Plot::AxisLimits & limits,
                            const ImPlotRange & plotRange)
{
  if(range.min != -range.inf || range.max != range.inf)
  {
    limits = {range.min, range.max};
    if(range.min == -range.inf)
    {
      limits->first = plotRange.Min;
    }
    if(range.max == range.inf)
    {
      limits->second = plotRange.Max;
    }
  }
  else
  {
    limits = std::nullopt;
  }
}

} // namespace

uint64_t Plot::UID = 0;

Plot::Plot(const std::string & title) : uid_(UID++), title_(title) {}

void Plot::setup_xaxis(const std::string & label, const mc_rtc::gui::plot::Range & range)
{
  x_label_ = label;
  range_to_limits(range, x_limits_, x_range_);
}

void Plot::setup_yaxis_left(const std::string & label, const mc_rtc::gui::plot::Range & range)
{
  y_label_ = label;
  range_to_limits(range, y_limits_, y_range_);
}

void Plot::setup_yaxis_right(const std::string & label, const mc_rtc::gui::plot::Range & range)
{
  y2_label_ = label;
  range_to_limits(range, y2_limits_, y2_range_);
}

void Plot::plot_point(uint64_t did,
                      const std::string & label,
                      double x,
                      double y,
                      mc_rtc::gui::Color color,
                      mc_rtc::gui::plot::Style style,
                      mc_rtc::gui::plot::Side side)
{
  auto get_plot = [&]() -> PlotLine & {
    if(!plots_.count(did))
    {
      plots_[did] = {};
      plots_[did].points.reserve(1024);
    }
    return plots_[did];
  };
  auto & plot = get_plot();
  plot.label = label;
  plot.color = color;
  plot.style = style;
  plot.side = side;
  plot.points.push_back({x, y});
  side == Side::Left ? y_plots_++ : y2_plots_++;
}

void Plot::plot_polygon(uint64_t did,
                        const std::string & label,
                        const mc_rtc::gui::plot::PolygonDescription & polygon,
                        mc_rtc::gui::plot::Side side)
{
  auto & poly = polygons_[did];
  if(poly.polygon != polygon)
  {
    poly.polygon = polygon;
  }
  poly.label = label;
  poly.side = side;
  side == Side::Left ? y_plots_++ : y2_plots_++;
}

void Plot::plot_polygons(uint64_t did,
                         const std::string & label,
                         const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                         mc_rtc::gui::plot::Side side)
{
  auto & group = polygonGroups_[did];
  if(group.polygons != polygons)
  {
    group.polygons = polygons;
  }
  group.label = label;
  group.side = side;
  side == Side::Left ? y_plots_++ : y2_plots_++;
}

void Plot::do_plot()
{
  ImPlotAxisFlags x_flags = ImPlotAxisFlags_AutoFit;
  ImPlotAxisFlags y_flags = ImPlotAxisFlags_AutoFit;
  ImPlotAxisFlags y2_flags = ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_Opposite;
  const char * y_label = y_label_.c_str();
  if(y_plots_ == 0)
  {
    y_flags = ImPlotAxisFlags_NoDecorations;
    y_label = nullptr;
  }
  else
  {
    y2_flags |= ImPlotAxisFlags_NoGridLines;
  }
  const char * y2_label = y2_label_.c_str();
  if(y2_plots_ == 0)
  {
    y2_flags = ImPlotAxisFlags_NoDecorations;
    y2_label = nullptr;
  }
  bool do_ = ImPlot::BeginPlot(fmt::format("{}##{}", title_, uid_).c_str(), ImVec2{-1, 0}, ImPlotFlags_YAxis2);
  if(!do_)
  {
    return;
  }
  ImPlot::SetupAxis(ImAxis_X1, x_label_.c_str(), x_flags);
  if(y_plots_ != 0)
  {
    ImPlot::SetupAxis(ImAxis_Y1, y_label, y_flags);
  }
  if(y2_plots_ != 0)
  {
    ImPlot::SetupAxis(ImAxis_Y2, y2_label, y2_flags);
  }
  if(x_limits_)
  {
    ImPlot::SetupAxisLimits(ImAxis_X1, x_limits_->first, x_limits_->second, ImGuiCond_Always);
  }
  if(y_limits_)
  {
    ImPlot::SetupAxisLimits(ImAxis_Y1, y_limits_->first, y_limits_->second, ImGuiCond_Always);
  }
  if(y2_limits_)
  {
    ImPlot::SetupAxisLimits(ImAxis_Y2, y2_limits_->first, y2_limits_->second, ImGuiCond_Always);
  }
  auto toImVec4 = [](const Color & color) {
    return ImVec4{static_cast<float>(color.r), static_cast<float>(color.g), static_cast<float>(color.b),
                  static_cast<float>(color.a)};
  };
  auto toImU32 = [](const Color & color) {
    return ImGui::ColorConvertFloat4ToU32({static_cast<float>(color.r), static_cast<float>(color.g),
                                           static_cast<float>(color.b), static_cast<float>(color.a)});
  };
  ImPlot::PushStyleVar(ImPlotStyleVar_FitPadding, ImVec2{0.1f, 0.1f});
  auto plot_poly = [this, &toImU32](const PolygonDescription & poly, Side side) {
    auto * draw_list = ImPlot::GetPlotDrawList();
    const auto & style = poly.style();
    // FIXME Handle style
    const auto & outlineColor = poly.outline();
    const auto & fillColor = poly.fill();
    bool closed = poly.closed();
    points_.resize(poly.points().size());
    ImPlot::SetAxis(side == Side::Left ? ImAxis_Y1 : ImAxis_Y2);
    for(size_t i = 0; i < poly.points().size(); ++i)
    {
      points_[i] = ImPlot::PlotToPixels(poly.points()[i][0], poly.points()[i][1]);
      if(ImPlot::FitThisFrame())
      {
        ImPlot::FitPoint({poly.points()[i][0], poly.points()[i][1]});
      }
    }
    if(fillColor.a != 0.0)
    {
      draw_list->AddConvexPolyFilled(points_.data(), points_.size(), toImU32(fillColor));
    }
    draw_list->AddPolyline(points_.data(), points_.size(), toImU32(outlineColor), closed, 2.0f);
  };
  for(const auto & pp : polygons_)
  {
    const auto & poly = pp.second;
    if(ImPlot::BeginItem(poly.label.c_str()))
    {
      plot_poly(poly.polygon, poly.side);
      ImPlot::EndItem();
    }
  }
  for(const auto & pp : polygonGroups_)
  {
    const auto & group = pp.second;
    if(ImPlot::BeginItem(group.label.c_str()))
    {
      for(const auto & p : group.polygons)
      {
        plot_poly(p, group.side);
      }
      ImPlot::EndItem();
    }
  }
  for(const auto & pp : plots_)
  {
    const auto & p = pp.second;
    ImPlot::SetAxis(p.side == Side::Left ? ImAxis_Y1 : ImAxis_Y2);
    if(p.style == Style::Point)
    {
      ImPlot::SetNextLineStyle({0, 0, 0, 0});
      ImPlot::PlotLine(p.label.c_str(), &p.points[0].x, &p.points[0].y, p.points.size(), 0, sizeof(Point));
      if(ImPlot::BeginItem(p.label.c_str()))
      {
        ImPlot::GetCurrentItem()->Color = toImU32(p.color);
        auto * draw_list = ImPlot::GetPlotDrawList();
        auto & point = p.points[p.points.size() - 1];
        auto ppoint = ImPlot::PlotToPixels(point.x, point.y);
        draw_list->AddCircleFilled(ppoint, 4.0f, toImU32(p.color));
        ImPlot::EndItem();
      }
    }
    else
    {
      // FIXME We can not plot dashed and dotted lines yet
      ImPlot::SetNextLineStyle(toImVec4(p.color));
      ImPlot::PlotLine(p.label.c_str(), &p.points[0].x, &p.points[0].y, p.points.size(), 0, sizeof(Point));
    }
  }
  {
    auto ctx = ImPlot::GetCurrentContext();
    x_range_ = ctx->CurrentPlot->Axes[ImAxis_X1].FitExtents;
    y_range_ = ctx->CurrentPlot->Axes[ImAxis_Y1].FitExtents;
    y2_range_ = ctx->CurrentPlot->Axes[ImAxis_Y2].FitExtents;
  }
  ImPlot::EndPlot();
}

} // namespace mc_rtc::imgui
