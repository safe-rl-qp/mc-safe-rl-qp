#pragma once

#include <mc_control/ControllerClient.h>

#include <imgui.h>

#include <memory>
#include <string>

// #ifdef SPDLOG_FMT_EXTERNAL
// #  include <fmt/ranges.h>
// #else
// #  include <spdlog/fmt/bundled/ranges.h>
// #endif

// #include <mc_rtc/fmt_eigen.h>
// #include <fmt/core.h>
// #include <vector>

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <string_view>
#include <vector>
#include <string>
#include <Eigen/Dense>

template <> 
struct fmt::formatter<Eigen::VectorXd> : fmt::formatter<std::string> {
    template <typename FormatContext>
    auto format(const Eigen::VectorXd& vec, FormatContext& ctx) {
        std::string result = "[";
        for (int i = 0; i < vec.size(); ++i) {
            result += fmt::format("{}{}", vec[i], (i < vec.size() - 1 ? ", " : ""));
        }
        result += "]";
        return formatter<std::string>::format(result, ctx);
    }
};



namespace mc_rtc::imgui
{

/* Typedef for brievity */
using ElementId = mc_control::ElementId;

/** Forward declaration */
struct Client;

/** A widget in the GUI */
struct Widget
{
  inline Widget(Client & client, const ElementId & id) : client(client), id(id) {}

  virtual ~Widget() = default;

  Client & client;
  ElementId id;
  bool seen = true;

  /** Draw the 2D elements of the widget */
  virtual void draw2D() {}

  /** Draw the 3D elements of the widget */
  virtual void draw3D() {}

  inline std::string label(std::string_view label, std::string_view extra = "")
  {
    return fmt::format("{}##{}{}{}", label, id.category, id.name, extra);
  }

  inline std::string label(std::string_view label, int i)
  {
    return fmt::format("{}##{}{}{}", label, id.category, id.name, i);
  }

  inline std::string label(std::string_view label, size_t i)
  {
    return fmt::format("{}##{}{}{}", label, id.category, id.name, i);
  }
};

using WidgetPtr = std::unique_ptr<Widget>;

} // namespace mc_rtc::imgui
