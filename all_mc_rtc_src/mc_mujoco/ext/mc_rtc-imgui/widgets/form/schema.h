#pragma once

#include "widgets.h"

namespace mc_rtc::imgui
{

namespace form
{

struct ArrayForm : public Widget
{
  ArrayForm(const ::mc_rtc::imgui::Widget & parent, const std::string & name, const mc_rtc::Configuration & schema);

  bool ready() override;

  void draw_() override;

  void collect(mc_rtc::Configuration & out) override;

  inline bool trivial() const override
  {
    return false;
  }

protected:
  mc_rtc::Configuration schema_;
  unsigned int minSize_;
  unsigned int maxSize_;
  bool isArrayOfObject_ = false;
  bool isArrayOfArray_ = false;
  std::vector<WidgetPtr> widgets_;
  size_t id_ = 0;

  void addWidget();

  void removeWidget(size_t idx);
};

struct ObjectForm : public Widget
{
  ObjectForm(const ::mc_rtc::imgui::Widget & parent,
             const std::string & name,
             const std::map<std::string, mc_rtc::Configuration> & properties,
             const std::vector<std::string> & required);

  bool ready() override;

  void draw(bool show_header);

  void draw_() override;

  void collect(mc_rtc::Configuration & out) override;

  using Widget::value;

  std::optional<std::string> value(const std::string & name) const;

  inline bool trivial() const override
  {
    return false;
  }

protected:
  std::vector<WidgetPtr> required_;
  std::vector<WidgetPtr> widgets_;
};

} // namespace form

} // namespace mc_rtc::imgui
