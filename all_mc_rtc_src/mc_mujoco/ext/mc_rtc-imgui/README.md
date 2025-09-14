mc_rtc-imgui
--

This project provides a base client for [mc_rtc](https://github.com/jrl-umi3218/mc_rtc) using [Dear ImGui](https://github.com/ocornut/imgui/). It provides only 2D widgets, i.e. it supports the following elements provided by mc\_rtc GUI server:

- ArrayInput
- ArrayLabel
- Button
- Checkbox
- ComboInput
- DataComboInput
- Form
- IntegerInput
- Label
- NumberInput
- NumberSlider
- Schema
- StringInput
- Table

You should use the files provided in your project directly (e.g. as a submodule) and derive from `mc_rtc::imgui::Client` to implement 3D elements.

The code supposes:
- Dear ImGui context has been initialized when `mc_rtc::imgui::Client` is used
- Dear ImGui headers are on the search path and you link with imgui library
- mc\_rtc headers are on the search path and you link with `mc_rtc::mc_control`
