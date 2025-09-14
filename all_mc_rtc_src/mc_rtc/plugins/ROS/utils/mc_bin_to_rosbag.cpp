/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>

#ifdef MC_RTC_ROS_IS_ROS2

#  include <geometry_msgs/msg/quaternion.hpp>
#  include <geometry_msgs/msg/transform.hpp>
#  include <geometry_msgs/msg/twist.hpp>
#  include <geometry_msgs/msg/vector3.hpp>
#  include <geometry_msgs/msg/wrench.hpp>
#  include <std_msgs/msg/bool.hpp>
#  include <std_msgs/msg/float32.hpp>
#  include <std_msgs/msg/float64.hpp>
#  include <std_msgs/msg/float64_multi_array.hpp>
#  include <std_msgs/msg/int16.hpp>
#  include <std_msgs/msg/int32.hpp>
#  include <std_msgs/msg/int64.hpp>
#  include <std_msgs/msg/int8.hpp>
#  include <std_msgs/msg/string.hpp>
#  include <std_msgs/msg/u_int16.hpp>
#  include <std_msgs/msg/u_int32.hpp>
#  include <std_msgs/msg/u_int64.hpp>
#  include <std_msgs/msg/u_int8.hpp>

#  include <rclcpp/rclcpp.hpp>
#  include <rosbag2_cpp/writer.hpp>

using Bool = std_msgs::msg::Bool;
using Int8 = std_msgs::msg::Int8;
using Int16 = std_msgs::msg::Int16;
using Int32 = std_msgs::msg::Int32;
using Int64 = std_msgs::msg::Int64;
using UInt8 = std_msgs::msg::UInt8;
using UInt16 = std_msgs::msg::UInt16;
using UInt32 = std_msgs::msg::UInt32;
using UInt64 = std_msgs::msg::UInt64;
using Float32 = std_msgs::msg::Float32;
using Float64 = std_msgs::msg::Float64;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using String = std_msgs::msg::String;

using Quaternion = geometry_msgs::msg::Quaternion;
using Transform = geometry_msgs::msg::Transform;
using Twist = geometry_msgs::msg::Twist;
using Vector3 = geometry_msgs::msg::Vector3;
using Wrench = geometry_msgs::msg::Wrench;

#else

#  include <geometry_msgs/Quaternion.h>
#  include <geometry_msgs/Transform.h>
#  include <geometry_msgs/Twist.h>
#  include <geometry_msgs/Vector3.h>
#  include <geometry_msgs/Wrench.h>
#  include <std_msgs/Bool.h>
#  include <std_msgs/Float32.h>
#  include <std_msgs/Float64.h>
#  include <std_msgs/Float64MultiArray.h>
#  include <std_msgs/Int16.h>
#  include <std_msgs/Int32.h>
#  include <std_msgs/Int64.h>
#  include <std_msgs/Int8.h>
#  include <std_msgs/String.h>
#  include <std_msgs/UInt16.h>
#  include <std_msgs/UInt32.h>
#  include <std_msgs/UInt64.h>
#  include <std_msgs/UInt8.h>

#  include <rosbag/bag.h>

using Bool = std_msgs::Bool;
using Int8 = std_msgs::Int8;
using Int16 = std_msgs::Int16;
using Int32 = std_msgs::Int32;
using Int64 = std_msgs::Int64;
using UInt8 = std_msgs::UInt8;
using UInt16 = std_msgs::UInt16;
using UInt32 = std_msgs::UInt32;
using UInt64 = std_msgs::UInt64;
using Float32 = std_msgs::Float32;
using Float64 = std_msgs::Float64;
using Float64MultiArray = std_msgs::Float64MultiArray;
using String = std_msgs::String;

using Quaternion = geometry_msgs::Quaternion;
using Transform = geometry_msgs::Transform;
using Twist = geometry_msgs::Twist;
using Vector3 = geometry_msgs::Vector3;
using Wrench = geometry_msgs::Wrench;

#endif

#include "mc_bin_utils.h"
#include <fstream>
#include <string>
#include <vector>

template<typename T>
struct DataToROS
{
  using ret_t = void;

  static ret_t convert(const T &) { static_assert(sizeof(T) == 0, "This should be specialized"); }
};

#define SIMPLE_CONVERT(CPPT, ROSMSGT)       \
  template<>                                \
  struct DataToROS<CPPT>                    \
  {                                         \
    using ret_t = ROSMSGT;                  \
    static ret_t convert(const CPPT & data) \
    {                                       \
      ret_t msg;                            \
      msg.data = data;                      \
      return msg;                           \
    }                                       \
  }

SIMPLE_CONVERT(bool, Bool);
SIMPLE_CONVERT(int8_t, Int8);
SIMPLE_CONVERT(int16_t, Int16);
SIMPLE_CONVERT(int32_t, Int32);
SIMPLE_CONVERT(int64_t, Int64);
SIMPLE_CONVERT(uint8_t, UInt8);
SIMPLE_CONVERT(uint16_t, UInt16);
SIMPLE_CONVERT(uint32_t, UInt32);
SIMPLE_CONVERT(uint64_t, UInt64);
SIMPLE_CONVERT(float, Float32);
SIMPLE_CONVERT(double, Float64);
SIMPLE_CONVERT(std::string, String);

#undef SIMPLE_CONVERT

template<>
struct DataToROS<std::vector<double>>
{
  using ret_t = Float64MultiArray;

  static ret_t convert(const std::vector<double> & data)
  {
    ret_t msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "data";
    msg.layout.dim[0].size = static_cast<unsigned int>(data.size());
    msg.layout.dim[0].stride = static_cast<unsigned int>(data.size());
    msg.data = data;
    return msg;
  }
};

template<>
struct DataToROS<Eigen::Vector6d>
{
  using ret_t = Float64MultiArray;

  static ret_t convert(const Eigen::Vector6d & data)
  {
    ret_t msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "data";
    msg.layout.dim[0].size = 6;
    msg.layout.dim[0].stride = 6;
    for(int i = 0; i < 6; ++i) { msg.data.push_back(data(i)); }
    return msg;
  }
};

template<>
struct DataToROS<Eigen::VectorXd>
{
  using ret_t = Float64MultiArray;

  static ret_t convert(const Eigen::VectorXd & data)
  {
    ret_t msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = "data";
    msg.layout.dim[0].size = static_cast<unsigned int>(data.size());
    msg.layout.dim[0].stride = static_cast<unsigned int>(data.size());
    for(int i = 0; i < data.size(); ++i) { msg.data.push_back(data(i)); }
    return msg;
  }
};

template<>
struct DataToROS<Eigen::Vector2d>
{
  using ret_t = Vector3;

  static ret_t convert(const Eigen::Vector2d & data)
  {
    ret_t msg;
    msg.x = data.x();
    msg.y = data.y();
    msg.z = 0;
    return msg;
  }
};

template<>
struct DataToROS<Eigen::Vector3d>
{
  using ret_t = Vector3;

  static ret_t convert(const Eigen::Vector3d & data)
  {
    ret_t msg;
    msg.x = data.x();
    msg.y = data.y();
    msg.z = data.z();
    return msg;
  }
};

template<>
struct DataToROS<Eigen::Quaterniond>
{
  using ret_t = Quaternion;

  static ret_t convert(const Eigen::Quaterniond & data)
  {
    ret_t msg;
    msg.w = data.w();
    msg.x = data.x();
    msg.y = data.y();
    msg.z = data.z();
    return msg;
  }
};

template<>
struct DataToROS<sva::PTransformd>
{
  using ret_t = Transform;

  static ret_t convert(const sva::PTransformd & pt)
  {
    ret_t msg;
    msg.rotation = DataToROS<Eigen::Quaterniond>::convert(Eigen::Quaterniond(pt.rotation()));
    msg.translation = DataToROS<Eigen::Vector3d>::convert(pt.translation());
    return msg;
  }
};

template<>
struct DataToROS<sva::ForceVecd>
{
  using ret_t = Wrench;

  static ret_t convert(const sva::ForceVecd & fv)
  {
    ret_t msg;
    msg.torque = DataToROS<Eigen::Vector3d>::convert(fv.couple());
    msg.force = DataToROS<Eigen::Vector3d>::convert(fv.force());
    return msg;
  }
};

template<>
struct DataToROS<sva::MotionVecd>
{
  using ret_t = Twist;

  static ret_t convert(const sva::MotionVecd & mv)
  {
    ret_t msg;
    msg.angular = DataToROS<Eigen::Vector3d>::convert(mv.angular());
    msg.linear = DataToROS<Eigen::Vector3d>::convert(mv.linear());
    return msg;
  }
};

template<typename T>
#ifdef MC_RTC_ROS_IS_ROS2
void write(rosbag2_cpp::Writer & bag,
           const rclcpp::Time & now,
#else
void write(rosbag::Bag & bag,
           const ros::Time & now,
#endif
           const mc_rtc::log::FlatLog & log,
           const std::string & entry,
           size_t idx)
{
  const T * data = log.getRaw<T>(entry, idx);
  if(data)
  {
#ifdef MC_RTC_ROS_IS_ROS2
    bag.write(DataToROS<T>::convert(*data), entry, now);
#else
    bag.write(entry, now, DataToROS<T>::convert(*data));
#endif
  }
}

#ifdef MC_RTC_ROS_IS_ROS2
void write(rosbag2_cpp::Writer & bag,
           const rclcpp::Time & now,
#else
void write(rosbag::Bag & bag,
           const ros::Time & now,
#endif
           const mc_rtc::log::FlatLog & log,
           const std::string & entry,
           mc_rtc::log::LogType type,
           size_t idx)
{
  switch(type)
  {
    case mc_rtc::log::LogType::Bool:
      write<bool>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Int8_t:
      write<int8_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Int16_t:
      write<int16_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Int32_t:
      write<int32_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Int64_t:
      write<int64_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Uint8_t:
      write<uint8_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Uint16_t:
      write<uint16_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Uint32_t:
      write<uint32_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Uint64_t:
      write<uint64_t>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Float:
      write<float>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Double:
      write<double>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::String:
      write<std::string>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Quaterniond:
      write<Eigen::Quaterniond>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Vector2d:
      write<Eigen::Vector2d>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Vector3d:
      write<Eigen::Vector3d>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::Vector6d:
      write<Eigen::Vector6d>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::VectorXd:
      write<Eigen::VectorXd>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::PTransformd:
      write<sva::PTransformd>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::ForceVecd:
      write<sva::ForceVecd>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::MotionVecd:
      write<sva::MotionVecd>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::VectorDouble:
      write<std::vector<double>>(bag, now, log, entry, idx);
      break;
    case mc_rtc::log::LogType::None:
      break;
  }
}

void mc_bin_to_rosbag(const std::string & in, const std::string & out, double dt)
{
  mc_rtc::log::FlatLog log(in);
  auto entries = utils::entries(log);
#ifdef MC_RTC_ROS_IS_ROS2
  int argc = 0;
  char * argv[] = {0};
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mc_bin_to_rosbag");
  auto now = node->now();
  rosbag2_cpp::Writer bag;
  bag.open(out);
#else
  ros::Time::init();
  auto now = ros::Time::now();
  rosbag::Bag bag(out, rosbag::bagmode::Write);
#endif
  for(size_t i = 0; i < log.size(); ++i)
  {
    for(const auto & e : entries) { write(bag, now, log, e.first, e.second, i); }
#ifdef MC_RTC_ROS_IS_ROS2
    now += rclcpp::Duration(std::chrono::duration<double>(dt));
#else
    now += ros::Duration(dt);
#endif
  }
  bag.close();
}
