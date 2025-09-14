#pragma once

#include <mc_rtc/logging.h>
#include <mc_rtc_ros/ros.h>

#include <mutex>
#include <thread>
#include <limits>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

// Generic thread-safe subscriber data wrapper
template<typename Data>
struct SubscriberData
{
  bool isValid() const noexcept
  {
    return time_ <= maxTime_;
  }

  void operator=(const SubscriberData<Data> & data)
  {
    value_ = data.value_;
    time_ = data.time_;
    maxTime_ = data.maxTime_;
  }

  const Data & value() const noexcept
  {
    return value_;
  }

  void tick(double dt)
  {
    time_ += dt;
  }

  void maxTime(double t)
  {
    maxTime_ = t;
  }

  double time() const noexcept
  {
    return time_;
  }

  double maxTime() const noexcept
  {
    return maxTime_;
  }

  void value(const Data & data)
  {
    value_ = data;
    time_ = 0;
  }

private:
  Data value_;
  double time_ = std::numeric_limits<double>::max();
  double maxTime_ = std::numeric_limits<double>::max();
};

// Generic base subscriber
template<typename Data>
struct Subscriber
{
  void tick(double dt)
  {
    data_.tick(dt);
  }

  void maxTime(double t)
  {
    data_.maxTime(t);
  }

  const SubscriberData<Data> data() const noexcept
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    return data_;
  }

protected:
  void value(const Data & data)
  {
    std::lock_guard<std::mutex> l(valueMutex_);
    data_.value(data);
  }

private:
  SubscriberData<Data> data_;
  mutable std::mutex valueMutex_;
};

// ROS-specific subscriber wrapper
template<typename ROSMessageType, typename TargetType>
struct ROS2Subscriber : public Subscriber<TargetType>
{
  template<typename ConverterFun>
  ROS2Subscriber(ConverterFun && fun) : converter_(fun)
  {
  }

  void subscribe(std::shared_ptr<rclcpp::Node> & node, const std::string & topic, const unsigned bufferSize = 1)
  {
    sub_ = node->create_subscription<ROSMessageType>(
      topic, bufferSize, std::bind(&ROS2Subscriber::callback, this, std::placeholders::_1));
  }

  std::string topic() const
  {
    return sub_->get_topic_name();
  }

  const rclcpp::Subscription<ROSMessageType> & subscriber() const
  {
    return sub_;
  }

protected:
  void callback(const std::shared_ptr<const ROSMessageType> & msg)
  {
    this->value(converter_(*msg));
  }

  typename rclcpp::Subscription<ROSMessageType>::SharedPtr sub_;
  std::function<TargetType(const ROSMessageType &)> converter_;
};

// ==============================
// Specific Subscriber for JointTrajectoryControllerState
// ==============================

struct JointTrajectoryPoint
{
  std::vector<double> positions;
  std::vector<double> velocities;
  double time_from_start; // In seconds
};

struct JointTrajectoryCommand
{
  std::vector<std::string> joint_names;
  std::vector<JointTrajectoryPoint> points;
};

struct ROSJointTrajectorySubscriber
: public ROS2Subscriber<trajectory_msgs::msg::JointTrajectory, JointTrajectoryCommand>
{
  ROSJointTrajectorySubscriber()
  : ROS2Subscriber([](const trajectory_msgs::msg::JointTrajectory & msg) {
      JointTrajectoryCommand cmd;
      cmd.joint_names = msg.joint_names;

      for(const auto & pt : msg.points)
      {
        JointTrajectoryPoint out_pt;
        out_pt.positions = pt.positions;
        out_pt.velocities = pt.velocities;

        // Convert ROS2 Duration to seconds
        out_pt.time_from_start = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;

        cmd.points.push_back(out_pt);
      }

      return cmd;
    })
  {
  }
};

