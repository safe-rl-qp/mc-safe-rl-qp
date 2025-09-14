#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>

namespace mc_rtc_ros2
{

/**
 * @brief A thread-safe, generic ROS 2 publisher wrapper for mc_rtc plugins.
 *
 * @tparam MsgType The type of ROS 2 message to publish (e.g. trajectory_msgs::msg::JointTrajectory)
 */
template<typename MsgType>
class ROS2Publisher
{
public:
  ROS2Publisher() = default;

  /**
   * @brief Initialize the ROS 2 publisher
   * 
   * @param node Shared pointer to rclcpp::Node
   * @param topic Name of the topic to publish to
   * @param qos Queue size (default: 10)
   */
  void init(std::shared_ptr<rclcpp::Node> & node, const std::string & topic, size_t qos = 50)
  {
    publisher_ = node->create_publisher<MsgType>(topic, qos);
    topic_ = topic;
  }

  /**
   * @brief Publish a message (thread-safe)
   *
   * @param msg The message to publish
   */
  void publish(const MsgType & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (publisher_)
    {
      publisher_->publish(msg);
    }
  }

  /**
   * @brief Returns the name of the topic being published
   */
  std::string topic() const
  {
    return topic_;
  }

  /**
   * @brief Check if the publisher is initialized
   */
  bool isInitialized() const
  {
    return publisher_ != nullptr;
  }

private:
  typename rclcpp::Publisher<MsgType>::SharedPtr publisher_;
  std::string topic_;
  std::mutex mutex_;
};

} // namespace mc_rtc_ros2
