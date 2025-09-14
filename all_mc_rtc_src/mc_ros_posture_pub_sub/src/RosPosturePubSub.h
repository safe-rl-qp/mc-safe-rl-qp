/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <Eigen/src/Core/Matrix.h>
#include <vector>

#ifdef MC_RTC_ROS_IS_ROS2
  #include "utils/ROS2Subscriber.h"
  #include "utils/ROS2Publisher.h"
#else
  #error "This plugin is designed for ROS2 only."
#endif

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>



namespace mc_plugin
{

struct RosPosturePubSub : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~RosPosturePubSub() override;

private:

  static std::vector<double> computeAngleOffsets(
    size_t actuator_count,
    const std::vector<double> &target_angles,
    const std::vector<double> &current_angles);

  double dt_;
  double pub_freq_;
  double pub_dt_;
  double counter_;
  size_t jointNumber_;
  std::shared_ptr<rclcpp::Node> node_;
  std::thread spinThread_;
  control_msgs::msg::JointTrajectoryControllerState msg_;

  std::vector<std::string> robotJointsName_;
  std::vector<double> robotPositions_;
  std::vector<double> robotVelocities_;

  std::map<std::string, std::vector<double>> postureReceived_;
  Eigen::VectorXd postureReceivedVector_;
  Eigen::VectorXd robotPositionsVector_;
  Eigen::VectorXd robotVelocitiesVector_;
  

  std::string postureCommandTopic_;
  std::string postureFeedbackTopic_;
  ROSJointTrajectorySubscriber posture_sub_;
  mc_rtc_ros2::ROS2Publisher<control_msgs::msg::JointTrajectoryControllerState> posture_pub_;

  void rosSpinner();
};

} // namespace mc_plugin
