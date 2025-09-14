/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{
/*! \brief Servo an end-effector depending on position error in camera frame */
struct MC_TASKS_DLLAPI PositionBasedVisServoTask : public TrajectoryTaskGeneric
{
public:
  /*! \brief Constructor (from frame)
   *
   * \param frame Control frame
   *
   * \param X_t_s Transformation from the target to the control frame
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  PositionBasedVisServoTask(const mc_rbdyn::RobotFrame & frame,
                            const sva::PTransformd & X_t_s = sva::PTransformd::Identity(),
                            double stiffness = 2.0,
                            double weight = 500.0);

  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param X_b_s Transformation from the controlled body to the surface being controlled
   *
   * \param X_t_s Transformation from the target to the surface
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  PositionBasedVisServoTask(const std::string & bodyName,
                            const sva::PTransformd & X_t_s,
                            const sva::PTransformd & X_b_s,
                            const mc_rbdyn::Robots & robots,
                            unsigned int robotIndex,
                            double stiffness = 2.0,
                            double weight = 500);

  /*! \brief Constructor (from mc_rbdyn::Surface information)
   *
   * \param surfaceName Name of the surface the control
   *
   * \param X_t_s Transformation from the target to the surface
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  PositionBasedVisServoTask(const std::string & surfaceName,
                            const sva::PTransformd & X_t_s,
                            const mc_rbdyn::Robots & robots,
                            unsigned int robotIndex,
                            double stiffness = 2.0,
                            double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective to the current body orientation
   */
  void reset() override;

  /*! \brief Set the current error
   *
   * \param X_t_s Transformation from the target frame to the control frame
   *
   */
  void error(const sva::PTransformd & X_t_s);

  void addToLogger(mc_rtc::Logger & logger) override;

private:
  sva::PTransformd X_t_s_;
};
} // namespace mc_tasks
