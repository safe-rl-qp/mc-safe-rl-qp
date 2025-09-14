/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/api.h>

namespace mc_tasks
{

/*! \brief Controls an end-effector relatively to another frame
 *
 * This task is the relative counter-part to mc_tasks::EndEffectorTask.
 * The difference is that the control is done relatively to a given frame rather than the world frame.
 */
struct MC_TASKS_DLLAPI RelativeEndEffectorTask : public EndEffectorTask
{
public:
  /*! \brief Constructor
   *
   * \param frame Control frame
   *
   * \param relative Relative frame
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  RelativeEndEffectorTask(const mc_rbdyn::RobotFrame & frame,
                          const mc_rbdyn::Frame & relative,
                          double stiffness = 10.0,
                          double weight = 1000.0);

  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param relBodyName Name of the body relatively to which the end-effector
   * is controlled. If empty, defaults to the robot's base.
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  RelativeEndEffectorTask(const std::string & bodyName,
                          const mc_rbdyn::Robots & robots,
                          unsigned int robotIndex,
                          const std::string & relBodyName = "",
                          double stiffness = 10.0,
                          double weight = 1000.0);

  void reset() override;

  void add_ef_pose(const sva::PTransformd & dtr) override;

  void set_ef_pose(const sva::PTransformd & tf) override;

  sva::PTransformd get_ef_pose() override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

private:
  mc_rbdyn::ConstFramePtr relative_;

  void update(mc_solver::QPSolver &) override;
};

} // namespace mc_tasks
