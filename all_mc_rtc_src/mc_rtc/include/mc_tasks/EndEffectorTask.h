/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PositionTask.h>

namespace mc_tasks
{

/*! \brief Controls an end-effector
 *
 * This task is a thin wrapper around the appropriate tasks in Tasks.
 * The task objective is given in the world frame. For relative control
 * see mc_tasks::RelativeEndEffectorTask
 */
struct MC_TASKS_DLLAPI EndEffectorTask : public MetaTask
{
public:
  /*! \brief Constructor
   *
   * \pararm frame Control frame
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  EndEffectorTask(const mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 1000.0);

  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
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
  EndEffectorTask(const std::string & bodyName,
                  const mc_rbdyn::Robots & robots,
                  unsigned int robotIndex,
                  double stiffness = 2.0,
                  double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the task objective to the current end-effector position
   */
  void reset() override;

  /*! \brief Increment the target position
   *
   * \param dtr Change in target position
   *
   */
  virtual void add_ef_pose(const sva::PTransformd & dtr);

  /*! \brief Change the target position
   *
   * \param tf New target position
   *
   */
  virtual void set_ef_pose(const sva::PTransformd & tf);

  /*! \brief Returns the current target positions
   *
   * \returns Current target position
   *
   */
  virtual sva::PTransformd get_ef_pose();

  void dimWeight(const Eigen::VectorXd & dimW) override;

  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override;

  Eigen::VectorXd speed() const override;

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  using MetaTask::name;

  void name(const std::string & name) override;

public:
  std::shared_ptr<mc_tasks::PositionTask> positionTask;
  std::shared_ptr<mc_tasks::OrientationTask> orientationTask;

  sva::PTransformd curTransform;

protected:
  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void addToSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

  void removeFromLogger(mc_rtc::Logger & logger) override;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  inline const mc_rbdyn::RobotFrame & frame() const noexcept { return *positionTask->frame_; }
};

} // namespace mc_tasks
