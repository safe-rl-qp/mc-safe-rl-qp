/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_end_effector_controller.h"

#include <mc_rtc/logging.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

MCEndEffectorController::MCEndEffectorController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
                                                 double dt,
                                                 const mc_rtc::Configuration & config,
                                                 Backend backend)
: MCController(robot_module, dt, backend)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(*compoundJointConstraint);
  solver().addTask(postureTask.get());
  if(robot().hasSurface("LFullSole") && robot().hasSurface("RFullSole"))
  {
    addContact({robot().name(), env().name(), "LFullSole", "AllGround"});
    addContact({robot().name(), env().name(), "RFullSole", "AllGround"});
  }
  else if(robot().hasSurface("LeftFoot") && robot().hasSurface("RightFoot"))
  {
    addContact({robot().name(), env().name(), "LeftFoot", "AllGround"});
    addContact({robot().name(), env().name(), "RightFoot", "AllGround"});
  }
  else if(robot().mb().joint(0).dof() == 6)
  {
    mc_rtc::log::error_and_throw("EndEffector sample does not support robot {}", robot().name());
  }

  std::string body = robot().mb().bodies().back().name();
  if(robot().hasBody("RARM_LINK7")) { body = "RARM_LINK7"; }
  else if(robot().hasBody("r_wrist")) { body = "r_wrist"; }
  efTask_ = std::make_shared<mc_tasks::EndEffectorTask>(body, robots(), robots().robotIndex(), 5.0, 200.0);
  solver().addTask(efTask_);
  if(robot().mb().joint(0).dof() != 0)
  {
    comTask_ = std::make_shared<mc_tasks::CoMTask>(robots(), robots().robotIndex());
    solver().addTask(comTask_);
  }
  postureTask->weight(1.0);
  if(config.has(robot().name()))
  {
    auto tasks = config(robot().name())("tasks", std::vector<mc_rtc::Configuration>{});
    for(auto & t : tasks)
    {
      tasks_.emplace_back(mc_tasks::MetaTaskLoader::load(solver(), t));
      solver().addTask(tasks_.back());
    }
  }
}

void MCEndEffectorController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  efTask_->reset();
  if(comTask_) { comTask_->reset(); }
  for(auto & t : tasks_) { t->reset(); }
}

} // namespace mc_control

using Controller = mc_control::MCEndEffectorController;
using Backend = Controller::Backend;

MULTI_CONTROLLERS_CONSTRUCTOR("EndEffector",
                              Controller(rm, dt, config, Backend::Tasks),
                              "EndEffector_TVM",
                              Controller(rm, dt, config, Backend::TVM))
