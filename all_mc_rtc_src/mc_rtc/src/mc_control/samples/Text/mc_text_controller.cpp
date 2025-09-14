/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_text_controller.h"

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/logging.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

MCTextController::MCTextController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
                                   double dt,
                                   const mc_rtc::Configuration & config,
                                   Backend backend)
: MCController(robot, dt, backend)
{
  if(!config.has("Text"))
  {
    mc_rtc::log::error_and_throw("No entries relative to the Text controller in the loaded configuration");
  }
  config_ = config("Text");
}

void MCTextController::reset(const mc_control::ControllerResetData & data)
{
  mc_control::MCController::reset(data);
  if(!config_.has("constraints")) { mc_rtc::log::error_and_throw("No constraints in the provided text file"); }
  if(!config_.has("tasks")) { mc_rtc::log::error_and_throw("No tasks in the provided text file"); }
  for(const auto & c : config_("constraints"))
  {
    constraints_.push_back(mc_solver::ConstraintSetLoader::load(solver(), c));
    solver().addConstraintSet(*constraints_.back());
  }
  postureTask->stiffness(2.0);
  postureTask->weight(10.0);
  solver().addTask(postureTask);
  for(const auto & t : config_("tasks"))
  {
    tasks_.push_back(mc_tasks::MetaTaskLoader::load(solver(), t));
    solver().addTask(tasks_.back());
  }
  std::vector<mc_rbdyn::Contact> contacts = {};
  if(config_.has("contacts")) { contacts = mc_rbdyn::Contact::loadVector(robots(), config_("contacts")); }
  for(const auto & c : contacts) { addContact(Contact::from_mc_rbdyn(*this, c)); }
}

} // namespace mc_control

MULTI_CONTROLLERS_CONSTRUCTOR("Text",
                              mc_control::MCTextController(rm, dt, config, mc_control::MCController::Backend::Tasks),
                              "Text_TVM",
                              mc_control::MCTextController(rm, dt, config, mc_control::MCController::Backend::TVM))
