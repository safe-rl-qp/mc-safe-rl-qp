/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_halfsitpose_controller.h"

#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>

namespace mc_control
{

/* Common stuff */
MCHalfSitPoseController::MCHalfSitPoseController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt), halfSitPose(robot().mbc().q)
{

  /* Set the halfSitPose in posture Task */
  const auto & halfSit = robot_module->stance();
  const auto & ref_joint_order = robot().refJointOrder();
  halfSitPose = postureTask->posture();
  for(const auto & j : ref_joint_order)
  {
    if(robot().hasJoint(j))
    {
      auto jIndex = robot().jointIndexByName(j);
      const auto & jTarget = halfSit.at(j);
      if(halfSitPose[jIndex].size() == jTarget.size()) { halfSitPose[jIndex] = jTarget; }
    }
  }

  qpsolver->addConstraintSet(selfCollisionConstraint);
  /* Get the complete collision constraint set */
  selfCollisionConstraint->addCollisions(solver(), robot_module->commonSelfCollisions());
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addTask(postureTask.get());
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->setContacts({});
  mc_rtc::log::success("MCHalfSitPoseController init done");
}

void MCHalfSitPoseController::reset(const ControllerResetData & reset_data)
{
  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  postureTask->reset();
  postureTask.get()->weight(100.);
  postureTask.get()->stiffness(2.);
  robot().forwardKinematics();
  robot().forwardVelocity();
  gui_->removeElement({"Controller"}, "Go half-sitting");
  gui_->addElement({"Controller"},
                   mc_rtc::gui::Button("Go half-sitting", [this]() { postureTask->posture(halfSitPose); }));
}

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("HalfSitPose", mc_control::MCHalfSitPoseController)
