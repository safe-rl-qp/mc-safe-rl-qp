/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/PositionTask.h>

#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

PositionTask::PositionTask(const std::string & bodyName,
                           const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           double stiffness,
                           double weight)
: PositionTask(robots.robot(robotIndex).frame(bodyName), stiffness, weight)
{
}

PositionTask::PositionTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: TrajectoryTaskGeneric(frame.robot().robots(), frame.robot().robotIndex(), stiffness, weight), frame_(frame)
{
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::PositionTask>(robots.mbs(), static_cast<int>(rIndex), frame.body(),
                                                        frame.position().translation(), frame.X_b_f().translation());
      break;
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::PositionFunction>(frame);
      break;
    default:
      mc_rtc::log::error_and_throw("[PositionTask] Not implemented for backend: {}", backend_);
  }
  type_ = "position";
  name_ = "position_" + frame.robot().name() + "_" + frame.name();
}

void PositionTask::reset()
{
  TrajectoryTaskGeneric::reset();
  position(frame_->position().translation());
}

void PositionTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target", this, [this]() { return position(); });
  logger.addLogEntry(name_ + "_curPos", this, [this]() { return frame_->position().translation(); });
  logger.addLogEntry(name_ + "_curVel", this, [this]() { return frame_->velocity().linear(); });
}

void PositionTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Point3D(
                     "pos_target", [this]() { return this->position(); },
                     [this](const Eigen::Vector3d & pos) { this->position(pos); }),
                 mc_rtc::gui::Point3D("pos", [this]() { return frame_->position().translation(); }));
}

} // namespace mc_tasks
