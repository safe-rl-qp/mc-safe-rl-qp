/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/TVMQPSolver.h>

#include <mc_tvm/Robot.h>

#include <tvm/ControlProblem.h>
#include <tvm/hint/internal/DiagonalCalculator.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/VelocityDamper.h>

namespace mc_solver
{

struct TVMKinematicsConstraint
{
  const mc_rbdyn::Robot & robot_;
  std::array<double, 3> damper_;
  std::array<double, 5> damperSecond_; // Second order dynamics dampers
  double velocityPercent_;
  std::vector<tvm::TaskWithRequirementsPtr> constraints_;
  std::vector<tvm::TaskWithRequirementsPtr> mimics_constraints_;

  TVMKinematicsConstraint(const mc_rbdyn::Robot & robot, const std::array<double, 3> & damper, double vp);
  TVMKinematicsConstraint(const mc_rbdyn::Robot & robot, const std::array<double, 5> & damperSecond, double vp);

  void addToSolver(mc_solver::TVMQPSolver & solver);

  void removeFromSolver(mc_solver::TVMQPSolver & solver);
};

} // namespace mc_solver
