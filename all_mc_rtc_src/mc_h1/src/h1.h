#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI H1RobotModule : public mc_rbdyn::RobotModule
{
  H1RobotModule();
};

} // namespace mc_robots
