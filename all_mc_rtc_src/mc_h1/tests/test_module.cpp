#include <mc_rbdyn/RobotLoader.h>

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    mc_rtc::log::critical("Usage: {} [MODULE_DIR]", argv[0]);
    return 1;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({argv[1]});
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("H1");
  mc_rtc::log::info("H1 has {} dof", rm->mb.nrDof());
  return (rm) ? 0 : 1;
}
