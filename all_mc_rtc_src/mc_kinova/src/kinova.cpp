#include "kinova.h"

#include "config.h"

#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

inline static std::string kinovaVariant(bool callib, bool use_bota)
{
  if(callib)
  {
    mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_callib'");
    return "kinova_callib";
  }
  else if(not callib)
  {
    if(use_bota)
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_bota");
      return "kinova_bota";
    }
    else
    {
      mc_rtc::log::info("KinovaRobotModule uses the kinova variant: 'kinova_default'");
      return "kinova_default";
    }
  }
  mc_rtc::log::error_and_throw("KinovaRobotModule does not provide this kinova variant ...");
  return "";
}

KinovaRobotModule::KinovaRobotModule(bool callib, bool use_bota)
: mc_rbdyn::RobotModule(KINOVA_DESCRIPTION_PATH, kinovaVariant(callib, use_bota))
{
  mc_rtc::log::success("KinovaRobotModule loaded with name: {}", name);
  if(use_bota)
  {
    urdf_path = KINOVA_URDF_PATH_BOTA;
  }
  else
  {
    urdf_path = KINOVA_URDF_PATH;
  }
  _real_urdf = urdf_path;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, true));

  rsdf_dir = KINOVA_RSDF_DIR + "/" + kinovaVariant(callib, use_bota) + "/";
  mc_rtc::log::success("KinovaRobotModule using path \"{}\" for rsdf", rsdf_dir);

  // Override position, velocity and effort bounds
  auto update_joint_limit = [this](const std::string & name, double limit_low, double limit_up)
  {
    assert(limit_up > 0);
    assert(limit_low < 0);
    assert(_bounds[0].at(name).size() == 1);
    _bounds[0].at(name)[0] = limit_low;
    _bounds[1].at(name)[0] = limit_up;
  };

  if(callib)
  {
    update_joint_limit("joint_2", -2.15, 2.15);
    update_joint_limit("joint_4", -2.45, 0.45);
    update_joint_limit("joint_5", -3.14, 3.14);
    update_joint_limit("joint_6", -2.0, 2.0);
    update_joint_limit("joint_7", -3.14, 3.14);
  }
  else
  {
    update_joint_limit("joint_2", -2.15, 2.15);
    update_joint_limit("joint_4", -2.45, 2.45);
    update_joint_limit("joint_6", -2.0, 2.0);
  }

  auto update_velocity_limit = [this](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(_bounds[2].at(name).size() == 1);
    _bounds[2].at(name)[0] = -limit;
    _bounds[3].at(name)[0] = limit;
  };
  update_velocity_limit("joint_1", 2.0944);
  update_velocity_limit("joint_2", 2.0944);
  update_velocity_limit("joint_3", 2.0944);
  update_velocity_limit("joint_3", 2.0944);
  update_velocity_limit("joint_4", 2.0944);
  update_velocity_limit("joint_5", 3.049);
  update_velocity_limit("joint_6", 3.049);
  update_velocity_limit("joint_7", 3.049);
  auto update_torque_limit = [this](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(_bounds[4].at(name).size() == 1);
    _bounds[4].at(name)[0] = -limit;
    _bounds[5].at(name)[0] = limit;
  };
  update_torque_limit("joint_1", 95);
  update_torque_limit("joint_2", 95);
  update_torque_limit("joint_3", 95);
  update_torque_limit("joint_4", 95);
  update_torque_limit("joint_5", 45);
  update_torque_limit("joint_6", 45);
  update_torque_limit("joint_7", 45);

  auto set_gear_ratio = [this](const std::string & name, double gr)
  {
    assert(gr > 0);
    mb.setJointGearRatio(mb.jointIndexByName(name), gr);
  };

  set_gear_ratio("joint_1", 100);
  set_gear_ratio("joint_2", 100);
  set_gear_ratio("joint_3", 100);
  set_gear_ratio("joint_4", 100);
  set_gear_ratio("joint_5", 100);
  set_gear_ratio("joint_6", 100);
  set_gear_ratio("joint_7", 100);

  auto set_rotor_inertia = [this](const std::string & name, double ir)
  {
    assert(ir > 0);
    mb.setJointRotorInertia(mb.jointIndexByName(name), ir);
  };

  double power = pow(10, -4);

  set_rotor_inertia("joint_1", (double)0.40 * power);
  set_rotor_inertia("joint_2", (double)0.40 * power);
  set_rotor_inertia("joint_3", (double)0.40 * power);
  set_rotor_inertia("joint_4", (double)0.40 * power);
  set_rotor_inertia("joint_5", (double)0.22 * power);
  set_rotor_inertia("joint_6", (double)0.22 * power);
  set_rotor_inertia("joint_7", (double)0.22 * power);

  // Automatically load the convex hulls associated to each body
  std::string convexPath = KINOVA_CONVEX_DIR + "/kinova/";
  mc_rtc::log::success("KinovaRobotModule using path \"{}\" for convex", convexPath);
  bfs::path p(convexPath);
  if(bfs::exists(p) && bfs::is_directory(p))
  {
    std::vector<bfs::path> files;
    std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
    for(const bfs::path & file : files)
    {
      size_t off = file.filename().string().rfind("-ch.txt");
      if(off != std::string::npos)
      {
        std::string name = file.filename().string();
        name.replace(off, 7, "");
        _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
      }
    }
  }

  // Add JointSensors for temperature/current logging
  for(size_t i = 0; i < _ref_joint_order.size(); ++i)
  {
    if(mb.jointIndexByName().count(_ref_joint_order[i]) != 0)
    {
      _jointSensors.push_back(mc_rbdyn::JointSensor(_ref_joint_order[i]));
    }
  }

  // Define a force sensor
  _forceSensors.push_back(mc_rbdyn::ForceSensor("EEForceSensor", "FT_sensor_wrench", sva::PTransformd::Identity()));
  // Define an IMU
  _bodySensors.push_back(mc_rbdyn::BodySensor("Accelerometer", "FT_sensor_imu", sva::PTransformd::Identity()));
  // Define a device sensor for external torque measurment
  // _devices.push_back(mc_rbdyn::ExternalTorqueSensor("externalTorqueSensor", 7).clone());
  // _devices.push_back(mc_rbdyn::VirtualTorqueSensor("virtualTorqueSensor", 7).clone());
  _devices.push_back(mc_rbdyn::VirtualTorqueSensor("ExtTorquesVirtSensor", 7).clone());

  // Clear body sensors
  _bodySensors.clear();

  const double i = 0.03;
  const double s = 0.015;
  const double d = 0.;
  // Define a minimal set of self-collisions
  _minimalSelfCollisions = {{"base_link", "spherical_wrist_1_link", i, s, d},
                            {"shoulder_link", "spherical_wrist_1_link", i, s, d},
                            {"half_arm_1_link", "spherical_wrist_1_link", i, s, d},
                            {"half_arm_2_link", "spherical_wrist_1_link", i, s, d},
                            {"base_link", "spherical_wrist_2_link", i, s, d},
                            {"shoulder_link", "spherical_wrist_2_link", i, s, d},
                            {"half_arm_1_link", "spherical_wrist_2_link", i, s, d},
                            {"half_arm_2_link", "spherical_wrist_2_link", i, s, d},
                            {"base_link", "bracelet_link", i, s, d},
                            {"shoulder_link", "bracelet_link", i, s, d},
                            {"half_arm_1_link", "bracelet_link", i, s, d},
                            {"half_arm_2_link", "bracelet_link", i, s, d},
                            {"base_link", "FT_adapter", i, s, d},
                            {"shoulder_link", "FT_adapter", i, s, d},
                            {"half_arm_1_link", "FT_adapter", i, s, d},
                            {"half_arm_2_link", "FT_adapter", i, s, d},
                            {"base_link", "FT_sensor_mounting", i, s, d},
                            {"shoulder_link", "FT_sensor_mounting", i, s, d},
                            {"half_arm_1_link", "FT_sensor_mounting", i, s, d},
                            {"half_arm_2_link", "FT_sensor_mounting", i, s, d}};

  /* Additional self collisions */

  _commonSelfCollisions = _minimalSelfCollisions;

  // Define simple grippers
  // _grippers = {{"l_gripper", {"L_UTHUMB"}, true}, {"r_gripper", {"R_UTHUMB"}, false}};

  // Default configuration of the floating base
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.0}};

  // Default joint configuration, if a joint is omitted the configuration is 0 or the middle point of the limit range if
  // 0 is not a valid configuration
  _stance["joint_1"] = {0.0};
  _stance["joint_2"] = {0.2618};
  _stance["joint_3"] = {3.14};
  _stance["joint_4"] = {-2.269};
  _stance["joint_5"] = {0.0};
  _stance["joint_6"] = {0.959878729};
  _stance["joint_7"] = {1.57};

  mc_rtc::log::success("KinovaRobotModule uses urdf_path {}", urdf_path);
}

} // namespace mc_robots
