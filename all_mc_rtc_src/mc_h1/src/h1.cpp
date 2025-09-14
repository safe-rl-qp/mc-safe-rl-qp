#include "h1.h"
#include "config.h"

#include <mc_rbdyn/VirtualTorqueSensor.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

H1RobotModule::H1RobotModule()
: RobotModule(mc_rtc::H1_DESCRIPTION_PATH,
              "h1",
              std::string{mc_rtc::H1_DESCRIPTION_PATH} + std::string{"/urdf/h1.urdf"})
{
  rsdf_dir = std::string(mc_rtc::H1_DESCRIPTION_PATH) + "/rsdf";

  // True if the robot has a fixed base, false otherwise
  bool fixed = false;
  // Makes all the basic initialization that can be done from an URDF file
  init(rbd::parsers::from_urdf_file(urdf_path, fixed));

#if 0
  // Build _convexHull, but conflict primitives defined in h1.urdf
  bfs::path convexPath = bfs::path(path) / "convex/h1";
  for(const auto & b : mb.bodies())
  {
    bfs::path ch = convexPath / (b.name() + ".txt");
    if(bfs::exists(ch))
    {
      _convexHull[b.name()] = {b.name(), ch.string()};
    }
  }
#endif

  _ref_joint_order = {
      "left_hip_yaw_joint",        "left_hip_roll_joint",      "left_hip_pitch_joint", "left_knee_joint",
      "left_ankle_joint",          "right_hip_yaw_joint",      "right_hip_roll_joint", "right_hip_pitch_joint",
      "right_knee_joint",          "right_ankle_joint",        "torso_joint",          "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",  "left_shoulder_yaw_joint",  "left_elbow_joint",     "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint"};

  // Override position, velocity and effort bounds
  auto update_joint_limit = [this](const std::string & name, double limit_low, double limit_up)
  {
    assert(limit_up > 0);
    assert(limit_low < 0);
    assert(_bounds[0].at(name).size() == 1);
    _bounds[0].at(name)[0] = limit_low;
    _bounds[1].at(name)[0] = limit_up;
  };
  auto update_velocity_limit = [this](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(_bounds[2].at(name).size() == 1);
    _bounds[2].at(name)[0] = -limit;
    _bounds[3].at(name)[0] = limit;
  };
  auto update_torque_limit = [this](const std::string & name, double limit)
  {
    assert(limit > 0);
    assert(_bounds[4].at(name).size() == 1);
    _bounds[4].at(name)[0] = -limit;
    _bounds[5].at(name)[0] = limit;
  };

  // update_joint_limit("left_hip_yaw_joint", -0.43, 0.43);
  // update_velocity_limit("left_hip_yaw_joint", 23.0);
  // update_torque_limit("left_hip_yaw_joint", 200.0);
  // update_joint_limit("left_hip_roll_joint", -0.43, 0.43);
  // update_velocity_limit("left_hip_roll_joint", 23.0);
  // update_torque_limit("left_hip_roll_joint", 200.0);
  // update_joint_limit("left_hip_pitch_joint", -3.14, 2.53);
  // update_velocity_limit("left_hip_pitch_joint", 23.0);
  // update_torque_limit("left_hip_pitch_joint", 200.0);
  // update_joint_limit("left_knee_joint", -0.26, 2.05);
  // update_velocity_limit("left_knee_joint", 14.0);
  // update_torque_limit("left_knee_joint", 300.0);
  // update_joint_limit("left_ankle_joint", -0.87, 0.52);
  // update_velocity_limit("left_ankle_joint", 9.0);
  // update_torque_limit("left_ankle_joint", 40.0);

  // update_joint_limit("right_hip_yaw_joint", -0.43, 0.43);
  // update_velocity_limit("right_hip_yaw_joint", 23.0);
  // update_torque_limit("right_hip_yaw_joint", 200.0);
  // update_joint_limit("right_hip_roll_joint", -0.43, 0.43);
  // update_velocity_limit("right_hip_roll_joint", 23.0);
  // update_torque_limit("right_hip_roll_joint", 200.0);
  // update_joint_limit("right_hip_pitch_joint", -3.14, 2.53);
  // update_velocity_limit("right_hip_pitch_joint", 23.0);
  // update_torque_limit("right_hip_pitch_joint", 200.0);
  // update_joint_limit("right_knee_joint", -0.26, 2.05);
  // update_velocity_limit("right_knee_joint", 14.0);
  // update_torque_limit("right_knee_joint", 300.0);
  // update_joint_limit("right_ankle_joint", -0.87, 0.52);
  // update_velocity_limit("right_ankle_joint", 9.0);
  // update_torque_limit("right_ankle_joint", 40.0);

  // update_joint_limit("torso_joint", -2.35, 2.35);
  // update_velocity_limit("torso_joint", 23.0);
  // update_torque_limit("torso_joint", 200.0);

  // update_joint_limit("left_shoulder_pitch_joint", -2.87, 2.87);
  // update_velocity_limit("left_shoulder_pitch_joint", 9.0);
  // update_torque_limit("left_shoulder_pitch_joint", 40.0);
  // update_joint_limit("left_shoulder_roll_joint", -0.34, 3.11);
  // update_velocity_limit("left_shoulder_roll_joint", 9.0);
  // update_torque_limit("left_shoulder_roll_joint", 40.0);
  // update_joint_limit("left_shoulder_yaw_joint", -1.3, 4.45);
  // update_velocity_limit("left_shoulder_yaw_joint", 20.0);
  // update_torque_limit("left_shoulder_yaw_joint", 18.0);
  // update_joint_limit("left_elbow_joint", -1.25, 2.61);
  // update_velocity_limit("left_elbow_joint", 20.0);
  // update_torque_limit("left_elbow_joint", 18.0);

  // update_joint_limit("right_shoulder_pitch_joint", -2.87, 2.87);
  // update_velocity_limit("right_shoulder_pitch_joint", 9.0);
  // update_torque_limit("right_shoulder_pitch_joint", 40.0);
  // update_joint_limit("right_shoulder_roll_joint", -3.11, 0.34);
  // update_velocity_limit("right_shoulder_roll_joint", 9.0);
  // update_torque_limit("right_shoulder_roll_joint", 40.0);
  // update_joint_limit("right_shoulder_yaw_joint", -4.45, 1.3);
  // update_velocity_limit("right_shoulder_yaw_joint", 20.0);
  // update_torque_limit("right_shoulder_yaw_joint", 18.0);
  // update_joint_limit("right_elbow_joint", -1.25, 2.61);
  // update_velocity_limit("right_elbow_joint", 20.0);
  // update_torque_limit("right_elbow_joint", 18.0);
  //

  update_joint_limit("left_hip_yaw_joint", -0.48, 0.48);
  update_velocity_limit("left_hip_yaw_joint", 23.0);
  update_torque_limit("left_hip_yaw_joint", 150.0);
  update_joint_limit("left_hip_roll_joint", -0.35, 0.35);
  update_velocity_limit("left_hip_roll_joint", 23.0);
  update_torque_limit("left_hip_roll_joint", 150.0);
  update_joint_limit("left_hip_pitch_joint", -2.58, 2.58);
  update_velocity_limit("left_hip_pitch_joint", 23.0);
  update_torque_limit("left_hip_pitch_joint", 150.0);
  update_joint_limit("left_knee_joint", 0.05, 2.1);
  update_velocity_limit("left_knee_joint", 14.0);
  update_torque_limit("left_knee_joint", 150.0);
  update_joint_limit("left_ankle_joint", -0.92, 0.57);
  update_velocity_limit("left_ankle_joint", 9.0);
  update_torque_limit("left_ankle_joint", 30.0);

  update_joint_limit("right_hip_yaw_joint", -0.48, 0.48);
  update_velocity_limit("right_hip_yaw_joint", 23.0);
  update_torque_limit("right_hip_yaw_joint", 150.0);
  update_joint_limit("right_hip_roll_joint", -0.35, 0.35);
  update_velocity_limit("right_hip_roll_joint", 23.0);
  update_torque_limit("right_hip_roll_joint", 150.0);
  update_joint_limit("right_hip_pitch_joint", -2.58, 2.58);
  update_velocity_limit("right_hip_pitch_joint", 23.0);
  update_torque_limit("right_hip_pitch_joint", 150.0);
  update_joint_limit("right_knee_joint", 0.05, 2.1);
  update_velocity_limit("right_knee_joint", 14.0);
  update_torque_limit("right_knee_joint", 150.0);
  update_joint_limit("right_ankle_joint", -0.92, 0.57);
  update_velocity_limit("right_ankle_joint", 9.0);
  update_torque_limit("right_ankle_joint", 30.0);

  update_joint_limit("torso_joint", -0.35, 0.35);
  update_velocity_limit("torso_joint", 23.0);
  update_torque_limit("torso_joint", 150.0);

  update_joint_limit("left_shoulder_pitch_joint", -1.05, 1.05);
  update_velocity_limit("left_shoulder_pitch_joint", 9.0);
  update_torque_limit("left_shoulder_pitch_joint", 30.0);
  update_joint_limit("left_shoulder_roll_joint", -0.39, 0.39);
  update_velocity_limit("left_shoulder_roll_joint", 9.0);
  update_torque_limit("left_shoulder_roll_joint", 30.0);
  update_joint_limit("left_shoulder_yaw_joint", -1.35, 1.35);
  update_velocity_limit("left_shoulder_yaw_joint", 20.0);
  update_torque_limit("left_shoulder_yaw_joint", 30.0);
  update_joint_limit("left_elbow_joint", -1.30, 1.30);
  update_velocity_limit("left_elbow_joint", 20.0);
  update_torque_limit("left_elbow_joint", 30.0);

  update_joint_limit("right_shoulder_pitch_joint", -1.05, 1.05);
  update_velocity_limit("right_shoulder_pitch_joint", 9.0);
  update_torque_limit("right_shoulder_pitch_joint", 30.0);
  update_joint_limit("right_shoulder_roll_joint", -0.39, 0.39);
  update_velocity_limit("right_shoulder_roll_joint", 9.0);
  update_torque_limit("right_shoulder_roll_joint", 30.0);
  update_joint_limit("right_shoulder_yaw_joint", -1.35, 1.35);
  update_velocity_limit("right_shoulder_yaw_joint", 20.0);
  update_torque_limit("right_shoulder_yaw_joint", 30.0);
  update_joint_limit("right_elbow_joint", -1.30, 1.30);
  update_velocity_limit("right_elbow_joint", 20.0);
  update_torque_limit("right_elbow_joint", 30.0);

  using namespace mc_rtc::constants;
  _stance["left_hip_yaw_joint"] = {0.0};
  _stance["left_hip_roll_joint"] = {0.0};
  _stance["left_hip_pitch_joint"] = {-0.2};
  _stance["left_knee_joint"] = {0.6};
  _stance["left_ankle_joint"] = {-0.4};
  _stance["right_hip_yaw_joint"] = {0.0};
  _stance["right_hip_roll_joint"] = {0.0};
  _stance["right_hip_pitch_joint"] = {-0.2};
  _stance["right_knee_joint"] = {0.6};
  _stance["right_ankle_joint"] = {-0.4};
  _stance["torso_joint"] = {0.0};
  _stance["left_shoulder_pitch_joint"] = {0.4};
  _stance["left_shoulder_roll_joint"] = {0.0};
  _stance["left_shoulder_yaw_joint"] = {0.0};
  _stance["left_elbow_joint"] = {-0.4};
  _stance["right_shoulder_pitch_joint"] = {0.4};
  _stance["right_shoulder_roll_joint"] = {0.0};
  _stance["right_shoulder_yaw_joint"] = {0.0};
  _stance["right_elbow_joint"] = {-0.4};

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.98}};

  // Add JointSensors for temperature/current logging
  for(size_t i = 0; i < _ref_joint_order.size(); ++i)
  {
    if(mb.jointIndexByName().count(_ref_joint_order[i]) != 0)
    {
      _jointSensors.push_back(mc_rbdyn::JointSensor(_ref_joint_order[i]));
    }
  }

  // Sensors
  _bodySensors.clear();
  _bodySensors.emplace_back("Accelerometer", "pelvis", sva::PTransformd(Eigen::Vector3d(-0.04452, -0.01891, 0.27756)));
  _bodySensors.emplace_back("FloatingBase", "pelvis", sva::PTransformd::Identity());

  _devices.push_back(mc_rbdyn::VirtualTorqueSensor("ExtTorquesVirtSensor", 25).clone());

  _minimalSelfCollisions = {mc_rbdyn::Collision("torso_link", "left_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "right_shoulder_yaw_link", 0.02, 0.001, 0.),
                            mc_rbdyn::Collision("torso_link", "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("torso_link", "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis", "left_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis", "right_shoulder_yaw_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis", "left_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("pelvis", "right_elbow_link", 0.05, 0.03, 0.),
                            mc_rbdyn::Collision("left_hip_pitch_link", "right_hip_pitch_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_knee_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_link", "right_ankle_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("left_ankle_link", "right_knee_link", 0.02, 0.01, 0.),
                            mc_rbdyn::Collision("right_ankle_link", "left_knee_link", 0.02, 0.01, 0.)};
  _commonSelfCollisions = _minimalSelfCollisions;
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"H1"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    ROBOT_MODULE_CHECK_VERSION("H1")
    if(n == "H1")
    {
      return new mc_robots::H1RobotModule();
    }
    else
    {
      mc_rtc::log::error("H1 module Cannot create an object of type {}", n);
      return nullptr;
    }
  }
}
