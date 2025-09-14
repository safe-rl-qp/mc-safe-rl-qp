/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

namespace mc_rbdyn
{

/**
 * @brief Configuration for mc_rbdyn::RobotConverter
 */
struct MC_RBDYN_DLLAPI RobotConverterConfig
{
  ///< Copy input robot's mbc to their corresponding output robot's mbc
  bool mbcToOutMbc_ = true;
  /**
   * Chosse which mbc properties to copy, only effective if mbcToOutMbc_ = true
   * @{
   */
  bool copyJointCommand_ = true;
  bool copyJointVelocityCommand_ = true;
  bool copyJointAccelerationCommand_ = true;
  bool copyJointTorqueCommand_ = true;
  ///< @}

  ///< Copy input encoder values to the output robot's mbc
  bool encodersToOutMbc_ = false;
  ///< Copy input encoder values to the output robot's mbc on the first run
  bool encodersToOutMbcOnce_ = true;
  ///< Compute the output robot's joint mimics
  bool enforceMimics_ = true;

  /* Copy the input robot's position in the world into the output
   *
   * This also triggers the robot's kinematic update (outputRobot.forwardKinematics()), if this option is false the
   * update must be done outside of the robot converter if the accurate position of the robot is needed
   */
  bool copyPosWorld_ = true;

#define ROBOT_CONVERTER_PROPERTY(NAME)                \
  inline RobotConverterConfig & NAME(bool b) noexcept \
  {                                                   \
    NAME##_ = b;                                      \
    return *this;                                     \
  }

  ROBOT_CONVERTER_PROPERTY(mbcToOutMbc)
  ROBOT_CONVERTER_PROPERTY(copyJointCommand)
  ROBOT_CONVERTER_PROPERTY(copyJointVelocityCommand)
  ROBOT_CONVERTER_PROPERTY(copyJointAccelerationCommand)
  ROBOT_CONVERTER_PROPERTY(copyJointTorqueCommand)
  ROBOT_CONVERTER_PROPERTY(encodersToOutMbc)
  ROBOT_CONVERTER_PROPERTY(encodersToOutMbcOnce)
  ROBOT_CONVERTER_PROPERTY(enforceMimics)
  ROBOT_CONVERTER_PROPERTY(copyPosWorld)

#undef ROBOT_CONVERTER_PROPERTY
};

} // namespace mc_rbdyn
