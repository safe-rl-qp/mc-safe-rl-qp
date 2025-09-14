#pragma once
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <mc_state_observation/measurements/Sensor.h>
#include <string>

namespace mc_state_observation::measurements
{
/**
 * Object making easier the handling of sensors within the observers.
 **/

/// @brief Class containing the information of an IMU.
struct IMU : public Sensor
{
public:
  inline IMU(int id, std::string name) : Sensor(id, name) {}

public:
  Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();
};
} // namespace mc_state_observation::measurements
