/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/PolygonInterpolator.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>

#include <Eigen/Core>
#include <memory>
#include <sstream>

namespace
{

template<typename T>
struct NoOpDeleter
{
  void operator()(T *) const {}
};

} // namespace

namespace mc_rbdyn
{

std::string CollisionToString(const Collision & c)
{
  std::stringstream ss;
  ss << c;
  return ss.str();
}

typedef std::shared_ptr<mc_rbdyn::RobotModule> RobotModulePtr;

template<typename... Args>
RobotModulePtr get_robot_module(const std::string & name, const Args &... args)
{
  return RobotLoader::get_robot_module(name, args...);
}

std::vector<std::string> available_robots()
{
  return RobotLoader::available_robots();
}

void update_robot_module_path(const std::vector<std::string> & paths)
{
  RobotLoader::update_robot_module_path(paths);
}

void clear_robot_module_path()
{
  RobotLoader::clear();
}

Robots & const_cast_robots(const Robots & rhs)
{
  return const_cast<Robots &>(rhs);
}

Robot & const_cast_robot(const Robot & rhs)
{
  return const_cast<Robot &>(rhs);
}

ForceSensor & const_cast_force_sensor(const ForceSensor & fs)
{
  return const_cast<ForceSensor &>(fs);
}

BodySensor & const_cast_body_sensor(const BodySensor & bs)
{
  return const_cast<BodySensor &>(bs);
}

Surface & const_cast_surface(const Surface & rhs)
{
  return const_cast<Surface &>(rhs);
}

Contact & const_cast_contact(const Contact & rhs)
{
  return const_cast<Contact &>(rhs);
}

std::vector<Contact> & const_cast_contact_vector(const std::vector<Contact> & rhs)
{
  return const_cast<std::vector<Contact> &>(rhs);
}

void contact_vector_set_item(std::vector<Contact> & v, unsigned int idx, const Contact & c)
{
  v[idx] = c;
}

std::shared_ptr<Robots> robots_shared_from_ref(Robots & p)
{
  return p;
}

std::shared_ptr<Robots> robots_make()
{
  return mc_rbdyn::Robots::make();
}

std::shared_ptr<Robots> robots_copy(std::shared_ptr<Robots> robots)
{
  auto ret = mc_rbdyn::Robots::make();
  ret->copy(*robots);
  return ret;
}

PolygonInterpolator * polygonInterpolatorFromTuplePairs(
    const std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> & pairs)
{
  std::vector<PolygonInterpolator::tuple_pair_t> tuple_pairs;
  for(const auto & p : pairs)
  {
    tuple_pairs.push_back({{{p.first.first, p.first.second}}, {{p.second.first, p.second.second}}});
  }
  return new PolygonInterpolator(tuple_pairs);
}

std::vector<double> robotModuleDefaultAttitude(RobotModulePtr rm)
{
  auto attitude = rm->default_attitude();
  std::vector<double> ret(7);
  for(size_t i = 0; i < 7; ++i) { ret[i] = attitude[i]; }
  return ret;
}

template<typename T>
size_t getBodySensorsSize(const T & rm)
{
  return rm.bodySensors().size();
}

template<typename T>
const BodySensor & getBodySensor(const T & rm, size_t i)
{
  return rm.bodySensors()[i];
}

RobotModulePtr copyRobotModule(const RobotModule & rm)
{
  return std::make_shared<RobotModule>(rm);
}

} // namespace mc_rbdyn
