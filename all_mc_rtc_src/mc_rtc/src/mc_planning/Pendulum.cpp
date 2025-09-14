#include <mc_planning/Pendulum.h>
#include <mc_rtc/constants.h>

namespace mc_planning
{
namespace constants = mc_rtc::constants;

Pendulum::Pendulum() {}

Pendulum::Pendulum(double lambda,
                   const Eigen::Vector3d & com,
                   const Eigen::Vector3d & comd,
                   const Eigen::Vector3d & comdd)
{
  reset(lambda, com, comd, comdd);
}

void Pendulum::reset(double lambda,
                     const Eigen::Vector3d & com,
                     const Eigen::Vector3d & comd,
                     const Eigen::Vector3d & comdd)
{
  com_ = com;
  comd_ = comd;
  comdd_ = comdd;
  comddd_ = Eigen::Vector3d::Zero();
  omega_ = std::sqrt(lambda);
  zmp_ = com - (constants::gravity + comdd) / lambda;
  zmpd_ = comd_ - comddd_ / lambda;
}

void Pendulum::resetCoMHeight(double height, const Eigen::Vector3d & p, const Eigen::Vector3d & n)
{
  com_ += (height + n.dot(p - com_)) * n;
  comd_ -= n.dot(comd_) * n;
  comdd_ -= n.dot(comdd_) * n;
  comddd_ -= n.dot(comddd_) * n;
  omega_ = std::sqrt(constants::GRAVITY / height);
}

void Pendulum::integrateIPM(Eigen::Vector3d zmp, double lambda, double dt)
{
  Eigen::Vector3d com_prev = com_;
  Eigen::Vector3d comd_prev = comd_;
  omega_ = std::sqrt(lambda);
  zmp_ = zmp;

  Eigen::Vector3d vrp = zmp_ + constants::gravity / lambda;
  double ch = std::cosh(omega_ * dt);
  double sh = std::sinh(omega_ * dt);
  comdd_ = lambda * (com_prev - zmp_) - constants::gravity;
  comd_ = comd_prev * ch + omega_ * (com_prev - vrp) * sh;
  com_ = com_prev * ch + comd_prev * sh / omega_ - vrp * (ch - 1.0);

  // default values for third-order terms
  comddd_ = Eigen::Vector3d::Zero();
  zmpd_ = comd_ - comddd_ / lambda;
}

void Pendulum::integrateCoMJerk(const Eigen::Vector3d & comddd, double dt)
{
  com_ += dt * (comd_ + dt * (comdd_ / 2 + dt * (comddd / 6)));
  comd_ += dt * (comdd_ + dt * (comddd / 2));
  comdd_ += dt * comddd;
  comddd_ = comddd;
}

void Pendulum::completeIPM(const Eigen::Vector3d & p, const Eigen::Vector3d & n)
{
  auto gravitoInertial = -(constants::gravity + comdd_);
  double lambda = n.dot(gravitoInertial) / n.dot(p - com_);
  zmp_ = com_ + gravitoInertial / lambda;
  zmpd_ = comd_ - comddd_ / lambda;
  omega_ = std::sqrt(lambda);
}
} // namespace mc_planning
