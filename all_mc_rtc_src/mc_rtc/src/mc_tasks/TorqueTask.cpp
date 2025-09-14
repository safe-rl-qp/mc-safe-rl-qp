/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/TorqueTask.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_solver/TVMQPSolver.h>
// #include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/TorqueFunction.h>

#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/NumberSlider.h>

namespace mc_tasks
{

namespace details
{

inline static mc_rtc::void_ptr_caster<mc_tvm::TorqueFunction> tvm_error{};

struct TVMTorqueTask : public TrajectoryTaskGeneric
{
  TVMTorqueTask(const mc_rbdyn::Robots & robots,
                unsigned int robotIndex,
                double weight,
                bool compensateExternalForces = false)
  : TrajectoryTaskGeneric(robots, robotIndex, 0, weight)
  {
    finalize<Backend::TVM, mc_tvm::TorqueFunction>(robots.robot(robotIndex), compensateExternalForces);
    type_ = "torque";
    name_ = std::string("torque_") + robots.robot(robotIndex).name();
    isTorqueTask_ = true;
  }

  void compensateExternalForces(bool compensate) { tvm_error(errorT)->compensateExternalForces(compensate); }

  bool isCompensatingExternalForces() const { return tvm_error(errorT)->isCompensatingExternalForces(); }

  void update(mc_solver::QPSolver & solver) override { TrajectoryTaskGeneric::update(solver); }

  void torque(const std::vector<std::vector<double>> & p) { tvm_error(errorT)->torque(p); }
};

} // namespace details

// inline static mc_rtc::void_ptr_caster<tasks::qp::TorqueTask> tasks_error{};
inline static mc_rtc::void_ptr_caster<details::TVMTorqueTask> tvm_error{};

inline static mc_rtc::void_ptr make_error(MetaTask::Backend backend,
                                          const mc_solver::QPSolver & solver,
                                          unsigned int rIndex,
                                          double weight)
{
  switch(backend)
  {
    // case MetaTask::Backend::Tasks:
    //   return mc_rtc::make_void_ptr<tasks::qp::TorqueTask>(solver.robots().mbs(), static_cast<int>(rIndex),
    //                                                        solver.robot(rIndex).mbc().tau, weight);
    case MetaTask::Backend::TVM:
      return mc_rtc::make_void_ptr<details::TVMTorqueTask>(solver.robots(), rIndex, weight);
    default:
      mc_rtc::log::error_and_throw("[TorqueTask] Not implemented for solver backend: {}", backend);
  }
}

TorqueTask::TorqueTask(const mc_solver::QPSolver & solver,
                       unsigned int rIndex,
                       double weight,
                       bool compensateExternalForces)
: robots_(solver.robots()), rIndex_(rIndex), pt_(make_error(backend_, solver, rIndex, weight)), dt_(solver.dt())
{
  compensateExternalForces_ = compensateExternalForces;
  eval_ = this->eval();
  speed_ = Eigen::VectorXd::Zero(eval_.size());
  torque_vector_ = Eigen::VectorXd::Zero(eval_.size());
  type_ = "torque";
  name_ = std::string("torque_") + robots_.robot(rIndex_).name();
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.isMimic())
    {
      mimics_[j.mimicName()].push_back(static_cast<int>(robots_.robot(rIndex_).jointIndexByName(j.name())));
    }
  }
  reset();
}

void TorqueTask::reset()
{
  torque(robots_.robot(rIndex_).mbc().jointTorque);
}

void TorqueTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("torque")) { this->torque(config("torque")); }
  if(config.has("target")) { this->target(config("target")); }
  if(config.has("weight")) { this->weight(config("weight")); }
  if(config.has("jointWeights")) { this->jointWeights(config("jointWeights")); }
}

void TorqueTask::dimWeight(const Eigen::VectorXd & dimW)
{
  switch(backend_)
  {
    // case Backend::Tasks:
    //   tasks_error(pt_)->dimWeight(dimW);
    //   break;
    case Backend::TVM:
      tvm_error(pt_)->dimWeight(dimW);
      break;
    default:
      break;
  }
}

Eigen::VectorXd TorqueTask::dimWeight() const
{
  switch(backend_)
  {
    // case Backend::Tasks:
    //   return tasks_error(pt_)->dimWeight();
    case Backend::TVM:
      return tvm_error(pt_)->dimWeight();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void TorqueTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                    const std::vector<std::string> & activeJointsName,
                                    const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex_), activeJointsName, "[" + name() + "::selectActiveJoints]");
  std::vector<std::string> unactiveJoints = {};
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.dof() && std::find(activeJointsName.begin(), activeJointsName.end(), j.name()) == activeJointsName.end())
    {
      unactiveJoints.push_back(j.name());
    }
  }
  selectUnactiveJoints(solver, unactiveJoints);
}

void TorqueTask::selectUnactiveJoints(mc_solver::QPSolver &,
                                      const std::vector<std::string> & unactiveJointsName,
                                      const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  ensureHasJoints(robots_.robot(rIndex_), unactiveJointsName, "[" + name() + "::selectUnActiveJoints]");
  Eigen::VectorXd dimW = dimWeight();
  dimW.setOnes();
  const auto & robot = robots_.robot(rIndex_);
  auto dofOffset = [&robot, this]()
  {
    switch(backend_)
    {
      // case Backend::Tasks:
      //   return 0;
      case Backend::TVM:
        return robot.mb().joint(0).dof();
      default:
        mc_rtc::log::error_and_throw("Not implemented in backend {}", backend_);
    }
  }();
  for(const auto & j : unactiveJointsName)
  {
    auto jIndex = static_cast<int>(robot.jointIndexByName(j));
    const auto & joint = robot.mb().joint(jIndex);
    if(joint.dof() == 6) { continue; }
    auto dofIndex = robot.mb().jointPosInDof(jIndex) - dofOffset;
    dimW.segment(dofIndex, joint.dof()).setZero();
  }
  dimWeight(dimW);
}

void TorqueTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  selectUnactiveJoints(solver, {});
}

Eigen::VectorXd TorqueTask::eval() const
{
  switch(backend_)
  {
    // case Backend::Tasks:
    // {
    //   auto & pt = *tasks_error(pt_);
    //   return pt.dimWeight().asDiagonal() * pt.eval();
    // }
    case Backend::TVM:
      return tvm_error(pt_)->eval();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

Eigen::VectorXd TorqueTask::speed() const
{
  return speed_;
}

void TorqueTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(inSolver_) { return; }
  inSolver_ = true;
  switch(backend_)
  {
    // case Backend::Tasks:
    //   tasks_solver(solver).addTask(tasks_error(pt_));
    //   break;
    case Backend::TVM:
      MetaTask::addToSolver(*tvm_error(pt_), solver);
      break;
    default:
      break;
  }
}

void TorqueTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_) { return; }
  inSolver_ = false;
  switch(backend_)
  {
    // case Backend::Tasks:
    //   tasks_solver(solver).removeTask(tasks_error(pt_));
    //   break;
    case Backend::TVM:
      MetaTask::removeFromSolver(*tvm_error(pt_), solver);
      break;
    default:
      break;
  }
}

void TorqueTask::update(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    // case Backend::Tasks:
    // {
    //   const auto & pt = *tasks_error(pt_);
    //   speed_ = pt.dimWeight().asDiagonal() * (pt.eval() - eval_) / dt_;
    //   eval_ = pt.eval();
    //   break;
    // }
    case Backend::TVM:
    {
      auto & pt = *tvm_error(pt_);
      pt.update(solver);
      speed_ = (pt.eval() - eval_) / dt_;
      eval_ = pt.dimWeight().asDiagonal() * pt.eval();
      break;
    }
    default:
      break;
  }
}

void TorqueTask::torque(const std::vector<std::vector<double>> & tau)
{
  torque_ = tau;

  switch(backend_)
  {
    // case Backend::Tasks:
    //   tasks_error(pt_)->torque(p);
    //   break;
    case Backend::TVM:
      tvm_error(pt_)->torque(tau);
      break;
    default:
      break;
  }
}

std::vector<std::vector<double>> TorqueTask::torque() const
{
  return torque_;
}

void TorqueTask::weight(double w)
{
  switch(backend_)
  {
    // case Backend::Tasks:
    //   tasks_error(pt_)->weight(w);
    //   break;
    case Backend::TVM:
      tvm_error(pt_)->weight(w);
      break;
    default:
      break;
  }
}

double TorqueTask::weight() const
{
  switch(backend_)
  {
    // case Backend::Tasks:
    //   return tasks_error(pt_)->weight();
    case Backend::TVM:
      return tvm_error(pt_)->weight();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

bool TorqueTask::inSolver() const
{
  return inSolver_;
}

void TorqueTask::compensateExternalForces(bool compensate)
{
  switch(backend_)
  {
    case Backend::TVM:
      tvm_error(pt_)->compensateExternalForces(compensate);
      break;
    default:
      mc_rtc::log::error_and_throw("Compensating external forces is only supported in TVM backend");
  }
}

bool TorqueTask::isCompensatingExternalForces() const
{
  switch(backend_)
  {
    case Backend::TVM:
      return tvm_error(pt_)->isCompensatingExternalForces();
    default:
      mc_rtc::log::error_and_throw("Compensating external forces is only supported in TVM backend");
  }
}

void TorqueTask::jointWeights(const std::map<std::string, double> & jws)
{
  Eigen::VectorXd dimW = dimWeight();
  const auto & robot = robots_.robot(rIndex_);
  const auto & mb = robot.mb();
  for(const auto & jw : jws)
  {
    if(robot.hasJoint(jw.first))
    {
      auto jIndex = mb.jointIndexByName(jw.first);
      if(mb.joint(jIndex).dof() > 0) { dimW[mb.jointPosInDof(jIndex)] = jw.second; }
      // No warning, it's probably over specified
    }
    else
    {
      mc_rtc::log::warning("[TorqueTask] No joint named {} in {}, joint weight will have no effect", jw.first,
                           robot.name());
    }
  }
  dimWeight(dimW);
}

void TorqueTask::target(const std::map<std::string, std::vector<double>> & joints)
{
  auto tau = torque();

  for(const auto & j : joints)
  {
    if(robots_.robot(rIndex_).hasJoint(j.first))
    {
      if(static_cast<size_t>(
             robots_.robot(rIndex_).mb().joint(static_cast<int>(robots_.robot(rIndex_).jointIndexByName(j.first))).dof())
         == j.second.size())
      {
        tau[robots_.robot(rIndex_).jointIndexByName(j.first)] = j.second;
        if(mimics_.count(j.first))
        {
          for(auto ji : mimics_.at(j.first))
          {
            const auto & mimic = robots_.robot(rIndex_).mb().joint(ji);
            if(static_cast<size_t>(mimic.dof()) == j.second.size())
            {
              for(unsigned i = 0; i < j.second.size(); i++)
              {
                tau[static_cast<size_t>(ji)][i] = mimic.mimicMultiplier() * j.second[i] + mimic.mimicOffset();
              }
            }
          }
        }
      }
      else { mc_rtc::log::error("TorqueTask::target dof missmatch for {}", j.first); }
    }
  }
  int pos = 0;
  if(robots_.robot(rIndex_).mb().nrJoints() > 0 && robots_.robot(rIndex_).mb().joint(0).type() == rbd::Joint::Free)
  {
    pos = 6; // Skip the floating base joints
  }
  int j0_ = robots_.robot(rIndex_).mb().joint(0).type() == rbd::Joint::Free ? 1 : 0;
  for(int jI = j0_; jI < robots_.robot(rIndex_).mb().nrJoints(); ++jI)
  {
    auto jIdx = static_cast<size_t>(jI);
    const auto & j = robots_.robot(rIndex_).mb().joint(jI);
    if(j.dof() == 1) // prismatic or revolute
    {
      torque_vector_[pos] = tau[jIdx][0];
      pos++;
    }
  }
  torque(tau);
}

void TorqueTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_eval", this, [this]() { return eval(); });
  logger.addLogEntry(name_ + "_speed", this, [this]() -> const Eigen::VectorXd & { return speed_; });
  logger.addLogEntry(name_ + "_torque", this, [this]() -> const Eigen::VectorXd & { return torque_vector_; });
}

void TorqueTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput(
                     "weight", [this]() { return this->weight(); }, [this](const double & w) { this->weight(w); }));
  std::vector<std::string> active_gripper_joints;
  for(const auto & g : robots_.robot(rIndex_).grippers())
  {
    for(const auto & n : g.get().activeJoints()) { active_gripper_joints.push_back(n); }
  }
  auto isActiveGripperJoint = [&](const std::string & j)
  { return std::find(active_gripper_joints.begin(), active_gripper_joints.end(), j) != active_gripper_joints.end(); };
  for(const auto & j : robots_.robot(rIndex_).mb().joints())
  {
    if(j.dof() != 1 || j.isMimic() || isActiveGripperJoint(j.name())) { continue; }
    auto jIndex = robots_.robot(rIndex_).jointIndexByName(j.name());
    bool isContinuous = robots_.robot(rIndex_).ql()[jIndex][0] == -std::numeric_limits<double>::infinity();
    auto updatePosture = [this](unsigned int jIndex, double v)
    {
      this->torque_[jIndex][0] = v;
      const auto & jName = robots_.robot(rIndex_).mb().joint(static_cast<int>(jIndex)).name();
      if(mimics_.count(jName))
      {
        for(auto ji : mimics_.at(jName))
        {
          const auto & mimic = robots_.robot(rIndex_).mb().joint(ji);
          this->torque_[static_cast<size_t>(ji)][0] = mimic.mimicMultiplier() * v + mimic.mimicOffset();
        }
      }
      torque(torque_);
    };
    if(isContinuous)
    {
      gui.addElement({"Tasks", name_, "Target"}, mc_rtc::gui::NumberInput(
                                                     j.name(), [this, jIndex]() { return this->torque_[jIndex][0]; },
                                                     [jIndex, updatePosture](double v) { updatePosture(jIndex, v); }));
    }
    else
    {
      gui.addElement({"Tasks", name_, "Target"},
                     mc_rtc::gui::NumberSlider(
                         j.name(), [this, jIndex]() { return this->torque_[jIndex][0]; },
                         [jIndex, updatePosture](double v) { updatePosture(jIndex, v); },
                         robots_.robot(rIndex_).ql()[jIndex][0], robots_.robot(rIndex_).qu()[jIndex][0]));
    }
  }
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "torque",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "torque");
      auto t = std::make_shared<mc_tasks::TorqueTask>(solver, robotIndex, config("weight", 10.));
      t->load(solver, config);
      return t;
    });
} // namespace
