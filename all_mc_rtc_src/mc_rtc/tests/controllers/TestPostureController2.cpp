/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif
#include <mc_control/api.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

struct MC_CONTROL_DLLAPI TestPostureController2 : public MCController
{
public:
  TestPostureController2(mc_rbdyn::RobotModulePtr rm, double dt) : MCController(rm, dt)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    qpsolver->addConstraintSet(contactConstraint);
    qpsolver->addConstraintSet(kinematicsConstraint);
    postureTask->stiffness(20);
    qpsolver->addTask(postureTask.get());
    qpsolver->setContacts({});
    BOOST_CHECK(robot().hasJoint("NECK_P"));
    BOOST_CHECK_NO_THROW(head_joint_index = robot().jointIndexByName("NECK_P"));
    head_joint_target = std::min(std::abs(robot().ql()[head_joint_index][0]), robot().qu()[head_joint_index][0]) - 0.1;
    mc_rtc::log::success("Created TestPostureController");
  }

  virtual bool run() override
  {
    bool ret = MCController::run();
    BOOST_CHECK(ret);
    if(nrIter++ > 250) { BOOST_CHECK_SMALL(robot().mbc().q[head_joint_index][0] - head_joint_target, 0.05); }
    return ret;
  }

  virtual void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    postureTask->target({{"NECK_P", {head_joint_target}}});
  }

private:
  unsigned int nrIter = 0;
  unsigned int head_joint_index;
  double head_joint_target;
};

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("TestPostureController2", mc_control::TestPostureController2)
