/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_solver/CoMIncPlaneConstr.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/OrientationTask.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

/** Build a cube as a set of planes from a given origin and size */
static std::vector<mc_rbdyn::Plane> makeCube(const Eigen::Vector3d & origin, double size)
{
  auto getPlane = [&](const Eigen::Vector3d & normal, const Eigen::Vector3d & point)
  { return mc_rbdyn::Plane{normal, -normal.dot(origin + point)}; };
  return {getPlane(Eigen::Vector3d{1, 0, 0}, Eigen::Vector3d{-size, 0, 0}),
          getPlane(Eigen::Vector3d{-1, 0, 0}, Eigen::Vector3d{size, 0, 0}),
          getPlane(Eigen::Vector3d{0, 1, 0}, Eigen::Vector3d{0, -size, 0}),
          getPlane(Eigen::Vector3d{0, -1, 0}, Eigen::Vector3d{0, size, 0}),
          getPlane(Eigen::Vector3d{0, 0, 1}, Eigen::Vector3d{0, 0, -size}),
          getPlane(Eigen::Vector3d{0, 0, -1}, Eigen::Vector3d{0, 0, size})};
}

/** Build a cube to show in GUI */
static std::vector<std::vector<Eigen::Vector3d>> makeCubePolygon(const Eigen::Vector3d & origin, double size)
{
  return {// Back-face
          {origin + Eigen::Vector3d{-size, -size, -size}, origin + Eigen::Vector3d{-size, size, -size},
           origin + Eigen::Vector3d{-size, size, size}, origin + Eigen::Vector3d{-size, size, -size}},
          // Front-face
          {origin + Eigen::Vector3d{size, -size, -size}, origin + Eigen::Vector3d{size, size, -size},
           origin + Eigen::Vector3d{size, size, size}, origin + Eigen::Vector3d{size, -size, size}},
          // Left-face
          {origin + Eigen::Vector3d{-size, -size, -size}, origin + Eigen::Vector3d{size, -size, -size},
           origin + Eigen::Vector3d{size, -size, size}, origin + Eigen::Vector3d{-size, -size, size}},
          // Right-face
          {origin + Eigen::Vector3d{-size, size, -size}, origin + Eigen::Vector3d{size, size, -size},
           origin + Eigen::Vector3d{size, size, size}, origin + Eigen::Vector3d{-size, size, size}},
          // Bottom-face
          {origin + Eigen::Vector3d{-size, -size, -size}, origin + Eigen::Vector3d{size, -size, -size},
           origin + Eigen::Vector3d{size, size, -size}, origin + Eigen::Vector3d{size, -size, -size}},
          // Top-face
          {origin + Eigen::Vector3d{-size, -size, size}, origin + Eigen::Vector3d{size, -size, size},
           origin + Eigen::Vector3d{size, size, size}, origin + Eigen::Vector3d{size, -size, size}}};
}

/** The main purpose of this test is to test CoMIncPlaneConstr
 *
 * We first constrain the CoM in a 5x5x5 cm box and target an objective 15 cm below the start position
 *
 * We slowly shrink then expand the box and check the CoM remains there
 */
struct MC_CONTROL_DLLAPI TestCoMInBoxController : public MCController
{
public:
  TestCoMInBoxController(mc_rbdyn::RobotModulePtr rm, double dt, Backend backend) : MCController(rm, dt, backend)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(dynamicsConstraint);
    solver().addTask(postureTask.get());
    addContact({"jvrc1", "ground", "LeftFoot", "AllGround"});
    addContact({"jvrc1", "ground", "RightFoot", "AllGround"});
    /* Create and add the CoM task with the default stiffness/weight */
    comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0);
    solver().addTask(comTask);
    comTask->stiffness(100.0);
    comTask->weight(1e5);
    /** Add an orientation task to keep the chest straight */
    auto chestTask = std::make_shared<mc_tasks::OrientationTask>(robot().frame("WAIST_R_S"));
    // solver().addTask(chestTask);
    /** Create the constraint */
    comConstraint = std::make_shared<mc_solver::CoMIncPlaneConstr>(robots(), 0, solver().dt());
    comConstraint->setInactiveJoints({"NECK_Y", "NECK_R", "NECK_P"});
    //  comConstraint->setActiveJoints({"Root", "R_HIP_P", "R_HIP_R", "R_HIP_Y", "R_KNEE", "R_ANKLE_R", "R_ANKLE_P",
    //                                  "L_HIP_P", "L_HIP_R", "L_HIP_Y", "L_KNEE", "L_ANKLE_R", "L_ANKLE_P", "WAIST_Y",
    //                                  "WAIST_P", "WAIST_R"});
    solver().addConstraintSet(*comConstraint);

    mc_rtc::log::success("Created TestCoMInBoxController");
  }

  bool run() override
  {
    if(!MCController::run()) { return false; }
    nrIter++;
    if(nrIter > 10 && !expanded)
    {
      updateConstraint(0.00005);
      if(size > 0.16) { expanded = true; }
    }
    else if(expanded && !shrinked)
    {
      updateConstraint(-0.00005);
      if(size < 0.02)
      {
        shrinked = true;
        mc_rtc::log::success("Finished at iter: {}", nrIter);
      }
    }
    return true;
  }

  void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    comTask->reset();
    /* Lower the CoM */
    comTask->com(comTask->com() + Eigen::Vector3d(0., 0., -0.15));
    /* Setup the constraint */
    origin = robot().com();
    comConstraint->setPlanes(solver(), makeCube(origin, size));
    /* Draw the box */
    poly = makeCubePolygon(robot().com(), size);
    gui()->addElement({"Constraints", "CoMBox"},
                      mc_rtc::gui::Polygon("polygon", mc_rtc::gui::Color::Red,
                                           [this]() -> const std::vector<std::vector<Eigen::Vector3d>> &
                                           { return poly; }));
  }

  void updateConstraint(double spd)
  {
    size += spd;
    auto planes = makeCube(origin, size);
    std::vector<Eigen::Vector3d> speeds(planes.size());
    std::vector<Eigen::Vector3d> normalDots(planes.size(), Eigen::Vector3d::Zero());
    for(size_t i = 0; i < planes.size(); ++i) { speeds[i] = -planes[i].normal * spd; }
    poly = makeCubePolygon(origin, size);
    comConstraint->setPlanes(solver(), planes, speeds, normalDots);
  }

private:
  unsigned int nrIter = 0;
  std::shared_ptr<mc_tasks::CoMTask> comTask = nullptr;
  std::shared_ptr<mc_solver::CoMIncPlaneConstr> comConstraint = nullptr;
  std::vector<std::vector<Eigen::Vector3d>> poly;
  Eigen::Vector3d origin;
  double size = 0.05;
  bool expanded = false;
  bool shrinked = false;
};

} // namespace mc_control

using Controller = mc_control::TestCoMInBoxController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestCoMInBoxController",
                              Controller(rm, dt, Backend::Tasks),
                              "TestCoMInBoxController_TVM",
                              Controller(rm, dt, Backend::TVM))
