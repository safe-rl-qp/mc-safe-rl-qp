/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/api.h>
#include <mc_control/fsm/states/api.h>

#include <mc_solver/ConstraintSet.h>

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/PostureTask.h>

#include <mc_rtc/Configuration.h>

namespace mc_control
{

namespace fsm
{

struct MC_CONTROL_FSM_DLLAPI Controller;

/** \class State
 *
 * A state of an FSM.
 *
 * A state implementation should create, add and remove the tasks/constraints
 * it needs, with the following exceptions:
 * - Contacts are handled at the global level, the state should go through the
 *   addContact/removeContact methods of FSMController
 * - Collision constraints are handled at the global level, the state should go
 *   through the addCollisons/removeCollisions methods of Controller
 * - Kinematics/Dynamics constraints are handled at the global level
 * - Posture tasks are handled at the global level, if a state removes a
 *   posture task from the solver, it should put it back afterwards
 *
 * Notably, a state should not keep constraints or tasks it created active
 * after the state finished its execution.
 *
 * Every methods can be called in a real-time context. You should be aware of
 * that, especially if you plan to have threads in your state (e.g. monitoring
 * a ROS topic, waiting for data on the network...) then you should be
 * especially careful when your state is destroyed.
 *
 * The life-span of a state is:
 * - the state is constructed and init is called (iteration 0)
 * - run(...) is called until it returns true (iteration 1 to N)
 * - teardown(...) is called and the state is destructed (iteration N, after
 *   run last call)
 *
 * The state might be interrupted (e.g. emergency behaviour triggered) in which
 * case, stopped(...) will be called. The default implementation does nothing
 * as teardown(...) is called anyway.
 *
 */
struct MC_CONTROL_FSM_DLLAPI State
{
  virtual ~State() {}

  /** Common implementation, handles the following options:
   *
   * - AddContacts/RemoveContacts: add and remove contacts during the state's
   *   start
   * - AddContactsAfter/RemoveContactsAfter: add and remove contacts during the
   *   state's teardown
   * - AddCollisions/RemoveCollisions: add and remove collisions during the state's
   *   start
   * - AddCollisionsAfter/RemoveCollisionsAfter: add and remove collisions during the
   *   state's teardown
   * - RemovePostureTask: if true, remove the robot posture task at the state's
   *   start
   */
  void configure_(const mc_rtc::Configuration & config);

  /** Common implementation, takes care of common options */
  void start_(Controller & ctl);

  /** Common implementation, takes care of common options */
  void teardown_(Controller & ctl);

  /** Called every iteration until it returns true */
  virtual bool run(Controller & ctl) = 0;

  /** Called if the state is interrupted */
  virtual void stop(Controller &) {}

  /** Handle read service call */
  virtual bool read_msg(std::string &) { return false; }

  /** Handle read/write service call */
  virtual bool read_write_msg(std::string &, std::string &) { return false; }

  /** Returns the output of the state, should only be consulted once run has
   * returned true */
  const std::string & output() const noexcept { return output_; }

  /** Returns the name of the state */
  const std::string & name() { return name_; }

  void name(const std::string & n) { name_ = n; }

protected:
  /** Output setter for derived classes */
  void output(const std::string & o) { output_ = o; }

  /** Called to configure the state.
   *
   * This is called multiple times:
   * - once for every level of state inheritance
   * - once with the executor configuration, this is the FSM global executor,
   *   the Meta state executor, the Parallel state or any state that handle
   *   other states
   *
   * The default implementation simply loads the provided configuration into
   * the `config_` protected members. You can override this behavior to
   * implement a more complex loading logic.
   *
   * You can access this configuration either via the `config_` variable
   */
  virtual void configure(const mc_rtc::Configuration & config);

  /** Called before the state starts being run
   *
   * This will be called only once with the state fully configured.
   *
   */
  virtual void start(Controller & ctl) = 0;

  /** Called right before destruction */
  virtual void teardown(Controller & ctl) = 0;

protected:
  /** AddContacts in the configuration */
  mc_rtc::Configuration add_contacts_config_;
  /** RemoveContacts in the configuration */
  mc_rtc::Configuration remove_contacts_config_;
  /** AddContactsAfter in the configuration */
  mc_rtc::Configuration add_contacts_after_config_;
  /** RemoveContactsAfter in the configuration */
  mc_rtc::Configuration remove_contacts_after_config_;
  /** AddCollisions in the configuration */
  mc_rtc::Configuration add_collisions_config_;
  /** RemoveCollisions in the configuration */
  mc_rtc::Configuration remove_collisions_config_;
  /** AddCollisionsAfter in the configuration */
  mc_rtc::Configuration add_collisions_after_config_;
  /** RemoveCollisionsAfter in the configuration */
  mc_rtc::Configuration remove_collisions_after_config_;
  /** constraints in the configuration */
  mc_rtc::Configuration constraints_config_;
  /** tasks in the configuration */
  mc_rtc::Configuration tasks_config_;
  /** RemovePostureTask in the configuration */
  mc_rtc::Configuration remove_posture_task_;
  /** Configuration obtained through several calls */
  mc_rtc::Configuration config_;

  /** Constraints managed by the state if any */
  std::vector<mc_solver::ConstraintSetPtr> constraints_;
  /** Tasks managed by the state if any */
  std::vector<std::pair<mc_tasks::MetaTaskPtr, mc_rtc::Configuration>> tasks_;
  /** Posture tasks that were removed by this state */
  std::vector<mc_tasks::PostureTaskPtr> postures_;

private:
  std::string name_ = "";
  std::string output_ = "";
};

using StatePtr = std::shared_ptr<State>;

} // namespace fsm

} // namespace mc_control

/* The following macros are used to simplify the required symbol exports */

#ifndef MC_RTC_BUILD_STATIC

#  ifdef WIN32
#    define FSM_STATE_API __declspec(dllexport)
#  else
#    if __GNUC__ >= 4
#      define FSM_STATE_API __attribute__((visibility("default")))
#    else
#      define FSM_STATE_API
#    endif
#  endif

#  define EXPORT_SINGLE_STATE(NAME, TYPE)                                   \
    extern "C"                                                              \
    {                                                                       \
      FSM_STATE_API void MC_RTC_FSM_STATE(std::vector<std::string> & names) \
      {                                                                     \
        names = {NAME};                                                     \
      }                                                                     \
      FSM_STATE_API void destroy(mc_control::fsm::State * ptr)              \
      {                                                                     \
        delete ptr;                                                         \
      }                                                                     \
      FSM_STATE_API mc_control::fsm::State * create(const std::string &)    \
      {                                                                     \
        return new TYPE();                                                  \
      }                                                                     \
    }

#else

#  define EXPORT_SINGLE_STATE(NAME, TYPE)                                                              \
    namespace                                                                                          \
    {                                                                                                  \
    static auto registered = []()                                                                      \
    {                                                                                                  \
      using fn_t = std::function<TYPE *()>;                                                            \
      mc_control::fsm::Controller::factory().register_object(NAME, fn_t([]() { return new TYPE(); })); \
      return true;                                                                                     \
    }();                                                                                               \
    }

#endif
