#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport eigen.c_eigen as c_eigen
cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_control.c_mc_control as c_mc_control
cimport mc_solver.c_mc_solver as c_mc_solver
cimport mc_tasks.c_mc_tasks as c_mc_tasks

from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr()
    shared_ptr(const shared_ptr[T]&)
    T * get()

cdef extern from "mc_control_fsm_wrapper.hpp":
  # Alias for std::function<void(const mc_rtc::Configuration&)>
  cdef cppclass configure_cb:
    pass
  configure_cb make_configure_cb[T,U](T, U)
  # Alias for std::function<void(Controller&)>
  cdef cppclass controller_cb:
    pass
  controller_cb make_controller_cb[T,U](T, U)
  # Alias for std::function<bool(Controller&)>
  cdef cppclass run_cb:
    pass
  run_cb make_run_cb[T,U](T, U)

cdef extern from "<mc_control/fsm/Controller.h>" namespace "mc_control::fsm":
  cdef cppclass Controller(c_mc_control.MCController):
    shared_ptr[c_mc_tasks.PostureTask] getPostureTask(const string&)
    c_mc_solver.ContactConstraint & contactConstraint()

cdef extern from "<mc_control/fsm/State.h>" namespace "mc_control::fsm":
  cdef cppclass State:
    pass

cdef extern from "<mc_control/fsm/PythonState.h>" namespace "mc_control::fsm":
  cdef cppclass PythonState:
    configure_cb configure_
    run_cb run_
    controller_cb start_
    controller_cb teardown_
    controller_cb stop_
    void output(string)
