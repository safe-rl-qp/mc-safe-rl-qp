#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_solver.c_mc_solver as c_mc_solver

from libcpp cimport bool as cppbool

cdef class ConstraintSet(object):
  cdef c_mc_solver.ConstraintSet * cs_base

cdef class ContactConstraint(ConstraintSet):
  cdef c_mc_solver.ContactConstraint * impl
  cdef cppbool own_impl__

cdef ContactConstraint ContactConstraintFromPtr(c_mc_solver.ContactConstraint *)

cdef class KinematicsConstraint(ConstraintSet):
  cdef c_mc_solver.KinematicsConstraint * impl
  cdef cppbool own_impl__

cdef KinematicsConstraint KinematicsConstraintFromPtr(c_mc_solver.KinematicsConstraint *)

cdef class DynamicsConstraint(KinematicsConstraint):
  cdef c_mc_solver.DynamicsConstraint * d_impl

cdef DynamicsConstraint DynamicsConstraintFromPtr(c_mc_solver.DynamicsConstraint *)

cdef class CollisionsConstraint(ConstraintSet):
  cdef c_mc_solver.CollisionsConstraint * impl
  cdef cppbool own_impl__

cdef CollisionsConstraint CollisionsConstraintFromPtr(c_mc_solver.CollisionsConstraint*)

cdef class QPSolver(object):
  cdef c_mc_solver.QPSolver * impl
  cdef cppbool own_impl__

cdef QPSolver QPSolverFromPtr(c_mc_solver.QPSolver*)

cdef QPSolver QPSolverFromRef(c_mc_solver.QPSolver&)
