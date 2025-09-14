#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_tasks.c_mc_tasks as c_mc_tasks

cimport tasks.qp.c_qp as c_qp

from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr()
    shared_ptr(T*)
    T* get()
    T& operator*()

cdef class MetaTask(object):
  cdef c_mc_tasks.MetaTask * mt_base

cdef class PostureTask(MetaTask):
  cdef c_mc_tasks.PostureTask * impl
  cdef cppbool own_impl__

cdef PostureTask PostureTaskFromPtr(c_mc_tasks.PostureTask *)

cdef class _CoMTrajectoryTask(MetaTask):
  cdef c_mc_tasks.TrajectoryTaskGeneric * ttg_base

cdef class _PositionTrajectoryTask(MetaTask):
  cdef c_mc_tasks.TrajectoryTaskGeneric * ttg_base

cdef class _OrientationTrajectoryTask(MetaTask):
  cdef c_mc_tasks.TrajectoryTaskGeneric * ttg_base

cdef class _VectorOrientationTrajectoryTask(MetaTask):
  cdef c_mc_tasks.TrajectoryTaskGeneric * ttg_base

cdef class _SurfaceTransformTask(MetaTask):
  cdef c_mc_tasks.TrajectoryTaskGeneric * ttg_base

cdef class _SplineTrajectoryTask(MetaTask):
  cdef c_mc_tasks.TrajectoryTaskGeneric * ttg_base

cdef class BSplineTrajectoryTask(_SplineTrajectoryTask):
  cdef c_mc_tasks.BSplineTrajectoryTask * impl
  cdef cppbool own_impl__

cdef class ExactCubicTrajectoryTask(_SplineTrajectoryTask):
  cdef c_mc_tasks.ExactCubicTrajectoryTask * impl
  cdef cppbool own_impl__

cdef class _TransformTask(MetaTask):
  cdef c_mc_tasks.TrajectoryTaskGeneric * ttg_base

cdef class CoMTask(_CoMTrajectoryTask):
  cdef c_mc_tasks.CoMTask * impl
  cdef cppbool own_impl__

cdef class PositionTask(_PositionTrajectoryTask):
  cdef c_mc_tasks.PositionTask * impl
  cdef cppbool own_impl__

cdef PositionTask PositionTaskFromSharedPtr(shared_ptr[c_mc_tasks.PositionTask])

cdef class OrientationTask(_OrientationTrajectoryTask):
  cdef c_mc_tasks.OrientationTask * impl
  cdef cppbool own_impl__

cdef OrientationTask OrientationTaskFromSharedPtr(shared_ptr[c_mc_tasks.OrientationTask])

cdef class VectorOrientationTask(_VectorOrientationTrajectoryTask):
  cdef c_mc_tasks.VectorOrientationTask * impl
  cdef cppbool own_impl__

cdef class EndEffectorTask(MetaTask):
  cdef c_mc_tasks.EndEffectorTask * impl
  cdef cppbool own_impl__

cdef class RelativeEndEffectorTask(EndEffectorTask):
  pass

cdef class SurfaceTransformTask(_SurfaceTransformTask):
  cdef c_mc_tasks.SurfaceTransformTask * impl
  cdef cppbool own_impl__

ctypedef fused AnyTTG:
  CoMTask
  PositionTask
  OrientationTask
  VectorOrientationTask
  SurfaceTransformTask

#Note : In recent versions of Cython, fused types can be fused and thus
# AnyTask can simply be the fusion of AnyTTG and other tasks
ctypedef fused AnyTask:
  PostureTask
  CoMTask
  PositionTask
  OrientationTask
  VectorOrientationTask
  SurfaceTransformTask
  EndEffectorTask
  RelativeEndEffectorTask
  BSplineTrajectoryTask
  ExactCubicTrajectoryTask
