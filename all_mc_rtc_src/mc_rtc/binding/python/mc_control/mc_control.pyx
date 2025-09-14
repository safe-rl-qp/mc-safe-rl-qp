# distutils: language = c++

#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_control.c_mc_control as c_mc_control

cimport eigen.eigen as eigen

cimport sva.c_sva as c_sva
cimport sva.sva as sva

cimport mc_observers.c_mc_observers as c_mc_observers
cimport mc_observers.mc_observers as mc_observers

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport mc_solver.mc_solver as mc_solver

cimport mc_tasks.mc_tasks as mc_tasks

cimport mc_rtc.mc_rtc as mc_rtc
cimport mc_rtc.gui.gui as mc_rtc_gui

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

import warnings

def deprecated():
  warnings.simplefilter('always', category=DeprecationWarning)
  warnings.warn("This call is deprecated", DeprecationWarning)
  warnings.simplefilter('ignore', category=DeprecationWarning)

cdef class ControllerResetData(object):
  def __cinit__(self):
    pass
  property q:
    def __get__(self):
      return self.impl.q

cdef ControllerResetData ControllerResetDataFromPtr(c_mc_control.ControllerResetData * p):
    cdef ControllerResetData ret = ControllerResetData()
    ret.impl = p
    return ret

cdef class Contact(object):
  def __ctor__(self, r1, r2, r1Surface, r2Surface, friction = mc_rbdyn.Contact.defaultFriction, eigen.Vector6d dof = None):
    if isinstance(r1, unicode):
      r1 = r1.encode(u'ascii')
    if isinstance(r1Surface, unicode):
      r1Surface = r1Surface.encode(u'ascii')
    if isinstance(r2, unicode):
      r2 = r2.encode(u'ascii')
    if isinstance(r2Surface, unicode):
      r2Surface = r2Surface.encode(u'ascii')
    if dof is None:
      self.impl = c_mc_control.Contact(r1, r2, r1Surface, r2Surface, friction)
    else:
      self.impl = c_mc_control.Contact(r1, r2, r1Surface, r2Surface, friction, dof.impl)
  def __cinit__(self, *args):
    if len(args) > 0:
      self.__ctor__(*args)
  property r1:
    def __get__(self):
      if self.impl.r1.has_value():
        return self.impl.r1.value()
      else:
        return None
    def __set__(self, r1):
      if isinstance(r1, unicode):
        r1 = r1.encode(u'ascii')
      self.impl.r1 = <string>(r1)
  property r1Surface:
    def __get__(self):
      return self.impl.r1Surface
    def __set__(self, r1Surface):
      if isinstance(r1Surface, unicode):
        r1Surface = r1Surface.encode(u'ascii')
      self.impl.r1Surface = r1Surface
  property r2:
    def __get__(self):
      if self.impl.r2.has_value():
        return self.impl.r2.value()
      else:
        return None
    def __set__(self, r2):
      if isinstance(r2, unicode):
        r2 = r2.encode(u'ascii')
      self.impl.r2 = <string>(r2)
  property r2Surface:
    def __get__(self):
      return self.impl.r2Surface
    def __set__(self, r2Surface):
      if isinstance(r2Surface, unicode):
        r2Surface = r2Surface.encode(u'ascii')
      self.impl.r2Surface = r2Surface
  property friction:
    def __get__(self):
      return self.impl.friction
    def __set__(self, friction):
      self.impl.friction = friction
  property dof:
    def __get__(self):
      return eigen.Vector6dFromC(self.impl.dof)
    def __set__(self, dof):
      if isinstance(dof, eigen.Vector6d):
        self.impl.dof = (<eigen.Vector6d>dof).impl
      else:
        self.dof = eigen.Vector6d(dof)

cdef Contact ContactFromC(const c_mc_control.Contact & c):
  cdef Contact ret = Contact()
  ret.impl = c
  return ret


cdef class MCController(object):
  def __cinit__(self):
    pass
  def run(self):
    return self.base.run()
  def reset(self, ControllerResetData data):
    self.base.reset(deref(data.impl))
  def env(self):
    return mc_rbdyn.RobotFromC(self.base.env())
  def robots(self):
    return mc_rbdyn.RobotsFromRef(self.base.robots())
  def supported_robots(self):
    supported = []
    self.base.supported_robots(supported)
    return supported
  def config(self):
    return mc_rtc.ConfigurationFromRef(self.base.config())
  def logger(self):
    return mc_rtc.LoggerFromRef(self.base.logger())
  def gui(self):
    return mc_rtc_gui.StateBuilderFromShPtr(self.base.gui())
  property timeStep:
    def __get__(self):
      return self.base.timeStep
  property contactConstraint:
    def __get__(self):
      return mc_solver.ContactConstraintFromPtr(self.base.contactConstraint.get())
  property dynamicsConstraint:
    def __get__(self):
      return mc_solver.DynamicsConstraintFromPtr(self.base.dynamicsConstraint.get())
  property kinematicsConstraint:
    def __get__(self):
      return mc_solver.KinematicsConstraintFromPtr(self.base.kinematicsConstraint.get())
  property selfCollisionConstraint:
    def __get__(self):
      return mc_solver.CollisionsConstraintFromPtr(self.base.selfCollisionConstraint.get())
  property postureTask:
    def __get__(self):
      return mc_tasks.PostureTaskFromPtr(self.base.postureTask.get())
  property qpsolver:
    def __get__(self):
      return mc_solver.QPSolverFromRef(self.base.solver())
  def hasObserverPipeline(self, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.base.hasObserverPipeline(name)
  def observerPipeline(self, name = None):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    if name is None:
      return mc_observers.ObserverPipelineFromRef(self.base.observerPipeline())
    else:
      return mc_observers.ObserverPipelineFromRef(self.base.observerPipeline(name))
  def observerPipelines(self):
    it = self.base.observerPipelines().begin()
    end = self.base.observerPipelines().end()
    ret = []
    while it != end:
      ret.append(mc_observers.ObserverPipelineFromRef(deref(it)))
      preinc(it)
    return ret
  def addCollisions(self, r1, r2, collisions):
    assert(all([isinstance(col, mc_rbdyn.Collision) for col in collisions]))
    cdef vector[c_mc_rbdyn.Collision] cols
    if isinstance(r1, unicode):
      r1 = r1.encode(u'ascii')
    if isinstance(r2, unicode):
      r2 = r2.encode(u'ascii')
    for col in collisions:
      cols.push_back((<mc_rbdyn.Collision>col).impl)
    self.base.addCollisions(r1, r2, cols)
  def removeCollisions(self, r1, r2, collisions = None):
    cdef vector[c_mc_rbdyn.Collision] cols
    if isinstance(r1, unicode):
      r1 = r1.encode(u'ascii')
    if isinstance(r2, unicode):
      r2 = r2.encode(u'ascii')
    if collisions is None:
      self.base.removeCollisions(r1, r2)
    else:
      for col in collisions:
        cols.push_back((<mc_rbdyn.Collision>col).impl)
      self.base.removeCollisions(r1, r2, cols)
  def hasRobot(self, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.base.hasRobot(name)
  def robot(self, name = None):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    if name is None:
      return mc_rbdyn.RobotFromC(self.base.robot())
    else:
      return mc_rbdyn.RobotFromC(self.base.robot(name))
  def addContact(self, c, *args):
    if isinstance(c, Contact):
      assert len(args) == 0, "addContact takes either an mc_control.Contact object or arguments for its construction"
      self.base.addContact((<Contact>c).impl)
    else:
      self.addContact(Contact(c, *args))
  def removeContact(self, c, *args):
    if isinstance(c, Contact):
      assert len(args) == 0, "removeContact takes either an mc_control.Contact object or arguments for its construction"
      self.base.removeContact((<Contact>c).impl)
    else:
      self.removeContact(Contact(c, *args))
  def contacts(self):
    cdef c_mc_control.ContactSet cs = self.base.contacts()
    return [ContactFromC(c) for c in cs]
  def hasContact(self, Contact c):
    self.base.hasContact(c.impl)

cdef MCController MCControllerFromPtr(c_mc_control.MCController * p):
    cdef MCController ret = MCController()
    ret.base = p
    return ret

cdef class PythonRWCallback(object):
  def __cinit__(self, succ, out):
    self.impl.success = succ
    self.impl.out = out
  property success:
    def __get__(self):
      return self.impl.success
    def __set__(self, value):
      self.impl.success = value
  property out:
    def __get__(self):
      return self.impl.out
    def __set__(self, value):
      self.impl.out = value

cdef cppbool python_to_run_callback(void * f) except+ with gil:
  return (<object>f).run_callback()

cdef void python_to_reset_callback(const c_mc_control.ControllerResetData & crd, void * f) except+ with gil:
  (<object>f).reset_callback(ControllerResetDataFromPtr(&(c_mc_control.const_cast_crd(crd))))

cdef c_sva.PTransformd python_af_callback(callback, const c_mc_rbdyn.Robot & robot) except+ with gil:
    cdef mc_rbdyn.Robot r = mc_rbdyn.RobotFromC(robot)
    cdef sva.PTransformd af = callback(r)
    return deref(af.impl)

cdef class MCPythonController(MCController):
  AF_CALLBACKS = []
  def __dealloc__(self):
    del self.impl
    self.impl = self.base = NULL
  def __cinit__(self, robot_modules, double dt):
    cdef mc_rbdyn.RobotModuleVector rmv = mc_rbdyn.RobotModuleVector(robot_modules)
    self.impl = self.base = new c_mc_control.MCPythonController(rmv.v, dt)
    try:
      self.run_callback
      c_mc_control.set_run_callback(deref(self.impl), &python_to_run_callback, <void*>(self))
    except AttributeError:
      raise TypeError("You need to implement a run_callback method in your object")
    try:
      self.reset_callback
      c_mc_control.set_reset_callback(deref(self.impl), &python_to_reset_callback, <void*>(self))
    except AttributeError:
      pass
  def addAnchorFrameCallback(self, name, callback):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    MCPythonController.AF_CALLBACKS.append(callback)
    c_mc_control.add_anchor_frame_callback(deref(self.impl), <string>(name), &python_af_callback, callback)
  def removeAnchorFrameCallback(self, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    c_mc_control.remove_anchor_frame_callback(deref(self.impl), name)

cdef class MCGlobalController(object):
  def __dealloc__(self):
    del self.impl
    self.impl = NULL
  def __cinit_simple__(self):
    self.impl = new c_mc_control.MCGlobalController()
  def __cinit_conf__(self, conf):
    if isinstance(conf, unicode):
      conf = conf.encode(u'ascii')
    self.impl = new c_mc_control.MCGlobalController(conf)
  def __cinit_full__(self, conf, mc_rbdyn.RobotModule rm):
    if isinstance(conf, unicode):
      conf = conf.encode(u'ascii')
    self.impl = new c_mc_control.MCGlobalController(conf, rm.impl)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.__cinit_simple__()
    elif len(args) == 1:
      self.__cinit_conf__(args[0])
    elif len(args) == 2:
      self.__cinit_full__(args[0], args[1])
    else:
      raise TypeError("Wrong arguments passed to MCGlobalController ctor")
  def init(self, q, pos = None):
    cdef c_mc_control.array7d p = c_mc_control.array7d()
    if pos is None:
      self.impl.init(q)
    else:
      assert(len(pos) == 7)
      for i,pi in enumerate(pos):
        p[i] = pos[i]
      self.impl.init(q, p)
  def setSensorPosition(self, eigen.Vector3d p):
    self.impl.setSensorPosition(p.impl)
  def setSensorOrientation(self, eigen.Quaterniond q):
    self.impl.setSensorOrientation(q.impl)
  def setSensorLinearVelocity(self, eigen.Vector3d lv):
    self.impl.setSensorLinearVelocity(lv.impl)
  def setSensorAngularVelocity(self, eigen.Vector3d av):
    self.impl.setSensorAngularVelocity(av.impl)
  def setSensorAcceleration(self, eigen.Vector3d a):
    deprecated()
    self.impl.setSensorLinearAcceleration(a.impl)
  def setSensorLinearAcceleration(self, eigen.Vector3d a):
    self.impl.setSensorLinearAcceleration(a.impl)
  def setEncoderValues(self, q):
    self.impl.setEncoderValues(q)
  def setEncoderVelocities(self, alpha):
    self.impl.setEncoderVelocities(alpha)
  def setJointTorques(self, tau):
    self.impl.setJointTorques(tau)
  def setWrenches(self, wrenchesIn):
    cdef cppmap[string, c_sva.ForceVecd] wrenches = cppmap[string, c_sva.ForceVecd]()
    for sensor,w in wrenchesIn.iteritems():
      if not isinstance(w, sva.ForceVecd):
        w = sva.ForceVecd(w)
      wrenches[sensor] = deref((<sva.ForceVecd>(w)).impl)
    self.impl.setWrenches(wrenches)

  def run(self):
    return self.impl.run()

  def timestep(self):
    return self.impl.timestep()
  def controller(self):
    return MCControllerFromPtr(&(self.impl.controller()))
  def ref_joint_order(self):
    return self.impl.ref_joint_order()
  def robot(self):
    return mc_rbdyn.RobotFromC(self.impl.robot())
  property running:
    def __get__(self):
      return self.impl.running
    def __set__(self, b):
      self.impl.running = b

cdef class ElementId(object):
  def __cinit__(self, category, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    self.impl = c_mc_control.ElementId([s.encode(u'ascii') if isinstance(s, unicode) else s for s in category] , name)
  property category:
    def __get__(self):
      return self.impl.category
  property name:
    def __get__(self):
      return self.impl.name

cdef class ControllerClient(object):
  def __cinit__(self, sub_conn_uri, push_conn_uri, timeout = 0.0):
    if isinstance(sub_conn_uri, unicode):
      sub_conn_uri = sub_conn_uri.encode(u'ascii')
    if isinstance(push_conn_uri, unicode):
      push_conn_uri = push_conn_uri.encode(u'ascii')
    self.impl = new c_mc_control.ControllerClient(sub_conn_uri, push_conn_uri, timeout)
  def send_request(self, element_id, data = None):
    if data is None:
      deref(self.impl).send_request((<ElementId>element_id).impl)
    else:
      deref(self.impl).send_request((<ElementId>element_id).impl, deref((<mc_rtc.Configuration>data).impl))
