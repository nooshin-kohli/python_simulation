# -*- coding: utf-8 -*-
"""
Created on Mon Aug 29 16:31:20 2016

@author: mshahbazi
"""

import numpy as np
import rbdl

from Centauro_ImportModel import BodyClass2d, JointClass2d
# import matplotlib.pyplot as plt
# from scipy.optimize import root, approx_fprime, minimize, fminbound
import scipy.integrate as integrate


# import time
# import subprocess


class Centauro_RobotClass(object):

    def __init__(self, t, q, qdot, p, u, dt, lua_file, param=None, terrain=None):
        """
        This is Centauro robot class
        """
        self.name = 'Centauro'
        self.param = param

        self.fb_dim = 3
        self.point_F_dim = 2

        self.model = rbdl.loadModel(lua_file)
        self.body = BodyClass2d()
        self.joint = JointClass2d()
        if param is None:
            self.body.l_end = 10
        else:
            self.body.l_end = self.param.l3h

        self.qdim = self.model.q_size
        self.S = np.hstack((np.zeros((self.qdim - self.fb_dim, self.fb_dim)), \
                            np.eye(self.qdim - self.fb_dim)))

        self.t = np.array([t])

        if q.any():
            self.q = np.array(q)  # states
        else:
            self.q = np.zeros((1, self.qdim))
        if qdot.any():
            self.qdot = np.array(qdot)  # states
        else:
            self.qdot = np.zeros((1, self.qdim))
        self.__p = list(p)  # contact feet
        if u.any():
            self.u = np.array(u)  # joint inputs
        else:
            self.u = np.zeros((1, self.qdim - self.fb_dim))
        self.cforce = []

        self.dt = dt

        self.terrain = terrain

        self.Jc = self.Jc_from_cpoints( \
            self.model, self.q[-1, :], self.body, self.__p[-1])  # TODO

        self.M = self.CalcM(self.model, self.q[-1, :])
        self.h = self.Calch(self.model, self.q[-1, :], self.qdot[-1, :])

        #        self.qqdot0 = np.concatenate((self.q[-1,:], self.qdot[-1, :]))
        #        print self.qqdot0.shape
        #        self.ForwardDynamics(self.qqdot0, self.M, self.h, self.S, \
        #        self.u[-1, :], self.Jc, self.__p[-1])
        #
        #        self.cforce.append(self.Lambda)

        self.total_mass = sum([self.model.mBodies[i].mMass \
                               for i in range(self.fb_dim, self.model.previously_added_body_id + 1)])

        self.foot_pose_h = 0
        self.foot_pose_f = 0

    def __call__(self):
        """
        executes hybrid system
        """
        self.t0 = self.t[-1]
        self.qqdot0 = np.concatenate((self.q[-1, :], self.qdot[-1, :])). \
            reshape(1, self.qdim * 2)
        self.qqdot0forRefine = self.qqdot0[-1, :].copy()
        self.__p0 = self.__p[-1]

        if not hasattr(self, 'u0'): self.u0 = np.zeros_like(self.u[-1, :])

        #        self.ComputeContactForce(self.qqdot0forRefine, self.__p0, self.u0)
        #        self.cforce.append(self.Lambda)

        #        self.qqdot0 = integrate.odeint(self.__dyn, self.qqdot0[-1,:], \
        #        np.array([0,self.dt]))

        dy = self.RK4(self.dyn_RK4)
        self.qqdot0 += dy(self.t0, self.qqdot0[-1, :], self.dt).reshape(1, self.qdim * 2)

        #        print "Event detection is avoided (devel)"
        #        indexes = []

        self.ev_i = None
        self.evt = list(self.__evts())
        self.ev = np.array([self.evt[i](self.t0 + self.dt, \
                                        self.qqdot0[-1, :]) for i in range(len(self.evt))])

        if self.ev[-1] == 0: raise ValueError('Simulation was terminated because:\
        one of the conditions in StopSimulation() is meet.')

        indexes = [i for i, ev in enumerate(self.ev) if ev is not None and ev > 0]

        if indexes:
            print("##########################")
            print("index of the occurred events: ", indexes)
            print("at the nominal time: ", self.t0)
            print("\n")

            #            print self.ev
            tr_list, qqdot0_list, index_list = [], [], []
            for i in indexes:
                if i in [4, 5]:
                    tr, qqdot0 = self.t0, self.qqdot0forRefine.reshape(1, self.qdim * 2)
                    # TODO: tr , qqdot0 = self.interpl(self.evt[i])
                else:
                    tr, qqdot0 = self.refine(self.evt[i])
                tr_list.append(tr);
                qqdot0_list.append(qqdot0);
                index_list.append(i)

            index = np.argmin(tr_list)
            print("the one that is applied: ", indexes[index])
            print("at the real time: ", tr_list[index])
            print("##########################\n\n")

            self.trefined, self.qqdot0 = tr_list[index], qqdot0_list[index]
            self.ev_i = index_list[index]

            self.qqdot0, self.__p0 = self.__trans()

        self.__AppendState()

        return None

    def ComputeContactForce(self, qqdot, p, u):

        q = qqdot[:self.qdim]
        qdot = qqdot[self.qdim:]

        Jc = self.Jc_from_cpoints(self.model, q, self.body, p)

        M = self.CalcM(self.model, q)

        h = self.Calch(self.model, q, qdot)

        self.ForwardDynamics(qqdot.flatten(), M, h, self.S, u, Jc, p)

        return None

    def __AppendState(self):
        self.q = np.append(self.q, [self.qqdot0[-1, :self.qdim]], axis=0)
        self.qdot = np.append(self.qdot, [self.qqdot0[-1, self.qdim:]], axis=0)
        self.__p.append(self.__p0)
        self.u = np.append(self.u, [self.u0.flatten()], axis=0)
        if self.ev_i is not None:
            self.t = np.append(self.t, [self.trefined], axis=0)
        else:
            self.t = np.append(self.t, [self.t0 + self.dt], axis=0)

        self.cforce.append(self.Lambda)
        return None

    def RK4(self, f):
        return lambda t, y, dt: (
            lambda dy1: (
                lambda dy2: (
                    lambda dy3: (
                        lambda dy4: (dy1 + 2 * dy2 + 2 * dy3 + dy4) / 6
                    )(dt * f(t + dt, y + dy3))
                )(dt * f(t + dt / 2, y + dy2 / 2))
            )(dt * f(t + dt / 2, y + dy1 / 2))
        )(dt * f(t, y))

    def __dyn(self, x, t):
        """
        .dyn  evaluates system dynamics
        """
        q = x[:self.qdim]
        qd = x[self.qdim:]

        self.M = self.CalcM(self.model, q)
        self.Jc = self.Jc_from_cpoints(self.model, q, self.body, self.__p0)
        self.h = self.Calch(self.model, q, qd)

        self.ForwardDynamics(x, self.M, self.h, self.S, self.u0, self.Jc, self.__p0)

        dx = np.concatenate((qd, self.qddot.flatten()))

        return dx

    def dyn_RK4(self, t, x):
        """
        .dyn  evaluates system dynamics
        """
        return self.__dyn(x, t)

    def ForwardDynamics(self, x, M, h, S, tau, Jc, cpoints):
        fdim = np.shape(Jc)[0]
        qdim = self.qdim
        q = x[:qdim]
        qdot = x[qdim:]

        if fdim == 0:

            self.qddot = np.dot(np.linalg.inv(M), np.dot(S.T, self.u0) - h).flatten()
            self.Lambda = np.zeros(fdim) * np.nan
        else:

            if np.nonzero(qdot)[0].any() and Jc.any():
                #                tic = time.time()
                #                gamma = self.CalcGamma(cpoints, q, qdot)
                gamma = self.CalcGamma(cpoints, q, qdot)
            #                print gamma - mygamma
            #                toc = time.time() - tic
            #                print toc
            else:
                gamma = - np.dot(np.zeros_like(Jc), qdot)

            aux1 = np.hstack((M, -Jc.T))
            aux2 = np.hstack((Jc, np.zeros((fdim, fdim))))
            A = np.vstack((aux1, aux2))

            B = np.vstack(((np.dot(S.T, tau) - h).reshape(qdim, 1), \
                           gamma.reshape(fdim, 1)))

            res = np.dot(np.linalg.inv(A), B).flatten()

            self.qddot = res[:-fdim]

            self.SetGRF(cpoints, res[-fdim:])

        #            print "======================================="
        #            print 'p, lambda:', self.__p[-1], self.Lambda
        #            print "======================================="

        return None

    def SetGRF(self, p, values):
        #        print 'yes', p
        last = 0
        if 1 in p:
            p_1 = last
            last += self.point_F_dim
        if 2 in p:
            p_2 = last
            last += self.point_F_dim
        #        if 3 in p:
        #            p_3 = last
        #            last += 3
        #        if 4 in p:
        #            p_4 = last
        #            last += 3
        self.Lambda = np.zeros(2 * self.point_F_dim) * np.nan
        if 1 in p:
            self.Lambda[:self.point_F_dim] = values[p_1:p_1 + self.point_F_dim]
        if 2 in p:
            self.Lambda[self.point_F_dim:2 * self.point_F_dim] = \
                values[p_2:p_2 + self.point_F_dim]
        #        if 3 in p:
        #            self.Lambda[6:9] = values[p_3:p_3+3]
        #        if 4 in p:
        #            self.Lambda[9:] = values[p_4:p_4+3]
        return None

    def computeJacobian23(self, body_part, swapped=False):
        q = self.q[-1, :]
        if body_part == 'h':
            th2 = q[2] + q[3]
            th3 = q[4]
            l1 = self.param.l2h
            l2 = self.param.l3h
        elif body_part == 'f':
            th2 = q[2] + q[5] + q[6]
            th3 = q[7]
            l1 = self.param.l2f
            l2 = self.param.l3f

        if swapped:
            temp = l1;
            l1 = l2;
            l2 = temp
            th2 += + th3 - np.pi
            th3 = - th3

        return self.computeJacobianRR(th2, th3, l1, l2)

    def computeJacobianRR(self, th1, th2, l1, l2):
        s1 = np.sin(th1)
        c1 = np.cos(th1)
        s12 = np.sin(th1 + th2)
        c12 = np.cos(th1 + th2)

        J = np.zeros((2, 2))
        J[0, 0] = -(l1 * s1 + l2 * s12)
        J[0, 1] = -(l2 * s12)
        J[1, 0] = (l1 * c1 + l2 * c12)
        J[1, 1] = (l2 * c12)
        return J

    def CalcM(self, model, q):
        M = np.zeros((model.q_size, model.q_size))
        rbdl.CompositeRigidBodyAlgorithm(model, q, M, True)
        return M

    def Calch(self, model, q, qdot):
        h = np.zeros(model.q_size)
        rbdl.InverseDynamics(model, q, qdot, np.zeros(model.qdot_size), h)
        return h

    def CalcJacobian(self, model, q, bodyid, point):
        Jc = np.zeros((3, model.dof_count))
        rbdl.CalcPointJacobian(model, q, bodyid, point, Jc)
        return Jc

    def Jc_from_cpoints(self, model, q, body, cpoints):

        Jc = np.array([])

        ftip_pose = np.array([body.l_end, 0., 0.])

        if 1 in cpoints:
            Jc_ = self.CalcJacobian(model, q, body.id('b3h'), ftip_pose)
            Jc = np.append(Jc, Jc_[:2, :])

        if 2 in cpoints:
            Jc_ = self.CalcJacobian(model, q, body.id('b3f'), ftip_pose)
            Jc = np.append(Jc, Jc_[:2, :])

        return Jc.reshape(np.size(Jc) // model.dof_count, model.dof_count)

    def CalcGamma(self, cp, q, qdot):

        self.cbody_id = []

        if 1 in cp:
            for i in range(self.point_F_dim): self.cbody_id.append( \
                self.body.id('b3h'))
        if 2 in cp:
            for i in range(self.point_F_dim): self.cbody_id.append( \
                self.body.id('b3f'))
        #        if 3 in cp:
        #            for i in range(3):self.cbody_id.append(self.body.id('knee_3'))
        #        if 4 in cp:
        #            for i in range(3):self.cbody_id.append(self.body.id('knee_4'))

        Normal = []
        for i in range(len(cp)):
            Normal.append(np.array([1., 0.]))
            Normal.append(np.array([0., 1.]))
        #            Normal.append(np.array([0., 0., 1.]))

        k = len(cp) * self.point_F_dim

        Gamma = np.zeros(k)

        prev_body_id = 0

        gamma_i = np.zeros(self.point_F_dim)

        for i in range(k):

            if prev_body_id != self.cbody_id[i]:
                gamma_i = rbdl.CalcPointAcceleration(self.model, q, \
                                                     qdot, np.zeros(self.qdim), self.cbody_id[i], \
                                                     np.array([self.body.l_end, 0., 0.]))[:2]
                prev_body_id = self.cbody_id[i]

            Gamma[i] = - np.dot(Normal[i], gamma_i)
        return Gamma

    def CalcJgdotQdot(self):
        actual_bodies = ['b1h', 'b2h', 'b3h', 'b1f', 'b2f', 'b3f']
        jdqds = dict()

        for body in self.body.bodies:
            if body in actual_bodies:
                if body == 'b1f':
                    pos = self.param.lg1f
                elif body == 'b1h':
                    pos = self.param.lg1h
                elif body == 'b2h':
                    pos = self.param.lg2h
                elif body == 'b2f':
                    pos = self.param.lg2f
                elif body == 'b3h':
                    pos = self.param.lg3h
                elif body == 'b3f':
                    pos = self.param.lg3f
                point_position = np.array([pos, 0., 0.])

                gamma_i = rbdl.CalcPointAcceleration(self.model, self.q[-1], \
                                                     self.qdot[-1], np.zeros(self.qdim), self.body.id(body), \
                                                     point_position)[:2]
                jdqds[body] = gamma_i

        Mf = self.param.m1f + self.param.m2f + self.param.m3f

        Mh = self.param.m1h + self.param.m2h + self.param.m3h

        jdqd_h = (self.param.m1h * jdqds['b1h'] + self.param.m2h * jdqds['b2h'] + \
                  self.param.m3h * jdqds['b3h']) / Mh

        jdqd_f = (self.param.m1f * jdqds['b1f'] + self.param.m2f * jdqds['b2f'] + \
                  self.param.m3f * jdqds['b3f']) / Mf

        return jdqd_h, jdqd_f

    def CalcAcceleration(self, q, qdot, qddot, body_id, body_point):
        body_accel = rbdl.CalcPointAcceleration(self.model, q, qdot, qddot, \
                                                body_id, body_point)
        return body_accel

    def TerrainHeight(self, x):
        if self.terrain is None:
            return 0

    def set_input(self, tau):
        if len(tau) == self.qdim:
            self.u0 = np.dot(self.S, tau)
        else:
            self.u0 = tau
        #        print self.u0
        return None

    def get_com(self, calc_velocity=False, calc_angular_momentum=False, \
                update=True, index=-1, body_part='hf', q=None, qdot=None):
        #        TODO: error for the whole body (body_part = 'hf') and calc_velocity = True

        com = np.zeros(3)
        if calc_velocity:
            com_vel = np.zeros(3)
        else:
            com_vel = None
        if calc_angular_momentum:
            angular_momentum = np.zeros(3)
        else:
            angular_momentum = None

        if q is not None:
            qq = q
        else:
            qq = self.q[index, :]
        if qdot is not None:
            qqdot = qdot
        else:
            qqdot = self.qdot[index, :]

        if body_part == 'hf':
            rbdl.CalcCenterOfMass(self.model, qq, \
                                  qqdot, com, com_vel, angular_momentum, update)

            if calc_velocity and calc_angular_momentum:
                return com, com_vel, angular_momentum
            elif calc_velocity and not calc_angular_momentum:
                return com, com_vel
            else:
                return com
        else:
            com, vel = self.__calculateBodyCOM(qq, \
                                               qqdot, calc_velocity, update, body_part)
            if calc_velocity:
                return com, vel
            else:
                return com

    def __calculateBodyCOM(self, q, dq, calc_velocity, update, body_part):
        if body_part == 'h':
            p1 = self.CalcBodyToBase(self.body.id('b1h'),
                                     np.array([self.param.lg1h, 0., 0.]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p2 = self.CalcBodyToBase(self.body.id('b2h'),
                                     np.array([self.param.lg2h, 0., 0.]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p3 = self.CalcBodyToBase(self.body.id('b3h'),
                                     np.array([self.param.lg3h, 0., 0.]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)

            if not calc_velocity:
                com = (self.param.m1h * p1 + self.param.m2h * p2 + self.param.m3h * p3) / \
                      (self.param.m1h + self.param.m2h + self.param.m3h)
                vel = None
            else:
                com = (self.param.m1h * p1[0] + self.param.m2h * p2[0] + self.param.m3h * p3[0]) / \
                      (self.param.m1h + self.param.m2h + self.param.m3h)
                vel = (self.param.m1h * p1[1] + self.param.m2h * p2[1] + self.param.m3h * p3[1]) / \
                      (self.param.m1h + self.param.m2h + self.param.m3h)

        if body_part == 'f':
            p1 = self.CalcBodyToBase(self.body.id('b1f'),
                                     np.array([self.param.lg1f, 0., 0.]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p2 = self.CalcBodyToBase(self.body.id('b2f'),
                                     np.array([self.param.lg2f, 0., 0.]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p3 = self.CalcBodyToBase(self.body.id('b3f'),
                                     np.array([self.param.lg3f, 0., 0.]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)

            if not calc_velocity:
                com = (self.param.m1f * p1 + self.param.m2f * p2 + self.param.m3f * p3) / \
                      (self.param.m1f + self.param.m2f + self.param.m3f)
                vel = None
            else:
                com = (self.param.m1f * p1[0] + self.param.m2f * p2[0] + self.param.m3f * p3[0]) / \
                      (self.param.m1f + self.param.m2f + self.param.m3f)
                vel = (self.param.m1f * p1[1] + self.param.m2f * p2[1] + self.param.m3f * p3[1]) / \
                      (self.param.m1f + self.param.m2f + self.param.m3f)

        return com, vel

    def CalcBodyToBase(self, body_id, body_point_position, \
                       calc_velocity=False, update_kinematics=True, index=-1, q=None, qdot=None):
        if q is not None:
            qq = q
        else:
            qq = self.q[index, :]
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, qq, \
                                              body_id, body_point_position, update_kinematics)
        if not calc_velocity:
            return pose
        else:
            if qdot is not None:
                qqdot = qdot
            else:
                qqdot = self.qdot[index, :]
            vel = rbdl.CalcPointVelocity(self.model, qq, \
                                         qqdot, body_id, body_point_position, update_kinematics)
            return pose, vel

    def computeJacobianCOM(self, body_part):
        bis = []
        pts = []
        ms = []
        if body_part == 'h':
            bis.append(self.body.id('b1h'))
            bis.append(self.body.id('b2h'))
            bis.append(self.body.id('b3h'))
            pts.append(np.array([self.param.lg1h, 0., 0.]))
            pts.append(np.array([self.param.lg2h, 0., 0.]))
            pts.append(np.array([self.param.lg3h, 0., 0.]))
            ms = [self.param.m1h, self.param.m2h, self.param.m3h]

        elif body_part == 'f':
            bis.append(self.body.id('b1f'))
            bis.append(self.body.id('b2f'))
            bis.append(self.body.id('b3f'))
            pts.append(np.array([self.param.lg1f, 0., 0.]))
            pts.append(np.array([self.param.lg2f, 0., 0.]))
            pts.append(np.array([self.param.lg3f, 0., 0.]))
            ms = [self.param.m1f, self.param.m2f, self.param.m3f]

        J = np.zeros((3, self.qdim))

        for i, bi in enumerate(bis):
            J += ms[i] * self.CalcJacobian(self.model, self.q[-1, :], bi, pts[i])

        return J[:2, :] / sum(ms)

    def computeJacobianCOM23(self, body_part):
        J = self.computeJacobianCOM(body_part)
        if body_part == 'h':
            return J[:, [3, 4]]
        elif body_part == 'f':
            return J[:, [6, 7]]

    def computeFootState(self, body_part, \
                         calc_velocity=False, update_kinematics=True, \
                         index=-1, q=None, qdot=None):

        point = np.array([self.body.l_end, 0., 0.])
        if body_part == 'h':
            body_id = self.body.id('b3h')
        elif body_part == 'f':
            body_id = self.body.id('b3f')

        return self.CalcBodyToBase(body_id, point, \
                                   calc_velocity=calc_velocity, \
                                   update_kinematics=update_kinematics, \
                                   index=index, q=q, qdot=q)

    def getContactFeet(self, total=False):
        if not total:
            return self.__p[-1]
        else:
            return self.__p

    #    def SetContactFeet(self, newp):
    #        self.__p[-1] = newp
    #        return None

    #    def __evts(self): return [lambda t,x : -1]

    def __evts(self):
        """
        x = qqdot0
        """
        p = self.__p[-1]
        return [
            lambda t, x: None if 1 in p else self.Touchdown(t, x, 1),
            lambda t, x: None if 2 in p else self.Touchdown(t, x, 2),
            #            lambda t, x: None if 3 in p else self.Touchdown(t, x, 3),
            #            lambda t, x: None if 4 in p else self.Touchdown(t, x, 4),
            lambda t, x: None if 1 not in p else self.Liftoff(t, x, 1),
            lambda t, x: None if 2 not in p else self.Liftoff(t, x, 2),
            #            lambda t, x: None if 3 not in p else self.Liftoff(t, x, 3),
            #            lambda t, x: None if 4 not in p else self.Liftoff(t, x, 4),
            lambda t, x: None if 1 not in p else self.Liftoff_GRF(t, x, 1),
            lambda t, x: None if 2 not in p else self.Liftoff_GRF(t, x, 2),
            #            lambda t, x: None if 3 not in p else self.Liftoff_GRF(t, x, 3),
            #            lambda t, x: None if 4 not in p else self.Liftoff_GRF(t, x, 4),
            lambda t, x: None if not self.StopSimulation(t, x, p) else 0]

    def StopSimulation(self, t, x, p):
        out = False  # change it if you want to stop simulation for some reasons

        #        if len(p) < 3:
        ##            out = True
        #            print 'less than 3 legs are in contact with the ground!'

        return out

    #
    def Touchdown(self, t, x, leg):
        """
        should return a positive value if leg penetrated the ground
        """
        q = x[:self.qdim]
        point = np.array([self.body.l_end, 0., 0.])

        if leg == 1:
            body_id = self.body.id('b3h')
        elif leg == 2:
            body_id = self.body.id('b3f')

        #        exec("body_id = self.body.id('b3"+repr(leg)+"')")

        pose = self.CalcBodyToBase(body_id, point, q=q)
        return - (pose[1] - self.TerrainHeight(pose[0]))

    def Liftoff(self, t, x, leg):
        return -1

    #    def Liftoff_GRF(self, t, x, leg, p):
    #        index = p.index(leg)
    ##        print 'index in liftoff', index
    ##        print abs(t - self.t[-1]) <= self.dt
    ##        abs(t - self.t[-1]) <= self.dt*1.05:
    #        if np.allclose(t, self.t[-1]): return - self.cforce[-1][index*3 + 2]
    ##        else:  return  - self.Lambda[index*3 \
    ##        + 2]
    #
    #        elif t > self.t[-1]: return - self.Lambda[index*3 + 2]
    #        else: raise ValueError('Liftoff_GRF() does not have relevant sol.')
    ##        else: raise ValueError('Liftoff_GRF() does not have relevant sol.')

    #    def Liftoff_GRF(self, t, x, leg, p):
    #        return -1

    def Liftoff_GRF(self, t, y, leg):
        if hasattr(self, 'for_refine'):
            u = self.u[-1, :]
        else:
            yprev = np.concatenate((self.q[-1, :], self.qdot[-1, :]))
            if np.allclose(y, yprev):
                u = self.u[-1, :]
            else:
                u = self.u0
        #        index = self.__p0.index(leg)
        self.ComputeContactForce(y, self.__p0, u)
        if leg == 1:
            tt = self.tt_h
        elif leg == 2:
            tt = self.tt_f

        if t - tt < .25 * self.slip_st_dur:
            return -1
        else:
            return - self.Lambda[(leg - 1) * 2 + 1]

    #        if leg == 1: return t - self.tt_h - self.slip_st_dur
    #        elif leg == 2: return t - self.tt_f - self.slip_st_dur
    #        elif leg == 2: return - self.Lambda[(leg - 1)*2 + 1] - 50

    def predictNextLiftoff(self, y, dy):
        p = [-1 / 2 * self.param.g0, dy, y - self.slip_yt]
        ts = np.roots(p)
        print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@', ts)
        return max(ts)

    def refine(self, evtFun, tol=1e-9, *args):
        """
           Find the time at which the trajectory is a root of the
           function evtFun.

           The original code is part of the package 'integro', which is adopted
           here for notation consistency.

           evtFun is a function taking (t,y,*args) and returning a real number.
           refine() uses trajectory patch to find the "time"
           at which evtFun(t,y(t),*args)==0.

           This code requires that the signs of evtFun in the beginning and
           end of the patch are opposite. It finds the root by bisection
           -- a binary search.

           returns t,y --the end time and state; t is correct to within +/- tol
        """
        t0, t1 = self.t0, self.t0 + self.dt
        for k in range(4):
            y = self.qqdot0forRefine.copy()
            f0 = evtFun(t0, y, *args)
            dy = self.RK4(self.dyn_RK4)
            y += dy(t0, self.qqdot0forRefine, np.abs(t1 - t0)).flatten()
            f1 = evtFun(t1, y, *args)
            #            print 't0,f0,t1,f1', t0,f0,t1,f1
            if f0 * f1 <= 0:
                break
            t0, t1 = t0 - (t1 - t0) / 2, t1 + (t1 - t0) / 2
            print("WARNING: function did not change sign -- extrapolating order ", k)
        if f1 < 0:
            t0, f0, t1, f1 = t1, f1, t0, f0

        ypre = self.qqdot0forRefine.copy()

        #        print t0, self.CalcBodyToBase(self.body.id('pelvis'), np.zeros(3), \
        #            q = ypre[:self.qdim])[0]
        dy = self.RK4(self.dyn_RK4)
        #        time = t0 + self.dt
        time = t1 + .001
        #        lambda_pre = self.cforce[-1]
        del self.Lambda
        y = ypre + dy(t0, ypre, np.abs(time - t0)).flatten()
        #        print np.allclose(self.cforce[-1], lambda_pre)
        #        print 'self.Lambda', self.Lambda
        self.for_refine = True
        #        print 'f1_new', evtFun(time, y,  *args)
        #        print t1, self.CalcBodyToBase(self.body.id('pelvis'), np.zeros(3), \
        #            q = y[:self.qdim])[0]
        while abs(t1 - t0) > tol:
            t = (t1 + t0) / 2.

            #            dy = self.RK4(self.dyn_RK4)
            #            ytest = ypre + dy(0, ypre, self.dt*0)
            #            ftest = evtFun(t0 + self.dt*.001, ytest, *args )

            #            print 'ftest', ftest
            #            print 'ttest, t', ttest, t

            dy = self.RK4(self.dyn_RK4)
            y = ypre + dy(t0, ypre, np.abs(t - t0))
            #            print 'lambda', self.Lambda[-4]

            #            print self.t[-1], t0
            #            y = integrate.odeint(self.dyn, ypre, \
            #            np.array([0, np.abs(t1 - t0)/2]))[-1, :]
            #            print y.shape
            f = evtFun(t, y, *args)
            #            print 't*****f', t, f
            #            print self.Lambda[8]
            #            print '++t', t
            #
            #            print t, self.CalcBodyToBase(self.body.id('pelvis'), np.zeros(3), \
            #            q = y[:self.qdim])[0]

            if f == 0:
                break
            if f > 0:
                t1, f1 = t, f
            else:
                t0, f0 = t, f
                ypre = y
        del self.for_refine
        return (t1 + t0) / 2, y.reshape(1, self.qdim * 2)

    #    def interpl(self, evtFun, *args):
    #        t0, t1 = self.t0, self.t0 + self.dt
    #        y0 = self.qqdot0forRefine.copy()
    #        f0 = evtFun(t0, y0, *args)
    #        dy = self.RK4(self.dyn_RK4)
    #        y1 = y0 + dy(t0, y0, np.abs(t1 - t0)).flatten()
    #        f1 = evtFun(t1, y1, *args)
    #        print 't0,f0,t1,f1', t0,f0,t1,f1
    #        if np.abs(f0)<np.abs(f1): self.for_refine = True
    #        t = t0 - f0*(t1 - t0)/(f1 - f0)
    #        y = y0 + dy(t0, y0, np.abs(t - t0)).flatten()
    #        if hasattr(self, 'for_refine'): del self.for_refine
    #        return t, y.reshape(1, self.qdim*2)

    def interpl(self, evtFun, *args):
        t0, t1 = self.t0, self.t0 + self.dt
        y0 = self.qqdot0forRefine.copy()
        f0 = evtFun(t0, y0, *args)
        dy = self.RK4(self.dyn_RK4)
        y1 = y0 + dy(t0, y0, np.abs(t1 - t0)).flatten()
        f1 = evtFun(t1, y1, *args)
        #        print 't0,f0,t1,f1', t0,f0,t1,f1
        if np.abs(f0) < np.abs(f1): self.for_refine = True
        t = t0 - f0 * (t1 - t0) / (f1 - f0)
        y = y0 + dy(t0, y0, np.abs(t - t0)).flatten()
        if hasattr(self, 'for_refine'): del self.for_refine
        return t, y.reshape(1, self.qdim * 2)

    def __trans(self):
        p0 = list(self.__p0)
        qqdot0 = self.qqdot0[-1, :].copy()
        q, qdot = qqdot0[:self.qdim], qqdot0[self.qdim:]
        #        touchdown = False

        #        for i in range(2):
        #            if self.ev_i == i: p0.append(i + 1); touchdown = True
        #        if touchdown:
        #            qdot = self.UpdateQdotCollision(q, qdot, p0)
        #            if self.ev_i == 0: self.tt_h = self.trefined
        #            elif self.ev_i == 1: self.tt_f = self.trefined

        #        for i in [4, 5]:
        #            if self.ev_i == i and i - 3 in p0:
        #                p0.remove(i - 3)
        #                if i - 3 == 1: self.tl_h = self.trefined
        #                elif i - 3 == 2: self.tl_f = self.trefined

        if self.ev_i == 0:  # touchdown of hind leg
            p0.append(1)
            qdot = self.UpdateQdotCollision(q, qdot, p0)
            self.tt_h = self.trefined
            self.foot_pose_h = self.computeFootState('h', q=q)[0]
            self.xt_h = self.get_com(body_part='h', q=q)

        elif self.ev_i == 1:  # touchdown of fore leg
            p0.append(2)
            qdot = self.UpdateQdotCollision(q, qdot, p0)
            self.tt_f = self.trefined
            self.foot_pose_f = self.computeFootState('f', q=q)[0]
            self.xt_f = self.get_com(body_part='f', q=q)

        elif self.ev_i == 4 and 1 in p0:  # liftoff of hind leg
            p0.remove(1)
            self.tl_h = self.trefined
            self.xl_h, self.dxl_h = \
                self.get_com(body_part='h', q=q, calc_velocity=True)
            self.ql = self.q[-1]
            # TODO: should be leg specific?
        #            self.slip_sw_dur = \
        #            max(self.slip_sw_dur, self.predictNextLiftoff(self.xl_h[1], self.dxl_h[1]))

        elif self.ev_i == 5 and 2 in p0:  # liftoff of fore leg
            p0.remove(2)
            self.tl_f = self.trefined
            self.xl_f, self.dxl_f = \
                self.get_com(body_part='f', q=q, calc_velocity=True)
            self.ql = self.q[-1]
            # TODO: should be leg specific?
        #            self.slip_sw_dur = \
        #            max(self.slip_sw_dur, self.predictNextLiftoff(self.xl_f[1], self.dxl_f[1]))

        #        if self.ev_i in [4, 5]: raise ValueError('liftoff occured!!!')

        p0.sort()
        qqdot0_new = np.concatenate((q, qdot)).reshape(1, self.qdim * 2)
        return qqdot0_new, p0

    def UpdateQdotCollision(self, q, qdot, p0, W=None):
        J = self.Jc_from_cpoints(self.model, q, self.body, p0)
        if W is None: W = self.CalcM(self.model, q)
        invW = np.linalg.inv(W)
        aux1 = np.dot(invW, J.T)
        aux2 = np.linalg.inv(np.dot(J, np.dot(invW, J.T)))
        invJ = np.dot(aux1, aux2)
        qdot_after = np.dot(np.eye(np.size(qdot)) - np.dot(invJ, J), qdot)
        return qdot_after

    def __trans2(self):
        """
        .__trans  transition between discrete modes
        """
        q, xh, xf, leg = self.q0.copy()
        m, I, l, lg, g = self.param
        m1, m2, m3, m4, m5 = m
        I1, I2, I3, I4, I5 = I
        l1, l2, l3, l4, l5 = l
        lg1, lg2, lg3, lg4, lg5 = lg
        x = self.x0[-1, :].copy()
        z, dz = x[:5], x[5:]
        e = self.e

        # SS after vlo
        if q == 0:
            #            from Matrices5D import MHc
            if any([e == i for i in range(1, len(self.ev))]):
                q = -1
            # Touchdown
            elif e == 0:
                self.td_proxy = 0
                dz = dz.reshape(5, 1)
                dz = np.dot(self.N(), dz)
                dz = self.DSRefine(z, dz.T[-1, :]).reshape(5, 1)
                xh = xf
                xf = MHc(np.hstack((z, dz.T[-1, :])), self.param)[0] + xh
                q = 1

        x = np.array(np.hstack((z.reshape(1, 5), dz.reshape(1, 5))))
        qq = np.array([q, xh, xf, leg])

        return x, qq

    def temp(self, i):

        self.dy = self.RK4(self.dyn_RK4)

        import matplotlib.pyplot as plt

        def create(q, qdot):
            return np.concatenate((q, qdot)).reshape(1, self.qdim * 2)

        q = self.q[i, :]
        qdot = self.qdot[i, :]

        y = create(q, qdot)

        plt.plot(self.Lambda, '--*')

        self.dy(self.t[i], y.flatten(), self.dt)

        plt.plot(self.Lambda, '-o')











