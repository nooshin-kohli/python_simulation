# -*- coding: utf-8 -*-
"""
Created on Sat Jul 2 20:42:48 2022
@author: Nooshin Kohli
"""
import numpy as np
import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/rbdl/build/python'
sys.path.append(dir)
import rbdl


class ROBOT():
    def __init__(self, q, qdot, urdf_path):     # TODO: path to leg_RBDL.urdf for robot without slider
        self.model = rbdl.loadModel(urdf_path)
        #self.q = self.fixq3(q)
       # self.qdot = self.fixq3(qdot)
        self.calf_length = -0.240
        self.thigh_length = -0.2131
        self.end_point = np.asarray([0.0, 0.0, self.calf_length])
        self.calf_point = np.asarray([0.0, 0.0, self.thigh_length])
        self.hip_point = np.asarray([0.03, 0.0, 0.0])
        self.mass_hip = 2.805
        self.mass_thigh = 1.062
        self.mass_calf = 0.133
        self.qdim = 3
    

    def get_com(self, calc_velocity=False, calc_angular_momentum=False, \
                update=True, index=-1, body_part='slide', q=None, qdot=None):
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

        if body_part == 'slider':
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

    def __calculateBodyCOM(self, q, dq, calc_velocity, update, body_part):
        if body_part == 'slide':
            # p0 = self.CalcBodyToBase(self.model.GetBodyId('jump'),
            #                          np.array([0.03, 0, 0.0]),
            #                          update_kinematics=update,
            #                          q=q, qdot=dq, calc_velocity=calc_velocity)
            p1 = self.CalcBodyToBase(self.model.GetBodyId('hip'),
                                     np.array([0.03, 0, 0.0]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p2 = self.CalcBodyToBase(self.model.GetBodyId('thigh'),
                                     np.array([0.0, 0.06, -0.1059]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)
            p3 = self.CalcBodyToBase(self.model.GetBodyId('calf'),
                                     np.array([0., 0.01, -0.120]),
                                     update_kinematics=update,
                                     q=q, qdot=dq, calc_velocity=calc_velocity)

            if not calc_velocity:
                com = (self.mass_hip * p1 + self.mass_thigh * p2 + self.mass_calf * p3) / \
                      (self.mass_hip + self.mass_thigh + self.mass_calf)
                vel = None

            else:
                com = (self.mass_hip * p1[0] + self.mass_thigh * p2[0] + self.mass_calf * p3[0]) / \
                      (self.mass_hip + self.mass_thigh + self.mass_calf)
                vel = (self.mass_hip * p1[1] + self.mass_thigh * p2[1] + self.mass_calf * p3[1]) / \
                      (self.mass_hip + self.mass_thigh + self.mass_calf)
        return com, vel

    def CalcJacobian(self, model, q, bodyid, point):
        Jc = np.zeros((3, model.dof_count))
        rbdl.CalcPointJacobian(model, q, bodyid, point, Jc)
        return Jc
    
    def j_hip(self,q):
        jc = np.zeros((3,3))
        rbdl.CalcPointJacobian(self.model, q, self.model.GetBodyId('calf'), np.asarray([0.0, 0.0, -0.22]), jc)
        
        # rbdl.CalcPointJacobian(self.model, q, self.model.GetBodyId('hip'), self.hip_point, jc)
        # print(jc)
        return jc

        
    
    def calcJcom(self,q,body_part='robot'):
        bis = []
        pts = []
        ms = []
        if body_part == 'robot':
            
            bis.append(self.model.GetBodyId('hip'))
            bis.append(self.model.GetBodyId('thigh'))
            bis.append(self.model.GetBodyId('calf'))
            ######################## from urdf model ########################
            
            pts.append(np.array([0.03 ,0.0, -0.03]))
            pts.append(np.array([0.0, 0.06, -0.1059]))
            pts.append(np.array([0.0, 0.01, -0.124]))
            ms = [self.mass_hip, self.mass_thigh, self.mass_calf]

        else:
            print("error")

        J = np.zeros((3, self.qdim))
        # print("model", bis)

        for i, bi in enumerate(bis):
            J += ms[i] * self.CalcJacobian(self.model, q, bi, pts[i])

        return J[:3,:] / sum(ms)


    def calcJc(self, q):
        #q = self.fixq3(q)
        jc = np.zeros((3, self.model.q_size))
        rbdl.CalcPointJacobian(self.model, q, self.model.GetBodyId('calf'), self.end_point, jc)
        return jc

        
    def J_task(q):
        j_c = self.calcJc(q)
        j_com = self.calcJcom(q)
        j_t = np.vstack((j_com,j_c))
        return j_t

    def velocity_end(self, q, qdot):
        #q = self.fixq3(q)
        #qdot = self.fixq3(qdot)
        vel = rbdl.CalcPointVelocity(self.model, q, qdot, self.model.GetBodyId('calf'), self.end_point)
        return vel

    def pose_end(self, q):
        #q = self.fixq3(q)
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, q, self.model.GetBodyId('calf'), self.end_point)
        return pose
    
    def pose_calf(self, q):
        #q = self.fixq3(q)
        pose = rbdl.CalcBodyToBaseCoordinates(self.model, q, self.model.GetBodyId('calf'), np.zeros(3))
        return pose

    def calcG(self,q):
        Tau = np.zeros(3)
        model = rbdl.loadModel('/home/lenovo/python_simulation/python_simulation/leg_RBDL.urdf')
        rbdl.InverseDynamics(self.model, q, np.zeros(3), np.zeros(3), Tau)
        S = np.array([[0,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        # Tau = np.dot(S,Tau)
        Tau = -Tau
        return Tau
path =  "/home/lenovo/python_simulation/python_simulation/leg_RBDL.urdf"
robot = ROBOT(np.zeros((3, 1)), np.zeros((3, 1)), path)
q = np.zeros(3)
qdot = np.zeros(3)
print(robot.j_hip(q))
# # # q[2] = 0.6
# print(robot.calcJcom(q))
# com = np.zeros(3)
# com_vel = np.zeros(3)
# update = np.zeros(3)
# angular_momentum = np.zeros(3)
# print(robot.get_com(q=q,qdot=qdot))
# print(rbdl.CalcCenterOfMass(robot.model, q, qdot, com, com_vel, angular_momentum, update))


# print(robot.calcJc(q))