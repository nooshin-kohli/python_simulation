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
        
    

    def calcJc(self, q):
        #q = self.fixq3(q)
        jc = np.zeros((3, self.model.q_size))
        rbdl.CalcPointJacobian(self.model, q, self.model.GetBodyId('calf'), self.end_point, jc)
        return jc

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
# path =  "/home/lenovo/python_simulation/python_simulation/leg_RBDL.urdf"
# robot = ROBOT(np.zeros((3, 1)), np.zeros((3, 1)), path)
# q = np.zeros(4)
# # q[2] = 0.6
# print(robot.calcG(q)) 