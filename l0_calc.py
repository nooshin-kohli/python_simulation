# -*- coding: utf-8 -*-
"""
Year: 2022

@author: Nooshin Kohli

This code use a PID controller to control posture of the robot in landing.
"""

from __future__ import division

import sys


import numpy as np
import time
import matplotlib.pyplot as plt

from leg_robotclass import leg_robotclass
#from Centauro_RobotClass import Centauro_RobotClass
# from leg_controlclass import leg_controlclass
# from leg_tasksetclass import TaskSet
from Utils import Anim_leg, Plot_base_coordinate, Plot_foot_tip, \
Plot_contact_force, traj_plan

plt.close('all')

# initiate time array
t = np.array([0])
dt = .005 # step size

# initiate stats with dummy values
q = np.zeros((1, 0)) # joint position
qdot = np.zeros((1, 0)) # joint velocity
tau = np.zeros(4) # control inputs
p = [[ ]] # the contact feet

leg = leg_robotclass(t=t,q=q,qdot=qdot,p=p,u=tau,dt=dt,\
    urdf_file='/home/lenovo/python_simulation/python_simulation/legRBDL.urdf',param=None,terrain=None)

############################################Homing initialize
leg.q[-1,0] = 0.0     #slide
leg.q[-1,1] = 0.0231  #hip
leg.q[-1,2] = 0.8    #thigh
leg.q[-1,3] = -1.2   #calf


slider = leg.CalcBodyToBase(leg.model.GetBodyId('jump'), np.array([0. , 0., 0.]))
foot = leg.CalcBodyToBase(leg.model.GetBodyId('calf'), np.array([0. , 0., -0.240]))
print("l0: ", slider - foot)
