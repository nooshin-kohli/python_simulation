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

leg = leg_robotclass(t=t,q=q,qdot=qdot,p=p,u=tau,dt=dt,urdf_file='/home/lenovo/python_simulation/python_simulation/legRBDL.urdf',param=None,terrain=None)
# ct = TaskSet(cr)
# cc = leg_controlclass(cr)
#cr.tt_h = 0.1 #TODO
#cr.tl_h = 0.3 #TODO
#cr.tt_f = 0.1 #TODO
#cr.tl_f = 0.3 #TODO
leg.slip_st_dur = 0.8 #TODO

#tau[1]  = 1
#tau[3] = .01

######################################
#### create initial configuration ####
#TODO: use FourBar function to sync this configuration with dual-slip model.

angle1 = np.deg2rad(20)
angle2 = np.deg2rad(100)
angle3 = np.deg2rad(80)
leg.q[-1,3] = -0.1
#cr.q[-1,0] = 0.9 

# cr.q[-1,3] = .05
# cr.qdot[-1,0] = -0.2

Time_end = 2
# tau[0,0] = 0
# tau[2] = 40
def pidctrl(q, qdot, p, d):
    q_des = [0, 0, -0.1, -0.3]
    qdot_des = [0, 0, 0, 0]
    Kp = [[p,0,0,0],
          [0,p,0,0],
          [0,0,p,0],
          [0,0,0,p]]
    Kd = [[d,0,0,0],
          [0,d,0,0],
          [0,0,d,0],
          [0,0,0,d]]
    tau = (np.dot((q_des-q),Kp) + np.dot((qdot_des-qdot),Kd)).flatten()
    # tau.reshape(4,1)
    # tau.flatten()
    # print("tau shape in PID:", np.shape(tau))
    return tau

stopflag = False
while leg.t[-1][0]<=Time_end:
    # print(np.shape(np.dot(cr.S.T, np.zeros_like(cr.u[-1, :]))))
    leg.set_input(tau)
#    print("leg.q: ", leg.q)
#    print("jc ", leg.Jc)
#    print(leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.])))
#    print("leg: ", leg.getContactFeet())
#    print("cpoint:",leg.)
    if 1 in leg.getContactFeet():
        
#        print(cr.CalcBodyToBase(cr.model.GetBodyId('jump'),np.array([0.,0.,0.])))
        tau = pidctrl(leg.q[-1: ], leg.qdot[-1,:],4,0.1)
#        q_prev = leg.q[-1]
#        print("Lambda: ", leg.Lambda)
#        print("tau:", tau)
        
        stopflag = True
        # print(np.shape(cr.S))
        # print(np.shape(tau))
        # print(np.shape(cr.h))
        # tau.reshape((4,1))
        # print(np.shape(tau))
    
#    if stopflag:
#        print("after PID")
#        print(tau)
    #     break
    leg()
   
#    print ("Contact foot", cr.getContactFeet())
    
print(leg.q)
robot_anim = Anim_leg(leg.model, leg.body, leg.joint, leg.q, leg.t)
Plot_contact_force(leg)


    


