# -*- coding: utf-8 -*-
"""

@author: Nooshin Kohli
"""

from __future__ import division

import sys

#sys.path.append('/home/mshahbazi/RBDL/build/python/')
# sys.path.append('/home/zahra/rbdl-dev/build/python')


import numpy as np
import time
import matplotlib.pyplot as plt

# from leg_robotclass import ROBOT
from Centauro_RobotClass import Centauro_RobotClass
from leg_controlclass import leg_controlclass
from leg_tasksetclass import TaskSet
from Utils import Anim_leg, Plot_base_coordinate, Plot_foot_tip, \
Plot_contact_force, traj_plan
# from PhyParam import PhyParam, write_lua

plt.close('all')

# assign robot parameters inside PhyParam class:
# param = PhyParam()

# # generate lua model
# write_lua(param)
# lua_file = 'robot2d.lua'

# initiate time array
t = np.array([0])
dt = .005 # step size

# initiate stats with dummy values
q = np.zeros((1, 0)) # joint position
qdot = np.zeros((1, 0)) # joint velocity
# u = np.zeros((1, 0)) # control inputs
tau = np.zeros(4)



p = [[ ]] # the contact feet
# strange behavior when contact = [[1, 2]] and the legs are upright!!!!

# instanciate robot object:

# cr = ROBOT(t, dt, q=q, p=p, mode = 'slider', qdot=qdot, u= tau)
cr = Centauro_RobotClass(t=t,q=q,qdot=qdot,p=p,u=tau,dt=dt,urdf_file='/home/kamiab/simulation-python-quadruped/python_simulation/legRBDL.urdf',param=None,terrain=None)
# ct = TaskSet(cr)
# cc = leg_controlclass(cr)
cr.tt_h = 0.1 #TODO
cr.tl_h = 0.3 #TODO
cr.tt_f = 0.1 #TODO
cr.tl_f = 0.3 #TODO
cr.slip_st_dur = 0.5 #TODO

#tau[1]  = 1
#tau[3] = .01

######################################
#### create initial configuration ####
#TODO: use FourBar function to sync this configuration with dual-slip model.

angle1 = np.deg2rad(20)
angle2 = np.deg2rad(100)
angle3 = np.deg2rad(80)
# cr.q[-1,0] = -0.2
# cr.q[-1,3] = .05
# cr.qdot[-1,0] = -0.2

Time_end = 0.4
# tau[0,0] = 0
# tau[2] = 40
def pidctrl(q, qdot, p, d):
    q_des = [100000, 0, -0.1, -0.5]
    qdot_des = [0, 0, 0, 0]
    Kp = [[p,0,0,0],
          [0,p,0,0],
          [0,0,p,0],
          [0,0,0,p]]
    Kd = [[d,0,0,0],
          [0,d,0,0],
          [0,0,d,0],
          [0,0,0,d]]
    tau = np.dot((q_des-q),Kp).flatten() #+ np.dot((qdot_des-qdot),Kd)
    # tau.reshape(4,1)
    # tau.flatten()
    # print("tau shape in PID:", np.shape(tau))
    return tau

stopflag = False
while cr.t[-1][0]<=Time_end:
    # print(np.shape(np.dot(cr.S.T, np.zeros_like(cr.u[-1, :]))))
    cr.set_input(tau)
    if 1 in cr.getContactFeet():
        stopflag = True
        tau = pidctrl(cr.q[-1: ], cr.qdot[-1,:],10,1)
        # print(np.shape(cr.S))
        # print(np.shape(tau))
        # print(np.shape(cr.h))
        # tau.reshape((4,1))
        # print(np.shape(tau))
    cr()
    # if stopflag:
    #     print("after PID")
    #     print(tau)
    #     break
   
#    print ("Contact foot", cr.getContactFeet())
    





                                            

#toc = time.time() - tic

#print toc
#print(cr.q)
robot_anim = Anim_leg(cr.model, cr.body, cr.joint, cr.q, cr.t)
Plot_contact_force(cr)
#x_des = np.zeros(3)

#qddot_des = cp.qddot_from_xbase_no_hierarchi(x_des = x_des)

# robot_anim = Anim_leg(cr.model, cr.body, cr.joint, cr.q, cr.t)

#plt.plot(cr.t[:], r2d(cr.x[:, cr.joint.q_i('knee_pitch_1')]))
#plt.plot(cr.t[:], r2d(cr.x[:, cr.joint.q_i('knee_pitch_2')]))
#plt.plot(cr.t[:], -r2d(cr.x[:, cr.joint.q_i('knee_pitch_3')]))
#plt.plot(cr.t[:], -r2d(cr.x[:, cr.joint.q_i('knee_pitch_4')]))
#plt.plot(cr.t[:], r2d(cr.q[:, cr.joint.q_i('j_arm2_7')]))


#plt.plot(cr.t, cr.u).

#Plot_base_coordinate(cr) 
#Plot_foot_tip(cr)   
#Plot_contact_force(cr)

#plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_1') - 6])
#plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_2') - 6])
#plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_3') - 6])
#plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_4') - 6])

    


