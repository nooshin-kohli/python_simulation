# -*- coding: utf-8 -*-
"""
Created on Mon Aug 29 18:01:36 2016

@author: mshahbazi
"""

from __future__ import division
from os.path import expanduser

home = expanduser("~")
dir = home + '/projects/rbdl/build/python'
sys.path.append(dir)
import rbdl
import sys

#sys.path.append('/home/mshahbazi/RBDL/build/python/')
sys.path.append('/home/zahra/rbdl-dev/build/python')


import numpy as np
import time
import matplotlib.pyplot as plt

from leg_robotclass import ROBOT
from leg_controlclass import leg_controlclass
from Utils import Anim_Centauro, Plot_base_coordinate, Plot_foot_tip, \
Plot_contact_force, traj_plan
from PhyParam import PhyParam, write_lua

plt.close('all')

# assign robot parameters inside PhyParam class:
# param = PhyParam()

# generate lua model
# write_lua(param)
# lua_file = 'robot2d.lua'
mode = 'slider'

# initiate time array
t = np.array([0])
dt = .005 # step size

# initiate stats with dummy values
# q = np.zeros((1, 0)) # joint position
q = np.zeros(4)
qdot = np.zeros(4)
# qdot = np.zeros((1, 0)) # joint velocity
u = np.zeros(4) # control inputs


p = [[]] # the contact feet
# strange behavior when contact = [[1, 2]] and the legs are upright!!!!

# instanciate robot object:
cr = ROBOT(t, dt, q, p, mode, qdot, u)

######################################
#### create initial configuration ####
#TODO: use FourBar function to sync this configuration with dual-slip model.

cr.q[0]=1

#cr.q[-1, cr.joint.q_i('reference_TY')] = - cr.CalcBodyToBase(cr.body.id('b3f'),
#                            np.array([cr.body.l_end, 0., 0.]))[1]

######################################
######################################

#cc = Centauro_ControlClass(cr)
#ct = TaskSet(cr)
#
##TODO: Do not allow manual definition of contact connstraint
#ct.AddTask(cr.body.id('pelvis'), np.zeros(3), [1, 0, 0], 1, 'pelvis_x')
#ct.AddTask(cr.body.id('pelvis'), np.zeros(3), [0, 1, 0], 1, 'pelvis_y')
#ct.AddTask(cr.body.id('pelvis'), np.zeros(3), [0, 0, 1], 3, 'pelvis_z')
#
#
#task_list = ['pelvis_x', 'pelvis_y', 'pelvis_z', 'foot_1_swing_z']
#x_start = cr.CalcBodyToBase(cr.body.id('pelvis'), np.zeros(3))
#
##x_start = np.append(x_start, .42)
##x_start = np.append(x_start, .6)
##x_start = np.append(x_start, 0.)
##x_start = np.append(x_start, 0.)
##x_start = np.append(x_start, 0.)
#x_start = np.append(x_start, .04)
#
#
#
#
#x_end = x_start.copy()
#x_end[0] += .05
##x_end[1] += -.1
##x_end[2] += -.1
#
##x_end[3] += -.3
##x_end[4] += +.2
##x_end[5] += +np.deg2rad(0)
#
##x_end[-1] += -.1
#
#
#
#z_start = [x_start, np.zeros(cr.qdim), np.zeros(cr.qdim)]
#z_end = [x_end, np.zeros(cr.qdim), np.zeros(cr.qdim)]
#
#Time_end = .2
#
#x_des_t, xdot_des_t, xddot_des_t = traj_plan(0, Time_end, z_start, z_end) 
#
#rlx = range(len(x_des_t))
#
#qdot_des_prev = np.zeros(cr.qdim) # TODO: we usually start from rest
#q_des_prev = cr.q[-1, :] 
#
#tic = time.time()
#
##error_ddot = []
#
##cr.q[-1, cr.joint.q_i('reference_TZ')] = -.1
#
#W = np.eye(cr.qdim - 6)
#weight = 1
#which_joint = 'hip_pitch'
#W[cr.joint.q_i(which_joint+'_1') - 6, cr.joint.q_i(which_joint+'_1') - 6] = weight
#W[cr.joint.q_i(which_joint+'_2') - 6, cr.joint.q_i(which_joint+'_2') - 6] = weight
#W[cr.joint.q_i(which_joint+'_3') - 6, cr.joint.q_i(which_joint+'_3') - 6] = weight
#W[cr.joint.q_i(which_joint+'_4') - 6, cr.joint.q_i(which_joint+'_4') - 6] = weight

Time_end = 5

while cr.t[-1][0]<=Time_end: 
    
    
    cr.set_input(np.dot(cr.S.T, np.zeros_like(cr.u)))
    
    cr()
                                        

robot_anim = Anim_Centauro(cr.model, cr.body, cr.joint, cr.q, cr.t)

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

    


