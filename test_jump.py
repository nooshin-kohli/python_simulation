"""
Created on Sat Jul 2 20:42:48 2022

@author: Kamiab yazdi & Nooshin Kohli
"""
from __future__ import division
from pickle import NONE

import sys


import numpy as np
import time
import matplotlib.pyplot as plt
from scipy import linspace

from leg_robotclass import leg_robotclass
from leg_controlclass import leg_control
from scipy.interpolate import InterpolatedUnivariateSpline as intp
from Utils import Anim_leg, Plot_base_coordinate, Plot_foot_tip, \
Plot_contact_force, traj_plan




plt.close('all')


# def extract_data(input_f, input_h):
#     GF_contact=[]
#     y_contact=[]
#     time_step = np.linspace(0,3,3000)
#     for i in time_step:
#         if input_f(i)<0:
#             pass
#         else:
#             while(input_f(i)>0):
#                 GF_contact.append(input_f(i))
#                 y_contact.append(input_h(i))
#                 i+=0.001
#             break
   
#     return GF_contact, y_contact



##################################importing the contact forces from slip model
# import test
from object import data_input
##################### 
h = data_input(dt=.01, m=3.825, L0=0.366, k0=1500)
time_test= np.linspace (0, 3, len(h.function(3)[0]))

intp_y_comp, intp_GF_comp = h.interpolate(h.function(3)[0], h.function(3)[1])
plt.figure()
plt.plot(h.time_slip(3),h.function(3)[0])
plt.plot(time_test,intp_GF_comp(time_test))

plt.figure()
plt.plot(time_test, intp_y_comp(time_test))

GF_contact, y_contact= h.extract_data(intp_GF_comp, intp_y_comp)


print("leeeen: ", len(GF_contact))
tau_s = np.linspace(0, 1, len(GF_contact))
plt.figure()
plt.plot(tau_s,GF_contact)
plt.show()

##### interpolate y_des & GF_contact
tau_s = np.linspace(0, 1, len(GF_contact))
# # print(tau_s)
# time_test= np.linspace (0, 3, len(h.function(3)[1]))
intp_gf = intp(tau_s, GF_contact, k=1)
intp_y = intp(tau_s, y_contact, k=1)

# intp_y_comp =  intp(time_test, h.function(3)[1], k=1)

# GF_contact, y_des_contact = extract_data(h.function(3)[0], h.function(3)[1])
# print("y_des_contact: ", y_des_contact)


xs = np.linspace(0, 1, 1000)
# print("intp:", intp_gf(xs)[-1])
plt.plot(xs, intp_gf(xs), 'r', lw=3, alpha=0.7)
plt.show()

# plt.figure()
# plt.plot(np.linspace(0,3, num=len(h.function(3)[1])),h.function(3)[1],'g')
# plt.show()



t = np.array([0])

dt = .001 # step size

# initiate stats with dummy values
q = np.zeros((1, 0)) # joint position
qdot = np.zeros((1, 0)) # joint velocity
# u = np.zeros((1, 0)) # control inputs
tau = np.zeros(4)



p = [[ ]] # the contact feet


leg = leg_robotclass(t=t,q=q,qdot=qdot,p=p,u=tau,dt=dt,\
    urdf_file='/home/lenovo/python_simulation/python_simulation/legRBDL.urdf',param=None,terrain=None)
leg.slip_st_dur = h.contact_time(h.function(3)[0], h.function(3)[1])



############################################Homing initialize
leg.q[-1,0] = 0.0     #slide
leg.q[-1,1] = 0.0231  #hip
leg.q[-1,2] = 0.8     #thigh
leg.q[-1,3] = -1.2    #calf

leg_control = leg_control(leg)




Time_end = 4

t = []
time_pre = time.time()
h_vec = []
first_check=0
stopflag = False
global e_pre 
e_pre = [0, 0, 0]
t_pre = 0

slip_gf_list=[]
time_list=[]
p_gain_list=[]
d_gain_list=[]
# x0 = y_des_contact[0]

while leg.t[-1][0]<=Time_end:

    
    leg.set_input(tau)
    if leg.getContactFeet():
        leg()
        if first_check == 0:
            t_td = leg.t[-1][0]
            t_lo = t_td+ h.contact_time(h.function(3)[0], h.function(3)[1])
            first_check=1
        stopflag = True
        slider_h = leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))
        TAU = leg_control.compute_TAU(leg.t[-1][0], t_td, t_lo)
        delta_time = leg.t[-1][0] - t_pre
        ################################################################ FOR PID CONTROLLER 
        # ydot_d = (intp_y(TAU) - x0)/0.01                # 0.01 is the step size in slip model

        tau,e_pre, p_gain, d_gain = leg_control.stance(slider_h, leg.Jc,\
             intp_gf(TAU), intp_y(TAU),delta_time, e_pre,leg)
        
        
        p_gain_list.append(p_gain)
        d_gain_list.append(d_gain)
        
        slip_gf_list.append(intp_gf(TAU))
        time_list.append(leg.t[-1][0])

        tau[0] = 0
        print("tau at the contact: ", tau)
        leg.tt_h = t_td #TODO
        leg.tl_h = t_lo #TODO
        leg.slip_st_dur = leg.tl_h-leg.tt_h
        print("foot position in landing: ",leg.computeFootState('h'))

        t_pre = leg.t[-1][0]
        
        
    else:
        tau = leg_control.flight(leg.q[-1,:],leg.qdot[-1,:])
        slip_gf_list.append(0)
        p_gain_list.append(0)
        d_gain_list.append(0)
        time_list.append(leg.t[-1][0])
        tau[0] = 0
        first_check=0
        t_pre = leg.t[-1][0]
        # e_pre = [0, 0, 0]
        
        
    # print(leg.get_com(body_part='h',calc_velocity=True))
    # print("jacobian COM================",leg.computeJacobianCOM('slider'))
    leg()
    h_vec.append(leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))[2])
    time_now = leg.t[-1,:]
    t.append(time_now)

    

plt.figure()
plt.title("slip height")
plt.plot(t,h_vec,'r')
plt.plot(np.linspace(0,3, num=len(h.function(3)[1])),h.function(3)[1],'g')
plt.legend(["simulation", "slip model"], loc ="upper right")

plt.figure()
plt.plot(time_list,p_gain_list,'r')
plt.plot(time_list, d_gain_list,'b')
plt.legend(["p_gain", "d_gain"], loc ="upper right")
plt.show()
robot_anim = Anim_leg(leg.model, leg.body, leg.joint, leg.q, leg.t)
# plt.figure()
# plt.plot(time_list, slip_gf_list)
# plt.show()
Plot_contact_force(leg, time_list, slip_gf_list)




