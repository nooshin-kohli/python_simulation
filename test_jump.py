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


def extract_data(input_f, input_h):
    GF_contact=[]
    y_contact=[]
    for i in range(len(input_f)):
        if input_f[i]<0:
            pass
        else:
            while(input_f[i]>0):
                GF_contact.append(input_f[i])
                y_contact.append(input_h[i])
                i+=1
            break
    return GF_contact, y_contact



##################################importing the contact forces from slip model
from object import data_input
##################### 
h = data_input(dt=.001, m=4, L0=0.366 , k0=800)#  0.329       0.362
GF_contact, y_des_contact = extract_data(h.function(3)[0], h.function(3)[1])

################################## interpolate y_des & GF_contact
tau_s = np.linspace(0, 1, len(GF_contact))
time_test= np.linspace (0, 3, len(h.function(3)[1]))
intp_gf = intp(tau_s, GF_contact, k=1)
intp_y = intp(tau_s, y_des_contact, k=1)
intp_y_comp =  intp(time_test, h.function(3)[1], k=1)

################################# plotting ground force from slip model
xs = np.linspace(0, 1, 1000)
plt.plot(xs, intp_gf(xs), 'r', lw=3, alpha=0.7)
plt.show()

################################# Assigning dynamic parameters  
t = np.array([0])
dt = .001 # step size
###################### Initiate stats with dummy values
q = np.zeros((1, 0))                                   # joint position
qdot = np.zeros((1, 0))                                # joint velocity
tau = np.zeros(4)                                      # control inputs
p = [[ ]]                                              # the contact feet

###################### Dynamic class of the robot
leg = leg_robotclass(t=t,q=q,qdot=qdot,p=p,u=tau,dt=dt,\
    urdf_file='/home/lenovo/python_simulation/python_simulation/legRBDL.urdf',param=None,terrain=None)
leg.slip_st_dur = len(GF_contact)*.001 



########################################## Homing initialize
leg.q[-1,0] = 0.0     #slide
leg.q[-1,1] = 0.0231  #hip
leg.q[-1,2] = 0.8     #thigh
leg.q[-1,3] = -1.2    #calf

###################### Control class of the robot
leg_control = leg_control(leg)

############# useful variables
Time_end = 4                                          # Total simulation time
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
y0 = 0

while leg.t[-1][0]<=Time_end:

    leg.set_input(tau)
    if leg.getContactFeet():
        leg()
        if first_check == 0:
            t_td = leg.t[-1][0]
            t_lo = t_td+len(GF_contact)*.001
            first_check=1
        stopflag = True
        slider_h = leg.get_com(body_part='h')
        TAU = leg_control.compute_TAU(leg.t[-1][0], t_td, t_lo)
        delta_time = leg.t[-1][0] - t_pre
        ################################################################ FOR PID CONTROLLER 
        ydot_d = (intp_y(TAU) - y0)/delta_time                # 0.01 is the step size in slip model
        y0 = intp_y(TAU)

        tau,e_pre, p_gain, d_gain = leg_control.stance(slider_h, leg.Jc,\
             intp_gf(TAU), intp_y(TAU),delta_time, e_pre,leg,ydot_d)
        
        
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
    leg()
    h_vec.append(leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))[2])
    COM = leg.get_com(calc_velocity=True,body_part='h')
    slider_h2 = leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))[2]
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
Plot_contact_force(leg, time_list, slip_gf_list)