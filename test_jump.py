"""
Created on Sat Jul 2 20:42:48 2022

@author: Kamiab yazdi & Nooshin Kohli
"""
from __future__ import division
from pickle import NONE

import sys

#sys.path.append('/home/mshahbazi/RBDL/build/python/')
# sys.path.append('/home/zahra/rbdl-dev/build/python')


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
# import test
from object import data_input
##################### 
h = data_input(dt=.01, m=3.825, L0=0.362, k0=1500)

GF_contact, y_des_contact = extract_data(h.function(3)[0], h.function(3)[1])

# plt.plot(linspace(0,1,len(GF_contact)), GF_contact)
# plt.show()
##### interpolate y_des & GF_contact
tau_s = np.linspace(0, 1, len(GF_contact))
time_test= np.linspace (0, 3, len(h.function(3)[1]))
intp_gf = intp(tau_s, GF_contact, k=1)
intp_y = intp(tau_s, y_des_contact, k=1)
intp_y_comp =  intp(time_test, h.function(3)[1], k=1)

xs = np.linspace(0, 1, 1000)
plt.plot(xs, intp_gf(xs), 'r', lw=3, alpha=0.7)
plt.show()

plt.figure()
plt.plot(np.linspace(0,3, num=len(h.function(3)[1])),h.function(3)[1],'g')
plt.show()



t = np.array([0])

dt = .001 # step size

# initiate stats with dummy values
q = np.zeros((1, 0)) # joint position
qdot = np.zeros((1, 0)) # joint velocity
# u = np.zeros((1, 0)) # control inputs
tau = np.zeros(4)



p = [[ ]] # the contact feet


leg = leg_robotclass(t=t,q=q,qdot=qdot,p=p,u=tau,dt=dt,urdf_file='/home/lenovo/python_simulation/python_simulation/legRBDL.urdf',param=None,terrain=None)

# cr.tt_h = 0.1 #TODO
# cr.tl_h = 0.3 #TODO
# cr.tt_f = 0.1 #TODO
# cr.tl_f = 0.3 #TODO
leg.slip_st_dur = len(GF_contact)*.01 #TODO



############################################Homing initialize
leg.q[-1,0] = 0.0     #slide
leg.q[-1,1] = 0.0231  #hip
leg.q[-1,2] = 0.8     #thigh
leg.q[-1,3] = -1.5     #calf

leg_control = leg_control(leg)



Time_end =3

def compute_TAU(t_now, t_td, t_lo):
    TAU = (t_now - t_td)/(t_lo - t_td)
    return TAU

def pose (q,qdot,p=6,d=0.1):
    q_des = [0, 0.0231, 0.8, -1.5]
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


def contact (slider_h, jc, GF, y_d, delta_time):        #slider_h, jc, GF, y_d
    global e_pre
    p = 50
    K_p = [[0, 0, 0, 0],
           [0, p, 0, 0],
           [0, 0, p, 0],
           [0, 0, 0, p]]
    e = y_d - slider_h[2]
    gain = [0, 0, 0, e]


    print("e:",e)
    print("e_pre:",e_pre)
    print("delta_time:",delta_time)

    e_dot = (e - e_pre)/delta_time
    e_pre=e
    e_dot = [0,0,0,e_dot]

    d= 1
    Kd = [[d,0,0,0],
          [0,d,0,0],
          [0,0,d,0],
          [0,0,0,d]]

    G_F=[0,0,-GF]
    J_t = jc.T
    Tau_ff = np.dot(J_t, G_F)
    print("D gain:",np.dot(Kd, e_dot))
    tau = (Tau_ff - np.dot(K_p, gain)- np.dot(Kd, e_dot)).flatten()
    print("fooooot:", leg.computeFootState('h'))
    # tau.reshape(4,1)
    # tau.flatten()
    # print("tau shape in PID:", np.shape(tau))
    return tau, e_pre

t = []
time_pre = time.time()
h_vec = []
first_check=0
stopflag = False
global e_pre 
e_pre = 0
t_pre = 0
while leg.t[-1][0]<=Time_end:
    # print(np.shape(np.dot(cr.S.T, np.zeros_like(cr.u[-1, :]))))
    leg.set_input(tau)
    if leg.getContactFeet():
        leg()
        if first_check == 0:
            t_td = leg.t[-1][0]
            t_lo = t_td+len(GF_contact)*.01
            first_check=1
        stopflag = True
        slider_h = leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))
        # TAU = leg_control.compute_TAU(leg.t[-1][0], t_td, t_lo)
        TAU = compute_TAU(leg.t[-1][0], t_td, t_lo)
#        print("jacooooooooooooob",leg.Jc)
        delta_time = leg.t[-1][0] - t_pre
        # tau,e_pre = leg_control.contact(slider_h, leg.Jc, intp_gf(TAU), intp_y(TAU),delta_time)
        tau,e_pre = contact(slider_h, leg.Jc, intp_gf(TAU), intp_y(TAU),delta_time)
        tau[0] = 0
        print("tau at the contact: ", tau)
        leg.tt_h = t_td #TODO
        leg.tl_h = t_lo #TODO
        leg.slip_st_dur = leg.tl_h-leg.tt_h

        t_pre = leg.t[-1][0]
        
        # print(np.shape(cr.S))
        # print(np.shape(tau))
        # print(np.shape(cr.h))
        # tau.reshape((4,1))
        # print(np.shape(tau))
    else:
        # tau = leg_control.pose(leg.q[-1,:],leg.qdot[-1,:])
        tau = pose (leg.q[-1,:],leg.qdot[-1,:])
        tau[0] = 0
        first_check=0
        
        # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

#        print("tau at the pose: ", tau)
    leg()
    h_vec.append(leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))[2])
    time_now = leg.t[-1,:]
#    time_pre = time_now
    t.append(time_now)
    # if stopflag:
    #     print("after PID")
    #     print(tau)
    #     break

print("time:",np.shape(t))
print("h_vec: ",np.shape(h_vec))
plt.figure()
plt.title("slip height")
plt.plot(t,h_vec,'r')
plt.plot(np.linspace(0,3, num=len(h.function(3)[1])),h.function(3)[1],'g')
plt.legend(["simulation", "slip model"], loc ="upper right")
plt.show()
robot_anim = Anim_leg(leg.model, leg.body, leg.joint, leg.q, leg.t)
Plot_contact_force(leg)

#x_des = np.zeros(3)