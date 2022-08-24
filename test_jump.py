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
h = data_input(dt=.001, m=4, L0=0.366 , k0=1200)#  0.329       0.362
GF_contact, y_des_contact = extract_data(h.function(3)[0], h.function(3)[1])

################################## interpolate y_des & GF_contact
tau_s = np.linspace(0, 1, len(GF_contact))
time_test= np.linspace (0, 3, len(h.function(3)[1]))
intp_gf = intp(tau_s, GF_contact, k=1)
intp_y = intp(tau_s, y_des_contact, k=1)
intp_y_comp =  intp(time_test, h.function(3)[1], k=1)

################################# plotting ground force from slip model
#xs = np.linspace(0, 1, 1000)
#plt.plot(xs, intp_gf(xs), 'r', lw=3, alpha=0.7)
#plt.show()

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
q_hip = []
q_thigh = []
q_calf = []
q_hip_t = []
q_thigh_t = []
q_calf_t = []
hip_command = []
thigh_command = []
calf_command = []

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
        q_hip.append(leg.q[-1,1])
        q_thigh.append(leg.q[-1,2])
        q_calf.append(leg.q[-1,3])
        q_hip_t.append(leg.q[-1,1])
        q_thigh_t.append(leg.q[-1,2])
        q_calf_t.append(leg.q[-1,3])
        hip_command.append(leg.q[-1,1])
        thigh_command.append(leg.q[-1,2])
        calf_command.append(leg.q[-1,3])
        
        
    else:
        tau = leg_control.flight(leg.q[-1,:],leg.qdot[-1,:])
        slip_gf_list.append(0)
        p_gain_list.append(0)
        d_gain_list.append(0)
        time_list.append(leg.t[-1][0])
        tau[0] = 0
        first_check=0
        t_pre = leg.t[-1][0]
        q_hip_t.append(0.0231)
        q_thigh_t.append(0.8)
        q_calf_t.append(-1.2)
        q_hip.append(leg.q[-1,1])
        q_thigh.append(leg.q[-1,2])
        q_calf.append(leg.q[-1,3])
        hip_command.append(0.0231)
        thigh_command.append(0.8)
        calf_command.append(-1.2)
    leg()
    # q_calf.append(leg.q[-1,3])
    h_vec.append(leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))[2])
    COM = leg.get_com(calc_velocity=True,body_part='h')
    slider_h2 = leg.CalcBodyToBase(leg.model.GetBodyId('jump'),np.array([0.,0.,0.]))[2]
    time_now = leg.t[-1,:]
    t.append(time_now)

# print("q is: ")   
# print(leg.q)


from scipy.signal import find_peaks
import xlsxwriter

global row, col

row=1
col=0

workbook = xlsxwriter.Workbook('/home/lenovo/python_simulation/tuning_result.xlsx')

#worksheet name
worksheet = workbook.add_worksheet("results")

worksheet.write(0, 0, "t")
worksheet.write(0, 1, "hip")
worksheet.write(0, 2, "thigh")
worksheet.write(0, 3, "calf")

#plt.figure()
#plt.plot(t,q_hip)
#plt.plot(t,hip_command)
#plt.title("hip position")
#plt.legend(["actual", "command"], loc ="upper left")
from scipy.signal import find_peaks
peaks, _ = find_peaks(q_hip_t, height=0)
print("*************************************************", peaks)
#plt.figure()
#plt.plot(t,q_thigh)
#plt.plot(t,thigh_command)
#plt.title("thigh position")
#plt.legend(["actual", "command"], loc ="upper left")
#plt.figure()
#plt.plot(t,q_calf)
#plt.plot(t,calf_command)
#plt.title("calf position")
#plt.legend(["actual", "command"], loc ="upper left")
#plt.figure()
#plt.title("slip height")
#plt.plot(t,h_vec,'r')
#plt.plot(np.linspace(0,3, num=len(h.function(3)[1])),h.function(3)[1],'g')
#plt.legend(["simulation", "slip model"], loc ="upper right")

#plt.figure()
#plt.plot(time_list,p_gain_list,'r')
#plt.plot(time_list, d_gain_list,'b')
#plt.legend(["p_gain", "d_gain"], loc ="upper right")
#plt.show()
hardware_hip=[]
r = 195
L0=0.366
time_xl = []
while 1:
    if q_hip_t[r] == 0.0231:
        break
    else:
        time_xl.append(t[r][-1]) 
        r = r+1
while(1):
    if (255<r<579):
        time_xl.append(t[r][-1])
        r = r+1
    else:
        break   
print("time", time_xl)
r =195
while 1:
    if q_hip_t[r] == 0.0231:
        break
    else:
        hardware_hip.append(q_hip_t[r]) 
        r = r+1
while(1):
    if (255<r<579):
        hardware_hip.append(q_hip_t[r])
        r = r+1
    else:
        break
# while(1):
#     if (302<r<783):
#         hardware_hip.append(q_hip_t[r])
#         r = r+1
#     else:
#         break

hardware_thigh = []
r=195
while 1:
    if q_thigh_t[r] ==0.8:
        break
    else:
        hardware_thigh.append(q_thigh_t[r]) 
        r = r+1
while (1):
    if (255<r<579):
        hardware_thigh.append(q_thigh_t[r])
        r = r+1
    else:
        break
hardware_calf = []
# r=259
r = 195
while 1:
    if q_calf_t[r] == -1.2:
        print("final data in stance mode:", r)
        break
    else:
        hardware_calf.append(q_calf_t[r]) 
        r = r+1

while(1):
    if (255<r<579):
        hardware_calf.append(q_calf_t[r])
        r = r+1
    else:
        break

print("############# hip:",hardware_hip)
print("############# thigh",hardware_thigh)
print("############# calf",hardware_calf)
plt.figure()
plt.plot(time_xl,hardware_hip)
plt.title("hip in comp")
plt.figure()
plt.plot(time_xl,hardware_thigh)
plt.title("thigh in comp")
plt.figure()
plt.plot(time_xl,hardware_calf)
plt.title("calf in comp")

################################################## data saving
# for data in range(len(hardware_hip)):
#     worksheet.write(row, col, time_xl[data])
#     worksheet.write(row, col+1, hardware_hip[data])
#     worksheet.write(row, col+2, hardware_thigh[data])
#     worksheet.write(row, col+3, hardware_calf[data])
#     row = row+1
# workbook.close()  
plt.show()

q_hip = np.array(q_hip_t)
q_max_hip = q_hip.max()
# print("q_max_hip")
# print(q_max_hip)

# mask = np.array(q_hip_t) == q_max_hip
# t_m = np.where(mask)
# print("t_m:", t_m[0][-1])
# print("first maximum Compression accurs in time: ",t[t_m[0][-1]][-1])

q_thigh = np.array(q_thigh_t)
q_max_thigh = q_thigh.max()
# print("q_max_tigh")
# print(q_max_thigh)

q_calf = np.array(q_calf_t)
q_min_calf = q_calf.min()
# print("q_min_calf")
# print(q_min_calf)
# q_final = [q_max_hip,q_max_thigh,q_min_calf]
q_hip_hardware =[]

    
# for i in range(len(t)):

# for h in h_vec:
#     if h == L0:
#         print(r)
#     r = r+1    
        
# for q in q_hip:
#     if not q==0:
#         q_hip_hardware.append(q)

# print(q_hip_hardware[259])    

# print("In  maximum compression joint states are: ", q_final)
robot_anim = Anim_leg(leg.model, leg.body, leg.joint, leg.q, leg.t)
Plot_contact_force(leg, time_list, slip_gf_list)

