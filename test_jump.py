
from __future__ import division
from pickle import NONE

import sys

#sys.path.append('/home/mshahbazi/RBDL/build/python/')
# sys.path.append('/home/zahra/rbdl-dev/build/python')


import numpy as np
import time
import matplotlib.pyplot as plt
from scipy import linspace

# from leg_robotclass import ROBOT
from Centauro_RobotClass import Centauro_RobotClass
from scipy.interpolate import InterpolatedUnivariateSpline as intp
from leg_controlclass import leg_controlclass
from leg_tasksetclass import TaskSet
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
h = data_input(dt=.01, m=1.825, L0=0.456, k0=420)

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
# plt.plot(xs, intp_y(xs), 'r', lw=3, alpha=0.7)
# plt.show()

t = np.array([0])

dt = .005 # step size

# initiate stats with dummy values
q = np.zeros((1, 0)) # joint position
qdot = np.zeros((1, 0)) # joint velocity
# u = np.zeros((1, 0)) # control inputs
tau = np.zeros(4)



p = [[ ]] # the contact feet


cr = Centauro_RobotClass(t=t,q=q,qdot=qdot,p=p,u=tau,dt=dt,urdf_file='/home/kamiab/simulation-python-quadruped/python_simulation/legRBDL.urdf',param=None,terrain=None)

# cr.tt_h = 0.1 #TODO
# cr.tl_h = 0.3 #TODO
# cr.tt_f = 0.1 #TODO
# cr.tl_f = 0.3 #TODO
cr.slip_st_dur = 0.5 #TODO



############################################Homing initialize
cr.q[-1,0] = 0.0     #slide
cr.q[-1,1] = 0.0231  #hip
cr.q[-1,2] = 0.1     #thigh
cr.q[-1,3] = -.3     #calf




Time_end = 0.8

def compute_TAU(t_now, t_td, t_lo):
    TAU = (t_now - t_td)/(t_lo - t_td)
    return TAU

def pose (q,qdot,p=10):
    q_des = [100000, 0.0231, 0.1, -0.3]
    qdot_des = [0, 0, 0, 0]
    Kp = [[p,0,0,0],
          [0,p,0,0],
          [0,0,p,0],
          [0,0,0,p]]
    # Kd = [[d,0,0,0],
    #       [0,d,0,0],
    #       [0,0,d,0],
    #       [0,0,0,d]]
    tau = np.dot((q_des-q),Kp).flatten() #+ np.dot((qdot_des-qdot),Kd)
    # tau.reshape(4,1)
    # tau.flatten()
    # print("tau shape in PID:", np.shape(tau))
    return tau








def contact (slider_h, jc, GF, y_d):        #slider_h, jc, GF, y_d
    Kp = [[p,0,0,0],
          [0,p,0,0],
          [0,0,p,0],
          [0,0,0,p]]
    # Kd = [[d,0,0,0],
    #       [0,d,0,0],
    #       [0,0,d,0],
    #       [0,0,0,d]]

    G_F=[0,0,-GF]
    J_t = jc.T
    Tau_ff = np.dot(J_t, G_F)




    tau = Tau_ff.flatten() #+ np.dot((qdot_des-qdot),Kd)
    # tau.reshape(4,1)
    # tau.flatten()
    # print("tau shape in PID:", np.shape(tau))
    return tau




first_check=0
stopflag = False
while cr.t[-1][0]<=Time_end:
    # print(np.shape(np.dot(cr.S.T, np.zeros_like(cr.u[-1, :]))))
    cr.set_input(tau)
    if cr.getContactFeet():
        cr()
        if first_check == 0:
            t_td = cr.t[-1][0]
            t_lo = t_td+len(GF_contact)*.01
            first_check=1
            
        


        stopflag = True
        slider_h = cr.CalcBodyToBase(cr.model.GetBodyId('jump'),np.array([0.,0.,0.]))
        TAU = compute_TAU(cr.t[-1][0], t_td, t_lo)
        print("jacooooooooooooob",cr.Jc)

        tau = contact(slider_h, cr.Jc, intp_gf(TAU), intp_y(TAU))
        cr.tt_h = t_td #TODO
        cr.tl_h = t_lo #TODO
        
        # print(np.shape(cr.S))
        # print(np.shape(tau))
        # print(np.shape(cr.h))
        # tau.reshape((4,1))
        # print(np.shape(tau))
    else:
        tau = pose (cr.q[-1,:],cr.qdot[-1,:])
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