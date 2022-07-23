import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/rbdl/build/python'
sys.path.append(dir)
import rbdl
import numpy as np
from numpy.linalg import inv, pinv
# import rbdl

from leg_importmodel import BodyClass3d, JointClass3d
# import matplotlib.pyplot as plt
# from scipy.optimize import root, approx_fprime, minimize, fminbound
import scipy.integrate as integrate


class leg_control(object):

    def __init__(self,robot):
        """
        Control class for robot
        """
    
        self.q_pre = np.zeros(4)


    def compute_TAU(self, t_now, t_td, t_lo):
        TAU = (t_now - t_td)/(t_lo - t_td)
        return TAU

    def flight (self,q,qdot,p=15,d=0.1):
        q_des = [0, 0.0231, 0.8, -1.2]
        qdot_des = [0, 0, 0, 0]
        Kp = [[p,0,0,0],
              [0,p,0,0],
              [0,0,p,0],
              [0,0,0,p]]
        Kd = [[d,0,0,0],
               [0,d,0,0],
               [0,0,d,0],
               [0,0,0,d]]
        # e = q_des-q
        # edot = (e-e_pre)/delta_time
        # e_pre = e
        tau = (np.dot((q_des-q),Kp) + np.dot((qdot_des-qdot),Kd)).flatten()
        # tau.reshape(4,1)
        # tau.flatten()
        # print("tau shape in PID:", np.shape(tau))
        return tau


    def stance (self,slider_h, jc, GF, y_d, delta_time, e_pre,leg):        #slider_h, jc, GF, y_d
        
        q = leg.q[-1,:]
        qdot = leg.qdot[-1,:]

        p = 9000
        K_p = [[p, 0, 0],
               [0, p, 0],
               [0, 0, p]]

        d= 120
        K_d = [[d, 0, 0],
               [0, d, 0],
               [0, 0, d]]

        j_COM = leg.computeJacobianCOM('slider')
        COM = leg.get_com(calc_velocity=True,body_part='h')
        # print("type COM: ", COM[0][2])

        ################################# desired task space #################################
        desire_pos = np.array([COM[0][0], COM[0][1], y_d])
        # desire_vel = np.array([COM[1][0], COM[1][1], ydot_d])
        # desired_vel = desire_vel.reshape(3,1)

        ################################# calculating COM errors #################################
        e = desire_pos - COM[0]
        e_dot = (e - e_pre)/delta_time
        e_pre = e
        # edot = desire_vel - COM[1]

        # print("e:",e)
        # print("e_pre:",e_pre)
        # print("delta_time:",delta_time)
        

        ################################### task space to joint space ################################### 
        tau_pd = np.dot(j_COM.T,(np.dot(K_p,e)+np.dot(K_d, e_dot)))
        # print("jacoooooooob:",jc.T)
        # print("jacoob_com:",j_COM.T)
        J_t = jc.T
        # tau_pd = np.dot(J_t,(np.dot(K_p,e)+np.dot(K_d, e_dot)))
        G_F=[0,0,-GF]
        
        Tau_ff = np.dot(J_t, G_F)
        print("Tau_ff: ", Tau_ff)
        print("tau_pd: ",tau_pd)
        # print("D gain:",np.dot(K_d, e_dot))
        tau = (Tau_ff - tau_pd).flatten()
        # tau = (Tau_ff - np.dot(K_p, gain)- np.dot(Kd, e_dot)).flatten()
        p_gain = np.dot(K_p,e)
        d_gain = np.dot(K_d, e_dot)
        # tau.reshape(4,1)
        # tau.flatten()
        # print("tau shape in PID:", np.shape(tau))
        return tau, e_pre, p_gain[2], d_gain[2]