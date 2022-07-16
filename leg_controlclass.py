import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/rbdl/build/python'
sys.path.append(dir)
import rbdl
import numpy as np
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


    def compute_TAU(self, t_now, t_td, t_lo):
        TAU = (t_now - t_td)/(t_lo - t_td)
        return TAU

    def pose (self,q,qdot,p=3.5,d=0.1):
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


    def contact (slef,slider_h, jc, GF, y_d, delta_time):        #slider_h, jc, GF, y_d
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
        # tau.reshape(4,1)
        # tau.flatten()
        # print("tau shape in PID:", np.shape(tau))
        return tau, e_pre
