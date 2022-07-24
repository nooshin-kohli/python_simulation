# -*- coding: utf-8 -*-
"""
Created on Thu May 7 2022


@author: Nooshin kohli & kamiab yazdi
"""


import numpy as np
import matplotlib.pyplot as plt
from Hopper1D import hopper1D
from scipy.interpolate import InterpolatedUnivariateSpline as intp

class data_input():
    def __init__(self,dt,m,L0,k0):
        self.L0 = L0
        self.dt = dt 
        self.mass = m
        self.k0 = k0
        self.g = 9.81
        self.alphaR = np.deg2rad(90) 
        self.foot = 0 
        self.mode_ini = 0

        self.q1 = np.array([[self.mode_ini,self.mass,self.L0,self.k0,self.g,self.alphaR,self.foot]])
        self.t1 = np.array([0])
        self.x1 = np.array([[0, 0.6,0,0]])     
        self.xdes1 = self.x1[-1, :]
	


        self.s1 = hopper1D(self.t1,self.x1,self.q1,self.xdes1,self.dt)
	

    def function(self,tend):
        while(self.s1.t[-1]<tend):
            self.s1()
        return -(self.s1.x[:, 1] - self.s1.q[0, 2])*self.s1.q[0, 3],self.s1.x[:, 1],self.s1.q[:, 0]
    
    def time_slip(self,tend):
        while(self.s1.t[-1]<tend):
            self.s1()
        return self.s1.t
    
    def extract_data(self,input_f,input_h):
        GF_contact=[]
        y_contact=[]
        time_step = np.linspace(0,3,3000)
        for i in time_step:
            if input_f(i)<0:
                pass
            else:
                while(input_f(i)>0):
                    GF_contact.append(input_f(i))
                    y_contact.append(input_h(i))
                    i+=0.001
                break
    
        return GF_contact, y_contact

    def interpolate(self,f,y):
        time_test= np.linspace (0, 3, len(f))
        intp_y_comp =  intp(time_test, y, k=1)
        intp_GF_comp =  intp(time_test, f, k=1)
        return intp_y_comp, intp_GF_comp
    
    def contact_time(self,input_f, input_h):
        GF_contact=[]
        y_contact=[]
#   
        for i in range(len(input_f)):
            if input_f[i]<0:
                pass
            else:
                while(input_f[i]>0):
                    GF_contact.append(input_f[i])
                    y_contact.append(input_h[i])
                    i+=1
                break
        if GF_contact[-1] == 0:
            pass
        else:
            GF_contact.append(0)
        return len(GF_contact)*0.01



###################################################### how to use ########################################### 
# print(h.time_slip(3))
# print("ground Force:")
# print(h.function(3)[0])
# print("vertical position:")
# print(h.function(3)[1])
# print("mode vec:")
# print(h.function(3)[2])

