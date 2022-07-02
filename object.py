# -*- coding: utf-8 -*-
"""
Created on Thu May 7 2022

"""


import numpy as np
import matplotlib.pyplot as plt
from Hopper1D import hopper1D

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
        self.x1 = np.array([[0, 0.9,0,0]])      #TODO: ask about this!!!!!!!!!!!!!!!!!
        self.xdes1 = self.x1[-1, :]
	


        self.s1 = hopper1D(self.t1,self.x1,self.q1,self.xdes1,self.dt)
	

    def function(self,tend):
        while(self.s1.t[-1]<tend):
            self.s1()
        return -(self.s1.x[:, 1] - self.s1.q[0, 2])*self.s1.q[0, 3],self.s1.x[:, 1],self.s1.q[:, 0]



###################################################### how to use ########################################### 

#h = data_input(dt=.01,m=2.643,L0=0.445,k0=300)
#print("ground Force:")
#print(h.function(3)[0])
#print("vertical position:")
#print(h.function(3)[1])
#print("mode vec:")
#print(h.function(3)[2])

