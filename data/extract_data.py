#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 24 23:38:09 2022

@author: Nooshin Kohli
"""
import matplotlib.pyplot as plt
from scipy.misc import electrocardiogram
from scipy.signal import find_peaks
import numpy as np



class DATA():
    def __init__(self):
        with open("hip_46.txt","r") as file:
                self.hip = eval(file.readline())
        with open("thigh_46.txt","r") as file:
                self.thigh = eval(file.readline())
        with open("calf_46.txt","r") as file:
                self.calf = eval(file.readline())
                
        with open("hip_vel_46.txt","r") as file:
                self.hip_vel = eval(file.readline())
        with open("thigh_vel_46.txt","r") as file:
                self.thigh_vel = eval(file.readline())
        with open("calf_vel_46.txt","r") as file:
                self.calf_vel = eval(file.readline()) 
        #########################################################        
        with open("time_f.txt", "r") as file:
            self.time_f = eval(file.readline())
        with open("hip_f.txt","r") as file:
            self.hip_f = eval(file.readline())
        with open("thigh_f.txt","r") as file:
            self.thigh_f = eval(file.readline())
        with open("calf_f.txt","r") as file:
            self.calf_f = eval(file.readline())
        
        with open("hip_vel_f.txt","r") as file:
            self.hip_f_vel = eval(file.readline())
        with open("thigh_vel_f.txt","r") as file:
            self.thigh_f_vel = eval(file.readline())
        with open("calf_vel_f.txt","r") as file:
            self.calf_f_vel = eval(file.readline())
            
    def get_pose_data(self):
        self.hip = np.array(self.hip)
        self.thigh = np.array(self.thigh)
        self.calf = np.array(self.calf)
        A = np.vstack((self.hip,self.thigh))
        result = np.vstack((A,self.calf))
        return result
    
    def get_vel_data(self):
        self.hip_vel = np.array(self.hip_vel)
        self.thigh_vel = np.array(self.thigh_vel)
        self.calf_vel = np.array(self.calf_vel)
        A = np.vstack((self.hip_vel,self.thigh_vel))
        result = np.vstack((A,self.calf_vel))
        return result
    
    def get_pose_data_f(self):
        self.hip_f = np.array(self.hip_f)
        self.thigh_f = np.array(self.thigh_f)
        self.calf_f = np.array(self.calf_f)
        A = np.vstack((self.hip_f,self.thigh_f))
        result = np.vstack((A,self.calf_f))
        return result
    def get_vel_data_f(self):
        self.hip_f_vel = np.array(self.hip_f_vel)
        self.thigh_f_vel = np.array(self.thigh_f_vel)
        self.calf_f_vel = np.array(self.calf_f_vel)
        A = np.vstack((self.hip_f_vel,self.thigh_f_vel))
        result = np.vstack((A,self.calf_f_vel))
        return result
    

################################# How to use this class:
#data = DATA()
#print(data.get_pose_data())
#print("**************************")
#print(data.get_pose_data_f())
#print("**************************")
#print(data.get_vel_data())
#print("**************************")
#print(data.get_vel_data_f())
