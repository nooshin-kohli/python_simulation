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
        
    def get_data(self):
        self.hip = np.array(self.hip)
        self.thigh = np.array(self.thigh)
        self.calf = np.array(self.calf)
        A = np.vstack((self.hip,self.thigh))
        result = np.vstack((A,self.calf))

        return result


data = DATA()
print(data.get_data())

