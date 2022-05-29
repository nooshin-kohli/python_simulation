#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 12 15:36:29 2017

@author: zahra
"""

class PhyParam():

    l_hip = 0.093
    l_thigh = 0.21183
    l_calf = 0.240
    
    lg1h = 1.*l_hip
    lg2h = 1./2*l_thigh
    lg3h = 1./2*l_calf
    w_hip = 0.08
    w_thigh = 0.07125
    w_calf = 0.02
    
    
    g0 = 9.81
    
    m1h = 17.5
    m2h = 1.5
    m3h = 1.
    m1f = 27.5
    m2f = m2h
    m3f = m3h
    
    
def write_lua(p):
    with open('param.lua', 'w') as f:
        f.write('grav = {{0., {g}, 0.}}\n\n'.format(g = -p.g0))
        
        f.write('l = {{ l1h = {l1h}, l2h = {l2h}, l3h = {l3h}, \
        l1f = {l1f}, l2f = {l2f}, l3f = {l3f}}}\n\n'\
        .format(l1h = p.l1h, l2h = p.l2h, l3h = p.l3h, l1f = p.l1f, l2f = p.l2f, l3f = p.l3f))
        
        f.write('w = {{ w1h = {w1h}, w2h = {w2h}, w3h = {w3h}, \
        w1f = {w1f}, w2f = {w2f}, w3f = {w3f}}}\n\n'\
        .format(w1h = p.w1h, w2h = p.w2h, w3h = p.w3h, w1f = p.w1f, w2f = p.w2f, w3f = p.w3f))
        
        f.write('m = {{ m1h = {m1h}, m2h = {m2h}, m3h = {m3h}, \
        m1f = {m1f}, m2f = {m2f}, m3f = {m3f}}}\n\n'\
        .format(m1h = p.m1h, m2h = p.m2h, m3h = p.m3h, m1f = p.m1f, m2f = p.m2f, m3f = p.m3f))
        
        f.write('lg = {{ lg1h = {lg1h}, lg2h = {lg2h}, lg3h = {lg3h}, \
        lg1f = {lg1f}, lg2f = {lg2f}, lg3f = {lg3f}}}\n\n'\
        .format(lg1h = p.lg1h, lg2h = p.lg2h, lg3h = p.lg3h,\
                lg1f = p.lg1f, lg2f = p.lg2f, lg3f = p.lg3f))