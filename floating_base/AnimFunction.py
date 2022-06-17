# -*- coding: utf-8 -*-
"""
Created on Fri Sep  2 16:46:47 2016

@author: user

"""
import sys
from os.path import expanduser
home = expanduser("~")
dir = home + '/projects/rbdl/build/python'
sys.path.append(dir)
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import rbdl
import numpy as np



def Anim_Centauro(model, body, joint, Q, time):

    plt.close('all')
    
    def update_joint(rbdl, model, body, joint, Q, i):
        jpose = {}
        q = Q[i, :]
#        if i == 1: print q[1]
        for j in joint.joints:
            if j[:9] != 'reference':
                child = joint.parent_child_body(j)[1]
                pose = rbdl.CalcBodyToBaseCoordinates(model, q, child, np.zeros(3))
                jpose[j] = pose
                
        # manually adding foottips positions
        pose = rbdl.CalcBodyToBaseCoordinates(model, q, body.id('calf'), \
        np.array([0., 0., -body.l_end]))
        jpose['ftip_1'] = pose
#        print pose
        
        return jpose
        
    def update_pairs(jpose, joint):
        data = []
        paired = list(joint.pairs())
        
        #manually adding ftip 'virtual' joints:
        paired.append(['calf', 'ftip_1']) 
        
        for i in range(len(paired)):
            p1 = jpose[paired[i][0]]
            p2 = jpose[paired[i][1]]
            data.append([[p1[0], p2[0]]])
            
#            if paired[i][0] == 'j_arm1_6': print p1
            
        return np.array(data, dtype = float)
        
    def update_limbs(i, Q, limbs, model, body, joint, time):
        jpose = update_joint(rbdl, model, body, joint, Q, i)
#        print jpose['hip_yaw_1'][1]
        data = update_pairs(jpose, joint)
        for line, dat in zip(limbs, data):
            line.set_data(dat[0:2, :])
            line.set_3d_properties(dat[2, :])
            
        time_text.set_text(time_template%(time[i]))
        
        return limbs, time_text
    
    
    # Attaching 3D axis to the figure
    fig = plt.figure(figsize=(20, 20))
    ax = p3.Axes3D(fig)

    ini_data = np.array([[0, 0], [0, 0], [0, 0]])
    limbs = [ax.plot(ini_data[0, :], ini_data[1, :], ini_data[2, :], '.-', lw = 3)[0] \
    for i in range(len(joint.pairs()) + 4)]
    
    time_text = ax.text(5, 1, 15, '', transform=ax.transAxes, fontsize=16)
    time_template = 'time: %.2f (s)'
    
    
    
    args = (Q, limbs, model, body, joint, time)
    robot_ani = animation.FuncAnimation(fig, update_limbs, len(Q), fargs=args,
                               interval=5, blit=False, repeat_delay=5000)
    
    
    ax.set_xlim3d([-1, 1.0])
    ax.set_xlabel('X')
    
    ax.set_ylim3d([-1, 1.])
    ax.set_ylabel('Y')
    
    ax.set_zlim3d([0.0, 2.])
    ax.set_zlabel('Z')

    ax.set_title('leg') 
    
#    plt.axis('equal')
    
    plt.show()
    
    return robot_ani