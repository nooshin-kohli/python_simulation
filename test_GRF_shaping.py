# -*- coding: utf-8 -*-
"""
Created on Mon Aug 29 18:01:36 2016

@author: mshahbazi
"""



import sys
# sys.path.append('/home/mshahbazi/RBDL/build/python/')
# sys.path.append('/home/zahra/rbdl-dev/build/python')


import numpy as np
import time as ptime
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.interpolate import InterpolatedUnivariateSpline as intp


from leg_robotclass import ROBOT
from Centauro_ControlClass import Centauro_ControlClass
from Centauro_TaskSetClass import TaskSet
from Utils import Anim_leg, Plot_base_coordinate, Plot_foot_tip, \
Plot_contact_force, traj_plan, Plot_ff_fb , Plot_coms, Plot_stance_positions,\
Plot_stance_velocities,Plot_footstates, plot_steps
from PhyParam import PhyParam, write_lua

exec(compile(open('./swing_traj/swing_traj_bezier.py', "rb").read(), './swing_traj/swing_traj_bezier.py', 'exec'))
exec(compile(open('./functions.py', "rb").read(), './functions.py', 'exec'))

plt.close('all')

# assign robot parameters inside PhyParam class:
# param = PhyParam()




# generate lua model
# write_lua(param)
# lua_file = 'robot2d.lua'
mode = 'slider'

# initiate time array
t = np.array([0])
dt = .002  # step size

# initiate stats with dummy values
q = np.zeros(4) # joint position
qdot = np.zeros(4) # joint velocity
u = np.zeros((1, 0)) # control inputs


p = [[]] # the contact feet
# strange behavior when contact = [[1, 2]] and the legs are upright!!!!

# instanciate robot object:
cr = ROBOT(t, dt, q, p, mode, qdot, u)

#==============================================================================
# create initial configuration
#==============================================================================

#guess the initial config:
# angle1 = np.deg2rad(20)
# angle2 = np.deg2rad(100)
# angle3 = np.deg2rad(80)
cr.q=q
cr.qdot=qdot
# cr.q[-1, cr.joint.q_i('j1h')] = np.pi + angle1
# cr.q[-1, cr.joint.q_i('j2h')] = + angle2
# cr.q[-1, cr.joint.q_i('j3h')] = - angle3
# cr.q[-1, cr.joint.q_i('j1f')] = - angle1 - cr.q[-1, cr.joint.q_i('j1h')]
# cr.q[-1, cr.joint.q_i('j2f')] = - angle2
# cr.q[-1, cr.joint.q_i('j3f')] = + angle3

#cr.q[-1, cr.joint.q_i('reference_TY')] = - cr.CalcBodyToBase(cr.body.id('b3f'),
#                            np.array([cr.body.l_end, 0., 0.]))[1]
#cr.q[-1, cr.joint.q_i('reference_TY')] = 1
     
#==============================================================================
# Control paramters from SLIP model
#==============================================================================

#tt = 0.1489017
#tl = 0.35038529
#ta = 0.49774645
#x0_h = np.array([0, .75])
#xt_h = np.array([0.44670511,  0.64124773])
#xl_h = np.array([1.0088164 ,  0.64094974])
#xa_h = np.array([1.45052468e+00,   7.47463342e-01])
#x_foot_h = 0.72742089
#dx0_h = np.array([3, 0])

loaded = loadmat('grf-for-bounding-3.mat') 
tt = np.asscalar(loaded['tt'])
tl = np.asscalar(loaded['tl'])
ta = np.asscalar(loaded['ta'])
x0_h = loaded['x0'].flatten()[:2]
xt_h = loaded['xt'].flatten()[:2]
xl_h = loaded['xl'].flatten()[:2]
xa_h = loaded['xa'].flatten()[:2]
x_foot_h = np.asscalar(loaded['x_foot'])
dx0_h = loaded['x0'].flatten()[2:]

#x_h = loaded['x'].flatten()
#y_h = loaded['y'].flatten()
#dx_h = loaded['dx'].flatten()
#dy_h = loaded['dy'].flatten()




shift = 0.9*(cr.param.l1h + cr.param.l1f)
cr.slip_shift = shift
horiz_shift = np.array([shift, 0])


# This part only for the IK of IC
x0_f, xt_f, xl_f, xa_f, x_foot_f = \
x0_h + horiz_shift, xt_h + horiz_shift, xl_h + horiz_shift, \
xa_h + horiz_shift, x_foot_h + horiz_shift[0]
dx0_f = dx0_h

#x_f, y_f =  x_h + horiz_shift, y_h + horiz_shift

des_config = tt, tl, ta, x0_h, xt_h, xl_h, xa_h, x_foot_h, dx0_h
                         


#==============================================================================
# ### pass desired states to robot object ###
#==============================================================================
cr.slip_st_dur = tl - tt
cr.slip_sw_dur = ta - cr.slip_st_dur
cr.slip_st_length = xl_h - xt_h


cr.tl_h = -1/2*cr.slip_sw_dur

cr.slip_nominal_vel = dx0_h[0]

cr.slip_yt = xt_h[1]
cr.slip_yl = xl_h[1]

cr.xl_h = np.array([xl_h[0] - xa_h[0], xl_h[1]])
cr.foot_pose_h = x_foot_h - xa_h[0]

cr.xl_f = cr.xl_h + horiz_shift
cr.foot_pose_f = cr.foot_pose_h + cr.slip_shift

cr.slip_sw_xt = x_foot_h - xt_h[0]
cr.slip_st_xl = xl_h[0] - x_foot_h


grf_flag_time = 0

cr.slip_fun_grf_x_t = intp(loaded['t'], loaded['fx'], k=1)
cr.slip_fun_grf_y_t = intp(loaded['t'], loaded['fy'], k=1)
cr.slip_fun_y_t = intp(loaded['t'], loaded['y'], k=1)
cr.slip_fun_dy_t = intp(loaded['t'], loaded['dy'], k=1)
cr.slip_fun_x_t = intp(loaded['t'], loaded['x'] - loaded['x'][0, 0], k=1)
cr.slip_fun_dx_t = intp(loaded['t'], loaded['dx'], k=1)

cr.slip_fun_grf_x_s = intp(loaded['s'], loaded['fx'], k=1)
cr.slip_fun_grf_y_s = intp(loaded['s'], loaded['fy'], k=1)
cr.slip_fun_y_s = intp(loaded['s'], loaded['y'], k=1)
cr.slip_fun_dy_s = intp(loaded['s'], loaded['dy'], k=1)
cr.slip_fun_x_s = intp(loaded['s'], loaded['x'] - loaded['x'][0, 0], k=1)
cr.slip_fun_dx_s = intp(loaded['s'], loaded['dx'], k=1)


print(type(cr.slip_fun_grf_x_t))
print("#################################################################3")
 
#### compute IC config
nlayer = 2
# solve IK using root:
res = computeInitialConfig(cr, des_config, nlayer)
cr.q = res[:8].reshape(1, 8)
#cr.q[0, 1] = 4
cr.qdot = res[8:].reshape(1, 8)
#cr.qdot[0, 0] = 3
flag = 1
flag_zero_times = []
flag_one_times = []

Q_des = [cr.q[0].tolist()]
QD_des = [cr.qdot[0].tolist()]
QDD_des = []

X_des = []
XD_des = []
XDD_des = []

Foot_des_h = []
Foot_h = []
Foot_des_f = []
Foot_f = []

qddots = np.zeros((8))


S = [[0,0]]
fix = 0
rxs = []
rys = []       
pitchs = []

apex_coms = []
apex_vels = []

diff_coms = []
diff_lt = []

#==============================================================================
# Simulation
#==============================================================================
Time_end = ta * 5
# Time_end = 0.01

cr.coms_h = cr.get_com(body_part='h')
cr.coms_vel_h = cr.get_com(body_part='h',calc_velocity=True)[1]


#qdes = cr.q[0, 3:]
#qdes[2] += np.deg2rad(0)

u = np.zeros(5)
cr.u_feedbacks = [u]

tt_s = []
tl_s = []

nostop = True
while cr.t[-1][0]<=Time_end and nostop:
    u = np.zeros(5)
    
    try:
        if (cr.tt_h[0],cr.tt_f[0]) not in tt_s:
            tt_s.append((cr.tt_h[0],cr.tt_f[0]))
    except:
        pass
    if (cr.tl_h,cr.tl_f) not in tl_s:
        tl_s.append((cr.tl_h,cr.tl_f))


    #flags-> 1 when we are close to apex
    if abs(cr.t[-1][0]) % ta < 0.003:
        flag = 1
        print('im here')


    
    if cr.getContactFeet() != [1,2]:

        

        if 1 in cr.getContactFeet():

            u[0], u[1] = ctrl_stance(cr, 'h')

            pass
        else:
            u[0], u[1] = ctrl_swing(cr, 'h')

            
            #u[2] = ctrl_swing(cr,'hf')
            
        if 2 in cr.getContactFeet():
            u[3], u[4] = ctrl_stance(cr, 'f')   
            pass
        else:
            u[3], u[4] = ctrl_swing(cr, 'f')

    


        

        # if cr.getContactFeet() == []:
        #     u_fb = ctrl_state_feedbacks(cr)
        #     # print(u_fb.shape,u.shape)
        
        #     cr.u_feedbacks.append(u_fb)
        
        #     u += u_fb

    else:
        # if cr.t[-1][0] > ta:
        #     nostop = False
        
        # fix = 1
        if flag:
            
            X0 = np.array([cr.get_com(body_part='h')[:2],\
                          cr.get_com(body_part='f')[:2]]).reshape(4,1)
            tt = cr.t[-1][0]
            apex_coms.append(X0)
            _,vel_h = cr.get_com(body_part='h',calc_velocity=True)
            _,vel_f = cr.get_com(body_part='f',calc_velocity=True)
            dX = np.array([vel_h,vel_f])
            apex_vels.append(dX)
            flag = 0

            # print('im here')
            

        x_des,xd_des,xdd_des = slip_data(cr,'s')
        # V = 1
        # v1 = -0.1
        # # fval = 5
        # freq = (cr.t[-1][0]- tt)
        # xdd_des =  np.array([0,0]*2).reshape(4,1)
        # xd_des = np.array([0,0]*2).reshape(4,1)
        # x_des = X0 + np.array([0,0]*2).reshape(4,1)
        x_des += np.array([X0[0][0],0.0,X0[2][0],0.0]).reshape(4,1)
        X_des.append(x_des)
        XD_des.append(xd_des)
        XDD_des.append(xdd_des)
        
        # print('des_y_h', x_des[1][0],cr.get_com(body_part='h')[1])
        # print('des_y_f', x_des[3][0],cr.get_com(body_part='f')[1])
        # print('des_yd_h',xd_des[1][0],cr.get_com(body_part='h',calc_velocity=True)[1][1])
            
        u = ctrl_stance_ID(cr,x_des,xd_des,xdd_des)
        u = np.dot(cr.S,u).reshape(5)
        
    cr.coms_h = np.vstack((cr.coms_h,cr.get_com(body_part='h')))
    cr.coms_f = np.vstack((cr.coms_f,cr.get_com(body_part='f')))
    cr.coms_vel_h = np.vstack((cr.coms_vel_h,cr.get_com(body_part='h',calc_velocity=True)[1]))
    cr.coms_vel_f = np.vstack((cr.coms_vel_f,cr.get_com(body_part='f',calc_velocity=True)[1]))


        

    if abs(cr.t[-1][0] - tl) < 0.00010:

        Q0 = cr.q[-1]


    u_fb = ctrl_state_feedbacks(cr)
    # print(cr.foot_pose_f,'foot pose f')
    # print(u_fb.shape,u.shape)
    
    cr.u_feedbacks.append(u_fb)
    
    u += u_fb
    
    hip_h = cr.CalcBodyToBase(cr.body.id('b1h'),\
                              np.array([cr.param.lg1h,0,0]),\
                              update_kinematics=True,q=cr.q[-1,:],\
                              qdot=cr.qdot[-1,:])
    
    
    hip_f = cr.CalcBodyToBase(cr.body.id('b1f'),\
                                  np.array([cr.param.lg1f,0,0]),\
                                  update_kinematics=True,q=cr.q[-1,:],\
                                  qdot=cr.qdot[-1,:])
        
    h_com = cr.get_com(body_part='h')
    f_com = cr.get_com(body_part='f')
    pitch_com = np.rad2deg(np.arctan(f_com[1] - h_com[1]) / (f_com[0] - h_com[0]) )
    
    pitch = np.rad2deg(np.arctan(hip_f[1] - hip_h[1]) / (hip_f[0] - hip_h[0]))
    
    # plt.scatter(cr.t[-1][0],pitch,label='hip pitch')
    # plt.scatter(cr.t[-1][0],pitch_com,label='com pitch')
    # print('pitch slope = {} , com = {}'.format(pitch,pitch_com))
    pitchs.append([pitch,pitch_com])
    
    
    # cr.coms_h.append( cr.get_com(body_part='h'))
    # cr.coms_h = np.vstack((cr.coms_h,cr.get_com(body_part='h')))
    # cr.coms_f = np.vstack((cr.coms_f,cr.get_com(body_part='f')))
    # cr.coms_vel_h = np.vstack((cr.coms_vel_h,cr.get_com(body_part='h',calc_velocity=True)[1]))
    # cr.coms_vel_f = np.vstack((cr.coms_vel_f,cr.get_com(body_part='f',calc_velocity=True)[1]))

    
    cr.set_input(np.dot(cr.S.T, np.clip(u, -500, 500)))
    # cr.set_input(u)

    diff_coms.append(f_com[0] - h_com[0])
    if hasattr(cr,'tt_h'):
        diff_lt.append([cr.tl_h - cr.tt_h, cr.tl_f - cr.tt_f])
    else:
        diff_lt.append([0,0])
    
    cr()
    qddots = np.vstack((qddots,cr.qddot))
#    if len(cr.getContactFeet()) == 2 and len(cr.getContactFeet(True)[-2]) == 2:
#        nostop = False

X_des = np.array(X_des)         
XD_des = np.array(XD_des)    
tt_s = [k for k in tt_s if abs(k[0] - k[1]) <0.03]

apex_coms = np.array(apex_coms)
apex_vels = np.array(apex_vels)

diff_coms = np.array(diff_coms)
diff_lt = np.array(diff_lt)


"""
#==============================================================================
# Animation and plotting
#==============================================================================

# cr.u_feedbacks = np.array(cr.u_feedbacks, dtype=float)


robot_anim = Anim_Centauro(cr.model, cr.body, cr.joint, cr.q, cr.t) #TODO

# Plot_footstates(Foot_h,Foot_des_h)
# Plot_footstates(Foot_f,Foot_des_f,'f')
# plt.plot(cr.t[:], r2d(cr.x[:, cr.joint.q_i('knee_pitch_1')]))
#plt.plot(cr.t[:], r2d(cr.x[:, cr.joint.q_i('knee_pitch_2')]))
#plt.plot(cr.t[:], -r2d(cr.x[:, cr.joint.q_i('knee_pitch_3')]))
#plt.plot(cr.t[:], -r2d(cr.x[:, cr.joint.q_i('knee_pitch_4')]))
#plt.plot(cr.t[:], r2d(cr.q[:, cr.joint.q_i('j_arm2_7')]))


# plt.plot(cr.t, cr.u)

# Plot_base_coordinate(cr) 
# Plot_foot_tip(cr)   
Plot_coms(cr)
# error = Plot_stance_positions(cr,X_des)
# vel_error = Plot_stance_velocities(cr,XD_des)
Plot_contact_force(cr)
plot_steps(cr)
# Plot_ff_fb(cr, cr.u_feedbacks)

# plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_1') - 6])
#plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_2') - 6])
#plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_3') - 6])
#plt.plot(cr.t, cr.u[:, cr.joint.q_i(which_joint+'_4') - 6])

# plt.plot(cr.t,cr.coms_h[:,1],label='com h')
# plt.plot(cr.t,cr.coms_f[:,1],label='com f')
# plt.vlines(tt_s, 0, 1)
# plt.legend()

# dx_c = []
# dxx_c = []
# for i in range(len(XDD_des)):
#     try:
#         dx_c.append((X_des[i+1] - X_des[i])/cr.dt)
#         dxx_c.append((XD_des[i+1] - XD_des[i])/cr.dt)
#     except:
#         pass

# t = np.linspace(0,1,len(XDD_des))
# dx_c = np.array(dx_c)
# dxx_c = np.array(dxx_c)
# XDD_des = np.array(XDD_des)
# plt.plot(t[:-1],dx_c[:,0],label='computed')
# plt.plot(t,XD_des[:,0],label='desired')
# plt.legend()
# plt.show()
    
# x_d_0 = np.gradient(X_des[:,0].flatten(),dt)
# x_d_1 = np.gradient(X_des[:,1].flatten(),dt)
# x_d_2 = np.gradient(X_des[:,2].flatten(),dt)
# x_d_3 = np.gradient(X_des[:,3].flatten(),dt)

# plt.plot(XD_des[:,0],label='desired xd_h')
# plt.plot(x_d_0,label='computed xd_h')
# plt.legend()
# plt.show()
# plt.figure()
# plt.plot(XD_des[:,2],label='desired xd_f')
# plt.plot(x_d_2,label='computed xd_f')
# plt.legend()
# plt.show()
# plt.figure()
# plt.plot(XD_des[:,1],label='desired yd_h')
# plt.plot(x_d_1,label='computed yd_h')
# plt.legend()
# plt.show()
# plt.figure()
# plt.plot(XD_des[:,3],label='desired yd_f')
# plt.plot(x_d_3,label='computed yd_f')
# plt.legend()
# plt.show()

# plt.figure(figsize=(10,6))

# QDD_des = np.array(QDD_des)
# qddots = np.delete(qddots,1,0)
# error = QDD_des - qddots
# error2 = [abs(i) for i in error]
# error2 = [sum(i) for i in error2]
# error2= sum(error2)
# print(error2)
# for i in range(1,9):
#     plt.subplot(4,2,i)
#     plt.plot(cr.t[1:],qddots[:,i-1],label='qdd_robot')    
#     plt.plot(cr.t[1:],QDD_des[:,i-1],label='qdd_desired')


# plt.legend()
# plt.show()

def plot_pitch_slope(robot,pitches,tt_s,tl_s):
    pitchs = np.array(pitches)
    plt.figure(figsize=(10,5))
    plt.plot(robot.t[1:],pitchs[:,0],label='hip')
    plt.plot(robot.t[1:],pitchs[:,1],label='com')
    plt.legend()
    for tt in tt_s:
        plt.vlines(tt[0], -3, 1, label='tt_h')
        plt.vlines(tt[1], -3, 1, label='tt_f')
        
    for tl in tl_s:
        plt.vlines(tl[0],-5,1,label='tl_h')
        plt.vlines(tl[1],-5,1,label='tl_f')

    plt.hlines(0, 0, robot.t[-1][0])
    plt.show()
"""   