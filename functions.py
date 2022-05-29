#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  3 18:00:04 2017

@author: mshahbazi
"""

import numpy as np
import copy
from leg_robotclass import ROBOT
from leg_controlclass import leg_controlclass
from scipy.optimize import root

#==============================================================================
# swing control
#==============================================================================
def ctrl_swing(robot, body_part,q,qdot):
    kp = np.diag([2500, 2500,2500])
    kd = kp/10
#    desired_param = t, tt, tl, ta, xt, xl, x_foot, dx0
#TODO: in get_com you should pass q and qdot
    r, dr = robot.get_com(body_part = 'slider', calc_velocity = True, q=q, qdot=qdot)
    
    x_des, dx_des = computeDesiredFootState(robot, body_part, r, dr)
    x, dx = robot.computeFootState(body_part, calc_velocity = True)
    

    e = x_des - x
    de = dx_des - dx

#    print body_part, e
#    print 'de', de
    F = np.dot(kp, e) + np.dot(kd, de)

#TODO: QUES??????
    # J23 = robot.computeJacobian23(body_part = body_part)
    J34 = robot.calcJc(robot.q)
    return np.dot(J34.T, F)
#==============================================================================
#  stance control
#============================================================================== 
def ctrl_stance(robot, body_part,only_force=False):

    
    t = robot.t[-1][0] + robot.dt
    if body_part == 'slider': 
        tt = robot.tt_h
        xt = robot.xt_h[0]
        xl_expected = robot.foot_pose_h + robot.slip_st_xl
        mg = (robot.mass_hip + robot.mass_thigh + robot.masss_calf)*robot.g0

    else: print("body part does not exist!!!")
    tl = tt + robot.slip_st_dur

    tau = (t - tt)/(tl - tt) 
    
#    h, dh = computeYShift(t, tt, tl, robot.slip_yt, robot.slip_yl)
    r, dr = robot.get_com(body_part=body_part, calc_velocity=True)
    
    s = (r[0] - xt)/(xl_expected - xt)
    
#    print 'tau: ', tau
#    print 's: ', s
    
    
    if grf_flag_time:   
        fx = mg*robot.slip_fun_grf_x_t(tau)
        fz = mg*robot.slip_fun_grf_y_t(tau)
    else:
        fx = mg*robot.slip_fun_grf_x_s(s)
        fz = mg*robot.slip_fun_grf_y_s(s)
   
    
#    if body_part == 'f':
#        print tau, fx
    F = np.array([fx, 0.0, fz]).flatten()
    if only_force:
        return F
    
    J34 = robot.calcJc(robot.q)
    # J23 = robot.computeJacobian23(body_part = body_part)
    tau = np.dot(J34.T, F)

            
    return -tau.flatten()



#==============================================================================
# Inverse Dynamics Control
#==============================================================================
def ctrl_stance_ID(robot,x_des,xd_des,xdd_des):
    

    
    xdd_ref = compute_xddref(robot,x_des,xd_des,xdd_des)
    # XDD_ref.append(xdd_ref)
    qdd_des = compute_qddot_des(robot,xdd_ref)
    # planer = Centauro_PlannerClass(robot)
    # x_des = np.append(x_des.flatten()[:2],0)
    # xd_des = np.append(x_des.flatten()[:2],0)
    # xdd_des = np.append(x_des.flatten()[:2],0)
    # qdd_des = planer.qddot_from_xbase_no_hierarchi(xddot_des=xdd_des,x_des=x_des,xdot_des=xd_des)

    control = leg_controlclass(robot)

    
    
    # qdot_des = qdd_des * robot.dt + QD_des[-1]
    qdot_des = qdd_des * robot.dt + robot.qdot

    q_des =  qdot_des * robot.dt + robot.q
    
    

    QDD_des.append(qdd_des)
    QD_des.append(qdot_des)
    Q_des.append(q_des)
    
    # return control.Gravity_Compensation(q_des,qdot_des)
    
    # control1 = Control(robot)
    # tau = control1.compute_torque(qdd_des,qdot_des,q_des)
    
    # w = np.eye(5)
    # W[2][2] = 100
    #################################################################################TODO: it was b1h, i put jump ???
    r_h, dr_h = robot.CalcBodyToBase(robot.body.id('jump'),\
                              np.array([0,0,0]),\
                              update_kinematics=True,q=robot.q,\
                              qdot=robot.qdot,calc_velocity=True)
    
    
    # r_f, dr_f = robot.CalcBodyToBase(robot.body.id('b1f'),\
    #                           np.array([robot.param.lg1f,0,0]),\
    #                           update_kinematics=True,q=robot.q[-1,:],\
    #                           qdot=robot.qdot[-1,:],calc_velocity=True)
    
    # kp0 = 100
    # kd0 = 10
    # tau0 = np.ones(5)*100
    # tau0 = ctrl_nullspace(robot)
    # tau0 = np.zeros(5)
    # print(tau0)
    # w,b = track_grf(robot)
    # tau0 = -b
    
    tau = control.InvDyn_qr(q_des, qdot_des, qdd_des)
    return tau.flatten()

def ctrl_nullspace(robot):
    u = np.zeros(5)
    u[2] = ctrl_torso(robot,robot.slip_shift)

    y_des, yd_des = robot.CalcBodyToBase(robot.body.id('b1f'),
                                 np.array([robot.param.l1f,0,0]),
                                 calc_velocity = True)
    
    y,yd = robot.CalcBodyToBase(robot.body.id('b1h'),
                                 np.array([robot.param.l1h,0,0]),
                                 calc_velocity = True)
    
    kp, kd = 400,100
    F = kp*(y_des - y + 0.07) + kd*(yd_des - yd)
    F = F[:2]
    J = robot.calcJc(robot.q)
    u[:2] = - np.dot(J.T,F)
    return u

def track_grf(robot):
    
    landa = np.zeros(4)
    u_h = ctrl_stance(robot, 'slider',True)
    # u_f = ctrl_stance(robot, 'f',True)
    
    landa[:2] = u_h
    landa[2:] = u_f
    landa.reshape(4,1)
    k = np.diag((1,1000,2000,1000))
    
    cforce = robot.cforce[-1]
    cost = 1/2 * np.matmul(np.matmul((cforce - landa).T ,k),cforce - landa)
    # print(cost)
    W_l = k
    b_l = - np.matmul(landa.T,k)
    return W_l,b_l
    

    
    
    

def compute_xddref(robot,x_des,xd_des,xdd_des):

    com_h , com_vel_h = robot.get_com(body_part='slider', calc_velocity=True)
    # com_f , com_vel_f = robot.get_com(body_part='f', calc_velocity=True)
    
    x_actual_h = np.array(com_h)
    xd_actual_h = np.array(com_vel_h)

    # x_actual_f = np.array(com_f[:2]).reshape(1,2)
    # xd_actual_f = np.array(com_vel_f[:2]).reshape(1,2)
    
    x_actual = x_actual_h
    xd_actual = xd_actual_h
    # x_actual = np.concatenate((x_actual_h,x_actual_f)).reshape(4,1)
    # xd_actual = np.concatenate((xd_actual_h,xd_actual_f)).reshape(4,1)
    
    
    xdd_ref = pd_helper(xdd_des.flatten(),(xd_des - xd_actual).flatten() , (x_des - x_actual).flatten())
    
    # print("here xdd ref {} \n derivatives".format(xdd_ref))
    return xdd_ref

def pd_helper(acceleration,velocity_variation,position_variation,kp=10000,kd=500):
    if acceleration.shape == velocity_variation.shape == position_variation.shape:
        # print(acceleration.shape,velocity_variation.shape,position_variation.shape)
        return (acceleration + kd * velocity_variation + kp * position_variation).reshape(acceleration.shape[0],1)
    else:
        raise Exception('matrices have different shapes')


def compute_qddot_des(robot,xdd_ref):

    jdqd = robot.calcJdQd()
    # jdqd = np.vstack((jdqd_h,jdqd_f)).reshape(4,1)
    
    jcdqd = -robot.CalcGamma(robot.getContactFeet(),robot.q,robot.qdot)
    
    # Jc = robot.Jc.reshape(4,8)
    Jc = robot.Jc_from_cpoints(robot.q, robot.getContactFeet())
    
    jG = robot.computeJacobianCOM('slider')
    
     


    # J = np.vstack((Jc,jG))
    aux = np.vstack((np.zeros((4,1)),xdd_ref.reshape(4,1))) - \
        np.vstack((jcdqd,jdqd))
    
    # print(J.shape,aux.shape,'shapes')
    


    # qdd_ref = np.matmul(np.linalg.pinv(jG),(xdd_ref-jdqd)).flatten()
    qdd_ref = np.matmul(np.linalg.pinv(J),aux).flatten()
    # print("qdd ref is {}".format(qdd_ref.shape))
    
    return qdd_ref

def compute_xddot_des(robot,qdd_des):
    jGh = robot.computeJacobianCOM('h')
    jGf = robot.computeJacobianCOM('f')
    
    jG = np.vstack((jGh,jGf)).reshape(4,8) 
    
    jdqd_h,jdqd_f = robot.CalcJgdotQdot()
    jdqd = np.vstack((jdqd_h,jdqd_f)).reshape(4,1) 
    
    return jdqd + np.matmul(jG,np.array(qdd_des).reshape(8,1)).reshape(4,1)
    



def slip_data(robot,base='s'):
    
    if base == 't':
        
        t = robot.t[-1][0] + robot.dt
        tt = max(robot.tt_h,robot.tt_f)
        tl = tt + robot.slip_st_dur
        
        
        
        t_s = (t - tt) / (tl - tt)
        # print('ts = {}'.format(t_s)) ##SLIP did'nt complete
        x = np.asscalar(robot.slip_fun_x_t(t_s))
        dx = np.asscalar(robot.slip_fun_dx_t(t_s))
        
        y = np.asscalar(robot.slip_fun_y_t(t_s))
        dy = np.asscalar(robot.slip_fun_dy_t(t_s))
        
        xx = [x,y] *2
        xx_d = [dx,dy] * 2
        xx_dd = [0,0] * 2
        
        x = np.array(xx).reshape(4,1)
        
        xd = np.array(xx_d).reshape(4,1)
        xdd = np.array(xx_dd).reshape(4,1)
    elif base == 's':
        x_h = robot.get_com(body_part='h')
        x_f = robot.get_com(body_part='f')
        xt_h = X0[0][0]
        xt_f = X0[2][0]
        xl_h = xt_h + robot.slip_st_length[0]
        xl_f = xt_f + robot.slip_st_length[0]
        
        s_h = (x_h[0] - xt_h) / (xl_h - xt_h)
        s_f = (x_f[0] - xt_f) / (xl_f - xt_f)
        # s_h = s_f
        
        S.append([s_h,s_f])
        # print('s_h = {}, s_f = {}'.format(s_h,s_f))
        x_h = np.asscalar(robot.slip_fun_x_s(s_h))
        x_f = np.asscalar(robot.slip_fun_x_s(s_f))
        dx_h = np.asscalar(robot.slip_fun_dx_s(s_h))
        dx_f = np.asscalar(robot.slip_fun_dx_s(s_f))
        
        y_h = np.asscalar(robot.slip_fun_y_s(s_h)) 
        y_f = np.asscalar(robot.slip_fun_y_s(s_f)) 
        dy_h = np.asscalar(robot.slip_fun_dy_s(s_h)) 
        dy_f = np.asscalar(robot.slip_fun_dy_s(s_f))
        
        x = np.array([x_h,y_h,x_f,y_f]).reshape(4,1)
        xd = np.array([dx_h,dy_h,dx_f,dy_f]).reshape(4,1)
        
        dt = robot.dt
        
        xdd_h = (dx_h - np.asscalar(robot.slip_fun_dx_s(S[-2][0]))) / dt
        xdd_f = (dx_f - np.asscalar(robot.slip_fun_dx_s(S[-2][1]))) / dt
        
        ydd_h = (dy_h - np.asscalar(robot.slip_fun_dy_s(S[-2][0]))) / dt 
        ydd_f = (dy_f - np.asscalar(robot.slip_fun_dy_s(S[-2][1]))) / dt 
        xdd = np.array([xdd_h,ydd_h,xdd_f,ydd_f]).reshape(4,1)
    else:
        raise Exception("slip data can be achived by base t or s. {} inserted".format(base))
    
    return x,xd,xdd



    
    
    
#==============================================================================
# state_feedback control
#==============================================================================




def ctrl_state_feedbacks(robot):
    
    t = robot.t[-1][0] + robot.dt

    c = 1
    kp, kd, kv = c*20000, c*2000, 1000*0
    kthp, kthd =  5000*0, 500*0
    kthp_dist, kthd_dist = 300*0, 30*0
    
    th_des, dth_des = 0, 0
    
    p = robot.getContactFeet() # stance legs
    
    J = np.zeros((len(p)*2 + 1, robot.qdim))
    F_com = np.zeros((len(p)*2 + 1, 1))
#    J = np.zeros((5, robot.qdim))
#    F_com = np.zeros((5, 1))

    # r_h, dr_h = robot.get_com(body_part='h', calc_velocity=True)
    # r_f, dr_f = robot.get_com(body_part='f', calc_velocity=True)
    
    
    r_h, dr_h = robot.CalcBodyToBase(robot.body.id('b1h'),\
                              np.array([robot.param.lg1h,0,0]),\
                              update_kinematics=True,q=robot.q[-1,:],\
                              qdot=robot.qdot[-1,:],calc_velocity=True)
    
    
    r_f, dr_f = robot.CalcBodyToBase(robot.body.id('b1f'),\
                              np.array([robot.param.lg1f,0,0]),\
                              update_kinematics=True,q=robot.q[-1,:],\
                              qdot=robot.qdot[-1,:],calc_velocity=True)
    
    d = r_f - r_h
    ddot = dr_f - dr_h
    
    J_th = computeJacobianTh(robot)
    J[-1, 5] = J_th[5] # always actuate torso
    
    th = np.arctan2(d[1], d[0])
    dth = np.dot(J_th, robot.q[-1, :])
#    print np.rad2deg(th), np.rad2deg(dth)
       
    d_norm = np.linalg.norm(d)
    dd_norm = np.dot(d, ddot)/d_norm

    # for the pitch
    F_com[-1] = kthp*(th_des - th) + kthd*(dth_des - dth) + \
                kthp_dist*(robot.slip_shift - d_norm) + kthd_dist*(- dd_norm)
                
    # print(F_com)
                
#    F_com[-1] = - F_com[-1]
    
    if 1 in p: 
        # print('im here in state feedback')
        tt = robot.tt_h
        tl = tt + robot.slip_st_dur
        xt = robot.xt_h[0]
        xl_expected = robot.foot_pose_h + robot.slip_st_xl
        
        r, dr = r_h, dr_h
        
        ### kind of constant height feedback
#        h, dh = computeYShift(t, tt, tl, robot.slip_yt, robot.slip_yl)

        ### exact slip feedback
        tau = (t - tt)/(tl - tt)
        s = (r[0] - xt)/(xl_expected - xt)
        # s = S[-1][0]
        
#        print 'h', tau, s
        
        if grf_flag_time:
            h, dh = robot.slip_fun_y_t(tau), robot.slip_fun_dy_t(tau)  
            xx, dxx = robot.slip_fun_x_t(tau) + xt, robot.slip_fun_dx_t(tau)
        else:
            h, dh = robot.slip_fun_y_s(s), robot.slip_fun_dy_s(s)  
            xx, dxx = robot.slip_fun_x_s(s) + xt, robot.slip_fun_dx_s(s)
        
        
        J[-1, [3, 4]] = J_th[[3, 4]]
        J[:2, [3, 4]] = robot.computeJacobianCOM23(body_part = 'h')
        
        Fx_h = kv*(robot.slip_nominal_vel - dr[0]) + \
               -kp*(xx - r[0]) + -kd*(dxx - dr[0])
               
        Fy_h = kp*(h - r[1]) + kd*(dh - dr[1])
        F_com[:2, 0] = [Fx_h, Fy_h]

#        print 'h', h,  h - r[1], dh - dr[1]
        
    if 2 in p:
        indi, indf = 2, 4
        if 1 not in p: indi, indf = 0, 2
        tt = robot.tt_f
        tl = tt + robot.slip_st_dur
        xt = robot.xt_f[0]
        xl_expected = robot.foot_pose_f + robot.slip_st_xl

        r, dr = r_f, dr_f
        
        ### kind of constant height feedback
#        h, dh = computeYShift(t, tt, tl, robot.slip_yt, robot.slip_yl)

        ### exact slip feedback
        tau = (t - tt)/(tl - tt)
        s = (r[0] - xt)/(xl_expected - xt)
        # s = S[-1][1]
        
        if grf_flag_time:
            h, dh = robot.slip_fun_y_t(tau), robot.slip_fun_dy_t(tau)  
            xx, dxx = robot.slip_fun_x_t(tau) + xt, robot.slip_fun_dx_t(tau)
        else:
            h, dh = robot.slip_fun_y_s(s), robot.slip_fun_dy_s(s)  
            xx, dxx = robot.slip_fun_x_s(s) + xt, robot.slip_fun_dx_s(s)
        
        
        J[-1, [6, 7]] = J_th[[6, 7]]
        J[indi:indf, [6, 7]] = robot.computeJacobianCOM23(body_part = 'f')
        
        Fx_f = kv*(robot.slip_nominal_vel - dr[0]) +\
               -5.5*kp*(xx - r[0]) - 5.5*kd*(dxx - dr[0])
        
        Fy_f = -5.5*kp*(h - r[1]) - 5.5*kd*(dh - dr[1])
        F_com[indi:indf, 0] = [Fx_f, Fy_f]

#        print '>>>> f >>>>', h - r[1], dh - dr[1]


#    print F_com.flatten()
        
#        print '\n\n new: \n', -np.dot(J.T, F_com)[3:].flatten()
    
    u_feedback = np.dot(J.T, F_com)[3:].flatten()
    
    u_feedback[2] = ctrl_torso(robot, robot.slip_shift)
#    u_feedback[2] = 0
    
    # print u_feedback
    
    return u_feedback









    
#==============================================================================
# common function
#==============================================================================
def computeDesiredFootState(robot, body_part, x_now, dx_now):
#    t, tt, tl, ta, xt, xl, x_foot= param
    
    t = robot.t[-1][0] + robot.dt

    if body_part == 'slider': 
        tl = robot.tl_h
        xxi = robot.foot_pose_h - robot.xl_h[0]
    else: print("body part does not exist!!")

    if robot.getContactFeet() == []:
        tl = robot.tl_f

    tt = tl + robot.slip_sw_dur
    

    xxf = robot.slip_sw_xt 

    r, dr, ddr = getTraj(t, tl, tt, xxi, xxf)

    # rxs.append(r[0])
    # rys.append(r[1])
    # print('r is {}'.format(r))
    h, dh = computeYShift(t, tl, tt, robot.slip_yt, robot.slip_yl)
    # h,dh = (0,0)
    
    # print 'h, dh',  h,  dh
#    print 'r', r

        
    # siwng foot global position
    x_swing = r.flatten() 
    x_swing[0] += x_now[0]
    x_swing[1] += (x_now[1] - h)
    
#    if body_part == 'h' and t > ta and (tt - t) < 0.05: 
#        print 'hind foot desired traj adjusted !!!!'
#        x_swing[1] = 0

    # siwng foot global velocity
    dx_swing = dr.flatten() 
    dx_swing[0] += dx_now[0]
    dx_swing[1] += (dx_now[1] - dh)
    return x_swing, dx_swing
    
    
def computeYShift(t, ti, tf, hl, ht):
    tau = (t - ti)/(tf - ti)
    return hl*(1 - tau) + ht*tau, ht - hl


def ctrl_torso(robot, shift):
    
    r1, dr1 = robot.get_com(body_part = 'h', calc_velocity = True)
    r2, dr2 = robot.get_com(body_part = 'f', calc_velocity = True)
    
    d = r2 - r1
    ddot = dr2 - dr1
    
    d_norm = np.linalg.norm(d)
    dd_norm = np.dot(d, ddot)/d_norm

    tau = 0

    tau += 300*(robot.slip_shift - d_norm) + 300*(- dd_norm)
    
#    tau += 500*(np.abs(d[1])) + 50*(-np.abs(ddot[1]))
        
    return tau


def computeJacobianTh(robot):
    r_h, dr_h = robot.get_com(body_part='h', calc_velocity=True)
    r_f, dr_f = robot.get_com(body_part='f', calc_velocity=True)
    xh, yh = r_h[:2]
    dxh, dyh = dr_h[:2]
    xf, yf = r_f[:2]
    dxf, dyf = dr_f[:2]
    
    J_h = robot.computeJacobianCOM('h')
    J_f = robot.computeJacobianCOM('f')
    
    mup = (xf - xh)*(J_f[1, :] - J_h[1, :]) - (yf - yh)*(J_f[0, :] - J_h[0, :])
    mu = 1/((xf - xh)**2 + (yf - yh)**2)
    J = mu*mup
    return J
    

#==============================================================================
# Initial configuration (IK)
#==============================================================================
def computeInitialConfig(robot, des_config, nlayer):
    tt, tl, ta, x0_h, xt_h, xl_h, xa_h, x_foot_h, dx0_h,\
                x0_f, xt_f, xl_f, xa_f, x_foot_f, dx0_f = des_config
                
    x_swing_h, dx_swing_h = computeDesiredFootState(robot, 'h', x0_h, dx0_h)
    
    x_swing_f, dx_swing_f = computeDesiredFootState(robot, 'f', x0_f, dx0_f)
    


    des = np.array([x0_h, x_swing_h, x0_f, x_swing_f, \
                    dx0_h, dx_swing_h, dx0_f, dx_swing_f]).flatten()
    
    model = Centauro_RobotClass(t, q, qdot, p, u, dt, lua_file, param)
    
    if nlayer == 1:
        args = ([model, des, np, nlayer, 'first'])
        IC = np.concatenate((robot.q[0, :], robot.qdot[0, :]))
        res = root(CostIK, IC, args=args)
        out = res.x.flatten()
        
    elif nlayer == 2:
        args = ([model, des[:8], np, nlayer, 'first'])
        IC = robot.q[0, :]
        res1 = root(CostIK, IC, args=args)
        
#        print 'res1.fun', res1.fun
        
        model.q = res1.x.reshape(1, 8)
        args = ([model, des[8:], np, nlayer, 'second'])
        IC = robot.qdot[0, :]
        res2 = root(CostIK, IC, args=args)
        
#        print 'res2.fun', res2.fun
        
        out = np.concatenate((res1.x.flatten(), res2.x.flatten()))
        
#    print res.fun

    return out
    
def CostIK(x, args):

    model, des, np, nlayer, segment = args  
    
    #x0_h, x_swing_h, x0_f, x_swing_f, dx0_h, dx_swing_h, dx0_f, dx_swing_f
    
    if nlayer == 1:
        r1, r2, r3, r4, r5, r6, r7, r8, \
        r9, r10, r11, r12, r13, r14, r15, r16 = des 
    
        model.q = x[:8].reshape(1, 8)
        model.qdot = x[8:].reshape(1, 8)
        model.qdot[0, 0] = r9

    elif nlayer == 2:
        if segment == 'first':
            r1, r2, r3, r4, r5, r6, r7, r8, = des 
            model.q = x.reshape(1, 8)

        elif segment == 'second':
            r9, r10, r11, r12, r13, r14, r15, r16 = des 
            model.qdot = x.reshape(1, 8)
#            model.qdot[0, 0] = r9
#            model.qdot[0, 5] = 0
            
    xh, dxh = model.get_com(body_part = 'h', calc_velocity = True)
    xf, dxf = model.get_com(body_part = 'f', calc_velocity = True)
    
    point = np.array([model.body.l_end, 0., 0.])
    
    x_swing_h, dx_swing_h  = \
    model.CalcBodyToBase(model.body.id('b3h'), point, calc_velocity = True)
    
    x_swing_f, dx_swing_f = \
    model.CalcBodyToBase(model.body.id('b3f'), point, calc_velocity = True)
    
    
    if nlayer == 1:
        e = list()
        e.append((xh[0] - r1)**2)
        e.append((xh[1] - r2)**2)
        e.append((x_swing_h[0] - r3)**2)
        e.append((x_swing_h[1] - r4)**2)
        e.append((xf[0] - r5)**2)
        e.append((xf[1] - r6)**2)
        e.append((x_swing_f[0] - r7)**2)
        e.append((x_swing_f[1] - r8)**2)
        
        e.append((dxh[0] - r9)**2)
        e.append((dxh[1] - r10)**2)
        e.append((dx_swing_h[0] - r11)**2)
        e.append((dx_swing_h[1] - r12)**2)
        e.append((dxf[0] - r13)**2)
        e.append((dxf[1] - r14)**2)
        e.append((dx_swing_f[0] - r15)**2)
        e.append((dx_swing_f[1] - r16)**2)
        
    elif nlayer == 2 and segment == 'first':
        e = list()
        e.append((xh[0] - r1)**2)
        e.append((xh[1] - r2)**2)
        e.append((x_swing_h[0] - r3)**2)
        e.append((x_swing_h[1] - r4)**2)
        e.append((xf[0] - r5)**2)
        e.append((xf[1] - r6)**2)
        e.append((x_swing_f[0] - r7)**2)
        e.append((x_swing_f[1] - r8)**2)
        
    elif nlayer == 2 and segment == 'second':
        e = list()
        e.append((dxh[0] - r9)**2)
        e.append((dxh[1] - r10)**2)
        e.append((dx_swing_h[0] - r11)**2)
#        e.append((dx_swing_h[1] - r12)**2)
        e.append((model.qdot[0, 0] - r9)**2)
        e.append((dxf[0] - r13)**2)
        e.append((dxf[1] - r14)**2)
        e.append((dx_swing_f[0] - r15)**2)
#        e.append((dx_swing_f[1] - r16)**2)
        e.append((model.qdot[0, 5])**2)
        
    
    return np.array(e, dtype=float)

def testInitialConfig():
    tt = 0.1489017
    tl = 0.35038529
    ta = 0.49774645
    x0_h = np.array([0, .75])
    xt_h = np.array([0.44670511,  0.64124773])
    xl_h = np.array([1.0088164 ,  0.64094974])
    xa_h = np.array([1.45052468e+00,   7.47463342e-01])
    x_foot_h = 0.72742089
    
    shift = .9*(cr.param.l1h + cr.param.l1f)
    horiz_shift = np.array([shift, 0])
    x0_f, xt_f, xl_f, xa_f, x_foot_f = \
    x0_h + horiz_shift, xt_h + horiz_shift, xl_h + horiz_shift, \
    xa_h + horiz_shift, x_foot_h + horiz_shift[0]
    
    des_config = tt, tl, ta, x0_h, xt_h, xl_h, xa_h, x_foot_h, \
                             x0_f, xt_f, xl_f, xa_f, x_foot_f
    
#    print computeInitialConfig(0, des_config)
    return computeInitialConfig(cr, des_config)   
    






#
#
#p = np.linspace(0, 1, 100)   
#for i in p:
#    plt.figure('grfx')
#    plt.plot(i, cr.slip_fun_grf_x(i), '+')
#    plt.plot(i, crt.slip_fun_grf_x(i), '*')
#    
#    plt.figure('grfy')
#    plt.plot(i, cr.slip_fun_grf_y(i), '+')
#    plt.plot(i, crt.slip_fun_grf_y(i), '*')
#    
#    plt.figure('x')
#    plt.plot(i, cr.slip_fun_x(i), '+')
#    plt.plot(i, crt.slip_fun_x(i), '*')
#    
#    plt.figure('dx')
#    plt.plot(i, cr.slip_fun_dx(i), '+')
#    plt.plot(i, crt.slip_fun_dx(i), '*')
#    
#    plt.figure('y')
#    plt.plot(i, cr.slip_fun_y(i), '+')
#    plt.plot(i, crt.slip_fun_y(i), '*')
#    
#    plt.figure('dy')
#    plt.plot(i, cr.slip_fun_dy(i), '+')
#    plt.plot(i, crt.slip_fun_dy(i), '*')
#
#
#
#

def check_derivations(Q_des,QD_des,QDD_des):
    dt = 0.002
    n = Q_des.shape[1]
    qdot = []
    qddot = []
    for i,q in enumerate(Q_des):
        try:
            qdot.append((Q_des[i+1] - Q_des[i])/dt)
            qddot.append((QD_des[i+1] - QD_des[i])/dt)
        except:
            pass
    
    
    QD_des = np.array(QD_des)
    qdot = np.array(qdot)
    QDD_des = np.array(QDD_des)
    qddot = np.array(qddot)
    plt.figure()
    for i in range(1,n):
        plt.subplot(4,2,i)
        plt.plot(np.linspace(0,1,qddot.shape[0]),qddot[:,i-1],label='dd_computed')    
        plt.plot(np.linspace(0,1,QDD_des.shape[0]),QDD_des[:,i-1],label='dd_desired')
    
    plt.legend()
    
    plt.figure()
    
    for i in range(1,n):
        plt.subplot(4,2,i)
        plt.plot(np.linspace(0,1,qdot.shape[0]),qdot[:,i-1],label='d_computed')    
        plt.plot(np.linspace(0,1,QD_des.shape[0]),QD_des[:,i-1],label='d_desired')

    
    plt.legend()









