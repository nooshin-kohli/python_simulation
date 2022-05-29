# -*- coding: utf-8 -*-
"""

@author: Nooshin Kohli
"""

import numpy as np
from numpy.linalg import inv, pinv
from scipy.linalg import sqrtm
#import rbdl
#import os

#from ImportModel import BodyClass, JointClass
#import matplotlib.pyplot as plt
#from scipy.optimize import root, approx_fprime, minimize, fminbound
#import scipy.integrate as integrate
#import time
#import subprocess

#fprint = True
#flag_fifo = True

class leg_controlclass(object):
    def __init__(self, robot):
        """
        This is Centauro Controllers class
        """        
        self.name = 'ROBOT'
        
        self.robot = robot  
        
        self.m = self.robot.qdim
        
        self.n = len(self.robot.getContactFeet())*2
        
        self.Sc = np.hstack((np.eye(self.n), np.zeros((self.n, self.m - self.n))))
        self.Su = np.hstack((np.zeros((self.m - self.n, self.n)), np.eye(self.m - self.n)))
        
            

    def QRDecomposition(self):
        
#        JT = self.robot.Jc.T
        JT = self.robot.Jc_from_cpoints(\
        self.robot.model, self.robot.q[-1, :], self.robot.body,\
        self.robot.getContactFeet()).T
        
        m, n = JT.shape
        
        if m == 0 or n == 0:
            raise TypeError('Try to calculate QR decomposition, while there is no contact!')
        
        from numpy.linalg import qr
        
        self.qr_Q, qr_R = qr(JT,mode='complete')
        
        self.qr_R = qr_R[:n, :]
        
        return None
        
    def CalcPqr(self): 
        
        self.QRDecomposition()
        
        return np.dot(self.Su, self.qr_Q.T)
    
        
    def InverseDynamics(self, qddot_des, P, W = None, tau_0 = None):
        S = self.robot.S
        M = self.robot.CalcM(self.robot.model, self.robot.q[-1, :])
        h = self.robot.Calch(self.robot.model, self.robot.q[-1, :], \
        self.robot.qdot[-1, :])
        
        
#        if P is None: P = np.eye(M.shape[0])
        if W is None: W = np.eye(S.shape[0])
        if tau_0 is None: tau_0 = np.zeros(S.shape[0])
        
        invw = inv(W)
        Mqh = np.dot(M, qddot_des).reshape(8,1) + h.reshape(8,1)
            
        if self.n >= 3:        
            aux1 = np.dot(invw, np.dot(S, P.T))
            aux21 = np.dot(P, np.dot(S.T, invw))
            aux22 = np.dot(S, P.T)
            aux2 = np.dot(aux21, aux22)
            winv = np.dot(aux1, pinv(aux2))
        else:
            w_m_s = np.linalg.matrix_power(sqrtm(W), -1)
            aux = pinv(np.dot(P, np.dot(S.T, w_m_s)))
            winv = np.dot(w_m_s, aux)
            
        aux3 = np.dot(np.eye(S.shape[0]) - np.dot(winv, np.dot(P, S.T)), invw)
        return np.dot(winv, np.dot(P, Mqh)).flatten() + np.dot(aux3, tau_0)
            
             
        
    def InvDyn_qr(self, q_des, qdot_des,  qddot_des, W = None):
        
        n = len(self.robot.getContactFeet())*3
        if self.n != n:
            self.n = n
            self.Sc = np.hstack((np.eye(self.n), np.zeros((self.n, self.m - self.n))))
            self.Su = np.hstack((np.zeros((self.m - self.n, self.n)), \
            np.eye(self.m - self.n)))

        
        kp = 4.
        kd = kp/10
        
#        print kd
        
        p_qr = self.CalcPqr()
        
#        print p_qr.shape
        
        invdyn = self.InverseDynamics(qddot_des, p_qr, W)
        
        q_des = np.array(q_des)
        qdot_des = np.array(qdot_des)
#        print('q_des is' , q_des.shape,'qdot_des',qdot_des.shape,'tes',self.robot.q_des)
        out1 = np.dot(self.robot.S.T, invdyn).reshape(8,1) 
        out2 = kp * (q_des.flatten() - self.robot.q[-1, :]).reshape(8,1) 
        out3 = kd * (qdot_des.flatten() - self.robot.qdot[-1, :]).reshape(8,1)
        out = out1 + out2 + out3;
        
#        print('out shape is',np.dot(self.robot.S,out).shape)
        return np.dot(self.robot.S,out)
        
        
        
    def Opr_Space(self, J, x_des, xdot_des, xddot_des, x, xdot, \
    tau_0 = None, tau_C = None):
        """
        implements Mistry/Righetti method.
        It does not seem efficient and elegant as it requires multiple evaluations
        of inverse of Intertia matrix
        TODO:
        1. Calculate derivatives of Jacobians: Jcdot, and Jdot
        """
        
        Jc = self.robot.Jc_from_cpoints(\
        self.robot.model, self.robot.q[-1, :], self.robot.body,\
        self.robot.getContactFeet())
        
        M = self.robot.CalcM(self.robot.model, self.robot.q[-1, :])
        h = self.robot.Calch(self.robot.model, self.robot.q[-1, :], \
        self.robot.qdot[-1, :])
        
        pinv_Jc = pinv(Jc)
        I = np.eye(self.robot.qdim)
        P = I - np.dot(pinv_Jc, Jc)  
        Mc = np.dot(P, M) + I - P
        inv_Mc = inv(Mc)
        C = - np.dot(pinv_Jc, Jcdot)
        Ac = inv(np.dot(J, np.dot(inv_Mc, np.dot(P, J.T))))
        aux_j = np.dto(J, np.dot(inv_Mc, P))
        J_T_hash = np.dot(Ac, aux_j)
        N = I - np.dot(J.T, J_T_hash)
        
        kp = 4.
        kd = kp/5
        xddot = xddot_des + kd*(xdot_des - xdot) + kp*(x_des - x)
        
        aux1 = np.dot(J, np.dot(inv_Mc, np.dot(P, h)))
        aux2 = np.dot(Jdot, self.robot.qdot[-1, :]) + \
        np.dot(J, np.dot(inv_Mc, C))
        aux = aux1 - aux2
        F = np.dot(Ac, xddot) + np.dot(Ac, aux)
        
        if np.isnan(tau_0).all(): tau_0 = np.zeros(M.shape[0] - 6)
        if np.isnan(tau_C).all(): tau_C = np.zeros(M.shape[0] - 6)
        tau = np.dot(P, np.dot(J.T, F)) + np.dot(P, np.dot(N, tau_0)) + \
        np.dot(I - P, tau_C)
        
        return tau
        
        
        
        
        
        
        
        
        
class Control(object):
    def __init__(self,robot):
        self.name = 'leg_Control'
        self.robot = robot  
        self.cp = robot.getContactFeet()
        self.k = len(self.cp) *3
#        self.Q , self.R = self.QRDecomposition(Jc)
    
    def QRDecomposition(self,matrix):
        matrix = np.array(matrix)
        q,r = np.linalg.qr(matrix,mode='complete')
        return q,r
    
    
    def generate_weight(self,size):
        b = [0]
        while any(i<0.1 for i in b):
            A = np.random.rand(size,size)
            B = np.dot(A,A.transpose())
            b = np.linalg.eig(B)[0]
#            print "again because b" , b
        return B
    
    def computeQuQc(self,Q):
        k = self.k
        Qc = Q[:,:k]
        n = Q.shape[0]
        Qu = Q[:,n-k:];
        return Qc,Qu
    
    def compute_QS(self,Qc,Qu):
        k = self.k
        w = self.generate_weight(self.robot.qdim-3)
        s = self.robot.S
        
        
        
        if k >= 3:
            SQu = np.matmul(s,Qu)
#            print('these shapes are (SQu,w)',SQu.shape,w.shape)
            lhs = np.matmul(np.linalg.inv(w), SQu)
            
            QuS = np.matmul(Qu.T,s.T)
            WS = np.matmul(np.linalg.inv(w),s)
            rlhs = np.matmul(QuS,WS)
            rhs = np.matmul(rlhs,Qu)
            rhs = np.linalg.inv(rhs)
            
            QS = np.matmul(lhs,rhs)
        else:
            QuS = np.matmul(Qu.T,s.T)
            W_1_2 = np.linalg.matrix_power(w,-1/2)
            pran = np.matmul(QuS,W_1_2)
            
            QS = np.matmul(W_1_2,np.linalg.pinv(pran))
#            raise Exception('k is smaller than 3');
            
            
        return QS
            
    
    def torque(self,qdd_des):
        
        jC = self.robot.Jc_from_cpoints(self.robot.model,self.robot.q,self.robot.body,self.robot.getContactFeet())
        
#        print('jc shape is ', jC.shape)
        
        Q,R = self.QRDecomposition(jC.T)
#        print('Q is ' , Q , 'shape is' , Q.shape)
        
        M = self.robot.CalcM(self.robot.model, self.robot.q)
        h = self.robot.Calch(self.robot.model, self.robot.q, self.robot.qdot)
        
        
#        print('qdd_des shape is ',qdd_des)
        Mqh = np.dot(M, qdd_des).reshape(8,1) + h.reshape(8,1)

        
        Qc,Qu = self.computeQuQc(Q)
        QS = self.compute_QS(Qc,Qu)
        QSQ = np.matmul(QS,Qu.T)
        torque = np.matmul(QSQ,Mqh)
        
        return torque
        
    
    
    
    
#class Control2010:
#    def __init__(self,robot):
#        self.robot = robot  
#        self.cp = robot.getContactFeet()
#        self.k = len(self.cp) *2
#        
#    
#    def QRDecomposition(self,matrix):
#        matrix = np.array(matrix)
#        q,r = np.linalg.qr(matrix,mode='complete')
#        return q,r
#    
#    
#    
#    def compute_scSu(k,n=5,fb=3):
#        Sc = 
#        
#    
#    def compute_SQS():
        

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        