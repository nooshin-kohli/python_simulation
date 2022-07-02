# -*- coding: utf-8 -*-
"""
Created on Wed Nov 12 09:49:44 2014

@author: mshahbazi
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import brute
import scipy.integrate as integrate
#import mpfuncs as mp
fprint = True

        
class hopper1D(object):
    def __init__(self, t, x, q, xdes, dt):
        """
        This is a 1D-hopper class
        """        
        self.name = '1D-hopper'
        
        self.t = np.array([t])
        self.x = np.array(x)
        self.q = np.array(q)
        
        self.xdes = np.array(xdes)
        self.e = np.array([])
        
        self.xa = self.x[-1, :]
        self.ta = self.t[-1]
        self.event = 'a'
        
        self.dt = dt
        
        self.t0 = self.t[-1]
        self.x0 = self.x[-1,:]
        self.q0 = self.q[-1, :]
        
        self.changeKinDcom = False
        self.Delta = 0
        self.cmode = 0
        
        self.kResolution = .1e3
        

    def __call__(self):
        """
        executes hybrid system
        """
        self.t0 = self.t[-1]
        self.x0 = self.x[-1,:]
        self.q0 = self.q[-1, :]
        
        self.xx = integrate.odeint(self.dyn, self.x0, np.array([0,self.dt]))                                   
        
        self.e = -5
        self.ev = list(self.evts())      
        self.ev = np.array([self.ev[i](self.t0 + self.dt, \
        self.xx[-1,:]) for i in range(len(self.ev))])
        

        if (self.ev>0).any():
            self.e = list(self.ev>0).index(True)
#            if self.e==1 and self.q[-1, 0]==2: self.event = 'k in dcom';self.Delta = 0
            self.trefined , self.xxrefined = self.refine(self.evts()[self.e], do_refine=True)
#            print '\n trefined', self.trefined
            self.xx = np.array([self.xxrefined])
            self.xx, self.q0 = self.trans()
                       
        if self.e == 0:            
            if self.q0[0]==0:
                self.xa = self.xx[-1,:]
                self.ta = self.trefined
                self.event = 'a'
            elif self.q0[0]==1:
                self.xt = self.xx[-1,:]
                self.tt = self.trefined
                self.event = 't'
            elif self.q0[0]==2:
                self.xb = self.xx[-1,:]
                self.tb = self.trefined
                self.event = 'b'
            elif self.q0[0]==3:
                self.xl = self.xx[-1,:]
                self.tl = self.trefined
                self.event = 'l'
                
        if self.e!=0: self.event = ''

        self.x = np.append(self.x, [self.xx[-1,:]], axis=0)
        self.q = np.append(self.q, [self.q0], axis=0)
        if self.e >= 0:
            self.t = np.append(self.t, [self.trefined], axis=0)
        else:
            self.t = np.append(self.t, [self.t0 + self.dt], axis=0)

        return None    
    
    def refine( self, evtFun, tol=1e-12, do_refine=False, *args ):
        """Find the time at which the trajectory is a root of the
           function evtFun.
           
           The original code is part of the package 'integro', which is adopted
           here for notation consistency.
        
           evtFun is a function taking (t,y,*args) and returning a real number.
           refine() uses trajectory patch to find the "time"
           at which evtFun(t,y(t),*args)==0.
           
           This code requires that the signs of evtFun in the beginning and
           end of the patch are opposite. It finds the root by bisection 
           -- a binary search.
           
           returns t,y --the end time and state; t is correct to within +/- tol
        """
        if not do_refine:
            return self.t0, self.x0
        else:
            t0,t1 = self.t0, self.t0 + self.dt
    #        for k in xrange(4):
    #            y = self.x0
    #            f0 = evtFun(t0,y, *args)
    #            y = integrate.odeint(self.dyn, self.x0, \
    #            np.array([0, np.abs(t1 - t0)]))[-1, :]
    #            f1 = evtFun(t1,y, *args)
    #            if f0*f1 <= 0:
    #                break
    #            t0,t1 = t0-(t1-t0)/2,t1+(t1-t0)/2
    #            print "WARNING: function did not change sign -- extrapolating order ",k
    ##        print 't0,f0,t1,f1', t0,f0,t1,f1
            y = self.x0
            f0 = evtFun(t0,y, *args)
            y = integrate.odeint(self.dyn, y, np.array([0, t1 - t0], dtype=float))[-1, :]
            f1 = evtFun(t1,y, *args)
            if f0*f1 <= 0:
                if f1<0:
                    t0,f0,t1,f1 = t1,f1,t0,f0
                ypre = self.x0
                while abs(t1-t0)>tol:
                    t = (t1+t0)/2.
                    y = integrate.odeint(self.dyn, ypre, \
                    np.array([0, np.abs(t1 - t0)/2], dtype=float))[-1, :]
                    f = evtFun(t,y, *args )
                    if f==0:
                        break
                    if f>0:
                        t1,f1 = t,f
                    else:
                        t0,f0 = t,f
                        ypre = y
                return (t1+t0)/2, y 
            else:
                print("WARNING: function did not change sign -- \
                It can be due to early changes in the system mode (no problem!)")
                return self.t0, self.x0

        
    def dyn(self, x, t):
        """
        .dyn  evaluates system dynamics
        """
        q,m,eta0,k,g,beta,foot = self.q0
       
        # stance
        if q == 1 or q == 2:        
            q1, q2, dq1, dq2 = x
            c1x = foot
            c1y = 0
            sqrL1 = (-c1x + q1)**2 + (-c1y + q2)**2
            L1 = (sqrL1)**0.5
            dx = np.array([dq1, dq2, 1/m*k*(eta0 - L1)*(q1 - c1x)/L1,
            -g + 1/m*k*(eta0 - L1)*(q2 - c1y)/L1])          

        # flight
        elif q == 0 or q == 3:
            x,y,dx,dy = x
            dx = np.array([dx, dy, 0, -g])

        return dx

    

    def trans(self):
        """
        .trans  transition between discrete modes
        """
        q,m,eta0,k,g,beta,foot = self.q0.copy()
        x = self.xx.copy()
        e = self.e
        
        # compression
        if q == 1:
            if e == 1 or e == 2 or e == 3:q = -1
            elif e == 0:
                k = 2*m/(eta0 - x[-1, 1])**2*(9.81*(self.xdes[1] - x[-1, 1]))
                # if self.t[-1]>1.2: print('x[-1, 1], self.xdes[1], k', \
                # x[-1, 1], self.xdes[1], k)
                if k <self.kResolution: k = self.kResolution
#                k += 2*m*g*(sself.xdes[1] - self.xa[1])/(eta0 - x[-1, 1])**2
                q = 2
                
        # decompression
        elif q == 2:
            if e == 2 or e == 3:q = -1
            elif e == 0:q = 3; self.cmode = 0
            elif e == 1 and self.changeKinDcom:
                self.changeKinDcom = False
                self.event = ''
                self.Delta = 0
                k = m/(eta0 - x[-1, 1])**2*(2*g*(self.xdes[1] - x[-1, 1]) - \
                x[-1, 3]**2)
                
#                print 'y at q=3', x[-1, 1]

        # descending
        elif q == 0:
            if e == 1:q = -1
            elif e == 0:q = 1; foot = x[-1, 0] + eta0*np.cos(beta)
#                print 'foot, x[0]', foot, x[0]
                     
           
        # ascending
        elif q == 3:
            if e == 1:q = -1
            elif e == 0:q = 0
        
        x = np.array(x)
        qq = np.array([q,m,eta0,k,g,beta,foot])
        return x, qq


    def evts(self):
        """
        .evts  returns event functions for given hybrid domain
        """
        m,eta0,k,g,beta,foot = self.q0[1:]
        Delta = self.Delta
#        x = self.xx.copy()
#        t = self.t0 + self.dt
        
        # compression
        if self.q0[0] == 1:

            return [lambda t,x : (x[2]*(x[0]-foot) + x[3]*x[1]) / 
            np.sqrt((x[0]-foot)**2 + x[1]**2),
                    lambda t,x : -x[1],
                    lambda t,x : -x[1],
                    lambda t,x : -x[1]]
                    
        # decompression
        elif self.q0[0] == 2:

            return [lambda t,x : -(eta0 - (np.sqrt((x[0] - 
            foot)**2 + x[1]**2))),
                    #lambda t,x : x[1] - (eta0 - Delta), # for changing k
                    lambda t,x : - x[3],
                    lambda t,x : - x[1]]

        # descending
        elif self.q0[0] == 0:
            # hip height
            h = eta0 * np.sin(beta)

            return [lambda t,x : -(x[1] - h),
                    lambda t,x : -x[1]]
        
        # ascending            
        elif self.q0[0] == 3:

            return [lambda t,x : -x[3],
                    lambda t,x : -x[1]]


    def get_phase(self):
        """
        Parameter
        ---------
        not needed.
        
        Return
        ------
        phase : string
                subphase ('dsce.', 'comp.', 'dcomp.', 'asce.')
        """
        if self.q0[0]==0:
            ph = 'dsce.'
        elif self.q0[0]==1 :
            ph = 'comp.'
        elif self.q0[0]==2:
            ph = 'dcomp.'
        elif self.q0[0]==3:
            ph = 'asce.'
        return ph
        

    
    
    
    
    
    
        
        
    def predict_flight_t(self, y, dy):
        g, eta0 = self.q[-1, 4]*0 + 9.81, self.q[-1, 2]
#        y, dy = self.x[-1, 1], self.x[-1, 3]
        v = -np.sqrt(2*g*(y - eta0) + dy**2)
        return np.asscalar((dy - v)/g + self.t[-1])
        
    def predict_comp_l(self, k, y0, dy0):
#        print 'yes'
        m,eta0,g = self.q[-1, [1, 2, 4]]
        g = 9.81
#        y0, dy0 = self.x[-1, 1], self.x[-1, 3]
        ydes = self.xdes[1]
        ya = self.xa[1]
               
        w = np.sqrt(k/m)
        F = (eta0 - g/w**2)
        A = y0 - F
        B = dy0/w
        tx_b = 1/w*np.arctan(B/A)
        if tx_b<0:
            tx_b += np.pi/w
#            print 'this'
        yb = A*np.cos(w*tx_b) + B*np.sin(w*tx_b) + F
    #    print ta_t + tt_b
    #    dyb = 0
        k = 2*m/(eta0 - yb)**2*(g*(ydes - yb))
#        k += 2*m*g*(ydes - ya)/(eta0 - yb)**2
#        print 'k', k
        w = np.sqrt(k/m)
        F = (eta0 - g/w**2)
        A = yb - F
        B = 0
        tb_l = 1/w*np.arccos(g/w**2/(yb - F))
        if tb_l<=0:
            tb_l += np.pi*2/w
#            print 'that'
            
#        dyl = -A*w*np.sin(w*tb_l)
        return np.asscalar(tx_b + tb_l + self.t[-1])
        
    def predict_dcomp_l(self, k, y0, dy0):
        
        q,m,eta0,kk,g,beta,foot = self.q[-1, :]
        g = 9.81
        
        w = np.sqrt(k/m)
        self.w = w
        F = (eta0 - g/w**2)
        A = y0 - F
        B = dy0/w
        
        

        if not(self.changeKinDcom):
            argsqrt = -(F - eta0)**2 + A**2 + B**2
            if argsqrt<1e-10 and argsqrt>-1e-10:
                argsqrt = 0          
            if argsqrt<0:
                print('warning! the sqrt argument is negative so we substitute\
                the liftoff time as equal to 2s (fake)')
                print('w, F, A, B, argsqrt', w, F, A, B, argsqrt)
                tx_l = 2
            else:
                tx_l1 = -2*np.arctan((B - np.sqrt(argsqrt))/(F - eta0 - A))/w
                tx_l2 = -2*np.arctan((B + np.sqrt(argsqrt))/(F - eta0 - A))/w
                condidate = [x for x in [tx_l1, tx_l2] if x>=0]
                self.condidate = condidate
                if condidate==[]:
                    print('WARNING!-- no solution for the equations in arctan!')
                else:
                    tx_l = min(condidate)                
        else:
            TobeSureThatThisWontOccur 
            argsqrt = -(F -(eta0 - self.Delta))**2+ A**2 + B**2
            if argsqrt<1e-10 and argsqrt>-1e-10:
                argsqrt = 0
            if argsqrt<0:
                print('warning! the sqrt argument is negative so we substitute\
                the liftoff time as equal to 2s (fake)')
                tx_d = 2
            else:
                tx_d1 = -2*np.arctan((B - np.sqrt(argsqrt))/((F - \
                (eta0 - self.Delta)) - A))/w
                tx_d2 = -2*np.arctan((B + np.sqrt(argsqrt))/((F - \
                (eta0 - self.Delta)) - A))/w
                condidate = [x for x in [tx_d1, tx_d2] if x>=0]
                if condidate == []:
                    print('WARNING!-- no solution for the equations in arctan!')
                else:
                    tx_d = min(condidate)
                
            dydelta = -A*w*np.sin(w*tx_d) + B*w*np.cos(w*tx_d)
            y0, dy0 = eta0 - self.Delta, dydelta
            k = m/(eta0 - y0)**2*(2*g*(self.xdes[1] - y0) - dy0**2)
            if k<0:k = .0001
            w = np.sqrt(k/m)
            F = (eta0 - g/w**2)
            A = y0 - F
            B = dy0/w
            argsqrt = -(F - eta0)**2 + A**2 + B**2
            if argsqrt<1e-10 and argsqrt>-1e-10:
                argsqrt = 0
            if argsqrt<0:
                print('warning! the sqrt argument is negative so we substitute\
                the liftoff time as equal to 2s (fake)')
                td_l = 2
            else:
                td_l1 = -2*np.arctan((B - np.sqrt(argsqrt))/((F - eta0) - A))/w
                td_l2 = -2*np.arctan((B + np.sqrt(argsqrt))/((F - eta0) - A))/w
                condidate = [x for x in [td_l1, td_l2] if x>=0]
                if condidate == []:
                    print('WARNING!-- no solution for the equations in arctan!')
                else:
                    td_l = min(condidate)                   
            tx_l = tx_d + td_l
        return np.asscalar(np.array([tx_l]) + self.t[-1])

    def SolveforK(self, k):
        return (self.tdes - self.get_tl_forK(k))**2
        
    def get_tl_forK(self, k):
        if self.get_phase()=='comp.':
            tliftoff = self.predict_comp_l(k, self.x[-1, 1].copy(), \
            self.x[-1, 3].copy())
            
        elif self.get_phase()=='dsce.':
            v = -np.sqrt(2*self.q[-1, 4]*(self.x[-1, 1] - self.q[-1, 2]) +\
            self.x[-1, 3]**2)
            tliftoff = self.predict_flight_t(self.x[-1, 1].copy(),\
            self.x[-1, 3].copy()) + \
            self.predict_comp_l(k, self.q[-1, 2], v) - self.t[-1]
        elif self.get_phase()=='dcomp.':
            tliftoff = self.predict_dcomp_l(k, self.x[-1, 1].copy(),\
            self.x[-1, 3].copy())        
        return tliftoff
            
        
    def set_control(self, tdes):
        m, L0, g = self.q[-1, [1, 2, 4]].copy()
        g = 9.81
        self.tdes = tdes
        if self.get_phase()=='comp.':
            if fprint: print('"comp." controller is invoked')            
            self.q[-1, 3] = brute(self.SolveforK, \
            np.array([slice(1.05*2*m*g*self.xa[1]/L0**2, 200e3,\
            self.kResolution)]), finish=None)
            if self.q[-1, 3]<self.kResolution: self.q[-1, 3] = self.kResolution

        elif self.get_phase()=='dcomp.' and self.cmode!=1:
            self.changeKinDcom = False
            if fprint: print('"dcomp." controller is invoked')
            y, dy = self.x[-1, [1, 3]].copy()
            ydes = self.xdes[1].copy()
#            cmode = self.get_control_mode(y, dy, ydes, m, L0, g)
#            self.set_control_dcomp(cmode, y, dy, ydes, m, L0, g)
            self.set_control_dcomp2(y, dy, ydes, m, L0, g) 
            
#            print 'y, dy, ydes', y, dy, ydes
#            print 'before', self.q[-1, 3]
#            self.q[-1, 3] = 2*m/(L0 - y)**2*(g*(ydes - y) - 1/2.*dy**2)
#            print 'after', self.q[-1, 3]
        return None
        
    def set_control_dcomp(self, cmode, y, dy, ydes, m, L0, g):
        if cmode == 1:
            self.q[-1, 3] = self.kResolution
            if self.t[-1][0]: plt.plot(self.t[-1], self.x[-1, 1], 'or')
        elif cmode == 2:
            self.q[-1, 3] = m/(L0 - y)**2*(2*g*(self.ybar - y) - dy**2)
            if self.q[-1, 3]<self.kResolution: self.q[-1, 3] = self.kResolution
        elif cmode == 3:
            self.Delta = (L0 - y)*.1*0; self.changeKinDcom = False
            klow = m/(L0 - y)**2*(2*g*(L0 - y) - dy**2)
            if klow<=0: klow = 1e-6
            kup = m/(L0 - y)**2*(2*g*(ydes - y) - dy**2)
#            print 'klow, kup', klow, kup
            self.q[-1, 3] = brute(self.SolveforK, \
            np.array([slice(klow, kup, self.kResolution)]),finish=None)
            if self.q[-1, 3]<self.kResolution: self.q[-1, 3] = self.kResolution            
        return None
        
    def get_control_mode(self, y, dy, ydes, m, L0, g):
        ya = dy**2/2./g + y
        if ya>L0:
            tawL = (dy - np.sqrt(dy**2 - 2*g*(L0 - y)))/g + self.t[-1]
            if tawL>self.tdes: 
                if np.abs(tawL - self.tdes)>2*self.dt: cmode = 3
                else: cmode = -1
#                cmode = 3; print 'tawL, tdes', tawL, self.tdes
            else:
                self.ybar = L0 + (ydes - L0)*.1
                if ya>self.ybar: cmode = 1
                else: cmode = 2
        else:
            k = m/(L0 - y)**2*(2*g*(L0 - y) - dy**2)
            if k<=0: k = 1e-6
            tawL = self.predict_dcomp_l(k, y, dy)
            if tawL>self.tdes: 
                if np.abs(tawL - self.tdes)>2*self.dt: cmode = 3
                else: cmode = -1
            else: cmode = 2; self.ybar = L0 + (ydes - L0)*.1
        self.cmode = cmode   
        return cmode
        
    def set_control_dcomp2(self, y, dy, ydes, m, L0, g):
         self.ybar = L0 + (ydes - L0)*.05
         kdmin = m/(L0 - y)**2*(2*g*(self.ybar - y) - dy**2)
         if kdmin<0: kdmin = self.kResolution
         kdmax = m/(L0 - y)**2*(2*g*(ydes - y) - dy**2)
         if kdmax<0: kdmax = self.kResolution
         if kdmin == self.kResolution and kdmax == self.kResolution:
             self.q[-1, 3] = self.kResolution
         else:
             self.q[-1, 3] = brute(self.SolveforK, \
             np.array([slice(kdmin, kdmax, self.kResolution)]),finish=None)
         if self.q[-1, 3]<self.kResolution: self.q[-1, 3] = self.kResolution            
         return None
    
 









       

        
        
    def get_evtime(self):
        if self.get_phase()=='dsce.' or self.get_phase()=='asce.':
            tevt = {'l': np.asscalar(self.tl), 't': self.predict_flight_t()}
        elif self.get_phase()=='comp.':
            tevt = {'t': np.asscalar(self.tt), 'l': self.predict_comp_l(\
            self.q[-1, 3].copy(), self.x[-1, 1].copy(), \
            self.x[-1, 3].copy())}           
        elif self.get_phase()=='dcomp.':
            tevt = {'t': np.asscalar(self.tt), 'l': self.predict_dcomp_l()}
#        elif self.get_phase()=='asce.':
#            tevt = {'l': self.tl, 't': self.predict_flight_t()}            
        return tevt
        
    def set_schedule(self, gait, tf, td, x0_mp):
        
        
#        print 'set_schedule'
#        x0 = np.array([[tevt[i]['t'] for i in range(len(tevt))] + \
#        [tevt[i]['l'] for i in range(len(tevt))]])    
        tg = (len(gait) - 1)*tf + len(gait)*td
    #    tg = .2
    #    print 'gait, tf, tg, td', gait, tf, tg, td
        A = mp.Alinsys(gait, tf, tg, td)
        x = x0_mp.copy()
#        for i in range(3):
#            x = np.append(x, mp.mul(A, np.array([x[:, -1]]).T), axis=1)
#        plt.figure()
#        plt.title(repr(self.event)+' ,x0 = '+repr(x0_mp.T))
#        mp.visualize(x, tf, tg, td)
#        plt.savefig('mp'+repr(self.event)+repr(self.t[-1])+'.pdf', format='pdf')
    #    print 'A, x0', A, x0, np.shape(A)
    #    print 'x+', mp.mul(A, x0.T)
    #    np.array([tdes[-1, :]]).T
#        print mp.mul(A, x0_mp)
        return mp.mul(A, x0_mp)
        
    def get_touchdown(self):
        if self.get_phase()=='dsce.' or self.get_phase()=='asce.':
            g, eta0 = self.q[-1, 4]*0 + 9.81, self.q[-1, 2]
            y, dy = self.x[-1, 1], self.x[-1, 3]
            v = - np.sqrt(2*g*(y - eta0) + dy**2)
            tt = np.asscalar((dy - v)/g + self.t[-1])
        elif self.get_phase()=='comp.' or self.get_phase()=='dcomp.':
            tt = self.tt
        return tt
    
    
    
    def set_liftoff(self, gait, tf, td, nsys):
        rlS = list(range(nsys))
        scheduls = np.empty_like(1, nsys)
        for i in rlS:
            if self.event!='':
                self.touch = self.get_touchdown()
                x0_mp = np.zeros(1, nsys*2); x0_mp[0, i] = self.touch
                self.schedul = self.set_schedule(list(gait), tf, td, x0_mp)
                scheduls = np.hstack((scheduls, self.schedul[-1, nsys:]))
            else:
                self.touch = None
                self.schedul = np.empty_like(1, nsys*2)
                
        for i in range(len(np.size(scheduls, 0)-1)):
            cond = scheduls[i+1, :] - scheduls[i, :]
            if np.abs(np.max(cond))>=np.abs(np.min(cond)):
                tdes = scheduls[i+1, :].T
            else:
                tdes = scheduls[i, :].T
        return tdes
