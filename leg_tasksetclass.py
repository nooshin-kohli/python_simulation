# -*- coding: utf-8 -*-
"""
@author: Nooshin Kohli
"""

from numpy.linalg import inv, pinv
import numpy as np

class TaskSet(object):
    
    def __init__(self, robot):
        
        self.bodies = []
        self.normals = []
        self.points = []
        self.priorities = []
        self.names = []
        self.constraint = []
        self.p1 = []
        self.p2 = []
        self.p3 = []
        self.pc = []
        self.robot = robot
        self.__xddot_des = {}
        
        
        self.feet_body_id_list =  [self.robot.body.id('hip'), 
                                           self.robot.body.id('thigh'),
                                           self.robot.body.id('calf')]
                                           
        self.world_normal_list = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
                                      
        self.contact_feet_name_list = ['foot']
                                       
        self.swing_feet_name_list = [['foot_swing_x', 'foot_swing_y', 'foot_swing_z']]
                                       
        self.base_rotation_name_list = ['base_rotation_x', 'base_rotation_y', 
                                        'base_rotation_z']
        self.CheckContactFeet()

        
        return None
        
    def AddTask(self, body_id, body_point, world_normal, task_priority, \
    task_name, IsConstraint = False):
        """
        task_name should be unique
        task_priority in [1, 2, 3]
        """
        
        if task_name in self.names:
            raise ValueError('task_name should be unique!\n maybe you are adding feet contacts manually!')
            
            
#        if body_id in self.feet_body_id_list and np.allclose(body_point, \
#        np.array([0., 0., -self.robot.body.l_end])) and IsConstraint and \
#        task_name != self.contact_feet_name_list[\
#        self.feet_body_id_list.index(body_id)][world_normal.index(True)]:
#            raise ValueError('you try to add feet contacts manually which is not allowed!')
            
            
        self.bodies.append(body_id)
        self.points.append(body_point)
        self.normals.append(list(world_normal))
        self.priorities.append(task_priority)
        self.names.append(task_name)
        self.constraint.append(IsConstraint)
        
        self.Categorize()
        return None
        
    def RemoveTask(self, name):
        i = self.names.index(name)
        self.bodies.pop(i)
        self.points.pop(i)
        self.normals.pop(i)
        self.priorities.pop(i)
        self.names.pop(i)['foot_swing_x', 'foot_swing_y', 'foot_swing_z']
        self.constraint.pop(i)
        
        self.Categorize()
        return None
        
    def CheckContactFeet(self):
        body_point = np.array([0., 0., -self.robot.body.l_end])
        p = self.robot.GetContactFeet(True)
        p_1 = p[-1]
        try: 
            p_2 = p[-2]
        except:
            p_2 = []
            
        for leg in [1, 2, 3, 4]:
            if leg in p_1 and leg not in p_2: #touchdown
                if len(p) > 1:
                    for name in self.swing_feet_name_list[leg - 1]:
                        self.RemoveTask(name)
                for i, name in enumerate(self.contact_feet_name_list[leg - 1]):
                    if name not in self.names:
                        self.AddTask(self.feet_body_id_list[leg - 1], \
                                body_point, self.world_normal_list[i], 1, name, True)
                                    
            elif leg not in p_1 and (leg in p_2 or p_2 == []): #liftoff
                if len(p) > 1:
                    for name in self.contact_feet_name_list[leg - 1]:
                        self.RemoveTask(name)
                for i, name in enumerate(self.swing_feet_name_list[leg - 1]):
                    if name not in self.names:
                        self.AddTask(self.feet_body_id_list[leg - 1], \
                                body_point, self.world_normal_list[i], 2, name)
                
                
                
                
#        if p_2!=p_1 or len(p) == 1:
#            for j in [1, 2, 3, 4]:
#                i = [i for i, e in enumerate(self.bodies) if e == \
#                self.feet_body_id_list[j - 1] and np.allclose(self.points[i],\
#                body_point) and self.constraint[i]]
#                if j in p_1:
#                    if not i:
#                        for k in [0, 1, 2]:
#                            self.AddTask(self.feet_body_id_list[j - 1], \
#                            body_point, [1, 0, 0], 1, \
#                            self.contact_feet_name_list[j - 1][k], True)
#                        if len(p) > 1:
#                            
#                        
#                else:
#                    if i and len(p) > 1:
#                        for k in i: 
#                            print self.names[k], self.robot.t[-1]
#                            self.RemoveTask(self.names[k])
                            
        return None
        
    def Categorize(self):
        self.p1 = [i for i, e in enumerate(self.priorities) if e == 1]
        self.p2 = [i for i, e in enumerate(self.priorities) if e == 2]
        self.p3 = [i for i, e in enumerate(self.priorities) if e == 3]
        self.pc = [i for i, e in enumerate(self.constraint) if e]
        return None
        
    def ComputeTaskParam(self):
        self.ComputeJandDJ()     
        self.ComputeN()
        return None
        
    def __ComputeJandDJ_aux(self, p, pc):
        J_A = np.array([])
        dJ_A_dq = np.array([])
        for i in p:
            x = [x for x in self.task_kinematic \
            if x[0] == self.bodies[i] and np.allclose(x[1], self.points[i])]
            J, dJ = x[0][5], x[0][3]
            
            if i in pc: dJ = - dJ
            
#            print self.names[i]
            
            J_A = np.append(J_A, J[self.normals[i].index(True), :])
            
            dJ_A_dq = np.append(dJ_A_dq, dJ[self.normals[i].index(True)])
            
        return J_A.reshape(len(p), self.robot.qdim), dJ_A_dq
        
    def ComputeJandDJ(self):
        self.J_A, self.dJ_A_dq = self.__ComputeJandDJ_aux(self.p1, self.pc)
        self.J_B, self.dJ_B_dq = self.__ComputeJandDJ_aux(self.p2, self.pc)
        self.J_E, self.dJ_E_dq = self.__ComputeJandDJ_aux(self.p3, self.pc)
        return None
        
    def ComputeN(self):
        self.N_A = np.eye(self.J_A.shape[1]) - np.dot(pinv(self.J_A), self.J_A)
        JAB = np.vstack((self.J_A, self.J_B))
        self.N_AB = np.eye(JAB.shape[1]) - np.dot(pinv(JAB), JAB)        
        return None
        
    def ComputeQddotDes(self):
        self.CheckContactFeet()
        self.ComputeTaskKinematics()
        self.ComputeXddotDes()
        self.ComputeTaskParam()        
        qddot = np.dot(pinv(self.J_A), self.xddot_des[self.p1] - self.dJ_A_dq)
        if self.J_B.shape[0]:
            qddot += np.dot(np.dot(self.N_A, pinv(self.J_B)), \
            self.xddot_des[self.p2] - self.dJ_B_dq)
        if self.J_E.shape[0]:
            qddot += np.dot(np.dot(self.N_AB, pinv(self.J_E)), \
            self.xddot_des[self.p3] - self.dJ_E_dq)
        return qddot
        
    def ComputeTaskKinematics(self):
        """
        this function should compute position, velocity, acceleration (this
        one with qddot = 0), Jacobian, and jacobian derivatives times angular 
        velocity for all the bodies (and points) present in the 
        task set.
        """
        mytask = list(zip(self.bodies, self.points))
        mytask = [list([elem[0], list(elem[1])]) for elem in mytask]
        unique_bp = [e for i, e in enumerate(mytask) if mytask.index(e) == i]
        
        self.task_kinematic = []
        for body_id, point in unique_bp:
            point = np.array(point)
            
            x, xdot = self.robot.CalcBodyToBase(body_id, point, True)
            
            xddot_qddotZero = self.robot.CalcAcceleration(self.robot.q[-1, :],\
            self.robot.qdot[-1, :], np.zeros(self.robot.qdim), body_id, point)
            
            J = self.robot.calcJc(self.robot.model, self.robot.q[-1, :],\
            body_id, point)
            
            if body_id == self.robot.body.id('pelvis') and \
            np.allclose(point, np.zeros_like(point)):
                if [br for br in self.base_rotation_name_list if br in self.names]:
                    x, xdot, xddot_qddotZero, J = self.KinematicsBaseRotation(\
                    x, xdot, xddot_qddotZero, J)
            
            self.task_kinematic.append([body_id, point, x, xdot, \
            xddot_qddotZero, J])
            self.temp = self.task_kinematic
        return None
        
        
    def SetTaskReference(self, *task_tuples):
        """
        input form: (task_name_list, [[xddot_des1, xdot_des1, x_des1], [...], ...]), (another_task_name_list, ...), ....
        \nfor a task:
            At least either of xddot_des or xdot_des or x_des must be provided.\n
            It is strongly recommended to provide xddot_des.\n
            If xddot_des is not provided, it is strongly recommended to provide
            both xdot_des and x_des.\n
            If neither provided, the previously provided values will be used.\n
            If no value at all, then xddot_des will set to zero.

        """
        for i in range(len(task_tuples)):
            for j in range(len(task_tuples[i][0])):
                self.__xddot_des[task_tuples[i][0][j]] = task_tuples[i][1][j]
        return None
        
    def SetContactReference(self):
        for name in sum(self.contact_feet_name_list, []):
            if name in self.names:
                self.__xddot_des[name] = [0, 0, None]
        return None
                

        
    def GetXddotDes(self):
        return self.__xddot_des
        
    def ComputeXddotDes(self):
        self.SetContactReference()
        kp = 400.
        kd = kp/5
        self.xddot_des = np.array([])
        for name in self.names:
            i = self.names.index(name)
            accel = 0
            try:
                if self.__xddot_des[name][0] is not None:
                    accel += self.__xddot_des[name][0]
                    
                if self.__xddot_des[name][1] is not None:
                    
                    kinematics = [x for x in self.task_kinematic \
                    if x[0] == self.bodies[i] and np.allclose(x[1], self.points[i])] 
                    
                    accel += kd*(self.__xddot_des[name][1] - \
                    kinematics[0][3][self.normals[i].index(True)])
                
                if  self.__xddot_des[name][2] is not None:
                    if self.__xddot_des[name][1] is None:
                        kinematics = [x for x in self.task_kinematic \
                        if x[0] == self.bodies[i] and np.allclose(x[1], self.points[i])]
                    
                        
                    accel += kp*(self.__xddot_des[name][2] - \
                    kinematics[0][2][self.normals[i].index(True)])
                    
                self.xddot_des = np.append(self.xddot_des, accel)
                    
            except: 
                self.xddot_des = np.append(self.xddot_des, 0)
        
        
        return self.xddot_des
        
        
    def AddTask_BaseRotation(self, rotation_axis, priority):
        rotation_axis = [0, 0, 0] + list(rotation_axis)
        self.AddTask(self.robot.body.id('trunk'), np.zeros(3), rotation_axis,\
        priority, self.base_rotation_name_list[rotation_axis.index(True) - 3])
        return self.base_rotation_name_list[rotation_axis.index(True) - 3]
        
    def KinematicsBaseRotation(self, x, xd, xdd, J):
        """
        TODO: Quaternion or Rotation matrix should be used instead of 
        angular position, which is not cumulative!
        for keeping all the angular positions of pelvis equal to zero,
        the current formulation may work.
        """
        x = np.append(x, self.robot.q[-1, 3:6])
        xd = np.append(xd, self.robot.qdot[-1, 3:6])
        xdd = np.append(xdd, np.zeros(3))
        J = np.vstack((J, np.zeros((3, self.robot.qdim))))
        J[3, 3] = 1.
        J[4, 4] = 1.
        J[5, 5] = 1.
        return x, xd, xdd, J

        
    
  

        
        
        