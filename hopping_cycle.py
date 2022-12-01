'''
Yazdi,Kohli
Created: Sep. 12, 2021

This code make the robot jump as many time as needed. 
'''
import sys
import os
userdir = os.path.expanduser('~')
sys.path.append(userdir+"/projects/actuator")
from actuator import Actuator
from homing import home
import matplotlib.pyplot as plt
import time
import numpy as np
from extract_data import DATA
from robot_class import ROBOT
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)
contact_pin = 13
GPIO.setup(contact_pin, GPIO.IN)



data = DATA()
hopping_data = data.get_pose_data_f()
hopping_velocity = data.get_vel_data_f()

with open("./just_stand/time_f40.txt", "r")as file:
    time_list= eval(file.readline())


def fixq3(q):
        r = 18./28.
        q[-1] = q[-1]*r
        return q
    
def fixq3inv(q):
        r = 28./18.
        q[-1] = q[-1]*r
        return q
################################################## ROBOT AND RBDL TRANSFORM
#################### convert position
def robot2rbdl(p1, p2, p3):
    q = [-(p1 - qh1), -(p2 - qh2), -(p3 - qh3)]
    q = fixq3(q)
    return q

def rbdl2robot(p1, p2, p3):
    p1, p2, p3 = fixq3inv([p1, p2, p3])
    rx = [-p1 + qh1, -p2 + qh2, -p3 + qh3]
    return rx

def detect_contact():
    a = GPIO.input(contact_pin)
    return a

def rbdl2robot_vel(v1, v2, v3):
    v1, v2, v3 = fixq3inv([v1, v2, v3])
    rx_dot = [-v1 + 0, -v2 + 0, -v3 + 0]
    return rx_dot
def robot2rbdl_vel(v1, v2, v3):
    qdot = [-(v1 - 0), -(v2 - 0), -(v3 - 0)]
    qdot = fixq3(qdot)
    return qdot

def cal_qdot(q_pre, q, t_pre, t_now):
    time_delta = t_now - t_pre
    qdot_cal = (q - q_pre)/time_delta
    return qdot_cal


def cubic_to_slip(t, t_td, t_des, q_td, qdot_td):
    T = t_des-t_td
    Tau = (t-t_td)/T
    if Tau>1:
        raise ValueError("tau > 1 in slip")
    q_td = np.array(q_td)
#     qdot_td = np.zeros(3)
    qdot_td = np.array(qdot_td)
    q_des = np.array(rbdl2robot(0.032,1.2014,-1.819)) # this hard code is from compression in slip model
    qdot_des = np.zeros(3)
    delta_q = q_des - q_td
    rho_0 = 0
    rho_1 = (T*qdot_td)/delta_q
    rho_2 = (-2*qdot_td-qdot_des)*T/delta_q + 3
    rho_3 = (qdot_td+qdot_des)*T/delta_q -2
    q = q_td + delta_q*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    return q


    
def cubic_comp(t, t_lo, t_ap, hip2calf_len):
    '''
    This function gets tip of the foot close to hip.
    '''
    T = t_ap - t_lo
    Tau = (t-t_lo)/T
    if Tau>1:
        raise ValueError("tau > 1 in comp")
    y_ap = 0.025             ##the height we want to descend in flight mode 
    y_lo = 0
    ydot_lo = 0
    ydot_ap = 0
    delta_y = y_ap - y_lo
    rho_0 = 0
    rho_1 = (T*ydot_lo)/delta_y
    rho_2 = (-2*ydot_lo-ydot_ap)*T/delta_y + 3
    rho_3 = (ydot_ap+ydot_lo)*T/delta_y -2
    y = y_lo + delta_y*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    y = hip2calf_len + y
    
    return y

def cubic_decomp(t, t_td, t_ap, hip2calf_len):
    '''
    This function gets tip of the foot far from hip.
    '''
    if detect_contact()==1:
        print(t)
        # cubic_to_slip(t, t_td, t_des, q_td, qdot_td)
    else:
        T = t_td - t_ap
        Tau = (t-t_ap)/T
        if Tau>1:
            raise ValueError("tau > 1 in decomp")
        y_ap = 0                    
        y_td = -0.025                  ## the height we want to ascend in flight mode
        ydot_td = 0
        ydot_ap = 0
        delta_y = y_td - y_ap
        rho_0 = 0
        rho_1 = (T*ydot_ap)/delta_y
        rho_2 = (-2*ydot_ap-ydot_td)*T/delta_y + 3
        rho_3 = (ydot_ap+ydot_td)*T/delta_y -2
        y = y_ap + delta_y*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
        y =  hip2calf_len + y  
        return y

def jump(velocity,robot,t_td,t_d,q_first):
    T = t_td - t_d
    Tau = (t-t_d)/T
    J = robot.calcJc(q_end)
    qdot_end = np.dot(np.linalg.inv(J),velocity)
    if Tau>1:
        raise ValueError("tau > 1 in decomp")
    q_first = np.array(q_first)
    qdot_first = np.zeros(3)
    q_end = np.array(rbdl2robot(0.022,0.7321,-1.05)) # this hard code is from liftoff in slip model
    qdot_end = np.zeros(3)
    delta_q = q_end - q_first
    rho_0 = 0
    rho_1 = (T*qdot_first)/delta_q
    rho_2 = (-2*qdot_first-qdot_end)*T/delta_q + 3
    rho_3 = (qdot_first+qdot_end)*T/delta_q -2
    q = q_first + delta_q*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    return q



def safety_check(pre_cond, cur_cond):
    if abs(cur_cond[0] - pre_cond[0]) > 0.15 :
        print("motor1 diff", cur_cond[0] - pre_cond[0])
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("big step detected!!!!!!!!!!")
        raise ValueError("motor1 had a big step!!!!!!!!!")
    if abs(cur_cond[1] - pre_cond[1]) > 0.15 :
        print("motor2 diff", cur_cond[1] - pre_cond[1])
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("big step detected!!!!!!!!!!")
        raise ValueError("motor2 had a big step!!!!!!!!!")
    if abs(cur_cond[2] - pre_cond[2]) > 0.2 :
        print("motor3 diff", cur_cond[2] - pre_cond[2])
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("big step detected!!!!!!!!!!")
        raise ValueError("motor3 had a big step!!!!!!!!!")
    
# plt.figure()
# plt.plot(time_list, hopping_data[0])#hip
# plt.plot(time_list, hopping_data[1])#thigh
# plt.plot(time_list, hopping_data[2])#calf
# plt.legend(["hip_q", "thigh_q", "calf_q"], loc="upper right")
# plt.title("desire q from RBDL")
# plt.figure()


robot_des_vel_hip = rbdl2robot_vel(hopping_velocity[0],0,0)
robot_des_vel_hip = robot_des_vel_hip[0]

robot_des_vel_thigh = rbdl2robot_vel(0,hopping_velocity[1],0)
robot_des_vel_thigh = robot_des_vel_thigh[1]

robot_des_vel_calf = rbdl2robot_vel(0,0,hopping_velocity[2])
robot_des_vel_calf = robot_des_vel_calf[2]


# plt.plot(time_list, robot_des_vel_hip)#hip
# plt.plot(time_list, robot_des_vel_thigh)#thigh
# plt.plot(time_list, robot_des_vel_calf)#calf
# plt.legend(["hip_vel", "thigh_vel", "calf_vel"], loc="upper right")
# plt.title("desire velocity for robot")
# plt.show()

hip_d = hopping_data[0]
thigh_d = hopping_data[1]
calf_d = hopping_data[2]

hip_vel_d = hopping_velocity[0]
thigh_vel_d = hopping_velocity[1]
calf_vel_d = hopping_velocity[2]




leg = Actuator('can0') #real robot
m1 = 0x01
m2 = 0x02
m3 = 0x05

# kp = 20
# kd = .5
# kp = 40
# kd = 0.8
kp = 60
kd = 0.9

leg.enable(m1)
leg.enable(m2)
leg.enable(m3)



q_home = home([leg, m1, m2, m3], kp=kp, kd=kd, enable_motors=False, disable_motors=False)
qh1 = q_home[0]
qh2 = q_home[1]
qh3 = q_home[2] - 3.7132 # different zero position

rx1 = leg.command(m1, qh1, 0, kp, kd, 0)
rx2 = leg.command(m2, qh2, 0, kp, kd, 0)
rx3 = leg.command(m3, 0, 0, 0, 0, 0) #TODO


############################################################## find the safe range during slip commands from data
hip_d_robot=[]
thigh_d_robot =[]
calf_d_robot = []

for i in range(len(hip_d)):
    hip_d_robot.append(rbdl2robot(hip_d[i],0,0)[0])
    thigh_d_robot.append(rbdl2robot(0,thigh_d[i],0)[1])
    calf_d_robot.append(rbdl2robot(0,0,calf_d[i])[2])
    
min_hip = min(hip_d_robot)
max_hip = max(hip_d_robot)
min_thigh = min(thigh_d_robot)
max_thigh= max(thigh_d_robot)
min_calf = min(calf_d_robot)
max_calf= max(calf_d_robot)
#############################################################
q_rbdl = robot2rbdl(rx1[1], rx2[1], rx3[1])
home_robot_q = [rx1[1],rx2[1],rx3[1]]

q_d_rbdl = [0.032, 1.201, -1.819]

############################################################## starting Initial positioning of HIP
xi = q_rbdl[0]
xf = q_d_rbdl[0]
dr = .02
if xi > xf: dr = -dr
pos = np.arange(xi, xf, dr)
dt = 1/len(pos)
tpre = time.time()
print("HIP:",pos)

time.sleep(2)
q_pre=rx1[1]

for p in pos:
    p_robot = rbdl2robot(p,0,0)
    diff = abs(q_pre - p_robot[0])
    if (diff > .1):
        print("diff:")
        print(diff)
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("Unsafe command to motors is detected!")
        time.sleep(0.5)
        raise ValueError("Unsafe command to motors is detected!")
    else:

        rx = leg.command(m1, p_robot[0], 0, kp, kd, 0)
        q_pre = rx[1]
        while(time.time() - tpre < dt): temp = 0
        tpre = time.time()
hip_motor_final = rx[1]
hip_rbdl_final = robot2rbdl(rx[1],0,0)[0]
time.sleep(1)

############################################################### starting initial positioning of calf

xi = q_rbdl[2]
xf = q_d_rbdl[2]
dr = .02
if xi > xf: dr = -dr
pos = np.arange(xi, xf, dr)
dt = 1/len(pos)
tpre = time.time()
print("CALF:", pos)
q_pre = rx3[1]
for p in pos:
    p_robot = rbdl2robot(0,0,p)
    diff = abs(q_pre - p_robot[2])

    if (diff > .12):
        print("diff:")
        print(diff)
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("Unsafe command to motors is detected!")
        time.sleep(0.5)
        raise ValueError("Unsafe command to motors is detected!")
    else:

        rx = leg.command(m3, p_robot[2], 0, kp, kd, 0)
        q_pre = rx[1]
        while(time.time() - tpre < dt): temp = 0
        tpre = time.time()

calf_motor_final = rx[1]
calf_rbdl_final = robot2rbdl(0,0,rx[1])[2]

time.sleep(1)
################################################################ starting initial positioning of THIGH
xi = q_rbdl[1]
xf = q_d_rbdl[1]
dr = .02
if xi > xf: dr = -dr
pos = np.arange(xi, xf, dr)
dt = 1/len(pos)
tpre = time.time()
q_pre = rx2[1]
print("THIGH:", pos)
for p in pos:
    p_robot = rbdl2robot(0,p,0)
    diff = abs(q_pre - p_robot[1])
    if (diff > .1):
        print("diff:")
        print(diff)
        rx1 = leg.command(m1, 0, 0, 0, 0, 0)
        rx2 = leg.command(m2, 0, 0, 0, 0, 0)
        rx3 = leg.command(m3, 0, 0, 0, 0, 0)
        leg.disable(m1)
        leg.disable(m2)
        leg.disable(m3)
        print ("Unsafe command to motors is detected!")
        time.sleep(0.5)
        raise ValueError("Unsafe command to motors is detected!")
    else:

        rx = leg.command(m2, p_robot[1], 0, kp, kd, 0)
        q_pre = rx[1]
        while(time.time() - tpre < dt): temp = 0
        tpre = time.time()


thigh_motor_final = rx[1]
thigh_rbdl_final = robot2rbdl(0,rx[1],0)[1]

time.sleep(1)
final_pose_motor=[hip_motor_final,thigh_motor_final,calf_motor_final]
final_pose_rbdl = [hip_rbdl_final,thigh_rbdl_final,calf_rbdl_final]
print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
print("final position of motors: ", final_pose_motor)
print("final position in rbdl: ", final_pose_rbdl)


time.sleep(20)

counter=1
t_first_cycle = time.time()
real_time_cycle = time.time() - t_first_cycle
real_time_cycle_list = []
data_cycle = []
contact_list_cycle = []


############################################################START HOPPING
while(counter<=4):

    kp = 60
    kd = 0.9

    # kp = 20
    # kd = 0.5
    time.sleep(0.1)

    path =  "leg_RBDL.urdf"
    robot = ROBOT(np.zeros((3, 1)), np.zeros((3, 1)), path) #model


    dt = 0.0025

    first_check = 0
    t_first = time.time()

    q_hip_com_1 = []
    q_thigh_com_1 = []
    q_calf_com_1 = []

    q_hip_motor_1 = []
    q_thigh_motor_1 = []
    q_calf_motor_1 = []

    qdot_hip_com_1 = []
    qdot_thigh_com_1 = []
    qdot_calf_com_1 = []

    qdot_hip_motor_1 = []
    qdot_thigh_motor_1 = []
    qdot_calf_motor_1 = []

    real_time_list_1 = []

    contact_list = []


    for i in range(len(hip_d)):
        real_time = time.time()-t_first
        real_time_list_1.append(real_time)
        
        robot_hip = rbdl2robot(hip_d[i],0,0)
        robot_hip = robot_hip[0]
        
        robot_thigh = rbdl2robot(0, thigh_d[i], 0)
        robot_thigh = robot_thigh[1]
        
        robot_calf = rbdl2robot(0,0,calf_d[i])
        robot_calf = robot_calf[2]
        ######################################################
        robot_hip_vel = rbdl2robot_vel(hip_vel_d[i],0,0)
        robot_hip_vel = robot_hip_vel[0]
        
        robot_thigh_vel = rbdl2robot_vel(0, thigh_vel_d[i], 0)
        robot_thigh_vel = robot_thigh_vel[1]
        
        robot_calf_vel = rbdl2robot_vel(0,0,calf_vel_d[i])
        robot_calf_vel = robot_calf_vel[2]
      
        
        if not ((min_hip-0.1) < robot_hip <(max_hip + 0.1)):
            print("hip:", robot_hip)
            rx1 = leg.command(m1, 0, 0, 0, 0, 0)
            rx2 = leg.command(m2, 0, 0, 0, 0, 0)
            rx3 = leg.command(m3, 0, 0, 0, 0, 0)
            leg.disable(m1)
            leg.disable(m2)
            leg.disable(m3)
            print ("Unsafe command to motors is detected!!!!!!!!!!")
            time.sleep(0.5)
            raise ValueError("your command position is not in the safe range!!!!!!!!!")
        elif not ((min_thigh-0.1) < robot_thigh <  (max_thigh+0.1)):
            print("thigh:", robot_thigh)
            rx1 = leg.command(m1, 0, 0, 0, 0, 0)
            rx2 = leg.command(m2, 0, 0, 0, 0, 0)
            rx3 = leg.command(m3, 0, 0, 0, 0, 0)
            leg.disable(m1)
            leg.disable(m2)
            leg.disable(m3)
            print ("Unsafe command to motors is detected!!!!!!!!!!")
            time.sleep(0.5)
            raise ValueError("your command position is not in the safe range!!!!!!!!!")

        elif not ((min_calf - 0.15) < robot_calf < (max_calf + 0.15)):
            print("calf:", robot_calf)
            rx1 = leg.command(m1, 0, 0, 0, 0, 0)
            rx2 = leg.command(m2, 0, 0, 0, 0, 0)
            rx3 = leg.command(m3, 0, 0, 0, 0, 0)
            leg.disable(m1)
            leg.disable(m2)
            leg.disable(m3)
            print ("Unsafe command to motors is detected!!!!!!!!!!")
            time.sleep(0.5)
            raise ValueError("your command position is not in the safe range!!!!!!!!!")

        else:
            
            rx1 = leg.command(m1, robot_hip, robot_hip_vel, kp, kd, 0)
            rx2 = leg.command(m2, robot_thigh, robot_thigh_vel, kp, kd, 0)
            rx3 = leg.command(m3, robot_calf, robot_calf_vel, kp, kd, 0)
            
            real_time_cycle = time.time() - t_first_cycle
            real_time_cycle_list.append(real_time_cycle)
            data_cycle.append([rx1,rx2,rx3])
            contact_list_cycle.append(detect_contact())
            
            q_hip_com_1.append(robot_hip)
            q_thigh_com_1.append(robot_thigh)
            q_calf_com_1.append(robot_calf)
            
            qdot_hip_com_1.append(robot_hip_vel)
            qdot_thigh_com_1.append(robot_thigh_vel)
            qdot_calf_com_1.append(robot_calf_vel)
            
            q_hip_motor_1.append(rx1[1])
            q_thigh_motor_1.append(rx2[1])
            q_calf_motor_1.append(rx3[1])
            
            qdot_hip_motor_1.append(rx1[2])
            qdot_thigh_motor_1.append(rx2[2])
            qdot_calf_motor_1.append(rx3[2])
            
            contact_list.append(detect_contact())

            while((time.time()-t_first) - real_time_list_1[-1] < dt): temp=0
    ##################################################################################FLIGHT MODE
    while(1):
        check = detect_contact()
        if check ==0:
            break



    q_rbdl = robot2rbdl(robot_hip, robot_thigh, robot_calf)
    hip2tip_len = robot.pose_end(np.array(q_rbdl))[2]
    home_pos = robot.pose_end(np.array(q_rbdl))

    z_com_2 = []
    z_tip_2 = []
    
    x_com_2 = []
    x_tip_2 = []
    
    y_com_2 = []
    y_tip_2 = []

    kp = [[750,0,0],
          [0,750,0],
          [0,0,700]]

    kd = [[0.02,0,0],
          [0,0.03,0],
          [0,0,0.02]]

    e_pre = np.zeros(3)
    t_pre = time.time()
    enter_decomp = 0

    torque_hip_com_2 = []
    torque_thigh_com_2 = []
    torque_calf_com_2 = []

    torque_hip_motor_2 = []
    torque_thigh_motor_2 = []
    torque_calf_motor_2 = []

    q_hip_motor_2 = []
    q_thigh_motor_2 = []
    q_calf_motor_2 = []

    q_hip_motor_2.append(robot_hip)
    q_thigh_motor_2.append(robot_thigh)
    q_calf_motor_2.append(robot_calf)

    t_first = time.time()
    real_time = time.time()- t_first
    t_lo = time.time()-t_first
    t_ap = t_lo + 0.09
    t_td = t_ap + 0.095
    real_time_list_2 = []
    contact_list_2 =[]
    while(real_time <= t_td and detect_contact() == 0):
        
        real_time = time.time() - t_first
        real_time_list_2.append(real_time)
        contact_list_2.append(detect_contact())
        ################################################compression
        
        if real_time <= t_ap :
            
            z_des = cubic_comp(real_time, t_lo, t_ap, hip2tip_len)
            z_com_2.append(z_des)
            x_com_2.append(home_pos[0])
            y_com_2.append(home_pos[1])
            pos_des = [home_pos[0], home_pos[1], z_des]
            end_pose = robot.pose_end(np.array(q_rbdl))
            error = pos_des - end_pose
            z_tip_2.append(end_pose[2])
            x_tip_2.append(end_pose[0])
            y_tip_2.append(end_pose[1])
            dt = time.time()-t_pre
            e_dot = (error - e_pre)/dt
            JC = robot.calcJc(np.array(q_rbdl))
            TAU = np.dot(JC.T, (np.dot(kp,error)+np.dot(kd,e_dot)))
            TAU = -TAU # rbdl orientetion is different from robot
            TAU[2] = TAU[2]*(28/18) # belt and pully on calf
            
            torque_hip_com_2.append( TAU[0])
            torque_thigh_com_2.append(TAU[1])
            torque_calf_com_2.append(TAU[2])
            
            rx1 = leg.command(m1, 0, 0, 0, 0, TAU[0])
            rx2 = leg.command(m2, 0, 0, 0, 0, TAU[1])
            rx3 = leg.command(m3, 0, 0, 0, 0, TAU[2])
            
            safety_check([q_hip_motor_2[-1],q_thigh_motor_2[-1],q_calf_motor_2[-1]],[rx1[1],rx2[1],rx3[1]])
            
            real_time_cycle = time.time() - t_first_cycle
            real_time_cycle_list.append(real_time_cycle)
            data_cycle.append([rx1,rx2,rx3])
            contact_list_cycle.append(detect_contact())
            
            
            torque_hip_motor_2.append(rx1[3])
            torque_thigh_motor_2.append(rx2[3])
            torque_calf_motor_2.append(rx3[3])
            
            q_hip_motor_2.append(rx1[1])
            q_thigh_motor_2.append(rx2[1])
            q_calf_motor_2.append(rx3[1])
            
            
            q_rbdl = np.array(robot2rbdl(rx1[1], rx2[1], rx3[1]))
            e_pre = error
            t_pre = time.time()
            real_time = time.time() - t_first
        ################################################decompression
        elif real_time > t_ap  and real_time <= t_td:
            if enter_decomp == 0:
    #             hip2tip_len = hip2tip_len + 0.025
                hip2tip_len = robot.pose_end(np.array(q_rbdl))[2]
                kp = [[700,0,0],
                      [0,700,0],
                      [0,0,650]]

                kd = [[0.02,0,0],
                      [0,0.04,0],
                      [0,0,0.02]]
                enter_decomp = 1
                
            z_des = cubic_decomp(real_time, t_td, t_ap, hip2tip_len)
            z_com_2.append(z_des)
            x_com_2.append(home_pos[0])
            y_com_2.append(home_pos[1])
            pos_des = [home_pos[0], home_pos[1], z_des]
            end_pose = robot.pose_end(np.array(q_rbdl))
            z_tip_2.append(end_pose[2])
            x_tip_2.append(end_pose[0])
            y_tip_2.append(end_pose[1])
            error = pos_des - end_pose
            dt = time.time()-t_pre
            e_dot = (error - e_pre)/dt
            JC = robot.calcJc(np.array(q_rbdl))
            TAU = np.dot(JC.T, (np.dot(kp,error)+np.dot(kd,e_dot)))
            TAU = -TAU # rbdl orientetion is different from robot
            TAU[2] = TAU[2]*(28/18) # belt and pully on calf
            
            torque_hip_com_2.append( TAU[0])
            torque_thigh_com_2.append(TAU[1])
            torque_calf_com_2.append(TAU[2])
            
            
            rx1 = leg.command(m1, 0, 0, 0, 0, TAU[0])
            rx2 = leg.command(m2, 0, 0, 0, 0, TAU[1])
            rx3 = leg.command(m3, 0, 0, 0, 0, TAU[2])
            
            safety_check([q_hip_motor_2[-1],q_thigh_motor_2[-1],q_calf_motor_2[-1]],[rx1[1],rx2[1],rx3[1]])
            
            real_time_cycle = time.time() - t_first_cycle
            real_time_cycle_list.append(real_time_cycle)
            data_cycle.append([rx1,rx2,rx3])
            contact_list_cycle.append(detect_contact())
            
            torque_hip_motor_2.append(rx1[3])
            torque_thigh_motor_2.append(rx2[3])
            torque_calf_motor_2.append(rx3[3])
            
            
            q_hip_motor_2.append(rx1[1])
            q_thigh_motor_2.append(rx2[1])
            q_calf_motor_2.append(rx3[1])
            
            
            q_rbdl = np.array(robot2rbdl(rx1[1], rx2[1], rx3[1]))
            e_pre = error
            t_pre = time.time()
            real_time = time.time() - t_first


    #################################################################TOCHDOWN MOMENT
    # rx1 = leg.command(m1, rx1[1], 0, 30, 0.5, 0)
    # rx2 = leg.command(m2, rx2[1], 0, 30, 0.5, 0)
    # rx3 = leg.command(m3, rx3[1], 0, 30, 0.5, 0)
    print("detect contact after while:", detect_contact())
    print("real_time - t_td = ", real_time - t_td)

    print("qdot_robot in touchdown: ", [rx1[2],rx2[2],rx3[2]])
    print("qdot_rbdl in toucdown: ", robot2rbdl_vel(rx1[2],rx2[2],rx3[2]))

    t_first = time.time()
    real_time = time.time()- t_first 
    real_time_list_3 = []
    t_td = time.time()- t_first
    t_des = t_td + 0.3
            
    q_td = [rx1[1], rx2[1] , rx3[1]] ### q touch down given from last phase

    qdot_td = [0.0805,5.044, -5.0757]#hard code estimate
    qdot_td = rbdl2robot_vel(qdot_td[0],qdot_td[1],qdot_td[2])



    q_hip_com_3 = []
    q_thigh_com_3 = []
    q_calf_com_3 = []

    q_hip_motor_3 = []
    q_thigh_motor_3 = []
    q_calf_motor_3 = []

    q_hip_motor_3.append(rx1[1])
    q_thigh_motor_3.append(rx2[1])
    q_calf_motor_3.append(rx3[1])


    kp=35
    kd=0.6

    while(real_time <= t_des):
        real_time = time.time() - t_first
        real_time_list_3.append(real_time)
       
        q_des = cubic_to_slip(real_time,t_td,t_des,q_td,qdot_td)
        print(q_des)
        
        rx1 = leg.command(m1, q_des[0], 0, kp, kd, 0)
        rx2 = leg.command(m2, q_des[1], 0, kp, kd, 0)
        rx3 = leg.command(m3, q_des[2], 0, kp, kd, 0)
        
        real_time_cycle = time.time() - t_first_cycle
        real_time_cycle_list.append(real_time_cycle)
        data_cycle.append([rx1,rx2,rx3])
        contact_list_cycle.append(detect_contact())
        
        q_hip_com_3.append(q_des[0])
        q_thigh_com_3.append(q_des[1])
        q_calf_com_3.append(q_des[2])
        
        safety_check([q_hip_motor_3[-1],q_thigh_motor_3[-1],q_calf_motor_3[-1]],[rx1[1],rx2[1],rx3[1]])    

        q_hip_motor_3.append(rx1[1])
        q_thigh_motor_3.append(rx2[1])
        q_calf_motor_3.append(rx3[1])
        real_time = time.time() - t_first
    print("this is the enddddd.... :)(")
    
#             
#         
# ################################################ phase 1 plots        
#     plt.figure()
#     plt.plot(real_time_list_1, q_hip_com_1,"--")
#     plt.plot(real_time_list_1, q_thigh_com_1,"--")
#     plt.plot(real_time_list_1, q_calf_com_1,"--")
#     plt.plot(real_time_list_1, q_hip_motor_1)
#     plt.plot(real_time_list_1, q_thigh_motor_1)
#     plt.plot(real_time_list_1, q_calf_motor_1)
#     plt.title("q_plots_PHASE1")
#     plt.legend(["hip_com","thigh_com","calf_com","hip_motor","thigh_motor","calf_motor"], loc = "upper left")
# 
# 
#     plt.figure()
#     plt.plot(real_time_list_1, qdot_hip_com_1,"--")
#     plt.plot(real_time_list_1, qdot_thigh_com_1,"--")
#     plt.plot(real_time_list_1, qdot_calf_com_1,"--")
#     plt.plot(real_time_list_1, qdot_hip_motor_1)
#     plt.plot(real_time_list_1, qdot_thigh_motor_1)
#     plt.plot(real_time_list_1, qdot_calf_motor_1)
#     plt.plot(real_time_list_1, contact_list)
#     plt.title("qdot_plots_PHASE1")
#     plt.legend(["hip","thigh","calf","hip_motor","thigh_motor","calf_motor","contact"], loc = "upper left")
# 
# 
# ################################################## phase2 plots
# 
# 
# 
#     plt.figure()
#     plt.plot(real_time_list_2,torque_hip_com_2,"--")
#     plt.plot(real_time_list_2,torque_thigh_com_2,"--")
#     plt.plot(real_time_list_2,torque_calf_com_2,"--")
# 
#     plt.plot(real_time_list_2, torque_hip_motor_2)
#     plt.plot(real_time_list_2, torque_thigh_motor_2)
#     plt.plot(real_time_list_2, torque_calf_motor_2)
#     plt.title("torques in flight phase")
#     plt.legend(["hip_com","thigh_com","calf_com","hip_m","thigh_m","calf_m"],loc = "upper left")
# 
# 
#     plt.figure()
#     plt.plot(real_time_list_2, z_tip_2)
#     plt.plot(real_time_list_2,z_com_2,"--")
#     plt.title("z_position of the tip point in flight phase")
#     plt.legend(["actual z position","z from qubic"],loc = "upper right")
# 
#     q_hip_motor_2.pop(0)
#     q_thigh_motor_2.pop(0)
#     q_calf_motor_2.pop(0)
# 
#     plt.figure()
#     plt.plot(real_time_list_2,q_hip_motor_2)
#     plt.plot(real_time_list_2,q_thigh_motor_2)
#     plt.plot(real_time_list_2,q_calf_motor_2)
#     plt.plot(real_time_list_2,contact_list_2)
#     plt.title("Q from motors in flight phase")
#     plt.legend(["hip","thigh", "calf","contact"], loc= "upper right")
# 
# 
#     ##############################################################PHASE 3 PLOTS
# 
#     q_hip_motor_3.pop(0)
#     q_thigh_motor_3.pop(0)
#     q_calf_motor_3.pop(0)
# 
#     plt.figure()
#     plt.plot(real_time_list_3,q_hip_com_3,"--")
#     plt.plot(real_time_list_3,q_thigh_com_3,"--")
#     plt.plot(real_time_list_3,q_calf_com_3,"--")
# 
#     plt.plot(real_time_list_3,q_hip_motor_3)
#     plt.plot(real_time_list_3,q_thigh_motor_3)
#     plt.plot(real_time_list_3,q_calf_motor_3)
#     plt.title("q_phase 3")
#     plt.legend(["q_hip_com","q_thigh_com","q_calf_com","q_hip_motor","q_thigh_motor","q_calf_motor"],loc="upper left")
#     
    ##############################prepare for another jump
    
    
    kp = 60
    kd = 0.9
    q_slip = np.array(rbdl2robot(0.032,1.2014,-1.819))
    rx1 = leg.command(m1, q_slip[0], 0, kp, kd, 0)
    rx2 = leg.command(m2, q_slip[1], 0, kp, kd, 0)
    rx3 = leg.command(m3, q_slip[2], 0, kp, kd, 0)
    
    real_time_cycle = time.time() - t_first_cycle
    real_time_cycle_list.append(real_time_cycle)
    data_cycle.append([rx1,rx2,rx3])
    contact_list_cycle.append(detect_contact())
    
    if (abs(q_slip[0]-rx1[1]) > 0.15) or (abs(q_slip[1] - rx2[1])> 0.15) or (abs(q_slip[2] - rx3[1]) > 0.15) : 
        raise ValueError("initial position is far from slip")
    
    counter+=1



data_cycle = np.array(data_cycle)

plt.figure()
plt.plot(real_time_cycle_list, data_cycle[:,0,1])   # [all datas, which motor, which data(id,q,qdot,curr)]
plt.plot(real_time_cycle_list, data_cycle[:,1,1])
plt.plot(real_time_cycle_list, data_cycle[:,2,1])
plt.plot(real_time_cycle_list, contact_list_cycle)
plt.title("whole cycle q from motor")
plt.legend(["hip","thigh","calf","contact"], loc = "upper right")

plt.figure()
plt.plot(real_time_cycle_list, data_cycle[:,0,3])
plt.plot(real_time_cycle_list, data_cycle[:,1,3])
plt.plot(real_time_cycle_list, data_cycle[:,2,3])
plt.plot(real_time_cycle_list, contact_list_cycle)
plt.title("whole cycle torques from motor")
plt.legend(["hip","thigh","calf","contact"], loc = "upper right")

plt.figure()
plt.plot(real_time_list_2, x_tip_2)
plt.plot(real_time_list_2,x_com_2,"--")

plt.plot(real_time_list_2, y_tip_2)
plt.plot(real_time_list_2,y_com_2,"--")

plt.plot(real_time_list_2, z_tip_2)
plt.plot(real_time_list_2,z_com_2,"--")

plt.title("position of the tip point in flight phase")
plt.legend(["actual x position","x from qubic","actual y position","z from qubic","actual x position","z from qubic"],loc = "upper right")


plt.show()










    
