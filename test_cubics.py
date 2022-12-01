import matplotlib.pyplot as plt
import numpy as np
from robot_class import ROBOT
qh1 = -1.411
qh2 = -0.6203
qh3 = -4.858275150682843
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
def rbdl2robot_vel(v1, v2, v3):
    v1, v2, v3 = fixq3inv([v1, v2, v3])
    rx_dot = [-v1 + 0, -v2 + 0, -v3 + 0]
    return rx_dot
def robot2rbdl_vel(v1, v2, v3):
    qdot = [-(v1 - 0), -(v2 - 0), -(v3 - 0)]
    qdot = fixq3(qdot)
    return qdot

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

def cubic_to_slip(t, t_td, t_des, q_td, qdot_td):
    T = t_des-t_td
    Tau = (t-t_td)/T
    q_td = np.array(q_td)
    qdot_td = np.array(qdot_td)
    qdot_des = np.zeros(3)
    q_des = np.array([0.032,1.2014,-1.819])
    delta_q = q_des - q_td
    rho_0 = 0
    rho_1 = (T*qdot_td)/delta_q
    rho_2 = (-2*qdot_td-qdot_des)*T/delta_q + 3
    rho_3 = (qdot_td+qdot_des)*T/delta_q -2
    q = q_td + delta_q*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    return q


def jump(velocity,robot,t_td,t_d,q_first):
    T = t_d - t_td
    Tau = (t-t_td)/T
    # print(t)
    q_end = np.array([0.022,0.7321,-1.05])
    # q_end = np.array(rbdl2robot(0.022,0.7321,-1.05)) # this hard code is from liftoff in slip model
    J = robot.j_hip(q_end)
    # print(J)
    qdot_end = np.dot(np.linalg.inv(J),velocity)
    print(qdot_end)
    if Tau>1:
        raise ValueError("tau > 1 in decomp")
    q_first = np.array(q_first)
    qdot_first = np.zeros(3)
    # qdot_end = np.zeros(3)
    delta_q = q_end - q_first
    rho_0 = 0
    rho_1 = (T*qdot_first)/delta_q
    rho_2 = (-2*qdot_first-qdot_end)*T/delta_q + 3
    rho_3 = (qdot_first+qdot_end)*T/delta_q -2
    q = q_first + delta_q*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    return q

def jump2(velocity,robot,t_td,t_d,q_first):
    T = t_d - t_td
    Tau = (t-t_td)/T
    # print(t)
    q_end = np.array([0.022,0.7321,-1.05])
    # q_end = np.array(rbdl2robot(0.022,0.7321,-1.05)) # this hard code is from liftoff in slip model
    J = robot.calcJc(q_end)
    # print(J)
    qdot_end = np.dot(np.linalg.inv(J),velocity)
    print(qdot_end)
    if Tau>1:
        raise ValueError("tau > 1 in decomp")
    q_first = np.array(q_first)
    qdot_first = np.zeros(3)
    # qdot_end = np.zeros(3)
    delta_q = q_end - q_first
    rho_0 = 0
    rho_1 = (T*qdot_first)/delta_q
    rho_2 = (-2*qdot_first-qdot_end)*T/delta_q + 3
    rho_3 = (qdot_first+qdot_end)*T/delta_q -2
    q = q_first + delta_q*(rho_0+rho_1*(Tau)+rho_2*(Tau**2)+rho_3*(Tau**3))
    return q

time = []
path = '/home/lenovo/python_simulation/python_simulation/leg_RBDL.urdf'
robot = ROBOT(np.zeros(3),np.zeros(3),path)
vel = [0, 0, -0.8]
vel = np.array(vel)
time_vec = np.linspace(0,0.2,100)
t_td = 0
t_d = 0.2
q_first = np.array([0.032, 1.2014, -1.819])
# q_first = np.array(rbdl2robot(0.032, 1.2014, -1.819))
q_vec = []
q2_vec = []
z_com = []
for t in time_vec:
    q = jump(vel, robot, t_td, t_d, q_first)
    q_vec.append(q)
    q2 = jump2(vel, robot, t_td, t_d, q_first)
    q2_vec.append(q2)
    z_com.append(robot.get_com(q=q,qdot=np.zeros(3))[2]) 
    # print(robot.get_com(q=q,qdot=np.zeros(3)))
    # y = cubic_comp(t, 0.668, 0.763, -0.366)
    # height.append(y)
    time.append(t)
q_vec = np.array(q_vec)
q2_vec = np.array(q2_vec)

# q_vec_2 = np.array(q_vec_2)
plt.figure()
plt.plot(time, q_vec[:, 0])
plt.plot(time, q_vec[:, 1])
plt.plot(time, q_vec[:, 2])

plt.plot(time, q2_vec[:, 0],"--")
plt.plot(time, q2_vec[:, 1],"--")
plt.plot(time, q2_vec[:, 2],"--")

# plt.plot(time, q_vec_2[:, 0])
# plt.plot(time, q_vec_2[:, 1])
# plt.plot(time, q_vec_2[:, 2])
plt.title("q cubic")
plt.legend(["hip","thigh","calf"],loc = "upper left")

plt.figure()
plt.plot(time, z_com)
plt.title("z COM")
plt.show()



