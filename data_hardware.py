
import matplotlib.pyplot as plt
import numpy as np
import sys
from os.path import expanduser
from robot_class import ROBOT
home = expanduser("~")
dir = home + '/rbdl/build/python'
sys.path.append(dir)
import rbdl

with open("real_time.txt","r") as file:
    real_time_list = eval(file.readline())

with open("q_calf.txt","r") as file:
    data2 = eval(file.readline())
with open("q_hip.txt","r") as file:
    data0 = eval(file.readline())
with open("q_thigh.txt","r") as file:
    data1 = eval(file.readline())

with open("contact_list.txt","r") as file:
    data3 = eval(file.readline())
# print(data0[-1])
s = 0
while data3[s] ==1:
    s = s+1
s = s-1

print(real_time_list[0]-real_time_list[s])

i = -1
while data3[i] == 1:
    # print(i)
    i = i-1
t = i
i = i+1

while data3[t] == 0:
    t = t-1
# t = -178    
while data3[t] == 1:
    t = t-1
print("t:",t)    
t = t+1
# j = j-1
hip_stance= []
thigh_stance = []
calf_stance = []
time_stance = []
# i = -91
r = 0
while (t<=-178):
     time_stance.append(real_time_list[t])
     hip_stance.append(data0[t])
     thigh_stance.append(data1[t])
     calf_stance.append(data2[t])
     t = t+1 

print("len##############",len(hip_stance))
print("len##############",len(thigh_stance))
print("len##############",len(calf_stance))
# print(i+1)    
# print(data0[i+1])
# print(data1[i+1])
# print(data2[i+1])



hip = []
thigh = []
calf = []
time = []
# i = -91
 
while (i <= -1):
     time.append(real_time_list[i])
     hip.append(data0[i])
     thigh.append(data1[i])
     calf.append(data2[i])
     i = i+1 

with open("hip.txt","w") as file:
   file.write(str(hip))
with open("thigh.txt","w") as file:
   file.write(str(thigh))
with open("calf.txt","w") as file:
   file.write(str(calf))



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

q_td = []
q_td.append(hip[0])
q_td.append(thigh[0])
q_td.append(calf[0])
path =  "/home/lenovo/Desktop/leg_RBDL.urdf"
robot = ROBOT(np.zeros((3, 1)), np.zeros((3, 1)), path) #model

# q_td = [-1.4929808499275197, -1.2581445029373555, -3.293392843518731]
qdot_td = [0.0805,5.044, -5.0757]#hard code estimate
qdot_td = rbdl2robot_vel(qdot_td[0],qdot_td[1],qdot_td[2])


# t_vec = np.linspace(0, 0.3, 50)
t_td = time[0]
t_des = time[-1]
q_hip = []
q_thigh = []
q_calf = []
time_list = []
z_list = []
i =0
for t in time:
    q = cubic_to_slip(t, t_td, t_des, q_td, qdot_td)
    # print(q)
    q_hip.append(q[0])
    q_thigh.append(q[1])
    q_calf.append(q[2])
    q_rbdl = robot2rbdl(hip[i], thigh[i], calf[i])
    q_rbdl = np.array(q_rbdl)
    z_list.append(-robot.pose_end(q_rbdl)[2]) 
    time_list.append(t)
    i=i+1
y = 0
z = []
for m in time_stance:
    q_rbdl = robot2rbdl(hip_stance[y], thigh_stance[y], calf_stance[y])
    q_rbdl = np.array(q_rbdl)
    z.append(-robot.pose_end(q_rbdl)[2])
    y = y + 1


# for t in t_vec:
#     q = cubic_to_slip(t, t_td, t_des, q_td, qdot_td)
#     q_hip.append(q[0])
#     q_thigh.append(q[1])
#     q_calf.append(q[2])
#     q_rbdl = np.array(robot2rbdl(q[0], q[1], q[2]))
#     z_list.append(-robot.pose_end(q_rbdl)[2]) 
#     time_list.append(t)


# print("last RBDL:", robot2rbdl(q[0], q[1], q[2]))
# print(robot.pose_end(np.array(robot2rbdl(q[0], q[1], q[2]))))
time_total =  time_list
plt.figure()
plt.plot(time_list, q_hip,"--")
plt.plot(time_list, q_thigh, "--")
plt.plot(time_list, q_calf, "--")
plt.scatter(time_list, hip, s=5)
plt.scatter(time_list, thigh, s=5)
plt.scatter(time_list, calf, s=5)
plt.legend(["hip_com","thigh_com","calf_com","hip_robot","thigh_robot","calf_robot"], loc="lower right")
plt.title("q in stance/compression")
# time_slip = np.array(time_slip)
# time_list = np.array(time_list)
# time_total = np.hstack((time_list,time_slip))
# print(len(time_list))
# print(len(time_slip))
# time_total.append(time_slip)
plt.figure()
plt.scatter(time_stance,z,s=4)
plt.title("slider hieght in stance")
plt.show()

print(rbdl2robot(q[0], q[1], q[2]))





# data_cycle = np.array(data_cycle)
# print("data cycle:", np.shape(data_cycle))
# print("data cycle:", data_cycle)
# print(data_cycle[:,0,1])

# plt.figure()
# plt.plot(data_cycle[:,0])
# plt.show()









