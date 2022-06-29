import numpy as np
import sys
from os.path import expanduser
import scipy.integrate as integrate

home = expanduser("~")
dir = home + '/projects/rbdl/build/python'
sys.path.append(dir)
import rbdl


def calcM(q):
    M = np.zeros((model.q_size, model.q_size))
    rbdl.CompositeRigidBodyAlgorithm(model, q, M, True)
    return M


def calch(q, qdot):
    h = np.zeros(model.q_size)
    rbdl.InverseDynamics(model, q, qdot, np.zeros(model.qdot_size), h)
    return h


def dyn(x, t):
    q = x[:model.q_size]
    qd = x[model.q_size:]
    M = calcM(q)
    h = calch(q, qd)
    qddot = np.dot(np.linalg.inv(M), (tau - h))
    dx = np.concatenate((qd, qddot.flatten()))
    return dx


# Create a new model
model = rbdl.loadModel("/home/nooshin/python_simulation/leg_RBDL.urdf")
# print("model q size:",model.q_size)
# model = rbdl.Model()


# Create numpy arrays for the state
q = np.zeros(model.q_size).reshape(1, model.q_size)
qdot = np.zeros(model.qdot_size).reshape(1, model.q_size)
qddot = np.zeros(model.qdot_size)
tau = np.zeros(model.qdot_size)

# Modify the state
# q[0] = 1.3
# q[1] = -0.5
# q[2] = 3.2
dt = 0.005

# Perform forward dynamics and print the result
# rbdl.ForwardDynamics (model, q, qdot, tau, qddot)
# print("qddot = ", qddot)
print("q is :", q)
print("qdot is:", qdot)

print("tau: ", tau)
t = np.linspace(0, 10, 101)
i=1
#################################### forwardDynamics
if i == 1:
    qqdot0 = np.concatenate((q[-1,:], qdot[-1, :])).reshape(1, model.q_size*2)
    i=2

qqdot0 = integrate.odeint(dyn, qqdot0[-1,:], np.array([0,dt]))
print(qqdot0)
