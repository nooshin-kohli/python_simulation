"""
@author: Nooshin Kohli
"""
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import sys
from os.path import expanduser

home = expanduser("~")
dir = home + '/projects/rbdl/build/python'
sys.path.append(dir)
import rbdl
import numpy as np
from leg_robotclass import ROBOT



def Anim_leg(model, body, joint, Q, time):

    plt.close('all')
    
    def update_joint(model, body, joint, Q):
        jpose = {}
        q = Q
        l_end = -0.240
#        if i == 1: print q[1]
        for j in joint.joints:
            if j[:9] != 'reference':
                child = joint.parent_child_body(j)[1]
                pose = rbdl.CalcBodyToBaseCoordinates(model, q, child, np.zeros(3))
                jpose[j] = pose
                
        # manually adding foottips positions
        pose = rbdl.CalcBodyToBaseCoordinates(model, q, model.GetBodyId('calf'), \
        np.array([0., 0., l_end]))
        jpose['ftip_1'] = pose
#        print pose
        
#        pose = rbdl.CalcBodyToBaseCoordinates(model, q, body.id('knee_3'), \
#        np.array([0., 0., -body.l_end]))
#        jpose['ftip_3'] = pose
#        
#        pose = rbdl.CalcBodyToBaseCoordinates(model, q, body.id('knee_4'), \
#        np.array([0., 0., -body.l_end]))
#        jpose['ftip_4'] = pose
        # print(jpose)
        return jpose

    
        
    def update_pairs(jpose, joint):
        data = []
        paired = list(joint.pairs())
        
        #manually adding ftip 'virtual' joints:
        paired.append(['calf_joint', 'ftip_1']) 
#        paired.append(['knee_pitch_3', 'ftip_3']) 
#        paired.append(['knee_pitch_4', 'ftip_4']) 
        
        for i in range(len(paired)):
            p1 = jpose[paired[i][0]]
            p2 = jpose[paired[i][1]]
            data.append([[p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]]])
            
#            if paired[i][0] == 'j_arm1_6': print p1
            
        return np.array(data, dtype = float)
    t = np.array([0])
    dt = .002
    print(update_joint(model,body,joint,Q))
    jpose = update_joint(model,body,joint,Q)
    print(update_pairs(jpose,joint))
    
        
    def update_limbs(i, Q, limbs, model, body, joint, time):
        jpose = update_joint(model, body, joint, Q)
#        print jpose['hip_yaw_1'][1]
        data = update_pairs(jpose, joint)
        print ("=====\n======")
        # print (data)
        shape = data.shape[0]
        ############################################# TODO:for 2d plot uncomment this ############################################# 
        # for h in range(shape):
        #     data[h][1] = data[h][2]
        print(data)
        for line, dat in zip(limbs, data):
            line.set_data(dat[0:2,])
            # print(line.set_data(dat[0:2,]))
            line.set_3d_properties(dat[2, :])
            # print("line:",line)
            # print("dat:",dat)
            
        time_text.set_text(time_template%(time[i]))
        
        return limbs, time_text

    
    
    
#    # Attaching 3D axis to the figure
    fig = plt.figure(figsize=(10, 10))
    fig.set_dpi(100)
    ax = p3.Axes3D(fig)
    # t = np.array([0])
    # dt = .002
    # print(update_joint(model,body,joint,Q))
    # jpose = update_joint(model,body,joint,Q)
    # print(update_pairs(jpose,joint))
    # print(update_limbs(0,Q,['hip','thigh'],model,body,joint,t))

    # fig, ax = plt.subplots(figsize=(18, 6))

    ini_data = np.array([[0, 0], [0, 0],[0, 0]])
    limbs = [ax.plot(ini_data[1, :], ini_data[0, :],ini_data[2, :],'.-', lw = 3)[0] for i in range(len(joint.pairs())+2)]

    
    time_text = ax.text(0, 0,0, '', transform=ax.transAxes, fontsize=16)
    time_template = 'time: %.2f (s)'
    
    
    
    args = (Q, limbs, model, body, joint, time)
    robot_ani = animation.FuncAnimation(fig, update_limbs, len(Q), fargs=args,
                               interval=1, blit=False, repeat_delay=5000)
    
    
    ax.set_xlim([-.25, 1.5])
    ax.set_xlabel('X')
    # set this to equal for 2d fig
    ax.axis('auto') 
    ax.set_ylim([0, 1.5])
    
    ax.plot([-1, 1], [0, 0], 'k')
    
    ax.set_ylabel('Y')
#    
    ax.set_zlim3d([0.0, 2.])
    ax.set_zlabel('Z')

    ax.set_title('single leg') 
    
   
    
    plt.show()
    f = zoom_factory(ax)
    
    
    return robot_ani
    
    
def Plot_base_coordinate(cr,q,qdot):
    
    x, xdot = [], []
    
    for i in range(len(cr.t)):
    
        xx, xxdot = cr.CalcBodyToBase(cr.model.GetBodyId('jump'),np.zeros(3), True, index = i,q=q,qdot=qdot)
            
        x.append(xx); xdot.append(xxdot)
    
    x = np.array(x); xdot = np.array(xdot)
        
    plt.figure()
    plt.subplot(311)
    plt.grid(1)
    plt.plot(cr.t, x[:, 0])
    plt.ylabel('x')
    plt.subplot(312)
    plt.grid(1)
    plt.plot(cr.t, x[:, 1])
    plt.ylabel('y')
    plt.subplot(313)
    plt.grid(1)
    plt.plot(cr.t, x[:, 2])
    plt.ylabel('z')
    
    plt.figure()
    plt.grid(1)
    plt.subplot(311)
    plt.grid(1)
    plt.plot(cr.t, xdot[:, 0])
    plt.ylabel('$\dot{x}$')
    plt.subplot(312)
    plt.grid(1)
    plt.plot(cr.t, xdot[:, 1])
    plt.ylabel('$\dot{y}$')
    plt.subplot(313)
    plt.grid(1)
    plt.plot(cr.t, xdot[:, 2])
    plt.ylabel('$\dot{z}$')
    
    plt.show()
    return None
    
def Plot_foot_tip(cr):
    
    point = np.array([0., 0., cr.calf_length])
    body_ids = [cr.model.GetBodyId('calf')]
    
    for num, leg in enumerate(body_ids):  
        x, xdot = [], []
        for i in range(len(cr.t)):
#            print num, leg, i
            xx, xxdot = cr.CalcBodyToBase(leg, point, True, index = i)
                
            x.append(xx); xdot.append(xxdot)
        
        x = np.array(x); xdot = np.array(xdot)
        plt.figure('foot '+repr(num + 1))
        plt.subplot(211)
        plt.grid(1)
        plt.plot(cr.t, x)
        plt.ylabel('position')
        plt.subplot(212)
        plt.grid(1)
        plt.plot(cr.t, xdot)
        plt.ylabel('velocity')
        
    plt.show()
    return None
    
    
def Plot_contact_force(cr):
    p = cr.getContactFeet(True)
    for leg in [1]:  
        force = []
        for i in range(len(cr.t) - 1):
            if leg in p[i]:
                f = cr.cforce[i]
                f = f[leg*2 - 2:leg*2]
                
            else: f = np.ones(2)*np.nan
                
            force.append(f)
        
        force = np.array(force)
#        print force.shape
#        plt.figure('foot '+repr(leg))
        fig, ax = plt.subplots()
#        pan_zoom = PanAndZoom(fig)  # Add support for pan and zoom

        plt.title('foot '+repr(leg))
        plt.grid(1)
        plt.plot(cr.t[:-1], force, '*-')
        plt.ylabel('ground reaction force')
        plt.legend(['x', 'y', 'z'])
        f = zoom_factory(ax)
        
    plt.show()
    
    return None
    
def Plot_ff_fb(cr, u_fb):
    plt.figure('Input')
    plt.subplot(222)
    plt.title('Total')
    plt.plot(cr.t, cr.u)
    
    
    plt.subplot(221)
    plt.title('Feedforward')
    plt.plot(cr.t, cr.u - u_fb)
    
    
    plt.subplot(223)
    plt.title('Feedback')
    plt.plot(cr.t, u_fb)
    
    
    plt.subplot(224)
    plt.title('Comparison')
    plt.plot(cr.t, np.clip(np.abs(u_fb/cr.u*100), 0, 100))
    
    

    plt.show()
    return None
    
def traj_plan(t_start, t_end, z_start, z_end, traj_type = 'Quantic'):
        
    x, xd, xdd = z_start
    y, yd, ydd = z_end
    
    if traj_type == 'Quantic':
        f = lambda x: [x**5, x**4, x**3, x**2, x, 1]
        fd = lambda x: [5*x**4, 4*x**3, 3*x**2, 2*x, 1, 0]
        fdd = lambda x: [20*x**3, 12*x**2, 6*x, 2, 0, 0]
        
        
        A = np.array([f(t_start), fd(t_start), fdd(t_start),\
                      f(t_end), fd(t_end), fdd(t_end)])
                          
        traj, traj_d, traj_dd = [], [], []                 
        for i in range(len(x)):
            B = np.array([[x[i], xd[i], xdd[i], y[i], yd[i], ydd[i]]]).T           
            p = np.dot(np.linalg.inv(A), B)
            traj += list([lambda x, p=p: sum([p[0]*x**5, p[1]*x**4, p[2]*x**3, \
            p[3]*x**2, p[4]*x, p[5]])])              
            traj_d.append(lambda x, p=p: sum([p[0]*5*x**4, p[1]*4*x**3, \
            p[2]*3*x**2, p[3]*2*x, p[4]]))                
            traj_dd.append(lambda x, p=p: sum([p[0]*20*x**3, p[1]*12*x**2, \
            p[2]*6*x, p[3]*2]))
        return [traj, traj_d, traj_dd]
        


def zoom_factory(ax, base_scale = 1.04):
    def zoom_fun(event):
        # get the current x and y limits
        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()
        cur_xrange = (cur_xlim[1] - cur_xlim[0])*.5
        cur_yrange = (cur_ylim[1] - cur_ylim[0])*.5
        xdata = event.xdata # get event x location
        ydata = event.ydata # get event y location
        if event.button == 'up':
            # deal with zoom in
            scale_factor = 1/base_scale
        elif event.button == 'down':
            # deal with zoom out
            scale_factor = base_scale
        else:
            # deal with something that should never happen
            scale_factor = 1
            print((event.button))
        # set new limits
        ax.set_xlim([xdata - cur_xrange*scale_factor,
                     xdata + cur_xrange*scale_factor])
        ax.set_ylim([ydata - cur_yrange*scale_factor,
                     ydata + cur_yrange*scale_factor])
        plt.draw() # force re-draw

    fig = ax.get_figure() # get the figure of interest
    # attach the call back
    fig.canvas.mpl_connect('scroll_event',zoom_fun)

    #return the function
    return zoom_fun
    
def Plot_contact_force_2(cr,desired,end_time,load):
    cf = cr.cforce
    cf_h_x = np.array([])
    cf_h_y = np.array([])
    t1 = cr.t[:-1]/end_time
    tt = np.asscalar(load['tt']) #touchdown time
    tl = np.asscalar(load['tl']) #lift-off time
    mhg = (cr.mass_hip + cr.mass_thigh + cr.mass_calf)*cr.g0
    # mfg = (cr.param.m1f + cr.param.m2f + cr.param.m3f)*cr.param.g0
    
    for i in range(len(cf)):
        if len(cf[i]) == 0:
            cf_h_x = np.append(cf_h_x,np.nan)
            cf_h_y = np.append(cf_h_y,np.nan)
        else:
            cf_h_x = np.append(cf_h_x,cf[i][0])
            cf_h_y = np.append(cf_h_y,cf[i][1])


    plt.figure()
    plt.title('foot 1')
    plt.plot(cr.t[:-1],cf_h_x,label='real x force')
    plt.plot(cr.t[:-1],cf_h_y,label='real y force')
    if desired:
        plt.plot(np.linspace(tt,tl,len(cr.t)-1),mhg*cr.slip_fun_grf_x_s(t1),label='desired x force')
        plt.plot(np.linspace(tt,tl,len(cr.t)-1),mhg*cr.slip_fun_grf_y_s(t1),label='desired y force')
    plt.legend()
    plt.grid()
def plot_contact_feet(p_array,robot):
    f1 = np.array([])
    # f2 = np.array([])
    for i in range(len(p_array)):
      f11 = contact_gen(p_array[i],0)
      f1 = np.append(f1,f11)
    #   f22 = contact_gen(p_array[i],1)
    #   f2 = np.append(f2,f22)
    
    plt.figure()
    plt.subplot(211)
    plt.plot(robot.t,f1)
    plt.title('hind leg')
    # plt.subplot(212)
    # plt.plot(robot.t,f2 , 'r')
    # plt.xlabel('fore leg')
    plt.show()
    
    return f1 
        
    
    
def contact_gen(cf,fn):
    if len(cf) == 2 :
        return cf[fn]
    if len(cf) == 1 :
        if fn+1 == cf[0]:
            return cf[0]
        else: return 0
    if len(cf) == 0:
        return 0

mode = 'slider'
# initiate time array
t = np.array([0])
dt = .002  # step size

# initiate stats with dummy values
q = np.zeros(4) # joint position
# q[3]=0.3
qdot = np.zeros(4) # joint velocity
u = np.zeros(4) # control inputs

p = [[]] # the contact feet
# strange behavior when contact = [[1, 2]] and the legs are upright!!!!

# instanciate robot object:
cr = ROBOT(t, dt, q, p, mode, qdot, u)
# print(cr.body.bodies)
# print(cr.joint)
anim = Anim_leg(cr.model,cr.body,cr.joint,q,t)
plot = Plot_base_coordinate(cr,q,qdot)
force = Plot_contact_force(cr)
# print(anim)


#
#
# x_prev = np.array([0., 0., .85])
# xdot_prev = np.zeros(3)
# for ii in range(len(cr.t)):     
#    x_des_now = np.array([x_des_t[i](cr.t[ii]) for i in rlx]).flatten()
#    xdot_des_now = np.array([xdot_des_t[i](cr.t[ii]) for i in rlx]).flatten()
#    xddot_des_now = np.array([xddot_des_t[i](cr.t[ii] + dt) for i in rlx]).flatten()
   
#    try: Dt = cr.t[-1] - cr.t[-2]
#    except: Dt = dt
   
#    xdot_new = xdot_prev + Dt*xddot_des_now
#    x_new = x_prev + Dt*xdot_new
   
#    x_prev, xdot_prev = x_new.copy(), xdot_new.copy()
   
   
   
#    plt.figure('x')
#    plt.plot(cr.t[ii], x_des_now[0], '*')
#    plt.plot(cr.t[ii], x_new[0], 'o')
   
#    plt.figure('xdot')
#    plt.plot(cr.t[ii], xdot_des_now[0], '*')
#    plt.plot(cr.t[ii], xdot_new[0], 'o')
   
#    plt.figure('xddot')
#    plt.plot(cr.t[ii], xddot_des_now[0], '*')
#    