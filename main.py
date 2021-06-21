import numpy as np
import matplotlib.pyplot as plt

from robot import Robot
from pid import PID

T = 5000
dt = 1e-3
p_gain = 10. 
d_gain = 10./np.sqrt(2) 
x0 =np.array([[2000.],[140.]])

#x0 = np.zeros([2, 1]) 
#print(np.zeros([2, 1]))

u_list = [0.]*T
x_list = [0.] *T
t_list = [0] *T
xa=np.array(x_list)
print(xa.shape)
theta_des = 0.
u_des = 0. 
t = 0.

robot = Robot(x0)
pid = PID(p_gain,0.,d_gain, t)

for i in range(T):

    print('simulation step: %d'%i)
    
    x = robot.getState()
    
    #######controller#######
    theta = x[0,0]
    pid.SetPoint = theta_des
    pid.update(theta, t)
    o=pid.output
    u = u_des - o
    #######controller#######

    x_list[i] = x
    u_list[i] = u
    robot.step(u)
    t += dt
    
    
xa=np.array(x_list)
ta=np.array(t_list)

xa.reshape(len(x_list),2)
plt.plot(t_list,xa[:,0])


plt.plot(t_list,xa[:,1])


plt.plot(t_list,u_list)

    

