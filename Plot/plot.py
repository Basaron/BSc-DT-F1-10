import matplotlib.pyplot as plt
import csv
import numpy as np
from pandas import *

# f1tenth simulator data
x_data = np.genfromtxt('x.csv', delimiter=',')
y_data = np.genfromtxt('y.csv', delimiter=',')
speed_data = np.genfromtxt('speed.csv', delimiter=',')
angle_data = np.genfromtxt('angle.csv', delimiter=',')

# cosim data
cosim_data = read_csv("outputs.csv")
x_cosim = cosim_data['{model}.m.x'].tolist()
y_cosim = cosim_data['{model}.m.y'].tolist()
speed_cosim = cosim_data['{model}.m.velocity'].tolist()
angle_cosim = cosim_data['{model}.m.steer_angle'].tolist()

# remove excess y data
n = len(y_data)-len(x_data)
for i in range(n):
    y_data = np.delete(y_data, len(y_data)-1)

fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
ax1.set_title('trajectory')
#ax1.xlabel("x")
#ax1.ylabel("y")

ax2.set_title('steer angle')
#plt.xlabel("time")
#plt.ylabel("angle")

ax3.set_title('speed ')
#plt.xlabel("time")
#plt.ylabel("speed")

# plot robot trajectory
ax1.plot(x_data, y_data, color='blue', linestyle='dashed', label='PT trajectory')
ax1.plot(x_cosim, y_cosim, color='orange', linestyle='dashed', label='DT trajectory')


simtime = 10

#plot steering angle controller performance
t1 = np.linspace(0, simtime, len(angle_data))
t2 = np.linspace(0, simtime, len(angle_cosim))
t3 = np.linspace(0, simtime, len(angle_cosim))
value = 0.2
reference = np.ones(len(t3))*value
ax2.plot(t1, angle_data, color='blue', label='PT steer angle')
ax2.plot(t2, angle_cosim, color='orange', label='DT steer angle')
ax2.plot(t3, reference, color='black',linestyle='dashed', label='Reference')

#plot speed controller performance
_t1 = np.linspace(0, simtime, len(speed_data))
_t2 = np.linspace(0, simtime, len(speed_cosim))
_t3 = np.linspace(0, simtime, len(speed_cosim))
_value = 3
_reference = np.ones(len(_t3))*_value
ax3.plot(_t1, speed_data, color='blue', label='PT speed')
ax3.plot(_t2, speed_cosim, color='orange', label='DT speed')
ax3.plot(_t3, _reference, color='black',linestyle='dashed', label='Reference')

ax1.legend()
ax2.legend()
ax3.legend()
plt.show()