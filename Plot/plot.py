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

"""
# plot robot trajectory
plt.plot(x_data, y_data, color='blue', linestyle='dashed', label='PT trajectory')
plt.plot(x_cosim, y_cosim, color='orange', linestyle='dashed', label='DT trajectory')
"""

simtime = 10

#plot steering angle controller performance
t1 = np.linspace(0, simtime, len(angle_data))
t2 = np.linspace(0, simtime, len(angle_cosim))
t3 = np.linspace(0, simtime, len(angle_cosim))
value = 1.5
reference = np.ones(len(t3))*value
plt.plot(t1, angle_data, color='blue', label='PT steer angle')
plt.plot(t2, angle_cosim, color='orange', label='DT steer angle')
plt.plot(t3, reference, color='black',linestyle='dashed', label='Reference')

#plot speed controller performance
_t1 = np.linspace(0, simtime, len(speed_data))
_t2 = np.linspace(0, simtime, len(speed_cosim))
_t3 = np.linspace(0, simtime, len(speed_cosim))
_value = 3
_reference = np.ones(len(_t3))*_value
plt.plot(_t1, speed_data, color='blue', label='PT speed')
plt.plot(_t2, speed_cosim, color='orange', label='DT speed')
plt.plot(_t3, _reference, color='black',linestyle='dashed', label='Reference')
"""
"""
#plt.xlabel("x")
#plt.ylabel("y")
plt.xlabel("time")
plt.ylabel("angle")
#plt.ylabel("speed")
plt.legend()
plt.show()