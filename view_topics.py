# loading in modules
# import sqlite3
import csv
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib.patches import Rectangle
import sqlite3
import time
import numpy as np
import math
from generate_path import Path
import statistics
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z # in radians

def RSD(data, reference):
    return math.sqrt(sum((reference - data[i]) ** 2 for i in range(len(data)))/(len(data)))


fig1, ax = plt.subplots()
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(False)

# create reader instance and open for reading
with Reader('/home/ketill/scrips/rosbag') as reader:
    
    map_frame_t = []
    map_frame_x = []
    map_frame_y = []
    map_frame_rot_x = []
    map_frame_rot_y = []
    map_frame_z = []
    map_frame_w = []
    ze = []

    odom_frame_t = []
    odom_frame_x = []
    odom_frame_y = []
    odom_frame_rot_x = []
    odom_frame_rot_y = []
    odom_frame_z = []
    odom_frame_w = []

    base_footprint_frame_t = []
    base_footprint_frame_x = []
    base_footprint_frame_y = []
    base_footprint_frame_theta = []

    a = []
    b = []
    c = []

    map_transform_arrived = False
    odom_transform_arrived = False
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/tf':
            msg = deserialize_cdr(rawdata, connection.msgtype)
            # print(msg.transforms[0].header.frame_id)

            if msg.transforms[0].header.frame_id == 'map':
                map_frame_t.append(msg.transforms[0].header.stamp.nanosec)
                map_frame_x.append(msg.transforms[0].transform.translation.x)
                map_frame_y.append(msg.transforms[0].transform.translation.y)
                map_frame_rot_x.append(msg.transforms[0].transform.rotation.x)
                map_frame_rot_y.append(msg.transforms[0].transform.rotation.y)
                map_frame_z.append(msg.transforms[0].transform.rotation.z)
                map_frame_w.append(msg.transforms[0].transform.rotation.w)
                map_transform_arrived = True

            if msg.transforms[0].header.frame_id == 'odom':
                odom_frame_t.append(msg.transforms[0].header.stamp.nanosec)
                odom_frame_x.append(msg.transforms[0].transform.translation.x)
                odom_frame_y.append(msg.transforms[0].transform.translation.y) 
                odom_frame_rot_x.append(msg.transforms[0].transform.rotation.x)
                odom_frame_rot_y.append(msg.transforms[0].transform.rotation.y)
                odom_frame_z.append(msg.transforms[0].transform.rotation.z)
                odom_frame_w.append(msg.transforms[0].transform.rotation.w)
                odom_transform_arrived = True

            if msg.transforms[0].header.frame_id == 'base_link':
                base_footprint_frame_t.append(msg.transforms[0].header.stamp.sec)
                base_footprint_frame_x.append(msg.transforms[0].transform.translation.x)
                base_footprint_frame_y.append(msg.transforms[0].transform.translation.y) 
                base_footprint_frame_theta.append(msg.transforms[0].transform.rotation.z)

            if map_transform_arrived and odom_transform_arrived:
                # odom_frame_x[-1] += (map_frame_x[-1] * math.cos(map_frame_theta[-1]) - map_frame_y[-1] * math.sin(map_frame_theta[-1]))
                # odom_frame_y[-1] += (map_frame_x[-1] * math.sin(map_frame_theta[-1]) + map_frame_y[-1] * math.cos(map_frame_theta[-1]))
                ze = euler_from_quaternion(map_frame_rot_x[-1], map_frame_rot_y[-1], map_frame_z[-1], map_frame_w[-1])
                
                # print(map_frame_t[-1]*10**-9)
                x = (odom_frame_x[-1]*math.cos(ze) - odom_frame_y[-1]*math.sin(ze))  + map_frame_x[-1]
                y = (odom_frame_x[-1]*math.sin(ze) + odom_frame_y[-1]*math.cos(ze)) + map_frame_y[-1]
                a.append(x)
                b.append(y)
                
        map_transform_arrived = False

plt.grid()
# p = Path(ax)
# path = p.generate_grid_path(3, 2, 1)
# p.plot_path(path)
ax.plot(a, b)


# ax.plot(a, b, 'b', linestyle='dashdot', label='Robot path')
# ax.add_patch(Rectangle((-0.2, -0.15), 0.4, 0.3, label='Start position', color='k'))
# ax.set(xlim=(-2, 3), ylim=(-2, 2))
plt.legend(loc='upper left')
plt.xlabel('X[m]')
plt.ylabel('Y[m]')
plt.show()
# fig2, ax = plt.subplots(nrows=2, ncols=2, figsize=[10,10])
# ax[0,0].spines['top'].set_visible(False)
# ax[0,0].spines['right'].set_visible(False)
# ax[0,0].spines['bottom'].set_visible(False)
# ax[0,0].spines['left'].set_visible(False)

# ax[0,1].spines['top'].set_visible(False)
# ax[0,1].spines['right'].set_visible(False)
# ax[0,1].spines['bottom'].set_visible(False)
# ax[0,1].spines['left'].set_visible(False)

# ax[1,0].spines['top'].set_visible(False)
# ax[1,0].spines['right'].set_visible(False)
# ax[1,0].spines['bottom'].set_visible(False)
# ax[1,0].spines['left'].set_visible(False)

# ax[1,1].spines['top'].set_visible(False)
# ax[1,1].spines['right'].set_visible(False)
# ax[1,1].spines['bottom'].set_visible(False)
# ax[1,1].spines['left'].set_visible(False)
# fig2.text(0.5, 0.04, 'Distance X[m]', ha='center', va='center')
# fig2.text(0.06, 0.5, 'Distance Y[m]', ha='center', va='center', rotation='vertical')
# start = 270; endpath = 420
# # Horizontal line 1
# ax[0,0].plot([-1.5,1.5], [0,0], 'r', label='Planed path')
# ax[0,0].plot(a[start:endpath], b[start:endpath], 'b', linestyle='dashdot', label='Robot path')
# ax[0,0].set_title('Horizontal path 1')
# ax[0,0].legend(loc='upper right')
# stdev1 = statistics.variance(a[start:endpath])
# # Formula for Residual Standard Deviation 

# # The residual standard deviation (RSD) is a measure of the dispersion of the residuals, 
# # which are the differences between the observed data points and the fitted values. 
# # The residual standard deviation is calculated as the square root of the mean squared 
# # error (MSE) of the residuals, which is the average of the squared differences between 
# # the observed data points and the fitted values.

# rsd1 = RSD(b[start:endpath], 0)
# print(rsd1)

# # Horizontal line 2
# start = 570; endpath = 730
# ax[0,1].plot([-1.5,1.5], [1,1], 'r', label='Planed path')
# ax[0,1].plot(a[start:endpath], b[start:endpath], 'b', linestyle='dashdot', label='Robot path')
# ax[0,1].set_title('Horizontal path 2')
# ax[0,1].legend(loc='upper right')
# rsd2 = RSD(b[start:endpath], 1)
# print(rsd2)

# # Vertical line 1
# start = 1000; endpath = 1170
# ax[1,0].plot([0.5,0.5], [-1,1], 'r', label='Planed path')
# ax[1,0].plot(a[start:endpath], b[start:endpath], 'b', linestyle='dashdot', label='Robot path')
# ax[1,0].set_title('Vertical path 1')
# ax[1,0].legend(loc='upper right')
# rsd3 = RSD(a[start:endpath], 0.5)
# print(rsd3)

# # Vertical line 1
# start = 1320; endpath = 1420
# ax[1,1].plot([-0.5,-0.5], [-1,1], 'r', label='Planed path')
# ax[1,1].plot(a[start:endpath], b[start:endpath], 'b', linestyle='dashdot', label='Robot path')
# ax[1,1].set_title('Vertical path 2')
# ax[1,1].legend(loc='upper right')
# rsd4 = RSD(a[start:endpath], -0.5)
# print(rsd4)

# average_rsd = (rsd1 + rsd2 + rsd3 + rsd4)/4
# print(average_rsd)

# ax[0,0].legend(loc='upper right')
# plt.show()

