# loading in modules
# import sqlite3
import csv
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import sqlite3

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import math


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
class Data:

    def __init__(self):
        self.arrival_time_twist = []
        self.arrival_time_linear = []
        self.position_x = []
        self.orientation_z = []
        self.total_orientation_z = []
        self.velocity_x = []
        self.velocity_angular = []

    def get_data(self, path_linear, path_twist):


        with Reader(path_linear) as reader:
            i = 0
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/odometry/filtered':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    
                    if i==0:
                        first_sec = msg.header.stamp.sec
                        i = 1
                    sec = msg.header.stamp.sec - first_sec
                    nanosec = msg.header.stamp.nanosec * 10 ** -9
                    self.arrival_time_linear.append(sec + nanosec)

                    self.position_x.append(msg.pose.pose.position.x)
                    # position_y.append(msg.pose.pose.position.x)
                    self.velocity_x.append(msg.twist.twist.linear.x)
                    
        # create reader instance and open for reading
        with Reader(path_twist) as reader:
            i = 0
            total_orientation = 0
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/odometry/filtered':
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    
                    if i==0:
                        first_sec = msg.header.stamp.sec
                        i = 1
                    sec = msg.header.stamp.sec - first_sec
                    nanosec = msg.header.stamp.nanosec * 10 ** -9
                    self.arrival_time_twist.append(sec + nanosec)

                    ox = msg.pose.pose.orientation.x
                    oy = msg.pose.pose.orientation.y
                    oz = msg.pose.pose.orientation.z 
                    ow = msg.pose.pose.orientation.w
                    # orientation_z.append(euler_from_quaternion(ox, oy, oz, ow))
                    # print(euler_from_quaternion(ox, oy, oz, ow))
                    self.orientation_z.append(math.sqrt(euler_from_quaternion(ox, oy, oz, ow) ** 2))
                    # Check for sign
                    if len(self.orientation_z) > 1:
                        total_orientation += math.sqrt((self.orientation_z[-1] - self.orientation_z[-2]) ** 2)
                    self.total_orientation_z.append(total_orientation)
                    self.velocity_angular.append(msg.twist.twist.angular.z)
                    # print(msg.twist.twist.angular)

# # Encoder
# path_list1 = ["/home/ketill/python/ekf_data/ekg_filter_encoder/rosbag2_2022_12_17-17_39_50",
#     "/home/ketill/python/ekf_data/ekg_filter_encoder/rosbag2_2022_12_17-17_46_17"]

# # IMU
# path_list2 = ["/home/ketill/python/ekf_data/ekg_filter_imu/rosbag2_2022_12_17-17_13_58",
#     "/home/ketill/python/ekf_data/ekg_filter_imu/rosbag2_2022_12_17-17_34_19"]

# # Enocer & IMU
# path_list3 = ["/home/ketill/python/ekf_data/ekg_filter_imu_encoder/rosbag2_2022_12_17-17_02_04",
#     "/home/ketill/python/ekf_data/ekg_filter_imu_encoder/rosbag2_2022_12_17-17_08_59"]

# Encoder
path_list1 = ["/home/ketill/Downloads/ekg_filter2/encoder/rosbag2_2022_12_19-20_14_44",
    "/home/ketill/Downloads/ekg_filter2/encoder/rosbag2_2022_12_19-20_17_12"]

# IMU
path_list2 = ["/home/ketill/python/ekf_data/ekg_filter_imu/rosbag2_2022_12_17-17_13_58",
    "/home/ketill/python/rosbag2_2022_12_20-20_50_23"]

# Enocer & IMU
path_list3 = ["/home/ketill/Downloads/ekg_filter2/filter/rosbag2_2022_12_19-19_56_00",
    "/home/ketill/Downloads/ekg_filter2/filter/rosbag2_2022_12_19-20_29_57"]

path_list = [path_list1, path_list2, path_list3]
print(*path_list[0])

fig, ax = plt.subplots(nrows=2, ncols=2, figsize=[12,10])
ax[0,0].spines['top'].set_visible(False)
ax[0,0].spines['right'].set_visible(False)
ax[0,0].spines['bottom'].set_visible(False)
ax[0,0].spines['left'].set_visible(False)

ax[0,1].spines['top'].set_visible(False)
ax[0,1].spines['right'].set_visible(False)
ax[0,1].spines['bottom'].set_visible(False)
ax[0,1].spines['left'].set_visible(False)

ax[1,0].spines['top'].set_visible(False)
ax[1,0].spines['right'].set_visible(False)
ax[1,0].spines['bottom'].set_visible(False)
ax[1,0].spines['left'].set_visible(False)

ax[1,1].spines['top'].set_visible(False)
ax[1,1].spines['right'].set_visible(False)
ax[1,1].spines['bottom'].set_visible(False)
ax[1,1].spines['left'].set_visible(False)

d1 = Data()
d1.get_data(*path_list[0])
ax[0,0].plot(d1.arrival_time_linear, d1.position_x, 'r', linestyle='dotted', label='Encoder')
ax[0,1].plot(d1.arrival_time_linear, d1.velocity_x, 'r', linestyle='dotted', label='Encoder')
ax[1,0].plot(d1.arrival_time_twist, d1.total_orientation_z, 'r', linestyle='dotted', label='Encoder')
ax[1,1].plot(d1.arrival_time_twist, d1.velocity_angular, 'r', linestyle='dotted', label='Encoder')

d2 = Data()
d2.get_data(*path_list[1])
ax[0,0].plot(d2.arrival_time_linear[0:1000], d2.position_x[0:1000], 'b', linestyle='dashed', label='IMU')
ax[0,1].plot(d2.arrival_time_linear[0:800], d2.velocity_x[0:800], 'b', linestyle='dashed', label='IMU')
ax[1,0].plot(d2.arrival_time_twist, d2.total_orientation_z, 'b', linestyle='dashed', label='IMU')
ax[1,1].plot(d2.arrival_time_twist[0:-1], d2.velocity_angular[0:-1], 'b', linestyle='dashed', label='IMU')

d3 = Data()
d3.get_data(*path_list[2])
ax[0,0].plot(d3.arrival_time_linear, d3.position_x, 'g', linestyle='dashdot', label='Filtered')
ax[0,1].plot(d3.arrival_time_linear, d3.velocity_x, 'g', linestyle='dashdot', label='Filtered')
ax[1,0].plot(d3.arrival_time_twist, d3.total_orientation_z, 'g', linestyle='dashdot', label='Filtered')
ax[1,1].plot(d3.arrival_time_twist, d3.velocity_angular, 'g', linestyle='dashdot', label='Filtered')

ax[0,0].plot([0, d3.arrival_time_linear[-1]], [10, 10], 'k', linestyle='solid', label='Reference')
ax[0,1].plot([0, d3.arrival_time_linear[-1]], [0.5, 0.5], 'k', linestyle='solid', label='Reference')
ax[1,0].plot([0, d3.arrival_time_twist[-1]], [math.pi*10, math.pi*10], 'k', linestyle='solid', label='Reference')
ax[1,1].plot([0, d3.arrival_time_twist[-1]], [-1.0, -1.0], 'k', linestyle='solid', label='Reference')


#ax[0,0].set_title("Position")
#ax[0,1].set_title("Velocity")
# ax.set(xlim=(-2, 3), ylim=(-2, 2))
ax[0,0].legend(loc='upper left')
ax[0,1].legend(loc='upper left')
ax[1,0].legend(loc='upper left')
ax[1,1].legend(loc='upper left')

ax[0,0].set_xlabel('Time sec')
ax[0,1].set_xlabel('Time sec')
ax[1,0].set_xlabel('Time sec')
ax[1,1].set_xlabel('Time sec')
ax[0,0].set_ylabel('Distance[m]')
ax[0,1].set_ylabel('Velocity[m/s]')
ax[1,0].set_ylabel('Rotations[Rad]')
ax[1,1].set_ylabel('Velocity[Rad/s]')

plt.show()
plt.clf()

a1 = [d1.position_x[-1]-10, sum(d1.velocity_x[500:1000])/500-0.5, d1.total_orientation_z[-1]-math.pi*10, sum(d1.velocity_angular[500:1000])/500+1]
a2 = [d2.position_x[-1]-10, sum(d2.velocity_x[500:1000])/500-0.5, d2.total_orientation_z[-1]-math.pi*10, sum(d2.velocity_angular[500:1000])/500+1]
a3 = [d3.position_x[-1]-10, sum(d3.velocity_x[500:1000])/500-0.5, d3.total_orientation_z[-1]-math.pi*10, sum(d3.velocity_angular[500:1000])/500+1]
print(a1)
print(a2)
print(a3)
