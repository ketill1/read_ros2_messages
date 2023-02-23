# loading in modules
# import sqlite3
import csv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.ticker import FormatStrFormatter
from matplotlib.patches import Rectangle, Circle
import sqlite3

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import math
import yaml
from scipy import ndimage

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
fig, ax = plt.subplots(1,2,figsize=[10,10])
ax[1].set_xlim((11,14))
ax[1].set_ylim((26,29))
ax[0].set_position([0.1, 0.1, 0.8, 0.8])
ax[1].set_position([0.6, 0.6, 0.35, 0.35])
rotation = 0
offset = [11, 8]
with open(r'/home/ketill/Downloads/inside_office/map.yaml') as file:
    map_yaml = yaml.load(file, Loader=yaml.FullLoader)
    map_origin = map_yaml["origin"]
    print(map_origin[0])
    map_resolution = map_yaml["resolution"]
    width = 50; height = 50
    img = mpimg.imread('/home/ketill/Downloads/inside_office/map.pgm')
    print(img.shape)

    ax[0].imshow(img, extent=[0, img.shape[1]*map_resolution, 0, img.shape[0]*map_resolution], cmap="gist_gray")
    ax[1].imshow(img[:height, :width], extent=[0, width*map_resolution+map_origin[0], 0, height*map_resolution], cmap="gist_gray")

def rotate_and_move(x, y, theta, path_offset):
    l1 = []
    l2 = []
    for i in range(len(x)):
        l1.append((x[i] * math.cos(theta) - y[i] * math.sin(theta)) + path_offset[0])
        l2.append((x[i] * math.sin(theta) + y[i] * math.cos(theta)) + path_offset[1])
    return l1, l2

class Data:

    def __init__(self):
        self.arrival_time_linear = []
        self.position_x = []
        self.position_y = []

    def get_data(self, path_linear):


        with Reader(path_linear) as reader:
            i = 0
            k=0
            color = ["b","g","y","b","k"]
            for j in range(0,8,8):
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == '/plan':
                        msg = deserialize_cdr(rawdata, connection.msgtype)
                        
                        if i==0:
                            first_sec = msg.header.stamp.sec
                            i = 1
                            print(msg.poses[0].pose.position.x)
                        sec = msg.header.stamp.sec - first_sec
                        nanosec = msg.header.stamp.nanosec * 10 ** -9
                        self.arrival_time_linear.append(sec + nanosec)
                        self.position_x.append(msg.poses[6].pose.position.x)
                        self.position_y.append(msg.poses[6].pose.position.y)
                l1, l2 = rotate_and_move(self.position_x, self.position_y, rotation*math.pi/180, offset)
                ax[0].plot(l1, l2, linestyle='solid', label='Planed path', color=color[k])
                ax[1].plot(l1, l2, linestyle='solid', label='Planed path', color=color[k])
                k+=1
                self.position_x=[]
                self.position_y=[]

path_list = ["/home/ketill/Downloads/obstacle_aviodance/rosbag2_2022_12_22-20_13_55"]
ax[0].spines['top'].set_visible(False)
d1 = Data()
d1.get_data(*path_list)


############################
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


ax[0].spines['top'].set_visible(False)
ax[0].spines['right'].set_visible(False)
ax[0].spines['bottom'].set_visible(False)
ax[0].spines['left'].set_visible(False)

ax[1].spines['top'].set_visible(False)
ax[1].spines['right'].set_visible(False)
ax[1].spines['bottom'].set_visible(False)
ax[1].spines['left'].set_visible(False)

# create reader instance and open for reading
with Reader('/home/ketill/Downloads/obstacle_aviodance/rosbag2_2022_12_22-20_13_59') as reader:
    
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



l1, l2 = rotate_and_move(a, b, rotation*math.pi/180, offset)
ax[0].plot(l1[0:-1:18], l2[0:-1:18], 'r', marker='o', markerfacecolor='none', linestyle='None', markersize=3, label='Robot path')
ax[1].plot(l1[0:-1:10], l2[0:-1:10], 'r', marker='o', markerfacecolor='none', linestyle='None', label='Robot path')
ax[0].add_patch(Rectangle((9.0,7.5), 0.8, 1.7, linewidth=1, edgecolor='orange', facecolor='orange'))
ax[0].add_patch(Rectangle((10.3,7.5), 1.3, 0.5, linewidth=1, edgecolor='orange', facecolor='orange'))
ax[0].add_patch(Rectangle((12.9,18), 0.5, 1.3, linewidth=1, edgecolor='orange', facecolor='orange'))
ax[0].add_patch(Circle((13,28), radius=0.3, linewidth=1, edgecolor='green', facecolor='green'))
ax[1].add_patch(Circle((13,28), radius=0.3, linewidth=1, edgecolor='green', facecolor='green'))


ax[0].legend(loc='upper left')
ax[0].set_xlabel('X[m]')
ax[0].set_ylabel('Y[m]')


plt.show()
plt.clf()

