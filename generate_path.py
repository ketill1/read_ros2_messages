import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.image as mpimg
import yaml
import math
import numpy as np



class Path:
    def __init__(self, ax):
        self.fig_size = [12, 8]
        self.pattern = [[1, 0, 1], [1, 1, 2], [0, 1, 1], [0, 2, 0]]
        self.same_start_end = False
        self.goals = 0
        # self.path_to_map = path_to_map
        self.grid = False
        self.goal_angle = math.pi/2
        self.ax = ax
        # self.fig, self.ax = plt.subplots(sharex = True, figsize=self.fig_size)
        # self.fig.subplots_adjust(bottom=0.2)

        # with open(r'/home/ketill/dev_ws/src/basic_mobile_robot/maps/'+ path_to_map +'/map.yaml') as file:
        #    map_yaml = yaml.load(file, Loader=yaml.FullLoader)

        # self.map_origin = map_yaml["origin"]
        # self.map_resolution = map_yaml["resolution"]
        # self.img = mpimg.imread('/home/ketill/dev_ws/src/basic_mobile_robot/maps/' + path_to_map + '/map.pgm')
        # self.dim_map = [len(self.img[0])*self.map_resolution, len(self.img)*self.map_resolution]
        # self.ax.imshow(self.img, extent=[0, self.dim_map[0], 0, self.dim_map[1]], cmap="gist_gray")

        self.fixed_size = [0, 0]

    def generate_path(self, size_path_x, size_path_y, path_density):
        # Max iterations that can fit x
        self.fixed_size[0] = int(size_path_x // path_density) * path_density
        # Max iterations that can fit x
        self.fixed_size[1] = int(size_path_y // path_density) * path_density

        wave = 0; wave_number = 1
        path = [[0,0,0]]
        y = 0; idx = 0
        limit = not self.grid
        while(y <= self.fixed_size[limit]+0.0001):
            
            x = self.pattern[wave][0] * self.fixed_size[self.grid]
            y = self.pattern[wave][1] * path_density + (wave_number-1) * path_density * 2   
            yaw = self.pattern[wave][2] * self.goal_angle
            
            path.append([x, y ,yaw])
            wave += 1
            idx +=1
            if wave > 3:
                wave = 0
                wave_number += 1  
        path.pop()
            
        # Change last goal rotation
        path[-1][2] *= -1

        return path
    
    def rotate_and_move(self, path, theta, path_offset):
        for idx, points in enumerate(path):
            x = points[0]
            y = points[1]
            path[idx][0] = (x * math.cos(theta) - y * math.sin(theta)) + path_offset[0]
            path[idx][1] = (x * math.sin(theta) + y * math.cos(theta)) + path_offset[1]
            path[idx][2] += theta

        return path
        
    def generate_grid_path(self, size_path_x, size_path_y, path_density):

        path_a = self.generate_path(size_path_x, size_path_y, path_density)
        self.grid = True
        
        if len(path_a) % 4 == 0:
            self.same_start_end = True
        
        path_b = self.generate_path(size_path_x, size_path_y, path_density)

        if self.same_start_end:
            path_offset = [0, self.fixed_size[1]]
            path_b = self.rotate_and_move(path_b, -math.pi/2, path_offset)
        else:
            for idx in range(len(path_b)):
                path_b[idx][1] *= -1
                path_b[idx][2] *= -1
            path_offset = self.fixed_size
            path_b = self.rotate_and_move(path_b, -math.pi/2, path_offset)
            
        path = path_a + path_b[1:]

        self.goals = len(path_b)
        self.same_start_end = False
        self.grid = False
        
        return path
    
    def plot_path(self, path):
        # self.ax.imshow(self.img, extent=[0, self.dim_map[0], 0, self.dim_map[1]], cmap="gist_gray")
        # self.ax.grid()
        i = -1.5 #-self.map_origin[0]
        j = -1 #-self.map_origin[1]
        x = []
        y = []
        self.ax.add_line(Line2D([path[0][0]+i, path[1][0]+i], [path[0][1]+j, path[1][1]+j], color='r', linewidth=1, label='Planed path'))
        for idx, p in enumerate(path):
            # dx = arrow_size*math.cos(p[2])
            # dy = arrow_size*math.sin(p[2])
            if idx <= len(path)-self.goals: color = "r"
            else: color = "r"      
            # self.ax.arrow(p[0]+i, p[1]+j, dx, dy, color=color, head_width=arrow_size*0.5) 
            if idx: self.ax.add_line(Line2D([p_old[0]+i, p[0]+i], [p_old[1]+j, p[1]+j], color=color, linewidth=1))
            x.append(p[0])
            y.append(p[1])
            p_old = p
