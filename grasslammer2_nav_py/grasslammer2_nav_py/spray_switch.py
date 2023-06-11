import rclpy
from rclpy.node import Node
import serial
import time

from sensor_msgs.msg import LaserScan

import numpy as np 
import matplotlib.pyplot as plt
	
import matplotlib.patches as mpatches

class SpraySwitch(Node):
    def __init__(self):
        super().__init__('spray_switch')

        self.area = np.array([-0.1, 0.1, 0.5, 0.5]) # rect shape x,y, width, height as the one from mppatches
    
        self.ser = serial.Serial('/dev/ttyACM0', 9600)
        self.point_treshold = 10

        self.scan_sub = self.create_subscription(LaserScan, '/scan_initial', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.fig, self.ax = plt.subplots()



    def scan_callback(self, msg): 
        coord = self.laser_scan_to_points(msg) # converts laser point to numpy array
        
        selected_coord_r = self.mask(coord, 'right') # takes only coordinate inside the area specified by self.area
        
        # Mask used to filter fake points 
        mask_1 = (selected_coord_r[:, 0] == 0) & (selected_coord_r[:, 1] == 0)
        mask_2 = (selected_coord_r[:, 0] == -1) & (selected_coord_r[:, 1] == -1)
        selected_coord_r = selected_coord_r[~mask_1 & ~mask_2]

        selected_coord_l = self.mask(coord, 'left') # takes only coordinate inside the area specified by self.area
        
        # Mask used to filter fake points 
        mask_3 = (selected_coord_l[:, 0] == 0) & (selected_coord_l[:, 1] == 0)
        mask_4 = (selected_coord_l[:, 0] == -1) & (selected_coord_l[:, 1] == -1)
        selected_coord_l = selected_coord_l[~mask_3 & ~mask_4]



        self.visualize(coord, selected_coord_r, selected_coord_l)

        if selected_coord_l.shape[0] > self.point_treshold:
            self.ser.write('G/l'.encode()+b'\n') 
        elif selected_coord_l.shape[0] <= self.point_treshold:
            self.ser.write('b/l'.encode()+b'\n') 
        if selected_coord_r.shape[0] > self.point_treshold:
            self.ser.write('G/r'.encode()+b'\n') 
        elif selected_coord_r.shape[0] <= self.point_treshold:
            self.ser.write('b/r'.encode()+b'\n') 


        # if np.any(selected_coord):

        #     self.filter_pub.publish(self.points_to_scan(selected_coord, msg))



    def laser_scan_to_points(self, msg):
        ranges = np.array(msg.ranges) # Converting ranges field into a numpy array 
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max-msg.angle_min)/720)#msg.angle_increment) # Return evenly spaced value of angles based on its index 

        x = ranges * np.cos(angles) # array of all the x coordinates in 2D
        y = ranges * np.sin(angles) # array of all the y coordinates in 2D
        
        
        return np.vstack((x, y, ranges)).T #Stack arrays in sequence vertically ([len(), 2]) ndarray
    
    def mask(self, points: np.ndarray, side: str):
        x_mask = (points[:, 0] >= self.area[0]) & (points[:, 0] <= self.area[0]+self.area[2]) # return a boolean array, after AND 
        if (side == 'right'): 
            y_mask = (points[:, 1] >= -(self.area[1]+self.area[3])) & (points[:, 1] <= -self.area[1])
        elif (side == 'left'): 
            y_mask = (points[:, 1] >= self.area[1]) & (points[:, 1] <= self.area[1]+self.area[3])
        range_mask = points[:,2] >=0
        mask = x_mask & y_mask & range_mask

        masked_points = np.full_like(points, -1)
        masked_points[mask] = points[mask]
        return masked_points

    def points_to_scan(self, points: np.ndarray, msg: LaserScan):
        
        #angle_increment = (msg.angle_max - msg.angle_min) / (points.shape[0] - 1)
        # polar to cartesian 
        ranges = points[:,2]
        #np.sqrt(points[:, 0]**2 + points[:,1]**2)

        nmsg = LaserScan()
        nmsg.header.stamp = msg.header.stamp
        nmsg.header.frame_id = msg.header.frame_id
        nmsg.angle_min = msg.angle_min
        nmsg.angle_max = msg.angle_max
        nmsg.angle_increment = msg.angle_increment
        nmsg.range_min = msg.range_min
        nmsg.range_max = msg.range_max
        nmsg.ranges = ranges.tolist()
        return nmsg
        

    def visualize(self, points, selected_points_r, selected_points_l):
        self.ax.clear()
        rect_r=mpatches.Rectangle((self.area[0],self.area[1]),self.area[2],self.area[3], 
                        fill = False,
                        color = "purple",
                        linewidth = 2)
        rect_l=mpatches.Rectangle((self.area[0],-(self.area[1]+self.area[3])),self.area[2],self.area[3], 
                        fill = False,
                        color = "purple",
                        linewidth = 2)
        plt.scatter(points[:, 0], points[:, 1], color='blue')
        plt.scatter(selected_points_r[:, 0], selected_points_r[:, 1], color='red')
        plt.scatter(selected_points_l[:, 0], selected_points_l[:, 1], color='green')
        plt.xlim(-3,3)
        plt.ylim(-3,3)
        plt.gca().add_patch(rect_r)
        plt.gca().add_patch(rect_l)
        self.fig.canvas.draw()
        plt.pause(0.01)

        

    
def main(args=None):
    rclpy.init(args=args)

    spray_switch = SpraySwitch()

    rclpy.spin(spray_switch)

    spray_switch.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    