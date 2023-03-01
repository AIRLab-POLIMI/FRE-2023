import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

import numpy as np 

class LaserReader(Node):
    def __init__(self):
        super().__init__('laser_reader')

        self.area = np.array([1.4, 2]) # rect shape x,y

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.filter_pub = self.create_publisher(LaserScan, '/scan/filtered', 1)



    def scan_callback(self, msg): 
        coord = self.laser_scan_to_points(msg) # converts laser point to numpy array
        
        selected_coord = self.mask(coord) # takes only coordinate inside the area specified by self.area
        # convert selected coord into a laser scan message and publish them throught our publisher
        if np.any(selected_coord):

            self.filter_pub.publish(self.points_to_scan(selected_coord, msg))



    def laser_scan_to_points(self, msg):
        ranges = np.array(msg.ranges) # Converting ranges field into a numpy array 
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=msg.angle_increment) # Return evenly spaced value of angles based on its index 

        x = ranges * np.cos(angles) # array of all the x coordinates in 2D
        y = ranges * np.sin(angles) # array of all the y coordinates in 2D
        
        
        return np.vstack((x, y, ranges)).T #Stack arrays in sequence vertically ([len(), 2]) ndarray
    
    def mask(self, points: np.ndarray):
        x_mask = (points[:, 0] >= 0) & (points[:, 0] <= self.area[1]) # return a boolean array, after AND 
        y_mask = (points[:, 1] >= -self.area[0]/2) & (points[:, 1] <= self.area[0]/2)#self.area[1])
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
        

        

    
def main(args=None):
    rclpy.init(args=args)

    laser_reader = LaserReader()

    rclpy.spin(laser_reader)

    laser_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    