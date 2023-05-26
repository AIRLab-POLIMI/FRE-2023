import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import MarkerArray, Marker

# laser_mono = /scan_mono
# laser_mono = /scan_initial
# laser = /scan

import numpy as np 

class LaserReader(Node):
    def __init__(self):
        super().__init__('laser_reader')

        self.area = np.array([2,2]) # rect shape x,y
        self.area_end_of_line = np.array([1.2, 0.75]) # rect shape x,y
        # scan_out
        self.scan_sub = self.create_subscription(LaserScan, '/scan_final', self.scan_callback_extended, 1)
        self.scan_sub # prevent unused variable warning 
        self.filter_pub = self.create_publisher(LaserScan, '/scan/filtered', 1)
        self.filter_pub_end_of_line = self.create_publisher(LaserScan, '/scan/filtered_end_of_line', 1)
        self.cluster1_pub = self.create_publisher(MarkerArray, '/cluster1', 1)
        # sself.extended_area = np.array([1, 0.8]) # rect shape x,y


    def scan_callback(self, msg): 
        coord = self.laser_scan_to_points(msg) # converts laser point to numpy array
        
        selected_coord = self.mask(coord) # takes only coordinate inside the area specified by self.area
        selected_coord_end_of_line = self.mask_end_of_line(coord) # takes only coordinate inside the area specified by self.area
        
        # convert selected coord into a laser scan message and publish them throught our publisher
        if np.any(selected_coord):
            self.filter_pub.publish(self.points_to_scan(selected_coord, msg))
        
        # convert selected coord into a laser scan message and publish them throught our publisher
        if np.any(selected_coord_end_of_line):
            self.filter_pub_end_of_line.publish(self.points_to_scan(selected_coord_end_of_line, msg))
        
        #self.visualize_points_2d(selected_coord)

    def scan_callback_extended(self, msg): 
        coord = self.laser_scan_to_points(msg) # converts laser point to numpy array
        
        selected_coord = self.mask_extended(coord) # takes only coordinate inside the area specified by self.area
        
        # convert selected coord into a laser scan message and publish them throught our publisher
        if np.any(selected_coord):
            self.filter_pub.publish(self.points_to_scan(selected_coord, msg))
        
        #self.visualize_points_2d(selected_coord)

    def laser_scan_to_points(self, msg):
        ranges = np.array(msg.ranges) # Converting ranges field into a numpy array 
        # print(len(ranges))
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) # Return evenly spaced value of angles based on its index 

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

    # mask for end of line
    def mask_end_of_line(self, points: np.ndarray):
        x_mask = (points[:, 0] >= 0) & (points[:, 0] <= self.area_end_of_line[1]) # return a boolean array, after AND 
        y_mask = (points[:, 1] >= -self.area_end_of_line[0]/2) & (points[:, 1] <= self.area_end_of_line[0]/2)#self.area[1])
        range_mask = points[:,2] >=0
        mask = x_mask & y_mask & range_mask

        masked_points = np.full_like(points, -1)
        masked_points[mask] = points[mask]
        return masked_points

    # mask extended
    def mask_extended(self, points: np.ndarray):
        # print((points[:, 0] >= -self.area[1]) & (points[:, 0] <= self.area[1]))
        x_mask = (points[:, 0] >= -self.area[1]/2) & (points[:, 0] <= self.area[1]/2) # return a boolean array, after AND 
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
        

    def visualize_points_2d(self, points_2d): 
        marker_msgs = MarkerArray()
        for i in range(len(points_2d)):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "laser_frame"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = points_2d[i, 0]
            marker.pose.position.y = points_2d[i, 1]
            marker.pose.position.z = 0.0
            marker_msgs.markers.append(marker)

        self.cluster1_pub.publish(marker_msgs)

    
def main(args=None):
    rclpy.init(args=args)

    laser_reader = LaserReader()

    rclpy.spin(laser_reader)

    laser_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    