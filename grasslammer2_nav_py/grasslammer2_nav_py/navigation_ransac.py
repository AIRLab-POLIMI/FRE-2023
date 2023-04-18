import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray, Marker

import math
import time
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
from collections import deque


# MA algorithm. Add consistency to goal point
number_points = 5
start_slope = 0
stop_slope = 2
start_intercept = 0
stop_intercept= 2
base_exp = 100
avg_execution = []


# global weight slope height
wieght_slope = 0.8
weight_intercept = 0.8


# num point needed to work properly
RANSAC_num_point = 1



class NavigationRansac(Node):
    def __init__(self):
        super().__init__('navigation_ransac')

        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 

        self.cmd_pub = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 1)
        
        # use deque to maintain history parameter most recent value
        self.parameters_positive_row = deque()
        self.parameters_negative_row = deque()
        self.model_parameters = deque()

        # weigths to add consistency to goal point
        self.weigths_slope = []
        self.weigths_intercept = []
        self.num_points_arrived = False

        # TO BE
        self.ransac = RANSACRegressor()

        # line_coefficient
        self.line_coefficient = [0,0]

        # previous_line 
        self.previous_line = [0.0, 0.0]
        self.first_iteration = False

        # to be defined by 
        self.fig, self.ax = plt.subplots()
  
    
    def scan_callback(self, scan_msg):
        start_time = time.perf_counter()
        # Transform laser scan msg of ROI points into cartesian coordinate
        points_2d = self.laser_scan_to_cartesian(scan_msg)
        # print("i'm in the callback")
        
        if(np.size(points_2d) < 15):
            # Used to prevent crash from no points detected in the region of interest
            print('No points found in ROI! ')
        else:
            # Use RANSAC
            self.RANSAC_with_MA(points_2d)
            # self.RANSAC(points_2d)
        end_time = time.perf_counter()
        execution_time = end_time - start_time
        avg_execution.append(execution_time)
        avg_execution_np = np.array(avg_execution)
        # print or file or so
        print("EXECUTION TIME: ",execution_time)
        print("AVG EXECUTION: ", np.mean(avg_execution_np), ", MIN VALUE: ", np.min(avg_execution_np), ", MAX VALUE: ", np.max(avg_execution_np))
        # Visualization using markers on Rviz
        # self.visualize_marker(points_2d[medoids[0]], points_2d[medoids[1]])
        # self.visualize_clusters(points_2d[clusters[0]], points_2d[clusters[1]])

    def calculate_weigths_slope(self, num_points):
        values = np.arange(start_slope,stop_slope, step=(stop_slope-start_slope)/num_points)
        
        # linear -> values_function = [value for value in values]
        # exp
        # values_function = [np.exp(value) for value in values]
        # exp 10
        # values_function = [pow(base_exp,value) for value in values]
        # all same weigth --> values_function = [1 for _ in values]
        values_function = [value for value in values]
        values_sum = np.sum(values_function)
        normalized_weigths = [value/values_sum for value in values_function]
        normalized_weigths.reverse()
        return normalized_weigths

    def calculate_weigths_intercept(self, num_points):
        values = np.arange(start_intercept,stop_intercept, step=(stop_intercept-start_intercept)/num_points)
        # print(values)
        # linear -> values_function = [value for value in values]
        # exp
        # values_function = [np.exp(value) for value in values]
        # exp 10
        values_function = [pow(base_exp,value) for value in values]
        values_sum = np.sum(values_function)
        normalized_weigths = [value/values_sum for value in values_function]
        normalized_weigths.reverse()
        # print(normalized_weigths, np.sum(normalized_weigths))
        return normalized_weigths

    def calculate_MA_slope_intercept(self,lines):
        # lines[0][0], lines [1][0] -> positive row
        # lines[0][1], lines [1][1] -> negative row
        # intercept = lines[1]
        # lope = lines[0]

        # calculate current bisectrice
        medium_slope = (lines[0][0]+lines[1][0]) / 2
        # set intercept = 0
        # medium_intercept = (intercept[0]+intercept[1]) / 2
        medium_intercept = 0

        # append value in array
        self.model_parameters.appendleft([medium_slope, medium_intercept])

        # queue dimension
        dim_self_param = len(self.model_parameters)

        # not reach maximum queue dimension
        # print("DIM MODEL PARAM ",dim_self_param)
        if(dim_self_param <= number_points):
            self.weigths_intercept = self.calculate_weigths_intercept(dim_self_param)
            self.weigths_slope = self.calculate_weigths_slope(dim_self_param)
        else:
            # remove last item, add current first element of queue
            self.model_parameters.pop()
            # one shot calculus
            if self.num_points_arrived == False:
                self.weigths_intercept = self.calculate_weigths_intercept(number_points)
                self.weigths_slope = self.calculate_weigths_slope(number_points)
        
        
        # test 1
        # print("INTERCEPT ", self.weigths_intercept, "SLOPE ", self.weigths_slope)
        # re calculate dim_self_param
        dim_self_param = len(self.model_parameters)
        # append new parameter as first item
        # calculate weigthed output slope
        output_slope = 0
        output_slope = [output_slope + self.weigths_slope[i]*self.model_parameters[i][0] for i in range(dim_self_param)]
        
        # calculate weighted intercept
        output_intercept = [0]
        # output_intercept = [output_intercept + self.weigths_intercept[i]*self.model_parameters[i][1] for i in range(dim_self_param)]
        # print("MODEL: ",self.model_parameters)
        output_line = [output_slope[0], output_intercept[0]]
        
        # output_line = [output_slope[0],0]
        # sobstitute the medium with avg 
        self.model_parameters.popleft()
        self.model_parameters.appendleft(output_line)
        # print("MODEL: ",self.model_parameters)

        return output_line
 
    # tmp1 save 
    def RANSAC_with_MA(self, points):
        # contains intercept + slope two interpolated lines
        crop_lines = []
        
        row_positive_value = points[np.where(points[:, 1] > self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1])]
        row_negative_value = points[np.where(points[:, 1] < self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1])]

        if (len(row_positive_value)>=1):
            self.ransac.fit(row_positive_value[:, 0].reshape(-1, 1), row_positive_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            crop_lines.append([slope, intercept])

        if (len(row_negative_value)>=1):
            self.ransac.fit(row_negative_value[:, 0].reshape(-1, 1), row_negative_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            crop_lines.append([slope, intercept])

        if(len(row_negative_value)>=RANSAC_num_point and len(row_positive_value)>=RANSAC_num_point):
            # print("CROP_LINES ", crop_lines)
            robot_line = self.calculate_MA_slope_intercept(crop_lines)
            # visualization
            self.visualize_ransac_MA(points, crop_lines, robot_line)
    
            # update boundaries
            self.line_coefficient = robot_line
        else:
            self.line_coefficient = [0, 0]
  
    # tmp1 save 
    def RANSAC(self, points):
        lines = []
        row_positive_value = points[np.where(points[:, 1] < 0)]
        row_negative_value = points[np.where(points[:, 1] > 0)]

        if (len(row_positive_value) >= 1): 
            self.ransac.fit(row_positive_value[:, 0].reshape(-1, 1), row_positive_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            lines.append((slope, intercept))

        if (len(row_negative_value) >= 1):
            self.ransac.fit(row_negative_value[:, 0].reshape(-1, 1), row_negative_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            lines.append((slope, intercept))

        # to do --> moving avarage exponential, linear --> toward data science
        if (len(row_positive_value) >= 1 and len(row_negative_value) >= 1):
            medium_slope = (lines[0][0]+lines[1][0]) / 2
            medium_intercept = 0 # (lines[0][1]+lines[1][1]) / 2
            lines.append((medium_slope, medium_intercept)) 
            self.visualize_ransac(points, lines)

    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=msg.angle_increment) 

        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))
        
        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]
    
        return points_filtered

    def visualize_marker(self, point1, point2):

        marker_msgs = MarkerArray()

        marker1 = Marker()
        marker1.id = 1
        marker1.header.frame_id = "laser_frame"
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD
        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05
        marker1.color.a = 1.0
        marker1.color.r = 0.0
        marker1.color.g = 0.0
        marker1.color.b = 1.0
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = point1[0]
        marker1.pose.position.y = point1[1]
        marker1.pose.position.z = 0.0
        marker_msgs.markers.append(marker1)

        marker2 = Marker()
        marker2.id = 2
        marker2.header.frame_id = "laser_frame"
        marker2.type = marker2.SPHERE
        marker2.action = marker2.ADD
        marker2.scale.x = 0.05
        marker2.scale.y = 0.05
        marker2.scale.z = 0.05
        marker2.color.a = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = point2[0]
        marker2.pose.position.y = point2[1]
        marker2.pose.position.z = 0.0
        marker_msgs.markers.append(marker2)
        
        self.mark_pub.publish(marker_msgs)
    
    def visualize_marker_points(self, points_2d):
    
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
        marker_msgs.markers = []

    def visualize_clusters(self, point_cluster1, point_cluster2):
        
        marker_msgs = MarkerArray()
        for i in range(len(point_cluster1)):
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
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point_cluster1[i, 0]
            marker.pose.position.y = point_cluster1[i, 1]
            marker.pose.position.z = 0.0
            marker_msgs.markers.append(marker)

        self.cluster1_pub.publish(marker_msgs)
    
        marker_msgs2 = MarkerArray()
        for i in range(len(point_cluster2)):
            marker2 = Marker()
            marker2.id = i
            marker2.header.frame_id = "laser_frame"
            marker2.type = marker.SPHERE
            marker2.action = marker.ADD
            marker2.scale.x = 0.05
            marker2.scale.y = 0.05
            marker2.scale.z = 0.05
            marker2.color.a = 1.0
            marker2.color.r = 0.0
            marker2.color.g = 1.0
            marker2.color.b = 0.0
            marker2.pose.orientation.w = 1.0
            marker2.pose.position.x = point_cluster2[i, 0]
            marker2.pose.position.y = point_cluster2[i, 1]
            marker2.pose.position.z = 0.0
            marker_msgs2.markers.append(marker2)

        self.cluster2_pub.publish(marker_msgs2)
    # re-analyzed
    def visualize_ransac_MA(self, points, crop_lines, robot_lines):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(points[:, 0], points[:, 1], color='blue')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)
        
        # for each line
        for line in crop_lines:
            # creates line
            y = line[0] * x + line[1]
            # plot line
            # print("LINE", line, "CROP",  crop_lines)
            plt.plot(x, y, color='red')

        # robot line
        y = robot_lines[0] * x + robot_lines[1]
        print("ROBOT ", robot_lines,"X", x)
        plt.plot(x, y, color='green')

        plt.xlim(0,3)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)

    def visualize_ransac(self, points, lines):
        self.ax.clear()
        plt.scatter(points[:, 0], points[:, 1], color='blue')
        # i=0
        for line in lines:
            # print(i, line, lines)
            # i=i+1
            x = np.linspace(0, 2, 10)
            y = line[0] * x + line[1]
            # print(x, y, line[0], line[1])
            plt.plot(x, y, color='red')
        plt.xlim(0,2)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)

    def goal_point_cluster(self, clusters):
        # Find medium point of cluster1 
        midpoint1 = np.array([np.mean(clusters[0][:, 0]), np.mean(clusters[0][:,1])])

        # Find medium point of cluster2 
        midpoint2 = np.array([np.mean(clusters[1][:, 0]), np.mean(clusters[1][:,1])])

        points = np.array((midpoint1, midpoint2))

        # Find goal point 
        x = np.mean(points[:, 0])
        y = np.mean(points[:, 1])
        goal = np.array((x,y))

        return goal
    
  

        
def main(args=None):
    rclpy.init(args=args)

    navigation = NavigationRansac()

    rclpy.spin(navigation)

    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    