import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import time
import numpy as np
import math
import matplotlib.pyplot as plt
import sys
sys.path.append("/home/ceru/robotics/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/")
import prediction

class InRowNavigation(Node):
    # assume type are exponentials
    def __init__(self, dimension_queue=5, ma_crop_flag=0, ma_navigation_flag=1, num_skip = 5,threshold_crop_line = 0.5, threshold_bisectrice_line = 0.5, min_num_required_points=15) :
        super().__init__('in_row_navigation')
        # topics where the data are published
        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.goal_pose_pub = self.create_publisher(Float64MultiArray, '/goal_position', 1)
        self.goal_pose_pub # prevent unused variable warning
        self.end_of_line_pose_topic = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)
        self.end_of_line_pose_topic # prevent unused variable warning

        # HELPED
        # publish most recent pose and oldest pose
        # TODO
        self.collision_detected_topic = self.create_publisher(PoseStamped, '/collision_detected_topic', 1)
        
        # minimum number points
        self.min_num_required_points = min_num_required_points

        # ?
        self.end_of_line_flag = False
        self.gap_left = False
        self.gap_right = False

        # parameters needed during perfomance calculation
        self.end_computation = -1
        self.start_computation = -1

        # prefiltering
        self.prefiltering_threshold = 0.75 

        # Prediction obj
        # add parameters
        self.prediction_instance = prediction.Prediction()

        # Display Ransac
        self.fig, self.ax = plt.subplots()

        # num_accepted_points
        self.counter_start_line = 0
        self.window_skipped_start_line = num_skip

        # needed for goal publication
        self.is_end_of_line = False
        self.first_time_ROI = True


    def scan_callback(self, scan_msg):
        self.start_computation = time.perf_counter()
        points_2d = self.laser_scan_to_cartesian(scan_msg)

        # TODO riflettere altro nodo
        # collision detected

        if(np.size(points_2d) < self.min_num_required_points):
            print('No points found in ROI! ')
            # assume roi = 0 reset counter start line
            self.counter_start_line = 0
            # parameter read from parameter server. TODO.
            # print(self.is_end_of_line, self.first_time_ROI)
            if(self.is_end_of_line == True):
                if (self.first_time_ROI == True):
                    # publish last goal pose
                    x, y, theta = self.calculate_goal_point()
                    # invoke publish_end_of_line
                    self.publish_end_of_line_pose(x, y, theta)
                    # reset first_time_roi
                    self.first_time_ROI = False
            else: 
                self.is_end_of_line = False
            
            # initialize_prediction
            self.prediction_instance.initialize_prediction()
            
        else:
            # reset first_time_ROI, is_end_of_line
            self.first_time_ROI = True
            self.is_end_of_line = True

            # invoke prefiltering
            row_positive_value, row_negative_value = self.prefilter_point_far_from_bisectrice(points_2d)
            
            # skip first measurements 
            # if (self.counter_start_line < self.window_skipped_start_line):
                # start_line = True
                # self.counter_start_line = self.counter_start_line + 1
            # else:
                # start_line = False
            
            # bisectrice prediction
            self.prediction_instance.compute_bisectrice_coefficients(row_positive_value, row_negative_value)

            # invoke calculate_goal_position
            x, y, theta = self.calculate_goal_point()

            # publish goal pose
            self.publish_goal_pose(x, y, theta)

            # display 
            self.display_prediction(row_positive_value, row_negative_value, x, y)


    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=msg.angle_increment) 

        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))
        
        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]
    
        return points_filtered
    

    def prefilter_point_far_from_bisectrice(self, points):
        # prefilter positive and negative values
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # print(slope, intercept, points)
        # y = self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]
        mask_pos = ((points[:, 1] > slope*points[:, 0]+ intercept) & (points[:, 1] < ((slope*points[:, 0]+ intercept) + self.prefiltering_threshold)))
        mask_neg = ((points[:, 1] < slope*points[:, 0]+ intercept) & (points[:, 1] > ((slope*points[:, 0]+ intercept) - self.prefiltering_threshold)))
        # print(mask_pos, mask_neg)

        return points[mask_pos], points[mask_neg]
    
    # def check_collision(self):
        # check if nearest points refilteres are inside a detection threshold
        # in case yes publish data
        

    def calculate_goal_point(self):
        # takes the last m/q value of bisectrice
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
        
        # calculate goal_points
        tmp_result = pow(slope,2)* pow(intercept,2) - (1+ pow(slope,2))*(pow(intercept,2)-1)
        
        # safety if-else statement
        if(tmp_result >=0):
            x_1 = (-slope*intercept + math.sqrt(tmp_result))/(1+slope**2)
            x_2 = (-slope*intercept - math.sqrt(tmp_result))/(1+slope**2)
        else:
            x_1 = 0
            x_2 = 0
            print("ERROR QUADRATIC")

        # take greatest value
        if x_1 >= x_2:
            x = x_1
        else:
            x = x_2
        
        # solve equation
        y = slope*x + intercept

        # take euler angle
        theta = math.atan(y/x)


        return x,y,theta
    
    def publish_goal_pose(self, x, y, theta):
        # create message Float64MultiArray
        goal_pose = Float64MultiArray()
        # update content
        goal_pose.data = [x, y, theta]
        # publish goal pose
        self.goal_pose_pub.publish(goal_pose)

    def publish_end_of_line_pose(self, x, y, theta):
        # create message Pose
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "map"

        # get x,y
        end_of_line_pose.pose.position.x = x
        end_of_line_pose.pose.position.y = y

        # get orientation
        quaternion = quaternion_from_euler(0, 0, theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        if(end_of_line_pose.pose.position.x == 1) and (end_of_line_pose.pose.orientation.w == 1):
            return
        else:
            # publish goal pose
            self.end_of_line_pose_topic.publish(end_of_line_pose)
        
    def display_prediction(self, row_positive_value, row_negative_value, x_goal, y_goal):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(row_positive_value[:, 0], row_positive_value[:, 1], color='blue')
        plt.scatter(row_negative_value[:, 0], row_negative_value[:, 1], color='blue')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # get proper slope, intercept for each value
        positive_slope, positive_intercept = self.prediction_instance.positive_crop_line.get_most_recent_coefficients()
        negative_slope, negative_intercept = self.prediction_instance.negative_crop_line.get_most_recent_coefficients()
        bisectrice_slope, bisectrice_intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # creates line
        y_pos = positive_slope * x + positive_intercept
        y_neg = negative_slope * x + negative_intercept

        # plot line
        plt.plot(x, y_pos, color='red')
        plt.plot(x, y_neg, color='red')

        # robot line
        y = bisectrice_slope * x + bisectrice_intercept

        # bisectrice
        plt.plot(x, y, color='green')

        # goal point 
        plt.plot(x_goal, y_goal, marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")

        plt.xlim(0,3)
        plt.ylim(-2,2)

        self.fig.canvas.draw()
        plt.pause(0.01)


    # def measure_performance(self):
    
    # def __repr__ (self):
        # return 'Line(slope=' + self.slope.__repr__() + ' ,intercept=' + self.intercept.__repr__()  + ')'
    
     
def main(args=None):
    rclpy.init(args=args)

    in_row_navigation = InRowNavigation()

    rclpy.spin(in_row_navigation)

    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


