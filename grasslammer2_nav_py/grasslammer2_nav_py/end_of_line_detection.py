import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import time
import numpy as np
from collections import deque
import math


#########################################
#################### COSTUM QUEUE OBJECT
#########################################
class CustomQueue:
    def __init__(self, dimension_queue):
        self.dimension_queue = dimension_queue
        self.queue = deque()


    # update queue regularly
    def update_queue(self, value):
        # print("update_queue_regularly ", len(self.queue),self.dimension_queue)
        if(len(self.queue) >= self.dimension_queue):
            self.queue.pop()
        self.queue.appendleft(value)
          
        
    # needed to compute validity data
    def return_element_time_t(self):
        if(len(self.queue) > 0):
            return self.queue[0]
        else:
            return False
    

    # needed to compute goal point position data
    def return_oldest_element(self):
        if(len(self.queue) > 0):
            return self.queue[-1]
        else:
            return False

    # initialize queue
    def initialize_queue(self):
        self.queue = deque()
    

###########################################
#################### END OF LINE DETECTION
###########################################
class EndOfLineDetection(Node):
    # assume type are exponentials
    def __init__(self) :
        super().__init__('end_of_line_detection')
        # my line is [m, q]
        self.queue_coefficient = CustomQueue(5)
        # goal position elements
        self.queue_goal_position = CustomQueue(5)
        # get data from laserscan
        self.scan_sub_end_of_line = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback_ROI, 1)
        
        # publish once end of line pose
        self.end_of_line_pose_topic_pub = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)

        # subscribe topic end of turning to understand publish/not
        self.sub_turning_status = self.create_subscription(Bool, '/end_of_turning', self.callback_update_bool, 1)
        

        # divide into three regions
        self.area = np.array([2, 2]) # rect shape x,y
        # needed trigger turning
        # publish topic end of turning after first publication
        # self.pub_turning_status = self.create_publisher(Bool, '/end_of_turning', 1)
        # initialize value. Local variable.
        # test -> trigger only first time
        self.first_in_row_navigation = False
        self.publish_goal_position = True
        self.distance_goal_position = 0.05

    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) 

        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))

        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]

       
        # if number of points 
        if(np.size(points_filtered) < self.min_num_build_quadrants):
            x_nord = np.where(((0 <= x) & (x <= self.nord_treshold)),x, -1)
            x_south = np.where(((-self.south_treshold <= x)&(x <= 0)), x, -1)

            y_west = np.where(((0 <= y)&(y<= self.area[0]/2)),y, -1)
            y_east = np.where(((-self.area[0]/2 <= y)&(y <= 0)),y, -1)

            points_nord_east = np.vstack((x_nord, y_east)).T
            points_nord_west = np.vstack((x_nord, y_west)).T
            points_south_east = np.vstack((x_south, y_east)).T
            points_south_west = np.vstack((x_south, y_west)).T

            points_nord_east = points_nord_east[y_east != -1]
            points_nord_west = points_nord_west[y_west != -1]
            points_south_east = points_south_east[y_east != -1]
            points_south_west = points_south_west[y_west != -1]

        else:
            
            slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

            if(slope==0 and intercept==0):
                x_nord = np.where(((0 <= x) & (x <= self.nord_treshold)),x, -1)
                x_south = np.where(((-self.south_treshold <= x)&(x <= 0)), x, -1)
                y_west = np.where(((0 <= y)&(y<= self.line_width)),y, -1)
                y_east = np.where(((-self.line_width <= y)&(y <= 0)),y, -1)

                points_nord_east = np.vstack((x_nord, y_east)).T
                points_nord_west = np.vstack((x_nord, y_west)).T
                points_south_east = np.vstack((x_south, y_east)).T
                points_south_west = np.vstack((x_south, y_west)).T

                points_nord_east = points_nord_east[y_east != -1]
                points_nord_west = points_nord_west[y_west != -1]
                points_south_east = points_south_east[y_east != -1]
                points_south_west = points_south_west[y_west != -1]

            else:
                # distance_from_goal +/- distance_from_bisectrice
                # self.distance_from_bisectrice = 0.3
                # self.tolerance_crop_distance = 0.5
                # take x point and make division east/west

                distance_east_above = intercept - self.distance_from_bisectrice + self.tolerance_crop_distance
                distance_east_below = intercept - self.distance_from_bisectrice - self.tolerance_crop_distance
                distance_west_above = intercept + self.distance_from_bisectrice + self.tolerance_crop_distance
                distance_west_below = intercept + self.distance_from_bisectrice - self.tolerance_crop_distance
                
                threshold_east_above = np.full_like(y, distance_east_above)
                threshold_east_below = np.full_like(y, distance_east_below)
                threshold_west_above = np.full_like(y, distance_west_above)
                threshold_west_below = np.full_like(y, distance_west_below)

                # print(np.size(threshold_west_above), np.size(threshold_west_below))

                # y east -> <=0
                # y_west -> >0

                y_west = np.where((y > np.add(slope*x, threshold_west_below))&(y < np.add(slope*x, threshold_west_above)),y, -1)
                y_east = np.where((y > np.add(slope*x, threshold_east_below))&(y < np.add(slope*x,threshold_east_above)),y, -1)
                
                # print(y_south_east, y_nord_east)
                # print(mask_nord_west, mask_nord_east)
                # mask_nord_east = ((point_nord_east[:, 1] < slope*point_nord_east[:, 0]+ intercept) & (point_nord_east[:, 1] > ((slope*point_nord_east[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_nord_west = ((point_nord_west[:, 1] > slope*point_nord_west[:, 0]+ intercept) & (point_nord_west[:, 1] < ((slope*point_nord_west[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_south_east = ((point_south_east[:, 1] < slope*point_south_east[:, 0]+ intercept) & (point_south_east[:, 1] > ((slope*point_south_east[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_south_west = ((point_south_west[:, 1] > slope*point_south_west[:, 0]+ intercept) & (point_south_west[:, 1] < ((slope*point_south_west[:, 0]+ intercept) + self.prefiltering_threshold)))
                
                threshold_nord = np.full_like(x, intercept*slope + self.nord_treshold)
                threshold_south = np.full_like(x, intercept*slope - self.south_treshold)

                m_q = np.full_like(x, slope*intercept)

                # x = -m*y + m*q
                x_nord_east = np.where((x > np.add(-slope*y_east, m_q))&(x < np.add(-slope*y_east, threshold_nord)),x, -1)
                x_nord_west = np.where((x > np.add(-slope*y_west, m_q))&(x < np.add(-slope*y_west,threshold_nord)),x, -1)
                x_south_east = np.where((x < np.add(-slope*y_east, m_q))&(x > np.add(-slope*y_east, threshold_south)),x, -1)
                x_south_west = np.where((x < np.add(-slope*y_west, m_q))&(x > np.add(-slope*y_west,threshold_south)),x, -1)
                    
                points_nord_east = np.vstack((x_nord_east, y_east)).T
                points_nord_west = np.vstack((x_nord_west, y_west)).T
                points_south_east = np.vstack((x_south_east, y_east)).T
                points_south_west = np.vstack((x_south_west, y_west)).T

                points_nord_east = points_nord_east[y_east != -1]
                points_nord_west = points_nord_west[y_west != -1]
                points_south_east = points_south_east[y_east != -1]
                points_south_west = points_south_west[y_west != -1]
                
            
        return points_nord_east, points_nord_west, points_south_east, points_south_west

    
    # understand empty region
    def scan_callback_ROI(self, scan_msg):
        self.start_computation = time.perf_counter()
        points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian(scan_msg)
        
    
    # update bool value
    def callback_update_bool(self, msg):
        self.publish_goal_position = True

    # update m,q
    def callback_line_coefficient(self, msg):
        self.queue_coefficient.update_queue(msg.data)

    # update goal pose 
    def callback_update_goal_position(self, msg):
        self.queue_goal_position.update_queue(msg.data)

    # publish end of line pose
    def publish_end_of_line_pose(self, x, y, theta):
        # create message Pose
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "base_footprint"

        # get x,y
        end_of_line_pose.pose.position.x = x
        end_of_line_pose.pose.position.y = y

        # get orientation
        quaternion = quaternion_from_euler(0, 0, theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        self.end_of_line_pose_topic_pub.publish(end_of_line_pose)

    # update turning status: make switch work
    def update_turning_status_after_pose_publication(self):
        self.publish_goal_position = False
        # msg = Bool()
        # msg.data = self.publish_goal_position
        # self.pub_turning_status(msg)

    # initialize queue
    def initialize_queue(self):
        self.queue_coefficient.initialize_queue()
        self.queue_goal_position.initialize_queue()



def main(args=None):
    rclpy.init(args=args)

    end_of_line_detection = EndOfLineDetection()

    rclpy.spin(end_of_line_detection)

    end_of_line_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


