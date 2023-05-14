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
        # get data from bisectrice
        self.scan_sub_goal_position = self.create_subscription(Float64MultiArray, '/bisectrice_coefficient', self.callback_line_coefficient, 1)
        # get data from laserscan
        self.scan_sub_end_of_line = self.create_subscription(LaserScan, '/scan/filtered_end_of_line', self.scan_callback_ROI, 1)
        # store goal position in queue
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_position',self.callback_update_goal_position, 1)
        
        # publish once end of line pose
        self.end_of_line_pose_topic_pub = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)
        
        # min points
        self.min_number_points = 0

        # divide into three regions
        self.area = np.array([1.2, 0.5]) # rect shape x,y

        # needed trigger turning
        # subscribe topic end of turning to understand publish/not
        self.sub_turning_status = self.create_subscription(Bool, '/end_of_turning', self.callback_update_bool, 1)
        # publish topic end of turning after first publication
        # self.pub_turning_status = self.create_publisher(Bool, '/end_of_turning', 1)
        # initialize value. Local variable.
        # test -> trigger only first time
        self.first_in_row_navigation = False
        self.publish_goal_position = False
        self.distance_goal_position = 0.75

    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/720) 

        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))
        
        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]
    
        return points_filtered

    # returns the three regions
    def split_points_into_regions(self, points):
        # print(points)
        # area width
        width = self.area[1]/6
        # left
        x_mask_left = (points[:, 0] >= 0) & (points[:, 0] <= self.area[1]) # return a boolean array, after AND 
        y_mask_left = (points[:, 1] >= -width*3) & (points[:, 1] < -width)#self.area[1])
        # middle
        x_mask_middle = (points[:, 0] >= 0) & (points[:, 0] <= self.area[1]) # return a boolean array, after AND 
        y_mask_middle = (points[:, 1] >= -width) & (points[:, 1] < width)#self.area[1])
        # rigth
        x_mask_rigth = (points[:, 0] >= 0) & (points[:, 0] <= self.area[1]) # return a boolean array, after AND 
        y_mask_rigth = (points[:, 1] >= width) & (points[:, 1] <= width*3)#self.area[1])


        # apply mask
        # range_mask = points[:,2] >=0
        mask_left = x_mask_left & y_mask_left # & range_mask
        mask_middle = x_mask_middle & y_mask_middle # & range_mask
        mask_rigth = x_mask_rigth & y_mask_rigth # & range_mask

        # remove empty points
        masked_points_left = np.full_like(points, -1)
        masked_points_left[mask_left] = points[mask_left]
        masked_points_middle = np.full_like(points, -1)
        masked_points_middle[mask_middle] = points[mask_middle]
        masked_points_rigth = np.full_like(points, -1)
        masked_points_rigth[mask_rigth] = points[mask_rigth]

        return masked_points_left, masked_points_middle, masked_points_rigth
    
    def calculate_goal_point(self):
        # takes the last m/q value of bisectrice
        recent_coefficient_value = self.queue_coefficient.return_element_time_t()
        if(recent_coefficient_value != False):
            slope, intercept = recent_coefficient_value[0], recent_coefficient_value[1]
            # calculate goal_points
            tmp_result = pow(slope,2)* pow(intercept,2) - (self.distance_goal_position+ pow(slope,2))*(pow(intercept,2)-1)
        
            # safety if-else statement
            if(tmp_result >=0):
                x_1 = (-slope*intercept + math.sqrt(tmp_result))/(self.distance_goal_position+slope**2)
                x_2 = (-slope*intercept - math.sqrt(tmp_result))/(self.distance_goal_position+slope**2)
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
        
        else:
            return False, False, False
    
    # understand empty region
    def scan_callback_ROI(self, scan_msg):
        self.start_computation = time.perf_counter()
        points_2d = self.laser_scan_to_cartesian(scan_msg)
        # divide the region in 3 spaces: middle, left, right
        mask_left, mask_middle, mask_rigth = self.split_points_into_regions(points_2d)
        # not have enough points and flag = true -> publish
        if self.publish_goal_position == True:
            if len(mask_left)<= self.min_number_points and len(mask_rigth)<= self.min_number_points : 
                # can publish
                # calculate goal point
                x, y, theta = self.calculate_goal_point()
                # print(x)
                # valuable data
                if x != False:
                    # TODO verify if has greater coordinates last goal
                    # print("INSIDE")
                    self.publish_end_of_line_pose(x, y , theta)
                    self.update_turning_status_after_pose_publication()
                # publish last pose, grater most recent pose
                # set variable to false
            # else:
                # print("ROI NOT EMPTY")
        else:
            if(self.first_in_row_navigation == True):
                self.publish_goal_position = True
                self.first_in_row_navigation = False
            # print("ROI HAS POINTS")
        # understand empty/not
        # all empty publish the end_of_line_pose
        # two ways: with/without goal pose
        # TODO how take perpendicular pose line
    
    # update bool value
    def callback_update_bool(self, msg):
        self.publish_goal_position = msg.data

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


