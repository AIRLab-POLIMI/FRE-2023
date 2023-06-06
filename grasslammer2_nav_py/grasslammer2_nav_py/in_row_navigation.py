import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray,Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_geometry_msgs import do_transform_pose_stamped


import time
import numpy as np
import math
import matplotlib.pyplot as plt
import numpy.ma as ma
import os

# QUEUE
from collections import deque
# LINE
import math
import matplotlib.pyplot as plt
import numpy.ma as ma
import os

# QUEUE
from collections import deque
# LINE
from sklearn.linear_model import RANSACRegressor
from numpy import float64
import json
# modified robots
# string_from_folder = 'ros2_humble/src/FRE-2023'
#absolute_path = os.path.realpath(string_from_folder+'/grasslammer2_nav_py/grasslammer2_nav_py/in_row_navigation_config/cornaredo.json')
# absolute_path = '/home/ceru/robotics/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/in_row_navigation_config/cornaredo.json'
absolute_path = '/home/alba/ros2_ws/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/in_row_navigation_config/cornaredo.json'

print(absolute_path)
config_file = open(absolute_path, 'r')
# dict_config = config_file.read()
config_json = json.loads(config_file.read())

#################################
########## MOVING AVARAGE CLASS
##################################

class MovingAvarage():
    # assume type are exponentials
    def __init__(self):
        self.weigths = []
        # self.start = start
        # self.stop = stop
        self.base_exp, self.start, self.stop = self.get_parameters_from_config_file()
        # print(self.base_exp, self.start, self.stop)

    def update_weights(self, length_queue, dimension_queue):
        # update weigths of moving avarage only when wwights < num_points 
        # print("update_weights ", length_queue, dimension_queue)
        if (length_queue <= dimension_queue):
            values = np.arange(self.start,self.stop, step=(self.stop-self.start)/length_queue)
            # linear -> values_function = [value for value in values]
            # exp
            # values_function = [np.exp(value) for value in values]
            # exp 10
            # values_function = [pow(base_exp,value) for value in values]
            # all same weigth --> values_function = [1 for _ in values]
            values_function = [pow(self.base_exp,value) for value in values]
            values_sum = np.sum(values_function)
            self.weigths = [value/values_sum for value in values_function]
            self.weigths.reverse()
        # else remains same
        return self.weigths
    

    def calculate_MA(self, coefficients):
        if(len(self.weigths) != len(coefficients)):
            print("calculate_MA: ERROR MA computation weights, queue", len(self.weigths), len(coefficients))
        else:
            output_MA = 0
            output_MA = [output_MA + self.weigths[i]*coefficients[i] for i in range(len(coefficients))]
            return output_MA[0]
    

    def get_weigths(self):
        return self.weigths
    

    def initialize_weigths(self):
        self.weights = []
    
    def get_parameters_from_config_file(self):
        base_exp = config_json['moving_avarage']['base_exp']
        start = config_json['moving_avarage']['start']
        stop = config_json['moving_avarage']['stop']
        return base_exp, start, stop


#################################
########## MOVING AVARAGE QUEUE
##################################

class MovingAvarageQueue():
    # assume type are exponentials
    def __init__(self, dimension_queue,reset_value):
        self.dimension_queue = dimension_queue
        self.queue = deque()
        self.weights = MovingAvarage()
        self.reset_value = reset_value
        
    # update queue regularly
    def update_queue_regularly(self, value):
        # print("update_queue_regularly ", len(self.queue),self.dimension_queue)
        if(len(self.queue) >= self.dimension_queue):
            self.queue.pop()
        
        self.queue.appendleft(value)
        self.weights.update_weights(len(self.queue), self.dimension_queue)

        # calculate the MA
        MA = self.weights.calculate_MA(self.queue)

        # add MA as most recent element
        self.queue.popleft()
        self.queue.appendleft(MA)
            
    # updated queue
    def update_queue(self, value):
        if(len(self.queue) == 0):
            self.queue.appendleft(self.reset_value)
        else:
            self.update_queue_regularly(value)
          
        
    # needed to compute validity data
    def return_element_time_t(self):
        if(len(self.queue) > 0):
            return self.queue[0]
        else:
            return self.reset_value
    

    # needed to compute goal point position data
    def return_oldest_element(self):
        if(len(self.queue) > 0):
            return self.queue[-1]
        else:
            return self.reset_value
    

    # initialize queue
    def initialize_queue(self):
        self.queue = deque()
        self.weights.initialize_weigths()
        self.queue.appendleft(0)
        self.weights.update_weights(len(self.queue), self.dimension_queue)

    def get_reset_value(self):
        return self.reset_value
    
    

#################################
#################### COSTUM QUEUE
##################################
class CustomQueue:
    def __init__(self, dimension_queue, reset_value):
        self.dimension_queue = dimension_queue
        self.queue = deque()
        self.reset_value = reset_value


    # update queue regularly
    def update_queue_regularly(self, value):
        # print("update_queue_regularly ", len(self.queue),self.dimension_queue)
        if(len(self.queue) >= self.dimension_queue):
            self.queue.pop()
        self.queue.appendleft(value)
            
    # updated queue
    def update_queue(self, value):
        if(len(self.queue) == 0):
            self.queue.appendleft(self.reset_value)
        else:
            self.update_queue_regularly(value)
          
        
    # needed to compute validity data
    def return_element_time_t(self):
        if(len(self.queue) > 0):
            return self.queue[0]
        else:
            return self.reset_value
    

    # needed to compute goal point position data
    def return_oldest_element(self):
        if(len(self.queue) > 0):
            return self.queue[-1]
        else:
            return self.reset_value
    
    def get_reset_value(self):
        return self.reset_value


    # initialize queue
    def initialize_queue(self):
        self.queue = deque()
        self.queue.appendleft(self.reset_value)
    
    
#################################
#################### LINE
##################################    

class Line():
    # assume type are exponentials
    def __init__(self, quadrant):
        # take parameters from config file
        dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope = self.get_parameters_from_config_file(quadrant)
        
        # check ma flag
        if ma_avarage_flag == True:
            self.slope = MovingAvarageQueue(dimension_queue=dimension_queue,reset_value=reset_value_slope)
            self.intercept = MovingAvarageQueue(dimension_queue=dimension_queue,reset_value=reset_value_intercept)
        else:
            self.slope = CustomQueue(dimension_queue, reset_value=reset_value_slope)
            self.intercept = CustomQueue(dimension_queue, reset_value=reset_value_intercept)
        
        # if not bisectrice, ransac needed
        self.is_bisectrice = is_bisectrice
        if not is_bisectrice:
            # min_samples=20, 
            self.ransac = RANSACRegressor(random_state=42)
        
        # threshold allowed btw two consecutive lines 
        self.threshold_btw_consecutive_lines_intercept = threshold_line_intercept
        self.threshold_btw_consecutive_lines_slope = threshold_line_slope # 15 degree

        # counter outdated lines
        self.counter_outdated_consecutive_values = 0
        # consistency on filter in terms of number required points
        self.window_allowed_outdated_parameters = dimension_queue
        # previous status
        self.last_valid_slope = 0
        self.last_valid_intercept = 0

       
    # initialization of queues
    def initialize_line(self):
        self.slope.initialize_queue()
        self.intercept.initialize_queue()

    # update line coefficient
    # slope, intercept = False -> update parameters last value
    def update_line_parameters(self, slope, intercept):
        self.slope.update_queue(slope) 
        self.intercept.update_queue(intercept) 
        # save last valid status
        self.last_valid_slope = slope
        self.last_valid_intercept = intercept
       
    
    # update line coefficient
    # slope, intercept = False -> update parameters last value
    def update_line_parameters_no_threshold(self, slope, intercept):
        self.slope.update_queue(slope) 
        self.intercept.update_queue(intercept)
        
    
    # select line in case the slope,line contained in threshold. Otherwise, takes the previous value
    def update_line_parameters_checking_threshold(self, slope, intercept):
        # check slope/intercept t vs slope/intercept t-1
        slope_time_t_min_1 = self.slope.return_element_time_t()
        intercept_time_t_min_1 = self.intercept.return_element_time_t()
        # check existance of data at time t and t-1
        if(self.threshold_btw_consecutive_lines_intercept != 0):
            if (self.counter_outdated_consecutive_values <= self.window_allowed_outdated_parameters):
                if slope_time_t_min_1 != False and intercept_time_t_min_1 != False:
                    # same value remain same otherwise take opposite value
                    intercept_time_t_min_1 = intercept_time_t_min_1 if intercept_time_t_min_1*intercept > 0 else -intercept_time_t_min_1
                    # only intercept
                    if abs(intercept_time_t_min_1 - intercept) > self.threshold_btw_consecutive_lines_intercept:
                        # update_last_valid_coefficient
                        self.slope.update_queue(self.last_valid_slope) 
                        self.intercept.update_queue(self.last_valid_intercept) 
                        # update window
                        self.counter_outdated_consecutive_values = self.counter_outdated_consecutive_values +1
                    else:
                        # add just computed parameters
                        self.update_line_parameters(slope, intercept)
                        # reset value -> normal update
                        self.counter_outdated_consecutive_values = 0

                else:    
                    # slope, intercept queue was empty 
                    self.update_line_parameters(slope, intercept)
                    # reset value -> normal update
                    self.counter_outdated_consecutive_values = 0
            else:
                # too outdated value -> add new one
                self.update_line_parameters(slope, intercept)
                # reset value -> normal update
                self.counter_outdated_consecutive_values = 0
        else:
            self.update_line_parameters_no_threshold(slope, intercept)

    
    def fitting_line(self, points):
        self.ransac.fit(points[:, 0].reshape(-1, 1), points[:, 1])
        slope = self.ransac.estimator_.coef_[0]
        intercept = self.ransac.estimator_.intercept_
        return slope, intercept
    

    def get_most_recent_coefficients(self):
        slope = float64(self.slope.return_element_time_t())
        intercept = float64(self.intercept.return_element_time_t())
        return slope, intercept
    
    
    def get_oldest_coefficients(self):
        slope = float64(self.slope.return_oldest_element())
        intercept =float64(self.intercept.return_oldest_element())
        return slope, intercept

    def get_parameters_from_config_file(self, quadrant):
        dimension_queue = config_json['prediction']['lines'][quadrant]['dimension_queue']
        ma_avarage_flag = config_json['prediction']['lines'][quadrant]['ma_avarage_flag']
        is_bisectrice = config_json['prediction']['lines'][quadrant]['is_bisectrice']
        threshold_line_intercept = config_json['prediction']['lines'][quadrant]['threshold_line_intercept']
        threshold_line_slope = config_json['prediction']['lines'][quadrant]['threshold_line_slope']
        reset_value_intercept = config_json['prediction']['lines'][quadrant]['reset_value_intercept']
        reset_value_slope = config_json['prediction']['lines'][quadrant]['reset_value_slope']
        return dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope
    
        
#################################
#################### PREDICTION
##################################  
class Prediction():
    # assume type are exponentials
    def __init__(self) :
        # dimension_queue -> 
        # nord east
        self.nord_east_line = Line('line_nord_east')
        self.nord_west_line = Line('line_nord_west')
        self.south_east_line = Line('line_south_east')
        self.south_west_line = Line('line_south_west')
        self.navigation_line =  Line('line_navigation')
        
        # min_point_for_predition + threshold
        self.num_min_points_required_for_fitting = 20
        self.line_with = 0.75
        self.tolerance_intercept = 0.2
        self.tolerance_slope = 0.17 # 10 degrees
    
    # initialization of lines
    def initialize_prediction(self):
        self.navigation_line.initialize_line()
        self.nord_east_line.initialize_line()
        self.nord_west_line.initialize_line() 
        self.south_east_line.initialize_line()
        self.south_west_line.initialize_line()

    def compute_bisectrice_coefficients_forward(self, nord_east_points, nord_west_points, south_east_points, south_west_points):
        nord_west_slope, nord_west_intercept,nord_east_slope, nord_east_intercept = self.calculate_crop_coefficients(self.nord_west_line, nord_west_points, self.nord_east_line, nord_east_points) 
        _,_,_,_ = self.calculate_crop_coefficients(self.south_west_line, south_west_points, self.south_west_line, south_east_points) 

        # compute bisectrice
        medium_slope = (nord_west_slope+nord_east_slope) / 2
        medium_intercept = (nord_west_intercept+nord_east_intercept) / 2

        # add coeddificent to bisectrice
        self.navigation_line.update_line_parameters(medium_slope, medium_intercept)

    def calculate_crop_coefficients(self,line_west, point_west, line_east, point_east):
        # print("num points", len(points))
        # print("num points", len(points))
        # add delta threshold
        if(len(point_west)> self.num_min_points_required_for_fitting) and (len(point_east)> self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope_west, intercept_west = line_west.fitting_line(point_west)
            # compute slope, intercept through fitting
            slope_east, intercept_east = line_east.fitting_line(point_east)

            slope_east = slope_east if slope_east*slope_west>0 else -slope_east
            intercept_east = intercept_east if intercept_east*intercept_west > 0 else -intercept_east
            # check if distanciated enough
            if(abs(intercept_west - intercept_east) < self.line_with - self.tolerance_intercept) and (abs(intercept_west - intercept_east) > self.line_with + self.tolerance_intercept):
                slope_west, intercept_west = line_west.get_most_recent_coefficients()
                slope_east, intercept_east = line_east.get_most_recent_coefficients()
            # check if parallel
            elif (abs(slope_west - slope_east) > self.tolerance_slope):
                slope_west, intercept_west = line_west.get_most_recent_coefficients()
                slope_east, intercept_east = line_east.get_most_recent_coefficients()
        elif (len(point_west) < self.num_min_points_required_for_fitting) and (len(point_east) > self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope_west, intercept_west = line_west.get_most_recent_coefficients()
            # compute slope, intercept through fitting
            slope_east, intercept_east = line_east.fitting_line(point_east)

            slope_east = slope_east if slope_east*slope_west>0 else -slope_east
            intercept_east = intercept_east if intercept_east*intercept_west > 0 else -intercept_east
            # check if distanciated enough
            if(abs(intercept_west - intercept_east) < self.line_with - self.tolerance_intercept) and (abs(intercept_west - intercept_east) > self.line_with + self.tolerance_intercept):
                slope_east, intercept_east = line_east.get_most_recent_coefficients()
            # check if parallel
            elif (abs(slope_west - slope_east) > self.tolerance_slope):
                slope_east, intercept_east = line_east.get_most_recent_coefficients()
        
        elif (len(point_west) > self.num_min_points_required_for_fitting) and (len(point_east) < self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope_west, intercept_west = line_west.fitting_line(point_west)
            # compute slope, intercept through fitting
            slope_east, intercept_east = line_east.get_most_recent_coefficients()

            slope_east = slope_east if slope_east*slope_west>0 else -slope_east
            intercept_east = intercept_east if intercept_east*intercept_west > 0 else -intercept_east
            # check if distanciated enough
            if(abs(intercept_west - intercept_east) < self.line_with - self.tolerance_intercept) and (abs(intercept_west - intercept_east) > self.line_with + self.tolerance_intercept):
                slope_west, intercept_west = line_west.get_most_recent_coefficients()
            # check if parallel
            elif (abs(slope_west - slope_east) > self.tolerance_slope):
                slope_west, intercept_west = line_west.get_most_recent_coefficients()
        else:
            slope_west, intercept_west = line_west.get_most_recent_coefficients()
            slope_east, intercept_east = line_east.get_most_recent_coefficients()
        
        return slope_west, intercept_west, slope_east, intercept_east
        

class InRowNavigation(Node):
    # assume type are exponentials
    def __init__(self, dimension_queue=5, ma_crop_flag=0, ma_navigation_flag=1, num_skip = 5,threshold_crop_line = 0.5, threshold_bisectrice_line = 0.5, min_num_required_points=20, num_points_traverse_line=30) :
        super().__init__('in_row_navigation')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # topics where the data are published
        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.goal_pose_pub = self.create_publisher(Float64MultiArray, '/goal_position', 1)
        self.goal_pose_pub # prevent unused variable warning
        self.end_of_line_pose_topic = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)
        self.end_of_line_pose_topic # prevent unused variable warning
        # topic to update the boolean variable to publish
        #self.sub_turning_status = self.create_subscription(Bool, '/end_of_turning', self.callback_update_bool, 1)

        self.area, dimension_queue, self.min_points, self.line_width, self.y_tolerance, self.distance_goal_point_forward, distance_goal_point_backward, nord_threshold, south_threshold = self.get_parameters_from_config_file()
        self.distance_from_bisectrice = self.line_width/2

        # min points required per direction
        self.min_point_direction = self.min_points * 2 -1

        # define thresholds
        self.nord_threshold = self.area[0]/2
        self.south_threshold = -self.area[0]/2
        self.west_threshold = self.line_width/2
        self.east_threshold = -self.line_width/2

        #define quadrants
        self.nord_east_quadrant = [[0,self.nord_threshold],[self.east_threshold,0]]
        self.nord_west_quadrant = [[0,self.nord_threshold],[0,self.west_threshold]]
        self.south_east_quadrant = [[self.south_threshold/2,0],[self.east_threshold,0]]
        self.south_west_quadrant = [[self.south_threshold/2,0],[0,self.west_threshold]]
        

        # modify is_begin/end line
        self.is_begin_line_forward = False
        self.is_end_line_backward = False

        # move forward
        self.moving_forward = True
  
    def get_parameters_from_config_file(self):
        area = config_json["in_row_navigation"]['area']
        dimension_queue = config_json["in_row_navigation"]['dimension_queue']
        min_number_required_points_per_quadrants = config_json["in_row_navigation"]['min_number_required_points_per_quadrants']
        line_width = config_json["in_row_navigation"]['line_width']
        tolerance_crop_distance_filtering = config_json["in_row_navigation"]['tolerance_crop_distance_filtering']
        distance_goal_point_forward = config_json["in_row_navigation"]['distance_goal_point_forward']
        distance_goal_point_backward = config_json["in_row_navigation"]['distance_goal_point_backward']
        nord_threshold = config_json["in_row_navigation"]['nord_threshold']
        south_threshold = config_json["in_row_navigation"]['south_threshold']
        return area, dimension_queue, min_number_required_points_per_quadrants, line_width, tolerance_crop_distance_filtering, distance_goal_point_forward, distance_goal_point_backward, nord_threshold, south_threshold
    
    def scan_callback(self, scan_msg):
        self.start_computation = time.perf_counter()
        # points_nord_east, points_nord_west, points_south_east, points_south_west
        points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian(scan_msg)

        # print(points_2d)
        # if going forward/backward 

        if(self.moving_forward):
            self.in_row_navigation_forward(points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west)
        else:
            self.in_row_navigation_backward(points_nord_east, points_nord_west, points_south_east, points_south_west)
           
    def in_row_navigation_forward(self, points_nord_east, points_nord_west, points_south_east, points_south_west):

        # get information about emptyness of regions
        is_nord_east_empty = True if np.size(points_nord_east) < self.min_num_required_points else False
        is_nord_west_empty = True if np.size(points_nord_west) < self.min_num_required_points else False
        is_south_east_empty = True if np.size(points_south_east) < self.min_num_required_points else False
        is_south_west_empty = True if np.size(points_south_west) < self.min_num_required_points else False

        print(len(points_nord_east), len(points_nord_west), len(points_south_east), len(points_south_west))
        # needed to save the last point to calculate the perpendicular angle
        #if is_nord_east_empty & is_nord_west_empty & (is_south_east_empty == False or is_south_west_empty == False):
        # IDEA -> anticipate it beforehand
        
        if (is_nord_east_empty == True & is_nord_west_empty == True) & (is_south_east_empty == False & is_south_west_empty        == False) & self.is_goal_published==True:
            tmp_south_east = points_south_east.max(axis=0)
            tmp_south_west = points_south_west.max(axis=0)
            # get the highest values
            if(tmp_south_east[0] > self.greatest_point_south_east_forward[0]):
                self.greatest_point_south_east_forward = tmp_south_east
            if(tmp_south_west[0] > self.greatest_point_south_west_forward[0]):
                self.greatest_point_south_west_forward = tmp_south_west
            print(self.greatest_point_south_east_forward, self.greatest_point_south_west_forward)

            # self.greatest_point_nord_east_backward = points_nord_east.max(axis=0)
            # self.greatest_point_nord_west_backward = points_nord_west.max(axis=0)
        
        if is_nord_east_empty & is_nord_west_empty & is_south_east_empty & is_south_west_empty:
            if (self.is_goal_published==True):
                # self.distance_goal_point_forward = 0.1
                # publish last goal pose    
                # x, y, theta = self.calculate_goal_point_forward()
                self.publish_end_of_line_pose_perpendicular_crop_forward()
                # reset_is_goal_published
                # self.distance_goal_point_forward = self.area[1]/2
                self.update_turning_status_after_pose_publication()
                self.prediction_instance.initialize_prediction()
                return

        # elif (is_nord_east_empty == False | is_nord_west_empty == False) & is_south_east_empty & is_south_west_empty:
                # self.is_begin_line_forward = True
                # self.is_end_line_backward = False
                # self.is_goal_published=True

        # compute bisectrice
        self.prediction_instance.compute_bisectrice_coefficients_forward(points_nord_east,points_nord_west,points_south_east,points_south_west)

        # invoke calculate_goal_position
        x, y, theta = self.calculate_goal_point_forward()

        # needed to store the last x,y
        # if is_nord_east_empty & is_nord_west_empty & (is_south_east_empty!= False or is_south_west_empty!=False):
            # self.previous_forward_goal = [x, y]

        # publish goal pose
        self.publish_goal_pose(x, y, theta)

        # display 
        self.display_prediction_forward(points_nord_east,points_nord_west, x, y)
    
    def in_row_navigation_backward(self, points_nord_east, points_nord_west, points_south_east, points_south_west):
        # get information about emptyness of regions
        is_nord_east_empty = True if np.size(points_nord_east) < self.min_num_required_points else False
        is_nord_west_empty = True if np.size(points_nord_west) < self.min_num_required_points else False
        is_south_east_empty = True if np.size(points_south_east) < self.min_num_required_points else False
        is_south_west_empty = True if np.size(points_south_west) < self.min_num_required_points else False

        print(len(points_nord_east), len(points_nord_west), len(points_south_east), len(points_south_west))
        # needed to save the last point to calculate the perpendicular angle
        #if is_nord_east_empty & is_nord_west_empty & (is_south_east_empty == False or is_south_west_empty == False):
        # IDEA -> anticipate it beforehand
        
        if (is_nord_east_empty == False & is_nord_west_empty == False) & (is_south_east_empty == True & is_south_west_empty == True) & self.is_goal_published==True:
            tmp_nord_east = points_nord_east.max(axis=0)
            tmp_nord_west = points_nord_west.max(axis=0)
            # get the highest values
            if(tmp_nord_east[0] > self.greatest_point_south_east_forward[0]):
                self.greatest_point_nord_east_backward = tmp_nord_east
            if(tmp_nord_west[0] > self.greatest_point_south_west_forward[0]):
                self.greatest_point_nord_west_backward = tmp_nord_west
            print(self.greatest_point_south_east_forward, self.greatest_point_south_west_forward)

            # self.greatest_point_nord_east_backward = points_nord_east.max(axis=0)
            # self.greatest_point_nord_west_backward = points_nord_west.max(axis=0)
        
        if is_nord_east_empty & is_nord_west_empty & is_south_east_empty & is_south_west_empty:
            if (self.is_goal_published==True):
                # self.distance_goal_point_forward = 0.1
                # publish last goal pose    
                # x, y, theta = self.calculate_goal_point_forward()
                self.publish_end_of_line_pose_perpendicular_crop_backward()
                # reset_is_goal_published
                # self.distance_goal_point_forward = self.area[1]/2
                self.update_turning_status_after_pose_publication()
                self.prediction_instance.initialize_prediction()
                return

        # elif (is_nord_east_empty == False | is_nord_west_empty == False) & is_south_east_empty & is_south_west_empty:
                # self.is_begin_line_forward = True
                # self.is_end_line_backward = False
                # self.is_goal_published=True
        # compute bisectrice
        self.prediction_instance.compute_bisectrice_coefficients_backward(points_nord_east, points_nord_west, points_south_east, points_south_west)

        # invoke calculate_goal_position
        x, y, theta = self.calculate_goal_point_backward()

        # publish goal pose
        self.publish_goal_pose(x, y, theta)

        # display 
        self.display_prediction_backward(points_south_east,points_south_west,  x, y)

    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) 

        x = np.where(ranges == -1, np.inf, ranges * np.cos(angles))
        y = np.where(ranges == -1, np.inf, ranges * np.sin(angles))
        # print(x, y)
        points = np.vstack((x, y)).T
        # print(type(points))
        points = points[~np.isinf(points).any(axis=1)]

       
        # if number of points 
        if(np.size(points) < self.min_points):
            x_nord = np.where(((0 <= x) & (x <= self.nord_threshold)),x, np.inf)
            x_south = np.where(((self.south_threshold <= x)&(x <= 0)), x, np.inf)

            y_west = np.where(((0 <= y)&(y<=self.west_threshold)),y, np.inf)
            y_east = np.where(((self.east_threshold <= y)&(y <= 0)),y, np.inf)

            points_nord_east = np.vstack((x_nord, y_east)).T
            points_nord_west = np.vstack((x_nord, y_west)).T
            points_south_east = np.vstack((x_south, y_east)).T
            points_south_west = np.vstack((x_south, y_west)).T
            points_east = np.stack((x, y_east)).T
            points_west = np.stack((x, y_west)).T

            points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
            points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]
            points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
            points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]
            points_east = points_east[~np.isinf(points_east).any(axis=1)]
            points_west = points_west[~np.isinf(points_west).any(axis=1)]



            # Drawing
            self.nord_east_quadrant = [[0,self.nord_threshold],[self.east_threshold,0]]
            self.nord_west_quadrant = [[0,self.nord_threshold],[0,self.west_threshold]]
            self.south_east_quadrant = [[self.south_threshold,0],[self.east_threshold,0]]
            self.south_west_quadrant = [[self.south_threshold,0],[0,self.east_threshold]]

        else:
            
            # distance_from_goal +/- distance_from_bisectrice
            # self.distance_from_bisectrice = 0.3
            # self.tolerance_crop_distance = 0.5
            # take x point and make division east/west

            distance_east_above = -self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_east_below = -self.distance_from_bisectrice - self.tolerance_crop_distance
            distance_west_above = self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_west_below = self.distance_from_bisectrice - self.tolerance_crop_distance

            x_nord = np.where(((0 <= x) & (x <= self.nord_threshold)),x, np.inf)
            x_south = np.where(((self.south_threshold <= x)&(x <= 0)), x, np.inf)

            y_east = np.where((y > distance_east_below)&(y < distance_east_above) ,y, np.inf)
            y_west = np.where((y > distance_west_below)&(y < distance_west_above),y, np.inf)
                    
            points_nord_east = np.vstack((x_nord, y_east)).T
            points_nord_west = np.vstack((x_nord, y_west)).T
            points_south_east = np.vstack((x_south, y_east)).T
            points_south_west = np.vstack((x_south, y_west)).T
            points_east = np.stack((x, y_east)).T
            points_west = np.stack((x, y_west)).T

            points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
            points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]
            points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
            points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]
            points_east = points_east[~np.isinf(points_east).any(axis=1)]
            points_west = points_west[~np.isinf(points_west).any(axis=1)]


            # Drawing
            self.nord_east_quadrant = [[0,self.nord_threshold],[distance_east_below, distance_east_above]]
            self.nord_west_quadrant = [[0,self.nord_threshold],[distance_west_below, distance_west_above]]
            self.south_east_quadrant = [[self.south_threshold,0],[distance_east_below, distance_east_above]]
            self.south_west_quadrant = [[self.south_threshold,0],[distance_west_below, distance_west_above]]
                
            
        return points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west

    def in_row_navigation_forward(self, points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west):
        # get information about emptyness of regions
        is_east_empty = True if np.size(points_east) < self.min_point_direction else False
        is_west_empty = True if np.size(points_west) < self.min_point_direction else False

        if is_east_empty or is_west_empty:
            print("END OF LINE")
            # to do -> publish goal point
            # to do -> reset values
            # reset values
        
        else:
            # enough points to proceed
            # compute bisectrice
            self.prediction_instance.compute_bisectrice_coefficients_forward(points_nord_east,points_nord_west,points_south_east,points_south_west)
            x, y, theta = self.calculate_goal_point_forward()
            # to do -> publish goal
            # to_do -> update queue with goal pose, add checks
            self.display_prediction_forward(points_nord_east, points_nord_west,x,y)
            
    def calculate_goal_point_forward(self):
        # get latest bisectrice coefficients
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # goal pose on robot frame
        theta = math.atan(slope)
        x_goal_robot = self.distance_goal_point_forward * math.cos(theta)
        y_goal_robot = self.distance_goal_point_forward * math.cos(theta)


        alpha = math.atan(math.pi/2 - (2*math.pi - theta))
        delta = math.atan(2*math.pi - theta)

        # project goal pose on actual bisectrice
        distance_robot_bis = intercept * math.cos(delta)
        x_proj_bis = distance_robot_bis * math.cos(alpha)
        y_proj_bis = distance_robot_bis * math.cos(alpha)
                     
        x_goal_pose = x_goal_robot + x_proj_bis
        y_goal_pose = y_goal_robot + y_proj_bis

        return x,y,theta
    
    def publish_goal_pose(self, x, y, theta):
        # create message Float64MultiArray
        goal_pose = Float64MultiArray()
        # update content
        goal_pose.data = [float(x), float(y), float(theta)]
        # publish goal pose
        self.goal_pose_pub.publish(goal_pose)

    def calculate_goal_point_backward(self):
        # takes the last m/q value of bisectrice
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
        
        # calculate goal_points
        tmp_result = pow(slope,2)* pow(intercept,2) - (1+ pow(slope,2))*(pow(intercept,2)-pow(self.distance_goal_point_backward,2))
        
        # safety if-else statement
        if(tmp_result >=0):
            x_1 = (-slope*intercept + math.sqrt(tmp_result))/(1+pow(slope,2))
            x_2 = (-slope*intercept - math.sqrt(tmp_result))/(1+pow(slope,2))
        else:
            x_1 = 0
            x_2 = 0
            print("ERROR QUADRATIC")

        # take smallest value
        if x_1 <= x_2:
            x = x_1
        else:
            x = x_2

        # solve equation
        y = slope*x + intercept

        # take euler angle
        # from atan to atan2
        theta = math.atan2(y,x)
        # turn orientation 180 degrees -> negative velocity
        # theta = math.radians(math.degrees(theta_forward) + 180)
        self.publication_goal_pose_rviz(x,y,theta)
        return x,y,theta

    def publication_goal_pose_rviz(self, x, y, theta):
        # create message Pose
        goal_pose_rviz_msg = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        goal_pose_rviz_msg.header.stamp = time_now.to_msg()
        goal_pose_rviz_msg.header.frame_id = "base_footprint"

        # get x,y
        goal_pose_rviz_msg.pose.position.x = float(x)
        goal_pose_rviz_msg.pose.position.y = float(y)

        # get orientation
        quaternion = quaternion_from_euler(0, 0, theta)
        goal_pose_rviz_msg.pose.orientation.x = quaternion[0]
        goal_pose_rviz_msg.pose.orientation.y = quaternion[1]
        goal_pose_rviz_msg.pose.orientation.z = quaternion[2]
        goal_pose_rviz_msg.pose.orientation.w = quaternion[3]

        #if(goal_pose_rviz_msg.pose.position.x == 1) and (goal_pose_rviz_msg.pose.orientation.w == 1):
            #return
        #else:
            # publish goal pose
        self.pub_goal_pose_rviz.publish(goal_pose_rviz_msg)
        
     # update bool value
    
    def callback_update_bool(self, msg):
        self.publish_goal_position = True
        
    # update turning status: make switch work
    def update_turning_status_after_pose_publication(self):
        self.is_goal_published = False
        self.initialize_parameters_end_of_line()
        # msg = Bool()
        # msg.data = self.publish_goal_position
        # self.pub_turning_status(msg)
    
    def initialize_parameters_end_of_line(self):
        self.greatest_point_south_west_forward = [-10,-10]
        self.greatest_point_south_east_forward = [-10,-10]
        self.greatest_point_nord_west_backward = [-10,-10]
        self.greatest_point_nord_east_backward = [-10,-10]

    def publish_end_of_line_pose(self, x, y, theta):
        # create message Pose
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "base_footprint"

        # get x,y
        end_of_line_pose.pose.position.x = float(x)
        end_of_line_pose.pose.position.y = float(y)

        # get orientation
        quaternion = quaternion_from_euler(0, 0, theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        #if(end_of_line_pose.pose.position.x == 1) and (end_of_line_pose.pose.orientation.w == 1):
            #return
        #else:
            # publish goal pose
        self.end_of_line_pose_topic.publish(end_of_line_pose)

    def publish_end_of_line_pose_perpendicular_crop_forward(self):
        # from last greatest point
        # if(self.greatest_point_south_east_forward != '') & (self.greatest_point_south_west_forward!= ''):
        point_east = self.greatest_point_south_east_forward
        point_west = self.greatest_point_south_west_forward
        # elif(self.greatest_point_nord_east_backward != '') & (self.greatest_point_nord_west_backward!= ''):
            # point_east = self.greatest_point_nord_east_backward
            # point_west = self.greatest_point_nord_west_backward
        # else:
           # print("ERROR")
        
        print(point_east, point_west)
        # m equationfrom two points
        if(point_west[1] - point_east[1] == 0):
            new_theta = math.radians(90)
        elif (point_west[0] - point_east[0] == 0):
            new_theta = math.radians(180)
        else:
            m = (point_west[1]- point_east[1])/(point_west[0] - point_east[0])
            # m_perpendicular
            m_perp = -1/m
            # atan of the new theta
            # Return the arc tangent of x, in radians
            new_theta = math.atan(m_perp)
            # perpendicular m
            # new_theta = math.tan(math.degrees(math.atan(m)) + 90)

        # take previous goal position
        # x,y = self.previous_forward_goal[0], self.previous_forward_goal[1]
        
        y = (point_west[1] - point_east[1])/2
        x = max(point_east[0], point_west[0])

        print("POINT ", x, y,new_theta)
        # create message Pose
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "base_footprint"

        # get x,y
        end_of_line_pose.pose.position.x = float(x)
        end_of_line_pose.pose.position.y = float(y)

        # get orientation
        quaternion = quaternion_from_euler(0, 0, new_theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        #transform from of the received odom to the current map
        transform = self._tf_buffer.lookup_transform('odom', end_of_line_pose.header.frame_id, end_of_line_pose.header.stamp, Duration(seconds=4, nanoseconds=0))
        goal_pose_final = PoseStamped()
        goal_pose_final.header.stamp = time_now.to_msg()
        goal_pose_final.header.frame_id = "odom"

        goal_pose_final.pose.position.x = end_of_line_pose.pose.position.x + transform.transform.translation.x
        goal_pose_final.pose.position.y = end_of_line_pose.pose.position.y + transform.transform.translation.y
        goal_pose_final.pose.position.z = end_of_line_pose.pose.position.z + transform.transform.translation.z
        goal_pose_final.pose.orientation.z = 0.0
        goal_pose_final.pose.orientation.w = 1.0
        goal_pose_final.pose.orientation.y = 0.0
        goal_pose_final.pose.orientation.x = end_of_line_pose.pose.orientation.x + transform.transform.rotation.x

        if(end_of_line_pose.pose.position.x == 1) and (end_of_line_pose.pose.orientation.w == 1):
            return
        else:
            # publish goal pose
            self.end_of_line_pose_topic.publish(goal_pose_final)

    def publish_end_of_line_pose_perpendicular_crop_backward(self):
        # from last greatest point
        # if(self.greatest_point_south_east_forward != '') & (self.greatest_point_south_west_forward!= ''):
        point_east = self.greatest_point_nord_east_backward
        point_west = self.greatest_point_nord_west_backward
        # elif(self.greatest_point_nord_east_backward != '') & (self.greatest_point_nord_west_backward!= ''):
            # point_east = self.greatest_point_nord_east_backward
            # point_west = self.greatest_point_nord_west_backward
        # else:
           # print("ERROR")
        
        print(point_east, point_west)
        # m equationfrom two points
        if(point_west[1] - point_east[1] == 0):
            new_theta = math.radians(90)
        elif (point_west[0] - point_east[0] == 0):
            new_theta = math.radians(180)
        else:
            m = (point_west[1]- point_east[1])/(point_west[0] - point_east[0])
            # m_perpendicular
            m_perp = -1/m
            # atan of the new theta
            # Return the arc tangent of x, in radians
            new_theta = math.atan(m_perp)
            # perpendicular m
            # new_theta = math.tan(math.degrees(math.atan(m)) + 90)

        # take previous goal position
        # x,y = self.previous_forward_goal[0], self.previous_forward_goal[1]
        
        y = (point_west[1] - point_east[1])/2
        x = max(point_east[0], point_west[0])

        print("POINT ", x, y,new_theta)
        # create message Pose
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "base_footprint"

        # get x,y
        end_of_line_pose.pose.position.x = float(x)
        end_of_line_pose.pose.position.y = float(y)

        # get orientation
        quaternion = quaternion_from_euler(0, 0, new_theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        if(end_of_line_pose.pose.position.x == 1) and (end_of_line_pose.pose.orientation.w == 1):
            return
        else:
            # publish goal pose
            self.end_of_line_pose_topic.publish(end_of_line_pose)

    def display_prediction_forward(self, nord_east_points,nord_west_points,x_goal, y_goal):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(nord_west_points[:, 0], nord_west_points[:, 1], color='red')
        plt.scatter(nord_east_points[:, 0], nord_east_points[:, 1], color='orange')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 1.5, 3)

        # get proper slope, intercept for each value
        nord_west_slope, nord_west_intercept = self.prediction_instance.nord_west_line.get_most_recent_coefficients()
        nord_east_slope, nord_east_intercept = self.prediction_instance.nord_east_line.get_most_recent_coefficients()
        bisectrice_slope, bisectrice_intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # creates line
        y_nord_west = nord_west_slope * x + nord_west_intercept
        y_nord_east = nord_east_slope * x + nord_east_intercept

        # plot line
        plt.plot(x, y_nord_west, color='red')
        plt.plot(x, y_nord_east, color='orange')

        # robot line
        y = bisectrice_slope * x + bisectrice_intercept

        # bisectrice
        plt.plot(x, y, color='green')

        # goal point 
        plt.plot(x_goal, y_goal, marker="o", markersize=10, markeredgecolor="green", markerfacecolor="green")

        # nord east
        x_values_1 = self.nord_east_quadrant[0]
        y_values_1 = [self.nord_east_quadrant[1][0], self.nord_east_quadrant[1][0]]
        plt.plot(x_values_1, y_values_1, color='orange',linestyle="--")

        x_values_2 = self.nord_east_quadrant[0]
        y_values_2 = [self.nord_east_quadrant[1][1], self.nord_east_quadrant[1][1]]
        plt.plot(x_values_2, y_values_2,color='orange', linestyle="--")

        # nord west
        x_values_3 = self.nord_west_quadrant[0]
        y_values_3 = [self.nord_west_quadrant[1][0], self.nord_west_quadrant[1][0]]
        plt.plot(x_values_3, y_values_3, color='red',linestyle="--")

        x_values_4 = self.nord_west_quadrant[0]
        y_values_4 = [self.nord_west_quadrant[1][1], self.nord_west_quadrant[1][1]]
        plt.plot(x_values_4, y_values_4,color='red', linestyle="--")

        plt.xlim(-3,3)
        plt.ylim(-3,3)

        self.fig.canvas.draw()
        plt.pause(0.01) 
    
def main(args=None):
    rclpy.init(args=args)

    in_row_navigation = InRowNavigation()

    rclpy.spin(in_row_navigation)

    in_row_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
