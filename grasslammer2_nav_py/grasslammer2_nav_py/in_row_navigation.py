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
absolute_path = '/home/carlo/Documenti/robotics/src/FRE-2023/grasslammer2_description/config/in_row_params.json'
#absolute_path = "/home/airlab/workspace/ros2/src/grasslammer2/grasslammer2_description/config/in_row_params.json"
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
        # if not is_bisectrice:
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
        print("num points", len(point_west), len(point_east))
        # print("num points", len(points))
        # add delta threshold
        if(len(point_west)> self.num_min_points_required_for_fitting) and (len(point_east)> self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope_west, intercept_west = line_west.fitting_line(point_west)
            # compute slope, intercept through fitting
            slope_east, intercept_east = line_east.fitting_line(point_east)

            slope_east = slope_east if slope_east*slope_west>0 else -slope_east
            
            # intercept_east = intercept_east if intercept_east*intercept_west > 0 else -intercept_east
            
            # reviewed
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
            # intercept_east = intercept_east if intercept_east*intercept_west > 0 else -intercept_east
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
            # intercept_east = intercept_east if intercept_east*intercept_west > 0 else -intercept_east
            # check if distanciated enough
            if(abs(intercept_west - intercept_east) < self.line_with - self.tolerance_intercept) and (abs(intercept_west - intercept_east) > self.line_with + self.tolerance_intercept):
                slope_west, intercept_west = line_west.get_most_recent_coefficients()
            # check if parallel
            elif (abs(slope_west - slope_east) > self.tolerance_slope):
                slope_west, intercept_west = line_west.get_most_recent_coefficients()
        else:
            slope_west, intercept_west = line_west.get_most_recent_coefficients()
            slope_east, intercept_east = line_east.get_most_recent_coefficients()
        
        print(slope_west, intercept_west, slope_east, intercept_east)
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

        self.area, dimension_queue, self.min_points, self.line_width, self.tolerance_crop_distance, self.distance_goal_point_forward, distance_goal_point_backward, nord_threshold, south_threshold = self.get_parameters_from_config_file()
        self.distance_from_bisectrice = self.line_width/2

        # min points required per direction
        self.min_point_direction = self.min_points * 2 -1

        # define thresholds
        self.nord_threshold = self.area[0]/2
        self.south_threshold = -self.area[0]/2
        self.west_threshold = self.area[1]/2
        self.east_threshold = -self.area[1]/2

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

        # prediction
        self.prediction_instance = Prediction()


        # save last goal position
        self.end_of_line_pose_message = ''
        self.line_entered = False

        # Display Ransac
        self.fig, self.ax = plt.subplots()

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
        points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian_static(scan_msg)

        # print(points_2d)
        # if going forward/backward 

        if(self.moving_forward):
            self.in_row_navigation_forward(points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west)
        else:
            self.in_row_navigation_backward(points_nord_east, points_nord_west, points_south_east, points_south_west)
           
    def laser_scan_to_cartesian_static(self, msg):
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

    def display_reasoning(self, x_goal_robot, y_goal_robot,x_proj_bis, y_proj_bis,x_goal_pose,y_goal_pose):
        origin = np.array([[0, 0, 0],[0, 0, 0]]) # origin point

        vector = np.array([[x_goal_robot,y_goal_robot],[x_proj_bis,y_proj_bis], [x_goal_pose,y_goal_pose]])

        plt.quiver(*origin, vector[:,0], vector[:,1], color=['r','b','g'], scale=21)
        plt.show()
    

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


        #self.display_reasoning(x_goal_robot, y_goal_robot, x_proj_bis, y_proj_bis, x_goal_pose, y_goal_pose)

        return x_goal_pose, y_goal_pose, theta

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
