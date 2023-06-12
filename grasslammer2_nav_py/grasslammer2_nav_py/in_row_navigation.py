import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray,Bool, Char
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

# QUEUEfile
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
# absolute_path = '/home/ceru/robotics/src/FRE-2023/grasslammer2_description/config/in_row_params.json'
# absolute_path = '/home/alba/ros2_ws/src/FRE-2023/grasslammer2_description/config/in_row_params.json'
pkg_path = os.path.realpath("src/FRE-2023/grasslammer2_description")
config_file = open(pkg_path + '/config/in_row_params_sim.json', 'r')
print(config_file)
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
        self.queue.appendleft(self.reset_value)
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
#################### COSTUM QUEUE
##################################
class PoseQueue:
    def __init__(self, dimension_queue):
        self.dimension_queue = dimension_queue
        self.queue = deque()

    # update queue regularly
    def update_queue_regularly(self, value):
        # print("update_queue_regularly ", len(self.queue),self.dimension_queue)
        if(len(self.queue) >= self.dimension_queue):
            self.queue.pop()
        self.queue.appendleft(value)
            
    # initialize queue
    def initialize_queue(self):
        self.queue = deque()
    
    # needed to compute validity data
    def return_element_time_t(self):
        if(len(self.queue) > 0):
            return self.queue[0]
        else:
            return PoseStamped()

    # calculate average yaw 
    def queue_average_theta(self):
        sum_tetha = 0.0
        if len(self.queue)>0:
            for item in self.queue:
                (_, _, yaw) = euler_from_quaternion([item.pose.orientation.x, item.pose.orientation.y, item.pose.orientation.z, item.pose.orientation.w])
                sum_tetha += yaw
            return sum_tetha/len(self.queue)
        else:
            return 0.0

    # calculate average yaw 
    def queue_average_x_y(self):
        sum_x = 0.0
        sum_y = 0.0

        if len(self.queue)>0:
            for item in self.queue:
                sum_x += item.pose.position.x
                sum_y += item.pose.position.y
            return sum_x/len(self.queue), sum_y/len(self.queue)
        else:
            return 0.0, 0.0
        


#################################
#################### LINE
##################################    

class Line():
    # assume type are exponentials
    def __init__(self, quadrant):
        # take parameters from config file
        dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope = self.get_parameters_from_config_file(quadrant)
        
        # check ma flag
        if ma_avarage_flag == 1:
            self.slope = MovingAvarageQueue(dimension_queue=dimension_queue,reset_value=reset_value_slope)
            self.intercept = MovingAvarageQueue(dimension_queue=dimension_queue,reset_value=reset_value_intercept)
        else:
            self.slope = CustomQueue(dimension_queue, reset_value=reset_value_slope)
            self.intercept = CustomQueue(dimension_queue, reset_value=reset_value_intercept)
        
        # if not bisectrice, ransac needed
        self.is_bisectrice = is_bisectrice
        # if not is_bisectrice:
        # min_samples=20, 
        if not self.is_bisectrice:
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
                    # intercept = intercept if intercept_time_t_min_1*intercept > 0 else -intercept
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
        intercept = float64(self.intercept.return_oldest_element())
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
        # line_width, min_num_ransac, tolerance_intercept
        # TODO tolerance slope
        self.line_with, self.num_min_points_required_for_fitting, self.tolerance_intercept = self.get_parameters_from_config_file()
    
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
    
    def compute_bisectrice_coefficients_backward(self, nord_east_points, nord_west_points, south_east_points, south_west_points):
        _,_,_,_ = self.calculate_crop_coefficients(self.nord_west_line, nord_west_points, self.nord_east_line, nord_east_points) 
        south_west_slope,south_west_intercept, south_east_slope, south_east_intercept = self.calculate_crop_coefficients(self.south_west_line, south_west_points, self.south_east_line, south_east_points) 

        # compute bisectrice
        medium_slope = (south_west_slope+south_east_slope) / 2
        medium_intercept = (south_east_intercept+south_west_intercept) / 2

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
            # print("delta intercept", abs(intercept_west - intercept_east), self.line_with - self.tolerance_intercept, self.line_with + self.tolerance_intercept)
            
            # reviewed
            # check if distanciated enough
            if not ((abs(intercept_west - intercept_east) > self.line_with - self.tolerance_intercept) and (abs(intercept_west - intercept_east) < self.line_with + self.tolerance_intercept)):
                # print("delta intercept 1")
                slope_west, intercept_west = line_west.get_most_recent_coefficients()
                slope_east, intercept_east = line_east.get_most_recent_coefficients()
            
        
        elif (len(point_west) < self.num_min_points_required_for_fitting) and (len(point_east) > self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope_west, intercept_west = line_west.get_most_recent_coefficients()
            # compute slope, intercept through fitting
            slope_east, intercept_east = line_east.fitting_line(point_east)

            # check if distanciated enough
            if not (abs(intercept_west - intercept_east) > self.line_with - self.tolerance_intercept) and (abs(intercept_west - intercept_east) < self.line_with + self.tolerance_intercept):
                slope_east, intercept_east = line_east.get_most_recent_coefficients()
        
        elif (len(point_west) > self.num_min_points_required_for_fitting) and (len(point_east) < self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope_west, intercept_west = line_west.fitting_line(point_west)
            # compute slope, intercept through fitting
            slope_east, intercept_east = line_east.get_most_recent_coefficients()
            # check if distanciated enough
            if not (abs(intercept_west - intercept_east) > self.line_with - self.tolerance_intercept) and (abs(intercept_west - intercept_east) < self.line_with + self.tolerance_intercept):
                slope_west, intercept_west = line_west.get_most_recent_coefficients()

        else:
            slope_west, intercept_west = line_west.get_most_recent_coefficients()
            slope_east, intercept_east = line_east.get_most_recent_coefficients()
        
        print("slope and intercept", slope_west, intercept_west, slope_east, intercept_east)
        
        # update lines
        line_west.update_line_parameters_checking_threshold(slope_west,intercept_west)
        line_east.update_line_parameters_checking_threshold(slope_east, intercept_east)
        return slope_west, intercept_west, slope_east, intercept_east
    
    def get_parameters_from_config_file(self):
        line_width = config_json['line_width']
        min_num_ransac = config_json['prediction']['min_num_ransac']
        tolerance_intercept = config_json['prediction']['tolerance_intercept']
        return line_width, min_num_ransac, tolerance_intercept


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
        self.sub_turning_status = self.create_subscription(Bool, '/end_of_turning', self.callback_update_bool, 1)   
        # topic needed for the integration with obstacle detection
        self.change_moving_direction_sub = self.create_subscription(Char, "/obstacle_detection", self.callback_update_moving, 1)
        self.end_of_line_pose_topic # prevent unused variable warning
        # topic to update the boolean variable to publish
        #self.sub_turning_status = self.create_subscription(Bool, '/end_of_turning', self.callback_update_bool, 1)

        self.area, self.min_num_quad_EOL, self.min_num_point_extended_region, self.dimension_queue, self.min_num_points_use_bis, self.line_width, self.tolerance_crop_distance, self.distance_goal_point_forward, self.distance_goal_point_backward, self.nord_threshold, self.south_threshold, self.nord_threshold_fw, self.south_threshold_bw = self.get_parameters_from_config_file()
        self.distance_from_bisectrice = self.line_width/2

        # min points required per direction
        self.min_point_direction = int(self.min_num_points_use_bis/2)

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

        # pose quee
        self.end_pose_queue = PoseQueue(self.dimension_queue)

        # buffer for looking at tf transformations
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # save last goal position
        self.end_of_line_pose_message = ''
        self.is_in_row_navigation = True
        self.flag_publish_goal_pose = True
        # last goal position
        self.last_goal_position_robot_frame = [0,0]

        # Display Ransac
        # self.fig, self.ax = plt.subplots()
        # Display Vectors
        # self.fig2, self.ax2 = plt.subplots()

    def get_parameters_from_config_file(self):
        area = config_json["area"]
        min_num_point_quad_EOL = config_json["min_num_point_EOL_quad"]
        min_num_point_extended_region = config_json["min_num_point_extended_region"]
        dimension_queue = config_json["in_row_navigation"]['dimension_queue']
        min_num_points_use_bis = config_json["in_row_navigation"]['min_num_points_use_bis']
        line_width = config_json['line_width']
        tolerance_crop_distance_filtering = config_json["in_row_navigation"]['tolerance_crop_distance_filtering']
        distance_goal_point_forward = config_json["in_row_navigation"]['distance_goal_point_forward']
        distance_goal_point_backward = config_json["in_row_navigation"]['distance_goal_point_backward']
        nord_threshold = config_json["in_row_navigation"]['nord_threshold']
        south_threshold = config_json["in_row_navigation"]['south_threshold']
        nord_threshold_fw = config_json["in_row_navigation"]['nord_threshold_fw']
        south_threshold_bw = config_json["in_row_navigation"]['south_threshold_bw']
        return area, min_num_point_quad_EOL, min_num_point_extended_region,dimension_queue, min_num_points_use_bis, line_width, tolerance_crop_distance_filtering, distance_goal_point_forward, distance_goal_point_backward, nord_threshold, south_threshold, nord_threshold_fw, south_threshold_bw
    
    def scan_callback(self, scan_msg):
        self.start_computation = time.perf_counter()
        # points_nord_east, points_nord_west, points_south_east, points_south_west
        # points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian_dynamic(scan_msg)

        # print(points_2d)
        # if going forward/backward 

        if(self.moving_forward):
            points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian_forward(scan_msg)
            self.in_row_navigation_forward(points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west)
        else:
            points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian_backward(scan_msg)
            self.in_row_navigation_backward(points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west)
           
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
        if(np.size(points) < self.min_num_points_use_bis):
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

    def laser_scan_to_cartesian_dynamic(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) 

        x = np.where(ranges == -1, np.inf, ranges * np.cos(angles))
        y = np.where(ranges == -1, np.inf, ranges * np.sin(angles))
        # print(x, y)
        points = np.vstack((x, y)).T
        # print(type(points))
        points = points[~np.isinf(points).any(axis=1)]


        # if number of points 
        if(np.size(points) < self.min_num_points_use_bis):
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
            # self.nord_east_quadrant = [[0,self.nord_threshold],[self.east_threshold,0]]
            # self.nord_west_quadrant = [[0,self.nord_threshold],[0,self.west_threshold]]
            # self.south_east_quadrant = [[self.south_threshold,0],[self.east_threshold,0]]
            # self.south_west_quadrant = [[self.south_threshold,0],[0,self.east_threshold]]

        else:
            # take slope, intercept 
            slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
            
            distance_east_above = intercept - self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_east_below = intercept - self.distance_from_bisectrice - self.tolerance_crop_distance
            distance_west_above = intercept + self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_west_below = intercept + self.distance_from_bisectrice - self.tolerance_crop_distance
                
            threshold_east_above = np.full_like(y, distance_east_above)
            threshold_east_below = np.full_like(y, distance_east_below)
            threshold_west_above = np.full_like(y, distance_west_above)
            threshold_west_below = np.full_like(y, distance_west_below)


            y_west = np.where((y > np.add(slope*x, threshold_west_below))&(y < np.add(slope*x, threshold_west_above)),y, np.inf)
            y_east = np.where((y > np.add(slope*x, threshold_east_below))&(y < np.add(slope*x,threshold_east_above)),y, np.inf)

            threshold_nord = np.full_like(x, intercept*slope + self.nord_threshold)
            threshold_south = np.full_like(x, intercept*slope + self.south_threshold)

            m_q = np.full_like(x, slope*intercept)

            # x = -my+qm
            # error multiply infinity
            x_nord_east = np.where((x > np.add(-slope*y_east, m_q))&(x < np.add(-slope*y_east, threshold_nord)) ,x, np.inf)
            x_nord_west = np.where((x > np.add(-slope*y_west, m_q))&(x < np.add(-slope*y_west,threshold_nord)),x, np.inf)
            x_south_east = np.where((x < np.add(-slope*y_east, m_q))&(x > np.add(-slope*y_east, threshold_south)) ,x, np.inf)
            x_south_west = np.where((x < np.add(-slope*y_west, m_q))&(x > np.add(-slope*y_west,threshold_south)) ,x, np.inf)
            
            points_east = np.stack((x, y_east)).T
            points_west = np.stack((x, y_west)).T
            points_nord_east = np.vstack((x_nord_east, y_east)).T
            points_nord_west = np.vstack((x_nord_west, y_west)).T
            points_south_east = np.vstack((x_south_east, y_east)).T
            points_south_west = np.vstack((x_south_west, y_west)).T

            points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
            points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]
            points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
            points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]
                
        return points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west

    def laser_scan_to_cartesian_forward(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) 

        x = np.where(ranges == -1, np.inf, ranges * np.cos(angles))
        y = np.where(ranges == -1, np.inf, ranges * np.sin(angles))
        # print(x, y)
        points = np.vstack((x, y)).T
        # print(type(points))
        points = points[~np.isinf(points).any(axis=1)]

        # if number of points 
        if(np.size(points) < self.min_num_points_use_bis):
            x_nord = np.where(((0 <= x) & (x <= self.nord_threshold_fw)),x, np.inf)
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
            
            # check have enough number of points
            is_backward_full = True if np.size(points_south_west) > self.min_num_point_extended_region and np.size(points_south_east) > self.min_num_point_extended_region else False
            is_forward_full = True if np.size(points_nord_west) > self.min_num_point_extended_region and np.size(points_nord_east) > self.min_num_point_extended_region else False
        
            # shrink region in case you have enough points
            if is_backward_full or is_forward_full:
                # recompute the nord points
                x_nord = np.where(((0 <= x) & (x <= self.nord_threshold)),x, np.inf)
                points_nord_east = np.vstack((x_nord, y_east)).T
                points_nord_west = np.vstack((x_nord, y_west)).T
                points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
                points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]



        else:
            # take slope, intercept 
            slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
            
            distance_east_above = intercept - self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_east_below = intercept - self.distance_from_bisectrice - self.tolerance_crop_distance
            distance_west_above = intercept + self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_west_below = intercept + self.distance_from_bisectrice - self.tolerance_crop_distance
                
            threshold_east_above = np.full_like(y, distance_east_above)
            threshold_east_below = np.full_like(y, distance_east_below)
            threshold_west_above = np.full_like(y, distance_west_above)
            threshold_west_below = np.full_like(y, distance_west_below)

            y_west = np.where((y > np.add(slope*x, threshold_west_below))&(y < np.add(slope*x, threshold_west_above)),y, np.inf)
            y_east = np.where((y > np.add(slope*x, threshold_east_below))&(y < np.add(slope*x,threshold_east_above)),y, np.inf)

            threshold_nord = np.full_like(x, intercept*slope + self.nord_threshold_fw)
            threshold_south = np.full_like(x, intercept*slope + self.south_threshold)

            m_q = np.full_like(x, slope*intercept)

            # x = -my+qm
            # error multiply infinity
            # updated
            x_east = np.where((x > np.add(-slope*y_east, threshold_south))&(x < np.add(-slope*y_east, threshold_nord)) ,x, np.inf)
            x_west = np.where((x > np.add(-slope*y_west, threshold_south))&(x < np.add(-slope*y_west,threshold_nord)),x, np.inf)
            points_east = np.stack((x_east, y_east)).T
            points_west = np.stack((x_west, y_west)).T

            x_nord_east = np.where((x > np.add(-slope*y_east, m_q))&(x < np.add(-slope*y_east, threshold_nord)) ,x, np.inf)
            x_nord_west = np.where((x > np.add(-slope*y_west, m_q))&(x < np.add(-slope*y_west,threshold_nord)),x, np.inf)
            x_south_east = np.where((x < np.add(-slope*y_east, m_q))&(x > np.add(-slope*y_east, threshold_south)) ,x, np.inf)
            x_south_west = np.where((x < np.add(-slope*y_west, m_q))&(x > np.add(-slope*y_west,threshold_south)) ,x, np.inf)
            
            points_nord_east = np.vstack((x_nord_east, y_east)).T
            points_nord_west = np.vstack((x_nord_west, y_west)).T
            points_south_east = np.vstack((x_south_east, y_east)).T
            points_south_west = np.vstack((x_south_west, y_west)).T

            points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
            points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]
            points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
            points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]

            # check have enough number of points
            is_backward_full = True if np.size(points_south_west) > self.min_num_point_extended_region and np.size(points_south_east) > self.min_num_point_extended_region else False
            is_forward_full = True if np.size(points_nord_west) > self.min_num_point_extended_region and np.size(points_nord_east) > self.min_num_point_extended_region else False
        
            # shrink region in case you have enough points
            # update only nord regions
            if is_backward_full or is_forward_full:
                # recompute the nord points
                threshold_nord = np.full_like(x, intercept*slope + self.nord_threshold)

                x_nord_east = np.where((x > np.add(-slope*y_east, m_q))&(x < np.add(-slope*y_east, threshold_nord)) ,x, np.inf)
                x_nord_west = np.where((x > np.add(-slope*y_west, m_q))&(x < np.add(-slope*y_west,threshold_nord)),x, np.inf)

                points_nord_east = np.vstack((x_nord_east, y_east)).T
                points_nord_west = np.vstack((x_nord_west, y_west)).T

                points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
                points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]

        return points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west

    def laser_scan_to_cartesian_backward(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) 

        x = np.where(ranges == -1, np.inf, ranges * np.cos(angles))
        y = np.where(ranges == -1, np.inf, ranges * np.sin(angles))
        # print(x, y)
        points = np.vstack((x, y)).T
        # print(type(points))
        points = points[~np.isinf(points).any(axis=1)]


        # if number of points 
        if(np.size(points) < self.min_num_points_use_bis):
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
            # self.nord_east_quadrant = [[0,self.nord_threshold],[self.east_threshold,0]]
            # self.nord_west_quadrant = [[0,self.nord_threshold],[0,self.west_threshold]]
            # self.south_east_quadrant = [[self.south_threshold,0],[self.east_threshold,0]]
            # self.south_west_quadrant = [[self.south_threshold,0],[0,self.east_threshold]]

        else:
            # take slope, intercept 
            slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
            
            distance_east_above = intercept - self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_east_below = intercept - self.distance_from_bisectrice - self.tolerance_crop_distance
            distance_west_above = intercept + self.distance_from_bisectrice + self.tolerance_crop_distance
            distance_west_below = intercept + self.distance_from_bisectrice - self.tolerance_crop_distance
                
            threshold_east_above = np.full_like(y, distance_east_above)
            threshold_east_below = np.full_like(y, distance_east_below)
            threshold_west_above = np.full_like(y, distance_west_above)
            threshold_west_below = np.full_like(y, distance_west_below)

            y_west = np.where((y > np.add(slope*x, threshold_west_below))&(y < np.add(slope*x, threshold_west_above)),y, np.inf)
            y_east = np.where((y > np.add(slope*x, threshold_east_below))&(y < np.add(slope*x,threshold_east_above)),y, np.inf)

            threshold_nord = np.full_like(x, intercept*slope + self.nord_threshold)
            threshold_south = np.full_like(x, intercept*slope + self.south_threshold)

            m_q = np.full_like(x, slope*intercept)

            # x = -my+qm
            # error multiply infinity
            x_nord_east = np.where((x > np.add(-slope*y_east, m_q))&(x < np.add(-slope*y_east, threshold_nord)) ,x, np.inf)
            x_nord_west = np.where((x > np.add(-slope*y_west, m_q))&(x < np.add(-slope*y_west,threshold_nord)),x, np.inf)
            x_south_east = np.where((x < np.add(-slope*y_east, m_q))&(x > np.add(-slope*y_east, threshold_south)) ,x, np.inf)
            x_south_west = np.where((x < np.add(-slope*y_west, m_q))&(x > np.add(-slope*y_west,threshold_south)) ,x, np.inf)
            
            points_east = np.stack((x, y_east)).T
            points_west = np.stack((x, y_west)).T
            points_nord_east = np.vstack((x_nord_east, y_east)).T
            points_nord_west = np.vstack((x_nord_west, y_west)).T
            points_south_east = np.vstack((x_south_east, y_east)).T
            points_south_west = np.vstack((x_south_west, y_west)).T

            points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
            points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]
            points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
            points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]
        
        is_forward_empty = True if np.size(points_nord_east) < self.min_num_quad_EOL and np.size(points_nord_east) < self.min_num_quad_EOL else False
        
        # update south points -> more points for prediction
        if is_forward_empty:
            x_south = np.where(((0 <= x) & (x <= self.south_threshold_bw)),x, np.inf)
            points_south_east = np.vstack((x_nord, y_east)).T
            points_south_west = np.vstack((x_nord, y_west)).T
            points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
            points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]

        return points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west

    def initialization_quadrants(self):
        self.nord_east_quadrant = [[0,self.nord_threshold],[self.east_threshold,0]]
        self.nord_west_quadrant = [[0,self.nord_threshold],[0,self.west_threshold]]
        self.south_east_quadrant = [[self.south_threshold,0],[self.east_threshold,0]]
        self.south_west_quadrant = [[self.south_threshold,0],[0,self.east_threshold]]

    def in_row_navigation_forward(self, points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west):
        # get information about emptyness of regions
        is_nord_east_empty = True if np.size(points_nord_east) < self.min_num_quad_EOL else False
        is_nord_west_empty = True if np.size(points_nord_west) < self.min_num_quad_EOL else False

        if (is_nord_east_empty or is_nord_west_empty) and (self.is_in_row_navigation):
            print("END OF LINE")
            # to do -> publish goal point
            self.publish_end_pose()
            # to do -> reset values
            self.end_pose_queue.initialize_queue()
            # reset values
            self.prediction_instance.initialize_prediction()
            # reset quadrants
            self.initialization_quadrants()
            # reset value
            self.is_in_row_navigation = False
        
        else:
            # enough points to proceed
            # compute bisectrice
            self.prediction_instance.compute_bisectrice_coefficients_forward(points_nord_east,points_nord_west,points_south_east,points_south_west)
            # calculate goal point
            goal_pose, x, y = self.calculate_goal_point_forward()
            # only 
            if self.flag_publish_goal_pose:
                # publish goal pose
                self.publish_goal_pose(x, y)
                # to_do -> update queue with goal pose, add checks
                self.validate_end_pose(goal_pose, points_nord_east, points_nord_west)
                # display prediction
                self.display_prediction_forward_dynamic(points_nord_east, points_nord_west, x, y)

    # update bool value   
    def callback_update_bool(self, msg):
        if(not self.is_in_row_navigation):
            time.sleep(2)
        self.is_in_row_navigation = msg.data
        self.flag_publish_goal_pose = True
        
    # modify behaviour 
    def callback_update_moving(self, msg):
        if msg.data == 'S':
            self.flag_publish_goal_pose = False
        elif msg.data == 'D':
            time.sleep(5)
            self.prediction_instance.initialize_prediction()
            self.flag_publish_goal_pose = True
        elif msg.data == 'H':
            self.moving_forward = False
            self.flag_publish_goal_pose = True

    # accept points enough in terms of quadrants
    def validate_end_pose(self, goal_pose, point_east, point_west):
        is_east_empty = True if np.size(point_east) < self.min_num_quad_EOL else False
        is_west_empty = True if np.size(point_west) < self.min_num_quad_EOL else False
        if not is_east_empty and not is_west_empty:
            self.end_pose_queue.update_queue_regularly(goal_pose)
    
    def publish_end_pose(self):
        end_pose = self.end_pose_queue.return_element_time_t()
        avg_thetha = self.end_pose_queue.queue_average_theta()
        avg_x, avg_y = self.end_pose_queue.queue_average_x_y()
        (x, y, z, w) = quaternion_from_euler(0, 0, avg_thetha)
        end_pose.pose.orientation.x = x
        end_pose.pose.orientation.y = y
        end_pose.pose.orientation.z = z
        end_pose.pose.orientation.w = w

        # modify position
        end_pose.pose.position.x = avg_x
        end_pose.pose.position.y = avg_y

        # update timestamp and frame
        # transform from of the received odom to the current map
        transform = self._tf_buffer.lookup_transform('map', 'odom', end_pose.header.stamp, Duration(seconds=4, nanoseconds=0))
        self.end_of_line_pose_topic.publish(do_transform_pose_stamped(end_pose, transform))
    
    def publish_goal_pose(self, x, y):
        # create message Float64MultiArray
        goal_pose = Float64MultiArray()
        theta = math.atan2(y,x)
        # update content
        goal_pose.data = [float(x), float(y), float(theta)]
        if(self.is_in_row_navigation):
            # publish goal pose
            self.goal_pose_pub.publish(goal_pose)

    def display_reasoning(self, x_goal_robot, y_goal_robot,x_proj_bis, y_proj_bis,x_goal_pose,y_goal_pose):
        # clear axes
        # to do -> segment
        plt.figure(1)
        plt.gca().clear()

        #plt.rcParams["figure.figsize"] = [7.50, 3.50]
        #plt.rcParams["figure.autolayout"] = True
        robot_center_point = [0,0]
        robot_goal_point = [x_goal_robot, y_goal_robot]
        proj_bis_point = [x_proj_bis, y_proj_bis]
        # goal_pose_point = [x_goal_pose, y_goal_pose]



        x_vector_robot = [robot_center_point[0], robot_goal_point[0]]
        y_vector_robot = [robot_center_point[1], robot_goal_point[1]]
        x_vector_projection = [robot_center_point[0], proj_bis_point[0]]
        y_vector_projection = [robot_center_point[1], proj_bis_point[1]]
        # x_vector_pose = [robot_center_point[0], goal_pose_point[0]]
        # y_vector_pose = [robot_center_point[1], goal_pose_point[1]]

        plt.plot(x_vector_robot, y_vector_robot, color="red", linestyle="solid")
        plt.plot(x_vector_projection, y_vector_projection, color ="orange", linestyle="solid")
        # plt.plot(x_vector_pose, y_vector_pose, color="green", linestyle="solid")
        plt.plot(x_goal_robot, y_goal_robot, marker="o", markersize=10, markeredgecolor="red", markerfacecolor="red")
        plt.plot(x_proj_bis, y_proj_bis, marker="o", markersize=10, markeredgecolor="orange", markerfacecolor="orange")
        plt.plot(x_goal_pose, y_goal_pose, marker="o", markersize=10, markeredgecolor="green", markerfacecolor="green")
        # plt.text(robot_center_point[0]-0.015, robot_center_point[1]+0.015, "robot_center_point")
        plt.text(robot_goal_point[0]-0.015, robot_goal_point[1]-0.015, "robot_goal_point")
        plt.text(proj_bis_point[0]-0.015, proj_bis_point[1]+0.015, "proj_bis_point")
        # plt.text(goal_pose_point[0]-0.015, goal_pose_point[1]+0.015, "goal_pose_point")

        plt.xlim(-0.6,0.6)
        plt.ylim(-0.1,0.1)

        plt.gcf().canvas.draw()
        plt.pause(0.01) 
    
    def calculate_goal_point_forward(self):
        # get latest bisectrice coefficients
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # goal pose on robot frame
        theta = math.atan(slope)
        x_goal_robot = self.distance_goal_point_forward * math.cos(theta)
        y_goal_robot = self.distance_goal_point_forward * math.sin(theta)

        # alpha = math.atan(math.pi/2 - (2*math.pi - theta))
        # delta = math.atan(2*math.pi - theta)
        delta = - theta
        alpha = math.atan(math.pi/2 - delta)

        # project goal pose on actual bisectrice
        distance_robot_bis = intercept * math.cos(delta)
        x_proj_bis = distance_robot_bis * math.cos(alpha)
        y_proj_bis = distance_robot_bis * math.sin(alpha)
                     
        x_goal_pose = x_goal_robot + x_proj_bis
        y_goal_pose = y_goal_robot + y_proj_bis

        self.display_reasoning(x_goal_robot, y_goal_robot, x_proj_bis, y_proj_bis, x_goal_pose, y_goal_pose)
        # print(x_goal_robot, y_goal_robot, x_proj_bis, y_proj_bis, x_goal_pose, y_goal_pose)

        # goal as a pose in odom frame from velodyne 
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "velodyne2"

        # get x,y
        end_of_line_pose.pose.position.x = float(x_goal_pose)
        end_of_line_pose.pose.position.y = float(y_goal_pose)
        end_of_line_pose.pose.position.z = float(0.0)
        # get orientation
        quaternion = quaternion_from_euler(0, 0, theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        #transform from of the received odom to the current map
        try:
            transform = self._tf_buffer.lookup_transform('odom', 'velodyne2', end_of_line_pose.header.stamp, Duration(seconds=4, nanoseconds=0))
            return do_transform_pose_stamped(end_of_line_pose, transform), x_goal_pose, y_goal_pose
        except:
            self.get_logger().info("ERROR IN POSE CALCULATION")
            pose_error = PoseStamped()
            return (pose_error, None, None)

    def display_prediction_forward_static(self, nord_east_points,nord_west_points,x_goal, y_goal):
        # clear axes
        # self.ax.clear()
        plt.figure(0)
        plt.gca().clear()
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

        plt.xlim(0,2)
        plt.ylim(-1,1)

        plt.gcf().canvas.draw()
        plt.pause(0.01) 
    
    def display_prediction_forward_dynamic(self, nord_east_points,nord_west_points,x_goal, y_goal):
        # clear axes
        # self.ax.clear()
        plt.figure(0)
        plt.gca().clear()
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
        y_nord_west_below = bisectrice_slope * x + bisectrice_intercept + self.distance_from_bisectrice - self.tolerance_crop_distance
        y_nord_west_above = bisectrice_slope * x + bisectrice_intercept + self.distance_from_bisectrice + self.tolerance_crop_distance
        y_nord_east_below = bisectrice_slope * x + bisectrice_intercept - self.distance_from_bisectrice - self.tolerance_crop_distance
        y_nord_east_above = bisectrice_slope * x + bisectrice_intercept - self.distance_from_bisectrice + self.tolerance_crop_distance
        
        # plot line
        plt.plot(x, y_nord_west_below, color='red', linestyle="--")
        plt.plot(x, y_nord_west_above, color='red', linestyle="--")
        plt.plot(x, y_nord_east_below, color='orange', linestyle="--")
        plt.plot(x, y_nord_east_above, color='orange', linestyle="--")


        plt.xlim(0,2)
        plt.ylim(-1,1)

        plt.gcf().canvas.draw()
        plt.pause(0.01) 
    
    def display_prediction_backup_dynamic(self, south_east_points,south_west_points,x_goal, y_goal):
        # clear axes
        # self.ax.clear()
        plt.figure(0)
        plt.gca().clear()
        # creates scatter plot
        plt.scatter(south_west_points[:, 0], south_west_points[:, 1], color='red')
        plt.scatter(south_east_points[:, 0], south_east_points[:, 1], color='orange')
        
        # takes 3 values btw 0/2
        x = np.linspace(-1.5, 0, 3)

        # get proper slope, intercept for each value
        south_west_slope, south_west_intercept = self.prediction_instance.south_west_line.get_most_recent_coefficients()
        south_east_slope, south_east_intercept = self.prediction_instance.south_east_line.get_most_recent_coefficients()
        bisectrice_slope, bisectrice_intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # creates line
        y_nord_west = south_west_slope * x + south_west_intercept
        y_nord_east = south_east_slope * x + south_east_intercept

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
        y_south_west_below = bisectrice_slope * x + bisectrice_intercept + self.distance_from_bisectrice - self.tolerance_crop_distance
        y_south_west_above = bisectrice_slope * x + bisectrice_intercept + self.distance_from_bisectrice + self.tolerance_crop_distance
        y_south_east_below = bisectrice_slope * x + bisectrice_intercept - self.distance_from_bisectrice - self.tolerance_crop_distance
        y_south_east_above = bisectrice_slope * x + bisectrice_intercept - self.distance_from_bisectrice + self.tolerance_crop_distance
        
        # plot line
        plt.plot(x, y_south_west_below, color='red', linestyle="--")
        plt.plot(x, y_south_west_above, color='red', linestyle="--")
        plt.plot(x, y_south_east_below, color='orange', linestyle="--")
        plt.plot(x, y_south_east_above, color='orange', linestyle="--")


        plt.xlim(-2,0)
        plt.ylim(-1,1)

        plt.gcf().canvas.draw()
        plt.pause(0.01) 
   
    def in_row_navigation_backward(self, points_east, points_west, points_nord_east, points_nord_west, points_south_east, points_south_west):
        # get information about emptyness of regions
        is_east_empty = True if np.size(points_east) < self.min_point_direction else False
        is_west_empty = True if np.size(points_west) < self.min_point_direction else False

        if (is_east_empty or is_west_empty) and (self.is_in_row_navigation):
            print("END OF LINE")
            # to do -> publish goal point
            self.publish_end_pose()
            # to do -> reset values
            self.end_pose_queue.initialize_queue()
            # reset values
            self.prediction_instance.initialize_prediction()
            # reset quadrants
            self.initialization_quadrants()
            # reset value
            self.is_in_row_navigation = False
        
        else:
            # enough points to proceed
            # compute bisectrice
            self.prediction_instance.compute_bisectrice_coefficients_backward(points_nord_east,points_nord_west,points_south_east,points_south_west)
            # calculate goal point
            goal_pose, x, y = self.calculate_goal_point_backward()
            # publish goal pose
            self.publish_goal_pose(x, y)
            # to_do -> update queue with goal pose, add checks
            self.validate_end_pose(goal_pose, points_south_east, points_south_west)
            # display prediction
            self.display_prediction_backup_dynamic(points_south_east, points_south_west, x, y)
    
    def calculate_goal_point_backward(self):
        # get latest bisectrice coefficients
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # goal pose on robot frame
        theta = math.atan(slope)
        x_goal_robot = - self.distance_goal_point_forward * math.cos(theta)
        y_goal_robot = - self.distance_goal_point_forward * math.sin(theta)

        # alpha = math.atan(math.pi/2 - (2*math.pi - theta))
        # delta = math.atan(2*math.pi - theta)
        delta = - theta
        alpha = math.atan(math.pi/2 - delta)

        # project goal pose on actual bisectrice
        distance_robot_bis = intercept * math.cos(delta)
        x_proj_bis = distance_robot_bis * math.cos(alpha)
        y_proj_bis = distance_robot_bis * math.sin(alpha)
                     
        x_goal_pose = x_goal_robot + x_proj_bis
        y_goal_pose = y_goal_robot + y_proj_bis

        self.display_reasoning(x_goal_robot, y_goal_robot, x_proj_bis, y_proj_bis, x_goal_pose, y_goal_pose)
        # print(x_goal_robot, y_goal_robot, x_proj_bis, y_proj_bis, x_goal_pose, y_goal_pose)

        # goal as a pose in odom frame from velodyne 
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "velodyne2"

        # get x,y
        end_of_line_pose.pose.position.x = float(x_goal_pose)
        end_of_line_pose.pose.position.y = float(y_goal_pose)
        end_of_line_pose.pose.position.z = float(0.0)
        # get orientation
        quaternion = quaternion_from_euler(0, 0, theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        #transform from of the received odom to the current map
        transform = self._tf_buffer.lookup_transform('odom', 'velodyne2', end_of_line_pose.header.stamp, Duration(seconds=4, nanoseconds=0))
        return do_transform_pose_stamped(end_of_line_pose, transform), x_goal_pose, y_goal_pose


def main(args=None):
    rclpy.init(args=args)

    in_row_navigation = InRowNavigation()

    rclpy.spin(in_row_navigation)

    in_row_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()