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
# QUEUE
from collections import deque
# LINE
from sklearn.linear_model import RANSACRegressor
from numpy import float64

#################################
########## MOVING AVARAGE CLASS
##################################

class MovingAvarage():
    # assume type are exponentials
    def __init__(self, start, stop, base_exp, dimension_queue):
        self.weigths = []
        self.start = start
        self.stop = stop
        self.base_exp = base_exp
    
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
    

    def __repr__ (self):
        tmp_weights = ''
        weights = [tmp_weights+ str(item)+'' for item in self.weigths]
        return str(weights)

#################################
########## MOVING AVARAGE QUEUE
##################################

class MovingAvarageQueue():
    # assume type are exponentials
    def __init__(self, dimension_queue, start, stop, base_exp, reset_value):
        self.dimension_queue = dimension_queue
        self.queue = deque()
        self.weights = MovingAvarage(start=start, stop=stop, base_exp=base_exp, dimension_queue=dimension_queue)
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

    def __repr__ (self):
        tmp_queue = ''
        queue_str = [tmp_queue+ str(item)+'' for item in self.queue]
        return 'weights: ' + self.weights.__repr__() + ' queue '+ str(queue_str)
    

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
        self.queue.appendleft(0)

    def __repr__ (self):
        tmp_queue = ''
        queue_str = [tmp_queue+ str(item)+'' for item in self.queue]
        # queue_str = ''.join(self.queue)
        return 'queue' + str(queue_str) + ' '
    
    def __str__ (self):
        tmp_queue = ''
        queue_str = [tmp_queue+ str(item)+'' for item in self.queue]
        return 'queue' + str(queue_str) + ' '
    
    
#################################
#################### LINE
##################################    

class Line():
    # assume type are exponentials
    def __init__(self, dimension_queue, ma_flag, is_bisectrice, threshold, window_allowed_outdated_parameters, reset_value_intercept):

        if ma_flag == True:
            self.slope = MovingAvarageQueue(dimension_queue=dimension_queue, start=0, stop=4, base_exp=100, reset_value=0)
            self.intercept = MovingAvarageQueue(dimension_queue=dimension_queue,start=0, stop=4, base_exp=100, reset_value=reset_value_intercept)
        else:
            self.slope = CustomQueue(dimension_queue, reset_value=0)
            self.intercept = CustomQueue(dimension_queue, reset_value=reset_value_intercept)
        
        # ? needed flag
        self.is_bisectrice = is_bisectrice
        # threshold allowed btw two consecutive lines 
        self.threshold_btw_consecutive_lines_intercept= threshold
        self.threshold_btw_consecutive_lines_slope = 0.26 # 15 degree
        # counter outdated lines
        self.counter_outdated_consecutive_values = 0
        # consistency on filter in terms of number required points
        self.window_allowed_outdated_parameters = dimension_queue
        
        # previous status
        self.last_valid_slope = 0
        self.last_valid_intercept = 0

        # TO DO add consistency num_trials and min sample
        # min_samples=20, 
        self.ransac = RANSACRegressor(random_state=42)
    
    # initialization of queues
    def initialize_line(self):
        self.slope.initialize_queue()
        self.intercept.initialize_queue()

    # update line coefficient
    # slope, intercept = False -> update parameters last value
    def update_line_parameters(self, slope, intercept):
        self.slope.update_queue(slope) 
        self.intercept.update_queue(intercept) 
        self.last_valid_slope = slope
        self.last_valid_intercept = intercept
        self.counter_outdated_consecutive_values = 0
    
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
        # print(slope_time_t_min_1, slope, intercept_time_t_min_1, intercept)
        # check existance of data at time t and t-1
        if(self.threshold_btw_consecutive_lines_intercept != 0):
            if (self.counter_outdated_consecutive_values <= self.window_allowed_outdated_parameters):
                if slope_time_t_min_1 != False and intercept_time_t_min_1 != False:
                    # calculate threshold
                    if abs(math.atan(slope) - math.atan(slope_time_t_min_1)) > self.threshold_btw_consecutive_lines_slope or abs(intercept_time_t_min_1 - intercept) > self.threshold_btw_consecutive_lines_intercept:
                        # update_last_valid_coefficient
                        self.slope.update_queue(self.last_valid_slope) 
                        self.intercept.update_queue(self.last_valid_intercept) 
                        # update window
                        self.counter_outdated_consecutive_values = self.counter_outdated_consecutive_values +1
                    else:
                        # add just computed parameters
                        self.update_line_parameters(slope, intercept)

                else:    
                    # slope, intercept queue was empty 
                    self.update_line_parameters(slope, intercept)
            else:
                # too outdated value -> add new one
                self.update_line_parameters(slope, intercept)
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
        slope = self.slope.return_oldest_element()
        intercept = self.intercept.return_oldest_element()
        return slope, intercept

        
    def __repr__ (self):
        return 'Line(slope=' + self.slope.__repr__() + ' ,intercept=' + self.intercept.__repr__()  + ')'

#################################
#################### PREDICTION
##################################  
class Prediction():
    # assume type are exponentials
    def __init__(self, dimension_queue=5, ma_crop_flag=0, ma_navigation_flag=1, window_allowed_outdated_parameters=5,threshold_crop_line = 0.30, threshold_bisectrice_line = 0) :
        # dimension_queue -> n
        
        # save right/left cluster data
        # in case of rototranslation -> to be developed
        # self.right_cluster = costum_queue.CustomQueue(dimension_queue, window_allowed_outdated_parameters)
        # self.left_cluster = costum_queue.CustomQueue(dimension_queue, window_allowed_outdated_parameters)
        # save crop/bisectrice parameters
        self.navigation_line =  Line(dimension_queue, ma_navigation_flag, True, threshold_bisectrice_line, window_allowed_outdated_parameters, 0)
        self.nord_east_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, 0.375)
        self.nord_west_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, -0.375)
        self.south_east_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, 0.375)
        self.south_west_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, -0.375)
        
        # min_point_for_predition + threshold
        self.num_min_points_required_for_fitting = 20
        self.angle_added = 5
        self.angle_removed = 5
        self.dividend_factor_bisectrice_and_crop = 50    # 15 degree

        self.delta_angle_I = 15
        # TO BE MODIDIFIED
        self.delta_angle_II = 15
        self.delta_angle_III = 15
        self.delta_angle_IV = 15
    
    # initialization of lines
    def initialize_prediction(self):
        self.navigation_line.initialize_line()
        self.nord_east_line.initialize_line()
        self.nord_west_line.initialize_line() 
        self.south_east_line.initialize_line()
        self.south_west_line.initialize_line()

    def compute_bisectrice_coefficients_forward(self, nord_east_points, nord_west_points, south_east_points, south_west_points):
        # compute nord east crop line
        self.compute_crop_line_coefficients(self.nord_east_line, nord_east_points)
        
        # compute nors west crop line
        self.compute_crop_line_coefficients(self.nord_west_line, nord_west_points)

        # compute south east crop line
        self.compute_crop_line_coefficients(self.south_east_line, south_east_points)

        # compute south west crop line
        self.compute_crop_line_coefficients(self.south_west_line, south_west_points)

        nord_west_slope, nord_west_intercept = self.nord_west_line.get_most_recent_coefficients() 
        nord_east_slope, nord_east_intercept = self.nord_east_line.get_most_recent_coefficients() 

        # compute bisectrice
        medium_slope = (nord_west_slope+nord_east_slope) / 2
        medium_intercept = (nord_west_intercept+nord_east_intercept) / 2
        # add coeddificent to bisectrice
        self.navigation_line.update_line_parameters(medium_slope, medium_intercept)

    def compute_bisectrice_coefficients_backward(self, nord_east_points, nord_west_points, south_east_points, south_west_points):
        # compute nord east crop line
        self.compute_crop_line_coefficients(self.nord_east_line, nord_east_points)
        # compute nors west crop line
        self.compute_crop_line_coefficients(self.nord_west_line, nord_west_points) 
        # compute south east crop line
        self.compute_crop_line_coefficients(self.south_east_line, south_east_points)
        # compute south west crop line
        self.compute_crop_line_coefficients(self.south_west_line, south_west_points)

        south_west_slope, south_west_intercept = self.south_west_line.get_most_recent_coefficients() 
        south_east_slope, south_east_intercept = self.south_east_line.get_most_recent_coefficients()

        # compute bisectrice
        medium_slope = (south_west_slope+south_east_slope) / 2
        medium_intercept = (south_west_intercept+south_east_intercept) / 2

        # add coeddificent to bisectrice
        self.navigation_line.update_line_parameters(medium_slope, medium_intercept)

    def compute_crop_line_coefficients(self, local_line, points):
        # print("num points", len(points))
        # add delta threshold
        if(len(points)> self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope, intercept = local_line.fitting_line(points)
            # update the queue with the new slope and intercept
            local_line.update_line_parameters_checking_threshold(slope, intercept)
        else:
            slope_bisectrice_time_t_min_1, intercept_bisectrice_time_t_min_1 = self.navigation_line.get_most_recent_coefficients()

            # mantain last value -> use False, False
            # get latest value (if exist)
            # else reset value
            slope, intercept = local_line.get_most_recent_coefficients()
            local_line.update_line_parameters_checking_threshold(slope, intercept)

    def compute_bisectrice_coefficients_forward_old(self, nord_east_points, nord_west_points, south_east_points, south_west_points):
                                                                                        

        nord_west_slope, nord_west_intercept = self.nord_west_line.get_most_recent_coefficients() 
        nord_east_slope, nord_east_intercept = self.nord_east_line.get_most_recent_coefficients() 

        # compute bisectrice
        medium_slope = (nord_west_slope+nord_east_slope) / 2
        medium_intercept = (nord_west_intercept+nord_east_intercept) / 2

        # add coeddificent to bisectrice
        self.navigation_line.update_line_parameters(medium_slope, medium_intercept)
    
    def compute_crop_line_coefficients_west_robustness_bisectrice(self, local_line, points):
        # print("num points", len(points))
        # add delta threshold
        slope_bisectrice_time_t_min_1, intercept_bisectrice_time_t_min_1 = self.navigation_line.get_most_recent_coefficients()
        
        if(len(points)> self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope, intercept = local_line.fitting_line(points)
            # update the queue with the new slope and intercept
        else:
            # mantain last value -> use False, False
            # get latest value (if exist)
            # else reset value
            slope, intercept = local_line.get_most_recent_coefficients()
        

        atan_slope_crop = math.atan(slope) 
        atan_slope_bisectrice = math.atan(slope_bisectrice_time_t_min_1)
        degree_slope_crop = math.degrees(atan_slope_crop)
        degree_slope_bisectrice = math.degrees(atan_slope_bisectrice) 

        # print("SLOPE BEFORE", slope, slope_bisectrice_time_t_min_1)
        # print("DEGREE SLOPE BEFORE", degree_slope_crop, degree_slope_bisectrice)

        # first case
        # remove 1/dividend_factor_bisectrice_and_crop from angle
        if (slope > 0 and slope_bisectrice_time_t_min_1 < 0) and (degree_slope_crop - degree_slope_bisectrice > self.delta_angle_I):
            # angle consistency -> threshold
            if (degree_slope_crop - degree_slope_bisectrice) > self.delta_angle_I:
                print("WEST, I QUAD")
                # get angle width
                degree_new_slope_crop = (degree_slope_crop - degree_slope_bisectrice)/self.dividend_factor_bisectrice_and_crop
                degree_new_slope_crop = degree_slope_crop - degree_new_slope_crop
                atan_new_slope_crop = math.radians(degree_new_slope_crop)
                slope = math.tan(atan_new_slope_crop) 

        # third case
        # add 1/dividend_factor_bisectrice_and_crop from angle
        elif (slope < 0 and slope_bisectrice_time_t_min_1 > 0) and (degree_slope_bisectrice - degree_slope_crop > self.delta_angle_III):
            if (degree_slope_bisectrice - degree_slope_crop) > self.delta_angle_IV:
                print("WEST, IV QUAD")
                # get angle width
                degree_new_slope_crop = (degree_slope_bisectrice - degree_slope_crop)/self.dividend_factor_bisectrice_and_crop
                # remove from a negative -> greater distance from bisectrice
                degree_new_slope_crop = degree_slope_crop + degree_new_slope_crop
                atan_new_slope_crop = math.radians(degree_new_slope_crop)
                slope = math.tan(atan_new_slope_crop)
        
        # second A scenario
        elif(slope < 0 and slope_bisectrice_time_t_min_1 < 0):
            if(degree_slope_crop > degree_slope_bisectrice) and (degree_slope_crop - degree_slope_bisectrice > self.delta_angle_II):
                print("WEST, II QUAD")
                degree_new_slope_crop = abs(degree_slope_crop + degree_slope_bisectrice)/self.dividend_factor_bisectrice_and_crop
                degree_slope_crop = degree_slope_bisectrice + degree_new_slope_crop
            # else remain same
            # get angle width
            slope = math.radians(degree_slope_crop)
        
        # third A scenario
        elif(slope > 0 and slope_bisectrice_time_t_min_1 > 0):
            if(degree_slope_crop < degree_slope_bisectrice) and (degree_slope_bisectrice - degree_slope_crop > self.delta_angle_III):
                print("WEST, III QUAD")
                degree_new_slope_crop =(degree_slope_bisectrice - degree_slope_crop)/self.dividend_factor_bisectrice_and_crop
                degree_slope_crop = degree_slope_bisectrice + degree_new_slope_crop
            # else remain same
            # get angle width
            slope = math.radians(degree_slope_crop)
       
        # print("SLOPE AFTER", slope)

        local_line.update_line_parameters_checking_threshold(slope, intercept)
    
    def compute_crop_line_coefficients_east_robustness_bisectrice(self, local_line, points):
        # print("num points", len(points))
        # add delta threshold
        slope_bisectrice_time_t_min_1, intercept_bisectrice_time_t_min_1 = self.navigation_line.get_most_recent_coefficients()
        
        if(len(points)> self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope, intercept = local_line.fitting_line(points)
            # update the queue with the new slope and intercept
        else:
            # mantain last value -> use False, False
            # get latest value (if exist)
            # else reset value
            slope, intercept = local_line.get_most_recent_coefficients()
        

        atan_slope_crop = math.atan(slope) 
        atan_slope_bisectrice = math.atan(slope_bisectrice_time_t_min_1)
        degree_slope_crop = math.degrees(atan_slope_crop)
        degree_slope_bisectrice = math.degrees(atan_slope_bisectrice) 

        # print("SLOPE BEFORE", slope, slope_bisectrice_time_t_min_1)
        # print("DEGREE SLOPE BEFORE", degree_slope_crop, degree_slope_bisectrice)

        # third case
        # remove 1/dividend_factor_bisectrice_and_crop from angle
        if (slope > 0 and slope_bisectrice_time_t_min_1 < 0) and (degree_slope_crop - degree_slope_bisectrice > self.delta_angle_III):
            print("EAST, III QUAD")
            # get angle width
            degree_new_slope_crop = (degree_slope_crop - degree_slope_bisectrice)/self.dividend_factor_bisectrice_and_crop
            # distanciate from bisectrice
            degree_new_slope_crop = degree_slope_crop - degree_new_slope_crop
            atan_new_slope_crop = math.radians(degree_new_slope_crop)
            slope = math.tan(atan_new_slope_crop) 

        # first
        # add 1/dividend_factor_bisectrice_and_crop from angle
        elif (slope < 0 and slope_bisectrice_time_t_min_1 > 0) and (degree_slope_bisectrice - degree_slope_crop > self.delta_angle_I):
            print("EAST, I QUAD")
            # get angle width
            degree_new_slope_crop = (degree_slope_bisectrice - degree_slope_crop)/self.dividend_factor_bisectrice_and_crop
            # add angle -> near bisectrice
            degree_new_slope_crop = degree_slope_crop + degree_new_slope_crop
            atan_new_slope_crop = math.radians(degree_new_slope_crop)
            slope = math.tan(atan_new_slope_crop)
        
        # second B scenario
        elif(slope < 0 and slope_bisectrice_time_t_min_1 < 0):
            if(degree_slope_crop > degree_slope_bisectrice) and (degree_slope_crop - degree_slope_bisectrice > self.delta_angle_II):
                print("EAST, II QUAD")
                degree_new_slope_crop = abs(degree_slope_crop + degree_slope_bisectrice)/self.dividend_factor_bisectrice_and_crop
                # distantiate
                degree_slope_crop = degree_slope_bisectrice + degree_new_slope_crop
            # else remain same
            # get angle width
            slope = math.radians(degree_slope_crop)
        
        # forth B scenario
        elif(slope > 0 and slope_bisectrice_time_t_min_1 > 0):
            if(degree_slope_crop > degree_slope_bisectrice) and (degree_slope_crop - degree_slope_bisectrice > self.delta_angle_IV):
                print("EAST, IV QUAD")
                degree_new_slope_crop = (degree_slope_bisectrice - degree_slope_crop)/self.dividend_factor_bisectrice_and_crop
                # distantiate
                degree_slope_crop = degree_slope_bisectrice - degree_new_slope_crop
            # else remain same
            # get angle width
            slope = math.radians(degree_slope_crop)
       
        
        

        # print("SLOPE AFTER", slope)

        local_line.update_line_parameters_checking_threshold(slope, intercept)
       
    def __repr__ (self):
        return 'Line(slope=' + self.slope.__repr__() + ' ,intercept=' + self.intercept.__repr__()  + ')'
    

class NearbyGoalPoints(Node):
    # assume type are exponentials
    def __init__(self, dimension_queue=5, ma_crop_flag=0, ma_navigation_flag=1, num_skip = 5,threshold_crop_line = 0.5, threshold_bisectrice_line = 0.5, min_num_required_points=7, num_points_traverse_line=30) :
        super().__init__('in_row_navigation')
        # topics where the data are published
        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.goal_pose_pub = self.create_publisher(Float64MultiArray, '/goal_position', 1)
        self.goal_pose_pub # prevent unused variable warning
        self.end_of_line_pose_topic = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)
        self.end_of_line_pose_topic # prevent unused variable warning

        

        self.goal_pose_pub # prevent unused variable warning

        # minimum number points
        self.min_num_required_points = min_num_required_points
        self.min_num_build_quadrants = min_num_required_points * 2 - 1

        # parameters needed during perfomance calculation
        self.end_computation = -1
        self.start_computation = -1

        # prefiltering
        # distance_from_goal +/- distance_from_bisectrice
        self.distance_from_bisectrice = 0.35
        self.tolerance_crop_distance = 0.15
        self.robot_width = 0.4
        self.line_width_RANSAC = self.line_width + 2* self.tolerance_crop_distance

        self.line_width = 0.7
        self.area = np.array([2*(self.line_width_RANSAC) + self.robot_width, 2.6]) 
        
        self.prefiltering_threshold = self.line_width
        self.filtering_from_crop = self.line_width

        # Prediction obj

        # add parameters
        self.prediction_instance_east = Prediction()
        # add parameters
        self.prediction_instance_west = Prediction()

        # Display Ransac
        self.fig, self.ax = plt.subplots()

        # num_accepted_points
        self.counter_start_line = 0
        self.window_skipped_start_line = num_skip

        # old slope, intercept, clusters
        self.last_slope = 0
        self.last_intercept = 0
        self.last_cluster_east = 0
        self.last_cluster_left = 0

        # distance_goal_point
        self.distance_goal_point_forward = self.area[1]/2
        self.distance_goal_point_backward = self.area[1]/2
        
        # needed for goal publication
        self.is_goal_published = False
        self.distance_goal_end_of_line_forward = 0.5
        self.distance_goal_end_of_line_backward = self.area[1]/2

        # modify is_begin/end line
        self.is_begin_line_forward = False
        self.is_end_line_backward = False

        # move forward
        self.moving_forward = True
  
    def scan_callback(self, scan_msg):
        self.start_computation = time.perf_counter()
        # points_nord_east, points_nord_west, points_south_east, points_south_west
        points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian(scan_msg)

        # print(points_2d)
        # if going forward/backward 
        print(len(points_nord_east), len(points_nord_west), len(points_south_east), len(points_south_west))

        if(self.moving_forward):
            self.in_row_navigation_forward(points_nord_east, points_nord_west, points_south_east, points_south_west)
        else:
            self.in_row_navigation_backward(points_nord_east, points_nord_west, points_south_east, points_south_west)
           
    def in_row_navigation_forward(self, points_nord_east, points_nord_west, points_south_east, points_south_west):

        # get information about emptyness of regions
        is_nord_east_empty = True if np.size(points_nord_east) < self.min_num_required_points else False
        is_nord_west_empty = True if np.size(points_nord_west) < self.min_num_required_points else False
        is_south_east_empty = True if np.size(points_south_east) < self.min_num_required_points else False
        is_south_west_empty = True if np.size(points_south_west) < self.min_num_required_points else False

        if is_nord_east_empty & is_nord_west_empty & is_south_east_empty & is_south_west_empty:
            if (self.is_goal_published==True):
                # publish last goal pose
                x, y, theta = self.calculate_goal_point_forward()
                self.publish_end_of_line_pose(x, y, theta)
                # reset_is_goal_published
                self.is_goal_published = False
                return

        elif (is_nord_east_empty == False | is_nord_west_empty == False) & is_south_east_empty & is_south_west_empty:
                self.is_begin_line_forward = True
                self.is_end_line_backward = False
                self.is_goal_published=True

        # compute bisectrice
        self.prediction_instance.compute_bisectrice_coefficients_forward(points_nord_east,points_nord_west,points_south_east,points_south_west)

        # invoke calculate_goal_position
        x, y, theta = self.calculate_goal_point_forward()

        # publish goal pose
        self.publish_goal_pose(x, y, theta)

        # display 
        self.display_prediction_forward(points_nord_east,points_nord_west, x, y)
    
    def in_row_navigation_backward(self, points_nord_east, points_nord_west, points_south_east, points_south_west):
        # compute bisectrice
        self.prediction_instance.compute_bisectrice_coefficients_backward(points_nord_east, points_nord_west, points_south_east, points_south_west)

        # invoke calculate_goal_position
        x, y, theta = self.calculate_goal_point_forward()

        # publish goal pose
        self.publish_goal_pose(x, y, theta)

        # display 
        self.display_prediction_backward(points_south_east,points_south_west,  x, y)

    def laser_scan_to_cartesian_old(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) 

        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))

        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]

       
        # if number of points 
        if(np.size(points_filtered) < self.min_num_build_quadrants):
            # EAST REGION
            y_west = np.where((0 <= -(self.line_width_RANSAC+self.robot_width/2))&(y<= -(self.line_width_RANSAC/2 + self.robot_width/2)),y, -1)
            y_east = np.where(((-(self.line_width_RANSAC/2+self.robot_width/2) <= y)&(y <= -self.robot_width/2)),y, -1)

            points_nord_east = np.vstack((x_nord, y_east)).T
            points_nord_west = np.vstack((x_nord, y_west)).T
            points_south_east = np.vstack((x_south, y_east)).T
            points_south_west = np.vstack((x_south, y_west)).T

            points_nord_east = points_nord_east[y_east != -1]
            points_nord_west = points_nord_west[y_west != -1]
            points_south_east = points_south_east[y_east != -1]
            points_south_west = points_south_west[y_west != -1]
            
            # WEST REGION
            y_east = np.where((self.robot_width/2 <= y)&(y<=(self.line_width_RANSAC/2 + self.robot_width/2)),y, -1)
            y_west = np.where((((self.line_width_RANSAC/2+self.robot_width/2) <= y)&(y <= self.robot_width)),y, -1)

            points_nord_east = np.vstack((x_nord, y_east)).T
            points_nord_west = np.vstack((x_nord, y_west)).T
            points_south_east = np.vstack((x_south, y_east)).T
            points_south_west = np.vstack((x_south, y_west)).T

            points_nord_east = points_nord_east[y_east != -1]
            points_nord_west = points_nord_west[y_west != -1]
            points_south_east = points_south_east[y_east != -1]
            points_south_west = points_south_west[y_west != -1]

            # todo south
            

        else:
            

            slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

            if(slope==0 and intercept==0):
                # EAST REGION
                y_west = np.where((0 <= -(self.line_width_RANSAC+self.robot_width/2))&(y<= -(self.line_width_RANSAC/2 + self.robot_width/2)),y, -1)
                y_east = np.where(((-(self.line_width_RANSAC/2+self.robot_width/2) <= y)&(y <= -self.robot_width/2)),y, -1)

                points_nord_east = np.vstack((x_nord, y_east)).T
                points_nord_west = np.vstack((x_nord, y_west)).T
                points_south_east = np.vstack((x_south, y_east)).T
                points_south_west = np.vstack((x_south, y_west)).T

                east_region_points_nord_east = points_nord_east[y_east != -1]
                east_region_points_nord_west = points_nord_west[y_west != -1]
                east_region_points_south_east = points_south_east[y_east != -1]
                east_region_points_south_west = points_south_west[y_west != -1]
            else:
                # distance_from_goal +/- distance_from_bisectrice
                # self.distance_from_bisectrice = 0.3
                # self.tolerance_crop_distance = 0.5
                x_nord = np.where(((0 <= x) & (x <= self.area[1]/2)),x, -1)
                x_south = np.where(((-self.area[1]/2 <= x)&(x <= 0)), x, -1)
                intercept_n = np.full_like(x_nord, intercept)
                intercept_s = np.full_like(x_south, intercept) 
                intercept_n_t_p = np.full_like(x_nord, self.filtering_from_crop)
                intercept_n_t_n = np.full_like(x_nord, -self.filtering_from_crop)
                intercept_s_t_p = np.full_like(x_south, self.filtering_from_crop)
                intercept_s_t_n = np.full_like(x_south, -self.filtering_from_crop)

                # y east -> <=0
                # y_west -> >0

                y_nord_west = np.where((y > np.add(slope*x_nord, intercept_n))&(y < np.add(np.add(slope*x_nord, intercept_n),intercept_n_t_p)),y, -1)
                y_nord_east = np.where((y < np.add(slope*x_nord, intercept_n))&(y > np.add(np.add(slope*x_nord,intercept_n),intercept_n_t_n)),y, -1)
                y_south_east = np.where((y < np.add(slope*x_south, intercept_s))&(y > np.add(np.add(slope*x_south,intercept_s),intercept_s_t_n)),y, -1)
                y_south_west = np.where((y > np.add(slope*x_south, intercept_s))&(y < np.add(np.add(slope*x_south,intercept_s),intercept_s_t_p)),y, -1)

                # print(y_south_east, y_nord_east)
                # print(mask_nord_west, mask_nord_east)
                # mask_nord_east = ((point_nord_east[:, 1] < slope*point_nord_east[:, 0]+ intercept) & (point_nord_east[:, 1] > ((slope*point_nord_east[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_nord_west = ((point_nord_west[:, 1] > slope*point_nord_west[:, 0]+ intercept) & (point_nord_west[:, 1] < ((slope*point_nord_west[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_south_east = ((point_south_east[:, 1] < slope*point_south_east[:, 0]+ intercept) & (point_south_east[:, 1] > ((slope*point_south_east[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_south_west = ((point_south_west[:, 1] > slope*point_south_west[:, 0]+ intercept) & (point_south_west[:, 1] < ((slope*point_south_west[:, 0]+ intercept) + self.prefiltering_threshold)))
                
                points_nord_east = np.vstack((x_nord, y_nord_east)).T
                points_nord_west = np.vstack((x_nord, y_nord_west)).T
                points_south_east = np.vstack((x_south, y_south_east)).T
                points_south_west = np.vstack((x_south, y_south_west)).T

                points_nord_east = points_nord_east[y_nord_east != -1]
                points_nord_west = points_nord_west[y_nord_west != -1]
                points_south_east = points_south_east[y_south_east != -1]
                points_south_west = points_south_west[y_south_west != -1]
            
            slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

            if(slope==0 and intercept==0):
                # WEST REGION
                y_east = np.where((self.robot_width/2 <= y)&(y<=(self.line_width_RANSAC/2 + self.robot_width/2)),y, -1)
                y_west = np.where((((self.line_width_RANSAC/2+self.robot_width/2) <= y)&(y <= self.robot_width)),y, -1)

                points_nord_east = np.vstack((x_nord, y_east)).T
                points_nord_west = np.vstack((x_nord, y_west)).T
                points_south_east = np.vstack((x_south, y_east)).T
                points_south_west = np.vstack((x_south, y_west)).T

                west_region_points_nord_east = points_nord_east[y_east != -1]
                west_region_points_nord_west = points_nord_west[y_west != -1]
                west_region_points_south_east = points_south_east[y_east != -1]
                west_region_points_south_west = points_south_west[y_west != -1]
            else:
                # distance_from_goal +/- distance_from_bisectrice
                # self.distance_from_bisectrice = 0.3
                # self.tolerance_crop_distance = 0.5
                x_nord = np.where(((0 <= x) & (x <= self.area[1]/2)),x, -1)
                x_south = np.where(((-self.area[1]/2 <= x)&(x <= 0)), x, -1)
                intercept_n = np.full_like(x_nord, intercept)
                intercept_s = np.full_like(x_south, intercept) 
                intercept_n_t_p = np.full_like(x_nord, self.filtering_from_crop)
                intercept_n_t_n = np.full_like(x_nord, -self.filtering_from_crop)
                intercept_s_t_p = np.full_like(x_south, self.filtering_from_crop)
                intercept_s_t_n = np.full_like(x_south, -self.filtering_from_crop)

                # y east -> <=0
                # y_west -> >0

                y_nord_west = np.where((y > np.add(slope*x_nord, intercept_n))&(y < np.add(np.add(slope*x_nord, intercept_n),intercept_n_t_p)),y, -1)
                y_nord_east = np.where((y < np.add(slope*x_nord, intercept_n))&(y > np.add(np.add(slope*x_nord,intercept_n),intercept_n_t_n)),y, -1)
                y_south_east = np.where((y < np.add(slope*x_south, intercept_s))&(y > np.add(np.add(slope*x_south,intercept_s),intercept_s_t_n)),y, -1)
                y_south_west = np.where((y > np.add(slope*x_south, intercept_s))&(y < np.add(np.add(slope*x_south,intercept_s),intercept_s_t_p)),y, -1)

                # print(y_south_east, y_nord_east)
                # print(mask_nord_west, mask_nord_east)
                # mask_nord_east = ((point_nord_east[:, 1] < slope*point_nord_east[:, 0]+ intercept) & (point_nord_east[:, 1] > ((slope*point_nord_east[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_nord_west = ((point_nord_west[:, 1] > slope*point_nord_west[:, 0]+ intercept) & (point_nord_west[:, 1] < ((slope*point_nord_west[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_south_east = ((point_south_east[:, 1] < slope*point_south_east[:, 0]+ intercept) & (point_south_east[:, 1] > ((slope*point_south_east[:, 0]+ intercept) + self.prefiltering_threshold)))
                # mask_south_west = ((point_south_west[:, 1] > slope*point_south_west[:, 0]+ intercept) & (point_south_west[:, 1] < ((slope*point_south_west[:, 0]+ intercept) + self.prefiltering_threshold)))
                
                points_nord_east = np.vstack((x_nord, y_nord_east)).T
                points_nord_west = np.vstack((x_nord, y_nord_west)).T
                points_south_east = np.vstack((x_south, y_south_east)).T
                points_south_west = np.vstack((x_south, y_south_west)).T

                points_nord_east = points_nord_east[y_nord_east != -1]
                points_nord_west = points_nord_west[y_nord_west != -1]
                points_south_east = points_south_east[y_south_east != -1]
                points_south_west = points_south_west[y_south_west != -1]
        

    
        return points_nord_east, points_nord_west, points_south_east, points_south_west

    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/len(ranges)) 

        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))

        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]

       
        # if number of points 
        if(np.size(points_filtered) < self.min_num_build_quadrants):
            x_nord = np.where(((0 <= x) & (x <= self.area[1]/2)),x, -1)
            x_south = np.where(((-self.area[1]/2 <= x)&(x <= 0)), x, -1)

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
                x_nord = np.where(((0 <= x) & (x <= self.area[1]/2)),x, -1)
                x_south = np.where(((-self.area[1]/2 <= x)&(x <= 0)), x, -1)
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
                
                threshold_nord = np.full_like(x, intercept*slope + self.area[1]/2)
                threshold_south = np.full_like(x, intercept*slope - self.area[1]/2)

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

    def calculate_goal_point_forward(self):
        # takes the last m/q value of bisectrice
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
        
        # calculate goal_points
        tmp_result = pow(slope,2)* pow(intercept,2) - (1+ pow(slope,2))*(pow(intercept,2)-pow(self.distance_goal_point_forward,2))
        
        # safety if-else statement
        if(tmp_result >=0):
            x_1 = (-slope*intercept + math.sqrt(tmp_result))/(1+pow(slope,2))
            x_2 = (-slope*intercept - math.sqrt(tmp_result))/(1+pow(slope,2))
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
        goal_pose.data = [float(x), float(y), float(theta)]
        # publish goal pose
        self.goal_pose_pub.publish(goal_pose)

    def calculate_goal_point_backward(self):
        # takes the last m/q value of bisectrice
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
        
        # calculate goal_points
        tmp_result = pow(slope,2)* pow(intercept,2) - (1+ pow(slope,2))*(pow(intercept,2)-pow(self.distance_goal_point,2))
        
        # safety if-else statement
        if(tmp_result >=0):
            x_1 = (-slope*intercept + math.sqrt(tmp_result))/(1+pow(slope,2))
            x_2 = (-slope*intercept - math.sqrt(tmp_result))/(1+pow(slope,2))
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

     # update bool value
    
    def callback_update_bool(self, msg):
        self.publish_goal_position = True

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

        if(end_of_line_pose.pose.position.x == 1) and (end_of_line_pose.pose.orientation.w == 1):
            return
        else:
            # publish goal pose
            self.end_of_line_pose_topic.publish(end_of_line_pose)

    def display_prediction_forward(self, nord_east_points,nord_west_points,x_goal, y_goal):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        # print(np.size(nord_west_points), np.size(nord_east_points))
        plt.scatter(nord_west_points[:, 0], nord_west_points[:, 1], color='red')
        plt.scatter(nord_east_points[:, 0], nord_east_points[:, 1], color='orange')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # get proper slope, intercept for each value
        nord_west_slope, nord_west_intercept = self.prediction_instance.nord_west_line.get_most_recent_coefficients()
        nord_east_slope, nord_east_intercept = self.prediction_instance.nord_east_line.get_most_recent_coefficients()
        bisectrice_slope, bisectrice_intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # print(nord_west_slope, nord_west_intercept ,nord_east_slope, nord_east_intercept)

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

        plt.xlim(0,3)
        plt.ylim(-3,3)

        self.fig.canvas.draw()
        plt.pause(0.01)
    
    def display_prediction_backward(self, south_east_points,south_west_points, x_goal, y_goal):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(south_west_points[:, 0], south_west_points[:, 1], color='red')
        plt.scatter(south_east_points[:, 0], south_east_points[:, 1], color='orange')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # get proper slope, intercept for each value
        south_west_slope, south_west_intercept = self.prediction_instance.south_east_line.get_most_recent_coefficients()
        south_east_slope, south_east_intercept = self.prediction_instance.south_west_line.get_most_recent_coefficients()
        bisectrice_slope, bisectrice_intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # creates line
        y_south_west = south_west_slope * x + south_west_intercept
        y_south_east = south_east_slope * x + south_east_intercept

        # plot line
        plt.plot(x, y_south_west, color='red')
        plt.plot(x, y_south_east, color='orange')

        # robot line
        y = bisectrice_slope * x + bisectrice_intercept

        # bisectrice
        plt.plot(x, y, color='green')

        # goal point 
        plt.plot(x_goal, y_goal, marker="o", markersize=10, markeredgecolor="green", markerfacecolor="green")

        plt.xlim(0,3)
        plt.ylim(-2,2)

        self.fig.canvas.draw()
        plt.pause(0.01)

    # def measure_performance(self):
    
    # def __repr__ (self):
        # return 'Line(slope=' + self.slope.__repr__() + ' ,intercept=' + self.intercept.__repr__()  + ')'
    
     
def main(args=None):
    rclpy.init(args=args)

    in_row_navigation = NearbyGoalPoints()

    rclpy.spin(in_row_navigation)

    in_row_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


