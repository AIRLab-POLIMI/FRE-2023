import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray,Bool
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import time
import numpy as np
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
string_from_folder = 'src/FRE-2023'
absolute_path = os.path.abspath(string_from_folder+'/grasslammer2_nav_py/grasslammer2_nav_py/in_row_navigation_config/cornaredo.json')
# print(absolute_path)
config_file = open(absolute_path, 'r')
# dict_config = config_file.read()
config_json = json.loads(config_file.read())

#################################
########## MOVING AVARAGE CLASS
##################################

class MovingAvarage():
    # assume type are exponentials
    def __init__(self, start, stop, base_exp, dimension_queue):
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
    def __init__(self, dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope):

        if ma_avarage_flag == True:
            self.slope = MovingAvarageQueue(dimension_queue=dimension_queue, start=0, stop=4, base_exp=100, reset_value=reset_value_slope)
            self.intercept = MovingAvarageQueue(dimension_queue=dimension_queue,start=0, stop=4, base_exp=100, reset_value=reset_value_intercept)
        else:
            self.slope = CustomQueue(dimension_queue, reset_value=reset_value_slope)
            self.intercept = CustomQueue(dimension_queue, reset_value=reset_value_intercept)
        
        # ? needed flag
        self.is_bisectrice = is_bisectrice
        # threshold allowed btw two consecutive lines 
        self.threshold_btw_consecutive_lines_intercept = threshold_line_intercept
        self.threshold_btw_consecutive_lines_slope = threshold_line_slope# 15 degree
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
    def __init__(self, dimension_queue=10, ma_crop_flag=0, ma_navigation_flag=1, window_allowed_outdated_parameters=10,threshold_crop_line = 0.30, threshold_bisectrice_line = 0) :
        # dimension_queue -> n
        
        # save right/left cluster data
        # in case of rototranslation -> to be developed
        # self.right_cluster = costum_queue.CustomQueue(dimension_queue, window_allowed_outdated_parameters)
        # self.left_cluster = costum_queue.CustomQueue(dimension_queue, window_allowed_outdated_parameters)
        # save crop/bisectrice parameters

        # self.navigation_line =  Line(dimension_queue, ma_navigation_flag, True, threshold_bisectrice_line, window_allowed_outdated_parameters, 0)
        # self.nord_east_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, 0.375)
        # self.nord_west_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, -0.375)
        # self.south_east_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, 0.375)
        # self.south_west_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, -0.375)
        
        # nord east
        # self, dimension_queue, ma_flag, is_bisectrice, threthreshold_bisectrice_lineshold, window_allowed_outdated_parameters, reset_value_intercept
        dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope = self.get_parameters_from_config_file('line_nord_east')
        self.nord_east_line = Line(dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope)
        
        dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope = self.get_parameters_from_config_file('line_nord_west')
        self.nord_west_line = Line(dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope)
        
        dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope = self.get_parameters_from_config_file('line_south_east')
        self.south_east_line = Line(dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope)
        
        dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope = self.get_parameters_from_config_file('line_south_west')
        self.south_west_line = Line(dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope)
        
        dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope = self.get_parameters_from_config_file('line_navigation')
        self.navigation_line =  Line(dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope)
        
        

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
       
    def get_parameters_from_config_file(self, quadrant):
        dimension_queue = config_json['prediction']['lines'][quadrant]['dimension_queue']
        ma_avarage_flag = config_json['prediction']['lines'][quadrant]['ma_avarage_flag']
        is_bisectrice = config_json['prediction']['lines'][quadrant]['is_bisectrice']
        threshold_line_intercept = config_json['prediction']['lines'][quadrant]['threshold_line_intercept']
        threshold_line_slope = config_json['prediction']['lines'][quadrant]['threshold_line_slope']
        reset_value_intercept = config_json['prediction']['lines'][quadrant]['reset_value_intercept']
        reset_value_slope = config_json['prediction']['lines'][quadrant]['reset_value_slope']
        return dimension_queue, ma_avarage_flag, is_bisectrice, threshold_line_intercept, threshold_line_slope, reset_value_intercept, reset_value_slope
    
    def __repr__ (self):
        return 'Line(slope=' + self.slope.__repr__() + ' ,intercept=' + self.intercept.__repr__()  + ')'
    
class InRowNavigation(Node):
    # assume type are exponentials
    def __init__(self, dimension_queue=5, ma_crop_flag=0, ma_navigation_flag=1, num_skip = 5,threshold_crop_line = 0.5, threshold_bisectrice_line = 0.5, min_num_required_points=20, num_points_traverse_line=30) :
        super().__init__('in_row_navigation')
        # topics where the data are published
        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.goal_pose_pub = self.create_publisher(Float64MultiArray, '/goal_position', 1)
        self.goal_pose_pub # prevent unused variable warning
        self.end_of_line_pose_topic = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)
        self.end_of_line_pose_topic # prevent unused variable warning
        self.sub_turning_status = self.create_subscription(Bool, '/end_of_turning', self.callback_update_bool, 1)

        area, dimension_queue, min_number_required_points_per_quadrants, line_width, tolerance_crop_distance_filtering, distance_goal_point_forward, distance_goal_point_backward, nord_threshold, south_threshold = self.get_parameters_from_config_file()
        self.area = area
        self.nord_treshold = self.area[1]/2
        self.south_treshold = self.area[1]/2

        self.goal_pose_pub # prevent unused variable warning

        # minimum number points entire region
        self.min_num_required_points = min_number_required_points_per_quadrants
        # proportion of 15 point
        self.min_num_build_quadrants = self.min_num_required_points * 2 - 1

        # parameters needed during perfomance calculation
        self.end_computation = -1
        self.start_computation = -1

        # prefiltering
        self.line_width = line_width
        # distance_from_goal +/- distance_from_bisectrice
        self.distance_from_bisectrice = self.line_width/2
        self.tolerance_crop_distance = tolerance_crop_distance_filtering
        self.prefiltering_threshold = self.line_width
        self.filtering_from_crop = self.line_width

        # Prediction obj
        # add parameters
        self.prediction_instance = Prediction()

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
        self.distance_goal_point_forward = (self.area[1]/2)/2 # 0.4
        self.distance_goal_point_backward = (-self.area[1]/2)/2 # -0.4
        
        # needed for goal publication
        self.is_goal_published = True
        self.distance_goal_end_of_line_forward = 0.1
        self.distance_goal_end_of_line_backward =  -0.1

        # TODO backward
        self.greatest_point_south_west = [-10,-10]
        self.greatest_point_south_east = [-10,-10]
        self.greatest_point_nord_west = [0,0]
        self.greatest_point_nord_east = [0,0]
        self.previous_forward_goal = ''
        self.previous_backward_goal = ''


        #TODO threshold to plot quadrants
        # nord east quadrants
        self.x_nord_east_nord = ''
        self.x_nord_east_south = ''
        self.y_nord_east_east = ''
        self.y_nord_east_west = ''
         # nord west quadrants
        self.x_nord_west_nord = ''
        self.x_nord_west_south = ''
        self.y_nord_west_east = ''
        self.y_nord_west_west = ''
         # south east quadrants
        self.x_south_east_nord = ''
        self.x_south_east_south = ''
        self.y_south_east_east = ''
        self.y_south_east_west = ''
         # south west quadrants
        self.x_south_west_nord = ''
        self.x_south_west_south = ''
        self.y_south_west_east = ''
        self.y_south_west_west = ''


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
        points_nord_east, points_nord_west, points_south_east, points_south_west = self.laser_scan_to_cartesian(scan_msg)

        # print(points_2d)
        # if going forward/backward 

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

        # print(len(points_nord_east), len(points_nord_west), len(points_south_east), len(points_south_west))
        # needed to save the last point to calculate the perpendicular angle
        #if is_nord_east_empty & is_nord_west_empty & (is_south_east_empty == False or is_south_west_empty == False):
        # IDEA -> anticipate it beforehand
        if (is_nord_east_empty == True & is_nord_west_empty == True) & (is_south_east_empty == False & is_south_west_empty==False) & self.is_goal_published==True:
            tmp_south_east = points_south_east.max(axis=0)
            tmp_south_west = points_south_west.max(axis=0)
            # get the highest values
            if(tmp_south_east[0] > self.greatest_point_south_east[0]):
                self.greatest_point_south_east = tmp_south_east
            if(tmp_south_west[0] > self.greatest_point_south_west[0]):
                self.greatest_point_south_west = tmp_south_west
            print(self.greatest_point_south_east, self.greatest_point_south_west)

            # self.greatest_point_nord_east = points_nord_east.max(axis=0)
            # self.greatest_point_nord_west = points_nord_west.max(axis=0)
        
        if is_nord_east_empty & is_nord_west_empty & is_south_east_empty & is_south_west_empty:
            if (self.is_goal_published==True):
                # self.distance_goal_point_forward = 0.1
                # publish last goal pose    
                # x, y, theta = self.calculate_goal_point_forward()
                self.publish_end_of_line_pose_perpendicular_crop()
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
        # compute bisectrice
        self.prediction_instance.compute_bisectrice_coefficients_backward(points_nord_east, points_nord_west, points_south_east, points_south_west)

        # invoke calculate_goal_position
        x, y, theta = self.calculate_goal_point_forward()

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
        if(np.size(points) < self.min_num_build_quadrants):
            x_nord = np.where(((0 <= x) & (x <= self.nord_treshold)),x, np.inf)
            x_south = np.where(((-self.south_treshold <= x)&(x <= 0)), x, np.inf)

            y_west = np.where(((0 <= y)&(y<= self.area[0]/2)),y, np.inf)
            y_east = np.where(((-self.area[0]/2 <= y)&(y <= 0)),y, np.inf)

            points_nord_east = np.vstack((x_nord, y_east)).T
            points_nord_west = np.vstack((x_nord, y_west)).T
            points_south_east = np.vstack((x_south, y_east)).T
            points_south_west = np.vstack((x_south, y_west)).T

            points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
            points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]
            points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
            points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]


            # nord east
            self.y_nord_east_east, self.y_nord_east_west, self.x_nord_east_nord, self.x_nord_east_south = -self.area[0]/2, 0,  self.nord_treshold,0
            # nord west
            self.y_nord_west_east, self.y_nord_west_west, self.x_nord_west_nord, self.x_nord_west_south = 0, self.area[0]/2, self.nord_treshold,0 
            # south east
            self.y_south_east_east, self.y_south_east_west, self.x_south_east_nord, self.x_south_east_south = -self.area[0]/2, 0, 0 , self.south_treshold
            # south wes
            self.y_south_west_east, self.y_south_west_west, self.x_south_west_nord, self.x_south_west_south = 0, self.area[0]/2, 0 , self.south_treshold


        else:
            
            slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

            if(slope==0 and intercept==0):
                x_nord = np.where(((0 <= x) & (x <= self.nord_treshold)),x, np.inf)
                x_south = np.where(((-self.south_treshold <= x)&(x <= 0)), x, np.inf)
                y_west = np.where(((0 <= y)&(y<= self.line_width)),y, np.inf)
                y_east = np.where(((-self.line_width <= y)&(y <= 0)),y, np.inf)

                points_nord_east = np.vstack((x_nord, y_east)).T
                points_nord_west = np.vstack((x_nord, y_west)).T
                points_south_east = np.vstack((x_south, y_east)).T
                points_south_west = np.vstack((x_south, y_west)).T

                points_nord_east = points_nord_east[~np.isinf(points_nord_east).any(axis=1)]
                points_nord_west = points_nord_west[~np.isinf(points_nord_west).any(axis=1)]
                points_south_east = points_south_east[~np.isinf(points_south_east).any(axis=1)]
                points_south_west = points_south_west[~np.isinf(points_south_west).any(axis=1)]

               
                # nord east
                self.y_nord_east_east, self.y_nord_east_west, self.x_nord_east_nord, self.x_nord_east_south = -self.area[0]/2, 0,  self.nord_treshold,0
                # nord west
                self.y_nord_west_east, self.y_nord_west_west, self.x_nord_west_nord, self.x_nord_west_south = 0, self.area[0]/2, self.nord_treshold,0 
                # south east
                self.y_south_east_east, self.y_south_east_west, self.x_south_east_nord, self.x_south_east_south = -self.area[0]/2, 0, 0 , self.south_treshold
                # south wes
                self.y_south_west_east, self.y_south_west_west, self.x_south_west_nord, self.x_south_west_south = 0, self.area[0]/2, 0 , self.south_treshold


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

                y_west = np.where((y > np.add(slope*x, threshold_west_below))&(y < np.add(slope*x, threshold_west_above)),y, np.inf)
                y_east = np.where((y > np.add(slope*x, threshold_east_below))&(y < np.add(slope*x,threshold_east_above)),y, np.inf)

                
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
                # x><0 KO
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
                

                # drawing
                y_east_baseline = slope* (-self.line_width/2)
                y_west_baseline = slope* (self.line_width/2) 
                y_east_east_threshold = y_east_baseline + distance_east_above
                y_east_west_threshold = y_east_baseline + distance_east_below
                y_west_east_threshold = y_west_baseline + distance_west_above
                y_west_west_threshold = y_west_baseline + distance_west_below

                y_nord_baseline = intercept*slope 
                y_south_baseline = intercept*slope 
                y_nord_nord_threshold = y_nord_baseline + self.nord_treshold
                y_nord_south_threshold = y_nord_baseline 
                y_south_nord_threshold = y_south_baseline
                y_south_south_threshold = y_south_baseline - self.south_treshold

                # nord east
                self.y_nord_east_east, self.y_nord_east_west, self.x_nord_east_nord, self.x_nord_east_south = y_east_east_threshold, y_east_west_threshold,  y_nord_nord_threshold, y_nord_south_threshold
                # nord west
                self.y_nord_west_east, self.y_nord_west_west, self.x_nord_west_nord, self.x_nord_west_south = y_west_east_threshold, y_west_west_threshold,  y_nord_nord_threshold, y_nord_south_threshold
                # south east
                self.y_south_east_east, self.y_south_east_west, self.x_south_east_nord, self.x_south_east_south = y_east_east_threshold, y_east_west_threshold,y_south_nord_threshold, y_south_south_threshold
                # south west
                self.y_south_west_east, self.y_south_west_west, self.x_south_west_nord, self.x_south_west_south = y_west_east_threshold, y_west_west_threshold, y_south_nord_threshold, y_south_south_threshold

                
            
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
        # from atan -> no division by zero to tan2
        theta = math.atan2(y,x)

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
        # from atan to atan2
        theta = math.atan2(y,x)
        return x,y,theta

     # update bool value
    
    def callback_update_bool(self, msg):
        self.publish_goal_position = True

    # update turning status: make switch work
    def update_turning_status_after_pose_publication(self):
        self.is_goal_published = False
        # msg = Bool()
        # msg.data = self.publish_goal_position
        # self.pub_turning_status(msg)

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

    def publish_end_of_line_pose_perpendicular_crop(self):
        # from last greatest point
        # if(self.greatest_point_south_east != '') & (self.greatest_point_south_west!= ''):
        point_east = self.greatest_point_south_east
        point_west = self.greatest_point_south_west
        # elif(self.greatest_point_nord_east != '') & (self.greatest_point_nord_west!= ''):
            # point_east = self.greatest_point_nord_east
            # point_west = self.greatest_point_nord_west
        # else:
           # print("ERROR")
        
        print(point_east, point_west)
        # m equationfrom two points
        m = (point_west[1]- point_east[1])/(point_west[0] - point_east[0])

        # m_perpendicular
        m_perp = -1/m

        # atan of the new theta
        new_theta = math.atan(m_perp)
        # perpendicular m
        # new_theta = math.tan(math.degrees(math.atan(m)) + 90)

        # take previous goal position
        # x,y = self.previous_forward_goal[0], self.previous_forward_goal[1]
        
        x = (point_west[1] - point_east[1])/2
        y = max(point_east[0], point_west[0])

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
        # print(np.size(nord_west_points), np.size(nord_east_points))
        plt.scatter(nord_west_points[:, 0], nord_west_points[:, 1], color='red')
        plt.scatter(nord_east_points[:, 0], nord_east_points[:, 1], color='orange')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 1.5, 3)

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

        # nord east
        x_values_1 = [self.x_nord_east_south, self.x_nord_east_nord]
        y_values_1 = [self.y_nord_east_east, self.y_nord_east_east]
        # print(x_values_1, y_values_1)
        plt.plot(x_values_1, y_values_1, color='orange',linestyle="--")

        x_values_2 = [self.x_nord_east_south, self.x_nord_east_nord]
        y_values_2 = [self.y_nord_east_west,self.y_nord_east_west]
        plt.plot(x_values_2, y_values_2,color='orange', linestyle="--")
        # print(x_values_2, y_values_2)

        # nord west
        x_values_3 = [self.x_nord_west_south, self.x_nord_west_nord]
        y_values_3 = [self.y_nord_west_east, self.y_nord_west_east]
        plt.plot(x_values_3, y_values_3, color='red',linestyle="--")

        x_values_4 = [self.x_nord_west_south, self.x_nord_west_nord]
        y_values_4 = [self.y_nord_west_west,self.y_nord_west_west]
        plt.plot(x_values_4, y_values_4,color='red', linestyle="--")

        # south east
        # plt.plot(x1, y1, x2, y2, marker = 'o') 
        # plt.plot(x1, y1, x2, y2, marker = 'o')

        # south west
        # plt.plot(x1, y1, x2, y2, marker = 'o') 
        # plt.plot(x1, y1, x2, y2, marker = 'o')

        plt.xlim(-3,3)
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

    in_row_navigation = InRowNavigation()

    rclpy.spin(in_row_navigation)

    in_row_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

