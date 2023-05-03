import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import MarkerArray, Marker

import math
import time
import csv
import os
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

# tunable distance goal point
distance_goal_point = 1
delta_from_bisectrice = 0.60


# data analysis path

data_analysis_path = "/home/alba/ros2_ws/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/data_analysis/"
number_samples = "/home/alba/ros2_ws/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/data_analysis/"
num_items = 200
min_samples=2
num_trials=10
class NavigationRansac(Node):
    def __init__(self):
        super().__init__('navigation_ransac')

        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.goal_poisition_pub = self.create_publisher(Float64MultiArray, '/goal_position', 1)
        
        # use deque to maintain history parameter most recent value
        self.parameters_positive_row = deque()
        self.parameters_negative_row = deque()
        self.model_parameters = deque()

        # weigths to add consistency to goal point
        self.weigths_slope = []
        self.weigths_intercept = []
        self.weigths_slope_crop_negative = []
        self.weigths_intercept_crop_negative = []
        self.weigths_slope_crop_positive = []
        self.weigths_intercept_crop_positive = []
        self.last_valid_positive = [0.0, 0.0]
        self.last_valid_negative = [0.0, 0.0]


        self.num_points_arrived = False
        
        # to be commented
        self.compare_bisectrice_no_incercept()

        # TO BE
        # self.ransac = RANSACRegressor(min_samples=min_samples, max_trials=num_trials)
        self.ransac = RANSACRegressor()

        # line_coefficient
        self.line_coefficient = [0,0]
        self.line_coefficient_no_intercept = [0,0]

        # previous_line 
        self.previous_valid_line = [0.0, 0.0]
        self.first_iteration = False

        # write performances in folder
        # print(data_analysis_path)
        self.performance_file = ''
        self.csv_writer = ''
        self.csv_writer_samples = ''
        self.samples = []
        self.last_sample = ''
        self.num_lines = 0
        # self.initialization_ransac_formula_analysis()mmit -m '
        # to be defined by 
        self.fig, self.ax = plt.subplots()
        
        # self.fig_2, self.ax_2 = plt.subplots(nrows=1, ncols=2)
  
    def scan_callback(self, scan_msg):
        start_time = time.perf_counter()
        # Transform laser scan msg of ROI points into cartesian coordinate
        points_2d = self.laser_scan_to_cartesian(scan_msg)
        # print("i'm in the callback")
        
        if(np.size(points_2d) < 15):
            # Used to prevent crash from no points detected in the region of interest
            # turning
            self.line_coefficient= [0, 0]
            print('No points found in ROI! ')
        else:
            # Use RANSAC
            # self.RANSAC(points_2d)
            # Use Ransac WIth MA
            # self.RANSAC_with_MA(points_2d)
            # Use Ransac WIth MA with intercept
            # self.RANSAC_with_MA_with_intercept_and_not(points_2d)
            self.RANSAC_with_MA_with_prefiltering(points_2d)
            # self.RANSAC_with_MA_with_prefiltering_enhanced(points_2d)
        end_time = time.perf_counter()
        # self.ransac_formula_analysis(start_time, end_time)
             
    def compare_bisectrice_no_incercept(self):
        # use deque to maintain history parameter most recent value
        self.parameters_positive_row_no_intercept = deque()
        self.parameters_negative_row_no_intercept = deque()
        self.model_parameters_no_intercept = deque()
        # weigths to add consistency to goal point
        self.weigths_slope_no_intercept = []

    def calculate_weigths_slope(self, num_points):
        values = np.arange(start_slope,stop_slope, step=(stop_slope-start_slope)/num_points)
        
        # linear -> values_function = [value for value in values]
        # exp
        # values_function = [np.exp(value) for value in values]
        # exp 10
        # values_function = [pow(base_exp,value) for value in values]
        # all same weigth --> values_function = [1 for _ in values]
        values_function = [pow(base_exp,value) for value in values]
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
        # medium_intercept = 0
        medium_intercept = (lines[0][1]+lines[1][1]) / 2
        

        # append value in array
        self.model_parameters.appendleft([medium_slope, medium_intercept])

        # queue dimension
        dim_self_param = len(self.model_parameters)

        # not reach maximum queue dimension
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
        output_intercept = 0
        # set intercept 0
        # output_intercept = [0]
        output_intercept = [output_intercept + self.weigths_intercept[i]*self.model_parameters[i][1] for i in range(dim_self_param)]
        # output_intercept = [output_intercept + self.weigths_intercept[i]*self.model_parameters[i][1] for i in range(dim_self_param)]
        # print("MODEL: ",self.model_parameters)
        output_line = [output_slope[0], output_intercept[0]]
        
        # output_line = [output_slope[0],0]
        # sobstitute the medium with avg 
        self.model_parameters.popleft()
        self.model_parameters.appendleft(output_line)
        # print("MODEL: ",self.model_parameters)

        return output_line

    def update_queue_positive_crop_MA(self, last_valid, crop_line):
        # queue dimension
        # do the same reasoning for 
        dim_self_param = len(self.parameters_positive_row)
        print("POSITIVE", dim_self_param)
        # not reach maximum queue dimension
        # zero
        if(dim_self_param <= number_points):
            if(dim_self_param == 0):
                self.parameters_positive_row.appendleft([0, 0])
                dim_self_param = len(self.parameters_positive_row)
            self.weigths_intercept_crop_positive = self.calculate_weigths_intercept(dim_self_param)
            self.weigths_slope_crop_positive = self.calculate_weigths_slope(dim_self_param)
        else:
            # remove last item, add current first element of queue
            self.parameters_positive_row.pop()
            self.weigths_intercept_crop_positive = self.calculate_weigths_intercept(dim_self_param)
            self.weigths_slope_crop_positive = self.calculate_weigths_slope(dim_self_param)
        

        if (last_valid):
            # take crop row and add queue
            self.parameters_positive_row.pop()
            self.parameters_positive_row.appendleft(crop_line)
            self.last_valid_positive = crop_line

        else:
            # take last avlid positive param row and add queue
            self.parameters_positive_row.pop()
            self.parameters_positive_row.appendleft(self.last_valid_positive)
        
    def update_queue_negative_crop_MA(self, last_valid, crop_line):
        # queue dimension
        # do the same reasoning for 
        dim_self_param = len(self.parameters_negative_row)

        # not reach maximum queue dimension
        if(dim_self_param <= number_points):
            if(dim_self_param == 0):
                self.parameters_negative_row.appendleft([0, 0])
                dim_self_param = len(self.parameters_negative_row)
            self.weigths_intercept_crop_negative = self.calculate_weigths_intercept(dim_self_param)
            self.weigths_intercept_crop_negative = self.calculate_weigths_slope(dim_self_param)
        else:
            # remove last item, add current first element of queue
            self.parameters_negative_row.pop()
            self.weigths_intercept_crop_negative = self.calculate_weigths_intercept(dim_self_param)
            self.weigths_intercept_crop_negative = self.calculate_weigths_slope(dim_self_param)
        

        if (last_valid):
            # take crop row and add queue
            self.parameters_negative_row.pop()
            self.parameters_negative_row.appendleft(crop_line)
            self.last_valid_negative= crop_line

        else:
            # take last avlid positive param row and add queue
            self.parameters_negative_row.pop()
            self.parameters_negative_row.appendleft(self.last_valid_negative)

    def calculate_positive_intercept_slope_MA(self):

        print(len(self.parameters_positive_row))
        dim_parameter = len(self.parameters_positive_row)
        output_slope = 0
        output_slope = [output_slope + self.weigths_slope_crop_positive[i]*self.parameters_positive_row[i][0] for i in range(dim_parameter)]
        
        # calculate weighted intercept
        output_intercept = 0
        # set intercept 0
        # output_intercept = [0]
        output_intercept = [output_intercept + self.weigths_slope_crop_positive[i]*self.parameters_positive_row[i][1] for i in range(dim_parameter)]
        # output_intercept = [output_intercept + self.weigths_intercept[i]*self.model_parameters[i][1] for i in range(dim_self_param)]
        # print("MODEL: ",self.model_parameters)

        return output_slope[0], output_intercept[0]
    
    def calculate_negative_intercept_slope_MA(self):
        dim_parameter = len(self.parameters_negative_row)
        print(dim_parameter, self.weigths_slope_crop_negative)
        output_slope = 0
        output_slope = [output_slope + self.weigths_slope_crop_negative[i]*self.parameters_negative_row[i][0] for i in range(dim_parameter)]
        
        # calculate weighted intercept
        output_intercept = 0
        # set intercept 0
        # output_intercept = [0]
        output_intercept = [output_intercept + self.weigths_slope_crop_negative[i]*self.parameters_negative_row[i][1] for i in range(dim_parameter)]
        # output_intercept = [output_intercept + self.weigths_intercept[i]*self.model_parameters[i][1] for i in range(dim_self_param)]
        # print("MODEL: ",self.model_parameters)
        return output_slope[0], output_intercept[0]

    def calculate_MA_bisectrice_slope_intercept(self):
        # take last value of weigths through weigthed avarage
        positive_weigths, positive_intercept = self.calculate_positive_intercept_slope_MA()
        negative_weigths, negative_intercept = self.calculate_negative_intercept_slope_MA()

        # calculate current bisectrice
        medium_slope = (positive_weigths+negative_weigths) / 2
        # set intercept = 0
        # medium_intercept = 0
        medium_intercept = (positive_intercept+negative_intercept) / 2
        

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
        output_intercept = 0
        # set intercept 0
        # output_intercept = [0]
        output_intercept = [output_intercept + self.weigths_intercept[i]*self.model_parameters[i][1] for i in range(dim_self_param)]
        # output_intercept = [output_intercept + self.weigths_intercept[i]*self.model_parameters[i][1] for i in range(dim_self_param)]
        # print("MODEL: ",self.model_parameters)
        output_line = [output_slope[0], output_intercept[0]]
        
        # output_line = [output_slope[0],0]
        # sobstitute the medium with avg 
        self.model_parameters.popleft()
        self.model_parameters.appendleft(output_line)
        # print("MODEL: ",self.model_parameters)

        self.line_coefficient = output_line

    # tmp1 save 
    def RANSAC_with_MA_with_prefiltering_no_crop_prediction(self, points):
        # contains intercept + slope two interpolated lines
        crop_lines = []
        # y = self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]
        mask_pos = ((points[:, 1] > self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) & (points[:, 1] < ((self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) + delta_from_bisectrice)))
        mask_neg = ((points[:, 1] < self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) & (points[:, 1] > ((self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) - delta_from_bisectrice)))
        # print(mask_pos, mask_neg)
        row_positive_value = points[mask_pos]
        row_negative_value = points[mask_neg]
        
        # add value number points
        self.last_sample = min(len(row_positive_value), len(row_negative_value))
        
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
            # calculate goal position 
            msg = self.calculate_goal_point_bisectrice_with_intercept(robot_line)
            # visualization
            # self.visualize_ransac_MA_with_goal_point(points, crop_lines, robot_line,msg)
            self.visualize_ransac_MA_with_goal_point_with_prefiltering(row_positive_value, row_negative_value, crop_lines, robot_line,msg)
    
            # update boundaries
            self.line_coefficient = robot_line
            
            # publish goal position
            self.goal_poisition_pub.publish(msg)
            
        else:
            # make prediction on left or rigth and calculate bisectrice
            # spray!!
            # Use MA to 'predict' value based on the past values. 
            # save the last valid m,q inside queue (dedicated)
            self.line_coefficient = [0, 0]
            # print(self.line_coefficient)
    
    def calculation_positive_crop_line_coefficient(self, masked_points):

        coefficient = []
        if(len(masked_points)>=1):
            self.ransac.fit(masked_points[:, 0].reshape(-1, 1), masked_points[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            coefficient = [slope, intercept]
            if(len(self.parameters_positive_row)>number_points):
                self.parameters_positive_row.pop()
                     
        else:
            # first get value
            # then add again
            if(len(self.parameters_positive_row)>=0):
                coefficient = self.parameters_positive_row.popleft()
                self.parameters_positive_row.appendleft(coefficient)
            else:
                coefficient = [0, 0]
            
           
            if(len(self.parameters_positive_row)>number_points):
                self.parameters_positive_row.pop()
            
        self.parameters_positive_row.appendleft(coefficient)
        return coefficient
    
    def calculation_negative_crop_line_coefficient(self, masked_points):
        coefficient = []
        if(len(masked_points)>=1):
            self.ransac.fit(masked_points[:, 0].reshape(-1, 1), masked_points[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            coefficient = [slope, intercept]
            if(len(self.parameters_negative_row)>number_points):
                self.parameters_negative_row.pop()
                     
        else:
            # first get value
            # then add again
            if(len(self.parameters_negative_row)>=0):
                coefficient = self.parameters_negative_row.popleft()
                self.parameters_negative_row.appendleft(coefficient)
            else:
                coefficient = [0, 0]
            
           
            if(len(self.parameters_negative_row)>number_points):
                self.parameters_negative_row.pop()
            
        self.parameters_negative_row.appendleft(coefficient)
        return coefficient

    def RANSAC_with_MA_with_prefiltering(self, points):
        # contains intercept + slope two interpolated lines
        crop_lines = []
        # y = self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]
        mask_pos = ((points[:, 1] > self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) & (points[:, 1] < ((self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) + delta_from_bisectrice)))
        mask_neg = ((points[:, 1] < self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) & (points[:, 1] > ((self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) - delta_from_bisectrice)))
        # print(mask_pos, mask_neg)
        row_positive_value = points[mask_pos]
        row_negative_value = points[mask_neg]
    
        self.last_sample = min(len(row_positive_value), len(row_negative_value))
        
        crop_lines.append(self.calculation_positive_crop_line_coefficient(row_positive_value))
        crop_lines.append(self.calculation_negative_crop_line_coefficient(row_negative_value))

        if(len(row_negative_value)>=RANSAC_num_point and len(row_positive_value)>=RANSAC_num_point):
            # print("CROP_LINES ", crop_lines)
            robot_line = self.calculate_MA_slope_intercept(crop_lines)
            # calculate goal position 
            msg = self.calculate_goal_point_bisectrice_with_intercept(robot_line)
            # visualization
            # self.visualize_ransac_MA_with_goal_point(points, crop_lines, robot_line,msg)
            self.visualize_ransac_MA_with_goal_point_with_prefiltering(row_positive_value, row_negative_value, crop_lines, robot_line,msg)
    
            # update boundaries
            self.line_coefficient = robot_line
            
            # publish goal position
            self.goal_poisition_pub.publish(msg)
            
        else:
            # make prediction on left or rigth and calculate bisectrice
            # spray!!
            # Use MA to 'predict' value based on the past values. 
            # save the last valid m,q inside queue (dedicated)

            msg = self.calculate_goal_point_bisectrice_with_intercept(self.line_coefficient)
            self.visualize_ransac_MA_with_goal_point_with_prefiltering(row_positive_value, row_negative_value, crop_lines, self.line_coefficient,msg)
            # publish goal position
            self.goal_poisition_pub.publish(msg)
        
    def RANSAC_with_MA_with_prefiltering_enhanced(self, points):
        # contains intercept + slope two interpolated lines
        crop_lines = []
        # y = self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]
        mask_pos = ((points[:, 1] > self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) & (points[:, 1] < ((self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) + delta_from_bisectrice)))
        mask_neg = ((points[:, 1] < self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) & (points[:, 1] > ((self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]) - delta_from_bisectrice)))
        # print(mask_pos, mask_neg)
        row_positive_value = points[mask_pos]
        row_negative_value = points[mask_neg]
    
        self.last_sample = min(len(row_positive_value), len(row_negative_value))

        print(len(row_positive_value), len(row_negative_value))
        if len(row_positive_value)>=RANSAC_num_point:
            # add queue new value
            self.ransac.fit(row_positive_value[:, 0].reshape(-1, 1), row_positive_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            self.update_queue_positive_crop_MA(True, [slope, intercept])
        else:
            # add queue last valid value
            self.update_queue_positive_crop_MA(False, [0, 0])
        

        if len(row_negative_value)>=RANSAC_num_point:
            # add queue new value
            self.ransac.fit(row_negative_value[:, 0].reshape(-1, 1), row_negative_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            intercept = self.ransac.estimator_.intercept_
            self.update_queue_negative_crop_MA(True, [slope, intercept])
        else:
            # add queue last valid value
            self.update_queue_negative_crop_MA(False, [0, 0])
        
        # print(self.parameters_negative_row, self.parameters_positive_row)
        # calculate bisectrice
        self.calculate_MA_bisectrice_slope_intercept()

        msg = self.calculate_goal_point_bisectrice_with_intercept(self.line_coefficient)
        # coefficient
        # take last value of weigths through weigthed avarage
        # take positive/negative value
        positive_weigths, positive_intercept = self.calculate_positive_intercept_slope_MA()
        negative_weigths, negative_intercept = self.calculate_negative_intercept_slope_MA()
        crop_lines = [[positive_weigths, positive_intercept], [negative_weigths, negative_intercept]]
        self.visualize_ransac_MA_with_goal_point_with_prefiltering(row_positive_value, row_negative_value, crop_lines, self.line_coefficient,msg)
        # publish goal position
        self.goal_poisition_pub.publish(msg)
 
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
            # calculate goal position 
            msg = self.calculate_goal_point_bisectrice(robot_line)
            # visualization
            self.visualize_ransac_MA_with_goal_point(points, crop_lines, robot_line,msg)
    
            # update boundaries
            self.line_coefficient = robot_line

            # last valid line
            self.previous_valid_line = robot_line
            # publish goal position
            self.goal_poisition_pub.publish(msg)
            
        else:
            self.line_coefficient = self.previous_valid_line

    def RANSAC_with_MA_no_intercept(self, points):
        # contains intercept + slope two interpolated lines
        crop_lines = []
        
        row_positive_value = points[np.where(points[:, 1] > self.line_coefficient_no_intercept[0]*points[:, 0]+ self.line_coefficient[1])]
        row_negative_value = points[np.where(points[:, 1] < self.line_coefficient_no_intercept[0]*points[:, 0]+ self.line_coefficient[1])]

        if (len(row_positive_value)>=1):
            self.ransac.fit(row_positive_value[:, 0].reshape(-1, 1), row_positive_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            # intercept = self.ransac.estimator_.intercept_
            intercept = 0
            crop_lines.append([slope, intercept])

        if (len(row_negative_value)>=1):
            self.ransac.fit(row_negative_value[:, 0].reshape(-1, 1), row_negative_value[:, 1])
            slope = self.ransac.estimator_.coef_[0]
            # intercept = self.ransac.estimator_.intercept_
            intercept = 0
            crop_lines.append([slope, intercept])

        if(len(row_negative_value)>=RANSAC_num_point and len(row_positive_value)>=RANSAC_num_point):
            # print("CROP_LINES ", crop_lines)
            robot_line = self.calculate_MA_slope_intercept(crop_lines)
            # calculate goal position 
            goal_point = self.calculate_goal_point_bisectrice(robot_line)
            self.line_coefficient_no_intercept = robot_line
            
        else:
            self.line_coefficient_no_intercept = [0, 0, 0]
            robot_line = [0, 0, 0]
            goal_point = [0, 0, 0]
        return crop_lines, robot_line, goal_point

    def RANSAC_with_MA_with_intercept(self, points):
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

        # calculate dual no intercept
        if(len(row_negative_value)>=RANSAC_num_point and len(row_positive_value)>=RANSAC_num_point):
            # print("CROP_LINES ", crop_lines)
            robot_line = self.calculate_MA_slope_intercept(crop_lines)
            # calculate goal position 
            # msg = self.calculate_goal_point_bisectrice_with_intercept
            msg = self.calculate_goal_point_bisectrice_with_intercept(robot_line)


            # TO BE DECOMMENTED
            self.visualize_ransac_MA_with_goal_point(points, crop_lines, robot_line, msg)
            
            # update boundaries
            self.line_coefficient = robot_line
            
            # publish goal position
            self.goal_poisition_pub.publish(msg)
            
        else:
            self.line_coefficient = [0, 0, 0]

    def RANSAC_with_MA_with_intercept_and_not(self, points):
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

        # to be commented
         # to be commented
        crop_lines_no_intercept, robot_line_no_intercept, goal_point_no_intercept = self.RANSAC_with_MA_no_intercept(points)

        # calculate dual no intercept
        if(len(row_negative_value)>=RANSAC_num_point and len(row_positive_value)>=RANSAC_num_point):
            # print("CROP_LINES ", crop_lines)
            robot_line = self.calculate_MA_slope_intercept(crop_lines)
            # calculate goal position 
            # msg = self.calculate_goal_point_bisectrice_with_intercept
            msg = self.calculate_goal_point_bisectrice_with_intercept(robot_line)

            # visualization to be commented
            self.visualize_ransac_MA_with_goal_point_intercept_and_not(points, crop_lines, robot_line, robot_line_no_intercept, msg, goal_point_no_intercept)
    
            # update boundaries
            self.line_coefficient = robot_line
            
            # publish goal position
            self.goal_poisition_pub.publish(msg)
            
        else:
            self.line_coefficient = [0, 0, 0]
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
        # for each line
        y_pos = y + delta_from_bisectrice
        y_neg = y - delta_from_bisectrice
        # plot line
        # print("LINE", line, "CROP",  crop_lines)
        plt.plot(x, y_pos, color='violet')
        plt.plot(x, y_neg, color='black')
        plt.xlim(0,3)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)

    def visualize_ransac_MA_with_goal_point(self, points, crop_lines, robot_lines, msg):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(points[:, 0], points[:, 1], color='blue')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # creates line
        y_pos = crop_lines[0][0] * x + crop_lines[0][1]
        y_neg = crop_lines[1][0] * x + crop_lines[1][1]
        # plot line
        # print("LINE", line, "CROP",  crop_lines)
        plt.plot(x, y_pos, color='red')
        plt.plot(x, y_neg, color='red')

        # robot line
        y = robot_lines[0] * x + robot_lines[1]
        # print("ROBOT ", robot_lines,"X", x)
        # bisectrice
        plt.plot(x, y, color='green')
        # goal point
        plt.plot(msg.data[0], msg.data[1], marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")
        # remove outlier from bisectrice
        y_pos_delta = y + delta_from_bisectrice
        y_neg_delta = y - delta_from_bisectrice
        # plot line
        print("LINE", robot_lines, "CROP",  crop_lines)
        plt.plot(x, y_pos_delta, color='violet')
        plt.plot(x, y_neg_delta, color='black')

        plt.xlim(0,3)
        plt.ylim(-2,2)

        self.fig.canvas.draw()
        plt.pause(0.01)

    def visualize_single_plot_DISCARDED(self, points, ax_index, crop_lines, robot_lines, goal_point):
        # clear axes
        self.ax_2[ax_index].clear()

        # creates scatter plot
        self.ax_2[ax_index].scatter(points[:, 0], points[:, 1], color='blue')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # creates line
        y_pos = crop_lines[0][0] * x + crop_lines[0][1]
        y_neg = crop_lines[1][0] * x + crop_lines[1][1]

        # plot line
        # print("LINE", line, "CROP",  crop_lines)
        self.ax_2[ax_index].plot(x, y_pos, color='red')
        self.ax_2[ax_index].plot(x, y_neg, color='red')

        # robot line
        y = robot_lines[0] * x + robot_lines[1]
        # print("ROBOT ", robot_lines,"X", x)
        # bisectrice
        self.ax_2[ax_index].plot(x, y, color='green')
        # goal point
        self.ax_2[ax_index].plot(goal_point.data[0], goal_point.data[1], marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")
        # remove outlier from bisectrice
        # y_pos_delta = y + delta_from_bisectrice
        # y_neg_delta = y - delta_from_bisectrice
        # plot line
        print("LINE", robot_lines, "CROP",  crop_lines)
        # plt.plot(x, y_pos_delta, color='violet')
        # plt.plot(x, y_neg_delta, color='black')

        self.ax_2[ax_index].xlim(0,3)
        self.ax_2[ax_index].ylim(-2,2)
  
    def visualize_ransac_MA_with_goal_point_intercept_and_not_DISCARDED_double_plot(self, points, crop_lines, crop_lines_no_intercept, robot_lines, robot_lines_no_intercept, msg, goal_point_no_intercept):
        self.visualize_single_plot(points, 0, crop_lines, robot_lines, msg)
        self.visualize_single_plot(points ,1, crop_lines_no_intercept, robot_lines_no_intercept, goal_point_no_intercept)
        self.fig.canvas.draw()
        plt.pause(0.01)

    def visualize_ransac_MA_with_goal_point_intercept_and_not(self, points, crop_lines, robot_lines,old_robot_lines,  msg, old_msg):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(points[:, 0], points[:, 1], color='blue')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # creates line
        y_pos = crop_lines[0][0] * x + crop_lines[0][1]
        y_neg = crop_lines[1][0] * x + crop_lines[1][1]
        # plot line
        # print("LINE", line, "CROP",  crop_lines)
        plt.plot(x, y_pos, color='red')
        plt.plot(x, y_neg, color='red')

        # robot line
        y = robot_lines[0] * x + robot_lines[1]
        # print("ROBOT ", robot_lines,"X", x)
        # bisectrice
        plt.plot(x, y, color='blue')

        # robot line
        y_old = old_robot_lines[0] * x + old_robot_lines[1]
        # print("ROBOT ", robot_lines,"X", x)
        # bisectrice
        plt.plot(x, y_old, color='green')
        # goal point
        plt.plot(msg.data[0], msg.data[1], marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")
        plt.plot(old_msg.data[0], old_msg.data[1], marker="o", markersize=10, markeredgecolor="green", markerfacecolor="green")
        # remove outlier from bisectrice
        # y_pos_delta = y + delta_from_bisectrice
        # y_neg_delta = y - delta_from_bisectrice
        # plot line
        # print("LINE", robot_lines, "CROP",  crop_lines)
        # plt.plot(x, y_pos_delta, color='violet')
        # plt.plot(x, y_neg_delta, color='black')

        plt.xlim(0,3)
        plt.ylim(-2,2)

        self.fig.canvas.draw()
        plt.pause(0.01)      

    def visualize_ransac_MA_with_goal_point_with_prefiltering(self, row_positive_value, row_negative_value, crop_lines, robot_lines,msg):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(row_positive_value[:, 0], row_positive_value[:, 1], color='blue')
        plt.scatter(row_negative_value[:, 0], row_negative_value[:, 1], color='blue')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # creates line
        y_pos = crop_lines[0][0] * x + crop_lines[0][1]
        y_neg = crop_lines[1][0] * x + crop_lines[1][1]
        # plot line
        # print("LINE", line, "CROP",  crop_lines)
        plt.plot(x, y_pos, color='red')
        plt.plot(x, y_neg, color='red')

        # robot line
        y = robot_lines[0] * x + robot_lines[1]
        # print("ROBOT ", robot_lines,"X", x)
        # bisectrice
        plt.plot(x, y, color='green')
        # goal point 
        plt.plot(msg.data[0], msg.data[1], marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")
        # remove outlier from bisectrice
        y_pos_delta = y + delta_from_bisectrice
        y_neg_delta = y - delta_from_bisectrice
        # plot line
        # print("LINE", robot_lines, "CROP",  crop_lines)
        plt.plot(x, y_pos_delta, color='violet')
        plt.plot(x, y_neg_delta, color='black')

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

    # return tunable goal position 
    # [x,y]
    def calculate_goal_point_bisectrice(self, robot_line):
        # calculate x,y having y=m*x+q (q=0) and 1 as hypothenous
        x = math.sqrt(1/(1+robot_line[0]**2))
        y = robot_line[0]*math.sqrt(1/(1+robot_line[0]**2))
        theta = math.atan(y/x)
        # publish on topic
        # self.cmd_pub.publish(cmd_msg)
        # create message 
        goal_position = Float64MultiArray()
        # goal_position.layout.dim = 3
        # goal_position.layout.data_offset = 0
        goal_position.data = [x,y,theta]
        return goal_position
    
    
        # return tunable goal position 
    # [x,y]

    # apply also intercept method
    def calculate_goal_point_bisectrice_with_intercept(self, robot_line):
        # calculate x,y having y=m*x+q (q=0) and 1 as hypothenous
        m = robot_line[0]
        q = robot_line[1]
        # take only positive value -> distance
        # solve system
        # vedere meglio perchè muore
        tmp = pow(m,2)* pow(q,2) - (1+ pow(m,2))*(pow(q,2)-1)
        # print(m,q,tmp)
        if(tmp >=0):
            x_1 = (-m*q + math.sqrt(tmp))/(1+m**2)
            x_2 = (-m*q - math.sqrt(tmp))/(1+m**2)
        else:
            x_1 = 0
            x_2 = 0
            print("ERROR QUADRATIC")

        # take greatest
        if x_1 >= x_2:
            x = x_1
        else:
            x = x_2
        
        # solve equation
        y = m*x + q

        theta = math.atan(y/x)
        # intercept = robot_line[1]
        # publish on topic
        # self.cmd_pub.publish(cmd_msg)
        # create message 
        goal_position = Float64MultiArray()
        # goal_position.layout.dim = 3
        # goal_position.layout.data_offset = 0
        goal_position.data = [x,y,theta]

        return goal_position

    def initialization_ransac_formula_analysis(self):
        # write performances in folder
        # print(data_analysis_path)
        self.performance_file = open(data_analysis_path+"data_analysis_trials_"+str(self.ransac.max_trials)+"_inliers_"+str(self.ransac.stop_n_inliers)+"_score_"+str(self.ransac.stop_score)+".csv", "x")
        self.csv_writer = csv.writer(self.performance_file)
        # write header
        self.csv_writer.writerow(['current', 'min', 'max', 'avg'])
        self.samples_file = open(data_analysis_path+"number_samples"+str(self.ransac.min_samples)+".csv", "x")
        
        self.csv_writer_samples = csv.writer(self.samples_file)
        self.csv_writer_samples.writerow(['current', 'min', 'max', 'avg'])
 
    def ransac_formula_analysis(self, end_time, start_time):   
        execution_time = end_time - start_time
        avg_execution.append(execution_time)
        avg_execution_np = np.array(avg_execution)
        
        # write num_items to evaluate performances
        if (self.num_lines < num_items):
            self.csv_writer.writerow([execution_time,np.min(avg_execution_np),np.max(avg_execution_np),np.mean(avg_execution_np)])
            # write on csv file
            if (self.last_sample!= ''):
                self.samples.append(self.last_sample)
                self.csv_writer_samples.writerow([self.last_sample, np.min(self.samples), np.max(self.samples), np.mean(self.samples)])
            self.num_lines = self.num_lines + 1
        else:
            self.performance_file.close()
            self.samples_file.close()

        # print or file or so
        # print("EXECUTION TIME: ",execution_time)
        print("AVG EXECUTION: ", np.mean(avg_execution_np), ", MIN VALUE: ", np.min(avg_execution_np), ", MAX VALUE: ", np.max(avg_execution_np))

def main(args=None):
    rclpy.init(args=args)

    navigation = NavigationRansac()

    rclpy.spin(navigation)

    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    