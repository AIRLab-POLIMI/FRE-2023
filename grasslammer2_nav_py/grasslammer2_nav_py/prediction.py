import numpy as np
import sys
sys.path.append("/home/alba/ros2_ws/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/")
import line, costum_queue

class Prediction():
    # assume type are exponentials
    def __init__(self, dimension_queue=5, ma_crop_flag=0, ma_navigation_flag=1, window_allowed_outdated_parameters=5,threshold_crop_line = 0.30, threshold_bisectrice_line = 0) :
        # dimension_queue -> n
        
        # save right/left cluster data
        # in case of rototranslation -> to be developed
        # self.right_cluster = costum_queue.CustomQueue(dimension_queue, window_allowed_outdated_parameters)
        # self.left_cluster = costum_queue.CustomQueue(dimension_queue, window_allowed_outdated_parameters)
        # save crop/bisectrice parameters
        self.navigation_line =  line.Line(dimension_queue, ma_navigation_flag, True, threshold_bisectrice_line, window_allowed_outdated_parameters, 0)
        self.positive_crop_line = line.Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, 0.375)
        self.negative_crop_line = line.Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, -0.375)

        # min_point_for_predition + threshold
        self.num_min_points_required_for_fitting = 3

    
    # initialization of lines
    def initialize_prediction(self):
        self.navigation_line.initialize_line()
        self.positive_crop_line.initialize_line()
        self.negative_crop_line.initialize_line() 

    
    def compute_bisectrice_coefficients(self, positive_points, negative_points):

        # compute positive crop line
        self.compute_crop_line_coefficients(self.positive_crop_line, positive_points)
        positive_slope, positive_intercept = self.positive_crop_line.get_most_recent_coefficients() 
        
        # compute negative crop line
        self.compute_crop_line_coefficients(self.negative_crop_line, negative_points)
        negative_slope, negative_intercept =self.negative_crop_line.get_most_recent_coefficients() 
        
        # compute bisectrice
        medium_slope = (positive_slope+negative_slope) / 2
        medium_intercept = (positive_intercept+negative_intercept) / 2

        # add coeddificent to bisectrice
        self.navigation_line.update_line_parameters(medium_slope, medium_intercept)


    def compute_crop_line_coefficients(self, local_line, points):
        # print("num points", len(points))

        if(len(points)> self.num_min_points_required_for_fitting):
            # compute slope, intercept through fitting
            slope, intercept = local_line.fitting_line(points)
            # update the queue with the new slope and intercept
            local_line.update_line_parameters_checking_threshold(slope, intercept)
        else:
            # mantain last value -> use False, False
            # get latest value (if exist)

            # else reset value
            slope, intercept = local_line.get_most_recent_coefficients()
            local_line.update_line_parameters_checking_threshold(slope, intercept)


    def __repr__ (self):
        return 'Line(slope=' + self.slope.__repr__() + ' ,intercept=' + self.intercept.__repr__()  + ')'
    
    

    

