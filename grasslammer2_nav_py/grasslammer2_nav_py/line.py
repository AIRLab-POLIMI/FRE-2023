import numpy as np
from sklearn.linear_model import RANSACRegressor
import sys
sys.path.append("/home/alba/ros2_ws/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/")
import costum_queue, moving_average_queue

class Line():
    # assume type are exponentials
    def __init__(self, dimension_queue, ma_flag, is_bisectrice, threshold, window_allowed_outdated_parameters, reset_value_intercept):

        if ma_flag == True:
            self.slope = moving_average_queue.MovingAvarageQueue(dimension_queue=dimension_queue, start=0, stop=4, base_exp=100, reset_value=0)
            self.intercept = moving_average_queue.MovingAvarageQueue(dimension_queue=dimension_queue,start=0, stop=4, base_exp=100, reset_value=reset_value_intercept)
        else:
            self.slope = costum_queue.CustomQueue(dimension_queue, reset_value=0)
            self.intercept = costum_queue.CustomQueue(dimension_queue, reset_value=reset_value_intercept)
        
        # ? needed flag
        self.is_bisectrice = is_bisectrice
        # threshold allowed btw two consecutive lines 
        self.threshold_btw_consecutive_lines = threshold
        # counter outdated lines
        self.counter_outdated_consecutive_values = 0
        # consistency on filter in terms of number required points
        self.window_allowed_outdated_parameters = dimension_queue
        
        # previous status
        self.last_valid_slope = 0
        self.last_valid_intercept = 0

        # TO DO add consistency num_trials and min sample
        self.ransac = RANSACRegressor()
    
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
        if(self.threshold_btw_consecutive_lines != 0):
            if (self.counter_outdated_consecutive_values <= self.window_allowed_outdated_parameters):
                if slope_time_t_min_1 != False and intercept_time_t_min_1 != False:
                    # calculate threshold
                    if abs(slope_time_t_min_1 - slope) > self.threshold_btw_consecutive_lines or abs(intercept_time_t_min_1 - intercept) > self.threshold_btw_consecutive_lines:
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
        slope = self.slope.return_element_time_t()
        intercept = self.intercept.return_element_time_t()
        return slope, intercept
    
    
    def get_oldest_coefficients(self):
        slope = self.slope.return_oldest_element()
        intercept = self.intercept.return_oldest_element()
        return slope, intercept

        
    def __repr__ (self):
        return 'Line(slope=' + self.slope.__repr__() + ' ,intercept=' + self.intercept.__repr__()  + ')'
    
    

    

