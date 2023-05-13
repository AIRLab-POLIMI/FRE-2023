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
        print("update_weights ", length_queue, dimension_queue)
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
        print("update_queue_regularly ", len(self.queue),self.dimension_queue)
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
        print("update_queue_regularly ", len(self.queue),self.dimension_queue)
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
        self.positive_crop_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, 0.375)
        self.negative_crop_line = Line(dimension_queue, ma_crop_flag, False, threshold_crop_line, window_allowed_outdated_parameters, -0.375)

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
    

class InRowNavigation(Node):
    # assume type are exponentials
    def __init__(self, dimension_queue=5, ma_crop_flag=0, ma_navigation_flag=1, num_skip = 5,threshold_crop_line = 0.5, threshold_bisectrice_line = 0.5, min_num_required_points=15) :
        super().__init__('in_row_navigation')
        # topics where the data are published
        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 
        self.goal_pose_pub = self.create_publisher(Float64MultiArray, '/goal_position', 1)
        self.goal_pose_pub # prevent unused variable warning
        self.end_of_line_pose_topic = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)
        self.end_of_line_pose_topic # prevent unused variable warning

        # HELPED
        # publish most recent pose and oldest pose
        # TODO
        self.collision_detected_topic = self.create_publisher(PoseStamped, '/collision_detected_topic', 1)
        
        # minimum number points
        self.min_num_required_points = min_num_required_points

        # ?
        self.end_of_line_flag = False
        self.gap_left = False
        self.gap_right = False

        # parameters needed during perfomance calculation
        self.end_computation = -1
        self.start_computation = -1

        # prefiltering
        self.prefiltering_threshold = 0.75 

        # Prediction obj
        # add parameters
        self.prediction_instance = Prediction()

        # Display Ransac
        self.fig, self.ax = plt.subplots()

        # num_accepted_points
        self.counter_start_line = 0
        self.window_skipped_start_line = num_skip

        # needed for goal publication
        self.is_end_of_line = False
        self.first_time_ROI = True


    def scan_callback(self, scan_msg):
        self.start_computation = time.perf_counter()
        points_2d = self.laser_scan_to_cartesian(scan_msg)

        # TODO riflettere altro nodo
        # collision detected

        if(np.size(points_2d) < self.min_num_required_points):
            print('No points found in ROI! ')
            # assume roi = 0 reset counter start line
            self.counter_start_line = 0
            # parameter read from parameter server. TODO.
            # print(self.is_end_of_line, self.first_time_ROI)
            if(self.is_end_of_line == True):
                if (self.first_time_ROI == True):
                    # publish last goal pose
                    x, y, theta = self.calculate_goal_point()
                    # invoke publish_end_of_line
                    self.publish_end_of_line_pose(x, y, theta)
                    # reset first_time_roi
                    self.first_time_ROI = False
            else: 
                self.is_end_of_line = False
            
            # initialize_prediction
            self.prediction_instance.initialize_prediction()
            
        else:
            # reset first_time_ROI, is_end_of_line
            self.first_time_ROI = True
            self.is_end_of_line = True

            # invoke prefiltering
            row_positive_value, row_negative_value = self.prefilter_point_far_from_bisectrice(points_2d)
            
            # skip first measurements 
            # if (self.counter_start_line < self.window_skipped_start_line):
                # start_line = True
                # self.counter_start_line = self.counter_start_line + 1
            # else:
                # start_line = False
            
            # bisectrice prediction
            self.prediction_instance.compute_bisectrice_coefficients(row_positive_value, row_negative_value)

            # invoke calculate_goal_position
            x, y, theta = self.calculate_goal_point()

            # publish goal pose
            self.publish_goal_pose(x, y, theta)

            # display 
            self.display_prediction(row_positive_value, row_negative_value, x, y)


    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=(msg.angle_max - msg.angle_min)/720) 

        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))
        
        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]
    
        return points_filtered
    

    def prefilter_point_far_from_bisectrice(self, points):
        # prefilter positive and negative values
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # print(slope, intercept, points)
        # y = self.line_coefficient[0]*points[:, 0]+ self.line_coefficient[1]
        mask_pos = ((points[:, 1] > slope*points[:, 0]+ intercept) & (points[:, 1] < ((slope*points[:, 0]+ intercept) + self.prefiltering_threshold)))
        mask_neg = ((points[:, 1] < slope*points[:, 0]+ intercept) & (points[:, 1] > ((slope*points[:, 0]+ intercept) - self.prefiltering_threshold)))
        # print(mask_pos, mask_neg)

        return points[mask_pos], points[mask_neg]
    
    # def check_collision(self):
        # check if nearest points refilteres are inside a detection threshold
        # in case yes publish data
        

    def calculate_goal_point(self):
        # takes the last m/q value of bisectrice
        slope, intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()
        
        # calculate goal_points
        tmp_result = pow(slope,2)* pow(intercept,2) - (1+ pow(slope,2))*(pow(intercept,2)-1)
        
        # safety if-else statement
        if(tmp_result >=0):
            x_1 = (-slope*intercept + math.sqrt(tmp_result))/(1+slope**2)
            x_2 = (-slope*intercept - math.sqrt(tmp_result))/(1+slope**2)
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
        goal_pose.data = [x, y, theta]
        # publish goal pose
        self.goal_pose_pub.publish(goal_pose)

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
        
    def display_prediction(self, row_positive_value, row_negative_value, x_goal, y_goal):
        # clear axes
        self.ax.clear()
        # creates scatter plot
        plt.scatter(row_positive_value[:, 0], row_positive_value[:, 1], color='blue')
        plt.scatter(row_negative_value[:, 0], row_negative_value[:, 1], color='blue')
        
        # takes 3 values btw 0/2
        x = np.linspace(0, 2, 3)

        # get proper slope, intercept for each value
        positive_slope, positive_intercept = self.prediction_instance.positive_crop_line.get_most_recent_coefficients()
        negative_slope, negative_intercept = self.prediction_instance.negative_crop_line.get_most_recent_coefficients()
        bisectrice_slope, bisectrice_intercept = self.prediction_instance.navigation_line.get_most_recent_coefficients()

        # creates line
        y_pos = positive_slope * x + positive_intercept
        y_neg = negative_slope * x + negative_intercept

        # plot line
        plt.plot(x, y_pos, color='red')
        plt.plot(x, y_neg, color='red')

        # robot line
        y = bisectrice_slope * x + bisectrice_intercept

        # bisectrice
        plt.plot(x, y, color='green')

        # goal point 
        plt.plot(x_goal, y_goal, marker="o", markersize=10, markeredgecolor="blue", markerfacecolor="blue")

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


