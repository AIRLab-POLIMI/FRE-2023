import sys
sys.path.append("/home/alba/ros2_ws/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/")
import moving_average
from collections import deque
class MovingAvarageQueue():

    # assume type are exponentials
    def __init__(self, dimension_queue, window_allowed_outdated_parameters, start, stop, base_exp):
        self.dimension_queue = dimension_queue
        self.queue = deque()
        self.weights = moving_average.MovingAvarage(start=start, stop=stop, base_exp=base_exp, dimension_queue=dimension_queue )
        # counter outdated lines
        self.counter_outdated_consecutive_values = 0
        # consistency on filter in terms of number required points
        self.window_allowed_outdated_parameters = window_allowed_outdated_parameters


    # update queue with last valid value
    def update_queue_last_valid_value(self):
        if(len(self.queue) >= self.dimension_queue) and len(self.queue)!= 0:
            self.last_valid_value = self.queue.popleft()
            self.window_allowed_outdated_parameters = self.window_allowed_outdated_parameters + 1
        else:
            self.counter_outdated_consecutive_values = 0
            self.last_valid_value = 0
        self.queue.appendleft(self.last_valid_value)
        # update counter
        
    # update queue regularly
    def update_queue_regularly(self, value):
        # re-initialize counter outdated values and last_valid_value
        self.counter_outdated_consecutive_values = 0
        self.last_valid_value = 0

        if len(self.queue) != 0:
            if(len(self.queue) >= self.dimension_queue):
                self.queue.pop()
            self.queue.appendleft(value)
            self.weights.update_weights(len(self.queue))
            # calculate the MA
            MA = self.weights.calculate_MA(self.queue)
            # add MA as most recent element
            self.queue.popleft()
            self.queue.appendleft(MA)
        else:
            self.queue.appendleft(0)
            

    
    # value = False -> use last value on queue UNLESS 5 consecutive times
    def update_queue(self, value):
        # append value to queue
        # print(current_dim_queue, self.queue)
        if self.counter_outdated_consecutive_values < self.window_allowed_outdated_parameters:
            if value == False:
                self.update_queue_last_valid_value()
            else:
                self.update_queue_regularly(value)
        else:
            self.update_queue_regularly(value)
          
        
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
        self.weights.initialize_weigths()

    def __repr__ (self):
        tmp_queue = ''
        queue_str = [tmp_queue+ str(item)+'' for item in self.queue]
        return 'weights: ' + self.weights.__repr__() + ' queue '+ str(queue_str)
    

    
    