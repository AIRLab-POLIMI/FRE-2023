import sys
sys.path.append("/home/ceru/robotics/src/FRE-2023/grasslammer2_nav_py/grasslammer2_nav_py/")
import moving_average
from collections import deque
class MovingAvarageQueue():

    # assume type are exponentials
    def __init__(self, dimension_queue, start, stop, base_exp, reset_value):
        self.dimension_queue = dimension_queue
        self.queue = deque()
        self.weights = moving_average.MovingAvarage(start=start, stop=stop, base_exp=base_exp, dimension_queue=dimension_queue)
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
    

    
    