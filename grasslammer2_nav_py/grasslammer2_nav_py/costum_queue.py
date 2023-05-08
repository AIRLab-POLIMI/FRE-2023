from collections import deque
class CustomQueue:
    def __init__(self, dimension_queue, window_allowed_outdated_parameters):
        self.dimension_queue = dimension_queue
        self.queue = deque()
        
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
        
        
    # update queue regularly
    def update_queue_regularly(self, value):
        # re-initialize counter outdated values and last_valid_value
        self.counter_outdated_consecutive_values = 0
        self.last_valid_value = 0

        # append value to queue
        if len(self.queue)!= 0:
            if(len(self.queue) >= self.dimension_queue):
                self.queue.pop()
        # empty queue take 0 as value
        else:
            value = 0
        self.queue.appendleft(value)

    
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

    def initialize_queue(self):
        self.queue = deque()
    
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

    def __repr__ (self):
        tmp_queue = ''
        queue_str = [tmp_queue+ str(item)+'' for item in self.queue]
        # queue_str = ''.join(self.queue)
        return 'queue' + str(queue_str) + ' '
    
    def __str__ (self):
        tmp_queue = ''
        queue_str = [tmp_queue+ str(item)+'' for item in self.queue]
        return 'queue' + str(queue_str) + ' '
    
    
    

