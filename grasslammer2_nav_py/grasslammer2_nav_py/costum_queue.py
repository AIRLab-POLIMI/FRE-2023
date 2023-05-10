from collections import deque
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
    
    
    

