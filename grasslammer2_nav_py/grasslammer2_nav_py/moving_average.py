import numpy as np
class MovingAvarage():
    # assume type are exponentials
    def __init__(self, start, stop, base_exp, dimension_queue):
        self.weigths = []
        self.start = start
        self.stop = stop
        self.base_exp = base_exp
        self.dimension_MA = dimension_queue
    
    def update_weights(self, length_queue):
        # update weigths of moving avarage only when wwights < num_points 
        if (length_queue <= self.dimension_MA):
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
    

    def calculate_MA(self, array):
        if(len(self.weigths) != len(array)):
            print("ERROR MA computation weights, queue", len(self.weigths), len(array))
            return 1
        else:
            output_MA = 0
            output_MA = [output_MA + self.weigths[i]*array[i] for i in range(len(array))]
            return output_MA[0]
    
    def get_weigths(self):
        return self.weigths
    
    def initialize_weigths(self):
        self.weights = []
    
    def __repr__ (self):
        tmp_weights = ''
        weights = [tmp_weights+ str(item)+'' for item in self.weigths]
        return str(weights)


        
