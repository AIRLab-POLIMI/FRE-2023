import numpy as np
from collections import deque
import os
number_points = 3
start=0
stop=2
def calculate_weigths():
        values = np.arange(start,stop, step=(stop-start)/number_points)
        print(values)
        # linear -> values_function = [value for value in values]
        # exp
        # values_function = [np.exp(value) for value in values]
        values_function = [1 for _ in values]
        values_sum = np.sum(values_function)
        print(values_sum)
        normalized_weigths = [value/values_sum for value in values_function]
        normalized_weigths.reverse()    
        print(normalized_weigths, np.sum(normalized_weigths))
        return normalized_weigths


def test_deque():
        test = deque([1,2,3,4,5])
        test.popleft()
        test.appendleft(1)
        print(test)

def test_numpy_comparison():
        A = [[1, 2, 3],[4, 5, 6]]
        B = [-1, 4]
        a_np = np.array(A)
        # b_np = np.array(B)

def test_numpy_matrices():
        A = []
        A.append([1,2])
        A.append([3,4])
        print(A[0], A[1])
        print(A[0][0], A[1][0])

def test_folder():
        data_analysis_path = os.path.abspath("data_analysis")
        performance_file = open(data_analysis_path+"/data_analysis_max_trials.csv", "x")
        print(performance_file)

# calculate_weigths()
# test_deque()

# test_numpy_comparison()

# test_numpy_matrices()

test_folder()