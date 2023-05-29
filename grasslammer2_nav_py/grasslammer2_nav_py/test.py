import numpy as np
from collections import deque
import os
import json
import math
import matplotlib.pyplot as plt
import numpy.ma as ma

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
        print(test)
        test.popleft()
        test.appendleft(1)
        print(test)
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


def test_mq():
        m = 3
        q = 4
        # solve system
        tmp = (m**2)*(q**2) + (1+(m**2))*(+(q**2)-1)
        print(tmp)
        x_pos = (-m*q+ math.sqrt((m**2)*(q**2) + (1+(m**2))*(+(q**2)-1)))/(1+m**2)
        x_neg = (-m*q- math.sqrt((m**2)*(q**2) + (1+(m**2))*(+(q**2)-1)))/(1+m**2)
        # solve equation
        y_pos = m**x_pos + q
        y_neg = m**x_neg + q

        print(x_pos, y_pos, "----------", x_neg, y_neg)

def visualize_single_plot(ax_2,  ax_index, x, y, y2):
        # clear axes
        ax_2[ax_index].clear()

        # creates scatter plot
        ax_2[ax_index].scatter(x, y2, color='blue')
        ax_2[ax_index].plot(x, y, color='green')


def test_plot():
        x = np.linspace(0,3,4)
        y = 0.5 * x + 0.7
        y2 = y + np.random.randint(0,1)
        fig_2, ax_2 = plt.subplots(nrows=1, ncols=2,figsize=(20,8))

        for i in range(2):
             visualize_single_plot(ax_2, i, x, y, y2)
        
        fig_2.canvas.draw()


def test_line():
        line_test = line.Line(5)
        line_test.set_line(2,3)
        for i in range(10):
                line_test.set_line(i,i**2)
                print(line_test)

def test_line_ma():
        line_test = line.Line(5, True, False, 0.5, 5)
        # line_test.set_line(2,3)
        for i in range(10):
                line_test.update_line_parameters(i,i**2)
                # print(line_test)
        line_test.initialize_line()
        for i in range(10):
                line_test.update_line_parameters_checking_threshold(i,i**2)
                #print(line_test)


def test_atan():
        #print(math.atan(0/1))
        print(math.atan2(0,0))

def test_delete():
        array = np.array([[-1,np.inf],[0,0],[-1,1],[1, np.inf]])
        array = array[~np.isinf(array).any(axis=1)]
        print(array)
        # rows, cols = np.where(array == -1)
        # print(rows, cols)
        # print(array[rows,cols])
        # print(array[ma.mask_rows(a).mask])
        # print(a, array)

def get_parameters():
        absolute_path = os.path.abspath('in_row_navigation_config/cornaredo.json')
        # print(absolute_path)
        config_file = open(absolute_path, 'r')
        # dict_config = config_file.read()
        test_json = json.loads(config_file.read())
        #print(dict_config)
        # print(test_json)
        #print(dict_config["in_row_navigation"], dict_config['prediction'], dict_config['moving_avarage'])
        print(test_json['in_row_navigation'])
# calculate_weigths()
# test_atan()
#test_delete()
get_parameters()