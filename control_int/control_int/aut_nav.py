import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import MarkerArray, Marker


from std_msgs.msg import Float64MultiArray

import math

import numpy as np
import matplotlib.pyplot as plt
import argparse


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        self.goal_point_sub = self.create_subscription(Float64MultiArray, '/goal_position', self.goal_callback, 1)
        self.goal_point_sub #prevent unused variable warning 

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_row_nav', 1)

        self.a = 4.0
        self.b = 1.0

    def goal_callback(self, goal):

        cmd_msg = Twist()

        theta = goal.data[2]


        cmd_msg.linear.x = self.b * pow(math.cos(theta),4)
        if abs(theta) > math.pi/5:
        	if theta > 0:
        		theta = math.pi/5
        	else:
        		theta = -math.pi/5
            
        cmd_msg.angular.z = self.a * theta # math.sin(theta)

        self.cmd_pub.publish(cmd_msg)
  

        
def main(args=None):
    rclpy.init(args=args)

    navigation = Navigation()

    rclpy.spin(navigation)

    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
