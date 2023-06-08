import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import MarkerArray, Marker


from std_msgs.msg import Float64MultiArray

import math

import numpy as np 
import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser(description = "aut_nav")
parser.add_argument('--a', type =float, default = 3.0)
parser.add_argument('--b', type =float, default = 0.3)
args = parser.parse_args()


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        self.goal_point_sub = self.create_subscription(Float64MultiArray, '/goal_position', self.goal_callback, 1)
        self.goal_point_sub #prevent unused variable warning 

        self.cmd_pub = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 1)
        self.a = args.a
        self.b = args.b

    def goal_callback(self, goal):
        #self.a = args.a # proportional gain on angular velocity
        #self.b = args.b # proportional gain on linear velocity
        cmd_msg = Twist()

        theta = goal.data[2]
        if(theta > math.pi/2 or theta < -math.pi/2):
            if(theta > 0):
                theta = theta - math.pi
            else:
                theta = theta + math.pi
            coef = -1
        else:
            coef = 1

        cmd_msg.linear.x = coef * self.b * math.cos(theta)
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

    