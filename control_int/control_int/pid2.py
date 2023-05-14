import rclpy
from rclpy.node import Node
import numpy as np
from introcs import Vector3
import math
import argparse

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion

from simple_pid import PID

parser = argparse.ArgumentParser(description = "PID Controller")
parser.add_argument('--p', type =float, default = 10.0)
parser.add_argument('--i', type =float, default = 0.0)
parser.add_argument('--d', type =float, default = 0.0)
parser.add_argument('--v', type =float, default = 1.0)
parser.add_argument('--t', type =float, default = 3.14/4)

args = parser.parse_args()

P = args.p
I = args.i
D = args.d
V_linear = args.v
theta_target = args.t
Wmax = 2.0

pid_params = PID(P, I, D)
pid_params.output_limits = (-Wmax, Wmax) 

class PIDControl(Node):

    def __init__(self):
        super().__init__('pid')
        self.publisher_ = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 10)
        self.publishertarget_ = self.create_publisher(Float64, '/theta_target', 10)
        self.publishertheta_ = self.create_publisher(Float64, '/theta', 10)
        self.subscription = self.create_subscription(Odometry,'/ground_truth',self.listener_callback,10)
        self.subscription 

    def listener_callback(self, fb):
        #theta = math.atan(fb.pose.pose.position.y/fb.pose.pose.position.x)
        lists = [fb.pose.pose.orientation.x,fb.pose.pose.orientation.y,fb.pose.pose.orientation.z,fb.pose.pose.orientation.w]
        euler = euler_from_quaternion(lists)
        theta = euler[2]
        pid_params.setpoint = theta_target
        omega = pid_params(theta)
        vel = Twist()
        target = Float64()
        current = Float64()
        current.data = theta
        target.data = theta_target
        vel.linear.x = V_linear
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = omega
        self.publisher_.publish(vel)
        self.publishertarget_.publish(target)
        self.publishertheta_.publish(current)



def main(args=None):
    rclpy.init(args=args)

    pid = PIDControl()


    rclpy.spin(pid)

    pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()