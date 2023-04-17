import rclpy
from rclpy.node import Node
import numpy as np
from introcs import Vector3
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from simple_pid import PID

P = 10
I = 0
D = 0

Wmax = 2.0

pid_params = PID(P, I, D)
pid_params.output_limits = (-Wmax, Wmax) 

class PIDControl(Node):

    def __init__(self):
        super().__init__('pid')
        self.publisher_ = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(PoseStamped,'move_base_simple/goal',self.listener_callback,10)
        self.subscription 

    def listener_callback(self, goal):
        theta = math.atan(goal.pose.position.y/goal.pose.position.x)
        pid_params.setpoint = theta
        omega = pid_params(0)
        vel = Twist()
        vel.linear.x = 1.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = omega
        self.publisher_.publish(vel)




def main(args=None):
    rclpy.init(args=args)

    pid = PIDControl()


    rclpy.spin(pid)

    pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
