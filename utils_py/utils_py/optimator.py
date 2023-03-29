import rclpy
from rclpy.node import Node

import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import numpy as np 

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class Optimator(Node):
    def __init__(self):
        super().__init__('optimator')

        self.tf_opti_broadcaster = TransformBroadcaster(self)
        self.tf_odom_broadcaster = TransformBroadcaster(self)


        self.opti_sub = self.create_subscription(Pose2D, '/car/ground_pose', self.opti_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.odom_callback, 1)
        
        self.flag = True
        self.pose_init = (0, 0, 0)


    def opti_callback(self, msg): 
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'opti'

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        if (self.flag):
            self.pose_init = (msg.x, msg.y, 0.0)
            self.flag = False

        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_opti_broadcaster.sendTransform(t)
        


    def odom_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'

        t.transform.translation.x = self.pose_init[0] + msg.pose.pose.position.y
        t.transform.translation.y = self.pose_init[1] + msg.pose.pose.position.x
        t.transform.translation.z = self.pose_init[2] + msg.pose.pose.position.z

        
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_odom_broadcaster.sendTransform(t)
        

        

    
def main(args=None):
    rclpy.init(args=args)

    node = Optimator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    