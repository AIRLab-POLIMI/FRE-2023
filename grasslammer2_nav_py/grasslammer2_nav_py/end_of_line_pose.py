import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from rclpy.time import Time
import math


class endOfLinePosePub(Node):
    def __init__(self):
        super().__init__('line_end_pub')
        #self.odom_sub = self.create_subscription(Odometry, '/grasslammer_velocity_controller/odom', self.saveOdom, 1)
        self.pose_pub = self.create_publisher(PoseStamped, '/end_of_line_pose', 1)
        self.main_loop()
    

    def pub_a_pose(self):

        time_now = Time()
        pose_to_pub = PoseStamped()

        pose_to_pub.header.stamp = time_now.to_msg()
        pose_to_pub.header.frame_id = "map"

        pose_to_pub.pose.position.x = 3.5
        pose_to_pub.pose.position.y = 11.0
        pose_to_pub.pose.position.z = 0.0

        qt = quaternion_from_euler(0, 0, math.pi/2)
        pose_to_pub.pose.orientation.x = qt[0]
        pose_to_pub.pose.orientation.y = qt[1]
        pose_to_pub.pose.orientation.z = qt[2]
        pose_to_pub.pose.orientation.w = qt[3]



        self.pose_pub.publish(pose_to_pub)

    def main_loop(self):
        self.pub_a_pose()
        print("Pose Published")


def main(args=None):
    rclpy.init(args=args)
    node = endOfLinePosePub()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

