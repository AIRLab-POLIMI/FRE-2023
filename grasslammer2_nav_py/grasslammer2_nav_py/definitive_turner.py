import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs
from geometry_msgs.msg._pose_stamped import PoseStamped
from tf_transformations import euler_from_quaternion
from rclpy.time import Time
import tf2_ros
import math

class TurnerFinal(Node):
    def __init__(self):
        super().__init__('turner_final')
        self.lineDimension = 0.75
        self.y_movement = -1.5
        self.starting_pose_sub = self.create_subscription(PoseStamped, '/end_of_line_pose', self.elaborate_goal_point, 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
    
    def elaborate_goal_point(self, msg):

        time_now = Time()
        poseToNavigate = PoseStamped()

        turningInfo = input("Select turning input: ").split(" ")

        if(turningInfo[1] == "L"):
            coeff = -float(turningInfo[0])
        else :
            coeff =  float(turningInfo[0])


        poseToNavigate.header.stamp = time_now.to_msg()
        poseToNavigate.header.frame_id = "odom"

        yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        print(yaw)

        staged_from_bf_to_odom = self.convert_coordinates(msg.pose, 'base_footprint', 'odom')

        poseToNavigate.pose.position.x = staged_from_bf_to_odom.pose.position.x + self.lineDimension*float(coeff)*math.sin(yaw[2]) + self.y_movement*math.sin(yaw[2] - math.pi/2)
        poseToNavigate.pose.position.y = staged_from_bf_to_odom.pose.position.y + self.lineDimension*float(coeff)*math.cos(yaw[2]) + self.y_movement*math.cos(yaw[2] - math.pi/2)
        poseToNavigate.pose.position.z = 0.0

        poseToNavigate.pose.orientation.x = msg.pose.orientation.x
        poseToNavigate.pose.orientation.y = msg.pose.orientation.y
        poseToNavigate.pose.orientation.z = msg.pose.orientation.z
        poseToNavigate.pose.orientation.w = - msg.pose.orientation.w

        self.goal_pub.publish(poseToNavigate)
        print("goal pubblished")


    def convert_coordinates(self, in_pose, starting_frame, ending_frame):
        time_now = Time()
        pose_now = tf2_geometry_msgs.PoseStamped()
        pose_now.header.stamp = time_now.to_msg()
        pose_now.pose = in_pose
        pose_now.header.frame_id = starting_frame
        tfBuffer = tf2_ros.Buffer()              #TODO SEE BETTER FUNCTIONING
        try:
            return tfBuffer.transform(pose_now, ending_frame, timeout=Duration(seconds=5,nanoseconds=0))
        except:
            print("trasform to " + ending_frame + "not endend well")


def main(args=None):
    rclpy.init(args=args)
    node = TurnerFinal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()