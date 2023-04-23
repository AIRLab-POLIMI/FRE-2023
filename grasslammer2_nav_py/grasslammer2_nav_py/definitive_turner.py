import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from rclpy.time import Time
import math

class TurnerFinal(Node):
    def __init__(self):
        super().__init__('turner_final')
        self.lineDimension = 1.0
        self.current_pose = [0.0, 0.0]
        self.coefficient = 1.0
        self.y_movement = -1.0
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
        poseToNavigate.header.frame_id = "map"

        yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        print(yaw)

        poseToNavigate.pose.position.x = msg.pose.position.x + self.lineDimension*float(coeff)*math.sin(yaw[2]) + self.y_movement*math.sin(yaw[2] - math.pi/2)
        poseToNavigate.pose.position.y = msg.pose.position.y + self.lineDimension*float(coeff)*math.cos(yaw[2]) + self.y_movement*math.cos(yaw[2] - math.pi/2)
        poseToNavigate.pose.position.z = 0.0

        poseToNavigate.pose.orientation.x = msg.pose.orientation.x
        poseToNavigate.pose.orientation.y = msg.pose.orientation.y
        poseToNavigate.pose.orientation.z = msg.pose.orientation.z
        poseToNavigate.pose.orientation.w = - msg.pose.orientation.w

        self.goal_pub.publish(poseToNavigate)
        print("goal pubblished")

def main(args=None):
    rclpy.init(args=args)
    node = TurnerFinal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()