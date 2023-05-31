import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg._pose_stamped import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.time import Time
import math
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import os
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool

class TurnerFromLine(Node):
    def __init__(self):
        super().__init__('turner_from_line')
        self.lineDimension = 0.75
        self.type = "from_tf"
        self.y_movement = 0.2
        self.turnNum = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.starting_pose_sub = self.create_subscription(PoseStamped, '/end_of_line_pose', self.elaborate_goal_point, 10)
        self.end_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.done = self.create_publisher(Bool, '/end_of_turning', 10)
        pkg_path = os.path.realpath("src/FRE-2023/grasslammer2_description")
        with open(pkg_path + "/config/pathTask1.txt") as path:
            self.turningCommands = path.readlines()
        print(self.turningCommands)
        self.navigator = BasicNavigator()
    
    def elaborate_goal_point(self, msg):
        #end_of_line_pose need to be in the map frame (I think) because I cannot know the old tf from base_footprint to map 
        #maybe is even needed in the odom frame, the map frame may have changed (Alba passes a pose from the past!!!) <--I think is this one because Odom is a fixed frame in time

        time_now = Time()
        end_of_line_pose = PoseStamped()
        end_of_line_pose.header = msg.header
        end_of_line_pose.pose = msg.pose
        turning_info = self.turningCommands[self.turnNum]
        self.turnNum += 1
        #get the supposed angle of the rows end line
        perpendicular_line_angle = end_of_line_pose.pose.orientation.x - math.pi/2 #<-----------------------------
        if(turning_info[1] == "L"):
            perpendicular_line_angle += math.pi



        #calculate goal pose in odom frame
        goal_pose_odom = PoseStamped()
        goal_pose_odom.header.stamp = time_now.to_msg()
        goal_pose_odom.header.frame_id = end_of_line_pose.header.frame_id

        goal_pose_odom.pose.position.x = end_of_line_pose.pose.position.x + math.cos(perpendicular_line_angle)*float(turning_info[0]) * self.lineDimension
        goal_pose_odom.pose.position.y = end_of_line_pose.pose.position.y + math.sin(perpendicular_line_angle)*float(turning_info[0]) * self.lineDimension
        goal_pose_odom.pose.position.z = 0.0

        goal_pose_odom.pose.orientation.x = end_of_line_pose.pose.orientation.x + math.pi
        goal_pose_odom.pose.orientation.y = 0.0
        goal_pose_odom.pose.orientation.z = 0.0
        goal_pose_odom.pose.orientation.w = 1.0
        if goal_pose_odom.pose.orientation.z > math.pi:
            goal_pose_odom.pose.orientation.z -= 2*math.pi  #angle must be between -pi and pi
        #self.end_pose_pub.publish(goal_pose_odom)


        #transform from of the received odom to the current map
        transform = self._tf_buffer.lookup_transform('map', goal_pose_odom.header.frame_id, goal_pose_odom.header.stamp, Duration(seconds=4, nanoseconds=0))
        goal_pose_final = PoseStamped()
        goal_pose_final.header.stamp = time_now.to_msg()
        goal_pose_final.header.frame_id = "map"

        goal_pose_final.pose.position.x = goal_pose_odom.pose.position.x - transform.transform.translation.x
        goal_pose_final.pose.position.y = goal_pose_odom.pose.position.y - transform.transform.translation.y
        goal_pose_final.pose.position.z = goal_pose_odom.pose.position.z - transform.transform.translation.z
        goal_pose_final.pose.orientation.z = 0.0
        goal_pose_final.pose.orientation.w = 1.0
        goal_pose_final.pose.orientation.y = 0.0
        goal_pose_final.pose.orientation.x = goal_pose_odom.pose.orientation.x + transform.transform.rotation.x
        if(goal_pose_final.pose.orientation.x > math.pi):
            goal_pose_final.pose.orientation.x -= 2*math.pi
        elif(goal_pose_final.pose.orientation.x < -math.pi):
            goal_pose_final.pose.orientation.x += 2*math.pi
        self.end_pose_pub.publish(goal_pose_final)

        
        self.navigator.goToPose(goal_pose_final)
        print("starting from: ",end_of_line_pose.header.frame_id, " ", end_of_line_pose.pose.position.x, " ", end_of_line_pose.pose.position.y)
        print("going to: ", goal_pose_final.header.frame_id," ", goal_pose_final.pose.orientation.x," ", goal_pose_final.pose.orientation.y)
        print("goal pubblished")


        """while not self.navigator.isTaskComplete():
            continue

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            mess = Bool()
            mess.data = True
            self.done.publish(mess)
            print("Goal Succeded")
        else:
            if result == TaskResult.FAILED:
                print("Failed To Reach The Goal")"""


def main(args=None):
    rclpy.init(args=args)
    node = TurnerFromLine()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()