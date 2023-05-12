import time
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
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

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
        poseToNavigate.header.frame_id = "map"

        yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        print(yaw)

        staged_from_bf_to_odom = self.get_tf_of_frames("base_footprint", "base_link")


        poseToNavigate.pose.position.x = staged_from_bf_to_odom.transform.translation.x + self.lineDimension*float(coeff)*math.sin(yaw[2]) + self.y_movement*math.sin(yaw[2] - math.pi/2)
        poseToNavigate.pose.position.y = staged_from_bf_to_odom.transform.translation.y + self.lineDimension*float(coeff)*math.cos(yaw[2]) + self.y_movement*math.cos(yaw[2] - math.pi/2)
        poseToNavigate.pose.position.z = 0.0

        poseToNavigate.pose.orientation.x = msg.pose.orientation.x
        poseToNavigate.pose.orientation.y = msg.pose.orientation.y
        poseToNavigate.pose.orientation.z = msg.pose.orientation.z
        poseToNavigate.pose.orientation.w = - msg.pose.orientation.w

        self.goal_pub.publish(poseToNavigate)
        print("goal pubblished")


    #def convert_coordinates(self, in_pose, starting_frame, ending_frame):
    #    time_now = Time()
    #    pose_now = tf2_geometry_msgs.PoseStamped()
    #    pose_now.header.stamp = time_now.to_msg()
    #    pose_now.pose = in_pose
    #    pose_now.header.frame_id = "base_footprint"
    #    tfBuffer = tf2_ros.Buffer()              #TODO SEE BETTER FUNCTIONING
    #    return tfBuffer.transform(pose_now, "odom", timeout=Duration(seconds=5,nanoseconds=0))


    def get_tf_of_frames(self, frameEnd, frameStart):
        first_name_ = frameEnd
        second_name_ = frameStart
        print("Transforming from {} to {}".format(second_name_, first_name_))
        _tf_buffer = Buffer()
        _tf_listener = TransformListener(_tf_buffer, self)
        #the problem should not be the frequency at which the tf is pubblished, or the number of tf between the two frames 
        #the only time it gives no error is tìwhen you pass the same frames (obviously)
        #it gives error even changing the order of the frame in the function call
        #time.sleep(2) maybe it takes sometime to listen
        #https://answers.ros.org/question/397914/tf2-lookup_transform-target_frame-does-not-exist/ page where i found this
        trans = _tf_buffer.lookup_transform(second_name_, first_name_, rclpy.time.Time(), timeout=Duration(seconds=4, nanoseconds=0))
        print(trans)
        return trans
        



def main(args=None):
    rclpy.init(args=args)
    node = TurnerFinal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()