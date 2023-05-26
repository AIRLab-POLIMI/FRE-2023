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

class TurnerFinal(Node):
    def __init__(self):
        super().__init__('turner_final')
        self.lineDimension = 0.70
        self.y_movement = -0.75
        self.turnNum = 0;
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.starting_pose_sub = self.create_subscription(PoseStamped, '/end_of_line_pose', self.elaborate_goal_point, 1)
        self.done = self.create_publisher(Bool, '/end_of_turning', 1)
        pkg_path = os.path.realpath("workspace/ros2/src/grasslammer2/grasslammer2_description")
        with open(pkg_path + "/config/pathTask1.txt") as path:
            self.turningCommands = path.readlines()
        print(self.turningCommands)
        self.navigator = BasicNavigator()
    
    def elaborate_goal_point(self, msg):

        time_now = Time()
        poseToNavigate = PoseStamped()

        turningInfo = self.turningCommands[self.turnNum]
        self.turnNum += 1
        
        staged_from_bf_to_odom = self.get_tf_of_frames("base_footprint", "map")

        #yawpose = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        yawtf = euler_from_quaternion([staged_from_bf_to_odom.transform.rotation.x, staged_from_bf_to_odom.transform.rotation.y, staged_from_bf_to_odom.transform.rotation.z, staged_from_bf_to_odom.transform.rotation.w])
        print(yawtf)

        yaw = yawtf[2] + math.pi/2

        if(turningInfo[1] == "L"):
            if(yaw <= 1.57): #wrong but solve first the yaw not visible problem
                coeff = float(turningInfo[0])
                print(coeff)
            else:
                coeff = -float(turningInfo[0])
                print(coeff)
        else :
            if(yaw <=  1.57):
                coeff = -float(turningInfo[0])
                print(coeff)

            else:
                coeff = float(turningInfo[0])
                print(coeff)



        poseToNavigate.header.stamp = time_now.to_msg()
        poseToNavigate.header.frame_id = "map"
        

        qt = quaternion_from_euler(-(yawtf[0]), -(yawtf[1]), -yaw)



        poseToNavigate.pose.position.x = staged_from_bf_to_odom.transform.translation.x + self.lineDimension*float(coeff)*math.cos(      yaw      ) + self.y_movement*math.cos(math.pi/2 - yaw)
        poseToNavigate.pose.position.y = staged_from_bf_to_odom.transform.translation.y + self.lineDimension*float(coeff)*math.cos(math.pi/2 - yaw) + self.y_movement*math.cos(      yaw      )
        poseToNavigate.pose.position.z = 0.0


        poseToNavigate.pose.orientation.x = qt[0]
        poseToNavigate.pose.orientation.y = qt[1]
        poseToNavigate.pose.orientation.z = qt[2]
        poseToNavigate.pose.orientation.w = qt[3]

        self.navigator.goToPose(poseToNavigate)
        print(staged_from_bf_to_odom.transform.translation.x, staged_from_bf_to_odom.transform.translation.y)
        print(poseToNavigate.pose.position.x, poseToNavigate.pose.position.y)
        print("goal pubblished")


        while not self.navigator.isTaskComplete():
            continue

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            mess = Bool()
            mess.data = True
            self.done.publish(mess)
            print("Goal Succeded")
        else:
            if result == TaskResult.FAILED:
                print("Failed To Reach The Goal")
                




    def get_tf_of_frames(self, frameEnd, frameStart):
        first_name_ = frameEnd
        second_name_ = frameStart
        print("Transforming from {} to {}".format(second_name_, first_name_))
        #the problem should not be the frequency at which the tf is pubblished, or the number of tf between the two frames 
        #the only time it gives no error is tìwhen you pass the same frames (obviously)
        #it gives error even changing the order of the frame in the function call
        #time.sleep(2) maybe it takes sometime to listen
        #https://answers.ros.org/question/397914/tf2-lookup_transform-target_frame-does-not-exist/ 
        #page where i found this
        trans = self._tf_buffer.lookup_transform(second_name_, first_name_, rclpy.time.Time(), timeout=Duration(seconds=4, nanoseconds=0))
        return trans
        



def main(args=None):
    rclpy.init(args=args)
    node = TurnerFinal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()