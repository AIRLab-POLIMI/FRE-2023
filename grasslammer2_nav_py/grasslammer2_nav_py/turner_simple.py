import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg._pose_stamped import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse
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
        self.lineDimension = 0.75
        self.y_movement = 0.20
        self.turnNum = 0
        self.HOffset = 0
        self.VOffset = 0.75
        self.side = "right"
        self._tf_buffer = Buffer()
        self.turningInput = None
        self.turningCommands = []
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.starting_pose_sub = self.create_subscription(PoseStamped, '/end_of_line_pose', self.elaborate_goal_point, 1)
        self.done = self.create_publisher(Bool, '/end_of_turning', 1)
        self.rviz_viewer = self.create_publisher(PoseStamped, '/waypoint_pose', 1)
        pkg_path = os.path.realpath("src/FRE-2023/grasslammer2_description")
        with open(pkg_path + "/config/pathTask1.txt") as path:
            self.turningInput = path.readline().split(" - ")
        for i in range(1, len(self.turningInput)):
            self.turningCommands.append([self.turningInput[i][:-1], self.turningInput[i][-1]])
        print(self.turningCommands)
        self.navigator = BasicNavigator()
    
    def elaborate_goal_point(self, msg):

        time_now = Time()
        poseToNavigate = PoseStamped()

        turningInfo = self.turningCommands[self.turnNum]
        if(turningInfo[0] == "F"):
            self.get_logger().info("Navigation Ended")
            return
        self.turnNum += 1

        if(turningInfo[1] == "L"):
            self.side = "left"
        else:
            self.side = "right"

        yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        #(x, y, theta) = self.turning(msg.pose.position.x, msg.pose.position.y, yaw[2], int(turningInfo[0]), self.side)


        waypoints = self.elaborateWayPointedPath(msg.pose.position.x, msg.pose.position.y, yaw[2], int(turningInfo[0]), self.side)


        for i in range(0, len(waypoints)):
            poseToNavigate.header.stamp = time_now.to_msg()
            poseToNavigate.header.frame_id = "map"
            

            qt = quaternion_from_euler(0, 0, waypoints[i][2])

            poseToNavigate.pose.position.x = waypoints[i][0]
            poseToNavigate.pose.position.y = waypoints[i][1]
            poseToNavigate.pose.position.z = 0.0

            poseToNavigate.pose.orientation.x = qt[0]
            poseToNavigate.pose.orientation.y = qt[1]
            poseToNavigate.pose.orientation.z = qt[2]
            poseToNavigate.pose.orientation.w = qt[3]
            self.rviz_viewer.publish(poseToNavigate)
            self.navigator.goToPose(poseToNavigate)
            self.get_logger().info("Goal Pubblished")


            while not self.navigator.isTaskComplete():
                continue

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if(i == len(waypoints)-1):
                    mess = Bool()
                    mess.data = True
                    self.done.publish(mess)
                    self.get_logger().info("Final Goal Succeded")
                else:    
                    self.get_logger().info("Goal N°" + str(i) + " Succeded")
            else:
                if result == TaskResult.FAILED:
                    self.get_logger().info("Failed To Reach The Goal")
                    




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
        
    def turning(self, x_s, y_s, theta_s, num_rows, side):
        """
        Turning method which takes as input a starting pose and returns the corresponding turning final pose to be
        exploited as goal point in Nav2.

        :param x_s:
            x-coordinate of the robot expressed in the /odom reference system

        :param y_s:
            x-coordinate of the robot expressed in the /odom reference system

        :param theta_s:
            heading of the robot expressed in the /odom reference system

        :param num_rows:
            number of the rows to be skipped by the robot before the proper turning

        :param side:
            which side to be used for the turning ("left" or "right")

        :return:
            final pose x_f, y_f, theta_f
        """
        # pre-conditions
        assert num_rows > 0, "Invalid number of maize rows! It must be greater than zero."
        assert side in ["left", "right"], "Invalid turning side! Please choose between: ['left', 'right']"

        # angle sign direction of turning
        turn = (+1) if side == "left" else (-1)

        # shifting towards maize row width
        theta_g = (theta_s + math.pi / 2 * turn) % (2 * math.pi)
        x_g = x_s + (self.lineDimension * num_rows) * math.cos(theta_g)
        y_g = y_s + (self.lineDimension * num_rows) * math.sin(theta_g)

        # correcting the position
        theta_f = (theta_g + math.pi / 2 * turn) % (2 * math.pi)
        x_f = x_g + self.y_movement * math.cos(theta_f)
        y_f = y_g + self.y_movement * math.sin(theta_f)

        # final pose
        return x_f, y_f, theta_f


    def calculatePoseWithOffset(self, x_s, y_s, theta_s, HOffset, VOffset, thetaOffset):
        
        x_g = x_s + (HOffset) * math.sin(theta_s)
        y_g = y_s + (HOffset) * math.cos(theta_s)

        # correcting the position
        theta_f  = theta_s + thetaOffset
        x_f = x_g + VOffset * math.cos(theta_s)
        y_f = y_g + VOffset * math.sin(theta_s)

        # final pose
        return x_f, y_f, theta_f


    def elaborateWayPointedPath(self, x_s, y_s, theta_s, num_rows, side):
        turn = -1 if side == "left" else 1
        (x_last, y_last, theta_last) = self.turning(x_s, y_s, theta_s, num_rows, side)
        (x_si, y_si, theta_si) = self.calculatePoseWithOffset(x_s, y_s, theta_s, self.HOffset, self.VOffset, 0 if self.HOffset == 0 else (-math.pi/2)*turn)
        (x_ff, y_ff, theta_ff) = self.calculatePoseWithOffset(x_last, y_last, theta_last, -self.HOffset, -self.VOffset, 0 if self.HOffset == 0 else (math.pi/2)*turn)
        if(self.HOffset == 0):
            (x_sii, y_sii, theta_sii) = self.calculatePoseWithOffset(x_si, y_si, theta_si, 0, 0, (-math.pi/2)*turn)
            (x_fff, y_fff, theta_fff) = self.calculatePoseWithOffset(x_ff, y_ff, theta_ff, 0, 0, (math.pi/2)*turn)

            return [(x_si, y_si, theta_si), (x_sii, y_sii, theta_sii), (x_fff, y_fff, theta_fff), (x_ff, y_ff, theta_ff), (x_last, y_last, theta_last)] 

        return [(x_si, y_si, theta_si), (x_ff, y_ff, theta_ff), (x_last, y_last, theta_last)] 
 


def main(args=None):
    rclpy.init(args=args)
    node = TurnerFinal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

