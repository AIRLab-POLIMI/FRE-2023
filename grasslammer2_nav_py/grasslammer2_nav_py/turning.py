import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time



class Turner(Node):
    def __init__(self):
        super().__init__('turning_controller')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.saveOdom, 1)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.actualPos = [0.0, 0.0]
        self.lineDimension = 0.75
        self.loopOfCommands()


    def saveOdom(self, msg):
        #trasform Odometry message in actualPos array with actualPos[0]=x and so oni
        self.actualPos[0] = float(msg.pose.pose.position.x)
        self.actualPos[1] = float(msg.pose.pose.position.y)


    def waitForCommandString(self):
        turnInfo = input("Insert your command here ---> ").split(" ")

        time_now = Time()
        poseToNavigate = PoseStamped()

        poseToNavigate.header.stamp = time_now.to_msg()
        poseToNavigate.header.frame_id = "map"

        poseToNavigate.pose.position.x = self.actualPos[0] - float(turnInfo[0])*self.lineDimension if turnInfo[1] == "L" else self.actualPos[0] + float(turnInfo[0])*self.lineDimension
        poseToNavigate.pose.position.y = self.actualPos[1] + 3
        poseToNavigate.pose.position.z = 0.0

        poseToNavigate.pose.orientation.x = 0.0
        poseToNavigate.pose.orientation.y = 0.0
        poseToNavigate.pose.orientation.z = 0.0
        poseToNavigate.pose.orientation.w = 1.0

        self.goal_pose_publisher.publish(poseToNavigate)

    def loopOfCommands(self):
        while(input("Do you want to quit: ") != "y"):
            self.waitForCommandString()


def main(args=None):
    rclpy.init(args=args)
    node = Turner()
    rclpy.spin_once(node)
    rclpy.shutdown

if __name__ == '__main__':
    main()