import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time



class PosePublisher(Node):
    def __init__(self):
        super().__init__('turning_controller')
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/end_of_line_pose', 10)

        self.loopOfCommands()


    def saveOdom(self, msg):
        #trasform Odometry message in actualPos array with actualPos[0]=x and so oni
        self.actualPos[0] = float(msg.pose.pose.position.x)
        self.actualPos[1] = float(msg.pose.pose.position.y)


    def waitForCommandString(self):
        turn_info = input("Insert your command here ---> ").split(" ")

        time_now = Time()
        start_pose = PoseStamped()

        start_pose.header.stamp = time_now.to_msg()
        start_pose.header.frame_id = "odom"

        start_pose.pose.position.x = float(turn_info[0])
        start_pose.pose.position.y = float(turn_info[1])
        start_pose.pose.position.z = 0.0

        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.0
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 0.0

        self.goal_pose_publisher.publish(start_pose)

    def loopOfCommands(self):
        while(input("Do you want to quit: ") != "y"):
            self.waitForCommandString()


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin_once(node)
    rclpy.shutdown

if __name__ == '__main__':
    main()