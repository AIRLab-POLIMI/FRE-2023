import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray
from tf_transformations import euler_from_quaternion
from numpy import float64


class CMDConverter(Node):

    def __init__(self):
        super().__init__('switcher')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 1)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel_row_nav', self.callback1, 1)
        self.turningOn = self.create_subscription(PoseStamped, '/end_of_line_pose', self.switchOn, 1)
        self.goal_pose_pub = self.create_publisher(Float64MultiArray, '/goal_position', 1)
        self.turningOff = self.create_subscription(Bool, "/end_of_turning", self.switchOff, 1)
        self.cmd_vel_unstamped_pub = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 1)
        self.turning = False 

    
    def callback(self, msg):
        #converts nav2 cmd_vel to cmd_vel_unstamped
        if self.turning:
            self.cmd_vel_unstamped_pub.publish(msg=msg)
    
    def callback1(self, msg):
        #converts controller cmd_vel to cmd_vel_unstamped
        if not self.turning:
            self.cmd_vel_unstamped_pub.publish(msg=msg)
    
    def switchOn(self, msg):
        #switch on the turning part
        # goal = Float64MultiArray()
        # theta = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # x = float64(msg.pose.position.x)
        # y = float64(msg.pose.position.y)
        # angle = float64(theta[2])
        # goal.data = [x, y, angle]
        # self.goal_pose_pub.publish(goal)
        self.turning = True
        print("EndOfRowReached - Switching to Turning")
    
    def switchOff(self, msg):
        #switch off the turning part
        self.turning = False
        print("GoalReached - Switching to RowNavigation")

    

def main(args=None):
    rclpy.init(args=args)
    node = CMDConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()