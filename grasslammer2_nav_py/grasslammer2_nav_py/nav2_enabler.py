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
        self.cmd_vel_unstamped_pub = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 1)
    
    def callback(self, msg):
        #converts nav2 cmd_vel to cmd_vel_unstamped
        self.cmd_vel_unstamped_pub.publish(msg=msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = CMDConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()