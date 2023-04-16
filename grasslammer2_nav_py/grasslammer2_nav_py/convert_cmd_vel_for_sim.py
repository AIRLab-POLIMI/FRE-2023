import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CMDConverter(Node):

    def __init__(self):
        super().__init__('cmd_vel_converter')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 1)
        self.cmd_vel_unstamped_pub = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 1)


    
    def callback(self, msg):
        #msg = Twist()
        #fields = self.cmd_vel_sub.topic.find()
        self.cmd_vel_unstamped_pub.publish(msg=msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = CMDConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()