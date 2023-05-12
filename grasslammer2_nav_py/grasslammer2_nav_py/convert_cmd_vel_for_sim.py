import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import TransitionEvent

class CMDConverter(Node):

    def __init__(self):
        super().__init__('cmd_vel_converter')
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 1)
        self.switcher = self.create_subscription(PoseStamped, '/end_of_line_pose', self.switch, 1)
        self.cmd_vel_unstamped_pub = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 1)
        self.turning = True 

    
    def callback(self, msg):
        #msg = Twist()
        #fields = self.cmd_vel_sub.topic.find()
        if(self.turning):
            self.cmd_vel_unstamped_pub.publish(msg=msg)
    
    # TO DO
    def switch(self, msg):
        self.turning = True 

    

def main(args=None):
    rclpy.init(args=args)
    node = CMDConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()