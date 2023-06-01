import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class YolotorTester(Node):
    def __init__(self):
        """
        Simple tester for Yolotor's topic subscription.
        """
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(String, 'yolotor', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('I am listening...')

    def listener_callback(self, msg):
        """
        Yolotor topic callback.

        Parameters
        ----------
            - msg:
                String msg for Yolotor prediction
        """
        self.get_logger().info('I detected: "%s"' % msg.data)


rclpy.init()

tester = YolotorTester()
rclpy.spin(tester)
tester.destroy_node()

rclpy.shutdown()
