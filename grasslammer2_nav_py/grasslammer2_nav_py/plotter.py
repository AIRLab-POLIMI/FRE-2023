import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
from std_msgs.msg import UInt8
import numpy as np
import pcl


class plotter_input(Node):
    def __init__(self):
        super().__init__('plotter')
        self.cv_bridge = cv_bridge.CvBridge()
        self.oakd_sub = self.create_subscription(Image, "/oakd_depth/depth/image_raw", self.depth_image_callback, 10)
        #self.points_pub = self.create_publisher(UInt8, "/points_to_plot_depth", 1)

    def depth_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().info(f'Error converting image: {e}')
            return

        # Access depth pixel values
        depth_values = cv_image

        # Process depth values as needed
        # ...

        # Print the depth values for demonstration
        self.get_logger().info(f'Depth values: {depth_values}')



def main(args=None):
    rclpy.init(args=args)
    node = plotter_input()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()