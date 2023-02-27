import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class LaserReader(Node):
    def __init__(self):
        super().__init__('laser_reader')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.scan_sub # prevent unused variable warning 

    def scan_callback(self, msg):
        print('I\'m hearing a scan message')
    
def main(args=None):
    rclpy.init(args=args)

    laser_reader = LaserReader()

    rclpy.spin(laser_reader)

    laser_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    