import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np

class LaserScanTransform(Node):
    def __init__(self):
        super().__init__('laser_scan_transform')
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            'scan_raw',
            self.laser_scan_callback,
            1
        )
        self.transformed_scan_pub = self.create_publisher(
            LaserScan,
            'scan',
            1
        )

    def laser_scan_callback(self, msg):

        ranges = np.array(msg.ranges)
        # Take the last 810 elements
        last_elements = ranges[:810]

        # Take the remaining elements except the last 810
        remaining_elements = ranges[810:]

        # Concatenate the last elements at the beginning
        shifted_arr = np.concatenate((remaining_elements, last_elements))

        number_increments = 410

        r_indexes = np.arange(0, int(number_increments/2))
        l_indexes = np.arange(3240-int(number_increments/2), 3240)
        shifted_arr[r_indexes] = 'nan'
        shifted_arr[l_indexes] = 'nan'

        mask = (shifted_arr[:] < 0.22)
        shifted_arr[mask] = 'nan'



        #print(np.nanmin(shifted_arr))


        nmsg = LaserScan()
        nmsg.header.stamp = msg.header.stamp
        nmsg.header.frame_id = msg.header.frame_id
        nmsg.angle_min = msg.angle_min
        nmsg.angle_max = msg.angle_max
        nmsg.angle_increment = msg.angle_increment
        nmsg.range_min = msg.range_min
        nmsg.range_max = msg.range_max
        nmsg.ranges = shifted_arr.tolist()

        

        self.transformed_scan_pub.publish(nmsg)


def main(args=None):
    rclpy.init(args=args)

    node = LaserScanTransform()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()