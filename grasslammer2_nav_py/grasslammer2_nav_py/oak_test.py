import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from rclpy.time import Time
import math

import open3d

from sensor_msgs.msg import Image, PointCloud2, PointField



from scipy.spatial.transform import Rotation as R

from scipy.spatial import cKDTree
from std_msgs.msg import Header
from ctypes import *
from rclpy.clock import ROSClock
import numpy as np

from sensor_msgs.msg import PointCloud2

import open3d as o3d

#import sensor_msgs.point_cloud2 as pc2

convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


class endOfLinePosePub(Node):
    def __init__(self):
        super().__init__('oak_test')
        self.odom_sub = self.create_subscription(PointCloud2, '/oakd_depth/points', self.pointcloudprocess, 1)
        self.pose_pub = self.create_publisher(PointCloud2, '/points_processed', 1)
        #self.main_loop()
    

    def pointcloudprocess(self, data):

        cloud_data = self.convertCloudFromRosToOpen3d(data)

        xyz = np.asarray(cloud_data.points)

        # Flatten the array and convert to bytes.
        byte_arr = xyz.flatten().tobytes()

        # Create the fields for the PointCloud2 message.
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Create the PointCloud2 message.
        msg = PointCloud2(
            data=byte_arr,
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='camera_optical_frame'),
            fields=fields,
            height=1,
            width=xyz.shape[0],
            is_bigendian=False,
            point_step=12,
            row_step=12*xyz.shape[0],
            is_dense=True,
        )
        print(msg)
        self.pose_pub.publish(msg)






    def convertCloudFromRosToOpen3d(self, ros_cloud):
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        #cloud_data = list(PointCloud2.read_points(ros_cloud, skip_nans=True, field_names = field_names))
        # Check empty
        # open3d_cloud = open3d.geometry.PointCloud()
        cloud_data = list(ros_cloud.data)
        point_step = ros_cloud.point_step
        row_step = ros_cloud.row_step
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Get xyz
        #xyz = [(x,y,z) for x,y,z,rgb in cloud_data ]
        xyz = [(cloud_data[i], cloud_data[i + 1], cloud_data[i + 2]) for i in range(0, len(cloud_data), point_step)]
        points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud = o3d.geometry.PointCloud(points)

        # Convert to ROS PointCloud message
        open3d_cloud_msg = PointCloud2()
        open3d_cloud_msg.header = ros_cloud.header
        open3d_cloud_msg.width = len(open3d_cloud.points)
        open3d_cloud_msg.height = 1
        open3d_cloud_msg.is_bigendian = False
        open3d_cloud_msg.point_step = point_step
        open3d_cloud_msg.row_step = row_step
        open3d_cloud_msg.is_dense = True

        open3d_cloud_msg.fields = []
        field_offset = 0
        for field_name in field_names:
            if(field_name != "rgb"):
                field = PointField()
                field.name = field_name
                field.offset = field_offset
                field.datatype = PointField.FLOAT32  # Assuming all fields are of type FLOAT32
                field.count = 1
                open3d_cloud_msg.fields.append(field)
                field_offset += 4

        open3d_cloud_msg.data = []
        for point in open3d_cloud.points:
            open3d_cloud_msg.data.extend(point.astype(np.float32).tobytes())

        #self.pose_pub.publish(open3d_cloud_msg)

        return open3d_cloud

    def convert_rgb_float_to_tuple(self, rgb):
        r = (rgb >> 16) & 0xff
        g = (rgb >> 8) & 0xff
        b = rgb & 0xff
        return (r, g, b)

    def convert_rgb_uint32_to_tuple(self, rgb):
        r = (rgb >> 16) & 0xff
        g = (rgb >> 8) & 0xff
        b = rgb & 0xff
        return (r, g, b)

    
    def main_loop(self):
        self.pub_a_pose()
        print("Pose Published")


def main(args=None):
    rclpy.init(args=args)
    node = endOfLinePosePub()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()

