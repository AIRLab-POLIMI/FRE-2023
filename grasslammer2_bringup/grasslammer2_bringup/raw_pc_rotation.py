import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
#import sensor_msgs.point_cloud2 as pc2
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np

class CloudTransform(Node):
    def __init__(self):
        super().__init__('cloud_transform')
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            'velodyne_points',
            self.cloud_callback,
            1
        )
        self.transformed_cloud_pub = self.create_publisher(
            PointCloud2,
            'velodyne_points2',
            1
        )

    def cloud_callback(self, msg):

        cloud_points = np.array(list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))))
        #cloud_points = pc2.read_points_numpy(msg, skip_nans=True)

        print(cloud_points[0])
        new_pc = np.array([])
        for pc in cloud_points:
            new_pc = np.vstack(new_pc, np.array([pc], dtype=np.float32))
            print()



        cloud_points = cloud_points.reshape(-1,3)

        ones_column = np.ones((cloud_points.shape[0], 1))
        print(ones_column.shape)
        
        cloud_points_with_ones = np.hstack((cloud_points, ones_column))

        new_cloud_points = transform_points(cloud_points_with_ones)
        
        nmsg = PointCloud2()
        nmsg.header.frame_id = msg.header.frame_id
        nmsg.height = msg.height
        nmsg.width = msg.width
        nmsg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        nmsg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        nmsg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        nmsg.is_bigendian = msg.is_bigendian
        nmsg.point_step = msg.point_step
        nmsg.row_step = msg.row_step
        nmsg.is_dense = msg.is_dense

        # Populate the point cloud data
        points = new_cloud_points
        nmsg.data = bytearray()
        for p in points:
            nmsg.data += bytearray(struct.pack('fff', p[0], p[1], p[2]))


        self.transformed_cloud_pub.publish(nmsg)

        #coord = self.laser_scan_to_points(msg) 
        #new_coord = self.transform_points(coord, transform_stamped)

        #self.transformed_scan_pub.publish(self.points_to_scan(new_coord, msg))


    def transform_points(self, points): 
     
        angle = np.pi/2

        R = np.array([  [np.cos(angle), -np.sin(angle), 0, 0],
                        [np.sin(angle), np.cos(angle), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1],
                        ])

        new_points = R @ points.T
    
        return new_points.T







def main(args=None):
    rclpy.init(args=args)

    node = CloudTransform()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()