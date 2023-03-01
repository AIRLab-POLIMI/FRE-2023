import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

from visualization_msgs.msg import MarkerArray, Marker

from pyclustering.cluster import cluster_visualizer
from pyclustering.cluster.kmedoids import kmedoids
from pyclustering.utils import read_sample
from pyclustering.samples.definitions import FCPS_SAMPLES
from pyclustering.utils.metric import distance_metric, type_metric

import numpy as np 

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')


        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 

        #self.pub_centroid_1 = self.create_publisher(Point, '/centroid1', 1)
        #self.pub_centroid_2 = self.create_publisher(Point, '/centroid2', 1)
        
        self.mark_pub = self.create_publisher(MarkerArray, '/centroids', 1)

        self.previous_centroids = None
        
    def scan_callback(self, scan_msg):
        
        points_2d = self.laser_scan_to_cartesian(scan_msg)

        if self.previous_centroids is None:
            initial_medoids = np.array([180, 520])
        else:
            initial_medoids = self.previous_centroids 


        metric = distance_metric(type_metric.EUCLIDEAN_SQUARE)
        k=2   
        kmedoids_instance = kmedoids(points_2d, initial_medoids, metric=metric)

        kmedoids_instance.process()
        medoids = kmedoids_instance.get_medoids()
        self.previous_centroids = medoids

        self.visualize_marker(points_2d[medoids[0]], points_2d[medoids[1]])
    

        # right_point = Point()
        # right_point.x = points_2d[medoids[0]][0]
        # right_point.y = points_2d[medoids[0]][1]
        # right_point.z = 0.0
        # self.pub_centroid_1.publish(right_point)

        # left_point = Point()
        # left_point.x = points_2d[medoids[1]][0]
        # left_point.y = points_2d[medoids[1]][1]
        # left_point.z = 0.0
        # self.pub_centroid_2.publish(left_point)


    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=msg.angle_increment) 

        x = ranges * np.cos(angles) 
        y = ranges * np.sin(angles) 
        
        
        return np.vstack((x, y)).T

    def visualize_marker(self, point1, point2):
        print("publishing")
        marker_msgs = MarkerArray()

        marker1 = Marker()
        marker1.id = 1
        marker1.header.frame_id = "laser_frame"
        marker1.type = marker1.SPHERE
        marker1.action = marker1.ADD
        marker1.scale.x = 0.05
        marker1.scale.y = 0.05
        marker1.scale.z = 0.05
        marker1.color.a = 1.0
        marker1.color.r = 0.0
        marker1.color.g = 0.0
        marker1.color.b = 1.0
        marker1.pose.orientation.w = 1.0
        marker1.pose.position.x = point1[0]
        marker1.pose.position.y = point1[1]
        marker1.pose.position.z = 0.0
        marker_msgs.markers.append(marker1)

        marker2 = Marker()
        marker2.id = 2
        marker2.header.frame_id = "laser_frame"
        marker2.type = marker2.SPHERE
        marker2.action = marker2.ADD
        marker2.scale.x = 0.05
        marker2.scale.y = 0.05
        marker2.scale.z = 0.05
        marker2.color.a = 1.0
        marker2.color.r = 0.0
        marker2.color.g = 0.0
        marker2.color.b = 1.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = point2[0]
        marker2.pose.position.y = point2[1]
        marker2.pose.position.z = 0.0
        marker_msgs.markers.append(marker2)
        
        self.mark_pub.publish(marker_msgs)
        




def main(args=None):
    rclpy.init(args=args)

    navigation = Navigation()

    rclpy.spin(navigation)

    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    