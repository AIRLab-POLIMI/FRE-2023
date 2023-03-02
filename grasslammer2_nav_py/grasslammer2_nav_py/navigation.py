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
import numpy as np
import matplotlib.pyplot as plt


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')


        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 

        #self.pub_centroid_1 = self.create_publisher(Point, '/centroid1', 1)
        #self.pub_centroid_2 = self.create_publisher(Point, '/centroid2', 1)
        
        self.mark_pub = self.create_publisher(MarkerArray, '/centroids', 1)
        self.cluster1_pub = self.create_publisher(MarkerArray, '/cluster1', 1)
        #self.cluster2_pub = self.create_publisher(MarkerArray, '/cluster2', 1)

        self.previous_centroids = None

        self.fig, self.ax = plt.subplots()
        
    def scan_callback(self, scan_msg):
        
        points_2d = self.laser_scan_to_cartesian(scan_msg)
        self.visualize_points_2d(points_2d)
       
        initial_medoids = np.array([0, 1])

        # if self.previous_centroids is None:
            
        # else:
        #     initial_medoids = self.previous_centroids 


        

        if(np.size(points_2d) == 0):
            print('No points found in ROI! ')
        else:
            print('Found ', len(points_2d), ' points in ROI ! ')
            metric = distance_metric(type_metric.EUCLIDEAN_SQUARE)
            k=2   
            kmedoids_instance = kmedoids(points_2d, initial_medoids, metric=metric)

            kmedoids_instance.process()
            clusters = kmedoids_instance.get_clusters()
            medoids = kmedoids_instance.get_medoids()
            self.previous_centroids = medoids

        #self.visualize_marker(points_2d[medoids[0]], points_2d[medoids[1]])

        #self.visualize_clusters(points_2d[clusters[0]], points_2d[clusters[1]])
        


    def laser_scan_to_cartesian(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.arange(start=msg.angle_min, stop=msg.angle_max, step=msg.angle_increment) 


        x = np.where(ranges == -1, -1, ranges * np.cos(angles))
        y = np.where(ranges == -1, -1, ranges * np.sin(angles))
        
        

        points = np.vstack((x, y)).T
        points_filtered = points[y != -1]
    
        return points_filtered

    def visualize_marker(self, point1, point2):
        
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
    
    def visualize_clusters(self, point_cluster1, point_cluster2):
        
        marker_msgs = MarkerArray()
        for i in range(len(point_cluster1)):
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "laser_frame"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point_cluster1[i, 0]
            marker.pose.position.y = point_cluster1[i, 1]
            marker.pose.position.z = 0.0
            marker_msgs.markers.append(marker)

        self.cluster1_pub.publish(marker_msgs)
    
        marker_msgs2 = MarkerArray()
        for i in range(len(point_cluster2)):
            marker2 = Marker()
            marker2.id = i
            marker2.header.frame_id = "laser_frame"
            marker2.type = marker.SPHERE
            marker2.action = marker.ADD
            marker2.scale.x = 0.05
            marker2.scale.y = 0.05
            marker2.scale.z = 0.05
            marker2.color.a = 1.0
            marker2.color.r = 0.0
            marker2.color.g = 1.0
            marker2.color.b = 0.0
            marker2.pose.orientation.w = 1.0
            marker2.pose.position.x = point_cluster2[i, 0]
            marker2.pose.position.y = point_cluster2[i, 1]
            marker2.pose.position.z = 0.0
            marker_msgs2.markers.append(marker2)

        self.cluster2_pub.publish(marker_msgs2)

    def visualize_points_2d(self, points_2d): 
                       
        self.ax.clear()
        plt.scatter(points_2d[:,0], points_2d[:,1])
        plt.xlim(0,3)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)

        # marker_msgs = MarkerArray()
        # for i in range(len(points_2d)):
        #     marker = Marker()
        #     marker.id = i
        #     marker.header.frame_id = "laser_frame"
        #     marker.type = marker.SPHERE
        #     marker.action = marker.ADD
        #     marker.scale.x = 0.05
        #     marker.scale.y = 0.05
        #     marker.scale.z = 0.05
        #     marker.color.a = 1.0
        #     marker.color.r = 0.0
        #     marker.color.g = 1.0
        #     marker.color.b = 0.0
        #     marker.pose.orientation.w = 1.0
        #     marker.pose.position.x = points_2d[i, 0]
        #     marker.pose.position.y = points_2d[i, 1]
        #     marker.pose.position.z = 0.0
        #     marker_msgs.markers.append(marker)

        # self.cluster1_pub.publish(marker_msgs)
        # marker_msgs.markers = []

        
def main(args=None):
    rclpy.init(args=args)

    navigation = Navigation()

    rclpy.spin(navigation)

    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    