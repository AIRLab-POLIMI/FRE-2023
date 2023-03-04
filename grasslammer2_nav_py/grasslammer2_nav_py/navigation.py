import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Joy
from geometry_msgs.msg import Point, Twist
from visualization_msgs.msg import MarkerArray, Marker

from pyclustering.cluster import cluster_visualizer
from pyclustering.cluster.kmedoids import kmedoids
from pyclustering.utils import read_sample
from pyclustering.samples.definitions import FCPS_SAMPLES
from pyclustering.utils.metric import distance_metric, type_metric

import math

import numpy as np 
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN, AgglomerativeClustering
from sklearn.linear_model import RANSACRegressor


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        self.NAV = False

        self.scan_sub = self.create_subscription(LaserScan, '/scan/filtered', self.scan_callback, 1)
        self.scan_sub # prevent unused variable warning 

        self.joy_sub = self.create_subscription(Joy, '/joy', self.navigation_active, 1)

        self.cmd_pub = self.create_publisher(Twist, '/grasslammer_velocity_controller/cmd_vel_unstamped', 1)
        
        #self.mark_pub = self.create_publisher(MarkerArray, '/centroids', 1)
        #self.cluster1_pub = self.create_publisher(MarkerArray, '/cluster1', 1)
        #self.cluster2_pub = self.create_publisher(MarkerArray, '/cluster2', 1)

        self.previous_centroids = None
        self.clustering_db = DBSCAN(eps= 0.5, min_samples=3)
        self.clustering_AG = AgglomerativeClustering(n_clusters=2, linkage='single')
        self.ransac = RANSACRegressor()

        self.fig, self.ax = plt.subplots()
        
        
    def scan_callback(self, scan_msg):
        
        # Transform laser scan msg of ROI points into cartesian coordinate
        points_2d = self.laser_scan_to_cartesian(scan_msg)
       
        

        if(np.size(points_2d) < 15):
            # Used to prevent crash from no points detected in the region of interest
            print('No points found in ROI! ')
            self.NAV = False
        else:

            # Use Kmedoids algorithm
            #self.algorithm_kmedoids(points_2d)
            
            # Use least square regression
            #self.algorithm_lsqr(points_2d)

            # Use DBSCAN
            #self.dbscan(points_2d)

            # Use RANSAC
            self.RANSAC(points_2d)
            
            # Use agglomerative clustering
            #clusters = self.agclustering(points_2d)

            #goal = self.goal_point_cluster(clusters)

            #self.visualize_matplot_DBSCAN(clusters[0], clusters[1], points_2d, goal)

            

            #print('Navigation: Active' if self.NAV else 'Navigation: Stopped')

            #self.heading(goal)

        
        
        # Visualization using markers on Rviz
        #self.visualize_marker(points_2d[medoids[0]], points_2d[medoids[1]])
        #self.visualize_clusters(points_2d[clusters[0]], points_2d[clusters[1]])
        
    def algorithm_kmedoids(self, points_2d):
        print('Found ', len(points_2d), ' points in ROI ! ') 

        # Medoids initialization
        initial_medoids = np.array([0, 1])
            
        # Set medoids parameter and instance an object
        metric = distance_metric(type_metric.EUCLIDEAN_SQUARE)
        k=2   
        kmedoids_instance = kmedoids(points_2d, initial_medoids, metric=metric)

        # Run the algorithm 
        kmedoids_instance.process()
        clusters = kmedoids_instance.get_clusters()
        # Bidimensional array, in [0,:] there are index of points belonging to cluster one
        medoids = kmedoids_instance.get_medoids()
        # Bidimensional array, in [0] there is the index of first medoid, in [1] index of 2nd medoid
        
        self.previous_centroids = medoids

        #Visualization using Matplotlib
        self.visualize_matplot_medoids(points_2d, points_2d[clusters[0]], points_2d[clusters[1]], points_2d[medoids[0]], points_2d[medoids[1]])

    def algorithm_lsqr(self, points):
        # I take all the points in the ROI and I perform a linear regression to separate points 
        x = points[:,0]
        y = points[:,1]
        A = np.vstack([x, np.ones(len(x))]).T
        m, c = np.linalg.lstsq(A, y, rcond=None)[0]

        # Lsqr on lower points
        points_minor = points[np.where(points[:,1] < m*x+c)]
        y_minor = points_minor[:,1]
        x_minor = points_minor[:,0]
        A_minor = np.vstack([x_minor, np.ones(len(x_minor))]).T
        m_minor, c_minor = np.linalg.lstsq(A_minor, y_minor, rcond=None)[0]

        # Lsqr on higher points
        points_greater = points[np.where(points[:,1] > m*x+c)]
        y_greater = points_greater[:,1]
        x_greater = points_greater[:,0]
        A_greater = np.vstack([x_greater, np.ones(len(x_greater))]).T
        m_greater, c_greater = np.linalg.lstsq(A_greater, y_greater, rcond=None)[0]



        self.visualize_matplot_lsqr(x, m, c, points, x_minor, y_minor, m_minor, c_minor, x_greater, y_greater, m_greater, c_greater)

    def agclustering(self, points):
        clusters = self.clustering_AG.fit(points)

        cluster1 = points[np.where(clusters.labels_ == 0)]
        cluster2 = points[np.where(clusters.labels_ == 1)]

        # print(clusters.labels_)
        
        
        
        return [cluster1, cluster2]

    def RANSAC(self, points):
        lines = []
        row1 = points[np.where( points[:, 1] < 0)]
        row2 = points[np.where( points[:, 1] > 0)]

        self.ransac.fit(row1[:, 0].reshape(-1, 1), row1[:, 1])
        slope = self.ransac.estimator_.coef_[0]
        intercept = self.ransac.estimator_.intercept_
        lines.append((slope, intercept))

        self.ransac.fit(row2[:, 0].reshape(-1, 1), row2[:, 1])
        slope = self.ransac.estimator_.coef_[0]
        intercept = self.ransac.estimator_.intercept_
        lines.append((slope, intercept))

        self.visualize_ransac(points, lines)

    def dbscan(self, points):
        clusters = self.clustering_db.fit(points)

        cluster1 = points[np.where(clusters.labels_ == 0)]
        cluster2 = points[np.where(clusters.labels_ == 1)]

        print(clusters.labels_)

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
    
    def visualize_marker_points(self, points_2d):
    
        marker_msgs = MarkerArray()
        for i in range(len(points_2d)):
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
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = points_2d[i, 0]
            marker.pose.position.y = points_2d[i, 1]
            marker.pose.position.z = 0.0
            marker_msgs.markers.append(marker)

        self.cluster1_pub.publish(marker_msgs)
        marker_msgs.markers = []

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

    def visualize_matplot_medoids(self, points_2d, point_cluster1, point_cluster2, medoid1, medoid2):
        self.ax.clear()
        #plt.scatter(points_2d[:, 0], points_2d[:, 1], color='blue')
        plt.scatter(point_cluster1[:, 0], point_cluster1[:, 1], color='red')
        plt.scatter(point_cluster2[:, 0], point_cluster2[:, 1], color='green')
        plt.scatter(medoid1[0], medoid1[1], color='blue')
        plt.scatter(medoid2[0], medoid2[1], color='blue')
        plt.xlim(0,3)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)

    def visualize_matplot_lsqr(self, x, m, c, points, x_minor, y_minor, m_minor, c_minor, x_greater, y_greater, m_greater, c_greater):
        self.ax.clear()
        #plt.scatter(points[:, 0], points[:, 1], color='blue')
        plt.scatter(x_minor, y_minor, color='green')
        plt.plot(x_minor, m_minor*x_minor + c_minor, color='g')
        plt.scatter(x_greater, y_greater, color='b')
        plt.plot(x_greater, m_greater*x_greater + c_greater, color='b')
        #plt.plot(x, m*x + c, color='r')
        plt.xlim(0,3)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)
        
    def visualize_matplot_DBSCAN(self, cluster1, cluster2, points, goal):
        self.ax.clear()
        plt.scatter(points[:, 0], points[:, 1], color='blue')
        plt.scatter(cluster1[:, 0], cluster1[:, 1], color='red')
        plt.scatter(cluster2[:, 0], cluster2[:, 1], color='green')
        plt.scatter(goal[0], goal[1], color = 'y')
        plt.xlim(0,3)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)

    def visualize_ransac(self, points, lines):
        self.ax.clear()
        plt.scatter(points[:, 0], points[:, 1], color='blue')
        for line in lines:
            x = np.linspace(0, 2, 3)
            y = line[0] * x + line[1]
            plt.plot(x, y, color='red')
        plt.xlim(0,3)
        plt.ylim(-2,2)
        self.fig.canvas.draw()
        plt.pause(0.01)

    def goal_point_cluster(self, clusters):
        # Find medium point of cluster1 
        midpoint1 = np.array([np.mean(clusters[0][:, 0]), np.mean(clusters[0][:,1])])

        # Find medium point of cluster2 
        midpoint2 = np.array([np.mean(clusters[1][:, 0]), np.mean(clusters[1][:,1])])

        points = np.array((midpoint1, midpoint2))

        # Find goal point 
        x = np.mean(points[:, 0])
        y = np.mean(points[:, 1])
        goal = np.array((x,y))

        return goal
    
    def heading(self, goal_point):
        alfa = 0.4
        beta = 0.6
        cmd_msg = Twist()
        if(self.NAV):
            theta = math.atan(goal_point[1]/goal_point[0])
            cmd_msg.linear.x = alfa*goal_point[0]
            cmd_msg.angular.z = beta*theta

            print(cmd_msg.linear.x, '\t', cmd_msg.angular.z)
            if (cmd_msg.linear.x > 0.1):
                self.cmd_pub.publish(cmd_msg)
            else: 
                self.NAV = False
        # else:
        #     cmd_msg.linear.x = 0
        #     cmd_msg.angular.z = 0
        #     self.cmd_pub.publish(cmd_msg)
    
    def navigation_active(self, msg):
        
        if(msg.buttons[9]==1):
            self.NAV = True
        if(msg.buttons[8]==1):
            self.NAV = False
        

    

        
def main(args=None):
    rclpy.init(args=args)

    navigation = Navigation()

    rclpy.spin(navigation)

    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    