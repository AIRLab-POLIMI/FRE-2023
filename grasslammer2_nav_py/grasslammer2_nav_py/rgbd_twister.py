#!/usr/bin/env python3

import math
import open3d
from turtle import width
from matplotlib.pyplot import bar
import rospy
import rospkg
import numpy as np
import cv2
import os
import threading
from geometry_msgs.msg import Pose
from pyzbar.pyzbar import decode, ZBarSymbol
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, PointCloud2

from detection.msg import DetectedPillar

from scipy.spatial.transform import Rotation as R

from scipy.spatial import cKDTree

from ctypes import *

convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


class Qrotor:

    MAX_DEPTH_QUEUE_SIZE = 50

    # intrinsic parameters
    kinect_fx = 554.25
    kinect_fy = 554.25
    kinect_cx = 320.5
    kinect_cy = 240.5

    # inverse of the camera matrix
    inv_K = np.array(
        [
            [1 / kinect_fx, 0, -kinect_cx / kinect_fx, 0],
            [0, 1 / kinect_fy, -kinect_cy / kinect_fy, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    )

# [0.195, 0.000, 0.266]

# [  0.0000000,  0.0000000,  1.0000000;
#   -1.0000000,  0.0000000,  0.0000000;
#    0.0000000, -1.0000000,  0.0000000 ]

    inv_Rt = np.array(
        [[0.0000000, 0.0000000, 1.0000000, 0.195],
        [-1.0000000, 0.0000000, 0.0000000, 0.000],
        [0.0000000, -1.0000000, 0.0000000 , 0.266],
        [0.0000000, 0.0000000, 0.0000000, 1.000]]
    )

    def __init__(self) -> None:
        self.model = None
        # os.environ['CUDA_VISIBLE_DEVICES'] = '-1'

        self.sub = rospy.Subscriber("/kinect_frontal_camera/color/image_raw", Image, queue_size=10, callback=self.callbackDetection)
        self.sub_depth = rospy.Subscriber("/oakd_depth/points", PointCloud2, queue_size=10, callback=self.callbackPointcloud)

        self.pub = rospy.Publisher("/image_qrotor_frontal", Image, queue_size=1)   
        self.bb_pub = rospy.Publisher("/detected_pillar", DetectedPillar, queue_size=100)

        self.depth_queue = []
        self.depth_queue_lock = threading.Lock()
        
        # self.M = self.K @ np.hstack([self.R, self.T.T])

        self.time = rospy.get_time()

        print("QRotor Initialized ")


    def from_uvd_to_xyz_cam(self, u, v, d=1):
        """
            From image coordinates to cam coordinates.
        """
        # input
        uvd_image = np.array([[u], [v], [1.], [1. / d]])

        # output
        xyz_cam = (d) * self.inv_K @ uvd_image

        return xyz_cam[0, 0], xyz_cam[1, 0], xyz_cam[2, 0]

    def from_xyz_cam_to_xyz_base_link(self, x, y, z):
        """
            From cam coordinates to base_link coordinates.
        """
        # input
        xyz_cam = np.array([[x], [y], [z], [1]])

        # output
        xyz_baselink = self.inv_Rt @ xyz_cam

        return xyz_baselink[0, 0], xyz_baselink[1, 0], xyz_baselink[2, 0]


    def callbackDetection(self, data):
        msg_secs = rospy.Time(data.header.stamp.secs, data.header.stamp.nsecs).to_sec()

        if msg_secs + 1 < self.time:
            return

        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data)
        
        frame = cv_image
        
        for barcode in decode(frame, symbols=[ZBarSymbol.QRCODE]):
            myData = barcode.data.decode("utf-8")

            if myData == "Location_marker_A":
                label = 'A'
                color = (255, 0, 0)
            elif myData == "Location_marker_B":
                label = 'B'
                color = (0, 0, 255)
            else:
                continue

            rect = barcode.rect

            qr_center_u = rect.left + int(rect.width/2)
            qr_center_v = rect.top + int(rect.width/2)

            # get the depth image
            depth_open3d = self.getPointcloud(data.header.stamp)

            if depth_open3d == -1:
                print(depth_open3d)
                break

            # estimate the normals
            depth_open3d.estimate_normals()

            # I can access the normal of the zeroth point at:
            #       depth_open3d.normals[0]

            x_cam, y_cam, _ = self.from_uvd_to_xyz_cam(qr_center_u, qr_center_v)

            dx = 10000
            dy = 10000
            selected_i = 0
            selected_point = None

            ppp_points = np.array(depth_open3d.points)[:, :2]
            print(ppp_points[0])

            treee = cKDTree(ppp_points)
            _, idx = treee.query([x_cam, y_cam])

            selected_i = idx
            selected_point = depth_open3d.points[selected_i]
            normal = depth_open3d.normals[selected_i]
            
            print("Selected i: ", selected_i)
            
            print("(x_cam, y_cam) ", x_cam, " ", y_cam)

            print("Point on cloud: ", end="")
            print(selected_point)
            
            print("Normal: ", end=" ")
            print(normal)

            inside_position = np.array([selected_point[0], selected_point[1], selected_point[2]]) - (0.5 * normal)
            
            print("Inside Position w.r.t. cam : ", end=" ")
            print(inside_position)

            pillar_base_link = self.from_xyz_cam_to_xyz_base_link(inside_position[0], inside_position[1], inside_position[2])

            print("Pillar Position w.r.t. base_link: ", end=" ")
            print(pillar_base_link)

            # ################ #
            # GENERATE MESSAGE #
            # ################ #
            det_object = DetectedPillar()

            det_object.header.frame_id= "base_link"
            det_object.header.stamp = data.header.stamp
            det_object.header.seq = 0
            det_object.x = pillar_base_link[0]
            det_object.y = pillar_base_link[1]
            det_object.id = label

            self.bb_pub.publish(det_object)


            #################################
            # PRINT IMAGE FOR VISUALIZATION #
            #################################
            
            pts = np.array([barcode.polygon], np.int32)
            cv2.polylines(frame, [pts], True, color, 5)
            pts2 = barcode.rect
            cv2.putText(frame, label, (pts2[0], pts2[1]), cv2.FONT_HERSHEY_COMPLEX, 1, color, 2)

            print(label)

            # TODO: ok???
            # assumption we can do only one observation of pillar at time!!
            break


        #################################
        # PRINT IMAGE FOR VISUALIZATION #
        #################################

        img3 = cv2.cvtColor(frame, cv2.CV_8UC1)

        ros_image = bridge.cv2_to_imgmsg(img3, "rgba8") #"rgb8")

        self.pub.publish(ros_image)

        self.time = rospy.get_time()



    def getPointcloud(self, stamp):
        """
            Returns the depth of the point (x,y)
        """
        pc = None
        self.depth_queue_lock.acquire()

        if len(self.depth_queue) == 0 or rospy.Time(stamp.secs, stamp.nsecs).to_sec() < rospy.Time(self.depth_queue[0][0].secs, self.depth_queue[0][0].nsecs).to_sec():
            # print("Len: " + str(len(self.depth_queue)))
            # if len(self.depth_queue) != 0:
            #     print("Secs: " + str(rospy.Time(self.depth_queue[0][0]).to_sec()))
            self.depth_queue_lock.release()
            return -1

        # print("a")

        while rospy.Time(stamp.secs, stamp.nsecs).to_sec() > rospy.Time(self.depth_queue[0][0].secs, self.depth_queue[0][0].nsecs).to_sec():
            # print(rospy.Time(stamp.secs, stamp.nsecs).to_sec())
            # print(rospy.Time(self.depth_queue[0][0].secs, self.depth_queue[0][0].nsecs).to_sec())

            pc = self.depth_queue.pop(0)

            if len(self.depth_queue) == 0:
                self.depth_queue_lock.release()
                return -1

            # while len(self.depth_queue) == 0:
            #     pass

        # print(rospy.Time(stamp.secs, stamp.nsecs).to_sec())
        # print(rospy.Time(self.depth_queue[0][0].secs, self.depth_queue[0][0].nsecs).to_sec())

        depth_image = self.depth_queue.pop()[1]
        
        self.depth_queue_lock.release()

        # return the pointcloud in open3d 
        depth_open3d = self.convertCloudFromRosToOpen3d(depth_image)

        return depth_open3d



    def callbackPointcloud(self, data):
        
        cloud_data = self.convertCloudFromRosToOpen3d(data)
        could_data_down = cloud_data.voxel_down_sample(voxel_size=0.03)
        

        


    def convertCloudFromRosToOpen3d(self, ros_cloud):
        # Get cloud data from ros_cloud
        field_names=[field.name for field in ros_cloud.fields]
        cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))
        # Check empty
        # open3d_cloud = open3d.geometry.PointCloud()
        if len(cloud_data)==0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD=3 # x, y, z, rgb
            
            # Get xyz
            xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

            # Get rgb
            # Check whether int or float
            if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
                rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
            else:
                rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

            # combine
            points = open3d.cpu.pybind.utility.Vector3dVector(np.array(xyz))
            colors = open3d.cpu.pybind.utility.Vector3dVector(np.array(rgb)/255.0)
        else:
            xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
            points = open3d.cpu.pybind.utility.Vector3dVector(np.array(xyz))

        open3d_cloud = open3d.geometry.PointCloud(points=points)

        # return
        return open3d_cloud


if __name__ == '__main__':
    rospy.init_node('qrotor', anonymous=True)
    
    seg = Qrotor()

    rospy.spin()
