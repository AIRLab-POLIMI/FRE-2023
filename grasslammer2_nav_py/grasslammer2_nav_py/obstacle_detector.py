import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import numpy as np


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.depthTrashold = 1.0
        self.consistencyTrashold = 5000
        self.offset = 50
        self.oakd_sub = self.create_subscription(Image, "/oakd_depth/depth/image_raw", self.detection, 1)
        self.pub = self.create_publisher(Float32, "/to_plot", 1)
        self.Obstacle_detection_pub = self.create_publisher(Bool, "/obstacle_detection", 1)

    def detection(self, message):
        #message = (Image)(msg)
        #stepToRead = message.step / message.width
        height = message.height
        width = message.width
        values = np.asarray(message.data, dtype=np.int8)
        pixels = values.reshape(-1, 4)
        points = pixels.view(dtype=np.uin)
        depths = points.reshape((height, width))
        up = (int)((height/2 - self.offset))
        down = (int)((height/2 + self.offset))
        left = (int)(width/2 - self.offset)
        right = (int)(width/2 + self.offset)
        subdepths = depths[up: down, left:right]
        #self.send_to_plotter(subdepths)
        near = lambda x: x <= self.depthTrashold
        if(self.subset_exists(subdepths, near, self.consistencyTrashold)):
            ppb = Bool()
            ppb.data = True
            print("Obstacle Detected: UAU")
            self.Obstacle_detection_pub.publish(ppb)

    def subset_exists(self, arr, condition, threshold):
        # Apply the condition to the array and check if any group of elements satisfies it and is bigger than the threshold
        subset_satisfies_condition = arr[condition(arr)]
        print(subset_satisfies_condition)
        print(len(subset_satisfies_condition))
        return len(subset_satisfies_condition) >= threshold
    
    def send_to_plotter(self, mat):
        for point in mat:
            print(point)
            #data = Float32()
            #data.data = (float)(point)
            #self.pub.publish(data)



def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()