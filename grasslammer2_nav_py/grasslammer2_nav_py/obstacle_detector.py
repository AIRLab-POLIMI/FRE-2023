import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import String
import numpy as np
import time


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector3')
        self.depthTrashold = 170
        self.consistencyTrashold = 3430
        self.offset = 35
        self.oakd_sub = self.create_subscription(Image, "/oak_depth_image", self.detection, 1)
        self.pub = self.create_publisher(Float32, "/to_plot", 1)
        self.Obstacle_detection_pub = self.create_publisher(String, "/obstacle_detection", 1)
        self.yolo_sub = self.create_subscription(String, "/yolotor", self.saveData, 10)
        self.detectionData = "UNKNOWN"
        self.errorTreshold = 5
        self.free = 2
        self.sentIndication = False
        self.timeStamp = self.get_clock().now()
        self.timeSlip = 1500000000
        self.firstDetectionTime = self.get_clock().now() 
        self.firstDetection = False
        self.window = 1000000000

    def detection(self, message):
        #message = (Image)(msg)
        #stepToRead = message.step / message.width
        height = message.height
        width = message.width
        values = np.asarray(message.data, dtype=np.uint8)
        points = self.changeDisparitiesToDepths(values)
        #print(points)
        #pixels = values.reshape(-1, 2)
        #points = pixels.view(dtype=np.uint16)
        depths = points.reshape((height, width))
        up = (int)((height/2 - self.offset))
        down = (int)((height/2 + self.offset))
        left = (int)(width/2 - self.offset)
        right = (int)(width/2 + self.offset)
        subdepths = depths[up: down, left:right]
        #self.send_to_plotter(subdepths)
        near = lambda x: x <= self.depthTrashold
        #print("MAX ",points.max())
        #print("MIN ",points.min())
        foundSomething = self.subset_exists(subdepths, near, self.consistencyTrashold)
        if foundSomething:
            if not self.sentIndication:
                if(self.errorTreshold >= 0):
                    self.errorTreshold -= 1
                else:
                    if not self.firstDetection:
                        self.firstDetectionTime = self.get_clock().now()
                        self.firstDetection = True
                        return
                    elif self.get_clock().now().nanoseconds - self.firstDetectionTime.nanoseconds <= self.window:
                        return
                        
                    #time.sleep(2.1)

                    #print("NOW: ", self.get_clock().now().nanoseconds)
                    #print("LAST: ", self.timeStamp.nanoseconds)
                    #self.get_logger().info((self.get_clock().now().nanoseconds - self.timeStamp.nanoseconds)/1000000000)
                    if self.get_clock().now().nanoseconds - self.timeStamp.nanoseconds <= self.timeSlip:
                        
                        self.errorTreshold = 5
                        indication = String()
                        indication.data = self.detectionData[0]
                        self.Obstacle_detection_pub.publish(indication)
                        self.get_logger().info(self.detectionData)
                        #comunicate with leds
                    else:
                        indication = String()
                        indication.data = "U"
                        self.Obstacle_detection_pub.publish(indication)
                        self.get_logger().info("UNKNOWN")
                        
                    self.sentIndication = True
        else:
            if self.free <= 0 :
                self.errorTreshold = 5
                self.free = 2
                self.sentIndication = False
                self.firstDetection = False
            else:
                self.free -= 1   

    def subset_exists(self, arr, condition, threshold):
        # Apply the condition to the array and check if any group of elements satisfies it and is bigger than the threshold
        subset_satisfies_condition = arr[condition(arr)]
        #print(subset_satisfies_condition)
        #print(len(subset_satisfies_condition))
        return len(subset_satisfies_condition) >= threshold

    def changeDisparitiesToDepths(self, values):
        focalLenght800P = 720.5
        focalLenght400P = 441.5
        baselineMM = 7.5

        values = values + 1e-12

        transform = lambda x : ((focalLenght800P * baselineMM)/x)
        transform_func = np.vectorize(transform)


        return transform_func(values)
    
    
    def saveData(self, msg):
        self.detectionData = msg.data
        self.timeStamp = self.get_clock().now()



def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
