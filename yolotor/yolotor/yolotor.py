import rclpy
import time
from cv_bridge import CvBridge
import cv2

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from yolotor.yolotor_utils import *
from yolotor.yolotor_classifier import *
import depthai as dai


class Yolotor(Node):
    def __init__(self, blob, config, img_dim, queue, confidence, interval, labels, debug):
        """
        Yolotor class in charge of loading the OAK pipeline to make inference on YOLO blob file.

        Parameters
        ----------
            - blob:
                YOLO blob file path.

            - config:
                YOLO configuration file path.

            - img_dim:
                YOLO input image size; The input is assumed as one integer since it is used a squared shape.

            - labels:
                List of labels recognized by the YOLO model.
        """
        # ROS node initialization
        super().__init__('yolotor_node')

        # starting OAK camera pipeline for YOLO inference
        self.pipeline = create_camera_pipeline(config_path=config, model_path=blob, camera_dim=(img_dim, img_dim))
        self.device = dai.Device(self.pipeline)
        self.device.startPipeline()

        # Create publishers
        self.rgb_pub = self.create_publisher(Image, 'oak_color_image', 100)
        self.depth_pub = self.create_publisher(Image, 'oak_depth_image', 100)
        self.yolo_pub = self.create_publisher(String, 'yolotor', 100)

        # yolo classifier
        self.classifier = YolotorClassifier(queue_size=queue, confidence_threshold=confidence, interval=interval)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # storing YOLO info for run-time execution
        self.labels = labels
        self.debug = debug

        padding = 250
        start_x = int((416 - padding) / 2)
        start_y = int((416 - padding / 2) / 2)
        x1 = start_x
        y1 = start_y
        x2 = start_x + padding
        y2 = 416

        self.valid_area = (x1, y1, x2, y2)

    def is_valid(self, frame, detection):
        # internal method used only here
        def frameNorm(frame, bbox):
            normVals = np.full(len(bbox), frame.shape[0])
            normVals[::2] = frame.shape[1]
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

        # bounding boxes display
        bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))

        # Extract the coordinates of the first bounding box
        x1_1, y1_1, x2_1, y2_1 = bbox[0], bbox[1], bbox[2], bbox[3]

        # Extract the coordinates of the second bounding box
        x1_2, y1_2, x2_2, y2_2 = self.valid_area

        # Calculate the area of the first bounding box
        area_box = (x2_1 - x1_1) * (y2_1 - y1_1)

        # Calculate the area of the intersection between the two bounding boxes
        intersection_width = min(x2_1, x2_2) - max(x1_1, x1_2)
        intersection_height = min(y2_1, y2_2) - max(y1_1, y1_2)
        intersection_area = max(0, intersection_width) * max(0, intersection_height)

        # Check if the ratio of the intersection area to the area of box1 is above a threshold
        threshold = 0.9  # Adjust this threshold as needed
        return intersection_area / area_box >= threshold


    def run(self):
        """
        Yolotor
        """
        self.get_logger().info('Yolotor and OAK-D Camera node is running.')

        rgb_reader = self.device.getOutputQueue(name="rgb", maxSize=30, blocking=False)
        depth_reader = self.device.getOutputQueue(name="depth", maxSize=30, blocking=False)
        yolo_reader = self.device.getOutputQueue(name="nn", maxSize=30, blocking=False)

        # to measure inference time performance
        if self.debug:
            start_time = time.monotonic()
            counter = 0

        while rclpy.ok():
            rgb_data = rgb_reader.get()
            depth_data = depth_reader.get()
            yolo_data = yolo_reader.get()

            if rgb_data is not None:
                # convert inRgb output to a format OpenCV library can work with
                rgb_frame = rgb_data.getCvFrame()
                rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, 'bgr8')
                self.rgb_pub.publish(rgb_msg)

            if depth_data is not None:
                depth_frame = depth_data.getCvFrame()
                #depth_frame = (depth_frame * (255 / 95)).astype(np.uint8) # 95 = max disparity value for normalization
                depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, '8UC1')
                self.depth_pub.publish(depth_msg)

            if yolo_data is not None:
                # If inDet is not None, fetch all the detections for a frame
                detections = yolo_data.detections

                # to measure inference time performance
                if self.debug:
                    counter += 1

            if rgb_frame is not None:
                highest_confidence_box = None
                highest_confidence = 0.0

                # find the most confident detection made by YOLO
                for out in detections:
                    if self.is_valid(rgb_frame, out) and out.confidence > highest_confidence:
                            highest_confidence = out.confidence
                            highest_confidence_box = out

                if highest_confidence_box is not None:
                    # if you found something then sent it on the topic
                    msg = String()
                    self.classifier.enqueue(highest_confidence, self.get_clock().now().nanoseconds, self.labels[highest_confidence_box.label - 1])
                    prediction = self.classifier.predict()
                    
                    if prediction is not None:
                        msg.data = prediction
                        self.yolo_pub.publish(msg)

                        if self.debug:
                            self.get_logger().info('Publishing: "%s"' % msg.data)
                            self.get_logger().info("Inference time [ms]: {:.2f}".format(1000 / (counter / (time.monotonic() - start_time))))
                            self.get_logger().info("---")

                    # it computes the annotated frame
                    if self.debug:
                        rgb_frame = annotateFrame(rgb_frame, highest_confidence_box, self.labels)
                
                # it visualizes the annotated frame
                if self.debug:
                    overlay = rgb_frame.copy()
                    alpha = 0.1
                    cv2.rectangle(overlay, (self.valid_area[0], self.valid_area[1]), (self.valid_area[2], self.valid_area[3]), (255, 0, 0), -1) 
                    rgb_frame = cv2.addWeighted(overlay, alpha, rgb_frame, 1 - alpha, 0)
                    cv2.imshow("Yolotor", rgb_frame)
                    cv2.waitKey(1)
