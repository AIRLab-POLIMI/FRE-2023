import rclpy
import time

from rclpy.node import Node
from std_msgs.msg import String
from yolotor.yolotor_utils import *


class Yolotor(Node):
    def __init__(self, blob, config, img_dim, labels, debug):
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

        # Yolotor publisher
        self.publisher_ = self.create_publisher(String, 'yolotor', 10)

        # OAK camera pipeline for YOLO inference
        self.pipeline = create_camera_pipeline(config_path=config, model_path=blob, camera_dim=(img_dim, img_dim))

        # storing YOLO info for run-time execution
        self.labels = labels
        self.debug = debug

        # ooookay lesgo...
        self.get_logger().info('Yolotor started...')
        

    def run(self):
        """
        Run the principal Yolotor method throughout the lifecycle of the ROS node to perform deep learning inference.
        """
        # depth-ai library takes into account the instantiated OAK pipeline
        with dai.Device(self.pipeline) as device:
            # RGB output stream
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            # YOLO output stream
            qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

            # to measure inference time performance
            if self.debug:
                start_time = time.monotonic()
                counter = 0

            # main loop cycle until shutdown
            while rclpy.ok():
                # getting RGB stream
                inRgb = qRgb.get()
                if inRgb is not None:
                    # convert inRgb output to a format OpenCV library can work with
                    frame = inRgb.getCvFrame()

                # getting YOLO stream
                inDet = qDet.get()
                if inDet is not None:
                    # If inDet is not None, fetch all the detections for a frame
                    detections = inDet.detections

                    # to measure inference time performance
                    if self.debug:
                        counter += 1

                if frame is not None:
                    highest_confidence_box = None
                    highest_confidence = 0.0

                    # find the most confident detection made by YOLO
                    for out in detections:
                        box_confidence = out.confidence
                        if box_confidence > highest_confidence:
                            highest_confidence = box_confidence
                            highest_confidence_box = out

                    if highest_confidence_box is not None:
                        # if you found something then sent it on the topic
                        msg = String()
                        msg.data = self.labels[highest_confidence_box.label - 1]
                        self.publisher_.publish(msg)
                        
                        # it computes the annotated frame
                        if self.debug:
                            self.get_logger().info('Publishing: "%s"' % msg.data)
                            self.get_logger().info("Inference time [ms]: {:.2f}".format(1000 / (counter / (time.monotonic() - start_time))))
                            self.get_logger().info("---")
                            frame = annotateFrame(frame, highest_confidence_box, self.labels)
                    
                    # it visualizes the annotated frame
                    if self.debug:
                        cv2.imshow("Yolotor", frame)
                        cv2.waitKey(1)
