import rclpy
import json

from yolotor.yolotor import Yolotor


rclpy.init()

with open("src/FRE-2023/yolotor/yolotor/yolotor_config.json") as f:
    param = json.load(f)

yolotor = Yolotor(
    param["yolo_blob"], 
    param["yolo_config"], 
    param["img_dim"], 
    param["labels"], 
    param["debug"]
)

yolotor.run()
yolotor.destroy_node()

rclpy.shutdown()
