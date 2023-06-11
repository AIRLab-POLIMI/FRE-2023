import rclpy
import json

from yolotor.yolotor import Yolotor

def main(args=None):
    rclpy.init(args=args)

    with open("src/grasslammer2/yolotor/yolotor/yolotor_config.json") as f:
        param = json.load(f)

    yolotor = Yolotor(
        param["yolo_blob"], 
        param["yolo_config"], 
        param["img_dim"], 
        param["queue"],
        param["confidence"],
        param["interval"],
        param["labels"], 
        param["debug"]
    )

    #rclpy.spin(yolotor)
    #yolotor.destroy_node()
    yolotor.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
