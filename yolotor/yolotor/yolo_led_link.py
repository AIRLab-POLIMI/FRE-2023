import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial




class LedComunicator(Node):
    def __init__(self):
        super().__init__('led_link_yolo')
        self.yolo_sub = self.create_subscription(String, "/obstacle_detection", self.sendColor, 1)
        self.ser = serial.Serial('/dev/arduino', 9600)

    def sendColor(self, message):
        print(message.data)
        if message.data != "S":

            print("sended_to_led" + message.data)
            self.ser.write(message.data.encode() + b'\n')


            





def main(args=None):
    rclpy.init(args=args)
    node = LedComunicator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()