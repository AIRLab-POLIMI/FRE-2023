import rclpy
from rclpy.node import Node
from std_msgs.msg import Char
import serial




class LedComunicator(Node):
    def __init__(self):
        super().__init__('led_link_yolo')
        self.yolo_sub = self.create_subscription(Char, "/obstacle_detection", self.sendColor, 1)
        self.ser = serial.Serial('/dev/arduino', 9600)
        self.output = ''

    def sendColor(self, message):

        if message.data != 'S':

            if(self.output != message.data):
                self.ser.write(self.output.encode() + b'\n')


            self.output = message.data





def main(args=None):
    rclpy.init(args=args)
    node = LedComunicator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()