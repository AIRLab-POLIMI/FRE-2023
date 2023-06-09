import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial




class LedComunicator(Node):
    def __init__(self):
        super().__init__('led_link_yolo')
        self.yolo_sub = self.create_subscription(String, "/yolotor", self.sendColor, 1)
        self.ser = serial.Serial('/dev/arduino', 9600)
        self.output = ""

    def sendColor(self, message):

        if(self.output != message.data):
            self.ser.write(self.output.encode() + b'\n')


        if message.data == "HUMAN":
        	self.output = "H"
        elif message.data == "DEER":
        	self.output = "D"





def main(args=None):
    rclpy.init(args=args)
    node = LedComunicator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()