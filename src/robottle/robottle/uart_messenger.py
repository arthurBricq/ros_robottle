import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import time
import serial

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10

class UARTMessenger(Node):
    """
    This node receives String message and sends them to the UART channel for the Arduino
    """

    def __init__(self):
        super().__init__("uart_messenger")

        # Create subscription for the uart commands to send 
        self.subscription1 = self.create_subscription(
            String,
            'uart_commands',
            self.listener_callback,
            1000)
        self.subscription1  # prevent unused variable warning

        # setup the uart port and wait a second for it
        self.serial_port = serial.Serial(
            port="/dev/ttyTHS1",
            baudrate=9600)
            
        time.sleep(1)
        
        self.i = 0
        #self.serial_port.write("hello arduino".encode())

    def listener_callback(self, msg):
        self.i += 1
        if self.i % 5 == 0:
            self.get_logger().info(msg.data)
            self.serial_port.write(msg.data.encode())



def main(args=None):
    rclpy.init(args=args)
    node = UARTMessenger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

