import rclpy
from rclpy.node import Node

from interfaces.msg import MotorsSpeed
import time
from datetime import datetime
import serial
from robottle_utils import uart_utils

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10

class UARTReader(Node):
    """
    This node is in charge of 
    * reading the motors speed from the UART
    * send it to the SLAM node 
    """

    def __init__(self):
        super().__init__("uart_messenger")


        # Create a publication for the motors speed
        self.speeds_publisher = self.create_publisher(MotorsSpeed, 'motors_speed', 1000)

        # setup the uart port and wait a second for it
        self.serial_port = serial.Serial(
            port="/dev/ttyTHS1",
            baudrate=9600)
        time.sleep(1)
        
        # read the speed here
        # this code continuously check if there is something in the buffer
        # and then if so try to cast it as a speed using the following convention
        # l --> following bytes are left speed
        # r --> following bytes are right speed
        last_datetime = datetime.now()
        while True:
            if self.serial_port.inWaiting() > 0:
                datas = []
                msg = MotorsSpeed()
                while self.serial_port.inWaiting() > 0:
                    try:
                        data = self.serial_port.read().decode('ascii')
                        if data == 'l': direction="l"
                        elif data == 'r': direction="r"
                        elif data == '\r': # the number is finished
                            speed = uart_utils.get_speed(datas)
                            datas = []
                            self.get_logger().info("speed {} is {} !".format(direction, speed))
                            # construct the ROS message here
                            if direction == 'l': msg.left = speed
                            elif direction == 'r': msg.right = speed
                        elif data != '\n': datas.append(data)
                    except:
                        print("error occured reading this character")
                # now that the loop is finish, publish the message
                new_datetime = datetime.now()
                msg.time_delta = (new_datetime - last_datetime).total_seconds()
                self.speeds_publisher.publish(msg)
                last_datetime = new_datetime



def main(args=None):
    rclpy.init(args=args)
    node = UARTReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
