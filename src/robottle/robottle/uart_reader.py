import rclpy
from rclpy.node import Node

from interfaces.msg import MotorsSpeed, Status
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
        super().__init__("uart_reader")


        # Create a publication for the motors speed
        self.speeds_publisher = self.create_publisher(MotorsSpeed, 'motors_speed', 1000)
        self.status_publisher = self.create_publisher(Status, 'arduino_status', 1000)

        # setup the uart port and wait a second for it
        self.serial_port = serial.Serial(
            port="/dev/ttyACM0",
            baudrate=9600)
        time.sleep(1)
        
        # Reads the UART here
        # this code continuously check if there is a character in the buffer
        # if so it first reads the type of message to cast it properly:
        # s --> status will be returned in next character 
        # (status is an Int which describes where is the robot )
        # l --> following bytes are left speed
        # r --> following bytes are right speed
        # (speeds are recorded with a timestamp as well for odometry)
        last_datetime = datetime.now()
        is_waiting_for_status = False
        while True:
            if self.serial_port.inWaiting() > 0:
                datas = []
                msg = MotorsSpeed()
                while self.serial_port.inWaiting() > 0:
                    try:
                        # data is a character
                        data = self.serial_port.read().decode('ascii')
                        if is_waiting_for_status: 
                            is_waiting_for_status = False
                            self.status_received(data)
                        if data == 's': 
                            is_waiting_for_status = True 
                            continue
                        if data == 'l': direction="l"
                        elif data == 'r': direction="r"
                        elif data == '\r': # the number is finished
                            speed = uart_utils.get_speed(datas)
                            datas = []
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

    def status_received(self, data):
        """
        Call this function when a status is received from the Arduino. 
        Arduino sends status when
        0 = the task it was asked to do failed
        1 = the task it was asked to do finished successfully
        2 = a task was received and will be processed
        """
        status = int(data)
        status_msg = Status()
        status_msg.status = status
        self.status_publisher.publish(status_msg)
        print("Status received: ", status)




def main(args=None):
    rclpy.init(args=args)
    node = UARTReader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
