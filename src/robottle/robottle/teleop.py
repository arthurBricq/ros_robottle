#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rplidar import RPLidar as Lidar

from std_msgs.msg import String
from interfaces.srv import FindMapCorner

import sys, select, termios, tty
settings = termios.tcgetattr(sys.stdin)

class TeleopRobotController(Node):

    def __init__(self):	
        super().__init__("teleop_controller")
        print("Teleop Running: ")
        print("Type(w,a,s,d,x) to move ")
        print("Type 'q' to quit the node")
        print("Type 'p' to take a picture now")
        print("Type 'k' to activate image detection")
        print("Type 'l' to de-activate image detection")
        print("Type 'm' to start the Motors")
        print("Type 'n' to stop the Motors")

        self.uart_publisher = self.create_publisher(String, 'uart_commands', 1000)
        self.cam_control_publisher = self.create_publisher(String, 'detectnet/camera_control', 1000)

        self.map_corner_client = self.create_client(FindMapCorner, "find_map_corner")

        try:
            while(1):
                key = self.getKey()
                if key == 'p':
                    self.send_service()
                elif key == 'l':
                    # stop detection
                    self.cam_control_publisher.publish(String(data="destroy"))
                elif key == 'k':
                    # activate detection
                    self.cam_control_publisher.publish(String(data="create"))
                elif key == 'm':
                    # start the motor
                    lidar = Lidar('/dev/ttyUSB0')
                    lidar.start_motor()
                elif key == 'n':
                    # stop the motor
                    lidar = Lidar('/dev/ttyUSB0')
                    lidar.stop_motor()
                elif key == 'q':
                    break
                else:
                    msg = String()
                    msg.data = key
                    self.uart_publisher.publish(msg)
            print("Finished with the loop")
        except:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def send_service(self):
        print("Sending request to take a picture")
        # must take a picture with the picture service
        while not self.map_corner_client.wait_for_service(timeout_sec=0.1):
            print("Waiting for service")
        # create the request and send it 
        request = FindMapCorner.Request()
        request.should_save = True
        future = self.map_corner_client.call_async(request)
        while True:
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                    print("reponse:", response)
                except Exception as e:
                    print("Error: ", e)
                break




def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    node = TeleopRobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

