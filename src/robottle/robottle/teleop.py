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

        args= sys.argv 
        self.name = "teleop_default_name"
        self.save_index = 0
        if "--name" in args:
            self.name = args[args.index("--name") + 1]

        print("Teleop Running: ")
        print("Type(w,a,s,d,x) to move ")
        print("Type 'q' to quit the node")
        print("Type 'k' to activate image detection")
        print("Type 'l' to de-activate image detection")
        print("Type 'b' to start the Motors")
        print("Type 'n' to stop the Motors")
        print("Type 'f' to flip the camera")
        print("Type '5' to change the map quality")
        print("Type '6' to draw a line in the controller 1 logs")
        print("Type '7' to take a picture now")
        print("--name = ", self.name)

        self.uart_publisher = self.create_publisher(String, 'uart_commands', 1000)
        self.cam_control_publisher = self.create_publisher(String, 'detectnet/camera_control', 1000)
        self.camera_flip_topic = self.create_publisher(String, 'video_source/flip_topic', 1000)
        self.map_corner_client = self.create_client(FindMapCorner, "find_map_corner")
        self.map_quality_control = self.create_publisher(String, 'slam_control', 1000)
        self.log_line_printer = self.create_publisher(String, 'log_line', 5)
        self.flip = True

        try:
            while(1):
                key = self.getKey()
                if key == 'l':
                    # stop detection
                    self.cam_control_publisher.publish(String(data="destroy"))
                elif key == 'k':
                    # activate detection
                    self.cam_control_publisher.publish(String(data="create"))
                elif key == 'b':
                    # start the motor
                    lidar = Lidar('/dev/ttyUSB0')
                    lidar.start_motor()
                elif key == 'n':
                    # stop the motor
                    lidar = Lidar('/dev/ttyUSB0')
                    lidar.stop_motor()
                elif key == 'f':
                    # flip the camera
                    msg = "normal" if self.flip else "flip"
                    self.flip = not self.flip
                    self.camera_flip_topic.publish(String(data=msg))
                elif key == '5':
                    # change map quality
                    self.map_quality_control.publish(String(data="nimportequoi"))
                elif key == '6':
                    self.log_line_printer.publish(String(data="pinguing"))
                if key == '7':
                    self.send_service()
                elif key == 'q':
                    break
                else:
                    msg = String()
                    msg.data = key
                    self.uart_publisher.publish(msg)
            raise RuntimeError("Teleoperation was aborted")
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
        request.name = self.name + "_" + str(self.save_index)
        future = self.map_corner_client.call_async(request)
        while True:
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                    self.save_index += 1
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
    print("Leaving code !")
    rclpy.shutdown()


if __name__ == '__main__':
    main()

