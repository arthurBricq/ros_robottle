#!/usr/bin/env python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sys, select, termios, tty
settings = termios.tcgetattr(sys.stdin)

class TeleopRobotController(Node):
    def getKey(self):
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key

    def __init__(self):	
        super().__init__("teleop_controller")
        self.uart_publisher = self.create_publisher(String, 'uart_commands', 1000)
        print("Teleop Running: type commands (w,a,s,d,x)")
        print("Type 'q' to quit the node")
        try:
            while(1):
                key = self.getKey()
                if key == 'q':
                    break
                msg = String()
                msg.data = key
                self.uart_publisher.publish(msg)
        except:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    if args is None:
        args = sys.argv
    rclpy.init(args=args)
    node = TeleopRobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

