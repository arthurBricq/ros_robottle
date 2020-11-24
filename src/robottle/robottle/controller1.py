import rclpy
from rclpy.node import Node
import numpy as np 

from interfaces.msg import Map
from interfaces.msg import Position
from std_msgs.msg import String

from robottle_utils import map_utils

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10

class ControllerOpenLoop(Node):
    """
    This node visualize the map received from the SLAM node.
    This node is quite hard to use, because it slows down a lot the process. 
    It should not be used in live with the Jetson 
    """
    def __init__(self):
        super().__init__("controller1")

        # Create subscription for the map 
        self.subscription1 = self.create_subscription(Position,'robot_pos',
            self.listener_callback_position,1000)
        self.subscription1  

        # Create subscription for the robot position
        self.subscription2 = self.create_subscription(Map,'world_map',
            self.listener_callback_map,1000)
        self.subscription2  

        # Create subscription for the UART reader (get signals from MC)
        self.subscription2 = self.create_subscription(Map,'world_map',
            self.listener_callback_map,1000)
        self.subscription2  
        
        # Create a publication for uart commands
        self.uart_publisher = self.create_publisher(String, 'uart_commands', 1000)

        # keep track of where is the robot within the class
        self.x = 0
        self.y = 0
        self.theta = 0 

        # STATE MACHINE
        self.state = "initial_rotation"


    def listener_callback_position(self, pos):
        # receive the position from the SLAM
        self.x = pos.x / 1000
        self.y = pos.y / 1000
        self.theta = pos.theta

    def listener_callback_map(self, map_message):
        self.get_logger().info("map received : {}".format(map_message.index))
        map_data = bytearray(map_message.map_data)
        occupancy_grid = map_utils.get_map(map_data)
        


def main(args=None):
    rclpy.init(args=args)
    node = ControllerOpenLoop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

