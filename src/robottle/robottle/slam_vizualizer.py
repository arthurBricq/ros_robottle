import sys
import rclpy
from rclpy.node import Node

from interfaces.msg import Map
from interfaces.msg import Position

import matplotlib.pyplot as plt
import numpy as np
from robottle_utils import map_utils

from roboviz import MapVisualizer

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10


class SlamVizualizer(Node):
    """
    This node visualize the map received from the SLAM node, using the PyRoboViz librairie.
    This node is quite hard to use, because it slows down a lot the process. It should not be used in live with the Jetson
    This node also takes a parameter when launched, so that it can:
    - first argument: name of the images to save
    """
    def __init__(self):
        super().__init__("slam_viz")

        # Create subscription for the map 
        self.subscription1 = self.create_subscription(
            Position,
            'robot_pos',
            self.listener_callback_position,
            1000)
        self.subscription1  # prevent unused variable warning

        # Create subscription for the robot position
        self.subscription2 = self.create_subscription(
            Map,
            'world_map',
            self.listener_callback_map,
            1000)
        self.subscription2  # prevent unused variable warning

        # setup the vizualistion tool (it will open a window showing the map and the robot within the map)
        self.viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, "SLAM")

        # keep track of where is the robot within the class
        self.x = 0
        self.y = 0
        self.theta = 0

        # set saving state (if True, then it will save some maps to a folder when they can be analysed)
        args = sys.argv
        if len(args) == 1: # it means no argument was passed to the ros node
            self.is_saving = False
        else:
            self.is_saving = True
            self.map_name = args[1]
            self.saving_index = 0


    def listener_callback_map(self, map_message):
        self.get_logger().info("map received : {}".format(map_message.index))
        # get the map data from the message
        map_data = bytearray(map_message.map_data)

        # plot data using roboviz
        if not self.viz.display(self.x, self.y, self.theta, map_data): exit(0)

        # save image for debugging
        if self.is_saving:
            if int(map_message.index) % 20 == 0: # do this times to times, else it's too many maps being saved ! 
                # get the map (and save it as raw, for debugging)
                m = map_utils.get_map(map_data)
                robot_pos = map_utils.pos_to_gridpos(self.x, self.y)
                np.save("/home/arthur/dev/ros/data/maps/" + self.map_name + str(self.saving_index) +  ".npy", m)
                self.saving_index += 1

                ### Image analysis happening here

                # a. filter the map 
                binary = map_utils.filter_map(m)
                # b. get rectangle around the map
                corners, contours = map_utils.get_bounding_rect(binary)
                # c. find zones 
                zones = map_utils.get_zones(corners, robot_pos)
                # d. make and save the nice figure
                save_name = "/home/arthur/dev/ros/data/maps/rects/"+self.map_name+str(self.saving_index)+".png"
                map_utils.make_nice_plot(binary, save_name, robot_pos, self.theta, contours, corners, zones)


    def listener_callback_position(self, pos):
        self.x = pos.x / 1000
        self.y = pos.y / 1000
        self.theta = pos.theta
        self.get_logger().info("position received from SLAM: {}, {}, {}  ".format(self.x,self.y,self.theta))


def main(args=None):
    rclpy.init(args=args)
    node = SlamVizualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

