import rclpy
from rclpy.node import Node

from interfaces.msg import Map
from interfaces.msg import Position

from roboviz import MapVisualizer

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10



class SlamVizualizer(Node):
    """
    This node visualize the map received from the SLAM node.
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



    def listener_callback_map(self, map_message):
        self.get_logger().info("map received from SLAM ! Let's draw it.")
        map_data = bytearray(map_message.map_data)
        self.get_logger().info(str(len(map_data)))
        
        if not self.viz.display(0, 0, 0, map_data):
            exit(0)


    def listener_callback_position(self, map_message):
        self.get_logger().info("position received from SLAM")
        # todo: include the position of the robot in the map, using class variables. 


def main(args=None):
    rclpy.init(args=args)
    node = SlamVizualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

