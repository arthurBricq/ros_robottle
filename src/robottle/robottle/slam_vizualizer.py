import rclpy
from rclpy.node import Node

from interfaces.msg import Map

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
            Map,
            'world_map',
            self.listener_callback,
            1000)
        self.subscription1  # prevent unused variable warning
        
        # setup the viz
        self.viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, "SLAM")



    def listener_callback(self, map_message):
        self.get_logger().info("map received from SLAM ! Let's draw it.")
        map_data = bytearray(map_message.map_data)
        self.get_logger().info(str(len(map_data)))
        
        if not self.viz.display(0, 0, 0, map_data):
            exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SlamVizualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

