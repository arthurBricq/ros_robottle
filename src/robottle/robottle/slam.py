
# imports for ROS
import rclpy
from rclpy.node import Node

# imports the interfaces
from vision_msgs.msg import Detection2DArray
from interfaces.msg import Nums
from interfaces.msg import LidarData 
from interfaces.msg import Position
from interfaces.msg import Map

# imports things for slam
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel


# Constants for run time
MIN_SAMPLES = 200 
MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10


class Slam(Node):
    """
    SLAM NODE
    This node is in charge of receiving data from the different sources and creating a map of the environment
    while locating the robot in the map.

    INPUT
    - Bottle detected by the Neuron
    - Lidar detections
    """
    def __init__(self):
        super().__init__('slam_node')

        # Create first subscripton (detectnet results)
        self.subscription1 = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.listener_callback_detectnet,
            1000)
        self.subscription1  # prevent unused variable warning

        # Create second subscription (SLAM)
        self.subscription2 = self.create_subscription(
            LidarData,
            'lidar_data',
            self.listener_callback_lidar,
            1000)
        self.subscription2  # prevent unused variable warning

        # Create publisher for the position of the robot, and for the map 
        self.publisher_position = self.create_publisher(Position, 'robot_pos', 10)
        self.publisher_map = self.create_publisher(Map, 'world_map', 10)

        # Initialize parameters for slam 
        self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        self.trajectory = []
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.previous_distances = None
        self.previous_angles    = None

 

    def listener_callback_detectnet(self, msg):
        self.get_logger().info('I heard something from vision ! ')



    def listener_callback_lidar(self, msg):
        self.get_logger().info('I heard something from lidar ! ')
        
        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(msg.distances) > MIN_SAMPLES:
            self.slam.update(list(msg.distances), scan_angles_degrees=list(msg.angles))
            self.previous_distances = list(msg.distances).copy()
            self.previous_angles    = list(msg.angles).copy()
        elif self.previous_distances is not None:
            # If not adequate, use previous
            self.slam.update(self.previous_distances, scan_angles_degrees=self.previous_angles)

        # Get current robot position
        x, y, theta = self.slam.getpos()
        self.get_logger().info("output of slam")
        self.get_logger().info(str(x))
        self.get_logger().info(str(y))
        
        # Get current map bytes as grayscale
        self.slam.getmap(self.mapbytes)

        # Send position in a topic 
        pos = Position()
        pos.x = float(x)
        pos.y = float(y)
        pos.theta = float(theta)
        self.publisher_position.publish(pos)
        
        # Send the map in a topic
        map_message = Map()
        self.get_logger().info("Sending map to subscribers -- " + str(type(self.mapbytes[0])))
        map_message.map_data = self.mapbytes
        self.get_logger().info(str(len(map_message.map_data)))
        # the size of this map is indeed 500 X 500 = 250 000
        self.publisher_map.publish(map_message)



def main(args=None):
    rclpy.init(args=args)
    slam_node = Slam()
    rclpy.spin(slam_node)


if __name__ == '__main__':
    main()

