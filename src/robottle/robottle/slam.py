
# imports for ROS
import rclpy
from rclpy.node import Node

# imports the interfaces
from vision_msgs.msg import Detection2DArray
from interfaces.msg import Nums
from interfaces.msg import LidarData 
from interfaces.msg import Position
from interfaces.msg import Map
from interfaces.msg import MotorsSpeed

# imports things for slam
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
import numpy as np

# import lidar library to analyse angles
from robottle_utils import lidar_utils

# Constants for run time
MIN_SAMPLES = 100 
MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10

class Slam(Node):
    """
    SLAM NODE
    This node is in charge of receiving data from the LIDAR and creating a map of the environment
    while locating the robot in the map.

    INPUT
    - Bottle detected by the Neuron (to be removed soon)
    - Lidar detections

    OUTPUT
    - Robot position: /robot_pos
    - Map as a byte array: /world_map
    (this map can be interpred using the map_utils.py package !)
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

        # Create third subscription (Motor speeds to compute robot pose change)
        self.subscription3 = self.create_subscription(
            MotorsSpeed,
            'motors_speed',
            self.listener_callback_motorsspeed,
            1000)
        self.subscription3  # prevent unused variable warning

        # Create publisher for the position of the robot, and for the map 
        self.publisher_position = self.create_publisher(Position, 'robot_pos', 10)
        self.publisher_map = self.create_publisher(Map, 'world_map', 10)

        # Initialize parameters for slam 
        self.slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)
        self.trajectory = []
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.previous_distances = None
        self.previous_angles    = None
        self.map_index = 0
        self.robot_pose_change = np.array([0.0,0.0,0.0])


    def listener_callback_detectnet(self, msg):
        self.get_logger().info('I heard something from vision ! ')

    def listener_callback_motorsspeed(self, msg):
        """
        Must compute the robot pose change since the last time the data was collected from the motor
        The input are the motors speed and the time change.

        Documentation of the SLAM library says :
            "pose_change is a tuple (dxy_mm, dtheta_degrees, dt_seconds) computed from odometry"
        """
        dx_left = msg.RADIUS * msg.left * (0.1047) * msg.time_delta
        dx_right = msg.RADIUS * msg.right * (0.1047) * msg.time_delta
        delta_r = 1/2 * (dx_left + dx_right) # [mm]
        delta_d =  dx_right - dx_left
        delta_theta = np.arctan(delta_d / msg.LENGTH) * 57.2958 # [deg]
        self.robot_pose_change += [delta_r, delta_theta, msg.time_delta]
        self.get_logger().info("robot pose change: {}".format(self.robot_pose_change))

    def listener_callback_lidar(self, msg):
        # UNCOMMENT this code to find indexes i1 and i2

        (i1, i2) = lidar_utils.get_valid_lidar_range(distances = msg.distances, 
                angles = msg.angles,
                n_points = 10)
        self.get_logger().info('indexes : {} : {} with angles {} - {}'
                .format(i1,i2,msg.angles[i1], msg.angles[i2]))

        # extract angles of interest
        angles = list(msg.angles[i1:i2])
        distances = list(msg.distances[i1:i2])

        # compute robot pose now
        # todo

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            self.slam.update(distances, scan_angles_degrees=angles, pose_change = tuple(self.robot_pose_change))
            self.robot_pose_change = np.array([0.0,0.0,0.0])
            self.previous_distances = distances.copy()
            self.previous_angles    = angles.copy()
        elif self.previous_distances is not None:
            # If not adequate, use previous
            self.slam.update(self.previous_distances, scan_angles_degrees=self.previous_angles)

        # Get current robot position
        x, y, theta = self.slam.getpos()

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
        # self.get_logger().info("Sending map {} to subscribers -- ".format(self.map_index) + str(type(self.mapbytes[0])))
        map_message.map_data = self.mapbytes
        map_message.index = self.map_index
        self.publisher_map.publish(map_message)
        self.map_index += 1



def main(args=None):
    rclpy.init(args=args)
    slam_node = Slam()
    rclpy.spin(slam_node)


if __name__ == '__main__':
    main()

