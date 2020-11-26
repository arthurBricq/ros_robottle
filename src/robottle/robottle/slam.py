
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
MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10

MIN_SAMPLES = 100 
MAP_QUALITY = 50
OFFSET_MM = 175
DETECTION_MARGIN = 65
OBSTACLE_WIDTH_MM = 600


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
        laser = LaserModel(detection_margin = DETECTION_MARGIN, offset_mm = OFFSET_MM)
        self.slam = RMHC_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, map_quality = MAP_QUALITY, hole_width_mm = OBSTACLE_WIDTH_MM)
        self.trajectory = []
        self.mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
        self.previous_distances = None
        self.previous_angles    = None
        self.robot_pose_change = np.array([0.0,0.0,0.0])

        # DEBUG parameters
        self.map_index = 0
        self.previous_pos = (0,0,0)

    def listener_callback_detectnet(self, msg):
        return 

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
        delta_d =  - (dx_right - dx_left)
        delta_theta = np.arctan(delta_d / msg.LENGTH) * 57.2958 # [deg]
        self.robot_pose_change += [delta_r, delta_theta, msg.time_delta]

    def listener_callback_lidar(self, msg):
        # transform angles to $FORMAT1
        angles = np.array(msg.angles)
        angles = list((angles + 180) % 360) # because LIDAR is facing the robot
        distances = list(msg.distances)

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            self.slam.update(distances, scan_angles_degrees=angles, pose_change = tuple(self.robot_pose_change))
            # self.analyse_odometry()
            self.robot_pose_change = np.array([0.0,0.0,0.0])
            self.previous_distances = distances.copy()
            self.previous_angles    = angles.copy()
        elif self.previous_distances is not None:
            # If not adequate, use previous
            self.slam.update(self.previous_distances, scan_angles_degrees=self.previous_angles)

        # Get current robot position and current map bytes as grayscale
        x, y, theta = self.slam.getpos()
        self.slam.getmap(self.mapbytes)

        # Send topics 
        pos = Position()
        pos.x = float(x)
        pos.y = float(y)
        pos.theta = float(theta)
        self.publisher_position.publish(pos)
        
        map_message = Map()
        map_message.map_data = self.mapbytes
        map_message.index = self.map_index
        self.publisher_map.publish(map_message)
        self.map_index += 1


    # debug functions here (do NOT call them on real runs)

    def analyse_odometry(self):
        """Compare the odometry resuls with the change in position according to SLAM. 
        For this function to work, self.robot_pose_change must be set to something different than zero.
        And the variabe self.previous_pos must exist
        """
        x, y, theta = self.slam.getpos()
        x_old, y_old, theta_old = self.previous_pos
        dr = np.sqrt((x - x_old) ** 2 + (y - y_old) ** 2)
        dtheta = theta - theta_old
        print("-------")
        print("SLAM     change of pos: {:.3f}  {:.3f}".format(dr, dtheta))
        print("Odometry change of pos: {:.3f}  {:.3f} ".format(self.robot_pose_change[0], self.robot_pose_change[1]))
        self.previous_pos = (x,y,theta)


    def find_lidar_range(self, distances, angles):
        """Finds the critical angles of the LIDAR and prints them. 
        Careful, this function works (currently) only with the previous angles format.
        """
        (i1, i2) = lidar_utils.get_valid_lidar_range(distances = distances, 
                angles = angles,
                n_points = 10)
        print('indexes : {} : {} with angles {} - {}'
                .format(i1,i2,angles[i1], angles[i2]))




def main(args=None):
    rclpy.init(args=args)
    slam_node = Slam()
    rclpy.spin(slam_node)


if __name__ == '__main__':
    main()

