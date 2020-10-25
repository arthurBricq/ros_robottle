# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from interfaces.msg import LidarData 

from breezyslam.sensors import RPLidarA1 as LaserModel

from breezyslam.algorithms import RMHC_SLAM

from rplidar import RPLidar as Lidar

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_DEVICE = '/dev/ttyUSB0'


class LidarPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # create publisher
        self.publisher_ = self.create_publisher(LidarData, 'lidar_data', 10)
        
        # create timer that will triger reading the data
        self.i = 0

        # setup the lidar object
        self.lidar = Lidar(LIDAR_DEVICE)
        self.iterator = self.lidar.iter_scans(max_buf_meas = 850)
        next(self.iterator)

        # launch infinite loop here 
        while True:
            self.timer_callback()


    def timer_callback(self):
        # get the data from the lidar
        items = [item for item in next(self.iterator)]
        angles    = [item[1] for item in items]
        distances = [item[2] for item in items]

        # create a message
        msg = LidarData()
        msg.angles = angles   
        msg.distances = distances  

        # publish the message
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    lidar_publisher = LidarPublisher()

    rclpy.spin(lidar_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
