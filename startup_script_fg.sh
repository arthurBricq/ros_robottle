#!/bin/bash

# source ROS2
source /home/arthur/dev/ros/ros.sh
. /home/arthur/dev/ros/workspace1/install/setup.sh

# launch all the nodes
# ros2 run robottle teleop & 
ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0 & ros2 launch robottle bottle_picking.launch.py ; fg

# ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0 &
# ros2 launch robottle bottle_picking.launch.py && fg
