#!/bin/bash

# source ROS2

source /home/arthur/dev/ros/ros.sh
. /home/arthur/dev/ros/workspace1/install/setup.sh
# cd $ROS_PATH
# git pull
# colcon build 

# launch all the nodes
# ros2 run robottle teleop & 
ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0 & ros2 launch robottle launch_nocontroller.launch.py & ros2 run robottle controller1 --search; 
