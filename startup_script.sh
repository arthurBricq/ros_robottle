#!/bin/bash

# source ROS2
cd $ROS_PATH
source /home/arthur/dev/ros/ros.sh
git pull
colcon build --packages-select robottle

# launch all the nodes
# ros2 run robottle teleop & 
ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0 & ros2 launch robottle launch_nocontroller.launch.py & ros2 run robottle controller1 --search; 
