# ROS for Robottle

This repo contains the ROS source code to run the project, that was writen by myself. It is the ROS2 workspace of our project !

## Some useful commands 

Here are some commands quite useful with the ROS setup

- topic publication to move the robot somewhere

ros2 topic pub --once /uart_commands std_msgs/msg/String "w"

- bag files recording sensor inputs (lidar data + motors speed so far)

ros2 bag record /lidar_data /motors_speed


## Differences between Jetson and Personal Computer

There are differences between the different setups, because the Jetson has to do things that I will not try to replicate at home (and just there results)

- ros_deep_learning: this is the ROS package for CUDA realtime accelerated neuron network. It is present only on the Jetson and not on the shared code. The only thing one has to include are the interfaces : "vision_msgs"
