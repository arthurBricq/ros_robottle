# ROS for Robottle

This repo contains the ROS source code to run the project, that was writen by myself. It is the ROS2 workspace of our project !

## Some useful commands 

Here are some commands quite useful with the ROS setup

- topic publication to move the robot somewhere
ros2 topic pub --once /uart_commands std_msgs/msg/String "w"

- bag files recording sensor inputs (lidar data + motors speed so far)
ros2 bag record /lidar_data /motors_speed


## Missing code for runtime

Some packages are not inserted, because I didn't write them and it makes no sense to put a lot of code in this github

- ros_deep_learning: this is the ROS package for CUDA realtime accelerated neuron network. I have just set it up, I still have a lot to do with this package, so far it is not needed on this git repo.
