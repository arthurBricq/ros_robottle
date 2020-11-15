# ROS for Robottle

This repo contains the ROS source code to run the project, that was writen by myself. It is the ROS2 workspace of our project !

It works in pair with the [Robottle Python Package](https://github.com/arthurBricq/robottle_python_packages) where there are some helper functions and algorithms. 

Here is the documentation of the [first controller](doc/controller1.md)

As the code of this repository and the code of the **Robottle Python Package** are constituting the **high-level controller**, the code of the Arduino Mega **low-level controller** is accessible [here](https://github.com/ljacqueroud/Robottle-low-level). 


## ROS Nodes

Here is a description of the nodes we created for the robot. 

**Input Nodes**
- lidar_publisher.py: Reads data from lidar as send them as collected in the topic `lidar_data`
- uart_speed_reader.py: Reads the speed from the motors, using UART communications and send them in topic `uart_motors_speed`. 
- *computer_vision_node*: Reads the camera and returns the bounding boxes. This package was made by NVidea and here is its [documentation](https://github.com/dusty-nv/ros_deep_learning)

**Internal Nodes** (i.e. the brain)
- slam.py: SLAM node (evaluate position and map). It outputs on topics `world_map` and `robot_pos`. 
- controller_ol.py: most basic controller, will avoid obstacles merely based on the SLAM output. It then communicates commands to the node *uart_messenger* (which then transfer them to the Arduino Mega)
- vision_analyser.py: service manager that can (i) take a picture on demand and (ii) analyse it straigth away. Offered services are the followings:
    - 'find_map_corners': take a picture and return angle change from beacons present on the image.

**OutputNodes**
- uart_messenger.py: sends UART message containing desired motor actions to Arduino Mega. Reads from topic `uart_commands`

**Debugging Nodes**
*Those nodes are not intended to be ran on the Jetson Nano*
- slam_vizualiser.py: prints the map and the position in another window.
- teleop.py: controls the robot remotely using keyboard.


## Some useful commands 

Here are some commands quite useful with the ROS setup

- topic publication to move the robot somewhere

`ros2 topic pub --once /uart_commands std_msgs/msg/String "data: w"`

- bag files recording sensor inputs (lidar data + motors speed so far)

`ros2 bag record /lidar_data /motors_speed`


## Differences between Jetson and Personal Computer

There are differences between the different setups, because the Jetson has to do things that I will not try to replicate at home (and just there results)

- ros_deep_learning: this is the ROS package for CUDA realtime accelerated neuron network. It is present only on the Jetson and not on the shared code. The only thing one has to include are the interfaces : "vision_msgs" (https://github.com/Kukanani/vision_msgs)
