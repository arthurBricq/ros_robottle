# ROS for Robottle

This repo contains the ROS source code to run the project, that was writen by myself. It is the ROS2 workspace of our project !

## Other repositories used ! 

The ROS code works in pair with the [Robottle Python Package](https://github.com/arthurBricq/robottle_python_packages) where there are some helper functions and algorithms. 

Here is the documentation of the [first controller](doc/controller1.md), which is still under implementation.

As the code of this repository and the code of the **Robottle Python Package** are constituting the **high-level controller**, the code of the Arduino Mega **low-level controller** is accessible [here](https://github.com/ljacqueroud/Robottle-low-level). 

For the *Cuda-Accelerated* code for the **Neuron Network** to detect bottles was based on the [Jetson-Inference](https://github.com/dusty-nv/jetson-inference) code (*NVidea*) and especially with their detectnet code. Their ROS repository has a [documentation](https://github.com/dusty-nv/ros_deep_learning) that is quite complete ! *However, it is writen all in C*





## ROS Nodes

Here is a description of the nodes we created for the robot. 

### Input Nodes
- **lidar_publisher.py**: Reads data from lidar as send them as collected in the topic `lidar_data`
- **uart_speed_reader.py**: Reads the speed from the motors, using UART communications and send them in topic `uart_motors_speed`. 
- **detectnet.py**: Reads the camera and returns the bounding boxes. This package was made by NVidea and here is its [documentation](https://github.com/dusty-nv/ros_deep_learning). Here are different topics one other node can subscribe to
    - *image_in*: simply the raw image
    - *detections*: the bounding boxes
    - *vision_info*: vision metadata
    - *overlay*: the image with the bounding boxes drawn on top of it

### Internal Nodes (i.e. the brain)
- **slam.py**: SLAM node (evaluate position and map). It outputs on topics `world_map` and `robot_pos`. It is an implementation of the 'TinySLAM' algorithm, which is built locally from this repository: [BreezySLAM](https://github.com/simondlevy/BreezySLAM). *Some portion of this code needs to be modified*
- **controller_ol.py**: most basic controller, will avoid obstacles merely based on the SLAM output. It then communicates commands to the node *uart_messenger* (which then transfer them to the Arduino Mega)
- **vision_analyser.py**: service manager that can (i) take a picture on demand and (ii) analyse it straigth away. At the moment, it not possible to use this node if **detectnet-py** also is open. Offered services are the followings:
    - 'find_map_corners': take a picture and return angle change from beacons present on the image.

### OutputNodes 
- **uart_messenger.py**: sends UART message containing desired motor actions to Arduino Mega. Reads from topic `uart_commands`

### Debugging Nodes
*Those nodes are not intended to be ran on the Jetson Nano*
- **slam_vizualiser.py**: prints the map and the position in another window.
- **teleop.py**: controls the robot remotely using keyboard.


## Some useful commands 

Here are some commands quite useful with the ROS setup

- topic publication to move the robot somewhere

`ros2 topic pub --once /uart_commands std_msgs/msg/String "data: w"`

- bag files recording sensor inputs (lidar data + motors speed so far)

`ros2 bag record /lidar_data /motors_speed`

- launch the detectnet node (*with their launch files*)

`ros2 launch ros_deep_learning detectnet.ros2.launch input:=csi://0 output:=display://0`

*It's possible to have (or not) an output when this node is launched, but for it we must change the launch file. Later, I will integrate this in my own launch nodes*

- launch the other ROS nodes (LIDAR, Motors Speed Reader, SLAM, Motors Commands)

`ros2 launch robottle launch2.launch.py`

## Differences between Jetson and Personal Computer

There are differences between the different setups, because the Jetson has to do things that I will not try to replicate at home (and just there results)

- ros_deep_learning: this is the ROS package for CUDA realtime accelerated neuron network. It is present only on the Jetson and not on the shared code. The only thing one has to include are the interfaces : "vision_msgs" (https://github.com/Kukanani/vision_msgs)
