import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package="robottle",
            node_executable="lidar",
            name="lidar",
            output="screen",
            emulate_tty=True
            ),
        Node( 
            package="robottle",
            node_executable="slam",
            name="slam"
            ),
        Node(
            package="robottle",
            node_executable="controller_ol",
            name="controller_ol",
            output="screen",
            emulate_tty=True
            ),
        Node(
            package="robottle",
            node_executable="uart",
            name="uart",
            output="screen",
            emulate_tty=True
            ),
        ])
