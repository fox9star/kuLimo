# ros2 param list
# ros2 param dump
# ros2 param dump turtlesim
# ros2 param dump turtlesim >> turtlesim.yaml
# ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
# ros2 launch hello_ros2 moveTurtle.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        "param_dir",
        default=os.path.join(
            get_package_share_directory("hello_ros2"), "param", "turtlesim.yaml"
        ),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_dir",
                default_value=param_dir,
                description="turtlesim paramdter dump file",
            ),
            Node(
                package="turtlesim", executable="turtlesim_node", parameters=[param_dir]
            ),
            Node(
                package="hello_ros2", executable="move_turtle", parameters=[param_dir]
            ),
            Node(package="hello_ros2", executable="change_color_client"),
        ]
    )