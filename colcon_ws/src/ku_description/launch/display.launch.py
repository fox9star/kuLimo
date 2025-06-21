# sudo apt install ros-humble-urdf-launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model_path = PathJoinSubstitution(["urdf", "myfirst.urdf"])
    model = DeclareLaunchArgument(
        name="model", default_value=default_model_path, description="myfirst urdf file"
    )
    return LaunchDescription(
        [
            model,
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("urdf_launch"), "launch", "display.launch.py"]
                ),
                launch_arguments={
                    "urdf_package": "ku_description",
                    "urdf_package_path": LaunchConfiguration("model"),
                }.items(),
            ),
        ]
    )