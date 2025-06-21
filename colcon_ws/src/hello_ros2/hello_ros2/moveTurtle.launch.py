import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',  # 실제 패키지 이름으로 변경하세요
            executable='move_turtle',
            name='move_turtle',
            output='screen',
            parameters=[os.path.join(os.getcwd(), 'config.yaml')],
        )
    ])
