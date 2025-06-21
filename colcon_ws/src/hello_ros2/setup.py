
import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hello_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            "share/" + package_name + "/param",
            glob(os.path.join("param", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aa",
    maintainer_email="freshmea@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hello_ros = hello_ros2.hello_ros:main",
            "move_turtle = hello_ros2.move_turtle:main",
            "simple_sub = hello_ros2.simple_sub:main",
            "simple_pub = hello_ros2.simple_pub:main",
            "simple_service_server = hello_ros2.simple_service_server:main",
            "simple_service_server2 = hello_ros2.simple_service_server2:main",
            "simple_service_client = hello_ros2.simple_service_client:main",
            "simple_parameter = hello_ros2.simple_parameter:main",
            "change_color_client = hello_ros2.change_color_client:main",
            "my_topic_pub = hello_ros2.my_topic_pub:main",
            "add_server = hello_ros2.add_server:main",
            "action_server = hello_ros2.action_server:main",
            "action_client = hello_ros2.action_client:main",
            "move_limo = hello_ros2.move_limo:main",
            "publish_map = hello_ros2.publish_map:main",
            "scan_map = hello_ros2.scan_map:main",
            "patrol = hello_ros2.patrol:main",
            "dynamic_tf = hello_ros2.dynamic_tf:main",
            "tf_listener = hello_ros2.tf_listener:main",
            "follow_turtlesim = hello_ros2.follow_turtlesim:main",
        ],
    },
)