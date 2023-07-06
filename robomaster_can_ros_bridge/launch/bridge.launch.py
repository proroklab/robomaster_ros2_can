import socket
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    namespace = socket.gethostname().replace("-", "_")
    return LaunchDescription(
        [
            Node(
                package="robomaster_can_ros_bridge",
                executable="controlling",
                namespace=namespace,
                name="robomaster_controlling",
            ),
            Node(
                package="robomaster_can_ros_bridge",
                executable="sensing",
                namespace=namespace,
                name="robomaster_sensing",
            ),
        ]
    )
