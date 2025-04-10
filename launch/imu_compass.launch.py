import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        os.getenv("AMENT_PREFIX_PATH").split(":")[0],  # First overlayed install space
        "share",
        "imu_compass",
        "config",
        "imu_compass.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="imu_compass",
                executable="imu_compass_node",
                name="imu_compass_node",
                parameters=[config_file],
                output="screen",
            )
        ]
    )
